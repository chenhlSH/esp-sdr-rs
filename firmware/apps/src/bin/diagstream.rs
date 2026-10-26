//! C4 无缝流交付：dump 自由运行 → GPIO 回环 → PARLIO 40 MHz → GDMA 环
//! → box16 → 三阶 CIC → IQ8 流帧（C5IQ type 2）→ USB-Serial/JTAG。
//!
//! 数据面零拷贝：CIC 输出进 IQ8 环后，组帧不做整帧拷贝——帧头 24 字节
//! 由小栈数组发送，payload 直接从 IQ8 环分两段写 USB（跨环边）。CRC
//! 同样直接在环上分两段计算。
//!
//! 发送方式 = C3 验证过的同步非阻塞逐字节（write_byte_nb，有界预算）。
//! async 发送（esp-hal write_async）在本固件实测会让 Windows USB-SJ
//! 主机驱动在高吞吐下读卡死（.NET 与 serialport-rs 同病）；逐字节在
//! 160 KB/s 下仅 ~0.7% CPU（240 MHz），完全可控。
//!
//! 单任务、纯轮询：每轮 poll → 消费 → 组帧/发送推进，无 await。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_println as _;
use esp_sdr_mmio::{CaptureRate, DiagStream, DumpEngine, RING_BYTES};
use esp_sdr_radio::Radio;
use esp_sdr_shared::dsp::{
    cic_inverse, diag_lsb_from_histogram, normalize, Cic, CIC_R_MIN, DIAG_BOX,
};
use esp_sdr_shared::frame;

esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY_MHZ: u32 = 2412;
const GAIN_CODE: u32 = 40;
/// CIC 抽取率：2.5 MS/s（40 MHz 视界 ÷ box16） / 32 = 78.1 kS/s IQ8
/// （与 C3 交付速率一致，USB-SJ 实测带宽内）。
const CIC_RATE: u32 = 32;
/// 每帧 IQ 样本对数（payload 2×4096 = 8 KB，19 帧/秒 @78.1 kS/s）。
/// 小帧（512 对 = 153 帧/秒）实测让 Windows USB-SJ 主机驱动在高吞吐下
/// 读卡死；16 KB 帧（9.5 帧/秒）的 IQ8 环顶爆块 1 前的 DRAM 预算，取 8 KB。
/// payload 零拷贝直发，不占额外静态缓冲。
const FRAME_PAIRS: usize = 4096;
const FRAME_PAYLOAD_BYTES: usize = FRAME_PAIRS * 2;
/// 每轮消费循环最多处理的环字节数（box16 对齐）。
const CONSUME_CHUNK: usize = 4096;
/// 每轮最多发送的字节数（非阻塞；FIFO 满即 WouldBlock 停本轮）。
const SEND_CHUNK: usize = 128;
/// IQ8 输出环形缓冲：2 的幂，恰容 2 帧（发送/组装错峰）。
const IQ8_RING: usize = FRAME_PAYLOAD_BYTES * 2;
const IQ8_MASK: usize = IQ8_RING - 1;

/// WiFi 混杂过滤器清零（espsoup pe_quiet_wifi：WiFi 驱动任务把帧递交
/// 上层的抢占长到足以让环过跑——过滤器全零后 MAC 不再向驱动递交）。
#[repr(C)]
struct WifiPromiscuousFilter {
    filter_mask: u32,
}
unsafe extern "C" {
    fn esp_wifi_set_promiscuous_filter(filter: *const WifiPromiscuousFilter) -> i32;
}

fn quiet_wifi_filter() {
    let filter = WifiPromiscuousFilter { filter_mask: 0 };
    let result = unsafe { esp_wifi_set_promiscuous_filter(&filter) };
    defmt::info!("promiscuous filter cleared: {}", result);
}

/// 突发快照选窗：20 MS/s 采 4096 字（10 bit I/Q）→ 幅度直方图 →
/// diag 窗口 lsb（robust：中值 ×2.2 抗 2.4 GHz 突发）。
/// 重试 3 次：adctrig 临界窗可被 async USB/WiFi 中断插入导致
/// CaptureIncomplete（双任务实测偶发；单任务版从未失败）。
/// dc_adc8 为已测基线 DC（ADC8 counts），统计前先减去——否则 DC 偏置
/// 会把 99.9% 点顶高，窗口白白加大一档。
fn select_lsb(dump_engine: &mut DumpEngine, dc_adc8: [i32; 2]) -> u8 {
    let mut words: &[u32] = &[];
    let mut last_error = None;
    for _attempt in 0..3 {
        match dump_engine.capture_burst(4096, CaptureRate::Msps20) {
            Ok((sample_words, _)) => {
                words = sample_words;
                break;
            }
            Err(error) => {
                defmt::warn!("level capture failed: {:?}, retrying", error);
                last_error = Some(error);
            }
        }
    }
    assert!(!words.is_empty(), "level capture failed 3x: {:?}", last_error);
    let mut histogram = [0u32; 1024];
    let mut total: u32 = 0;
    for word in words.iter().copied() {
        let i = ((word << 22) as i32) >> 22; // 10 bit 符号扩展
        let q = ((word << 12) as i32) >> 22;
        // ADC8 DC → 10bit 量纲（×4）
        let i_centered = (i - dc_adc8[0] * 4).clamp(-512, 511);
        let q_centered = (q - dc_adc8[1] * 4).clamp(-512, 511);
        histogram[i_centered.unsigned_abs() as usize] += 1;
        histogram[q_centered.unsigned_abs() as usize] += 1;
        total = total.saturating_add(2);
    }
    assert!(total > 0, "level capture produced no samples");
    let lsb = diag_lsb_from_histogram(&histogram, total, true);
    defmt::info!("lsb auto-selected: {}", lsb);
    lsb
}

/// 发送统计（主机接入后由发送状态机累加）。
static STATS_FRAMES_SENT: AtomicU32 = AtomicU32::new(0);

/// 流任务：LSB 已选、DC 基线已测；持续运行（每 60 秒窗口统计）。
#[embassy_executor::task]
async fn stream_task(
    lsb: u8,
    dc_offset_counts: [i32; 2],
    mut usb_rx: UsbSerialJtagRx<'static, esp_hal::Blocking>,
    mut usb_tx: UsbSerialJtagTx<'static, esp_hal::Blocking>,
) {
    quiet_wifi_filter();
    let mut stream = DiagStream::start(lsb);
    let inverse = cic_inverse(CIC_RATE, lsb);
    let mut cic = Cic::new(CIC_RATE);
    defmt::info!(
        "streaming: lsb={} R={} inverse=0x{:08x}",
        lsb,
        CIC_RATE,
        inverse
    );

    // 消费与发送状态
    let mut consumed: u64 = 0;
    let mut consume_offset: usize = 0; // 环内偏移（增量维护）
    let mut skipped_total: u64 = 0;
    let mut dropped_total: u64 = 0;
    let mut overrun_was = false;
    // IQ8 环
    static mut IQ8_BUF: [u8; IQ8_RING] = [0; IQ8_RING];
    let mut iq8_write: u64 = 0;
    let mut iq8_read: u64 = 0;
    // CIC 批量输出（I/Q 交错；最坏 R=R_MIN 时的容量 + 裕量）
    const ACC_CAPACITY: usize = 2 * (CONSUME_CHUNK / DIAG_BOX / CIC_R_MIN as usize + 4);
    let mut acc = [0i32; ACC_CAPACITY];
    // 帧发送状态机：send_pos == 0 空闲；1..=24 头字节；>24 为 payload 偏移
    let mut send_pos: usize = 0;
    let mut send_frame_start: u64 = 0;
    let mut header = [0u8; frame::HEADER_LEN];
    let mut sequence: u16 = 0;
    // 主机握手（'G'）后才发送：无主机时帧通道满丢帧（保实时）
    let mut host_ready = false;

    let started = esp_hal::time::Instant::now();
    let mut window_elapsed = esp_hal::time::Duration::from_secs(0);
    let mut window_produced = 0u64;
    let mut window_consumed = 0u64;

    let mut tick: u32 = 0;
    loop {
        let producer = stream.poll_producer();
        tick = tick.wrapping_add(1);

        // 过跑恢复：消费游标跳到最新 16 字节对齐位置（丢块计数）
        if stream.overrun() && !overrun_was {
            overrun_was = true;
            skipped_total += producer - consumed;
            consumed = producer & !(DIAG_BOX as u64 - 1);
            consume_offset = stream.producer_ring_offset() & !(DIAG_BOX - 1);
        } else {
            overrun_was = stream.overrun();
        }

        // 消费：环内连续段批量过 CIC（汇编路径，3.38 c/B）
        let available = stream.available_at(consume_offset);
        if available >= DIAG_BOX {
            let take = available.min(CONSUME_CHUNK) & !(DIAG_BOX - 1);
            let bytes = stream.ring_slice_at(consume_offset, take);
            let pairs = cic.run_asm(bytes, &mut acc);
            consume_offset = (consume_offset + take) % RING_BYTES;
            consumed += take as u64;
            // 归一 → IQ8 入环。软件 DC 补偿：减启动基线（>>6 后量纲 =
            // 10bit counts = ADC8×4，故基线换算 ×4）
            for pair in 0..pairs {
                let i = (normalize(acc[2 * pair], inverse) >> 6) - dc_offset_counts[0];
                let q = (normalize(acc[2 * pair + 1], inverse) >> 6) - dc_offset_counts[1];
                let i8v = i.clamp(-128, 127) as i8 as u8;
                let q8v = q.clamp(-128, 127) as i8 as u8;
                let offset = (iq8_write & IQ8_MASK as u64) as usize;
                unsafe {
                    let buffer = &mut *core::ptr::addr_of_mut!(IQ8_BUF);
                    buffer[offset] = i8v;
                    buffer[(offset + 1) & IQ8_MASK] = q8v;
                }
                iq8_write += 2;
            }
        }

        // 主机握手（每轮尝试一次非阻塞读）
        if !host_ready {
            match usb_rx.read_byte() {
                Ok(b'G') => {
                    host_ready = true;
                    defmt::info!("host handshake ok, streaming frames");
                }
                Ok(_) => {}
                Err(_) => {}
            }
        }

        // 帧状态机：空闲→组帧（头+CRC，payload 零拷贝）→ 分批发送
        if host_ready && send_pos == 0 && iq8_write - iq8_read >= FRAME_PAYLOAD_BYTES as u64 {
            // 读落后过多（发送端被主机拖死过）：跳到最新完整帧
            if iq8_write - iq8_read > IQ8_RING as u64 - FRAME_PAYLOAD_BYTES as u64 {
                let newest = iq8_write - FRAME_PAYLOAD_BYTES as u64;
                dropped_total += newest - iq8_read;
                iq8_read = newest;
            }
            send_frame_start = iq8_read;
            sequence = sequence.wrapping_add(1);
            let header_frame = frame::CaptureFrameHeader {
                frame_type: frame::STREAM_FRAME_TYPE,
                rate_index: 0,
                sequence,
                sample_word_count: FRAME_PAIRS as u16,
                frequency_mhz: FREQUENCY_MHZ,
                elapsed_us: 0, // 定长帧协议：时间戳由帧序+帧率推算
                crc32: 0,
            };
            header[..frame::HEADER_LEN].copy_from_slice(&header_frame.encode());
            // CRC 覆盖帧头前 20 字节 + payload：payload 在环上分两段直算
            let mut crc_context = frame::Crc32::new();
            crc_context.update(&header[..frame::CRC_COVERED_HEADER_LEN]);
            unsafe {
                let buffer = &*core::ptr::addr_of!(IQ8_BUF);
                let start = (send_frame_start & IQ8_MASK as u64) as usize;
                let end = (start + FRAME_PAYLOAD_BYTES) & IQ8_MASK;
                if start < end {
                    crc_context.update(&buffer[start..end]);
                } else {
                    crc_context.update(&buffer[start..]);
                    crc_context.update(&buffer[..end]);
                }
            }
            header[frame::HEADER_LEN - 4..].copy_from_slice(&crc_context.finalize().to_le_bytes());
            send_pos = 1;
        }
        if send_pos > 0 {
            // 头 + payload 共 FRAME_PAYLOAD_BYTES + 24 字节；有界非阻塞
            let total_bytes = frame::HEADER_LEN + FRAME_PAYLOAD_BYTES;
            let mut sent = 0;
            while send_pos <= total_bytes && sent < SEND_CHUNK {
                let byte = if send_pos <= frame::HEADER_LEN {
                    header[send_pos - 1]
                } else {
                    let payload_offset = send_pos - 1 - frame::HEADER_LEN;
                    let index = ((send_frame_start + payload_offset as u64) & IQ8_MASK as u64)
                        as usize;
                    unsafe { (*core::ptr::addr_of!(IQ8_BUF))[index] }
                };
                match usb_tx.write_byte_nb(byte) {
                    Ok(()) => {
                        send_pos += 1;
                        sent += 1;
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => {
                        // 端点错误：丢弃本帧继续（保实时）
                        send_pos = 0;
                        break;
                    }
                }
            }
            if send_pos > total_bytes {
                let _ = usb_tx.flush_tx_nb();
                iq8_read = send_frame_start + FRAME_PAYLOAD_BYTES as u64;
                STATS_FRAMES_SENT.fetch_add(1, Ordering::Relaxed);
                send_pos = 0;
            }
        }

        // 每 60 秒打印窗口统计（流不停）
        if tick & 0xFF == 0 {
            let now = started.elapsed();
            if now - window_elapsed >= esp_hal::time::Duration::from_secs(60) {
                defmt::info!(
                    "STREAM 60s window: produced={} consumed={} skipped={} dropped={} sent_frames={} rate={} B/s orphaned={}",
                    producer - window_produced,
                    consumed - window_consumed,
                    skipped_total,
                    dropped_total,
                    STATS_FRAMES_SENT.load(Ordering::Relaxed),
                    (consumed - window_consumed) / 60,
                    stream.orphaned_reclaimed()
                );
                window_elapsed = now;
                window_produced = producer;
                window_consumed = consumed;
            }
        }
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    // CPU 240 MHz 显式设置：esp-hal C5 的 CpuClock 默认 80 MHz（从 C0 起
    // 全部固件漏设，导致消费吞吐全部按错误频率核算——§10.4 的"43.5%
    // 钉死"真因）。
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 完整射频 bring-up（C2 验证路径）：STA 启动 + 混杂模式（接收链在线）
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_config(&esp_radio::wifi::Config::Station(Default::default()))
        .unwrap();
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    controller
        .set_channel(1, esp_radio::wifi::SecondaryChannel::None)
        .unwrap();
    let sniffer = controller.sniffer();
    sniffer.set_promiscuous_mode(true).unwrap();

    let mut radio = Radio::new();
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();
    drop(sniffer);
    drop(controller);

    // 选窗 + DC 校准在优先级提升/spawn 之前同步完成：高优先级线程 +
    // async USB 任务会干扰 vendor adctrig 的让渡时序（实测快照被垃圾字
    // 污染：Q 均值 -10175 ADC8，远超 ±512 的 10bit 范围）。
    // 顺序照 espsoup pe_start：**DC 校准在前、选窗在后**（其 dc_calibrate
    // 先 prepare_rx；adctrig 会触碰 PHY 状态，先校准才能读到正确工作点）。
    let mut dump_engine = DumpEngine::new();
    {
        // PBUS 自检（诊断 force_test/rd 语义）
        let probe = esp_sdr_radio::pbus_self_test();
        defmt::info!(
            "PBUS self-test: rd(2,2) work=0x{:x} debug=0x{:x} force400=0x{:x} force(3,2,100)=0x{:x}",
            probe[0],
            probe[1],
            probe[2],
            probe[3]
        );
    }
    // 软件 DC 补偿基线（硬件 DAC 校准在本固件实测无响应）：
    // dc_adc8 = 实测基线；dc_offset_counts = 10bit counts（ADC8×4）
    let mut dc_adc8 = [0i32; 2];
    let mut dc_offset_counts = [0i32; 2];
    {
        // 20 MS/s 快照测 DC（espsoup dc_measure 同款原理）。环境差异：
        // adctrig 快照偶发坏帧（抢占让渡时序），算术均值会被极端字拉爆
        // ——改用直方图中值（与选窗同款抗扰）+ 一致性重试。
        let mut measure = |m: &mut [f32; 2]| -> bool {
            let mut previous: Option<[f32; 2]> = None;
            for _attempt in 0..5 {
                let Ok((words, _)) = dump_engine.capture_burst(16380, CaptureRate::Msps20) else {
                    continue;
                };
                let mut histogram_i = [0u32; 1024];
                let mut histogram_q = [0u32; 1024];
                for word in words.iter().copied() {
                    let i = ((word << 22) as i32) >> 22;
                    let q = ((word << 12) as i32) >> 22;
                    histogram_i[(i + 512) as usize] += 1;
                    histogram_q[(q + 512) as usize] += 1;
                }
                let median = |histogram: &[u32; 1024]| -> i32 {
                    let half = 16380 / 2;
                    let mut accumulated = 0u32;
                    for (bin, &count) in histogram.iter().enumerate() {
                        accumulated += count;
                        if accumulated > half {
                            return bin as i32 - 512;
                        }
                    }
                    0
                };
                let median_i = median(&histogram_i) as f32 / 4.0;
                let median_q = median(&histogram_q) as f32 / 4.0;
                if median_i.abs() > 300.0 || median_q.abs() > 300.0 {
                    defmt::warn!(
                        "DC measure rejected: i={} q={} centi",
                        (median_i * 100.0) as i32,
                        (median_q * 100.0) as i32
                    );
                    continue;
                }
                if let Some(prev) = previous {
                    if (median_i - prev[0]).abs() < 5.0 && (median_q - prev[1]).abs() < 5.0 {
                        m[0] = (median_i + prev[0]) / 2.0;
                        m[1] = (median_q + prev[1]) / 2.0;
                        return true;
                    }
                }
                previous = Some([median_i, median_q]);
            }
            match previous {
                Some(prev) => {
                    // 一致性未凑齐：单帧中值本身已抗离群，采信最后一帧
                    m[0] = prev[0];
                    m[1] = prev[1];
                    true
                }
                None => false,
            }
        };
        match radio.calibrate_dc(&mut measure) {
            Ok(report) => {
                dc_adc8 = [report.before_centi[0] / 100, report.before_centi[1] / 100];
                dc_offset_counts = [dc_adc8[0] * 4, dc_adc8[1] * 4];
                defmt::info!(
                    "DC cal: before [{} {}] after [{} {}] centi base [{},{}] best [{},{}] steps {} skipped {} responsive {} response [{},{},{},{}] centi; software offset {} {} counts",
                    report.before_centi[0],
                    report.before_centi[1],
                    report.after_centi[0],
                    report.after_centi[1],
                    report.base[0],
                    report.base[1],
                    report.best[0],
                    report.best[1],
                    report.steps,
                    report.skipped,
                    report.responsive,
                    report.response_centi[0],
                    report.response_centi[1],
                    report.response_centi[2],
                    report.response_centi[3],
                    dc_offset_counts[0],
                    dc_offset_counts[1]
                );
            }
            Err(error) => defmt::warn!("DC cal failed: {:?} (no software offset)", error),
        }
    }
    let lsb = select_lsb(&mut dump_engine, dc_adc8);
    drop(dump_engine);

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let (usb_rx, usb_tx) = usb.split();

    defmt::info!("diagstream ready");
    // 消费任务优先级高于 WiFi 驱动任务（30）：保证 56% 的消费预算。
    esp_rtos::CurrentThreadHandle::get().set_priority(31);
    spawner.spawn(stream_task(lsb, dc_offset_counts, usb_rx, usb_tx).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}