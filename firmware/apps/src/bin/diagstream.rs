//! C4 无缝流交付：dump 自由运行 → GPIO 回环 → PARLIO 40 MHz → GDMA 环
//! → box16 → 三阶 CIC → IQ8 流帧（C5IQ type 2）→ USB-Serial/JTAG。
//!
//! 与 capburst（突发拼流）的区别：数据面零间隙（环零拷贝消费），
//! USB 发送穿插在消费循环里（每轮小批量非阻塞），杜绝 >1.4 ms 的
//! 停顿（环一圈仅 1.43 ms）。验收口径：60 秒 overrun 计数 + 帧计数 +
//! 消费速率对照 40 MB/s。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::usb::usb_serial_jtag::{
    UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx,
};
use esp_println as _;
use esp_sdr_mmio::{CaptureRate, DiagStream, DumpEngine};
use esp_sdr_radio::Radio;
use esp_sdr_shared::dsp::{
    cic_inverse, diag_lsb_from_histogram, normalize, Cic, CIC_R_MIN, DIAG_BOX,
};
use esp_sdr_shared::frame;

esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY_MHZ: u32 = 2412;
const GAIN_CODE: u32 = 40;
/// CIC 抽取率：2.5 MS/s / 32 = 78.1 kS/s IQ8（与 C3 交付速率一致，
/// USB-SJ 实测带宽内）。
const CIC_RATE: u32 = 32;
/// 每帧 IQ 样本对数（payload 2×512 = 1024 字节）。
const FRAME_PAIRS: usize = 512;
/// 每轮消费循环最多处理的环字节数（box16 对齐）。
const CONSUME_CHUNK: usize = 4096;
/// 每轮消费循环最多发送的帧内字节数（非阻塞小批量）。
const SEND_CHUNK: usize = 32;
/// IQ8 输出环形缓冲（发送背压时丢旧保实时，跳块计数）。
const IQ8_RING: usize = 8192;

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
fn select_lsb(dump_engine: &mut DumpEngine) -> u8 {
    let (words, _) = dump_engine
        .capture_burst(4096, CaptureRate::Msps20)
        .expect("level capture failed");
    let mut histogram = [0u32; 1024];
    let mut total: u32 = 0;
    for word in words.iter().copied() {
        let i = ((word << 22) as i32) >> 22; // 10 bit 符号扩展
        let q = ((word << 12) as i32) >> 22;
        histogram[i.unsigned_abs() as usize] += 1;
        histogram[q.unsigned_abs() as usize] += 1;
        total = total.saturating_add(2);
    }
    assert!(total > 0, "level capture produced no samples");
    let lsb = diag_lsb_from_histogram(&histogram, total, true);
    defmt::info!("lsb auto-selected: {}", lsb);
    lsb
}

/// 消费统计（单任务直管，结尾一次打印）。
static STATS_FRAMES: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn stream_task(
    mut usb_rx: UsbSerialJtagRx<'static, esp_hal::Blocking>,
    mut usb_tx: UsbSerialJtagTx<'static, esp_hal::Blocking>,
) {
    // 主机握手（与 capburst 同协议）：'G' 启动，20 s 超时兜底
    defmt::info!("waiting for host handshake ('G'), timeout 20 s");
    let mut waited_ms: u64 = 0;
    'handshake: loop {
        Timer::after(Duration::from_millis(50)).await;
        waited_ms += 50;
        loop {
            match usb_rx.read_byte() {
                Ok(b'G') => break 'handshake,
                Ok(_) => continue,
                Err(_) => break,
            }
        }
        if waited_ms >= 20_000 {
            defmt::warn!("handshake timeout, streaming anyway");
            break 'handshake;
        }
    }

    // 选窗快照（burst 引擎）→ 启动无缝流
    let mut dump_engine = DumpEngine::new();
    let lsb = select_lsb(&mut dump_engine);
    drop(dump_engine);
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

    // 消费与发送状态（单任务：无需原子）
    let mut consumed: u64 = 0;
    let mut skipped_total: u64 = 0;
    let mut dropped_total: u64 = 0;
    let mut overrun_was = false;
    // IQ8 环（发送背压时丢旧保实时）
    static mut IQ8_BUF: [u8; IQ8_RING] = [0; IQ8_RING];
    let mut iq8_write: u64 = 0;
    let mut iq8_read: u64 = 0;
    // CIC 批量输出（I/Q 交错；最坏 R=R_MIN 时的容量 + 裕量）
    const ACC_CAPACITY: usize =
        2 * (CONSUME_CHUNK / DIAG_BOX / CIC_R_MIN as usize + 4);
    let mut acc = [0i32; ACC_CAPACITY];
    // 帧组装/发送状态机
    let mut frame_buf = [0u8; frame::HEADER_LEN + FRAME_PAIRS * 2];
    let mut frame_send_pos: usize = 0; // 0 = 无帧在发
    let mut sequence: u16 = 0;

    let started = esp_hal::time::Instant::now();
    let run_for = esp_hal::time::Duration::from_secs(60);

    while started.elapsed() < run_for {
        let producer = stream.poll_producer();

        // 过跑恢复：跳到最新 16 字节对齐位置（丢块计数，流时间线前跳）
        if stream.overrun() && !overrun_was {
            overrun_was = true;
            let newest = producer & !(DIAG_BOX as u64 - 1);
            skipped_total += newest - consumed;
            consumed = newest;
        } else {
            overrun_was = stream.overrun();
        }

        // 消费：环内连续段批量过 CIC（out 布局 I/Q 交错）
        let available = stream.contiguous_available(consumed);
        if available >= DIAG_BOX {
            let take = available.min(CONSUME_CHUNK) & !(DIAG_BOX - 1);
            let bytes = stream.ring_slice(consumed, take);
            let pairs = cic.run(bytes, &mut acc);
            consumed += take as u64;
            // 归一 → IQ8 入环
            for pair in 0..pairs {
                let i = normalize(acc[2 * pair], inverse) >> 6; // 10bit×16 → 8bit
                let q = normalize(acc[2 * pair + 1], inverse) >> 6;
                let i8v = i.clamp(-128, 127) as i8 as u8;
                let q8v = q.clamp(-128, 127) as i8 as u8;
                let offset = (iq8_write % IQ8_RING as u64) as usize;
                unsafe {
                    let buffer = &mut *core::ptr::addr_of_mut!(IQ8_BUF);
                    buffer[offset] = i8v;
                    buffer[(offset + 1) % IQ8_RING] = q8v;
                }
                iq8_write += 2;
            }
        }

        // 发送：无帧在发且数据够一帧 → 组帧；有帧 → 非阻塞小批量推进
        if frame_send_pos == 0 && iq8_write - iq8_read >= FRAME_PAIRS as u64 * 2 {
            // 读落后过多（发送端被主机拖死过）：跳到最新完整帧
            if iq8_write - iq8_read > IQ8_RING as u64 - FRAME_PAIRS as u64 * 2 {
                let newest = iq8_write - FRAME_PAIRS as u64 * 2;
                dropped_total += newest - iq8_read;
                iq8_read = newest;
            }
            // 组帧（头 + payload）
            sequence = sequence.wrapping_add(1);
            let header = frame::CaptureFrameHeader {
                frame_type: frame::STREAM_FRAME_TYPE,
                rate_index: 0,
                sequence,
                sample_word_count: FRAME_PAIRS as u16,
                frequency_mhz: FREQUENCY_MHZ,
                elapsed_us: started.elapsed().as_micros() as u32,
                crc32: 0,
            };
            frame_buf[..frame::HEADER_LEN].copy_from_slice(&header.encode());
            let payload = &mut frame_buf[frame::HEADER_LEN..];
            unsafe {
                let buffer = &*core::ptr::addr_of!(IQ8_BUF);
                let start = (iq8_read % IQ8_RING as u64) as usize;
                let end = (start + FRAME_PAIRS * 2) % IQ8_RING;
                if start < end {
                    payload.copy_from_slice(&buffer[start..end]);
                } else {
                    let split = IQ8_RING - start;
                    payload[..split].copy_from_slice(&buffer[start..]);
                    payload[split..].copy_from_slice(&buffer[..end]);
                }
            }
            let crc = frame::crc32(&frame_buf[..frame::CRC_COVERED_HEADER_LEN]);
            let tail = frame::HEADER_LEN + FRAME_PAIRS * 2 - 4;
            frame_buf[tail..].copy_from_slice(&crc.to_le_bytes());
            frame_send_pos = 1;
            iq8_read += FRAME_PAIRS as u64 * 2;
            STATS_FRAMES.fetch_add(1, Ordering::Relaxed);
        }
        if frame_send_pos > 0 {
            let end = frame::HEADER_LEN + FRAME_PAIRS * 2;
            let mut sent = 0;
            while frame_send_pos < end && sent < SEND_CHUNK {
                match usb_tx.write_byte_nb(frame_buf[frame_send_pos]) {
                    Ok(()) => {
                        frame_send_pos += 1;
                        sent += 1;
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => {
                        frame_send_pos = 0; // 端点坏：丢帧重试
                        break;
                    }
                }
            }
            if frame_send_pos >= end {
                let _ = usb_tx.flush_tx_nb();
                frame_send_pos = 0;
            }
        }
    }

    let produced = stream.producer_bytes();
    stream.stop();
    let seconds = 60u64;
    let frames = STATS_FRAMES.load(Ordering::Relaxed);
    defmt::info!(
        "FINAL(diagstream): produced={} consumed={} skipped={} dropped={} frames={} cons_rate={} B/s",
        produced,
        consumed,
        skipped_total,
        dropped_total,
        frames,
        consumed / seconds
    );

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

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
    controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None).unwrap();
    let sniffer = controller.sniffer();
    sniffer.set_promiscuous_mode(true).unwrap();

    let mut radio = Radio::new();
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();
    drop(radio);
    drop(sniffer);
    drop(controller);

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let (usb_rx, usb_tx) = usb.split();

    defmt::info!("diagstream ready");
    spawner.spawn(stream_task(usb_rx, usb_tx).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
