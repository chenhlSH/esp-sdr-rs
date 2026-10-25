//! C4 无缝流交付：dump 自由运行 → GPIO 回环 → PARLIO 20 MHz → GDMA 环
//! → box16 → 三阶 CIC → IQ8 流帧（C5IQ type 2）→ USB-Serial/JTAG。
//!
//! 双任务架构（消费瓶颈破案后定型，见 docs/c5-registers.md §10.3）：
//! - stream_task：纯轮询消费（poll → CIC → IQ8 环 → 组帧），无 await。
//!   逐字节 write_byte_nb 的跨时钟域 MMIO 是实测主瓶颈（每轮 ~260 µs），
//!   USB 绝不进消费循环。
//! - usb_task：async 中断驱动（write_async：64B 纯 FIFO 写 + 中断 Future
//!   等待），embassy Channel 交接帧（容量 4，满则丢帧保实时）。
//! 验收口径：60 秒 overrun/跳块计数 + 帧计数 + 消费速率对照生产速率。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
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
/// CIC 抽取率：1.25 MS/s（20 MHz 视界 ÷ box16） / 16 = 78.1 kS/s IQ8。
const CIC_RATE: u32 = 16;
/// 每帧 IQ 样本对数（payload 2×512 = 1024 字节）。
const FRAME_PAIRS: usize = 512;
const FRAME_BYTES: usize = frame::HEADER_LEN + FRAME_PAIRS * 2;
/// 每轮消费循环最多处理的环字节数（box16 对齐）。
const CONSUME_CHUNK: usize = 4096;
/// IQ8 输出环形缓冲（发送背压时丢旧保实时）。2 的幂：位与寻址。
const IQ8_RING: usize = 8192;
const IQ8_MASK: usize = IQ8_RING - 1;
/// 帧交接通道深度（满则丢帧：USB 落后超过 4 帧≈59 ms 即丢）。
const FRAME_QUEUE_DEPTH: usize = 4;

type StreamFrame = [u8; FRAME_BYTES];

static FRAME_CHANNEL: Channel<CriticalSectionRawMutex, StreamFrame, FRAME_QUEUE_DEPTH> =
    Channel::new();

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
fn select_lsb(dump_engine: &mut DumpEngine) -> u8 {
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
        histogram[i.unsigned_abs() as usize] += 1;
        histogram[q.unsigned_abs() as usize] += 1;
        total = total.saturating_add(2);
    }
    assert!(total > 0, "level capture produced no samples");
    let lsb = diag_lsb_from_histogram(&histogram, total, true);
    defmt::info!("lsb auto-selected: {}", lsb);
    lsb
}

/// 发送统计（usb_task 写，结尾由 stream_task 打印前读取）。
static STATS_FRAMES_SENT: AtomicU32 = AtomicU32::new(0);

/// USB 发送任务：'G' 握手 → 持续取帧写 USB（中断驱动，等待不占 CPU）。
#[embassy_executor::task]
async fn usb_task(
    mut usb_rx: esp_hal::usb::usb_serial_jtag::UsbSerialJtagRx<'static, esp_hal::Async>,
    mut usb_tx: esp_hal::usb::usb_serial_jtag::UsbSerialJtagTx<'static, esp_hal::Async>,
) {
    use embedded_io_async::{Read as _, Write as _};
    // 主机握手（与 capburst 同协议）：'G' 启动。无主机时本任务静候，
    // 消费任务照常运行（通道满丢帧，FINAL 统计仍产出）。
    defmt::info!("waiting for host handshake ('G')");
    let mut handshake_byte = [0u8; 1];
    loop {
        match usb_rx.read(&mut handshake_byte).await {
            Ok(1) if handshake_byte[0] == b'G' => break,
            Ok(_) => continue,
            Err(_) => continue,
        }
    }
    defmt::info!("handshake ok, streaming");

    loop {
        let frame_buffer = FRAME_CHANNEL.receive().await;
        match usb_tx.write_all(&frame_buffer).await {
            Ok(()) => {
                STATS_FRAMES_SENT.fetch_add(1, Ordering::Relaxed);
            }
            Err(_) => {
                // 端点错误（主机未接入/拔出）：丢弃继续（保实时）
            }
        }
    }
}

/// 消费任务：选窗 → 无缝流 → CIC → IQ8 环 → 组帧入通道。纯轮询无 await。
#[embassy_executor::task]
async fn stream_task() {
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
    let mut consume_offset: usize = 0; // 环内偏移（增量维护）
    let mut skipped_total: u64 = 0;
    let mut dropped_total: u64 = 0;
    let mut queued_total: u64 = 0;
    let mut overrun_was = false;
    // IQ8 环
    static mut IQ8_BUF: [u8; IQ8_RING] = [0; IQ8_RING];
    let mut iq8_write: u64 = 0;
    let mut iq8_read: u64 = 0;
    // CIC 批量输出（I/Q 交错；最坏 R=R_MIN 时的容量 + 裕量）
    const ACC_CAPACITY: usize =
        2 * (CONSUME_CHUNK / DIAG_BOX / CIC_R_MIN as usize + 4);
    let mut acc = [0i32; ACC_CAPACITY];
    // 组帧状态
    let mut frame_buf: StreamFrame = [0; FRAME_BYTES];
    let mut sequence: u16 = 0;

    let started = esp_hal::time::Instant::now();
    let run_for = esp_hal::time::Duration::from_secs(60);

    // elapsed() 走系统时间源，每轮调用是实测热点之一：节流到每 256 轮
    let mut tick: u32 = 0;
    let mut time_up = false;
    while !time_up {
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

        // 消费：环内连续段批量过 CIC（out 布局 I/Q 交错）
        let available = stream.available_at(consume_offset);
        if available >= DIAG_BOX {
            let take = available.min(CONSUME_CHUNK) & !(DIAG_BOX - 1);
            let bytes = stream.ring_slice_at(consume_offset, take);
            let pairs = cic.run(bytes, &mut acc);
            consume_offset = (consume_offset + take) % RING_BYTES;
            consumed += take as u64;
            // 归一 → IQ8 入环（2 的幂：位与寻址）
            for pair in 0..pairs {
                let i = normalize(acc[2 * pair], inverse) >> 6;
                let q = normalize(acc[2 * pair + 1], inverse) >> 6;
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

        // 组帧：够一帧即从 IQ8 环拷出，交接给 usb_task（通道满丢帧）
        while iq8_write - iq8_read >= FRAME_PAIRS as u64 * 2 {
            if iq8_write - iq8_read > IQ8_RING as u64 - FRAME_PAIRS as u64 * 2 {
                let newest = iq8_write - FRAME_PAIRS as u64 * 2;
                dropped_total += newest - iq8_read;
                iq8_read = newest;
            }
            sequence = sequence.wrapping_add(1);
            let header = frame::CaptureFrameHeader {
                frame_type: frame::STREAM_FRAME_TYPE,
                rate_index: 0,
                sequence,
                sample_word_count: FRAME_PAIRS as u16,
                frequency_mhz: FREQUENCY_MHZ,
                elapsed_us: 0, // 定长帧协议：时间戳由帧序+帧率推算
                crc32: 0,
            };
            frame_buf[..frame::HEADER_LEN].copy_from_slice(&header.encode());
            let payload = &mut frame_buf[frame::HEADER_LEN..];
            unsafe {
                let buffer = &*core::ptr::addr_of!(IQ8_BUF);
                let start = (iq8_read & IQ8_MASK as u64) as usize;
                let end = (start + FRAME_PAIRS * 2) & IQ8_MASK;
                if start < end {
                    payload.copy_from_slice(&buffer[start..end]);
                } else {
                    let split = IQ8_RING - start;
                    payload[..split].copy_from_slice(&buffer[start..]);
                    payload[split..].copy_from_slice(&buffer[..end]);
                }
            }
            // CRC 覆盖帧头前 20 字节 + 载荷全部（capburst 同款增量两段）
            let mut crc_context = frame::Crc32::new();
            crc_context.update(&frame_buf[..frame::CRC_COVERED_HEADER_LEN]);
            crc_context.update(&frame_buf[frame::HEADER_LEN..]);
            frame_buf[FRAME_BYTES - 4..].copy_from_slice(&crc_context.finalize().to_le_bytes());
            match FRAME_CHANNEL.try_send(frame_buf) {
                Ok(()) => queued_total += 1,
                Err(_) => dropped_total += 1, // USB 落后：丢整帧保实时
            }
            iq8_read += FRAME_PAIRS as u64 * 2;
        }

        // 时间源读取节流：256 轮一次
        if tick & 0xFF == 0 && started.elapsed() >= run_for {
            time_up = true;
        }
    }

    let produced = stream.producer_bytes();
    stream.stop();
    // 给 USB 任务一点排空时间后打印总结
    Timer::after(Duration::from_millis(500)).await;
    let sent = STATS_FRAMES_SENT.load(Ordering::Relaxed);
    defmt::info!(
        "FINAL(diagstream): produced={} consumed={} skipped={} dropped={} queued={} sent={} cons_rate={} B/s",
        produced,
        consumed,
        skipped_total,
        dropped_total,
        queued_total,
        sent,
        consumed / 60
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

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();

    defmt::info!("diagstream ready");
    // 消费任务优先级高于 WiFi 驱动任务（30）：TIMING 实测 executor 线程
    // 在优先级 29 下仅获 ~28% CPU（WiFi+中断吃其余），消费 8.7 MB/s
    // 钉死与 USB/结构无关。31 抢占 WiFi；WiFi 内部 task watchdog 保命
    // （espsoup FreeRTOS 同环境 40 MB/s 证明 WiFi 留活时间足够）。
    esp_rtos::CurrentThreadHandle::get().set_priority(31);
    spawner.spawn(usb_task(usb_rx, usb_tx).unwrap());
    spawner.spawn(stream_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
