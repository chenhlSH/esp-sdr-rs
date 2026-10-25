//! C2 突发采集 PoC：调谐 2412 MHz + 锁增益 → 每 1 s 一次 80 MS/s 突发采集
//! → 帧头+CRC32 经 USB-Serial/JTAG 下机（载荷零拷贝直写）。
//!
//! 内存布局：堆全部移出块 1（64 KiB 在 .dram2_uninit/块 2 + 24 KiB 在 .bss），
//! 链接脚本断言静态内存不越过 0x40820000，采集前断言栈指针不在块 1。
//! 主机握手：收到 'G' 才开始采集（避免 USB 写阻塞在无人读取的 FIFO 上）。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU16, Ordering};
use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_println as _;
use esp_sdr_mmio::{CaptureRate, DumpEngine};
use esp_sdr_radio::Radio;
use esp_sdr_shared::frame;

esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY_MHZ: u32 = 2412;
const GAIN_CODE: u32 = 40;
const CAPTURE_WORD_COUNT: usize = 16380;
/// 突发周期：USB-Serial/JTAG 实测 ~330 kB/s，每帧 IQ8 16 KB 约 50 ms，
/// 100 ms 周期 ≈ 8.2 MS/s 等效交付（10 Hz 突发）。
const CAPTURE_PERIOD_MS: u64 = 100;

static RADIO: Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, Radio> =
    Mutex::new(Radio::new());
static SEQUENCE: AtomicU16 = AtomicU16::new(0);

/// 有界发送：write_byte_nb 逐字节写 + 总超时。FIFO 长期不排空（主机未接入，
/// USB-SJ 的 TX 在 DTR=off 时发过的数据会卡死端点）则整体放弃本帧，
/// 下一采集周期重试；主机接入后 FIFO 排空，帧流自然恢复。
fn send_frame_bounded(
    usb_tx: &mut UsbSerialJtagTx<'static, esp_hal::Blocking>,
    bytes: &[u8],
) -> bool {
    const TX_TIMEOUT_MS: u64 = 500;
    let deadline = esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(TX_TIMEOUT_MS);
    for byte in bytes {
        loop {
            match usb_tx.write_byte_nb(*byte) {
                Ok(()) => break,
                Err(nb::Error::WouldBlock) => {
                    if esp_hal::time::Instant::now() > deadline {
                        return false;
                    }
                }
                Err(_) => return false,
            }
        }
    }
    loop {
        match usb_tx.flush_tx_nb() {
            Ok(()) => return true,
            Err(nb::Error::WouldBlock) => {
                if esp_hal::time::Instant::now() > deadline {
                    return false;
                }
            }
            Err(_) => return false,
        }
    }
}

/// 采集任务：等主机发 'G' 握手（20 s 超时兜底）后周期性突发采集 + 组帧 + 有界发送。
/// 阻塞只发生在本任务线程，增益维护任务（独立线程）不受影响。
#[embassy_executor::task]
async fn capture_task(
    mut usb_rx: UsbSerialJtagRx<'static, esp_hal::Blocking>,
    mut usb_tx: UsbSerialJtagTx<'static, esp_hal::Blocking>,
) {
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
            defmt::warn!("handshake timeout, capturing anyway");
            break 'handshake;
        }
    }
    defmt::info!("capturing");

    let mut dump_engine = DumpEngine::new();
    let mut tx_stall_logged = false;
    loop {
        Timer::after(Duration::from_millis(CAPTURE_PERIOD_MS)).await;

        let radio = RADIO.lock().await;
        let capture_result = dump_engine.capture_burst(CAPTURE_WORD_COUNT, CaptureRate::Msps80);
        let (sample_words, elapsed_us) = match capture_result {
            Ok(capture) => capture,
            Err(error) => {
                defmt::error!("capture failed {:?}", error);
                drop(radio);
                continue;
            }
        };

        // 载荷零拷贝：捕获区 u32 数组直接按字节视图发送（LE 一致）
        let payload_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(sample_words.as_ptr().cast::<u8>(), sample_words.len() * 4)
        };

        let mut header = frame::CaptureFrameHeader {
            frame_type: frame::CAPTURE_FRAME_TYPE,
            rate_index: CaptureRate::Msps80.index(),
            sequence: SEQUENCE.fetch_add(1, Ordering::Relaxed),
            sample_word_count: sample_words.len() as u16,
            frequency_mhz: radio.frequency_mhz(),
            elapsed_us,
            crc32: 0,
        };
        let mut header_bytes = header.encode();
        let mut crc_context = frame::Crc32::new();
        crc_context.update(&header_bytes[0..frame::CRC_COVERED_HEADER_LEN]);
        crc_context.update(payload_bytes);
        header.crc32 = crc_context.finalize();
        header_bytes = header.encode();

        drop(radio); // 释放射频锁再进入 USB 写

        // 先帧头后载荷：两次有界发送（共享同一个 500 ms 预算的简化：各自计时）
        let header_sent = send_frame_bounded(&mut usb_tx, &header_bytes);
        let payload_sent = if header_sent {
            send_frame_bounded(&mut usb_tx, payload_bytes)
        } else {
            false
        };
        if !(header_sent && payload_sent) {
            if !tx_stall_logged {
                defmt::warn!("tx stalled (host absent?), dropping frames until it drains");
                tx_stall_logged = true;
            }
            continue;
        }
        tx_stall_logged = false;
    }
}

/// 增益维护任务：AGC 周期性夺回强制增益，~20 ms 重锁一次
/// （esp-sdr-zephyr hardware-quirks.md）。
#[embassy_executor::task]
async fn gain_maintain_task() {
    loop {
        RADIO.lock().await.maintain_gain();
        Timer::after(Duration::from_millis(20)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 完整射频 bring-up（对齐上游 app_main 的 esp_wifi_start 路径）：
    // WifiController::new = esp_wifi_init；set_config 触发 esp_wifi_start；
    // STA 不连接仅点亮 MAC。控制器保持存活（Drop 会停止 WiFi）。
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_config(&esp_radio::wifi::Config::Station(Default::default()))
        .unwrap();
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None).unwrap();

    // 关键（C2 现场教训）：混杂模式必须开启——未连接的 STA 接收链会下电，
    // ADC 输出冻结为常量，dump engine 采到的是死总线。开启后 RX 持续在线。
    let sniffer = controller.sniffer();
    sniffer.set_promiscuous_mode(true).unwrap();

    let mut radio = RADIO.lock().await;
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();
    drop(radio);

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let (usb_rx, usb_tx) = usb.split();

    defmt::info!("capburst ready: freq={} mhz, gain={}", FREQUENCY_MHZ, GAIN_CODE);

    spawner.spawn(capture_task(usb_rx, usb_tx).unwrap());
    spawner.spawn(gain_maintain_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
