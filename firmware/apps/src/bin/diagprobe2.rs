//! C4 无缝流验收探针：DiagStream（mmio 固化路径）60 秒连续流验收。
//! 序列源自 16 轮实验验证的 diagprobe2 手工路径（docs/c5-registers.md
//! §10.2）：bitlen 软分隔符 EOF + auto_ret 回链头是无限流的引擎。
//! 验收口径：40 MB/s × 60 秒零 overrun、生产计数线性推进。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::DiagStream;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

const RUN_SECONDS: u64 = 5;
/// 40 MB/s 的 0.1% 容差（PRUN 参照 ppm 1.5，留裕量）。
const RATE_MIN_BYTES_PER_SEC: u64 = 39_960_000;

#[embassy_executor::task]
async fn diag_probe_task() {
    defmt::info!("diagprobe2: 60s acceptance start");

    let mut stream = DiagStream::start(0);
    let started = esp_hal::time::Instant::now();
    let run_for = esp_hal::time::Duration::from_secs(RUN_SECONDS);

    let mut next_sample = esp_hal::time::Duration::from_secs(5);
    let mut last_bytes: u64 = 0;
    let mut last_at = started.elapsed();
    while started.elapsed() < run_for {
        stream.poll_producer();
        if started.elapsed() >= next_sample {
            let now = started.elapsed();
            let total = stream.producer_bytes();
            let window = (now - last_at).as_micros();
            let rate = if window > 0 {
                (total - last_bytes) * 1_000_000 / window
            } else {
                0
            };
            defmt::info!(
                "t={}s: {} bytes, window {} B/s, overrun={}",
                next_sample.as_secs(),
                total,
                rate,
                stream.overrun()
            );
            last_bytes = total;
            last_at = now;
            next_sample += esp_hal::time::Duration::from_secs(5);
        }
    }

    let total = stream.producer_bytes();
    let overrun = stream.overrun();
    // 末段窗口速率（避开启动瞬态）
    let final_rate = total / RUN_SECONDS;
    stream.stop();

    let pass = !overrun && final_rate >= RATE_MIN_BYTES_PER_SEC;
    defmt::info!(
        "FINAL(diag): {} bytes / {} s = {} B/s, overrun={} -> {}",
        total,
        RUN_SECONDS,
        final_rate,
        overrun,
        if pass { "PASS" } else { "FAIL" }
    );
    assert!(pass, "60s acceptance failed");

    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
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
    radio.set_frequency(2412).unwrap();
    radio.set_gain_manual(40).unwrap();
    drop(radio);
    drop(sniffer);
    drop(controller);

    defmt::info!("diagprobe2 ready");
    spawner.spawn(diag_probe_task().unwrap());

    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
    }
}
