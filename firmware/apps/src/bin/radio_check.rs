//! C1 射频点亮验证：esp-radio 点亮射频（不启动、不关联）→
//! esp-sdr-radio 调谐 2412 MHz + 锁定手动增益 → 20 ms 增益重锁任务。
//!
//! 验收：日志回读频点、已校准增益上限、强制状态位。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

static RADIO: Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, Radio> =
    Mutex::new(Radio::new());

/// AGC 周期性夺回强制增益，~20 ms 重锁一次（esp-sdr-zephyr hardware-quirks.md）。
#[embassy_executor::task]
async fn gain_maintain_task() {
    loop {
        RADIO.lock().await.maintain_gain();
        Timer::after(Duration::from_millis(20)).await;
    }
}

/// 每秒回读射频状态。
#[embassy_executor::task]
async fn status_report_task() {
    loop {
        let radio = RADIO.lock().await;
        defmt::info!(
            "radio: freq={} mhz, gain_mode={:?}, gain_code={}, calibrated_max={}, status_bit23={}",
            radio.frequency_mhz(),
            radio.gain_mode(),
            radio.gain_code(),
            radio.calibrated_gain_maximum(),
            radio.gain_status_bit23(),
        );
        drop(radio);
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("radio_check: boot");

    // esp-radio 要求：堆分配器 + 先启动调度器再初始化射频
    esp_alloc::heap_allocator!(size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timer_group0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 点亮射频电源域与 PHY 校准：建立未启动的 WiFi 控制器（不关联、不收发）
    let _controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())
        .unwrap();
    defmt::info!("radio_check: radio powered");

    let mut radio = RADIO.lock().await;
    match radio.set_frequency(2412) {
        Ok(()) => defmt::info!("radio_check: tuned 2412 mhz"),
        Err(error) => defmt::error!("radio_check: tune failed {:?}", error),
    }
    match radio.set_gain_manual(40) {
        Ok(()) => defmt::info!("radio_check: manual gain 40 locked"),
        Err(error) => defmt::error!("radio_check: gain failed {:?}", error),
    }
    // 越界请求必须显式报错（显式错误故障模型验证）
    match radio.set_gain_manual(999) {
        Err(esp_sdr_radio::RadioError::Gain(
            esp_sdr_radio::GainError::CodeAboveMaximum {
                requested_code,
                calibrated_maximum,
            },
        )) => defmt::info!(
            "radio_check: out-of-range gain rejected as designed: {} > {}",
            requested_code,
            calibrated_maximum
        ),
        _ => defmt::error!("radio_check: out-of-range gain unexpectedly accepted"),
    }
    drop(radio);

    spawner.spawn(gain_maintain_task().unwrap());
    spawner.spawn(status_report_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
