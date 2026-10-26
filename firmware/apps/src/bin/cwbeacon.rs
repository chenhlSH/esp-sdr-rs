//! C5 vendor CW 载波信标：不依赖数据路径的"确定发射"验证。
//!
//! 用 vendor WiFi 单音路径持续发射未调制载波（`esp_phy_wifi_tx_tone(1)`
//! 且不停止）——这是完整 WiFi 发射链（PA/功率表全部激活），也是
//! zephyr rfprobe 的 tone 命令同款。与 paced/replay 数据路径（TXDC
//! 注入 / DAC 回放）完全独立：若本固件可被外部接收机收到而数据路径
//! 信标不可见，则问题在数据路径配置而非功率/天线。
//!
//! 频点：每 30 秒轮换 2484 / 5180 / 5745 MHz，共 5 分钟；
//! 功率步 15（无回退）。发射期间零 defmt 输出。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::replay::cycle_count;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

/// 2.4G ISM 上沿 + 两个 5G 非 DFS 信道。
const HOP_MHZ: [u32; 3] = [2484, 5180, 5745];
const SEGMENTS: u32 = 10;
const WINDOW_CYCLES: u32 = 240_000_000; // 1 秒
const SEGMENT_SECONDS: u32 = 30;
const TX_POWER: u32 = 15;

#[embassy_executor::task]
async fn cw_beacon_task(mut radio: Radio) {
    for segment in 0..SEGMENTS {
        let mhz = HOP_MHZ[segment as usize % HOP_MHZ.len()];
        radio.set_frequency(mhz).unwrap();
        radio.tx_cw_on(TX_POWER).unwrap();
        let started = cycle_count();
        let limit = WINDOW_CYCLES * SEGMENT_SECONDS;
        while cycle_count().wrapping_sub(started) < limit {
            core::hint::spin_loop();
        }
        radio.tx_cw_off();
    }
    // 总结（发射结束后打印）
    defmt::info!("cwbeacon done: 5 min CW over 2484/5180/5745 MHz");
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

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
    drop(controller);

    let mut radio = Radio::new();
    radio.set_frequency(HOP_MHZ[0]).unwrap();
    radio.set_gain_manual(40).unwrap();

    // 无启动打印（进入零输出发射模式）
    spawner.spawn(cw_beacon_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}