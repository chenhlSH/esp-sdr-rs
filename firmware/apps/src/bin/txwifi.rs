//! C5 raw 802.11 Beacon 发射：用 vendor WiFi 协议栈的 raw TX
//! （`esp_wifi_80211_tx`）周期发送手工构造的 Beacon 帧，让普通 WiFi
//! 设备（本机 PC 网卡）扫描到 SSID。
//!
//! 与 beacon/cwbeacon 的区别：PHY 编码（前导/OFDM/DSSS/速率）完全交给
//! vendor WiFi 栈——这是"真正能被 WiFi 协议栈解析"的帧，而非裸波形。
//! 帧构造见 `shared::wifi_beacon`（纯逻辑，主机已单测）。
//!
//! 参数：SSID "ESP-SDR-TX"、信道 1（2412 MHz）、Beacon 间隔 100 TU、
//! 未加密 ESS。每 100 ms 发一次，持续 10 分钟。
//!
//! 前置：STA 配置已 start 且未连接（raw TX 的允许状态），混杂模式开启
//! （同时保活接收链）。发射期间零 defmt 输出（避免无人读串口阻塞）；
//! 结果由 PC 侧 `netsh wlan show networks` 观察。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_shared::wifi_beacon;
use esp_wifi_sys_esp32c5::include::{
    esp_wifi_80211_tx, esp_wifi_start, wifi_interface_t_WIFI_IF_STA,
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &[u8] = b"ESP-SDR-TX";
const CHANNEL: u8 = 1;
/// SA/BSSID：本地管理 MAC（0x02 前缀）。
const MAC: [u8; 6] = [0x02, 0x45, 0x53, 0x50, 0x44, 0x52];
/// 发射时长：10 分钟（600 × 1 秒）。
const RUN_SECONDS: u32 = 600;

#[embassy_executor::task]
async fn tx_wifi_task(controller: esp_radio::wifi::WifiController<'static>) {
    // controller 必须保持存活：其 Drop（引用计数归零）会 deinit WiFi，
    // 之后 esp_wifi_start/80211_tx 全部报 ESP_ERR_WIFI_NOT_INIT（实测
    // 0x3001/0x3004——正是 drop(controller) 后调用的症状）。
    let _controller = controller;
    let mut frame = [0u8; wifi_beacon::MAX_FRAME_BYTES];
    let length = wifi_beacon::build(&mut frame, MAC, SSID, CHANNEL);

    // 启动 WiFi（esp-radio 的 set_config 只在 mode 变化时调 esp_wifi_start；
    // 初始配置已是目标模式时从未启动。幂等补一次。）
    let start_result = unsafe { esp_wifi_start() };
    defmt::info!("esp_wifi_start -> {} (0=OK)", start_result);

    // 前 8 次打印返回码（诊断阶段；之后零输出）
    let mut attempt: u32 = 0;
    for _second in 0..RUN_SECONDS {
        let second_start = esp_hal::time::Instant::now();
        while second_start.elapsed() < esp_hal::time::Duration::from_secs(1) {
            // 每 100 ms 一个 Beacon；en_sys_seq=true 由驱动填序列号
            let result = unsafe {
                esp_wifi_80211_tx(
                    wifi_interface_t_WIFI_IF_STA,
                    frame.as_ptr() as *const _,
                    length as i32,
                    true,
                )
            };
            if attempt < 8 {
                defmt::info!("80211_tx[{}] -> {} (len {})", attempt, result, length);
                attempt += 1;
            } else {
                attempt += 1;
            }
            Timer::after(Duration::from_millis(100)).await;
        }
    }

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

// STA 模式（默认初始配置）：未连接时即可发 raw beacon/probe 帧
    // （IDF 文档明确允许；这也是标准的 beacon 注入配置）。AP 模式实测
    // 发送成功（返回值 0）但外部扫描不到，故改 STA。
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    controller
        .set_channel(CHANNEL as u8, esp_radio::wifi::SecondaryChannel::None)
        .unwrap();

    // 无启动打印（进入零输出发射模式；结果由 PC 扫描观察）
    // controller 传入任务保持存活（drop 会 deinit WiFi）
    spawner.spawn(tx_wifi_task(controller).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}