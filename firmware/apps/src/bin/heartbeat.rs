#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn heartbeat_task() {
    loop {
        defmt::info!("heartbeat");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("esp-sdr heartbeat start");

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    spawner.spawn(heartbeat_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
