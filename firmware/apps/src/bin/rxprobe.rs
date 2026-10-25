//! C2 诊断：USB-SJ RX 通路回显探针。收到任何字节即打日志。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let (mut usb_rx, mut usb_tx) = usb.split();

    defmt::info!("rx echo probe ready");
    loop {
        match usb_rx.read_byte() {
            Ok(byte) => {
                defmt::info!("rx byte: 0x{:02x}", byte);
                // 回显（嵌入式 IO Write 的阻塞写，逐字节）
                use embedded_io::Write as _;
                let _ = usb_tx.write_all(&[byte]);
                let _ = usb_tx.flush();
            }
            Err(_) => Timer::after(Duration::from_millis(20)).await,
        }
    }
}
