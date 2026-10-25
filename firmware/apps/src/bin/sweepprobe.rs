//! C2 诊断：lane 配置对照探针。
//! vendor adctrig 只做种子（时钟/打包），随后裸寄存器触发，
//! 对照 vendor lane 与 zephyr IQ lane（信号 24..27）的采样活跃度。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

const COUNT: usize = 16380;
const CAPTURE_BANK_PTR: *mut u32 = 0x4083_0000 as *mut u32;
const SRAM_OWNER_REGISTER: u32 = 0x6009_5004;

unsafe extern "C" {
    fn adctrig(a1: u32, a2: u32, a3: u32, a4: u32, a5: u32, a6: u32, a7: u32, a8: u32, a9: u32);
}

#[unsafe(no_mangle)]
unsafe extern "C" fn cmd_parse(
    _c: *mut u8, _n: *mut u8, _ac: *mut i32, _av: *mut *mut u8,
) -> i32 {
    -1
}

fn rd(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn wr(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn rd_cycles() -> u32 {
    let low: u32;
    unsafe {
        core::arch::asm!("rdcycle {lo}", lo = out(reg) low, options(nomem, nostack, preserves_flags));
    }
    low
}

/// 裸寄存器采集（vendor 种子后）：源 15、指定 lane、指定速率档。
/// 返回（相邻字变化次数, 首字）。
fn raw_capture_with_lanes(lane_value: u32, rate_index: u32) -> (usize, u32) {
    let bank = unsafe { core::slice::from_raw_parts_mut(CAPTURE_BANK_PTR, COUNT) };
    for (index, word) in bank.iter_mut().enumerate() {
        unsafe { core::ptr::write_volatile(word, 0xa5a0_055a ^ index as u32) };
    }
    let owner = rd(SRAM_OWNER_REGISTER);

    wr(0x600a_9004, 0);
    wr(0x600a_9c04, u32::MAX);
    let clk = rd(0x600a_0800);
    wr(0x600a_0800, clk | 0b100);
    let lane = rd(0x600a_9018);
    wr(0x600a_9018, (lane & !0x01FF_FFFF) | (lane_value & 0x01FF_FFFF));
    let mode = rd(0x600a_9008);
    wr(0x600a_9008, (mode & !(0xF << 17)) | (15 << 17));
    wr(SRAM_OWNER_REGISTER, (owner & !(0xF << 8)) | (2 << 8) | (1 << 16));
    unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };

    let m2 = rd(0x600a_9008);
    wr(0x600a_9008, (m2 & !(0x7 << 21)) | ((rate_index & 7) << 21));
    let count_field = (COUNT as u32) & 0x0001_FFFF;
    wr(0x600a_9004, count_field | (1 << 18));
    wr(0x600a_9004, (1 << 31) | count_field);
    wr(0x600a_9004, (1 << 31) | count_field | (1 << 19));
    wr(0x600a_9004, (1 << 31) | count_field);

    let start = rd_cycles();
    loop {
        if rd(0x600a_9004) & (1 << 18) != 0 {
            break;
        }
        if rd_cycles().wrapping_sub(start) > 240_000_000 / 25 {
            break;
        }
    }
    wr(0x600a_9004, 0);
    wr(SRAM_OWNER_REGISTER, owner);
    unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };

    let mut changes = 0usize;
    let mut previous = unsafe { core::ptr::read_volatile(bank.as_ptr()) };
    for &word in bank.iter() {
        if word != previous {
            changes += 1;
            previous = word;
        }
    }
    (changes, bank[0])
}

#[embassy_executor::task]
async fn sweep_task(controller: esp_radio::wifi::WifiController<'static>) {
    // 混杂模式：让接收链持续在线（对齐上游 esp_wifi_set_promiscuous(true)）
    let mut sniffer = controller.sniffer();
    sniffer.set_promiscuous_mode(true).unwrap();
    static PACKET_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    sniffer.set_receive_cb(|_packet| {
        PACKET_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    });

    loop {
        Timer::after(Duration::from_secs(30)).await;
        let packets_before = PACKET_COUNT.load(core::sync::atomic::Ordering::Relaxed);
        defmt::info!("probe begin, packets so far={}", packets_before);

        unsafe {
            adctrig(255, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        let (c1, w1) = raw_capture_with_lanes(0x1006_DA58, 0);
        let packets_after = PACKET_COUNT.load(core::sync::atomic::Ordering::Relaxed);
        defmt::info!(
            "lanes=zephyr: changes={}, first=0x{:08x}, packets {} -> {}",
            c1, w1, packets_before, packets_after
        );
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

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
    controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None).unwrap();

    let mut radio = Radio::new();
    radio.set_frequency(2412).unwrap();
    radio.set_gain_manual(40).unwrap();
    defmt::info!("sweep probe ready");
    drop(radio);

    spawner.spawn(sweep_task(controller).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
