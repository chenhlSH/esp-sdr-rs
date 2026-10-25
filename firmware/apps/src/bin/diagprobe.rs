//! C4-A 诊断探针：自由运行 dump → GPIO 矩阵回环 → PARLIO(40 MHz) → GDMA 环。
//! 5 秒轮询生产计数，验证 40 MB/s 无缝采集（预期 ~200 MB 累计、无死锁）。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::DiagStream;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn diag_probe_task() {
    // 寄存器级诊断：启动 2 s 后转储各环节状态
    defmt::info!("diag probe: start");
    let mut stream = DiagStream::start(0);
    Timer::after(Duration::from_secs(2)).await;
    let _ = stream.poll_producer();

    // 描述符前 3 个 dw0
    for index in 0..3 {
        let dw0 = unsafe { core::ptr::read_volatile((0x4082_E000 + index * 12) as *const u32) };
        defmt::info!("desc[{}] dw0=0x{:08x}", index, dw0);
    }

    // GDMA 状态（原始寄存器值）
    let dma = unsafe { esp32c5::DMA::steal() };
    let in_state = dma.ch(0).in_state().read().bits();
    defmt::info!("dma in_state: 0x{:08x}", in_state);
    let int_raw = dma.in_int_ch(0).raw().read().bits();
    defmt::info!("dma int_raw: 0x{:08x}", int_raw);

    // PARLIO 状态（原始寄存器值）
    let parlio = unsafe { esp32c5::PARL_IO::steal() };
    let rx_st0 = parlio.rx_st0().read().bits();
    defmt::info!("parlio rx_st0: 0x{:08x}", rx_st0);
    let int_raw = parlio.int_raw().read().bits();
    defmt::info!("parlio int_raw: 0x{:08x}", int_raw);
    let st = parlio.st().read().bits();
    defmt::info!("parlio st: 0x{:08x}", st);
    // 读回配置寄存器：验证 rx_bitlen 是否被字段宽度截断
    let rx_data_cfg = parlio.rx_data_cfg().read().bits();
    defmt::info!("parlio rx_data_cfg readback: 0x{:08x}", rx_data_cfg);
    let rx_st1 = parlio.rx_st1().read().bits();
    defmt::info!("parlio rx_st1: 0x{:08x}", rx_st1);
    let rx_mode_cfg = parlio.rx_mode_cfg().read().bits();
    defmt::info!("parlio rx_mode_cfg readback: 0x{:08x}", rx_mode_cfg);

    stream.stop();
    defmt::info!("diag probe diag cycle complete, looping producer test");

    loop {
        Timer::after(Duration::from_secs(60)).await;
        let mut stream = DiagStream::start_with_isr(0);
        let started = esp_hal::time::Instant::now();
        let run_for = esp_hal::time::Duration::from_secs(5);
        while started.elapsed() < run_for {
            // ISR 推进计数；这里只周期采样打印
            Timer::after(Duration::from_millis(500)).await;
            let bytes = stream.producer_bytes_isr();
            let mbps_x10 = bytes / 100_000;
            defmt::info!(
                "producer tick: {} bytes ({}.{} MB/s), overrun={}",
                bytes,
                mbps_x10 / 10,
                mbps_x10 % 10,
                stream.overrun_isr()
            );
        }
        let total = stream.producer_bytes_isr();
        let overrun = stream.overrun_isr();
        stream.stop();
        let mbps_x10 = total / 5 / 100_000;
        defmt::info!(
            "FINAL: {} bytes ({}.{} MB/s), overrun={}",
            total,
            mbps_x10 / 10,
            mbps_x10 % 10,
            overrun
        );
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

    defmt::info!("diagprobe ready");
    spawner.spawn(diag_probe_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
