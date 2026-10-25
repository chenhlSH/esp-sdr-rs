//! C4 消费热点基准：无缝流运行中用 rdcycle 实测各段 cycles。
//! 已定位的结论（docs/c5-registers.md §10.3）：CIC.run(4096B) =
//! ~32000 cycles（7.8 cycles/byte，Rust 安全代码 vs zephyr 手写汇编
//! 4.1-4.4）；poll/available 微不足道；消费循环无抢占长间隙。
//! 消费速率钉在生产速率 ~43% 的根因分析见 §10.4（未解项）。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::DiagStream;
use esp_sdr_radio::Radio;
use esp_sdr_shared::dsp::{cic_inverse, normalize, Cic, DIAG_BOX};

esp_bootloader_esp_idf::esp_app_desc!();

const CONSUME_CHUNK: usize = 4096;
const BENCH_BATCHES: usize = 2000;

#[inline(always)]
fn cycle_now() -> u32 {
    let value: u32;
    unsafe { core::arch::asm!("rdcycle {}", out(reg) value, options(nostack)) };
    value
}

#[embassy_executor::task]
async fn diag_probe_task() {
    defmt::info!("diagprobe2: consumption micro-bench start");
    let mut stream = DiagStream::start(0);
    let inverse = cic_inverse(32, 0);
    let mut cic = Cic::new(32);
    let mut acc = [0i32; 2 * (CONSUME_CHUNK / DIAG_BOX / 8 + 4)];

    // 预热：消费一段让环进入稳态
    let mut consumed_offset = 0usize;
    for _ in 0..200 {
        let available = stream.available_at(consumed_offset);
        if available >= DIAG_BOX {
            let take = available.min(CONSUME_CHUNK) & !(DIAG_BOX - 1);
            let bytes = stream.ring_slice_at(consumed_offset, take);
            let _pairs = cic.run(bytes, &mut acc);
            consumed_offset = (consumed_offset + take) % esp_sdr_mmio::RING_BYTES;
        }
    }

    // 分段计时 + 轮耗时分布（定位每轮失踪 cycles：抢占/慢 MMIO/调度）
    let mut cycles_cic: u64 = 0;
    let mut bytes_total: u64 = 0;
    let mut batches: usize = 0;
    let mut consume_batches: usize = 0;
    let mut loop_max: u32 = 0;
    let mut loop_sum: u64 = 0;
    let mut preemptions: u32 = 0; // 轮间空隙 >208 µs 的次数
    let mut t_prev = cycle_now();
    while batches < BENCH_BATCHES {
        let t_loop_start = cycle_now();
        stream.poll_producer();
        let available = stream.available_at(consumed_offset);
        if available >= DIAG_BOX {
            let take = available.min(CONSUME_CHUNK) & !(DIAG_BOX - 1);
            let bytes = stream.ring_slice_at(consumed_offset, take);
            let t2 = cycle_now();
            let pairs = cic.run(bytes, &mut acc);
            let t3 = cycle_now();
            for pair in 0..pairs {
                let _i = normalize(acc[2 * pair], inverse) >> 6;
                let _q = normalize(acc[2 * pair + 1], inverse) >> 6;
            }
            consumed_offset = (consumed_offset + take) % esp_sdr_mmio::RING_BYTES;
            bytes_total += take as u64;
            cycles_cic += u64::from(t3.wrapping_sub(t2));
            consume_batches += 1;
        }
        let t_loop_end = cycle_now();
        let loop_cycles = t_loop_end.wrapping_sub(t_loop_start);
        let gap = t_loop_start.wrapping_sub(t_prev);
        if gap > 50_000 {
            preemptions += 1;
        }
        t_prev = t_loop_end;
        if loop_cycles > loop_max {
            loop_max = loop_cycles;
        }
        loop_sum += u64::from(loop_cycles);
        batches += 1;
    }

    let producer = stream.producer_bytes();
    stream.stop();

    let real_batches = batches.max(1) as u64;
    let per_batch_cic = cycles_cic / consume_batches.max(1) as u64;
    defmt::info!(
        "BENCH: batches={} consume_batches={} bytes={} produced={}",
        batches,
        consume_batches,
        bytes_total,
        producer
    );
    defmt::info!(
        "loop: avg={} max={} preemptions(gap>208us)={}",
        loop_sum / real_batches,
        loop_max,
        preemptions
    );
    defmt::info!(
        "cic/run(4096B)={} cycles => {} cycles/byte (20MB/s needs {} MHz)",
        per_batch_cic,
        per_batch_cic / (CONSUME_CHUNK as u64),
        per_batch_cic / (CONSUME_CHUNK as u64) * 20
    );

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

    // 完整射频 bring-up（C2 验证路径）
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
