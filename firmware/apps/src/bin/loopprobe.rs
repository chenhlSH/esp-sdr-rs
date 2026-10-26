//! C5 duplex 自环验证：TX 恒定载波期间用 RX 链采集自身泄漏，判断
//! 空口发射路径是否工作（zephyr rfprobe "tx carrier" + duplex 同款）。
//!
//! 三步对比（20 MS/s 无需——引擎默认 80 MS/s，采 16380 字）：
//! A. TX 关：RX 基线统计；
//! B. TX 开（功率步 15 = 无回退）+ 恒定 TXDC 载波 + duplex RX on：
//!    若空口路径工作，RX 统计应显著偏离基线（幅度/饱和/均值）；
//! C. TX 关：统计应回到 A 附近（验证恢复）。
//!
//! 判据：B 与 A 的 RMS 差 > 20% 或饱和计数显著 → 发射有输出。
//! 合法频段：2412 MHz（ISM），功率步 15 为 vendor WiFi 额定功率。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::replay::cycle_count;
use esp_sdr_mmio::tx::TxPaced;
use esp_sdr_mmio::{CaptureRate, DumpEngine};
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY_MHZ: u32 = 2412;
const GAIN_CODE: u32 = 40;
/// 最大功率步（无回退，vendor WiFi 额定功率）。
const TX_POWER: u32 = 15;
/// 恒定载波：TXDC I=300、Q=0 → LO 处纯载波（zephyr tx carrier 同款）。
const CARRIER_I: i32 = 300;
const CARRIER_Q: i32 = 0;

/// 整数平方根（牛顿法；no_std 无 f64::sqrt）。
fn isqrt(value: u64) -> u64 {
    if value == 0 {
        return 0;
    }
    let mut guess = value;
    let mut next = (guess + 1) / 2;
    while next < guess {
        guess = next;
        next = (guess + value / guess) / 2;
    }
    guess
}

/// 16380 字的时域统计。
fn statistics(words: &[u32]) -> (i32, u64, u32, u32) {
    let mut sum: i64 = 0;
    let mut square_sum: u64 = 0;
    let mut saturated: u32 = 0;
    for &word in words {
        let i = ((word << 22) as i32) >> 22;
        let q = ((word << 12) as i32) >> 22;
        sum += i64::from(i) + i64::from(q);
        square_sum += (i as i64 * i as i64 + q as i64 * q as i64) as u64;
        if i.abs() >= 500 || q.abs() >= 500 {
            saturated += 1;
        }
    }
    let count = (words.len() * 2) as i64;
    let mean = (sum / count.max(1)) as i32;
    let rms = isqrt(square_sum / count.max(1) as u64);
    (mean, rms, saturated, words.len() as u32)
}

static STATS_A: AtomicU32 = AtomicU32::new(0);

/// vendor DumpEngine 采集一次并统计（adctrig 路径，C2 已证实有效）。
fn capture_statistics(dump_engine: &mut DumpEngine) -> Option<(i32, u64, u32)> {
    match dump_engine.capture_burst(16380, CaptureRate::Msps20) {
        Ok((words, _)) => {
            let (mean, rms, saturated, _) = statistics(words);
            Some((mean, rms, saturated))
        }
        Err(error) => {
            defmt::error!("capture failed: {:?}", error);
            None
        }
    }
}

#[embassy_executor::task]
async fn loop_probe_task(mut radio: Radio) {
    defmt::info!("loopprobe: start ({} MHz, power step {})", FREQUENCY_MHZ, TX_POWER);
    let mut dump_engine = DumpEngine::new();

    // A) 基线（TX 关）
    let Some((mean_a, rms_a, sat_a)) = capture_statistics(&mut dump_engine) else {
        return;
    };
    STATS_A.store(rms_a as u32, Ordering::Relaxed);
    defmt::info!("A baseline (TX off): mean={} rms={} saturated={}", mean_a, rms_a, sat_a);

    // B) TX 载波 + duplex RX 采集
    radio.tx_chain_on(TX_POWER).unwrap();
    let mut paced = TxPaced::start(1_000_000, 1).unwrap();
    // 注入恒定值两次（第二次写入第一次计算的值；之后 DAC 保持该值）
    let mut polls = 0u32;
    while polls < 2 {
        if paced.poll(|| (CARRIER_I as i16 * 64, CARRIER_Q as i16 * 64)) {
            polls += 1;
        }
    }
    radio.rx_chain_duplex_on();
    Timer::after(Duration::from_millis(5)).await;
    let (mean_b, rms_b, sat_b) = capture_statistics(&mut dump_engine).unwrap_or((0, 0, 0));
    let rms_delta_permille = if rms_a > 0 {
        rms_b.abs_diff(rms_a) * 1000 / rms_a
    } else {
        0
    };
    defmt::info!(
        "B duplex (TX on): mean={} rms={} saturated={} rms-delta={}‰",
        mean_b,
        rms_b,
        sat_b,
        rms_delta_permille
    );

    // C) TX 关（恢复）后统计
    paced.stop();
    radio.tx_chain_off();
    Timer::after(Duration::from_millis(5)).await;
    let (mean_c, rms_c, sat_c) = capture_statistics(&mut dump_engine).unwrap_or((0, 0, 0));
    defmt::info!("C after TX off: mean={} rms={} saturated={}", mean_c, rms_c, sat_c);

    // 判定
    let tx_visible = rms_delta_permille > 200 || sat_b > sat_a + 100;
    defmt::info!(
        "VERDICT: TX {} (rms A={} B={} C={}, delta={}‰, sat A={} B={} C={})",
        if tx_visible { "VISIBLE at RX" } else { "NOT VISIBLE at RX" },
        rms_a,
        rms_b,
        rms_c,
        rms_delta_permille,
        sat_a,
        sat_b,
        sat_c
    );
    let _ = cycle_count();

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
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();

    defmt::info!("loopprobe ready");
    spawner.spawn(loop_probe_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}