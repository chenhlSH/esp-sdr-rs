//! C5b TX 符号流探针：paced TX（CPU 逐样本注入 DAC）+ QPSK 符号流
//! + 半双工切换时序实测。
//!
//! 流程：
//! 1. 基带符号：伪随机 QPSK（±1±j）× 幅度，符号速率 1 MS/s；
//! 2. paced TX：interp=4 → DAC 样本率 4 MS/s（step=60 cycles），
//!    多相 Blackman-sinc 成形（shared::tx Shaper）；
//! 3. 跑 100 ms，统计样本数/迟到（late/late_max）；
//! 4. 停止 → TX 链关 → **半双工恢复时序**：从 tx_chain_off 到首次
//!    成功采集的 CPU 周期数（连续 capture_burst 计时）；
//! 5. RX 采集 4096 字校验。
//!
//! 合法性：2412 MHz ISM、功率步 0（最大回退）、总发射 ~100 ms。

#![no_std]
#![no_main]

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
const TX_POWER: u32 = 0;
/// 符号速率与插值：DAC 样本率 = 100 kS/s × 4 = 400 kS/s。
/// 实测 TXDC 注入路径每样本 ~329 cycles（跨时钟域写主导）→ 可持续上限
/// ≈730 kS/s；400 kS/s 留足裕量（目标 late=0）。
const SYMBOL_RATE_HZ: u32 = 100_000;
const INTERP: u32 = 4;
/// 发射时长（CPU 周期；100 ms @240 MHz）。
const TX_CYCLES: u32 = 24_000_000;
/// QPSK 幅度（16 位量纲；Shaping 内部 >>6 → DAC ±~500）。
const SYMBOL_AMPLITUDE: i16 = 6000;

/// 伪随机 QPSK 符号（xorshift32；±1±j 等概率）。
struct QpskSource {
    state: u32,
}

impl QpskSource {
    fn new() -> Self {
        Self { state: 0x2545_F491 }
    }

    fn next(&mut self) -> (i16, i16) {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        let i = if x & 1 != 0 { SYMBOL_AMPLITUDE } else { -SYMBOL_AMPLITUDE };
        let q = if x & 2 != 0 { SYMBOL_AMPLITUDE } else { -SYMBOL_AMPLITUDE };
        (i, q)
    }
}

#[embassy_executor::task]
async fn tx_symbol_task(mut radio: Radio) {
    defmt::info!("txsym: start");

    // ---- TX 链开 ----
    radio.tx_chain_on(TX_POWER).unwrap();
    defmt::info!("tx chain on (power step {}), QPSK {} kS/s x{}", TX_POWER, SYMBOL_RATE_HZ / 1_000, INTERP);

    // ---- paced TX 100 ms ----
    {
        let mut paced = match TxPaced::start(SYMBOL_RATE_HZ, INTERP) {
            Ok(paced) => paced,
            Err(error) => {
                defmt::error!("paced start failed: {:?}", error);
                radio.tx_chain_off();
                return;
            }
        };
        let mut source = QpskSource::new();
        let started = cycle_count();
        let mut polls: u32 = 0;
        while cycle_count().wrapping_sub(started) < TX_CYCLES {
            if paced.poll(|| source.next()) {
                // 样本已注入
            }
            polls = polls.wrapping_add(1);
        }
        let stats = paced.stats();
        let elapsed = cycle_count().wrapping_sub(started);
        let sample_rate_khz = stats.samples as u64 * 240_000 / u64::from(elapsed.max(1));
        defmt::info!(
            "paced TX: samples={} polls={} elapsed={} cycles (~{} kHz DAC), late={} late_max={} cycles",
            stats.samples,
            polls,
            elapsed,
            sample_rate_khz,
            stats.late,
            stats.late_max
        );
        paced.stop();
    }

    // ---- 半双工切换时序：TX 链关 → 首次成功采集 ----
    let switch_start = cycle_count();
    radio.tx_chain_off();
    let chain_off_cycles = cycle_count().wrapping_sub(switch_start);
    defmt::info!("tx chain off took {} cycles", chain_off_cycles);

    {
        let mut dump_engine = DumpEngine::new();
        let mut attempts = 0u32;
        let capture_start = cycle_count();
        loop {
            attempts += 1;
            match dump_engine.capture_burst(4096, CaptureRate::Msps20) {
                Ok((words, _)) => {
                    let first_capture_cycles = cycle_count().wrapping_sub(capture_start);
                    let mut nonzero = 0u32;
                    for word in words.iter().copied() {
                        if word != 0 {
                            nonzero += 1;
                        }
                    }
                    defmt::info!(
                        "RX after TX: attempts={} first-capture={} cycles, 4096 words nonzero={}",
                        attempts,
                        first_capture_cycles,
                        nonzero
                    );
                    break;
                }
                Err(error) => {
                    defmt::warn!("capture attempt {} failed: {:?}", attempts, error);
                    if attempts >= 10 {
                        defmt::error!("RX did not recover after 10 attempts");
                        break;
                    }
                }
            }
        }
    }

    defmt::info!("txsym: done");
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    // 显式 240 MHz（esp-hal C5 默认 80 MHz）
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

    defmt::info!("txsym ready");
    spawner.spawn(tx_symbol_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}