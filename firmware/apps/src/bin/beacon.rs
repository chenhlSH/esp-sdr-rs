//! C5 长时发射信标 v2：最大功率、连续 QPSK、三频点轮换。
//!
//! 相对 v1 的修正（v1 收不到的根因）：
//! - 功率步 0 → **15**：vendor 每步 2 dB 回退，v1 实际衰减 30 dB（比
//!   WiFi 额定功率弱 1000 倍）；
//! - 1 秒通断 → **连续发射**（密度 100%）；
//! - 单频点 2412 → **30 秒轮换 2484 / 5180 / 5745 MHz**（2.4G ISM 上沿
//!   与两个 5G 非 DFS 信道），共 5 分钟。
//!
//! 参数：QPSK 100 kS/s × interp 4 = 400 kS/s DAC（带宽 ≈ ±200 kHz），
//! 多相 Blackman-sinc 成形。发射期间零 defmt 输出（无人读串口时阻塞
//! 会卡死固件）；总结在结束后打印。

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

/// 频点轮换表（ISMS 合法频段：2484 = 2.4G ISM 上沿；
/// 5180 = 5G ch36、5745 = 5G ch149，均非 DFS）。
const HOP_MHZ: [u32; 3] = [2484, 5180, 5745];
/// 每频段 30 秒 × 10 段 = 5 分钟。
const SEGMENT_WINDOWS: u32 = 10;
const WINDOW_SECONDS: u32 = 30;
const WINDOW_CYCLES: u32 = 240_000_000;
/// 最大功率步（无回退，vendor WiFi 额定功率）。
const TX_POWER: u32 = 15;
const SYMBOL_RATE_HZ: u32 = 100_000;
const INTERP: u32 = 4;
const SYMBOL_AMPLITUDE: i16 = 6000;

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
async fn beacon_task(mut radio: Radio) {
    let mut source = QpskSource::new();
    let mut total_samples: u64 = 0;
    let mut total_late: u32 = 0;
    let mut late_max: u32 = 0;

    for segment in 0..SEGMENT_WINDOWS {
        let mhz = HOP_MHZ[segment as usize % HOP_MHZ.len()];
        // 换频 + 重开 TX 链（tone 路径按当前频点调谐）
        radio.set_frequency(mhz).unwrap();
        radio.tx_chain_on(TX_POWER).unwrap();
        let mut paced = TxPaced::start(SYMBOL_RATE_HZ, INTERP).unwrap();
        let segment_start = cycle_count();
        let segment_cycles = WINDOW_CYCLES * WINDOW_SECONDS;
        while cycle_count().wrapping_sub(segment_start) < segment_cycles {
            paced.poll(|| source.next());
        }
        let stats = paced.stats();
        total_samples += stats.samples;
        total_late += stats.late;
        late_max = late_max.max(stats.late_max);
        paced.stop();
        radio.tx_chain_off();
    }

    // 结束：RX 恢复确认 + 总结（发射已结束，打印阻塞无害）
    let mut dump_engine = DumpEngine::new();
    let rx_nonzero = match dump_engine.capture_burst(4096, CaptureRate::Msps20) {
        Ok((words, _)) => words.iter().filter(|word| **word != 0).count(),
        Err(_) => 0,
    };
    defmt::info!(
        "beacon v2 done: 300 s, samples={} late={} late_max={} cycles, RX nonzero={}/4096",
        total_samples,
        total_late,
        late_max,
        rx_nonzero
    );

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
    spawner.spawn(beacon_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}