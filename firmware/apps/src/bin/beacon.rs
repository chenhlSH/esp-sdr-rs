//! C5 长时发射信标：QPSK 400 kS/s 符号流，5 分钟，1 秒通断交替。
//!
//! 用途：外部接收机（SDR/从机）验证发射链路。参数：
//! - 中心频率 2412 MHz（ISM 信道 1），功率步 0（厂商最大回退）；
//! - 调制：QPSK 随机符号（xorshift32），多相 Blackman-sinc 成形，
//!   DAC 样本率 400 kS/s（占用带宽 ≈ ±200 kHz）；
//! - 时序：1 秒发射 / 1 秒静默交替，共 300 秒；
//! - 结束后自动停链、恢复 RX 并打印总结。
//!
//! **发射期间零 defmt 输出**：无人读串口时 esp-println 会阻塞并卡死
//! 固件（C2 教训）——总结放在发射结束后打印（此时卡死也无害）。

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
/// 厂商功率步 0 = 最大回退（15 步 × 8 单位）。
const TX_POWER: u32 = 0;
/// 符号速率 100 kS/s × interp 4 = 400 kS/s DAC（实测每样本 ~329 cycles，
/// 上限 ~730 kS/s；400 kS/s 时 CPU 占用约 55%）。
const SYMBOL_RATE_HZ: u32 = 100_000;
const INTERP: u32 = 4;
/// 1 秒窗口 @240 MHz。
const WINDOW_CYCLES: u32 = 240_000_000;
/// 300 个窗口 = 5 分钟。
const WINDOWS: u32 = 300;
/// QPSK 幅度（16 位量纲；Shaper 内 >>6 → DAC ±~500）。
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
    // 发射：5 分钟，1 秒通断
    radio.tx_chain_on(TX_POWER).unwrap();
    let mut paced = TxPaced::start(SYMBOL_RATE_HZ, INTERP).unwrap();
    let mut source = QpskSource::new();

    for window in 0..WINDOWS {
        let transmitting = window % 2 == 0;
        let window_start = cycle_count();
        if transmitting {
            while cycle_count().wrapping_sub(window_start) < WINDOW_CYCLES {
                paced.poll(|| source.next());
            }
        } else {
            // 静默窗口：DAC 样本推 0（无载波）
            while cycle_count().wrapping_sub(window_start) < WINDOW_CYCLES {
                paced.poll(|| (0, 0));
            }
        }
    }
    let samples = paced.stats().samples;
    let late = paced.stats().late;
    let late_max = paced.stats().late_max;
    paced.stop();

    // 结束：停链 + 恢复 RX
    radio.tx_chain_off();
    let mut dump_engine = DumpEngine::new();
    let rx_note = match dump_engine.capture_burst(4096, CaptureRate::Msps20) {
        Ok((words, _)) => {
            let nonzero = words.iter().filter(|word| **word != 0).count();
            nonzero
        }
        Err(_) => 0,
    };
    // 总结（发射已结束；若无人读串口，此打印的阻塞无害）
    defmt::info!(
        "beacon done: 300 s, samples={} late={} late_max={} cycles, RX nonzero={}/4096 restored",
        samples,
        late,
        late_max,
        rx_note
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
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();

    // 无启动打印（发射开始前进入零输出模式）

    spawner.spawn(beacon_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}