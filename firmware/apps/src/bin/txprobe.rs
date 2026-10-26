//! C5a TX 回放探针：DAC 引擎循环回放单音（zephyr esp_sdr_replay 同款）。
//!
//! 流程：射频 bring-up → TX 链开（最低功率步 0）→ 填 4096 字整周期
//! 单音（80 MS/s，19.53 kHz 偏置）→ 有限回放测耗时（常量 DONE 语义）
//! → 无尽回放 200 ms 验证 RUN/DONE 状态 → 停止 → TX 链关 + RX 恢复
//! → 短采集验证接收链恢复。
//!
//! 合法性：2412 MHz（ISM 信道 1）、功率步 0（最大回退）、总发射 <0.5 秒。
//! 数据面不验证空口（半双工无法自收；zephyr 以 ctrl/cycles 为验收标准）。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::replay::{buffer, cycle_count, Replay, REPLAY_MAX_WORDS};
use esp_sdr_mmio::{CaptureRate, DumpEngine};
use esp_sdr_radio::Radio;
use esp_sdr_shared::tone::fill_tone;

esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY_MHZ: u32 = 2412;
const GAIN_CODE: u32 = 40;
/// TX 功率步：0 = 最大回退（15 步 × 8 = 120 单位），合法频段内最小暴露。
const TX_POWER: u32 = 0;
/// 单音字数：4096 @80 MS/s = 51.2 µs/次；1 整周期 = 19.53 kHz 偏置。
const TONE_WORDS: usize = 4096;
const TONE_CYCLES: u32 = 1;
const TONE_AMPLITUDE: i32 = 300;

#[embassy_executor::task]
async fn tx_probe_task(mut radio: Radio) {
    defmt::info!("txprobe: start");

    // 填单音（块 1 起始；TX 与采集同址、半双工不并发）
    {
        let replay_buffer = buffer();
        assert!(TONE_WORDS <= REPLAY_MAX_WORDS);
        fill_tone(&mut replay_buffer[..TONE_WORDS], TONE_CYCLES, TONE_AMPLITUDE);
        defmt::info!(
            "tone filled: {} words, {} cycles, amp {}/{}, first word 0x{:08x}",
            TONE_WORDS,
            TONE_CYCLES,
            TONE_AMPLITUDE,
            REPLAY_MAX_WORDS,
            replay_buffer[0]
        );
        // 相位 0：I = 0，Q ≈ +amp（cos）
        assert_eq!(replay_buffer[0] & 0x3FF, 0, "I at phase 0 must be 0");
        let q0 = ((replay_buffer[0] >> 10) & 0x3FF) as i16;
        let q0signed = if q0 & 0x200 != 0 { q0 - 1024 } else { q0 };
        assert!((q0signed - TONE_AMPLITUDE as i16).abs() <= 4, "Q at phase 0 must be ~amp");
    }

    // ---- 阶段 1：TX 链开（单音路径交 DAC）----
    radio.tx_chain_on(TX_POWER).unwrap();
    defmt::info!("tx chain on (power step {})", TX_POWER);

    // ---- 阶段 2：有限回放 passes=1，测 DAC 完成耗时 ----
    // 期望：4096 字 @80 MS/s = 51.2 µs = 12288 cycles @240 MHz（±测量开销）
    {
        let mut replay = Replay::start(TONE_WORDS, false, 1, 0).unwrap();
        defmt::info!("finite replay armed, ctrl=0x{:08x}", replay.ctrl());
        match replay.wait_done(240_000_000) {
            Ok(cycles) => {
                // 实测反推 DAC 速率：count / (cycles / 240 MHz)。
                // zephyr 标称 80/40 MS/s（HALF 位），本板实测约 120 MS/s
                // （8140 cycles / 4096 字）——时钟基准差异待 C5b 标定。
                let elapsed_ns = u64::from(cycles) * 1_000_000 / 240_000;
                let rate_khz = if elapsed_ns > 0 {
                    TONE_WORDS as u64 * 1_000_000 / elapsed_ns * 1000
                } else {
                    0
                };
                defmt::info!(
                    "finite replay done: {} cycles = {} ns, implied DAC {} kHz, ctrl=0x{:08x} done={}",
                    cycles,
                    elapsed_ns,
                    rate_khz,
                    replay.ctrl(),
                    replay.done()
                );
            }
            Err(error) => defmt::error!("finite replay wait failed: {:?}", error),
        }
        replay.stop();
    }

    // ---- 阶段 3：无尽回放 200 ms（RUN 置位、DONE 不置位）----
    {
        let mut replay = Replay::start(TONE_WORDS, false, 0, 0).unwrap();
        let started = cycle_count();
        let mut polls: u32 = 0;
        let mut done_seen = false;
        while cycle_count().wrapping_sub(started) < 48_000_000 {
            // 200 ms @240 MHz
            if replay.done() {
                done_seen = true;
            }
            polls = polls.wrapping_add(1);
        }
        let ctrl = replay.ctrl();
        defmt::info!(
            "endless replay 200 ms: ctrl=0x{:08x} run={} done_seen={} polls={} (arms={})",
            ctrl,
            ctrl & (1 << 31) != 0,
            done_seen,
            polls,
            ctrl & 0x3FFF
        );
        assert!(ctrl & (1 << 31) != 0, "RUN must stay set for endless replay");
        assert!(!done_seen, "DONE must never set for endless replay");
        replay.stop();
    }

    // ---- 阶段 4：TX 链关 + RX 恢复，短采集验证接收链活着 ----
    radio.tx_chain_off();
    defmt::info!("tx chain off, RX restored");
    {
        let mut dump_engine = DumpEngine::new();
        match dump_engine.capture_burst(4096, CaptureRate::Msps20) {
            Ok((words, _)) => {
                let mut checksum: u64 = 0;
                let mut nonzero = 0u32;
                for word in words.iter().copied() {
                    checksum = checksum.wrapping_add(u64::from(word));
                    if word != 0 {
                        nonzero += 1;
                    }
                }
                defmt::info!(
                    "RX after TX: 4096 words checksum=0x{:016x} nonzero={}",
                    checksum,
                    nonzero
                );
            }
            Err(error) => defmt::error!("RX capture after TX failed: {:?}", error),
        }
    }

    defmt::info!("txprobe: done");
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

    // 射频 bring-up（WiFi 栈建立 PHY；不做 sniffer——TX 不需要接收链）
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
    // 仅创建 sniffer handle（避免接收链在 TX 前被 WiFi 下电影响）；
    // 单独 TX 链由 vendor tone 路径控制
    drop(controller);

    let mut radio = Radio::new();
    radio.set_frequency(FREQUENCY_MHZ).unwrap();
    radio.set_gain_manual(GAIN_CODE).unwrap();

    defmt::info!("txprobe ready");
    spawner.spawn(tx_probe_task(radio).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}