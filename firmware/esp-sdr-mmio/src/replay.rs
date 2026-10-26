//! C5 TX 回放引擎：调制解调器 DAC 循环回放（zephyr esp_sdr_replay.c 的
//! Rust 重写，寄存器序列逐行对照，docs/c5-registers.md §11）。
//!
//! 数据面：CPU 把 IQ 字（I bits9:0、Q bits19:10，±511，与采集格式对称）
//! 写满块 1 起始后启动引擎；引擎循环读块 1 驱动 DAC，CPU 不参与搬运。
//!
//! 前置条件（由 esp-sdr-radio 负责）：TX 链已开启（vendor CW tone
//! 路径：esp_phy_wifi_tx_tone + phy_stop_tx_tone），频点在合法频段。
//!
//! 半双工：TX 期间接收链关闭；[Replay::stop] 只收引擎，TX 链关闭与 RX
//! 恢复由 radio 层完成。

use core::ptr::{read_volatile, write_volatile};

/// 单次回放最大字数（DAC_TRIG COUNT 字段 14 位，zephyr REPLAY_MAX）。
pub const REPLAY_MAX_WORDS: usize = 16383;

/// 回放缓冲基址（块 1 起始；与采集区同址——半双工不并发）。
const BANK_START_ADDRESS: u32 = 0x4082_0000;

// DAC 触发寄存器（zephyr esp_sdr_priv.h C5 分支）。
const DAC_TRIG_REG: u32 = 0x600a_900c;
const DAC_TRIG_RUN: u32 = 1 << 31;
const DAC_TRIG_REPEAT_SHIFT: u32 = 20;
const DAC_TRIG_LOOP: u32 = 1 << 19;
const DAC_TRIG_DONE: u32 = 1 << 18;
/// 置位 = 40 MS/s；清零 = 80 MS/s（HALF 时钟）。
const DAC_TRIG_HALF: u32 = 1 << 17;
const DAC_TRIG_COUNT_MASK: u32 = 0x3FFF;

// 输出刻度寄存器（每字 I/Q 各 8 位缩放；默认 15——tone 停止留下的
// 满幅会削波，zephyr SCALE_DEFAULT 同值）。
const DAC_SCALE_REG: u32 = 0x600a_0c04;
const DAC_SCALE_MASK: u32 = 0xFF << 8;
const SCALE_DEFAULT: u32 = 15;

// dump engine 寄存器（与 diag.rs 同源；TX 复用同一引擎的块 1 让渡）。
const DUMP_CTRL_REG: u32 = 0x600a_9004;
const DUMP_MODE_REG: u32 = 0x600a_9008;
const DUMP_MODE_SOURCE_MASK: u32 = 0xF << 17;
/// source 15：IQ/回放数据路径（采集与 TX 同值，zephyr DUMP_SOURCE_IQ）。
const DUMP_MODE_SOURCE_IQ: u32 = 15 << 17;
const DUMP_CONFIG_REG: u32 = 0x600a_9018;
const DUMP_CONFIG_LANES_MASK: u32 = 0x01FF_FFFF;
const DUMP_CONFIG_IQ: u32 = 0x1006_DA58;
const DUMP_CLK_FORCE_REG: u32 = 0x600a_9c04;
const DUMP_CLK_EN_REG: u32 = 0x600a_0800;
const DUMP_CLK_EN_BIT: u32 = 1 << 2;
const DUMP_USAGE_REG: u32 = 0x6009_5004;
const DUMP_USAGE_FIELD_MASK: u32 = 0xF << 8;
const DUMP_USAGE_BANK1: u32 = 1 << 9;

/// 回放失败模型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ReplayError {
    /// count 为 0 或超出 16383。
    BadCount { requested: usize },
    /// 等待 DONE 超时（有限回放）。
    Timeout,
}

fn rd(address: u32) -> u32 {
    unsafe { read_volatile(address as *const u32) }
}

fn wr(address: u32, value: u32) {
    unsafe { write_volatile(address as *mut u32, value) }
}

fn fence() {
    unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };
}

/// 回放缓冲（块 1 起始，CPU 拥有时可写）。调用方先填 IQ 字再 start。
#[must_use]
pub fn buffer() -> &'static mut [u32] {
    unsafe { core::slice::from_raw_parts_mut(BANK_START_ADDRESS as *mut u32, REPLAY_MAX_WORDS) }
}

/// 打包一个回放字（10 位有符号 I/Q，±511，与采集格式对称）。
#[must_use]
pub fn replay_word(i: i32, q: i32) -> u32 {
    let i = i.clamp(-511, 511);
    let q = q.clamp(-511, 511);
    ((i as u32) & 0x3FF) | (((q as u32) & 0x3FF) << 10)
}

/// 一次回放会话。Drop 不收尾（显式 stop；RX 恢复需要 radio 参与）。
pub struct Replay {
    saved_usage: u32,
    saved_scale: u32,
    ctrl: u32,
    endless: bool,
    running: bool,
}

impl Replay {
    /// 启动回放。`count` 为字数（≤ 16383）；`rate_40msps=false` 即
    /// 80 MS/s；`passes=0` 为无尽（直到 stop），1..=255 为有限次数。
    /// 前置：缓冲已填、TX 链已开。
    pub fn start(count: usize, rate_40msps: bool, passes: u8, scale: u8) -> Result<Self, ReplayError> {
        if count == 0 || count > REPLAY_MAX_WORDS {
            return Err(ReplayError::BadCount { requested: count });
        }
        let usage_saved = rd(DUMP_USAGE_REG);
        // dump 引擎时钟强制（zephyr esp_sdr_dump_arm 序列的 TX 变体：
        // 所有访问都在 owner 让渡之前）
        wr(DUMP_CTRL_REG, 0);
        wr(DUMP_CLK_FORCE_REG, u32::MAX);
        let clk_en = rd(DUMP_CLK_EN_REG);
        wr(DUMP_CLK_EN_REG, clk_en | DUMP_CLK_EN_BIT);
        let lane = rd(DUMP_CONFIG_REG);
        wr(DUMP_CONFIG_REG, (lane & !DUMP_CONFIG_LANES_MASK) | DUMP_CONFIG_IQ);
        let mode = rd(DUMP_MODE_REG);
        wr(DUMP_MODE_REG, (mode & !DUMP_MODE_SOURCE_MASK) | DUMP_MODE_SOURCE_IQ);
        // 块 1 让渡给引擎并**保持**（TX 期间引擎读块 1 驱动 DAC）
        wr(
            DUMP_USAGE_REG,
            (usage_saved & !DUMP_USAGE_FIELD_MASK) | DUMP_USAGE_BANK1 | 1 << 16,
        );
        fence();
        let _flush = rd(DUMP_USAGE_REG);

        let scale = if scale == 0 { SCALE_DEFAULT } else { u32::from(scale) };
        let saved_scale = rd(DAC_SCALE_REG);
        wr(
            DAC_SCALE_REG,
            (saved_scale & !DAC_SCALE_MASK) | (scale << 16) | (scale << 8),
        );

        let mut ctrl = DAC_TRIG_RUN
            | (count as u32 & DAC_TRIG_COUNT_MASK)
            | if rate_40msps { DAC_TRIG_HALF } else { 0 };
        let endless = passes == 0;
        // zephyr 语义：passes != 1 时设 LOOP + REPEAT=passes——
        // passes=0（REPEAT 0 + LOOP）为无尽，直到 RUN 清零；passes=1
        // 不设 LOOP（单次，DONE 置位结束）。
        if passes != 1 {
            ctrl |= DAC_TRIG_LOOP | ((u32::from(passes) & 0xFF) << DAC_TRIG_REPEAT_SHIFT);
        }
        wr(DAC_TRIG_REG, 0);
        wr(DAC_TRIG_REG, ctrl);
        Ok(Self {
            saved_usage: usage_saved,
            saved_scale,
            ctrl,
            endless,
            running: true,
        })
    }

    /// 当前 DAC_TRIG 寄存器原始值。
    #[must_use]
    pub fn ctrl(&self) -> u32 {
        rd(DAC_TRIG_REG)
    }

    /// 引擎是否置 DONE（有限回放完成；无尽回放永不置位）。
    #[must_use]
    pub fn done(&self) -> bool {
        self.ctrl() & DAC_TRIG_DONE != 0
    }

    /// 无尽回放判定。
    #[must_use]
    pub fn endless(&self) -> bool {
        self.endless
    }

    /// 阻塞等待有限回放完成；返回消耗的 CPU 周期数（rdcycle 差）。
    /// 轮询在临界区内进行（屏蔽中断）：中断延迟会污染 rdcycle 计时，
    /// 实测两次测量差 16%，屏蔽后与引擎时序一致（zephyr 在 irq_lock
    /// 内轮询，同款做法）。
    pub fn wait_done(&self, timeout_cycles: u32) -> Result<u32, ReplayError> {
        assert!(!self.endless, "wait_done on an endless replay");
        critical_section::with(|_| {
            let started = cycle_count();
            loop {
                if self.done() {
                    return Ok(cycle_count().wrapping_sub(started));
                }
                if cycle_count().wrapping_sub(started) >= timeout_cycles {
                    return Err(ReplayError::Timeout);
                }
            }
        })
    }

    /// 重启有限回放（RUN 清零 → 写回 ctrl；zephyr esp_sdr_replay_restart）。
    pub fn restart(&mut self) {
        wr(DAC_TRIG_REG, self.ctrl & !DAC_TRIG_RUN);
        wr(DAC_TRIG_REG, self.ctrl);
    }

    /// 停止并恢复 SRAM 属主与刻度（TX 链关闭由 radio 层完成）。
    pub fn stop(&mut self) {
        if !self.running {
            return;
        }
        wr(DAC_TRIG_REG, self.ctrl & !DAC_TRIG_RUN);
        wr(DAC_TRIG_REG, 0);
        wr(DUMP_USAGE_REG, self.saved_usage);
        fence();
        let _flush = rd(DUMP_USAGE_REG);
        wr(DAC_SCALE_REG, self.saved_scale);
        self.running = false;
    }
}

impl Drop for Replay {
    fn drop(&mut self) {
        self.stop();
    }
}

/// CPU 周期计数（rdcycle；240 MHz 下 1 周期 ≈ 4.17 ns）。
#[must_use]
pub fn cycle_count() -> u32 {
    let value: u32;
    unsafe { core::arch::asm!("rdcycle {}", out(reg) value, options(nostack)) };
    value
}

/// 一次原始 dump 采集（手写序列，zephyr esp_sdr_rx.c capture_locked 同款）：
/// 让渡块 1 给引擎 → 有限计数触发 → 等 DONE → 属主复原 → 返回块 1 的
/// `word_count` 字切片。采样率 = 引擎当前档（默认 80 MS/s；本函数不动
/// 速率字段）。
///
/// 与 [Replay] 互斥：两者都占用 dump engine 的块 1 属主（调用方保证
/// 不同时进行——本函数自带完整的让渡/复原，结束即归还）。
pub fn capture_raw(word_count: usize) -> Result<&'static [u32], ReplayError> {
    if word_count == 0 || word_count > 65_535 {
        return Err(ReplayError::BadCount {
            requested: word_count,
        });
    }
    const DUMP_CTRL_RUN: u32 = 1 << 31;
    const DUMP_CTRL_TRIGGER: u32 = 1 << 19;
    const DUMP_CTRL_DONE: u32 = 1 << 18;

    let usage_saved = rd(DUMP_USAGE_REG);
    wr(DUMP_CTRL_REG, 0);
    wr(DUMP_CLK_FORCE_REG, u32::MAX);
    let clk_en = rd(DUMP_CLK_EN_REG);
    wr(DUMP_CLK_EN_REG, clk_en | DUMP_CLK_EN_BIT);
    let lane = rd(DUMP_CONFIG_REG);
    wr(DUMP_CONFIG_REG, (lane & !DUMP_CONFIG_LANES_MASK) | DUMP_CONFIG_IQ);
    let mode = rd(DUMP_MODE_REG);
    wr(DUMP_MODE_REG, (mode & !DUMP_MODE_SOURCE_MASK) | DUMP_MODE_SOURCE_IQ);
    wr(
        DUMP_USAGE_REG,
        (usage_saved & !DUMP_USAGE_FIELD_MASK) | DUMP_USAGE_BANK1 | 1 << 16,
    );
    fence();
    let _flush = rd(DUMP_USAGE_REG);

    let count = word_count as u32;
    wr(DUMP_CTRL_REG, count | DUMP_CTRL_DONE);
    wr(DUMP_CTRL_REG, count | DUMP_CTRL_RUN);
    wr(DUMP_CTRL_REG, count | DUMP_CTRL_RUN | DUMP_CTRL_TRIGGER);
    wr(DUMP_CTRL_REG, count | DUMP_CTRL_RUN);
    // 有限计数（≤65535）完成时置 DONE；20 ms 超时（240 MHz）
    let started = cycle_count();
    while rd(DUMP_CTRL_REG) & DUMP_CTRL_DONE == 0 {
        if cycle_count().wrapping_sub(started) > 4_800_000 {
            break;
        }
    }
    // 收尾：CTRL=0 + 属主归还 CPU（espsoup：引擎拥有时 CPU 读到 stale 值）
    wr(DUMP_CTRL_REG, 0);
    wr(DUMP_USAGE_REG, usage_saved);
    fence();
    let _flush = rd(DUMP_USAGE_REG);

    Ok(unsafe {
        core::slice::from_raw_parts(BANK_START_ADDRESS as *const u32, word_count)
    })
}