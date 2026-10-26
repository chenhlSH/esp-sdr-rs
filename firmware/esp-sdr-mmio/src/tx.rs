//! C5 paced TX：CPU 逐样本注入 DAC（zephyr esp_sdr_tx_paced.c 的
//! Rust 重写，寄存器与控制流逐行对照，docs/c5-registers.md §12）。
//!
//! 与 replay 引擎的区别：replay 由引擎自由读取块 1（速率由引擎时钟
//! 决定，本板实测 HALF=0 约 138.6 MS/s、HALF=1 约 54.5 MS/s）；paced
//! 由 CPU 以 rdcycle 节拍逐样本写 TXDC 注入寄存器——**速率完全可控**
//! （CPU_HZ 的整数分频），可承载任意符号流与任意插值。
//!
//! 前置：TX 链已开（esp-sdr-radio::tx_chain_on，vendor CW 路径把 DAC
//! 交数据路径）。停止后由 radio 层关链并恢复 RX。

use esp_sdr_shared::tx::Shaper;

/// 注入寄存器（TXDC_REG；zephyr esp_sdr_priv.h C5 分支）。
pub const TXDC_REG: u32 = 0x600a_0c08;
/// I/Q 字段（bits19:0，10 位有符号 ×2）。
const TXDC_IQ_MASK: u32 = 0x000F_FFFF;
/// 注入使能（bit26）：CPU 写入的样本被 DAC 采用。
const TXDC_INJECT: u32 = 1 << 26;

/// CPU 时钟（240 MHz；节拍步长的分母，zephyr ESP_SDR_CPU_HZ）。
pub const CPU_HZ: u32 = 240_000_000;

/// paced TX 失败模型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TxError {
    /// CPU_HZ 不能被 rate×interp 整除（节拍无法整数化）。
    RateNotDivisible { rate_hz: u32, interp: u32 },
    /// 速率/插值超出范围。
    BadRate { rate_hz: u32, interp: u32 },
}

/// paced TX 统计（与 zephyr esp_sdr_tx_paced_stats 对应）。
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub struct TxStats {
    /// 发出的 DAC 样本数。
    pub samples: u64,
    /// 输入端事件数（本实现未使用回调耗尽计，保留字段对齐）。
    pub events: u32,
    /// 迟到 ≥1 个节拍步的样本数。
    pub late: u32,
    /// 最大迟到（CPU 周期）。
    pub late_max: u32,
}

fn rd(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn wr(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn cycles() -> u32 {
    let value: u32;
    unsafe { core::arch::asm!("rdcycle {}", out(reg) value, options(nostack)) };
    value
}

/// 打包 10 位 I/Q 采样字（与 replay/采集格式对称）。
#[must_use]
pub fn iq10(i: i32, q: i32) -> u32 {
    let i = i.clamp(-511, 511);
    let q = q.clamp(-511, 511);
    ((i as u32) & 0x3FF) | (((q as u32) & 0x3FF) << 10)
}

/// paced TX 会话。样本节拍 = step 个 CPU 周期；每次 poll 在到点时写
/// 一个 DAC 样本（到点前返回 false，可由调用方做其它工作）。
pub struct TxPaced {
    /// 节拍步长（CPU 周期/样本 = CPU_HZ / (rate_hz × interp)）。
    step: u32,
    /// 下一个样本的截止时刻（rdcycle 值，回绕比较）。
    target: u32,
    /// TXDC 静态位（保留原值 + INJECT）。
    upper: u32,
    /// 原始 TXDC 值（停止时恢复）。
    original: u32,
    shaper: Shaper,
    interp: u32,
    input_scale: u32,
    stats: TxStats,
    running: bool,
}

impl TxPaced {
    /// 启动 paced TX。`rate_hz` 为**符号/输入速率**（写 DAC 的样本率 =
    /// rate_hz × interp）。`interp` ∈ {1,2,4,8}。
    ///
    /// 前置：TX 链已开（radio::tx_chain_on）。输入样本为 16 位（全幅
    /// 32767 → DAC 511，内部 >>6，interp=1 时直通 10 位）。
    pub fn start(rate_hz: u32, interp: u32) -> Result<Self, TxError> {
        if rate_hz == 0 || !matches!(interp, 1 | 2 | 4 | 8) {
            return Err(TxError::BadRate { rate_hz, interp });
        }
        let sample_rate = rate_hz.saturating_mul(interp);
        if sample_rate == 0 || CPU_HZ % sample_rate != 0 {
            return Err(TxError::RateNotDivisible { rate_hz, interp });
        }
        let step = CPU_HZ / sample_rate;
        let original = rd(TXDC_REG);
        let upper = (original & !TXDC_IQ_MASK) | TXDC_INJECT;
        Ok(Self {
            step,
            target: cycles().wrapping_add(step),
            upper,
            original,
            shaper: Shaper::new(interp as usize),
            interp,
            input_scale: 6,
            stats: TxStats::default(),
            running: true,
        })
    }

    /// 距下一个样本截止还有多少 CPU 周期（0 = 已到点）。
    #[must_use]
    pub fn slack(&self) -> u32 {
        let now = cycles();
        let delta = self.target.wrapping_sub(now) as i32;
        if delta > 0 {
            delta as u32
        } else {
            0
        }
    }

    /// 到点则写入下一个 DAC 样本，返回 true。
    /// `symbol` 提供输入速率的 16 位 I/Q 样本（每次需要新符号时调用
    /// 一次——由 shaper 的相位决定；调用方可返回 (0,0) 表示空闲）。
    pub fn poll(&mut self, mut symbol: impl FnMut() -> (i16, i16)) -> bool {
        if !self.running {
            return false;
        }
        let now = cycles();
        if (now.wrapping_sub(self.target) as i32) < 0 {
            return false;
        }
        let late = now.wrapping_sub(self.target);
        if late > self.stats.late_max {
            self.stats.late_max = late;
        }
        if late >= self.step {
            self.stats.late += 1;
        }
        if self.shaper.symbol_due() {
            let (i, q) = symbol();
            self.shaper.push_if_due(i, q, self.input_scale);
        }
        let (i, q) = self.shaper.next();
        wr(TXDC_REG, self.upper | iq10(i, q));
        self.stats.samples += 1;
        self.target = self.target.wrapping_add(self.step);
        true
    }

    /// 重新对齐节拍（清空相位并重置截止时刻；zephyr paced_sync 同款）。
    pub fn sync(&mut self) {
        self.target = cycles().wrapping_add(self.step);
    }

    /// 统计快照。
    #[must_use]
    pub fn stats(&self) -> TxStats {
        self.stats
    }

    /// 插值因子。
    #[must_use]
    pub fn interp(&self) -> u32 {
        self.interp
    }

    /// 停止：恢复 TXDC 原值（TX 链关闭由 radio 层完成）。
    pub fn stop(&mut self) {
        if !self.running {
            return;
        }
        wr(TXDC_REG, self.original);
        self.running = false;
    }
}

impl Drop for TxPaced {
    fn drop(&mut self) {
        self.stop();
    }
}