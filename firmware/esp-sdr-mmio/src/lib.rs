//! ESP32-C5 SDR 底层寄存器访问层。
//!
//! 当前职责：WiFi 基带 dump engine 的突发采集，走 vendor blob 的 adctrig
//! 入口（与上游 esp-sdr targets/esp32c5/chip.h 的 stock_capture 逐字一致）。
//! 手写寄存器序列的完整文档（含 rev v1.2 实测差异）见 docs/c5-registers.md——
//! C2 实测：手写让渡在 esp_wifi 启动后的运行环境里触发 CPU 锁死（让渡瞬间
//! 有未知上下文访问块 1），vendor 代码内部的处理规避了该问题。
//! 后续检查点落地：DAC 回放引擎（TX）、诊断总线到 PARLIO+GDMA 的无缝采集。
//!
//! 硬件路径（FFI、寄存器）按 riscv32 目标门控；纯逻辑（位域、速率表）
//! 不依赖目标，可在主机端单元测试。

#![no_std]

/// 单次突发采集的最大采样字数（esp-sdr chip.h：IQ_WORDS = 16380）。
pub const SAMPLE_WORD_COUNT_MAX: usize = 16380;
/// 块 1 起始地址（栈运行时范围断言用）。
pub const DUMP_BANK_LOWER_BOUND: u32 = 0x4082_0000;
/// 捕获内存基址（块 1 上半区；MAC_DUMP_ALLOC 使样本落在块内偏移 64 KiB 处）。
pub const CAPTURE_BANK_ADDRESS: u32 = 0x4083_0000;

/// dump engine 采样率档（C5 六档，esp-sdr LIMITS 表）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureRate {
    Msps80,
    Msps40,
    Msps20,
    Msps10,
    Msps8,
    Msps4,
}

impl CaptureRate {
    /// LIMITS 表中的档位索引（进帧头用）。
    #[must_use]
    pub const fn index(self) -> u8 {
        match self {
            CaptureRate::Msps80 => 0,
            CaptureRate::Msps40 => 1,
            CaptureRate::Msps20 => 2,
            CaptureRate::Msps10 => 3,
            CaptureRate::Msps8 => 4,
            CaptureRate::Msps4 => 5,
        }
    }

    /// 标称采样率（Hz）。
    #[must_use]
    pub const fn rate_hz(self) -> u32 {
        match self {
            CaptureRate::Msps80 => 80_000_000,
            CaptureRate::Msps40 => 40_000_000,
            CaptureRate::Msps20 => 20_000_000,
            CaptureRate::Msps10 => 10_000_000,
            CaptureRate::Msps8 => 8_000_000,
            CaptureRate::Msps4 => 4_000_000,
        }
    }
}

/// 突发采集失败的显式故障模型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CaptureError {
    /// 采样字数超出引擎上限。
    SampleWordCountTooLarge {
        requested: usize,
        maximum: usize,
    },
    /// 采集区仍有哨兵字：硬件未写满（对应上游 capture_timeout）。
    CaptureIncomplete {
        remaining_sentinel_words: usize,
    },
    /// 守卫字被改写：硬件过跑（对应上游 capture_overrun）。
    CaptureOverrun,
}

/// 采样字解码转发：I bits 9:0 / Q bits 19:10（定义见 esp-sdr-shared::iq）。
///
/// 注意：esp-sdr-zephyr 的 C5/C6 代码主张相反（I 19:10）——两个已验证项目
/// 矛盾，本仓取上游 how-it-works 文档口径；C4 无缝路径（DIAG 总线）用
/// zephyr 口径，届时以实测偏置信号裁定。
#[must_use]
pub const fn unpack_iq_word(word: u32) -> (i16, i16) {
    esp_sdr_shared::iq::unpack_word(word)
}

// ---- 以下为 riscv32 固件路径（依赖 vendor blob 与硬件寄存器） ----

#[cfg(target_arch = "riscv32")]
mod hardware {
    use core::ptr::{read_volatile, write_volatile};
    use super::{
        CaptureError, CaptureRate, CAPTURE_BANK_ADDRESS, DUMP_BANK_LOWER_BOUND,
        SAMPLE_WORD_COUNT_MAX,
    };

    /// SRAM 属主寄存器（0x60095004）：采集期间块 1 交给 dump engine，结束后恢复。
    const SRAM_OWNER_REGISTER: u32 = 0x6009_5004;
    /// 哨兵字：采集前预填，采集后仍存在 = 硬件未写满（capture_timeout 语义）。
    const SENTINEL_WORD: u32 = 0xa5a0_055a;
    /// 守卫字基值：采集区尾 4 字，采集后被改写 = 过跑。
    const GUARD_WORD_BASE: u32 = 0x5a5a_a5a5;
    const GUARD_WORD_COUNT: usize = 4;

    // vendor blob：librftest.a（vendor/，nm 已验证）。libbttestmode.a 提供
    // librftest shell 所需的 BT 测试符号（ble_rx_start 等）。
    unsafe extern "C" {
        /// vendor 采集触发：阻塞至硬件采集完成。
        /// 上游 C5 调用形态：adctrig(n - 1, 0, 0, divider * 2, 0, 0, 0, 0, 0)。
        fn adctrig(
            sample_count_minus_one: u32,
            arg2: u32,
            arg3: u32,
            clock_divider_doubled: u32,
            arg5: u32,
            arg6: u32,
            arg7: u32,
            arg8: u32,
            arg9: u32,
        );
    }

    /// librftest.a 引用的 shell 钩子；无 shell 时必须存在（上游 receiver.c 同款空实现）。
    #[unsafe(no_mangle)]
    unsafe extern "C" fn cmd_parse(
        _command: *mut u8,
        _name: *mut u8,
        _argc: *mut i32,
        _argv: *mut *mut u8,
    ) -> i32 {
        -1
    }

    #[inline]
    fn read_stack_pointer() -> u32 {
        let stack_pointer: u32;
        // SAFETY: 只读 sp 寄存器，无内存与标志位副作用。
        unsafe {
            core::arch::asm!(
                "mv {sp}, sp",
                sp = out(reg) stack_pointer,
                options(nomem, nostack, preserves_flags)
            );
        }
        stack_pointer
    }

    /// 采集前断言：当前栈不进入块 1。启动栈在 RAM 顶端（块 2），任务栈在堆
    /// （.dram2_uninit，块 2）；违反即内存布局回归，必须响亮崩溃。
    pub fn assert_stack_clear_of_dump_bank() {
        let stack_pointer = read_stack_pointer();
        assert!(
            stack_pointer >= DUMP_BANK_LOWER_BOUND,
            "stack pointer inside dump bank: sp=0x{:08x}",
            stack_pointer
        );
    }

    /// WiFi 基带 dump engine 的突发采集控制。
    ///
    /// 单一所有者约束：同一时刻只允许一个 &mut self 持有者驱动采集。
    pub struct DumpEngine;

    impl DumpEngine {
        #[must_use]
        pub const fn new() -> Self {
            Self
        }

        /// 阻塞式突发采集：写入引擎捕获区（0x40830000），返回零拷贝采样字视图与耗时。
        ///
        /// 前置条件：射频已点亮并完成调谐（建议调谐后 ≥3 ms 再采集，PLL 建立时间）；
        /// 调用任务的栈不在块 1（内部断言）。
        pub fn capture_burst(
            &mut self,
            sample_word_count: usize,
            rate: CaptureRate,
        ) -> Result<(&'static mut [u32], u32), CaptureError> {
            if sample_word_count == 0 || sample_word_count > SAMPLE_WORD_COUNT_MAX {
                return Err(CaptureError::SampleWordCountTooLarge {
                    requested: sample_word_count,
                    maximum: SAMPLE_WORD_COUNT_MAX,
                });
            }
            assert_stack_clear_of_dump_bank();

            let capture_bank = unsafe {
                core::slice::from_raw_parts_mut(
                    CAPTURE_BANK_ADDRESS as *mut u32,
                    sample_word_count + GUARD_WORD_COUNT,
                )
            };

            // 哨兵与守卫预填（此时 CPU 仍是块 1 的属主）。
            // volatile：这些单元将被硬件接管，存储必须真正落地。
            for index in 0..sample_word_count {
                unsafe { write_volatile(capture_bank.as_mut_ptr().add(index), SENTINEL_WORD) };
            }
            for offset in 0..GUARD_WORD_COUNT {
                unsafe {
                    write_volatile(
                        capture_bank.as_mut_ptr().add(sample_word_count + offset),
                        GUARD_WORD_BASE ^ offset as u32,
                    );
                }
            }

            let owner_saved = unsafe { read_volatile(SRAM_OWNER_REGISTER as *const u32) };

            let cycles_start = read_cycle_count();
            // SAFETY: 序列逐字对齐 reference/esp-sdr targets/esp32c5/chip.h
            // 的 stock_capture；块让渡的细节由 vendor 代码处理。
            unsafe {
                adctrig(
                    sample_word_count as u32 - 1,
                    0,
                    0,
                    rate.index() as u32 * 2,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            let cycles_elapsed = read_cycle_count().wrapping_sub(cycles_start);
            // 归还块 1 给 CPU（恢复采集前的属主值）
            unsafe { write_volatile(SRAM_OWNER_REGISTER as *mut u32, owner_saved) };

            // 校验 1：不得残留哨兵（写满才算成功）。
            // volatile 读：值由硬件写入，编译器视角内的"已知值"不可信。
            let mut remaining_sentinel_words = 0usize;
            for index in 0..sample_word_count {
                let word = unsafe { read_volatile(capture_bank.as_ptr().add(index)) };
                if word == SENTINEL_WORD {
                    remaining_sentinel_words += 1;
                }
            }
            if remaining_sentinel_words != 0 {
                return Err(CaptureError::CaptureIncomplete {
                    remaining_sentinel_words,
                });
            }
            // 校验 2：守卫字必须原样（过跑检测）
            for offset in 0..GUARD_WORD_COUNT {
                let word =
                    unsafe { read_volatile(capture_bank.as_ptr().add(sample_word_count + offset)) };
                if word != GUARD_WORD_BASE ^ offset as u32 {
                    return Err(CaptureError::CaptureOverrun);
                }
            }

            let elapsed_us = (cycles_elapsed / CYCLES_PER_US) as u32;
            let sample_words = unsafe {
                core::slice::from_raw_parts_mut(CAPTURE_BANK_ADDRESS as *mut u32, sample_word_count)
            };
            Ok((sample_words, elapsed_us))
        }
    }

    /// CPU 时钟（esp_hal::Config::default 在 C5 上为最高 240 MHz）。
    const CPU_CLOCK_HZ: u64 = 240_000_000;
    /// 周期→微秒换算除数。
    const CYCLES_PER_US: u64 = CPU_CLOCK_HZ / 1_000_000;

    #[inline]
    fn read_cycle_count() -> u64 {
        let low: u32;
        let high: u32;
        // SAFETY: 只读 mcycle/mcycleh CSR。
        unsafe {
            core::arch::asm!(
                "rdcycleh {hi}",
                "rdcycle {lo}",
                hi = out(reg) high,
                lo = out(reg) low,
                options(nomem, nostack, preserves_flags)
            );
        }
        ((high as u64) << 32) | low as u64
    }
}

#[cfg(target_arch = "riscv32")]
pub use hardware::{assert_stack_clear_of_dump_bank, DumpEngine};

/// C4 无缝流：诊断总线 → GPIO 矩阵回环 → PARLIO → GDMA 环。
#[cfg(target_arch = "riscv32")]
pub mod diag;
#[cfg(target_arch = "riscv32")]
pub use diag::{DiagStream, BLOCK_BYTES, DIAG_PINS, RING_BYTES};

#[cfg(test)]
mod tests {
    use super::{unpack_iq_word, CaptureRate};

    #[test]
    fn rate_table_matches_limits() {
        assert_eq!(CaptureRate::Msps80.index(), 0);
        assert_eq!(CaptureRate::Msps4.index(), 5);
        assert_eq!(CaptureRate::Msps80.rate_hz(), 80_000_000);
        assert_eq!(CaptureRate::Msps4.rate_hz(), 4_000_000);
    }

    #[test]
    fn iq_unpack_sign_extends() {
        // I = -1 (10位: 0x3FF), Q = +1
        let word = 0x3FF | (1 << 10);
        assert_eq!(unpack_iq_word(word), (-1, 1));
        // I = -512 (0x200), Q = +511 (0x1FF)
        let word = 0x200 | (0x1FF << 10);
        assert_eq!(unpack_iq_word(word), (-512, 511));
        // I = 0, Q = 0
        assert_eq!(unpack_iq_word(0), (0, 0));
        // 增益字段 bits 27:20 不影响解包
        let word = 0x3FF | (0x3FF << 10) | (0xAB << 20);
        assert_eq!(unpack_iq_word(word), (-1, -1));
    }
}
