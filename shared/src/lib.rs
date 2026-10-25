//! 固件与主机工具共用的纯逻辑层：帧协议、寄存器位域、定点 DSP。
//!
//! 约束：no_std 兼容、无 unsafe、无堆分配、全部纯函数，
//! 因此同一份代码在主机端跑单元测试，在固件端零拷贝使用。

#![no_std]

/// 频率与调谐的纯逻辑（esp-sdr common/rx_tuning.h 的 C5 规则）。
pub mod frequency {
    /// 软件调谐限幅，与射频标称频段无关（reference/esp-sdr common/rx_tuning.h）。
    pub const MIN_MHZ: u32 = 100;
    pub const MAX_MHZ: u32 = 6000;

    /// C5 的 phy_set_chanfreq 直接收 MHz；此函数决定是否放行。
    #[must_use]
    pub const fn is_valid(mhz: u32) -> bool {
        mhz >= MIN_MHZ && mhz <= MAX_MHZ
    }
}

/// 增益的纯逻辑（esp-sdr common/burst_gain.h 的 C5 规则）。
pub mod gain {
    /// AGCPWR_CTRL7（0x600a702c）bits 8..14：调谐时由 PHY 填入的最大已校准低表索引。
    pub const STATUS_MAX_INDEX_SHIFT: u32 = 8;
    pub const STATUS_MAX_INDEX_MASK: u32 = 0x7f;

    /// C5 允许的最大增益码：索引必须落在已校准表内（< 90），否则视为 0。
    #[must_use]
    pub const fn max_from_status_register(register_value: u32) -> u32 {
        let calibrated_max = (register_value >> STATUS_MAX_INDEX_SHIFT) & STATUS_MAX_INDEX_MASK;
        if calibrated_max < 90 {
            calibrated_max
        } else {
            0
        }
    }

    /// 增益状态位（0x600a702c bit 23）。
    #[must_use]
    pub const fn forced_from_status_register(register_value: u32) -> bool {
        (register_value >> 23) & 1 == 1
    }

    /// 把请求的增益码截断到合法区间 [0, calibrated_max]。
    #[must_use]
    pub const fn clamp_code(requested: u32, calibrated_max: u32) -> u32 {
        if requested > calibrated_max {
            calibrated_max
        } else {
            requested
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{frequency, gain};

    #[test]
    fn frequency_bounds() {
        assert!(!frequency::is_valid(frequency::MIN_MHZ - 1));
        assert!(frequency::is_valid(frequency::MIN_MHZ));
        assert!(frequency::is_valid(2412));
        assert!(frequency::is_valid(5500));
        assert!(frequency::is_valid(frequency::MAX_MHZ));
        assert!(!frequency::is_valid(frequency::MAX_MHZ + 1));
        assert!(!frequency::is_valid(0));
    }

    #[test]
    fn gain_max_extracts_calibrated_index() {
        // 索引 48 在 bits 8..14，bit 23 状态位为 1
        let register_value = (48 << gain::STATUS_MAX_INDEX_SHIFT) | (1 << 23);
        assert_eq!(gain::max_from_status_register(register_value), 48);
        assert!(gain::forced_from_status_register(register_value));
    }

    #[test]
    fn gain_max_rejects_uncalibrated() {
        // 字段仅 7 位；90..127 表示超出已校准表，视为 0
        let register_value = 90 << gain::STATUS_MAX_INDEX_SHIFT;
        assert_eq!(gain::max_from_status_register(register_value), 0);
        let register_value = 127 << gain::STATUS_MAX_INDEX_SHIFT;
        assert_eq!(gain::max_from_status_register(register_value), 0);
    }

    #[test]
    fn gain_clamp_saturates() {
        assert_eq!(gain::clamp_code(40, 48), 40);
        assert_eq!(gain::clamp_code(60, 48), 48);
        assert_eq!(gain::clamp_code(0, 0), 0);
    }
}
