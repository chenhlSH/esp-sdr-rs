//! 固件与主机工具共用的纯逻辑层：帧协议、寄存器位域、定点 DSP。
//!
//! 约束：no_std 兼容、无 unsafe、无堆分配、全部纯函数，
//! 因此同一份代码在主机端跑单元测试，在固件端零拷贝使用。

#![no_std]

/// 诊断总线流的定点信号处理（box16 + CIC + 归一化），主机可测。
pub mod dsp;

/// TX 多相脉冲成形（Blackman-sinc 低通，zephyr esp_sdr_paced.c
/// fir_design 同款；表由 tools/gen_tx_fir.ps1 预生成）。
///
/// 用途：paced TX 的插值——符号速率样本经 L 倍插值多相 FIR 到 DAC
/// 速率。L=1 直通（无表）。Q15 系数、8 抽头。
pub mod tx {
    /// FIR 抽头数（zephyr FIR_TAPS）。
    pub const FIR_TAPS: usize = 8;
    /// 最大插值因子（zephyr FIR_MAX_L）。
    pub const FIR_MAX_INTERP: usize = 8;

    /// L=2 多相系数（2 相位 × 8 抽头，Q15）。
    pub const COEF_2: [[i16; FIR_TAPS]; 2] = include!("tx_fir_2.rs");
    /// L=4 多相系数。
    pub const COEF_4: [[i16; FIR_TAPS]; 4] = include!("tx_fir_4.rs");
    /// L=8 多相系数。
    pub const COEF_8: [[i16; FIR_TAPS]; 8] = include!("tx_fir_8.rs");

    /// 取插值因子对应的相位表（L=1 无表，返回空）。
    #[must_use]
    pub fn coefficients(interp: usize) -> &'static [[i16; FIR_TAPS]] {
        match interp {
            2 => &COEF_2,
            4 => &COEF_4,
            8 => &COEF_8,
            _ => &[],
        }
    }

    /// 平移历史环：样本入队（最新在前，镜像存两份使窗口连续——
    /// zephyr hi[2*FIR_TAPS] 同款）。L=1 直通不需要历史。
    #[derive(Debug, Clone)]
    pub struct Shaper {
        history_i: [i16; 2 * FIR_TAPS],
        history_q: [i16; 2 * FIR_TAPS],
        /// 下一符号写入位置（FIR_TAPS-1 递减到 0，环形）。
        position: usize,
        /// 当前插值相位。
        phase: usize,
        /// 插值因子（1/2/4/8）。
        interp: usize,
        coefficients: &'static [[i16; FIR_TAPS]],
    }

    impl Shaper {
        #[must_use]
        pub fn new(interp: usize) -> Self {
            assert!(
                interp == 1 || interp == 2 || interp == 4 || interp == 8,
                "interp must be 1/2/4/8"
            );
            Self {
                history_i: [0; 2 * FIR_TAPS],
                history_q: [0; 2 * FIR_TAPS],
                position: 0,
                phase: 0,
                interp,
                coefficients: coefficients(interp),
            }
        }

        /// 检查是否该接收新符号（相位 0；L=1 每个输出样本都需要新符号）。
        #[must_use]
        pub fn symbol_due(&self) -> bool {
            self.interp == 1 || self.phase == 0
        }

        /// 检查是否该接收新符号（相位 0），若需要则把符号压入历史，
        /// 返回 true。`scale` 为输入到 DAC 量程的移位（zephyr >>6：
        /// 16 位全幅 → 10 位）。
        pub fn push_if_due(&mut self, i: i16, q: i16, scale: u32) -> bool {
            if self.interp > 1 && self.phase != 0 {
                return false;
            }
            let shift = if self.interp == 1 { 0 } else { scale };
            let i = if shift >= 16 { 0 } else { i >> shift };
            let q = if shift >= 16 { 0 } else { q >> shift };
            self.position = if self.position == 0 {
                FIR_TAPS - 1
            } else {
                self.position - 1
            };
            self.history_i[self.position] = i;
            self.history_i[self.position + FIR_TAPS] = i;
            self.history_q[self.position] = q;
            self.history_q[self.position + FIR_TAPS] = q;
            true
        }

        /// 生成下一个 DAC 样本（10 位有符号 I/Q，±511 语义）并推进相位。
        /// L=1 时直通当前符号。
        pub fn next(&mut self) -> (i32, i32) {
            if self.interp == 1 {
                return (
                    i32::from(self.history_i[self.position]),
                    i32::from(self.history_q[self.position]),
                );
            }
            let coefficients = &self.coefficients[self.phase];
            let mut accumulator_i: i32 = 0;
            let mut accumulator_q: i32 = 0;
            for tap in 0..FIR_TAPS {
                let coefficient = i32::from(coefficients[tap]);
                accumulator_i += coefficient * i32::from(self.history_i[self.position + tap]);
                accumulator_q += coefficient * i32::from(self.history_q[self.position + tap]);
            }
            self.phase = (self.phase + 1) & (self.interp - 1);
            (accumulator_i >> 15, accumulator_q >> 15)
        }
    }

    #[cfg(test)]
    mod tests {
        use super::Shaper;

        #[test]
        fn dc_gain_is_unity() {
            // 常数输入经黑曼-sinc 低通后应收敛到同值（增益 1，Q15）
            for interp in [2usize, 4, 8] {
                let mut shaper = Shaper::new(interp);
                let mut last = (0i32, 0i32);
                for _ in 0..64 {
                    shaper.push_if_due(256, -256, 0);
                    for _ in 0..interp {
                        last = shaper.next();
                    }
                }
                assert!((last.0 - 256).abs() <= 8, "interp {} I drift: {}", interp, last.0);
                assert!((last.1 + 256).abs() <= 8, "interp {} Q drift: {}", interp, last.1);
            }
        }

        #[test]
        fn impulse_is_centered_and_bounded() {
            let mut shaper = Shaper::new(4);
            // 单个脉冲后跟零
            shaper.push_if_due(400, 0, 0);
            let mut peak = 0i32;
            for cycle in 0..8 {
                for _ in 0..4 {
                    let (i, _) = shaper.next();
                    peak = peak.max(i.abs());
                }
                if cycle > 0 {
                    shaper.push_if_due(0, 0, 0);
                }
            }
            // 插值脉冲峰值不应超过输入（低通），且明显非零
            assert!(peak > 0 && peak <= 400, "impulse peak {}", peak);
        }

        #[test]
        fn interp_one_passthrough() {
            let mut shaper = Shaper::new(1);
            shaper.push_if_due(123, -45, 6);
            assert_eq!(shaper.next(), (123, -45));
        }
    }
}

/// TX 回放单音发生器（定点查表，主机可测）。
pub mod tone {
    /// 正弦表 256 点，Q15（-32768..=32767）。
    /// 由 tools 一次性生成（sin(2πk/256)×32767 四舍五入；主机测试
    /// 验证端点与正交性）。
    const SIN_Q15: [i16; 256] = {
        include!("sin_q15_table.rs")
    };

    /// 在缓冲中填入整周期单音：`cycles` 个周期铺满 `buffer.len()` 字。
    /// 每字 I/Q 交错布局与 dump 采集相同（I bits9:0、Q bits19:10，
    /// 10 位有符号 ±511）——TX 与 RX 的采样字格式对称（zephyr
    /// esp_sdr_replay_word 同款）。
    ///
    /// `cycles` 与 buffer.len() 互质与否都无妨：整周期保证首尾连续；
    /// 相位以 Q32 累加器推进（无累积误差）。
    pub fn fill_tone(buffer: &mut [u32], cycles: u32, amplitude: i32) {
        assert!(!buffer.is_empty(), "tone buffer empty");
        let amplitude = amplitude.clamp(1, 511);
        // 每字相位增量 = cycles × 2^32 / len（整周期）
        let step = ((u64::from(cycles) << 32) / buffer.len() as u64) as u32;
        let mut phase: u32 = 0;
        for word in buffer.iter_mut() {
            let index = (phase >> 24) as usize; // 高 8 位 → 256 点表
            let i_table = SIN_Q15[index] as i32;
            let q_table = SIN_Q15[(index + 64) & 0xFF] as i32; // cos = sin + 90°
            let i = (i_table * amplitude) >> 15;
            let q = (q_table * amplitude) >> 15;
            *word = ((i as u32) & 0x3FF) | (((q as u32) & 0x3FF) << 10);
            phase = phase.wrapping_add(step);
        }
    }

    #[cfg(test)]
    mod tests {
        use super::fill_tone;

        #[test]
        fn tone_words_pack_and_wrap() {
            let mut buffer = [0u32; 64];
            fill_tone(&mut buffer, 1, 511);
            // 所有 I/Q 字段必须在 10 位有符号范围内且非全零
            let mut any_nonzero = false;
            for word in buffer {
                let i = (word & 0x3FF) as i16;
                let q = ((word >> 10) & 0x3FF) as i16;
                // 10 位补码在 i16 中是 0..1023；转有符号
                let i_signed = if i & 0x200 != 0 { i - 1024 } else { i };
                let q_signed = if q & 0x200 != 0 { q - 1024 } else { q };
                assert!((-511..=511).contains(&i_signed));
                assert!((-511..=511).contains(&q_signed));
                if i_signed != 0 || q_signed != 0 {
                    any_nonzero = true;
                }
            }
            assert!(any_nonzero, "tone must not be all zero");
        }

        #[test]
        fn tone_first_and_quadrature() {
            // 第 0 字：I = sin(0) = 0，Q = cos(0) = 满幅
            let mut buffer = [0u32; 256];
            fill_tone(&mut buffer, 1, 500);
            let i0 = (buffer[0] & 0x3FF) as i16;
            let q0 = ((buffer[0] >> 10) & 0x3FF) as i16;
            let i0s = if i0 & 0x200 != 0 { i0 - 1024 } else { i0 };
            let q0s = if q0 & 0x200 != 0 { q0 - 1024 } else { q0 };
            assert_eq!(i0s, 0);
            assert!(q0s.abs() >= 495, "cos(0) should be near full scale, got {}", q0s);
        }
    }
}

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

/// C2 帧协议：突发采集帧的编解码与 CRC-32。
///
/// 字节序显式为小端；CRC 为反射 CRC-32（多项式 0xEDB88320，初值/终值 0xFFFFFFFF，
/// 与 zlib 一致）。CRC 覆盖帧头前 20 字节 + 载荷全部字节。
pub mod frame {
    pub const MAGIC: [u8; 4] = *b"C5IQ";
    pub const CAPTURE_FRAME_TYPE: u8 = 1;
    /// C4 无缝流帧：载荷为 IQ8 字节流（I,Q 交错，每字节独立削减位数）。
    /// sample_word_count 语义为帧内 IQ 样本对数；rate_index 对流帧无效
    /// （固定 0），实际速率由发送端配置（box16 + CIC 抽取比）决定。
    pub const STREAM_FRAME_TYPE: u8 = 2;
    /// 帧头定长：4 magic + 1 type + 1 rate + 2 sequence + 2 word_count + 2 reserved
    ///          + 4 frequency_mhz + 4 elapsed_us + 4 crc32 = 24。
    pub const HEADER_LEN: usize = 24;
    /// CRC 覆盖范围：帧头除 crc32 字段之外的部分。
    pub const CRC_COVERED_HEADER_LEN: usize = 20;

    /// 采集帧头（24 字节，小端编码，无填充）。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct CaptureFrameHeader {
        pub frame_type: u8,
        /// dump engine 速率档索引（0=80 MS/s ... 5=4 MS/s）。
        pub rate_index: u8,
        pub sequence: u16,
        /// u32 采样字个数（每字含一对 IQ）。
        pub sample_word_count: u16,
        pub frequency_mhz: u32,
        /// 采集耗时（微秒）。
        pub elapsed_us: u32,
        pub crc32: u32,
    }

    /// 帧解码失败的显式故障模型。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum FrameError {
        BadMagic,
        BadFrameType,
        CrcMismatch { expected: u32, actual: u32 },
    }

    impl CaptureFrameHeader {
        /// 把帧头编码为 24 字节小端字节串。
        #[must_use]
        pub fn encode(&self) -> [u8; HEADER_LEN] {
            let mut bytes = [0u8; HEADER_LEN];
            bytes[0..4].copy_from_slice(&MAGIC);
            bytes[4] = self.frame_type;
            bytes[5] = self.rate_index;
            bytes[6..8].copy_from_slice(&self.sequence.to_le_bytes());
            bytes[8..10].copy_from_slice(&self.sample_word_count.to_le_bytes());
            bytes[10..12].copy_from_slice(&0u16.to_le_bytes()); // reserved
            bytes[12..16].copy_from_slice(&self.frequency_mhz.to_le_bytes());
            bytes[16..20].copy_from_slice(&self.elapsed_us.to_le_bytes());
            bytes[20..24].copy_from_slice(&self.crc32.to_le_bytes());
            bytes
        }

        /// 从 24 字节小端字节串解码帧头（不校验 CRC，见 verify_crc）。
        pub fn decode(bytes: &[u8; HEADER_LEN]) -> Result<Self, FrameError> {
            if bytes[0..4] != MAGIC {
                return Err(FrameError::BadMagic);
            }
            let frame_type = bytes[4];
            if frame_type != CAPTURE_FRAME_TYPE && frame_type != STREAM_FRAME_TYPE {
                return Err(FrameError::BadFrameType);
            }
            Ok(Self {
                frame_type,
                rate_index: bytes[5],
                sequence: u16::from_le_bytes([bytes[6], bytes[7]]),
                sample_word_count: u16::from_le_bytes([bytes[8], bytes[9]]),
                frequency_mhz: u32::from_le_bytes([
                    bytes[12], bytes[13], bytes[14], bytes[15],
                ]),
                elapsed_us: u32::from_le_bytes([
                    bytes[16], bytes[17], bytes[18], bytes[19],
                ]),
                crc32: u32::from_le_bytes([
                    bytes[20], bytes[21], bytes[22], bytes[23],
                ]),
            })
        }

        /// 校验帧头 + 载荷的 CRC。
        #[must_use]
        pub fn verify_crc(&self, header_bytes: &[u8; HEADER_LEN], payload: &[u8]) -> bool {
            let mut context = Crc32::new();
            context.update(&header_bytes[0..CRC_COVERED_HEADER_LEN]);
            context.update(payload);
            context.finalize() == self.crc32
        }
    }

    const fn make_crc_table() -> [u32; 256] {
        let mut table = [0u32; 256];
        let mut index = 0usize;
        while index < 256 {
            let mut value = index as u32;
            let mut bit = 0;
            while bit < 8 {
                if value & 1 != 0 {
                    value = (value >> 1) ^ 0xEDB8_8320;
                } else {
                    value >>= 1;
                }
                bit += 1;
            }
            table[index] = value;
            index += 1;
        }
        table
    }

    static CRC_TABLE: [u32; 256] = make_crc_table();

    /// 增量式 CRC-32（编译期表驱动，无堆、无宏）。
    pub struct Crc32 {
        value: u32,
    }

    impl Crc32 {
        #[must_use]
        pub const fn new() -> Self {
            Self { value: 0xFFFF_FFFF }
        }

        pub fn update(&mut self, data: &[u8]) {
            let mut value = self.value;
            for &byte in data {
                let table_index = ((value ^ byte as u32) & 0xFF) as usize;
                value = (value >> 8) ^ CRC_TABLE[table_index];
            }
            self.value = value;
        }

        #[must_use]
        pub const fn finalize(&self) -> u32 {
            self.value ^ 0xFFFF_FFFF
        }
    }

    /// 一次性计算 CRC-32。
    #[must_use]
    pub fn crc32(data: &[u8]) -> u32 {
        let mut context = Crc32::new();
        context.update(data);
        context.finalize()
    }
}

/// 采样字位域（dump engine 捕获区格式）。
pub mod iq {
    /// 解码采样字：I bits 9:0 / Q bits 19:10（10 位有符号，符号扩展到 i16）。
    /// 增益字段 bits 27:20 由调用方按需处理。
    #[must_use]
    pub const fn unpack_word(word: u32) -> (i16, i16) {
        let i_raw = (word & 0x3FF) as u16;
        let q_raw = ((word >> 10) & 0x3FF) as u16;
        (sign_extend_10(i_raw), sign_extend_10(q_raw))
    }

    const fn sign_extend_10(value: u16) -> i16 {
        // 10 位有符号：bit 9 为符号位
        if value & 0x200 != 0 {
            (value | 0xFC00) as i16
        } else {
            value as i16
        }
    }

    #[cfg(test)]
    mod tests {
        use super::unpack_word;

        #[test]
        fn unpack_sign_extends() {
            // I = -1 (10位: 0x3FF), Q = +1
            let word = 0x3FF | (1 << 10);
            assert_eq!(unpack_word(word), (-1, 1));
            // I = -512 (0x200), Q = +511 (0x1FF)
            let word = 0x200 | (0x1FF << 10);
            assert_eq!(unpack_word(word), (-512, 511));
            // I = 0, Q = 0
            assert_eq!(unpack_word(0), (0, 0));
            // 增益字段 bits 27:20 不影响解包
            let word = 0x3FF | (0x3FF << 10) | (0xAB << 20);
            assert_eq!(unpack_word(word), (-1, -1));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{frame, frequency, gain};

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

    #[test]
    fn crc32_matches_standard_check_value() {
        // CRC-32 标准校验值："123456789" -> 0xCBF43926
        assert_eq!(frame::crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(frame::crc32(b""), 0x0000_0000);
    }

    #[test]
    fn frame_header_roundtrip() {
        let header = frame::CaptureFrameHeader {
            frame_type: frame::CAPTURE_FRAME_TYPE,
            rate_index: 0,
            sequence: 7,
            sample_word_count: 16380,
            frequency_mhz: 2412,
            elapsed_us: 205,
            crc32: frame::crc32(b"payload"),
        };
        let bytes = header.encode();
        assert_eq!(bytes.len(), frame::HEADER_LEN);
        assert_eq!(bytes[0..4], frame::MAGIC);
        let decoded = frame::CaptureFrameHeader::decode(&bytes).unwrap();
        assert_eq!(decoded, header);
    }

    #[test]
    fn frame_crc_verification() {
        let payload: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut header = frame::CaptureFrameHeader {
            frame_type: frame::CAPTURE_FRAME_TYPE,
            rate_index: 2,
            sequence: 1,
            sample_word_count: 2,
            frequency_mhz: 5500,
            elapsed_us: 822,
            crc32: 0,
        };
        let mut bytes = header.encode();
        // 正确 CRC：覆盖帧头前 20 字节 + 载荷
        let mut context = frame::Crc32::new();
        context.update(&bytes[0..frame::CRC_COVERED_HEADER_LEN]);
        context.update(&payload);
        header.crc32 = context.finalize();
        bytes = header.encode();
        assert!(header.verify_crc(&bytes, &payload));

        // 篡改载荷 -> CRC 失败
        let tampered_payload: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 9];
        assert!(!header.verify_crc(&bytes, &tampered_payload));

        // 坏魔数 -> BadMagic
        let mut bad_bytes = bytes;
        bad_bytes[0] = b'X';
        assert_eq!(
            frame::CaptureFrameHeader::decode(&bad_bytes),
            Err(frame::FrameError::BadMagic)
        );
    }
}
