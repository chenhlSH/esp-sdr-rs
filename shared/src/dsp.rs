//! 诊断总线环的信号处理（C4）：box16 + 三阶 CIC + 归一化。
//!
//! 纯 32 位整数运算、无 unsafe、无堆，主机端与固件端同一份代码。
//! 算法逐行对照 esp-sdr-zephyr lib/esp_sdr/src/esp_sdr_diag_dsp.h（GPL-3.0，
//! 只读参考；公式本身为公开的 CIC 结构）。
//!
//! 数据流：环字节 = Q 低 4 位 | I 高 4 位（有符号 nibble × 2^lsb）@ 40 MB/s。
//! box16 每 16 字节求出一对和（2.5 MS/s）；CIC 再按 R 抽取；
//! 归一化把 CIC 输出缩回"10 bit 原始计数 × 16"量纲。

/// box16 每输出消耗的环字节数。
pub const DIAG_BOX: usize = 16;
/// CIC 抽取率范围（R^3 须保持 32 位倒数在范围内，zephyr DIAG_CIC_R_*）。
pub const CIC_R_MIN: u32 = 8;
pub const CIC_R_MAX: u32 = 255;

/// 把 16 个环字节（4 个 u32）的两个 nibble 半边分别求和。
/// 有符号 nibble 经 XOR 0x88 偏移为 0..15，横向求和后减去 8*16=128。
#[must_use]
pub fn box16(words: &[u32; 4]) -> (i32, i32) {
    const MASK: u32 = 0x0F0F_0F0F;
    const XOR: u32 = 0x8888_8888;
    let a = words[0] ^ XOR;
    let b = words[1] ^ XOR;
    let c = words[2] ^ XOR;
    let d = words[3] ^ XOR;
    let lo = (a & MASK) + (b & MASK) + (c & MASK) + (d & MASK);
    let hi = ((a >> 4) & MASK) + ((b >> 4) & MASK) + ((c >> 4) & MASK) + ((d >> 4) & MASK);
    // 每字节 lane 至多 60，× 0x01010101 横向累加取最高字节（至多 240）
    let q = ((lo.wrapping_mul(0x0101_0101)) >> 24) as i32 - 128;
    let i = ((hi.wrapping_mul(0x0101_0101)) >> 24) as i32 - 128;
    (i, q)
}

/// 三阶 CIC 抽取器状态（积分器按输入速率回绕 32 位是有意设计：
/// 只有差值有意义）。
#[derive(Debug, Clone)]
pub struct Cic {
    i1: u32, i2: u32, i3: u32,
    q1: u32, q2: u32, q3: u32,
    di1: u32, di2: u32, di3: u32,
    dq1: u32, dq2: u32, dq3: u32,
    rate: u32,
    phase: u32,
    /// 滤波器充填期仍需丢弃的输出个数。
    settle: u32,
}

impl Cic {
    #[must_use]
    pub fn new(rate: u32) -> Self {
        Self {
            i1: 0, i2: 0, i3: 0,
            q1: 0, q2: 0, q3: 0,
            di1: 0, di2: 0, di3: 0,
            dq1: 0, dq2: 0, dq3: 0,
            rate,
            phase: 0,
            settle: 3,
        }
    }

    /// 喂入 nbytes（DIAG_BOX 的倍数、4 字节对齐）环字节；
    /// 把 CIC 输出以 I、Q 交错追加到 out（未归一化），返回输出对数。
    /// 初始化后的前三个输出被丢弃（settling）。
    #[must_use]
    pub fn run(&mut self, ring_bytes: &[u8], out: &mut [i32]) -> usize {
        debug_assert!(ring_bytes.len() % DIAG_BOX == 0);
        let mut produced = 0usize;
        for chunk16 in ring_bytes.chunks_exact(DIAG_BOX) {
            let mut words = [0u32; 4];
            for (index, word) in words.iter_mut().enumerate() {
                let offset = index * 4;
                *word = u32::from_le_bytes([
                    chunk16[offset],
                    chunk16[offset + 1],
                    chunk16[offset + 2],
                    chunk16[offset + 3],
                ]);
            }
            let (si, sq) = box16(&words);
            self.i1 = self.i1.wrapping_add(si as u32);
            self.i2 = self.i2.wrapping_add(self.i1);
            self.i3 = self.i3.wrapping_add(self.i2);
            self.q1 = self.q1.wrapping_add(sq as u32);
            self.q2 = self.q2.wrapping_add(self.q1);
            self.q3 = self.q3.wrapping_add(self.q2);
            self.phase += 1;
            if self.phase == self.rate {
                self.phase = 0;
                // 三级梳状：x 减去自身上一次的值，重复三次
                let mut ti = self.i3.wrapping_sub(self.di1);
                self.di1 = self.i3;
                let mut tq = self.q3.wrapping_sub(self.dq1);
                self.dq1 = self.q3;
                let ui = ti.wrapping_sub(self.di2);
                let uq = tq.wrapping_sub(self.dq2);
                self.di2 = ti;
                self.dq2 = tq;
                ti = ui.wrapping_sub(self.di3);
                tq = uq.wrapping_sub(self.dq3);
                self.di3 = ui;
                self.dq3 = uq;
                if self.settle != 0 {
                    self.settle -= 1;
                } else {
                    out[2 * produced] = ti as i32;
                    out[2 * produced + 1] = tq as i32;
                    produced += 1;
                }
            }
        }
        produced
    }
}

/// 归一化倒数：2^(32+lsb) / R^3，四舍五入。每个流计算一次。
#[must_use]
pub fn cic_inverse(rate: u32, lsb: u8) -> u32 {
    let rate_cubed = u64::from(rate) * u64::from(rate) * u64::from(rate);
    (((1u64 << (32 + u32::from(lsb))) + rate_cubed / 2) / rate_cubed) as u32
}

/// 一个 CIC 输出归一到"10 bit 原始计数 × 16"量纲：acc × 2^lsb / R^3。
#[must_use]
pub fn normalize(acc: i32, inverse: u32) -> i32 {
    ((i64::from(acc) * i64::from(inverse)) >> 32) as i32
}

/// diag 窗口位置上限（lsb 0..=6；zephyr ESP_SDR_DIAG_LSB_MAX）。
pub const DIAG_LSB_MAX: u8 = 6;

/// 从 10 bit I/Q 幅度直方图选 diag 窗口 lsb（espsoup pc_level 公式）。
///
/// hist 为 |x| 的计数（bin 0..=1023，I/Q 两路合计），total 为总样本数。
/// peak 取 99.9% 点；robust 模式用中值 × 2.2 替代（FM 恒包络下突发
/// （2.4 GHz Wi-Fi）会把 99.9% 点顶到 lsb 6——espsoup 实测 137 次失锁
/// 的教训）。lsb 从 2 起：`7.5 * 2^lsb >= 1.15 * peak`，即 4 位码
/// 不回绕也不浪费量程。
#[must_use]
pub fn diag_lsb_from_histogram(hist: &[u32; 1024], total: u32, robust: bool) -> u8 {
    debug_assert!(total > 0);
    let mut acc: u32 = 0;
    let mut median = 1024u32;
    let mut peak = 1024u32;
    let have_median = total / 2;
    let have_peak = total - total / 1000;
    for (value, &count) in hist.iter().enumerate() {
        acc = acc.saturating_add(count);
        if median == 1024 && acc >= have_median {
            median = value as u32;
        }
        if acc >= have_peak {
            peak = value as u32;
            break;
        }
    }
    if robust {
        // 中值 × 2.2（含噪声裕量），不超过实测 peak
        let from_median = (median * 22 / 10).min(peak);
        if from_median > 0 {
            peak = from_median;
        }
    }
    let mut lsb: u8 = 2;
    while lsb < DIAG_LSB_MAX && (15u32 << lsb) < peak.saturating_mul(23) / 10 {
        lsb += 1;
    }
    lsb
}

#[cfg(test)]
mod tests {
    use super::{
        box16, cic_inverse, diag_lsb_from_histogram, normalize, Cic, CIC_R_MAX, CIC_R_MIN,
        DIAG_BOX,
    };

    #[test]
    fn box16_zero_and_extremes() {
        // 全零字节：nibble 全 0 → 和 = 0
        assert_eq!(box16(&[0, 0, 0, 0]), (0, 0));
        // 全 0x8888_8888（nibble 全 -8，即 0x8）→ 两个半边都是 16×(-8) = -128
        assert_eq!(box16(&[0x8888_8888; 4]), (-128, -128));
        // 全字节 0x80：I nibble 全 -8 → si = -128；Q nibble 全 0 → sq = 0
        assert_eq!(box16(&[0x8080_8080; 4]), (-128, 0));
        // 全字节 0x08：Q nibble 全 -8 → sq = -128
        assert_eq!(box16(&[0x0808_0808; 4]), (0, -128));
        // 单个 I nibble = +7、其余 nibble 全 0（字节 0x70）→ si = +7，sq = 0
        let (i, q) = box16(&[0x7000_0000, 0, 0, 0]);
        assert_eq!(i, 7);
        assert_eq!(q, 0);
    }

    #[test]
    fn box16_matches_naive() {
        // 与朴素实现逐字节对照
        let mut naive_i = 0i32;
        let mut naive_q = 0i32;
        for byte in [0x12u8, 0xFE, 0x70, 0x8A, 0x55, 0x00, 0xFF, 0x37,
                     0x41, 0xC2, 0x99, 0x0D, 0xE6, 0x78, 0x23, 0xB1] {
            let i = ((byte >> 4) as i8) << 4 >> 4; // 符号扩展 nibble
            let q = (byte as i8) << 4 >> 4;
            naive_i += i as i32;
            naive_q += q as i32;
        }
        let words: [u32; 4] = [
            u32::from_le_bytes([0x12, 0xFE, 0x70, 0x8A]),
            u32::from_le_bytes([0x55, 0x00, 0xFF, 0x37]),
            u32::from_le_bytes([0x41, 0xC2, 0x99, 0x0D]),
            u32::from_le_bytes([0xE6, 0x78, 0x23, 0xB1]),
        ];
        assert_eq!(box16(&words), (naive_i, naive_q));
    }

    #[test]
    fn cic_dc_gain_and_settle() {
        // 恒定输入的 CIC 稳态输出 = R³ × 每输出窗输入和（传递函数
        // (1-z^-R)³/(1-z^-1)³ 在 DC 处增益 R³）。
        // 输入 0x18：I nibble +1 → si=16/box；R=10 → 稳态 I 输出 = 1000×16 = 16000。
        let rate = 10;
        let mut cic = Cic::new(rate);
        let mut out = [0i32; 256];
        let chunk = [0x18u8; DIAG_BOX];
        let mut produced_total = 0;
        for _ in 0..400 {
            let n = cic.run(&chunk, &mut out);
            produced_total += n;
        }
        assert!(produced_total >= 37, "应有 ≥37 个输出（扣 3 个 settle），得 {produced_total}");
        // 稳态检查：最后一批输出 I ≈ 16000
        let n = cic.run(&[0x18u8; DIAG_BOX * 10], &mut out);
        assert!(n >= 1, "应至少产出一个输出");
        for pair in out[..2 * n].chunks_exact(2) {
            assert!(
                (pair[0] - 16000).abs() <= 2,
                "直流稳态输出应≈16000，得 {}",
                pair[0]
            );
        }
    }

    #[test]
    fn cic_tone_gain() {
        // 幅度 ±7 nibble、周期 8000 字节的正弦（out 频率 f=0.02，避开 CIC 的
        // k/R 零点；R=10 响应≈0.82），归一化后幅度应回到"10bit×16"量纲（≈90）。
        let rate = 10;
        let inverse = cic_inverse(rate, 0);
        let mut cic = Cic::new(rate);
        let mut out = [0i32; 512];
        let mut phase = 0u32;
        let mut max_i = 0i32;
        // 三个完整周期（24000 字节 = 1500 box = 150 输出）足够越过 settle 并见峰
        for _ in 0..1500 {
            let mut bytes = [0u8; DIAG_BOX];
            for byte in bytes.iter_mut() {
                phase = phase.wrapping_add(1);
                let angle = f64::from(phase % 8000) / 8000.0 * core::f64::consts::TAU;
                let i_nibble = ((7.0 * angle.sin()) as i64) as u8 & 0x0F;
                let q_nibble = ((7.0 * angle.cos()) as i64) as u8 & 0x0F;
                *byte = (i_nibble << 4) | q_nibble;
            }
            let n = cic.run(&bytes, &mut out);
            for pair in out[..2 * n].chunks_exact(2) {
                let scaled = normalize(pair[0], inverse);
                max_i = max_i.max(scaled.abs());
            }
        }
        assert!(max_i > 60 && max_i < 200, "音调归一化幅度异常：{max_i}");
    }

    #[test]
    fn inverse_math() {
        // R=8, lsb=0：2^32 / 512 = 8388608
        assert_eq!(cic_inverse(8, 0), 8_388_608);
        // 范围约束成立
        assert!(CIC_R_MIN <= 10 && 10 <= CIC_R_MAX);
    }

    #[test]
    fn lsb_selection() {
        // 小信号（|x| 峰 ~20）：peak*1.15=23，7.5*2^2=30 ≥ 23 → lsb 2
        let mut hist = [0u32; 1024];
        for value in 0..20 {
            hist[value] = 10;
        }
        hist[19] += 10; // 99.9% 点在 19-20 之间
        let total: u32 = hist.iter().sum();
        assert_eq!(diag_lsb_from_histogram(&hist, total, false), 2);

        // 满量程信号（|x| 峰 ~500）：需要 lsb 6（7.5*2^6=480）
        let mut big = [0u32; 1024];
        for value in 0..500 {
            big[value] = 10;
        }
        let total_big: u32 = big.iter().sum();
        assert_eq!(diag_lsb_from_histogram(&big, total_big, false), 6);

        // robust：主体中值 ~20 + 1% 突发到 1000（非 robust 的 99.9% 点
        // 被突发顶到 1000 → lsb 6；robust 用中值 20×2.2=44 →
        // 44*1.15≈101 → lsb 3（15<<3=120 ≥ 101））
        let mut bursty = [0u32; 1024];
        for value in 0..41 {
            bursty[value] = 999;
        }
        bursty[1000] = 1000; // ≈1% 突发
        let total_bursty: u32 = bursty.iter().sum();
        assert_eq!(diag_lsb_from_histogram(&bursty, total_bursty, false), 6);
        assert_eq!(diag_lsb_from_histogram(&bursty, total_bursty, true), 3);
    }
}
