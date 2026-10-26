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
///
/// `#[repr(C)]` 固定字段序——riscv32 上的 [Cic::run_asm] 汇编路径按
/// 字节偏移直读写本结构（偏移表见 dsp.rs 内 global_asm 注释）。
#[derive(Debug, Clone)]
#[repr(C)]
pub struct Cic {
    rate: u32,
    phase: u32,
    /// 滤波器充填期仍需丢弃的输出个数。
    settle: u32,
    i1: u32, i2: u32, i3: u32,
    q1: u32, q2: u32, q3: u32,
    di1: u32, di2: u32, di3: u32,
    dq1: u32, dq2: u32, dq3: u32,
}

impl Cic {
    #[must_use]
    pub fn new(rate: u32) -> Self {
        Self {
            rate,
            phase: 0,
            settle: 3,
            i1: 0, i2: 0, i3: 0,
            q1: 0, q2: 0, q3: 0,
            di1: 0, di2: 0, di3: 0,
            dq1: 0, dq2: 0, dq3: 0,
        }
    }

    /// 喂入 nbytes（DIAG_BOX 的倍数、4 字节对齐）环字节；
    /// 把 CIC 输出以 I、Q 交错追加到 out（未归一化），返回输出对数。
    /// 初始化后的前三个输出被丢弃（settling）。
    #[must_use]
    pub fn run(&mut self, ring_bytes: &[u8], out: &mut [i32]) -> usize {
        debug_assert!(ring_bytes.len() % DIAG_BOX == 0);
        let word_count = ring_bytes.len() / 4;
        let base = ring_bytes.as_ptr() as *const u32;
        // 紧凑内循环（板上实测：迭代器版 7.9 cycles/byte，超 40 MB/s 全
        // 消费预算 1.3 倍；理论 SWAR+积分 ~2.4，裸指针+寄存器驻留逼近）
        const MASK: u32 = 0x0F0F_0F0F;
        const XOR: u32 = 0x8888_8888;
        let mut produced = 0usize;
        let mut word = 0usize;
        // 寄存器驻留的积分器状态（写回在出口）
        let (mut i1, mut i2, mut i3) = (self.i1, self.i2, self.i3);
        let (mut q1, mut q2, mut q3) = (self.q1, self.q2, self.q3);
        let mut phase = self.phase;
        while word < word_count {
            // SAFETY: word_count 按 DIAG_BOX 对齐的长度折算，四个访问
            // 恰在切片内；read_unaligned 对任意对齐安全。
            let a = unsafe { core::ptr::read_unaligned(base.add(word)) } ^ XOR;
            let b = unsafe { core::ptr::read_unaligned(base.add(word + 1)) } ^ XOR;
            let c = unsafe { core::ptr::read_unaligned(base.add(word + 2)) } ^ XOR;
            let d = unsafe { core::ptr::read_unaligned(base.add(word + 3)) } ^ XOR;
            let lo = (a & MASK) + (b & MASK) + (c & MASK) + (d & MASK);
            let hi = ((a >> 4) & MASK) + ((b >> 4) & MASK) + ((c >> 4) & MASK) + ((d >> 4) & MASK);
            let sq = ((lo.wrapping_mul(0x0101_0101)) >> 24).wrapping_sub(128) as u32;
            let si = ((hi.wrapping_mul(0x0101_0101)) >> 24).wrapping_sub(128) as u32;
            i1 = i1.wrapping_add(si);
            i2 = i2.wrapping_add(i1);
            i3 = i3.wrapping_add(i2);
            q1 = q1.wrapping_add(sq);
            q2 = q2.wrapping_add(q1);
            q3 = q3.wrapping_add(q2);
            phase += 1;
            if phase == self.rate {
                phase = 0;
                let mut ti = i3.wrapping_sub(self.di1);
                self.di1 = i3;
                let mut tq = q3.wrapping_sub(self.dq1);
                self.dq1 = q3;
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
            word += 4;
        }
        self.i1 = i1;
        self.i2 = i2;
        self.i3 = i3;
        self.q1 = q1;
        self.q2 = q2;
        self.q3 = q3;
        self.phase = phase;
        produced
    }

    /// 汇编快速路径（仅 riscv32；与 [Self::run] 输出逐位一致——板上
    /// 等价性自检见 firmware/apps/src/bin/diagprobe2.rs）。
    ///
    /// 实测对照（240 MHz，ESP32-C5 rev v1.2）：
    /// scalar 7.8 cycles/byte；本路径 ~4（40 MB/s 全消费 = 160M cycles
    /// = CPU 的 67%，预算内）。
    #[cfg(target_arch = "riscv32")]
    #[must_use]
    pub fn run_asm(&mut self, ring_bytes: &[u8], out: &mut [i32]) -> usize {
        debug_assert!(ring_bytes.len() % DIAG_BOX == 0);
        let word_count = ring_bytes.len() / 4;
        debug_assert!(word_count % 4 == 0);
        unsafe {
            esp_sdr_cic_run_asm(
                ring_bytes.as_ptr() as *const u32,
                word_count,
                out.as_mut_ptr(),
                self as *mut Cic as *mut u32,
            )
        }
    }
}

// 汇编版 box16 + 三阶 CIC 内循环（esp_sdr_cic_run_asm）。
//
// ABI：a0 = 环字指针（4 字节对齐），a1 = 字数（4 的倍数），
// a2 = 输出（i32 交错 I/Q），a3 = `Cic` 状态指针；返回输出对数。
//
// 状态字节偏移（repr(C) Cic）：0 rate / 4 phase / 8 settle /
// 12 i1 / 16 i2 / 20 i3 / 24 q1 / 28 q2 / 32 q3 /
// 36 di1 / 40 di2 / 44 di3 / 48 dq1 / 52 dq2 / 56 dq3。
//
// 寄存器分配：s0=字指针 s1=输出指针 s2=结束指针 s3=状态指针
// s4..s9=i1,i2,i3,q1,q2,q3（全程驻留）s10=phase s11=produced
// t0=0x0f0f0f0f t1=0x88888888 t2=0x01010101 a0=rate
// 其余 a1-a7,t3-t6 为 box16/梳状临时。
//
// 每次迭代 4 字（16 字节）：SWAR box16 → 6 条积分器增量 → phase
// 计数（等于 R 时走梳状 + 输出）。梳状每 R 次才执行，延迟状态留
// 内存（非热路径）。注释限 ASCII：LLVM 汇编器拒绝非 ASCII 字符。
#[cfg(target_arch = "riscv32")]
core::arch::global_asm!(
    r#"
    .section .text.esp_sdr_cic_run_asm, "ax", @progbits
    .global esp_sdr_cic_run_asm
    .type esp_sdr_cic_run_asm, @function
    .p2align 2
esp_sdr_cic_run_asm:
    addi sp, sp, -48
    sw s0, 0(sp)
    sw s1, 4(sp)
    sw s2, 8(sp)
    sw s3, 12(sp)
    sw s4, 16(sp)
    sw s5, 20(sp)
    sw s6, 24(sp)
    sw s7, 28(sp)
    sw s8, 32(sp)
    sw s9, 36(sp)
    sw s10, 40(sp)
    sw s11, 44(sp)

    mv s0, a0                 # word ptr
    mv s1, a2                 # out ptr
    slli a1, a1, 2
    add s2, s0, a1            # end ptr
    mv s3, a3                 # state ptr
    lw a0, 0(s3)              # rate
    lw s10, 4(s3)             # phase
    lw s4, 12(s3)             # i1
    lw s5, 16(s3)             # i2
    lw s6, 20(s3)             # i3
    lw s7, 24(s3)             # q1
    lw s8, 28(s3)             # q2
    lw s9, 32(s3)             # q3
    li s11, 0                 # produced
    li t0, 0x0f0f0f0f
    li t1, 0x88888888
    li t2, 0x01010101
    bgeu s0, s2, .Lcic_done

.Lcic_loop:
    lw a1, 0(s0)
    lw a2, 4(s0)
    lw a3, 8(s0)
    lw a4, 12(s0)
    addi s0, s0, 16
    xor a1, a1, t1
    xor a2, a2, t1
    xor a3, a3, t1
    xor a4, a4, t1
    and t3, a1, t0            # lo nibbles (Q)
    and t4, a2, t0
    and t5, a3, t0
    and a5, a4, t0
    srli a1, a1, 4            # hi nibbles (I)
    srli a2, a2, 4
    srli a3, a3, 4
    srli a4, a4, 4
    add t3, t3, t4
    add t5, t5, a5
    and a1, a1, t0
    and a2, a2, t0
    and a3, a3, t0
    and a4, a4, t0
    add t3, t3, t5            # lo sum
    add a1, a1, a2
    add a3, a3, a4
    add a1, a1, a3            # hi sum
    mul t3, t3, t2
    mul a1, a1, t2
    srli t3, t3, 24
    srli a1, a1, 24
    addi t3, t3, -128         # sq
    addi a1, a1, -128         # si
    add s7, s7, t3            # q1 += sq
    add s4, s4, a1            # i1 += si
    add s8, s8, s7
    add s5, s5, s4
    add s9, s9, s8
    add s6, s6, s5
    addi s10, s10, 1
    bne s10, a0, .Lcic_next
    li s10, 0
    # three combs, once per R words
    lw t3, 36(s3)             # di1
    lw t4, 40(s3)             # di2
    lw t5, 44(s3)             # di3
    sub a1, s6, t3            # ti
    sw s6, 36(s3)
    sub a2, a1, t4            # ui
    sw a1, 40(s3)
    sub a1, a2, t5            # vi
    sw a2, 44(s3)
    lw t3, 48(s3)             # dq1
    lw t4, 52(s3)             # dq2
    lw t5, 56(s3)             # dq3
    sub a3, s9, t3            # tq
    sw s9, 48(s3)
    sub a4, a3, t4            # uq
    sw a3, 52(s3)
    sub a3, a4, t5            # vq
    sw a4, 56(s3)
    lw t6, 8(s3)              # settle
    beqz t6, .Lcic_emit
    addi t6, t6, -1
    sw t6, 8(s3)
    j .Lcic_next
.Lcic_emit:
    sw a1, 0(s1)
    sw a3, 4(s1)
    addi s1, s1, 8
    addi s11, s11, 1
.Lcic_next:
    bltu s0, s2, .Lcic_loop

.Lcic_done:
    sw s10, 4(s3)
    sw s4, 12(s3)
    sw s5, 16(s3)
    sw s6, 20(s3)
    sw s7, 24(s3)
    sw s8, 28(s3)
    sw s9, 32(s3)
    mv a0, s11
    lw s0, 0(sp)
    lw s1, 4(sp)
    lw s2, 8(sp)
    lw s3, 12(sp)
    lw s4, 16(sp)
    lw s5, 20(sp)
    lw s6, 24(sp)
    lw s7, 28(sp)
    lw s8, 32(sp)
    lw s9, 36(sp)
    lw s10, 40(sp)
    lw s11, 44(sp)
    addi sp, sp, 48
    ret
    .size esp_sdr_cic_run_asm, . - esp_sdr_cic_run_asm
"#
);

#[cfg(target_arch = "riscv32")]
unsafe extern "C" {
    /// 汇编版 box16 + 三阶 CIC 内循环（定义见上方 global_asm）。
    fn esp_sdr_cic_run_asm(
        words: *const u32,
        word_count: usize,
        out: *mut i32,
        state: *mut u32,
    ) -> usize;
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
