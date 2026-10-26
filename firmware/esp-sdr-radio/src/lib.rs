//! ESP32-C5 射频初始化与控制层。
//!
//! 前置契约：射频电源域与 PHY 校准必须已由 vendor WiFi 栈完成
//! （esp_radio::wifi::new 建立未启动的控制器即满足）。
//!
//! 职责：vendor blob 函数绑定（phy_set_chanfreq / force_rx_gain / RX 链路准备序列）、
//! 增益锁定维护。全部公开 API 以 Result 返回，显式错误枚举定义故障模型。

#![no_std]

use core::ptr::{read_volatile, write_volatile};

use esp_sdr_shared::{frequency, gain};

/// AGCPWR_CTRL7（C5）：增益状态寄存器（reference/esp-sdr common/burst_gain.h）。
const GAIN_STATUS_REGISTER: u32 = 0x600a_702c;

/// 调谐失败的显式故障模型。
#[derive(Debug, defmt::Format)]
pub enum TuneError {
    /// 请求频点超出软件限幅（C5 软件限幅 100..=6000 MHz）。
    FrequencyOutOfRange {
        requested_mhz: u32,
        minimum_mhz: u32,
        maximum_mhz: u32,
    },
}

/// 增益控制失败的显式故障模型。
#[derive(Debug, defmt::Format)]
pub enum GainError {
    /// 请求的增益码超出已校准索引表。
    CodeAboveMaximum {
        requested_code: u32,
        calibrated_maximum: u32,
    },
}

/// 射频控制失败。
#[derive(Debug, defmt::Format)]
pub enum RadioError {
    Tune(TuneError),
    Gain(GainError),
    /// TX 功率步超过 15（zephyr TX_POWER_MAX）。
    TxPower { requested: u32 },
}

/// TX 功率步上限（每步 2 dB 回退；zephyr TX_POWER_MAX）。
pub const TX_POWER_MAX: u32 = 15;
/// 每功率步的回退量（zephyr TX_BACKOFF_STEP）。
const TX_BACKOFF_STEP: u32 = 8;

/// 增益控制模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum GainMode {
    /// 锁定手动增益码（接收链不再自动调整）。
    Manual,
    /// 交给硬件 AGC（释放强制）。
    Hardware,
}

// vendor blob 符号：libphy.a（esp-wifi-sys-esp32c5 传递链接）与 librftest.a（vendor/）。
unsafe extern "C" {
    /// C5 直接收 MHz；mode 为信道带宽标志（0 = 20 MHz）。
    fn phy_set_chanfreq(mhz: u32, channel_bandwidth_mode: u32);
    fn phy_stop_tx_tone(arg: u32);
    fn phy_pbus_workmode();
    fn phy_pbus_xpd_tx_off();
    /// 打开接收链（半双工恢复时也调用此函数）。
    fn phy_pbus_xpd_rx_on(arg: u32);
    fn phy_set_rxclk_en(arg: u32);
    /// C5 签名三参数：manual、增益码、保留 0（librftest.a）。
    fn force_rx_gain(manual: u32, gain_code: u32, reserved: u32);
    /// 模拟前端信道设置（C5 直接收 MHz；libphy.a，nm 已验证）。
    fn phy_chip_set_chan_ana(mhz: u32);
    /// RX DC/增益表重校准（reference/esp-sdr common/rx_recalibration.c C5 分支）。
    fn phy_set_rx_gain_cal_dc(a: u32, b: u32, table_a: *mut u8, table_b: *mut u8);
    fn phy_adc_rate_cal_rxdc();
    fn phy_set_rx_gain_table(mhz: u32, arg: u32);
    /// PHY 参数表（libphy.a 全局，尺寸未知，按指针访问）。
    static mut phy_param: u8;
    // PBUS 调试模式（DC 校准用；libphy.a，nm 已验证；声明照
    // espsoup dc_cal.h——其固件在 C5 真机验证过该校准有效）。
    /// 进入 PBUS debug 模式（DC DAC 可由软件强制；退出用 phy_pbus_workmode）。
    fn phy_pbus_debugmode();
    /// 强制 PBUS 表项：block 0..=3、bank 1..=2、value 0..=511
    /// （block 2/3 的 bank 2 是 I/Q 基带 DC DAC 码）。
    fn phy_pbus_force_test(block: u32, bank: u32, value: u32);
    /// 读 PBUS 表项（debug/工作模式下均为当前生效值）。
    fn phy_pbus_rd(block: u32, bank: u32) -> u32;
    /// vendor 单音发射（TX 链开关：libphy.a，zephyr esp_sdr_tx_chain_on
    /// 同款用法）。start=1 开链并调谐到 mhz，start=0 关链。
    /// backoff 为 2 dB 步进的衰减档（C5 无实际衰减，保留语义）。
    fn esp_phy_wifi_tx_tone(start: u32, mhz: u32, backoff: u32);
}

/// C5 逐频点接收重校准（reference/esp-sdr common/rx_recalibration.c C5 分支）。
///
/// 上游 prepare_rx 在调谐前必调：不做这步，接收链 DC 工作点不随频率更新，
/// 实测表现为捕获区恒定满幅样本（C2 现场证据）。必须持有射频锁。
fn rx_recalibrate_locked(frequency_mhz: u32) {
    // 释放手动增益位（AGCPWR_CTRL7 bit 23）：测量用的增益扫描不能被覆盖
    unsafe {
        let value = read_volatile(GAIN_STATUS_REGISTER as *const u32);
        write_volatile(GAIN_STATUS_REGISTER as *mut u32, value & !(1 << 23));
        phy_chip_set_chan_ana(frequency_mhz);
    }
    let phy_param_base = core::ptr::addr_of_mut!(phy_param);
    let band = unsafe { phy_param_base.add(42).read() };
    let flags = unsafe { phy_param_base.add(148) as *mut u32 };
    // 保留 PHY 已装的 IQ 系数；只使 DC/增益表缓存过期（0x280 位）
    unsafe {
        *flags &= !0x280;
        if band != 0 {
            phy_set_rx_gain_cal_dc(0, 1, phy_param_base.add(384), phy_param_base.add(912));
        } else {
            phy_set_rx_gain_cal_dc(0, 1, phy_param_base.add(312), phy_param_base.add(896));
            phy_adc_rate_cal_rxdc();
            phy_set_rx_gain_cal_dc(1, 1, phy_param_base.add(348), phy_param_base.add(904));
        }
        *flags |= 0x480;
        phy_set_rx_gain_table(frequency_mhz, 0);
    }
}

/// C5 接收链准备序列（reference/esp-sdr families/c5_c6_c61/receiver.c prepare_rx）。
///
/// 每次换频后必须重新执行。
fn vendor_prepare_receive(frequency_mhz: u32, channel_bandwidth_mode: u32) {
    rx_recalibrate_locked(frequency_mhz);
    unsafe {
        phy_set_chanfreq(frequency_mhz, channel_bandwidth_mode);
        phy_stop_tx_tone(1);
        phy_pbus_workmode();
        phy_pbus_xpd_tx_off();
        phy_pbus_xpd_rx_on(1);
        phy_set_rxclk_en(1);
    }
}

/// 射频控制状态机。调用方保证同一时刻只有一个所有者（单任务使用或加锁）。
pub struct Radio {
    frequency_mhz: u32,
    channel_bandwidth_mode: u32,
    gain_mode: GainMode,
    gain_code: u32,
    receive_ready: bool,
}

impl Radio {
    /// 创建控制器。要求射频电源域已点亮（esp_radio::wifi::new 之后）。
    ///
    /// 初始为硬件增益、频点 2412 MHz 但未执行接收链准备。
    #[must_use]
    pub const fn new() -> Self {
        Self {
            frequency_mhz: 2412,
            channel_bandwidth_mode: 0,
            gain_mode: GainMode::Hardware,
            gain_code: 40,
            receive_ready: false,
        }
    }

    /// 当前频点（最近一次成功调谐的值）。
    #[must_use]
    pub fn frequency_mhz(&self) -> u32 {
        self.frequency_mhz
    }

    /// 当前增益模式与增益码。
    #[must_use]
    pub const fn gain_mode(&self) -> GainMode {
        self.gain_mode
    }

    #[must_use]
    pub const fn gain_code(&self) -> u32 {
        self.gain_code
    }

    /// 从 AGCPWR_CTRL7 读最大已校准增益索引。
    ///
    /// 寄存器字段在调谐后由 PHY 填入；未调谐时读数无意义。
    #[must_use]
    pub fn calibrated_gain_maximum(&self) -> u32 {
        gain::max_from_status_register(unsafe { read_volatile(GAIN_STATUS_REGISTER as *const u32) })
    }

    /// AGCPWR_CTRL7 bit 23 的原始读数。
    ///
    /// 语义未经本板验证（上游从未标注该位含义）；C2 有真实信号后确认
    /// 是否等价于"增益已强制"。不要据此做控制决策。
    #[must_use]
    pub fn gain_status_bit23(&self) -> bool {
        gain::forced_from_status_register(unsafe {
            read_volatile(GAIN_STATUS_REGISTER as *const u32)
        })
    }

    /// 执行接收链准备（调谐 + 关发射 + 开接收 + 增益应用）。
    fn prepare_receive(&mut self) {
        vendor_prepare_receive(self.frequency_mhz, self.channel_bandwidth_mode);
        self.apply_gain();
        self.receive_ready = true;
    }

    /// 调谐到指定频点并重新准备接收链。
    pub fn set_frequency(&mut self, mhz: u32) -> Result<(), RadioError> {
        if !frequency::is_valid(mhz) {
            return Err(RadioError::Tune(TuneError::FrequencyOutOfRange {
                requested_mhz: mhz,
                minimum_mhz: frequency::MIN_MHZ,
                maximum_mhz: frequency::MAX_MHZ,
            }));
        }
        self.frequency_mhz = mhz;
        self.receive_ready = false;
        self.prepare_receive();
        Ok(())
    }

    /// 锁定手动增益码。超出已校准范围时报错（不静默截断）。
    pub fn set_gain_manual(&mut self, code: u32) -> Result<(), RadioError> {
        let maximum = self.calibrated_gain_maximum();
        if code > maximum {
            return Err(RadioError::Gain(GainError::CodeAboveMaximum {
                requested_code: code,
                calibrated_maximum: maximum,
            }));
        }
        self.gain_mode = GainMode::Manual;
        self.gain_code = code;
        if self.receive_ready {
            self.apply_gain();
        }
        Ok(())
    }

    /// 交还硬件 AGC。
    pub fn set_gain_hardware(&mut self) {
        self.gain_mode = GainMode::Hardware;
        if self.receive_ready {
            self.apply_gain();
        }
    }

    /// 重锁当前增益设置（AGC 会周期性夺回强制增益，调用方以 ~20 ms 周期驱动）。
    pub fn maintain_gain(&self) {
        if self.receive_ready {
            self.apply_gain();
        }
    }

    fn apply_gain(&self) {
        let manual = self.gain_mode == GainMode::Manual;
        unsafe {
            force_rx_gain(manual as u32, self.gain_code, 0);
        }
    }

    /// 开启 TX 链（zephyr esp_sdr_tx_chain_on）：vendor CW 单音路径打开
    /// 发射链（PBUS debug 模式、TX 时钟、校准 DC）后停掉音调，把 DAC
    /// 交给数据路径并保持链开。`power` 0..=15（每步 2 dB 回退）。
    ///
    /// 前置：已调谐到目标频点（set_frequency）；回放引擎（mmio replay）
    /// 随后启动。仅限合法频段由调用方保证。
    pub fn tx_chain_on(&mut self, power: u32) -> Result<(), RadioError> {
        if power > TX_POWER_MAX {
            return Err(RadioError::TxPower { requested: power });
        }
        self.receive_ready = false;
        unsafe {
            esp_phy_wifi_tx_tone(1, self.frequency_mhz, (TX_POWER_MAX - power) * TX_BACKOFF_STEP);
            phy_stop_tx_tone(1);
        }
        Ok(())
    }

    /// 关闭 TX 链并恢复 RX（zephyr esp_sdr_tx_chain_off：vendor tone 关 +
    /// 重新准备接收链）。
    pub fn tx_chain_off(&mut self) {
        unsafe {
            esp_phy_wifi_tx_tone(0, self.frequency_mhz, 0);
        }
        self.prepare_receive();
    }

    /// duplex 模式：TX 链保持开启的同时重新打开接收部件
    /// （zephyr rfprobe "tx carrier" 的 rx bits 1/2/4：workmode → RX 电源
    /// → RX 时钟，顺序照抄）。用于自环验证：RX 可采集自身 TX 的泄漏，
    /// 判断空口路径是否工作。
    pub fn rx_chain_duplex_on(&mut self) {
        unsafe {
            phy_pbus_workmode();
            phy_pbus_xpd_rx_on(1);
            phy_set_rxclk_en(1);
        }
    }

    /// 基带 DC 硬件校准（移植自 espsoup dc_cal.h / ESPARGOS
    /// receiver_dc_calibrate；算法为公开的 Newton 2x2 步进）。
    ///
    /// 原理：接收链基带 DC 吃掉 4 位 diag 窗口（实测 30-43 ADC8 计数），
    /// 用 PBUS debug 模式强制 I/Q DC DAC 码（block 2/3、bank 2）把 DC
    /// 压到 <0.5 ADC8。测量由调用方提供（20 MS/s dump 快照均值 /4）。
    ///
    /// 约束：
    /// - 仅手动增益（AGC 需要工作模式，debug 模式下 PBUS 不再跟随 AGC）；
    /// - 校准后 PBUS 停留在 debug 模式，**下一次调谐（prepare_receive）
    ///   会切回工作模式并作废修正**——换频后必须重校准（espsoup 同款
    ///   语义，其 DCAUTO 在每次运行前重做）；
    /// - measure 返回 false（快照失败）时中止并还原初始 DAC 码。
    ///
    /// f32 仅用于本一次性路径的 2x2 解算与 Newton 步进（C5 无 FPU，
    /// 软浮点足够：总运算量 ~百次级，不在数据面）。
    pub fn calibrate_dc<F>(&mut self, mut measure: F) -> Result<DcCalReport, DcCalError>
    where
        F: FnMut(&mut [f32; 2]) -> bool,
    {
        if self.gain_mode != GainMode::Manual {
            return Err(DcCalError::HardwareGain);
        }
        let mut report = DcCalReport::default();
        let mut before = [0.0f32; 2];
        if !measure(&mut before) {
            return Err(DcCalError::MeasurementFailed);
        }
        report.before_centi = [to_centi(before[0]), to_centi(before[1])];
        report.after_centi = report.before_centi;

        // DC 已足够小（幅度平方 < 0.25，即 |DC| < 0.5 ADC8）→ 不动硬件
        if dc_cost(before) < 0.25 {
            report.skipped = true;
            return Ok(report);
        }

        // 读全部 PBUS live 值并保留（debug 模式接管后原样回填，
        // 保持当前 RF/基带状态）
        let mut live = [[0u32; 2]; 4];
        for block in 0..4u32 {
            for bank in 1..=2u32 {
                live[block as usize][bank as usize - 1] =
                    unsafe { phy_pbus_rd(block, bank) };
            }
        }
        let base = [live[2][1] as i32, live[3][1] as i32];
        report.base = base;

        unsafe {
            phy_pbus_debugmode();
        }
        for block in 0..4u32 {
            for bank in 1..=2u32 {
                unsafe {
                    phy_pbus_force_test(block, bank, live[block as usize][bank as usize - 1]);
                }
            }
        }

        let mut best = base;
        let mut best_cost = dc_cost(before);
        let mut after = before;

        // 极性探测：强制 +64 码，看 DC 各轴响应符号与幅度（决定下降方向）。
        // 不用 espsoup 的 ±16 小步 Jacobian：本板实测灵敏度低于其阈值
        // （det<0.002 被拒、steps=0），大步长探测更鲁棒。
        // 注意 phy_pbus_rd 读 RAM 影子表，无法验证 force 写入——唯一有效
        // 的验证就是测量 DC 变化本身。
        dc_apply(clamp_i32(base[0] + 64, 0, 511), base[1]);
        let mut probe_i = [0.0f32; 2];
        let probe_i_ok = measure(&mut probe_i);
        dc_apply(base[0], clamp_i32(base[1] + 64, 0, 511));
        let mut probe_q = [0.0f32; 2];
        let probe_q_ok = measure(&mut probe_q);
        dc_apply(base[0], base[1]);
        if !probe_i_ok || !probe_q_ok {
            return Err(DcCalError::MeasurementFailed);
        }
        report.response_centi = [
            to_centi(probe_i[0] - before[0]),
            to_centi(probe_i[1] - before[1]),
            to_centi(probe_q[0] - before[0]),
            to_centi(probe_q[1] - before[1]),
        ];
        // I 轴极性：+64 码使 DC_I 增大则为正向（DAC 码与 DC 同向）
        let axis_i_sign: i32 = if probe_i[0] > before[0] { 1 } else { -1 };
        let axis_q_sign: i32 = if probe_q[1] > before[1] { 1 } else { -1 };
        // 灵敏度门槛：每码响应至少 0.002 ADC8（否则 DAC 无实际作用，放弃）
        let sensitivity_i = abs_f32(probe_i[0] - before[0]) / 64.0;
        let sensitivity_q = abs_f32(probe_q[1] - before[1]) / 64.0;
        report.responsive = sensitivity_i >= 0.002 || sensitivity_q >= 0.002;
        if !report.responsive {
            report.best = base;
            report.after_centi = report.before_centi;
            return Ok(report);
        }

        // 坐标下降：步长从 64 逐档减半到 1；每档每轴试 ±step，取实测
        // 更优者；|DC| < 0.5 提前收敛。
        let mut step = 64i32;
        while step >= 1 {
            if best_cost <= 0.25 {
                break;
            }
            for axis in 0..2usize {
                if dc_cost(after) <= 0.25 {
                    break;
                }
                let direction = if axis == 0 { axis_i_sign } else { axis_q_sign };
                let mut axis_best = best[axis];
                let mut axis_best_cost = dc_cost(after);
                let mut axis_best_dc = after;
                for attempt in [1i32, -1] {
                    let candidate = best[axis] + direction * attempt * step;
                    let candidate = candidate.clamp(base[axis] - 192, base[axis] + 192);
                    let candidate = clamp_i32(candidate, 0, 511);
                    if candidate == best[axis] {
                        continue;
                    }
                    let mut trial = best;
                    trial[axis] = candidate;
                    dc_apply(trial[0], trial[1]);
                    report.steps += 1;
                    let mut observed = [0.0f32; 2];
                    if !measure(&mut observed) {
                        continue;
                    }
                    let cost = dc_cost(observed);
                    if cost < axis_best_cost {
                        axis_best_cost = cost;
                        axis_best = candidate;
                        axis_best_dc = observed;
                    }
                }
                best[axis] = axis_best;
                after = axis_best_dc;
                dc_apply(best[0], best[1]);
            }
            best_cost = dc_cost(after);
            step /= 2;
        }
        // 锁存实测最优码（最后一轮 apply 已是 best）
        report.best = best;
        report.after_centi = [to_centi(after[0]), to_centi(after[1])];
        Ok(report)
    }
}

/// DC 校准失败模型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DcCalError {
    /// 硬件 AGC 模式下不可校准（debug 模式会让 PBUS 脱离 AGC）。
    HardwareGain,
    /// 测量回调返回失败（快照超时/过跑）。
    MeasurementFailed,
}

/// DC 校准报告（定点字段，defmt 直打；×100/×1000 缩放）。
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct DcCalReport {
    /// 校准前 DC（ADC8 counts × 100）。
    pub before_centi: [i32; 2],
    /// 校准后 DC（ADC8 counts × 100）。
    pub after_centi: [i32; 2],
    /// 初始 DAC 码（I, Q）。
    pub base: [i32; 2],
    /// 锁存的最优 DAC 码（I, Q）。
    pub best: [i32; 2],
    /// 实际测量步数。
    pub steps: u32,
    /// +64 码探测响应（DC 变化 ADC8 counts × 100）：
    /// [I 轴→DC_I, I 轴→DC_Q, Q 轴→DC_I, Q 轴→DC_Q]。
    pub response_centi: [i32; 4],
    /// DAC 是否有实际作用（灵敏度 ≥ 0.002 ADC8/码）。
    pub responsive: bool,
    /// DC 已足够小或响应异常，未做步进。
    pub skipped: bool,
}

impl Default for DcCalReport {
    fn default() -> Self {
        Self {
            before_centi: [0; 2],
            after_centi: [0; 2],
            base: [0; 2],
            best: [0; 2],
            steps: 0,
            response_centi: [0; 4],
            responsive: false,
            skipped: false,
        }
    }
}

fn dc_cost(dc: [f32; 2]) -> f32 {
    dc[0] * dc[0] + dc[1] * dc[1]
}

fn dc_apply(i: i32, q: i32) {
    unsafe {
        phy_pbus_force_test(2, 2, i as u32);
        phy_pbus_force_test(3, 2, q as u32);
    }
    // DAC 稳定延时（espsoup vTaskDelay 10 ms；忙等保守实现）
    for _ in 0..2_400_000u32 {
        core::hint::spin_loop();
    }
}

fn clamp_i32(value: i32, low: i32, high: i32) -> i32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

fn abs_f32(value: f32) -> f32 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}

fn round_f32(value: f32) -> i32 {
    if value >= 0.0 {
        (value + 0.5) as i32
    } else {
        (value - 0.5) as i32
    }
}

fn to_centi(value: f32) -> i32 {
    round_f32(value * 100.0)
}

/// PBUS 调试通道自检（DC 校准诊断）：进入 debug 模式，读 (2,2) 现值，
/// debug 模式内再读、强制写 400 后读回、写 (3,2) 后读回、还原。
/// 返回 [work 模式 rd(2,2)、debug 模式 rd(2,2)、force(2,2,400) 后、
/// force(3,2,100) 后 rd(3,2)]。
#[must_use]
pub fn pbus_self_test() -> [u32; 4] {
    unsafe {
        let work_before = phy_pbus_rd(2, 2);
        phy_pbus_debugmode();
        let debug_before = phy_pbus_rd(2, 2);
        phy_pbus_force_test(2, 2, 400);
        let forced_i = phy_pbus_rd(2, 2);
        phy_pbus_force_test(3, 2, 100);
        let forced_q = phy_pbus_rd(3, 2);
        phy_pbus_force_test(2, 2, work_before);
        phy_pbus_force_test(3, 2, 258);
        phy_pbus_workmode();
        [work_before, debug_before, forced_i, forced_q]
    }
}
