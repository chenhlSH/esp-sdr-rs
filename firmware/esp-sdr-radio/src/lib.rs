//! ESP32-C5 射频初始化与控制层。
//!
//! 前置契约：射频电源域与 PHY 校准必须已由 vendor WiFi 栈完成
//! （esp_radio::wifi::new 建立未启动的控制器即满足）。
//!
//! 职责：vendor blob 函数绑定（phy_set_chanfreq / force_rx_gain / RX 链路准备序列）、
//! 增益锁定维护。全部公开 API 以 Result 返回，显式错误枚举定义故障模型。

#![no_std]

use core::ptr::read_volatile;

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
}

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
}

/// C5 接收链准备序列（reference/esp-sdr families/c5_c6_c61/receiver.c prepare_rx）。
///
/// 每次换频后必须重新执行。
fn vendor_prepare_receive(frequency_mhz: u32, channel_bandwidth_mode: u32) {
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
}
