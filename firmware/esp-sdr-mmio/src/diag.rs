//! C4 无缝流硬件层：诊断总线 → GPIO 矩阵回环 → PARLIO → GDMA 环。
//!
//! 方法源自 esp-sdr-zephyr lib/esp_sdr/src/esp_sdr_diag.c（其自述来自
//! ESPsoup / C5VRX 的独立实现，GPL-3.0，本文件为只读参考后的 Rust 重写）：
//! dump engine 以无尽计数自由运行、SRAM 块交还 CPU；调制解调器诊断总线
//! MODEM_DIAG[19:0] 承载 I/Q 各一个 4 位窗口；GPIO 矩阵把 8 根内部信号线
//! 接到 8 个 pad 再回环进 PARLIO RX 数据线；GDMA 环形描述符无 CPU 收集。
//!
//! 全部寄存器经 PAC（esp32c5）直写，不经过 esp-hal 驱动——软分隔符 +
//! 环形描述符 + 长度写回是裸用法，驱动语义不覆盖。

use core::ptr::{read_volatile, write_volatile};

/// 回环占用 8 个 pad（板卡上不得接任何东西；zephyr default_pins）。
pub const DIAG_PINS: [u8; 8] = [0, 1, 3, 4, 5, 7, 10, 23];
/// 环大小（字节）：7 个 BLOCK，40 MB/s 下 1.43 ms 一圈。
pub const RING_BYTES: usize = 57_344;
/// 消费单位（字节）。
pub const BLOCK_BYTES: usize = 8192;
const NODE_BYTES: usize = 3584;
const NODES: usize = RING_BYTES / NODE_BYTES;

/// 环与描述符的驻留地址（块 1 下半区；CPU 在自由运行态拥有整块）。
const RING_ADDRESS: u32 = 0x4082_0000;
const DESC_ADDRESS: u32 = RING_ADDRESS + RING_BYTES as u32;

// dump engine 寄存器（与 hardware 模块同源，docs/c5-registers.md）。
const DUMP_CTRL_REG: u32 = 0x600a_9004;
const DUMP_CTRL_RUN: u32 = 1 << 31;
const DUMP_CTRL_TRIGGER: u32 = 1 << 19;
const DUMP_CTRL_DONE: u32 = 1 << 18;
const DUMP_MODE_REG: u32 = 0x600a_9008;
const DUMP_MODE_RATE_MASK: u32 = 0x7 << 21;
const DUMP_MODE_SOURCE_MASK: u32 = 0xF << 17;
const DUMP_MODE_SOURCE_IQ: u32 = 15 << 17;
const DUMP_CONFIG_REG: u32 = 0x600a_9018;
const DUMP_CONFIG_LANES_MASK: u32 = 0x01FF_FFFF;
/// 接收 I/Q 的 lane 配置（信号 24..27 + bit 24，zephyr 实测唯一有效值）。
const DUMP_CONFIG_IQ: u32 = 0x1006_DA58;
const DUMP_CLK_FORCE_REG: u32 = 0x600a_9c04;
const DUMP_CLK_EN_REG: u32 = 0x600a_0800;
const DUMP_CLK_EN_BIT: u32 = 1 << 2;
const DUMP_USAGE_REG: u32 = 0x6009_5004;
const DUMP_USAGE_FIELD_MASK: u32 = 0xF << 8;
/// 块 1 属主值：bits 11:8 = 2（BIT(1)<<8）+ bit 16（zephyr 命名 MAC_DUMP_ALLOC；
/// espsoup 实测语义为半区选择：1=上半区 0x40830000。同一位，置 1 为工作态）。
const DUMP_USAGE_BANK1: u32 = 1 << 9;
/// dump engine 计数字段 17 位；>65535 永不置 DONE → 自由运行（zephyr DUMP_ENDLESS）。
const DUMP_ENDLESS: u32 = 0x1_FFFF;

// GPIO 矩阵信号编号（esp-idf components/soc/esp32c5/include/soc/gpio_sig_map.h）。
const MODEM_DIAG0_IDX: u32 = 106;
const PARL_RX_DATA0_IDX: u32 = 48;
const SIG_GPIO_OUT_IDX: u32 = 256;

/// GDMA 外设触发号（PARLIO0）。
const DMA_TRIGGER_PARLIO: u8 = 9;

/// GDMA 描述符 dw0 位域（hal/dma_types.h）：size 0..11，length 12..23
/// （RX 由硬件写回），owner bit 31（1=DMA，0=CPU）。
const DW0_LENGTH_MASK: u32 = 0xFFF << 12;
const DW0_OWNER_DMA: u32 = 1 << 31;

/// 诊断流失败模型。
#[derive(Debug, defmt::Format)]
pub enum DiagError {
    /// 环被生产者套圈（消费太慢）。
    Overrun,
    /// 启动后 500 ms 环内无数据（回环/PARLIO/DMA 未通）。
    NoData,
}

/// 诊断流的采集侧句柄。启动后 CPU 不参与搬运，只轮询描述符推进生产计数。
pub struct DiagStream {
    usage_saved: u32,
    head: usize,
    producer_bytes: u64,
    overrun: bool,
    running: bool,
}

fn rd(address: u32) -> u32 {
    unsafe { read_volatile(address as *const u32) }
}

fn wr(address: u32, value: u32) {
    unsafe { write_volatile(address as *mut u32, value) }
}

fn riscv_fence() {
    unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };
}

/// 诊断线 k（0..8）在窗口 lsb 下对应的 MODEM_DIAG 信号号（zephyr lane_of）。
const fn lane_of(k: u32, lsb: u32) -> u32 {
    if k < 4 {
        lsb + k
    } else {
        10 + lsb + (k - 4)
    }
}

/// 8 个 pad 的 IO_MUX + 矩阵回环配置。
fn pads_route(lsb: u32) {
    let io_mux = unsafe { esp32c5::IO_MUX::steal() };
    let gpio = unsafe { esp32c5::GPIO::steal() };
    for (k, &pin) in DIAG_PINS.iter().enumerate() {
        let k = k as u32;
        let pin32 = u32::from(pin);
        // pad：GPIO 功能（mcu_sel=0）、无上下拉、输入使能、最低驱动档
        io_mux.gpio(pin as usize).modify(|_, w| unsafe {
            w.mcu_sel().bits(0);
            w.mcu_wpu().bit(false);
            w.mcu_wpd().bit(false);
            w.mcu_ie().bit(true);
            w.mcu_drv().bits(0);
            w.filter_en().bit(false);
            w.hys_en().bit(false);
            w
        });
        // 输出：MODEM_DIAG 信号 → pad
        gpio.func_out_sel_cfg(pin as usize).write(|w| unsafe {
            w.bits(
                (MODEM_DIAG0_IDX + lane_of(k, lsb))
                    | (0 << 9) /* inv_sel */
                    | (0 << 10) /* oen_sel */
                    | (0 << 11), /* oen_inv_sel */
            )
        });
        gpio.enable_w1ts().write(|w| unsafe { w.bits(1 << pin32) });
        // 输入：pad → PARL_RX_DATA 信号
        gpio.func_in_sel_cfg((PARL_RX_DATA0_IDX + k) as usize).write(|w| unsafe {
            w.bits(pin32 | (1 << 2)) // in_sel | sel(使能矩阵输入)
        });
    }
}

/// 归还 8 个 pad（输出断开、矩阵输入断开）。
fn pads_release() {
    let gpio = unsafe { esp32c5::GPIO::steal() };
    for (k, &pin) in DIAG_PINS.iter().enumerate() {
        let pin32 = u32::from(pin);
        gpio.enable_w1tc().write(|w| unsafe { w.bits(1 << pin32) });
        gpio.func_out_sel_cfg(pin as usize)
            .write(|w| unsafe { w.bits(SIG_GPIO_OUT_IDX) });
        gpio.func_in_sel_cfg((PARL_RX_DATA0_IDX + k as u32) as usize)
            .write(|w| unsafe { w.bits(0) });
    }
}

/// GDMA 环形描述符初始化：16 节点 × 3584 字节闭环。
fn ring_init() {
    for index in 0..NODES {
        let desc = DESC_ADDRESS + (index as u32) * 12;
        let buffer = RING_ADDRESS + (index as u32) * NODE_BYTES as u32;
        let next = DESC_ADDRESS + (((index + 1) % NODES) as u32) * 12;
        unsafe {
            write_volatile(desc as *mut u32, NODE_BYTES as u32 | DW0_OWNER_DMA);
            write_volatile((desc + 4) as *mut u32, buffer);
            write_volatile((desc + 8) as *mut u32, next);
        }
    }
}

/// GDMA RX 中断状态：ISR 与轮询共用。
static DMA_IN_STATUS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// 中断驱动模式标志（start 时置位）。
#[allow(dead_code)]
static ISR_ARMED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);/// ISR 消费游标与累计（与 poll 字段同步由 ISR 模式下独占）。
static ISR_HEAD: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
/// RV32IMAC 的 AtomicU64 不可用：高低各一个原子字（读侧先低后高再读低校验）。
static ISR_BYTES_LO: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static ISR_BYTES_HI: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
/// ISR 完成一圈以上未消费的标志（overrun）。
static ISR_OVERRUN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
/// 新数据到达标志（ISR 置位，消费任务清零）。
static DATA_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// GDMA in_ch0 中断服务（zephyr diag_isr 的 Rust 复刻）。
/// 清状态 + 走描述符推进生产计数。esp-hal 向量表调用，签名固定。
#[unsafe(no_mangle)]
extern "C" fn esp_sdr_diag_dma_isr() {
    let dma = unsafe { esp32c5::DMA::steal() };
    let status = dma.in_int_ch(0).st().read().bits();
    dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
    DMA_IN_STATUS.fetch_or(status, core::sync::atomic::Ordering::Relaxed);

    let mut head = ISR_HEAD.load(core::sync::atomic::Ordering::Relaxed);
    let mut bytes_added: u64 = 0;
    let mut walked = 0;
    while walked < NODES {
        let desc = DESC_ADDRESS + (head as u32) * 12;
        let dw0 = unsafe { read_volatile(desc as *const u32) };
        let length = (dw0 & DW0_LENGTH_MASK) >> 12;
        if length == 0 {
            break;
        }
        unsafe { write_volatile(desc as *mut u32, dw0 & !DW0_LENGTH_MASK | DW0_OWNER_DMA) };
        bytes_added += u64::from(length);
        head = (head + 1) % NODES;
        walked += 1;
    }
    ISR_HEAD.store(head, core::sync::atomic::Ordering::Relaxed);
    if walked == NODES {
        ISR_OVERRUN.store(true, core::sync::atomic::Ordering::Relaxed);
    }
    if bytes_added != 0 {
        // 64 位计数在 32 位原子上的拆分：低位 wrapping add，进位时高位 +1
        let lo = ISR_BYTES_LO
            .fetch_add(bytes_added as usize, core::sync::atomic::Ordering::Relaxed)
            as u64;
        let total_lo = lo + bytes_added;
        if total_lo > u32::MAX as u64 {
            ISR_BYTES_HI.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        DATA_READY.store(true, core::sync::atomic::Ordering::Relaxed);
    }
}

/// 启动诊断总线数据源：dump engine 自由运行（无尽计数）+ 8 pad 回环的
/// 输出侧。PARLIO/GDMA 由调用方配置——裸路径见 [DiagStream::start]，
/// esp-hal 驱动路径（parl_io::ParlIoRx + DmaRxStreamBuf）由应用层直接
/// 使用 esp-hal API。返回 usage_saved 供 [diag_bus_stop] 归还。
///
/// 前置条件：射频已点亮（混杂模式开启，接收链在线）。
pub fn diag_bus_start(lsb: u8) -> u32 {
    // ---- dump engine 自由运行（zephyr esp_sdr_diag_run 序列）----
    let usage_saved = rd(DUMP_USAGE_REG);
    wr(DUMP_CTRL_REG, 0);
    wr(DUMP_CLK_FORCE_REG, u32::MAX);
    let clk_en = rd(DUMP_CLK_EN_REG);
    wr(DUMP_CLK_EN_REG, clk_en | DUMP_CLK_EN_BIT);
    let lane = rd(DUMP_CONFIG_REG);
    wr(DUMP_CONFIG_REG, (lane & !DUMP_CONFIG_LANES_MASK) | DUMP_CONFIG_IQ);
    let mode = rd(DUMP_MODE_REG);
    wr(DUMP_MODE_REG, (mode & !DUMP_MODE_SOURCE_MASK) | DUMP_MODE_SOURCE_IQ);
    // 让渡：bits 11:8=块 1 + bit16（zephyr 的 MAC_DUMP_ALLOC 位；
    // espsoup 语义为半区选择=1→上半区，两种命名同一位，置 1 为实测工作态）
    wr(
        DUMP_USAGE_REG,
        (usage_saved & !DUMP_USAGE_FIELD_MASK) | DUMP_USAGE_BANK1 | 1 << 16,
    );
    riscv_fence();
    let _flush = rd(DUMP_USAGE_REG);
    // 速率字段清零 + 无尽计数触发
    let mode = rd(DUMP_MODE_REG);
    wr(DUMP_MODE_REG, mode & !DUMP_MODE_RATE_MASK);
    let ctrl = DUMP_CTRL_RUN | DUMP_ENDLESS;
    wr(DUMP_CTRL_REG, DUMP_ENDLESS | DUMP_CTRL_DONE);
    wr(DUMP_CTRL_REG, ctrl);
    wr(DUMP_CTRL_REG, ctrl | DUMP_CTRL_TRIGGER);
    wr(DUMP_CTRL_REG, ctrl);
    // 块 1 立即交还 CPU：环与描述符住在下半区
    wr(DUMP_USAGE_REG, usage_saved);
    riscv_fence();
    let _flush = rd(DUMP_USAGE_REG);

    // ---- pad 回环 ----
    pads_route(u32::from(lsb));
    usage_saved
}

/// 停止诊断总线数据源（[diag_bus_start] 的配对收尾）。
pub fn diag_bus_stop(usage_saved: u32) {
    pads_release();
    wr(DUMP_CTRL_REG, 0);
    wr(DUMP_USAGE_REG, usage_saved);
    riscv_fence();
    let _flush = rd(DUMP_USAGE_REG);
}

impl DiagStream {
    /// 启动无缝流（中断驱动版：GDMA in_done ISR 推进生产计数，
    /// zephyr 原路径的忠实复刻）。前置条件与 [Self::start] 相同。
    pub fn start_with_isr(lsb: u8) -> Self {
        let stream = Self::start(lsb);
        // start 内已做过一轮总线复位/配置；此处补中断绑定与使能
        ISR_HEAD.store(0, core::sync::atomic::Ordering::Relaxed);
        ISR_BYTES_LO.store(0, core::sync::atomic::Ordering::Relaxed);
        ISR_BYTES_HI.store(0, core::sync::atomic::Ordering::Relaxed);
        ISR_OVERRUN.store(false, core::sync::atomic::Ordering::Relaxed);
        DATA_READY.store(false, core::sync::atomic::Ordering::Relaxed);
        ISR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);

        let dma = unsafe { esp32c5::DMA::steal() };
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
        dma.in_int_ch(0).ena().modify(|_, w| unsafe {
            // in_done（RX_DONE 逐描述符）+ in_suc_eof + 错误位全开（对齐 zephyr
            // AHB_DMA_LL_RX_EVENT_MASK = 全事件）
            w.bits(u32::MAX & !(1 << 5)) // 排除保留位
        });
        let handler = esp_hal::interrupt::InterruptHandler::new(
            esp_sdr_diag_dma_isr,
            esp_hal::interrupt::Priority::Priority1,
        );
        esp_hal::interrupt::bind_handler(esp32c5::Interrupt::DMA_IN_CH0, handler);
        esp_hal::interrupt::enable(
            esp32c5::Interrupt::DMA_IN_CH0,
            esp_hal::interrupt::Priority::Priority1,
        );
        stream
    }

    /// 中断模式下的生产计数（高低位原子拼 64 位）。
    #[must_use]
    pub fn producer_bytes_isr(&self) -> u64 {
        loop {
            let hi = ISR_BYTES_HI.load(core::sync::atomic::Ordering::Relaxed) as u64;
            let lo = ISR_BYTES_LO.load(core::sync::atomic::Ordering::Relaxed) as u64;
            let hi_again = ISR_BYTES_HI.load(core::sync::atomic::Ordering::Relaxed) as u64;
            if hi == hi_again {
                return (hi << 32) | lo;
            }
        }
    }

    /// 中断模式下的 overrun 状态。
    #[must_use]
    pub fn overrun_isr(&self) -> bool {
        ISR_OVERRUN.load(core::sync::atomic::Ordering::Relaxed)
    }
    /// 启动无缝流。前置条件：射频已点亮（capburst 同款 bring-up，
    /// 混杂模式开启），调谐与增益已设置。
    ///
    /// lsb 为 nibble 窗口位置（0..=6；值过低会削波，见 zephyr
    /// esp_sdr_diag_level 的 99.9% 幅度法，自动选窗在 C4 后续版本提供）。
    pub fn start(lsb: u8) -> Self {
        let pcr = unsafe { esp32c5::PCR::steal() };
        let parlio = unsafe { esp32c5::PARL_IO::steal() };
        let dma = unsafe { esp32c5::DMA::steal() };

        // ---- dump engine 自由运行 + pad 回环（与驱动路径共用）----
        let usage_saved = diag_bus_start(lsb);

        // ---- 环与描述符 ----
        ring_init();

        // ---- PARLIO RX 配置（先于 DMA：zephyr 顺序，diagprobe2 16 轮验证）----
        // 顺序敏感：rx_data_cfg 的 bitlen 必须在时钟域/FIFO 复位与时钟
        // 关闭之后、reg_update 之前写入（否则字段静默不生效——C4 实测）。
        // PCR 总线复位（zephyr parlio_ll_reset_register）
        pcr.parl_io_conf().modify(|_, w| {
            w.parl_clk_en().bit(true);
            w.parl_rst_en().bit(true);
            w
        });
        pcr.parl_io_conf().modify(|_, w| {
            w.parl_rst_en().bit(false);
            w
        });
        // 时钟源先建立（对齐 esp-hal PeripheralGuard 行为：en=true 状态下
        // 写 sel。sel 在 en=false 下写入实测不生效——回灌首版 0 字节，
        // 时钟源未切到 PLL_F240M 则无采样时钟）。
        pcr.parl_clk_rx_conf().modify(|_, w| unsafe {
            w.parl_clk_rx_en().bit(true);
            w.parl_clk_rx_sel().bits(2); // PLL_F240M
            w
        });
        let mut settle = 0;
        while settle < 100 {
            settle += 1;
        }
        // RX 时钟域复位 + FIFO 复位（zephyr rx_reset_clock / rx_reset_fifo）
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_rx_rst_en().bit(true);
            w
        });
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_rx_rst_en().bit(false);
            w
        });
        parlio.fifo_cfg().modify(|_, w| {
            w.rx_fifo_srst().bit(true);
            w
        });
        parlio.fifo_cfg().modify(|_, w| {
            w.rx_fifo_srst().bit(false);
            w
        });
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_clk_rx_en().bit(false);
            w
        });
        parlio.rx_start_cfg().write(|w| {
            w.rx_start().bit(false);
            w
        });
        // PLL_F240M / 6 = 40 MHz（div_num 语义 integer-1，PRUN 实测 ppm 1.5）
        pcr.parl_clk_rx_conf().modify(|_, w| unsafe {
            w.parl_clk_rx_sel().bits(2);
            w
        });
        pcr.parl_clk_rx_conf().modify(|_, w| unsafe {
            w.parl_clk_rx_div_num().bits(5);
            w
        });
        // 软分隔符：期望位长 = 环总位（19 位字段；EOF=RX_FULL 在 bitlen 处，
        // 每圈一个 EOF + auto_ret 回链头 = 无限流的引擎，C4 破案关键）
        parlio.rx_data_cfg().write(|w| unsafe {
            w.rx_bus_wid_sel().bits(3); // 8 位
            w.rx_bitlen().bits((RING_BYTES * 8) as u32);
            w.rx_data_order_inv().bit(false); // LSB 先行
            w
        });
        // 写入断言：bitlen 字段读回必须等于期望值（时序不满足时静默为 0，
        // 流会在 0-4 个描述符后停——C4 实测，响亮崩溃优于静默错流）
        let data_cfg = parlio.rx_data_cfg().read().bits();
        assert!(
            (data_cfg >> 9) & 0x7_FFFF == (RING_BYTES * 8) as u32,
            "rx_bitlen write lost: rx_data_cfg=0x{:08x}",
            data_cfg
        );
        // EOF=RX_FULL + 超时关（复位默认开：数据间隙触发 error EOF）+ gating 关
        parlio.rx_genrl_cfg().modify(|_, w| {
            w.rx_gating_en().bit(false);
            w.rx_timeout_en().bit(false);
            w.rx_eof_gen_sel().bit(false);
            w
        });
        parlio.rx_clk_cfg().write(|w| {
            w.rx_clk_i_inv().bit(false);
            w.rx_clk_o_inv().bit(false);
            w
        });
        parlio.rx_mode_cfg().modify(|_, w| unsafe {
            w.rx_smp_mode_sel().bits(2); // 软接收模式
            w.rx_sw_en().bit(false);
            w
        });
        parlio.reg_update().write(|w| {
            w.rx_reg_update().bit(true);
            w
        });
        // SVD 将 reg_update 标记为只写（不可回读自清位），有界自旋替代轮询
        let mut spin = 0;
        while spin < 1000 {
            spin += 1;
        }

        // ---- GDMA：AHB 组通道 0，触发 PARLIO（DMA armed 在单元启动前）----
        // PCR 总线时钟必须先开（手工验证路径由 esp-hal PeripheralGuard
        // 代办；裸路径缺它则 DMA 寄存器写入全部无效——C4 回灌实测全零）
        pcr.gdma_conf().modify(|_, w| {
            w.gdma_clk_en().bit(true);
            w.gdma_rst_en().bit(true);
            w
        });
        pcr.gdma_conf().modify(|_, w| {
            w.gdma_rst_en().bit(false);
            w
        });
        dma.misc_conf().modify(|_, w| {
            w.clk_en().bit(true);
            w
        });
        // 可访问内存窗口（复位为空 = 任何写都报 dscr_err；esp-hal 的 C5
        // setup() 未配置此项——注释掉的 TODO）
        dma.intr_mem_start_addr().write(|w| unsafe { w.bits(0x4080_0000) });
        dma.intr_mem_end_addr().write(|w| unsafe { w.bits(0x4400_0000) });
        let ch = dma.ch(0);
        ch.in_conf0().modify(|_, w| {
            w.in_rst().bit(true);
            w
        });
        ch.in_conf0().modify(|_, w| {
            w.in_rst().bit(false);
            w
        });
        ch.in_conf1().modify(|_, w| {
            w.in_check_owner().bit(false);
            w
        });
        ch.in_peri_sel().modify(|_, w| unsafe {
            w.peri_in_sel().bits(DMA_TRIGGER_PARLIO);
            w
        });
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
        dma.in_link_addr_ch(0)
            .write(|w| unsafe { w.bits(DESC_ADDRESS) });
        // EOF 后自动回链头（复位默认 1，显式写出意图）
        dma.ch(0).in_link().modify(|_, w| {
            w.inlink_auto_ret().bit(true);
            w
        });
        ch.in_link().modify(|_, w| {
            w.inlink_start().bit(true);
            w
        });

        // ---- 启动三连（zephyr：单元 → 时钟 → 软分隔符）----
        parlio.rx_start_cfg().write(|w| {
            w.rx_start().bit(true);
            w
        });
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_clk_rx_en().bit(true);
            w
        });
        parlio.rx_mode_cfg().modify(|_, w| {
            w.rx_sw_en().bit(true);
            w
        });

        Self {
            usage_saved,
            head: 0,
            producer_bytes: 0,
            overrun: false,
            running: true,
        }
    }

    /// 轮询推进生产计数：遍历描述符自上次位置起已完成的长度写回，
    /// 并清 GDMA RX 中断状态（zephyr ISR 同款；状态粘滞会挂起 IN 状态机，
    /// 表现为 FIFO 溢出 + 链表停走，C4 实测）。
    pub fn poll_producer(&mut self) -> u64 {
        let dma = unsafe { esp32c5::DMA::steal() };
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
        let mut walked = 0;
        while walked < NODES {
            let desc = DESC_ADDRESS + (self.head as u32) * 12;
            let dw0 = unsafe { read_volatile(desc as *const u32) };
            let length = (dw0 & DW0_LENGTH_MASK) >> 12;
            if length == 0 {
                break;
            }
            // 清 length 并把 owner 归还 DMA（zephyr ISR 同款两步；不重设 owner，
            // DMA 回绕到该描述符时停链——C4 关键发现）
            unsafe { write_volatile(desc as *mut u32, (dw0 & !DW0_LENGTH_MASK) | DW0_OWNER_DMA) };
            self.producer_bytes += u64::from(length);
            self.head = (self.head + 1) % NODES;
            walked += 1;
        }
        if walked == NODES {
            self.overrun = true;
        }
        self.producer_bytes
    }

    /// 生产者是否曾套圈（消费太慢，环被覆写）。
    #[must_use]
    pub fn overrun(&self) -> bool {
        self.overrun
    }

    /// 累计生产字节数。
    #[must_use]
    pub fn producer_bytes(&self) -> u64 {
        self.producer_bytes
    }

    /// 自 consumed_bytes 起的环内连续可读长度（不跨环尾；消费侧零拷贝
    /// 分段依据）。consumed 不得超过生产计数。
    #[must_use]
    pub fn contiguous_available(&self, consumed_bytes: u64) -> usize {
        assert!(
            consumed_bytes <= self.producer_bytes,
            "consumer ahead of producer"
        );
        let available = self.producer_bytes - consumed_bytes;
        let ring_offset = (consumed_bytes % RING_BYTES as u64) as usize;
        let to_ring_end = RING_BYTES - ring_offset;
        available.min(to_ring_end as u64) as usize
    }

    /// 自 consumed_bytes 起 len 字节的环内切片（len 须 ≤
    /// [Self::contiguous_available]；调用方消费后推进自身游标）。
    #[must_use]
    pub fn ring_slice(&self, consumed_bytes: u64, len: usize) -> &[u8] {
        let ring_offset = (consumed_bytes % RING_BYTES as u64) as u32;
        assert!(len <= self.contiguous_available(consumed_bytes));
        unsafe { core::slice::from_raw_parts((RING_ADDRESS + ring_offset) as *const u8, len) }
    }

    /// 环内某块的零拷贝视图（block_index 以 BLOCK_BYTES 为单位）。
    #[must_use]
    pub fn block(&self, block_index: u64) -> &[u8] {
        let offset = ((block_index as usize * BLOCK_BYTES) % RING_BYTES) as u32;
        unsafe { core::slice::from_raw_parts((RING_ADDRESS + offset) as *const u8, BLOCK_BYTES) }
    }

    /// 停止：PARLIO/DMA/pad/dump 全部收尾。
    pub fn stop(&mut self) {
        if !self.running {
            return;
        }
        let pcr = unsafe { esp32c5::PCR::steal() };
        let parlio = unsafe { esp32c5::PARL_IO::steal() };
        let dma = unsafe { esp32c5::DMA::steal() };

        parlio.rx_mode_cfg().modify(|_, w| {
            w.rx_sw_en().bit(false);
            w
        });
        parlio.rx_start_cfg().write(|w| {
            w.rx_start().bit(false);
            w
        });
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_clk_rx_en().bit(false);
            w
        });
        dma.ch(0).in_link().modify(|_, w| {
            w.inlink_stop().bit(true);
            w
        });
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });

        diag_bus_stop(self.usage_saved);
        self.running = false;
    }
}

impl Drop for DiagStream {
    fn drop(&mut self) {
        self.stop();
    }
}
