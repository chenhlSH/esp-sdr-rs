//! C4 option A 诊断探针：esp-hal 官方 parl_io::ParlIoRx + DmaRxStreamBuf
//! 驱动路径（对齐 espsoup 的 IDF 驱动用法），与裸寄存器路径（diagprobe）
//! 互为跨实现数据点。数据源仍为 dump engine 自由运行 + pad 回环
//! （diag_bus_start）。
//!
//! 关键差异记录（docs/c5-registers.md §10.2）：
//! - 驱动路径用软件链（DmaRxStreamBuf consume 追加），非 zephyr 硬件环。
//! - esp-hal apply_clock_divider 与 hal fork 的 div_num 语义相悖
//!   （直分频 vs integer-1）：本探针传 40 MHz 由实测吞吐裁定语义。
//! - esp-hal 1.2.2 C5 AHB-GDMA setup() 未配置可访问内存窗口（注释掉的
//!   TODO），需手工补 0x40800000..0x44000000。
//! - do_prepare 强制 indscr_burst_en(true)：软件链链尾 NULL 会被 C5
//!   AHB-GDMA 判 in_dscr_err，需闭环描述符链 + 关突发预取。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::diag_bus_start;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

/// 驱动路径缓冲位置：块 1 下半区（与裸路径同址——.bss 放不下 57 KB，
/// 链接脚本断言 .bss 不得进入块 1；数据点只换 PARLIO/GDMA 配置路径）。
const RING_BYTES: usize = 57_344;
const CHUNK_BYTES: usize = 3584;
const RING_ADDRESS: u32 = 0x4082_0000;
const DESC_ADDRESS: u32 = RING_ADDRESS + RING_BYTES as u32;

#[embassy_executor::task]
async fn diag_probe_task() {
    defmt::info!("diagprobe2: esp-hal driver path start");

    // 数据源：dump 自由运行 + pad 回环输出侧（与裸路径共用）
    let usage_saved = diag_bus_start(0);

    // esp-hal 驱动路径。RxEightBits::configure 会把 8 个 pad 绑到
    // PARL_RX_DATA0..7（输入侧），不触碰输出侧的 MODEM_DIAG 绑定。
    let peripherals = unsafe { esp_hal::peripherals::Peripherals::steal() };
    let parl_io = esp_hal::parl_io::ParlIo::new(
        peripherals.PARL_IO,
        peripherals.DMA_CH0,
    )
    .unwrap();
    let rx_pins = esp_hal::parl_io::RxEightBits::new(
        peripherals.GPIO0,
        peripherals.GPIO1,
        peripherals.GPIO3,
        peripherals.GPIO4,
        peripherals.GPIO5,
        peripherals.GPIO7,
        peripherals.GPIO10,
        peripherals.GPIO23,
    );
    let config = esp_hal::parl_io::RxConfig::default()
        // esp-hal apply_clock_divider 直分频语义写 div_num=source/freq；
        // C5 实际语义 integer-1（hal fork/IDF）。要 40 MHz（240/6）须
        // div_num=5，即请求 240/5=48 MHz（48>40 会撞 esp-hal 上限检查？
        // 不会——上限检查的是请求值 48>40 会被拒！故 40 MHz 请求得
        // div_num=6=34.29 MB/s。40 MHz 需 PAC 直改 div_num——见启动段）。
        .with_frequency(esp_hal::time::Rate::from_mhz(40))
        .with_bit_order(esp_hal::parl_io::BitPackOrder::Lsb);
    let parl_rx = parl_io
        .rx
        .with_config(rx_pins, esp_hal::gpio::NoPin, config)
        .unwrap();
    // ParlIoRx 持有外设 guard（时钟保持），勿提前 drop
    let _parl_rx_holder = parl_rx;

    // 块 1 裸地址构造 DmaRxStreamBuf（地址 4 字节对齐；CPU 拥有整块）
    use esp_hal::dma::DmaDescriptor;
    let nodes = RING_BYTES / CHUNK_BYTES;
    let ring: &'static mut [u8] = unsafe {
        core::slice::from_raw_parts_mut(RING_ADDRESS as *mut u8, RING_BYTES)
    };
    let descriptors: &'static mut [DmaDescriptor] = unsafe {
        core::slice::from_raw_parts_mut(DESC_ADDRESS as *mut DmaDescriptor, nodes)
    };
    for byte in ring.iter_mut() {
        *byte = 0;
    }
    for descriptor in descriptors.iter_mut() {
        *descriptor = DmaDescriptor::EMPTY;
    }
    let mut buffer = esp_hal::dma::DmaRxStreamBuf::new(
        unsafe {
            esp_hal::dma::aligned::DmaAlignedMut::new_unchecked(descriptors)
        },
        unsafe { esp_hal::dma::aligned::DmaAlignedMut::new_unchecked(ring) },
    )
    .unwrap();

    // esp-hal 1.2.2 的 C5 AHB-GDMA setup() 未配置可访问内存窗口
    // （ahb_v2.rs 中被注释的 TODO）。zephyr/IDF 的
    // ahb_dma_ll_set_default_memory_range 显式写 0x40800000..0x44000000。
    // 窗口寄存器是 DMA 全局的，此处补齐（实测必要：不补时数据写报
    // in_dscr_err）。
    {
        let dma = unsafe { esp32c5::DMA::steal() };
        // 通道占用诊断：WiFi blob 起来后看 3 个通道谁在跑（inlink_addr
        // 非零 = 活跃链）
        for channel in 0..3 {
            defmt::info!(
                "dma ch{} pre: link_addr=0x{:08x} state=0x{:08x} int=0x{:08x}",
                channel,
                dma.in_link_addr_ch(channel as usize).read().bits(),
                dma.ch(channel as usize).in_state().read().bits(),
                dma.in_int_ch(channel as usize).raw().read().bits()
            );
        }
        dma.intr_mem_start_addr()
            .write(|w| unsafe { w.bits(0x4080_0000) });
        dma.intr_mem_end_addr()
            .write(|w| unsafe { w.bits(0x4400_0000) });
    }

    // 手工启动（绕过 ParlIoRx::read）：read() 内 do_prepare 强制
    // indscr_burst_en(true)，软件链链尾 next=NULL，C5 AHB-GDMA 启动即报
    // in_dscr_err（三轮实测）。此处用 pub trait DmaRxBuffer::prepare 建链，
    // 再闭环描述符（环形链）+ 手工复刻 do_prepare 的 DMA 配置与 PARLIO
    // 启动序列。保留 esp-hal 的 buffer/view 消费逻辑（into_view/pop）。
    {
        use esp_hal::dma::DmaRxBuffer as _;
        let preparation = buffer.prepare();
        assert!(preparation.start as u32 == DESC_ADDRESS);
    }
    // 闭环描述符链（C5 AHB-GDMA：NULL next = in_dscr_err；顺序链不可行，
    // 第四轮实测 0 字节 + dscr_err）。环形链 + owner 轮询消费 + 空队列
    // restart 唤醒（IDF gdma append 语义）。
    {
        let nodes = (RING_BYTES / CHUNK_BYTES) as u32;
        let last = DESC_ADDRESS + (nodes - 1) * 12;
        unsafe {
            core::ptr::write_volatile((last + 8) as *mut u32, DESC_ADDRESS);
        }
    }
    let _ = buffer;

    // PARLIO 配置（对齐 zephyr parlio_start 全序列——esp-hal
    // with_config/read 不覆盖其中的 bitlen/EOF 模式/超时/FIFO 复位）。
    // 关键：rx_bitlen 复位=0 + EOF 模式复位=bitlen → 数据一到位就 EOF，
    // DMA 传输即终止（实测 1 desc 停）。zephyr：bitlen=RING*8 + EOF=RX_FULL
    // + 超时关。
    let parlio = unsafe { esp32c5::PARL_IO::steal() };
    let pcr = unsafe { esp32c5::PCR::steal() };
    // PCR 级复位（zephyr parlio_ll_reset_register / gdma_ll_reset_register）
    pcr.parl_io_conf().modify(|_, w| {
        w.parl_clk_en().set_bit();
        w.parl_rst_en().set_bit();
        w
    });
    pcr.parl_io_conf().modify(|_, w| {
        w.parl_rst_en().clear_bit();
        w
    });
    // RX 时钟域复位 + FIFO 复位（zephyr rx_reset_clock / rx_reset_fifo）
    pcr.parl_clk_rx_conf().modify(|_, w| {
        w.parl_rx_rst_en().set_bit();
        w
    });
    pcr.parl_clk_rx_conf().modify(|_, w| {
        w.parl_rx_rst_en().clear_bit();
        w
    });
    // 真 40 MHz：纠正 esp-hal off-by-one（其写 div_num=6→34.29 MHz；
    // 正确值 5→240/(5+1)=40 MHz，与 IDF/hal fork/PRUN 实测一致）
    pcr.parl_clk_rx_conf().modify(|_, w| unsafe {
        w.parl_clk_rx_div_num().bits(5);
        w
    });
    parlio.fifo_cfg().modify(|_, w| {
        w.rx_fifo_srst().set_bit();
        w
    });
    parlio.fifo_cfg().modify(|_, w| {
        w.rx_fifo_srst().clear_bit();
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
    // 软分隔符：期望位长 = 环总位（zephyr 同值；字段 19 位装得下）
    parlio.rx_data_cfg().write(|w| unsafe {
        w.rx_bus_wid_sel().bits(3); // 8 位
        w.rx_bitlen().bits((RING_BYTES * 8) as u32);
        w.rx_data_order_inv().bit(false); // LSB 先行
        w
    });
    // bitlen 写入验证：diagprobe 历史 readback 0x70000000 显示 bitlen 区
    // （bits9:27）为零——若仍写不进，分字段单写重试
    let data_cfg_readback = parlio.rx_data_cfg().read().bits();
    defmt::info!(
        "rx_data_cfg readback: 0x{:08x} (bitlen field=0x{:x})",
        data_cfg_readback,
        (data_cfg_readback >> 9) & 0x7_FFFF
    );
    if (data_cfg_readback >> 9) & 0x7_FFFF == 0 {
        parlio
            .rx_data_cfg()
            .modify(|_, w| unsafe { w.rx_bitlen().bits((RING_BYTES * 8) as u32) });
        let second = parlio.rx_data_cfg().read().bits();
        defmt::info!(
            "bitlen retry readback: 0x{:08x} (field=0x{:x})",
            second,
            (second >> 9) & 0x7_FFFF
        );
    }
    // EOF=RX_FULL（软分隔符在 bitlen 处）+ 超时关 + gating 关
    parlio.rx_genrl_cfg().modify(|_, w| {
        w.rx_eof_gen_sel().bit(false);
        w.rx_timeout_en().bit(false);
        w.rx_gating_en().bit(false);
        w
    });
    parlio.rx_mode_cfg().modify(|_, w| unsafe {
        w.rx_smp_mode_sel().bits(2); // 软接收模式
        w
    });
    parlio.reg_update().write(|w| {
        w.rx_reg_update().set_bit();
        w
    });
    let mut spin = 0;
    while spin < 1000 {
        spin += 1;
    }
    // DMA 配置 + 启动（zephyr 顺序：PARLIO 配置完 → DMA armed → 单元启动。
    // 若 DMA 先启动，随后的 PARLIO PCR/FIFO 复位脉冲会扰乱已建立的握手）
    let dma = unsafe { esp32c5::DMA::steal() };
    dma.ch(0).in_conf0().modify(|_, w| {
        w.indscr_burst_en().bit(false);
        w.in_data_burst_en().bit(false);
        w
    });
    dma.ch(0).in_conf1().modify(|_, w| {
        // owner 检查关闭（zephyr/裸路径均 false；esp-hal 默认 true）
        w.in_check_owner().bit(false);
        w
    });
    dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
    dma.ch(0).in_conf0().modify(|_, w| {
        w.in_rst().set_bit();
        w
    });
    dma.ch(0).in_conf0().modify(|_, w| {
        w.in_rst().clear_bit();
        w
    });
    dma.in_link_addr_ch(0)
        .write(|w| unsafe { w.bits(DESC_ADDRESS) });
    dma.ch(0).in_peri_sel().write(|w| unsafe {
        w.peri_in_sel().bits(9); // PARLIO
        w
    });
    dma.ch(0).in_link().modify(|_, w| {
        w.inlink_start().set_bit();
        w
    });

    // 启动顺序（zephyr：DMA 就位 → rx_start → clk_en → sw_en）
    parlio.rx_start_cfg().write(|w| {
        w.rx_start().set_bit();
        w
    });
    pcr.parl_clk_rx_conf().modify(|_, w| {
        w.parl_clk_rx_en().set_bit();
        w
    });
    parlio.rx_mode_cfg().modify(|_, w| {
        w.rx_sw_en().set_bit();
        w
    });

    // 手工构造消费视图（不再使用：view 的顺序扫描与 DMA park 位置失步。
    // 消费改为裸 owner 轮询——owner=CPU 的 desc 有 length 写回即新数据）
    use esp_hal::dma::DmaRxBuffer as _;
    core::mem::forget(buffer.into_view());
    defmt::info!("diagprobe2: manual start complete");

    // 消费侧静态汇（内容不解析，只计流量）
    static mut SINK: [u8; CHUNK_BYTES] = [0; CHUNK_BYTES];
    let _sink = unsafe { &*core::ptr::addr_of!(SINK) };

    const NODES: usize = RING_BYTES / CHUNK_BYTES;
    const DW0_LENGTH_MASK: u32 = 0xFFF << 12;
    const DW0_OWNER_DMA: u32 = 1 << 31;
    let mut consume_head: usize = 0;
    let mut consume_bytes: u64 = 0;

    let started = esp_hal::time::Instant::now();
    let run_for = esp_hal::time::Duration::from_secs(5);
    let mut total: u64 = 0;
    let mut samples = [0u64; 5];
    let mut sample_count = 0usize;
    let mut next_sample = esp_hal::time::Duration::from_secs(1);
    // 停滞检测：预取队列耗尽（in_dscr_empty）后 DMA 暂停，写 inlink_restart
    // 唤醒（IDF gdma append 语义）。阈值 50 次空轮询（~微秒级响应）。
    let mut restart_count: u32 = 0;
    let mut stalled_polls: u32 = 0;
    while started.elapsed() < run_for {
        // 裸 owner 轮询消费：从消费游标起扫 owner=CPU 的 desc
        let mut walked = 0;
        while walked < NODES {
            let desc = DESC_ADDRESS as usize + consume_head * 12;
            let dw0 = unsafe { core::ptr::read_volatile(desc as *const u32) };
            let length = (dw0 & DW0_LENGTH_MASK) >> 12;
            if length == 0 {
                break;
            }
            unsafe {
                // 归还：清 length，owner 置 DMA（环链 next 永连，无需尾插）
                core::ptr::write_volatile(
                    desc as *mut u32,
                    (dw0 & !DW0_LENGTH_MASK) | DW0_OWNER_DMA,
                );
            }
            consume_bytes += u64::from(length);
            consume_head = (consume_head + 1) % NODES;
            walked += 1;
        }
        total = consume_bytes;
        if walked == 0 {
            stalled_polls += 1;
            if stalled_polls >= 50 {
                dma.ch(0).in_link().modify(|_, w| {
                    w.inlink_restart().set_bit();
                    w
                });
                dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
                restart_count += 1;
                stalled_polls = 0;
            }
        } else {
            stalled_polls = 0;
        }
        if started.elapsed() >= next_sample && sample_count < samples.len() {
            samples[sample_count] = total;
            sample_count += 1;
            // dump 写位置（DUMP_MODE bits0-16）：在动 = 引擎持续写 SRAM
            // = 数据源活着（espsoup dump_pos 同款）
            let dump_pos =
                unsafe { core::ptr::read_volatile(0x600a_9008_u32 as *const u32) } & 0x1_FFFF;
            defmt::info!(
                "t={}s: {} bytes (restarts={}, dump_pos=0x{:x})",
                sample_count,
                total,
                restart_count,
                dump_pos
            );
            next_sample += esp_hal::time::Duration::from_secs(1);
        }
    }

    // DMA/PARLIO 状态快照（诊断打印）
    defmt::info!(
        "dma ch0 int_raw: 0x{:08x} state: 0x{:08x}",
        dma.in_int_ch(0).raw().read().bits(),
        dma.ch(0).in_state().read().bits()
    );
    defmt::info!(
        "parlio int_raw: 0x{:08x} st: 0x{:08x}",
        parlio.int_raw().read().bits(),
        parlio.st().read().bits()
    );

    let mbps_x10 = total / 5 / 100_000;
    defmt::info!(
        "FINAL(driver): {} bytes ({}.{} MB/s)",
        total,
        mbps_x10 / 10,
        mbps_x10 % 10
    );

    // 停止：手工停 PARLIO/DMA（view 无 Drop 清理，探针直接收尾）
    parlio.rx_mode_cfg().modify(|_, w| {
        w.rx_sw_en().clear_bit();
        w
    });
    parlio.rx_start_cfg().write(|w| {
        w.rx_start().clear_bit();
        w
    });
    pcr.parl_clk_rx_conf().modify(|_, w| {
        w.parl_clk_rx_en().clear_bit();
        w
    });
    dma.ch(0).in_link().modify(|_, w| {
        w.inlink_stop().set_bit();
        w
    });
    dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
    esp_sdr_mmio::diag_bus_stop(usage_saved);
    defmt::info!("diagprobe2 done");

    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 完整射频 bring-up（C2 验证路径）：STA 启动 + 混杂模式（接收链在线）
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_config(&esp_radio::wifi::Config::Station(Default::default()))
        .unwrap();
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None).unwrap();
    let sniffer = controller.sniffer();
    sniffer.set_promiscuous_mode(true).unwrap();

    let mut radio = Radio::new();
    radio.set_frequency(2412).unwrap();
    radio.set_gain_manual(40).unwrap();
    drop(radio);
    drop(sniffer);
    drop(controller);

    defmt::info!("diagprobe2 ready");
    spawner.spawn(diag_probe_task().unwrap());

    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
    }
}
