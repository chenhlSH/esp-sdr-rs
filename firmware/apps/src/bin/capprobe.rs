//! C2 诊断探针：逐步执行 dump engine 采集序列，每步打日志，
//! 用于定位 CPU_LOCKUP 的确切步骤。定位完成后保留作诊断工具。

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_radio::Radio;

esp_bootloader_esp_idf::esp_app_desc!();

// 与 esp-sdr-mmio hardware 模块相同的寄存器定义（探针专用副本，逐日志步进）
const DUMP_CTRL_REG: u32 = 0x600a_9004;
const DUMP_CTRL_RUN: u32 = 1 << 31;
const DUMP_CTRL_TRIGGER: u32 = 1 << 19;
const DUMP_CTRL_DONE: u32 = 1 << 18;
const DUMP_CTRL_COUNT_MASK: u32 = 0x0001_FFFF;
const DUMP_MODE_REG: u32 = 0x600a_9008;
const DUMP_MODE_SOURCE_MASK: u32 = 0xF << 17;
const DUMP_MODE_SOURCE_IQ: u32 = 15 << 17;
const DUMP_MODE_RATE_MASK: u32 = 0x7 << 21;
const DUMP_CONFIG_REG: u32 = 0x600a_9018;
const DUMP_CONFIG_LANES_MASK: u32 = 0x01FF_FFFF;
const DUMP_CONFIG_IQ: u32 = 24 | (25 << 6) | (26 << 12) | (27 << 18) | (1 << 24);
const DUMP_CLK_FORCE_REG: u32 = 0x600a_9c04;
const DUMP_CLK_EN_REG: u32 = 0x600a_0800;
const DUMP_CLK_EN_BIT: u32 = 1 << 2;
const DUMP_USAGE_REG: u32 = 0x6009_5004;
const DUMP_USAGE_FIELD_MASK: u32 = 0xF << 8;
const DUMP_USAGE_BANK1: u32 = 1 << 8;
const DUMP_USAGE_ALLOC: u32 = 1 << 16;
const CAPTURE_BANK_PTR: *mut u32 = 0x4083_0000 as *mut u32;

fn rd(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn wr(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn read_stack_pointer() -> u32 {
    let stack_pointer: u32;
    unsafe {
        core::arch::asm!(
            "mv {sp}, sp",
            sp = out(reg) stack_pointer,
            options(nomem, nostack, preserves_flags)
        );
    }
    stack_pointer
}

fn read_cycle_count() -> u64 {
    let low: u32;
    let high: u32;
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

const COUNT: usize = 16380;

#[embassy_executor::task]
async fn probe_task() {
    loop {
        Timer::after(Duration::from_secs(10)).await;
        defmt::info!("probe: begin");

        let sp = read_stack_pointer();
        defmt::info!("step 1 sp=0x{:08x}", sp);
        let cycles = read_cycle_count();
        defmt::info!("step 2 cycles=0x{:08x}", cycles as u32);

        // 哨兵与守卫预填（volatile：即将被硬件接管的内存，存储不可被重排）
        let bank = unsafe { core::slice::from_raw_parts_mut(CAPTURE_BANK_PTR, COUNT + 4) };
        for (index, word) in bank[0..COUNT].iter_mut().enumerate() {
            unsafe { core::ptr::write_volatile(word, 0xa5a0_055a ^ index as u32) };
        }
        for (offset, word) in bank[COUNT..].iter_mut().enumerate() {
            unsafe { core::ptr::write_volatile(word, 0x5a5a_a5a5 ^ offset as u32) };
        }
        defmt::info!("step 3 sentinel+guard filled");

        // step 10 探针：本硅版（rev v1.2）上引擎写块 1 无需让渡（实测 16379/16380 字
        // 在 usage=0 时落地）。本探针完全不动 SRAM_USAGE，验证最简协议。
        let usage_saved = rd(DUMP_USAGE_REG);
        defmt::info!("step 4 usage_saved=0x{:08x} (probe: never written)", usage_saved);

        wr(DUMP_CTRL_REG, 0);
        defmt::info!("step 5 ctrl=0 done");

        wr(DUMP_CLK_FORCE_REG, u32::MAX);
        defmt::info!("step 6 clk_force written");

        let clk_en = rd(DUMP_CLK_EN_REG);
        wr(DUMP_CLK_EN_REG, clk_en | DUMP_CLK_EN_BIT);
        defmt::info!("step 7 clk_en 0x{:08x} -> 0x{:08x}", clk_en, clk_en | DUMP_CLK_EN_BIT);

        let lane = rd(DUMP_CONFIG_REG);
        wr(DUMP_CONFIG_REG, (lane & !DUMP_CONFIG_LANES_MASK) | DUMP_CONFIG_IQ);
        defmt::info!("step 8 lanes 0x{:08x} -> 0x{:08x}", lane, (lane & !DUMP_CONFIG_LANES_MASK) | DUMP_CONFIG_IQ);

        let mode = rd(DUMP_MODE_REG);
        wr(DUMP_MODE_REG, (mode & !DUMP_MODE_SOURCE_MASK) | DUMP_MODE_SOURCE_IQ);
        defmt::info!("step 9 source 0x{:08x} -> 0x{:08x}", mode, (mode & !DUMP_MODE_SOURCE_MASK) | DUMP_MODE_SOURCE_IQ);

        // step 10（微步隔离）：write / fence / readback / irq-enable 各打一条日志
        let mstatus_saved: u32;
        unsafe {
            core::arch::asm!("csrr {ms}, mstatus", ms = out(reg) mstatus_saved, options(nomem, nostack));
            core::arch::asm!("csrci mstatus, 8", options(nomem, nostack)); // 清 MIE
        }
        wr(DUMP_USAGE_REG, (usage_saved & !DUMP_USAGE_FIELD_MASK) | DUMP_USAGE_BANK1 | DUMP_USAGE_ALLOC);
        defmt::info!("step 10.1 write done");
        unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };
        defmt::info!("step 10.2 fence done");
        let _flush = rd(DUMP_USAGE_REG);
        defmt::info!("step 10.3 readback done = 0x{:08x}", _flush);
        unsafe {
            core::arch::asm!("csrs mstatus, {ms}", ms = in(reg) mstatus_saved & 8, options(nomem, nostack));
        }
        defmt::info!("step 10.4 irq back on");
        defmt::info!("step 10 granted (irq-off window)");

        let mode2 = rd(DUMP_MODE_REG);
        let rate_value = (mode2 & !DUMP_MODE_RATE_MASK) | ((0u32 << 21) & DUMP_MODE_RATE_MASK);
        wr(DUMP_MODE_REG, rate_value);
        defmt::info!("step 11 rate 0x{:08x} -> 0x{:08x}", mode2, rate_value);

        let count_field = (COUNT as u32) & DUMP_CTRL_COUNT_MASK;
        wr(DUMP_CTRL_REG, count_field | DUMP_CTRL_DONE);
        wr(DUMP_CTRL_REG, DUMP_CTRL_RUN | count_field);
        wr(DUMP_CTRL_REG, DUMP_CTRL_RUN | count_field | DUMP_CTRL_TRIGGER);
        wr(DUMP_CTRL_REG, DUMP_CTRL_RUN | count_field);
        defmt::info!("step 12 triggered");

        let cycles_start = read_cycle_count();
        let mut done = false;
        loop {
            if rd(DUMP_CTRL_REG) & DUMP_CTRL_DONE != 0 {
                done = true;
                break;
            }
            if (read_cycle_count().wrapping_sub(cycles_start) / 240_000_000) as u32 >= 20_000 {
                break;
            }
        }
        let raw_cycles = read_cycle_count().wrapping_sub(cycles_start);
        defmt::info!("step 13 done={} raw_cycles={}", done, raw_cycles);

        // step 14：停引擎 + reclaim（关中断窗口）
        unsafe {
            core::arch::asm!("csrci mstatus, 8", options(nomem, nostack));
        }
        wr(DUMP_CTRL_REG, 0);
        wr(DUMP_USAGE_REG, usage_saved);
        unsafe { core::arch::asm!("fence rw, rw", options(nostack)) };
        let _flush = rd(DUMP_USAGE_REG);
        unsafe {
            core::arch::asm!("csrs mstatus, {ms}", ms = in(reg) mstatus_saved & 8, options(nomem, nostack));
        }
        defmt::info!("step 14 stopped+reclaimed (irq-off window)");

        // 逐字校验：统计未写索引（哨兵残留下标）与过跑
        let mut sentinel_left = 0usize;
        let mut first_unwritten: Option<usize> = None;
        for index in 0..COUNT {
            let word = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(index)) };
            if word == 0xa5a0_055a ^ index as u32 {
                sentinel_left += 1;
                if first_unwritten.is_none() {
                    first_unwritten = Some(index);
                }
            }
        }
        let guard_ok = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(COUNT)) } == 0x5a5a_a5a5
            && unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(COUNT + 1)) } == 0x5a5a_a5a5 ^ 1
            && unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(COUNT + 2)) } == 0x5a5a_a5a5 ^ 2
            && unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(COUNT + 3)) } == 0x5a5a_a5a5 ^ 3;
        let word0 = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR) };
        let word1 = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(1)) };
        let word2 = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(2)) };
        let word3 = unsafe { core::ptr::read_volatile(CAPTURE_BANK_PTR.add(3)) };
        defmt::info!(
            "step 15 verify: sentinel_left={}, first_unwritten={:?}, guard_ok={}",
            sentinel_left, first_unwritten, guard_ok
        );
        defmt::info!(
            "step 16 words[0..4]=0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}",
            word0, word1, word2, word3
        );
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // 隔离实验 A：esp_wifi 介入之前裸写让渡寄存器（判定写语义 vs 状态依赖）
    defmt::info!("bare write test: begin");
    let usage_addr = 0x6009_5004u32;
    let before = unsafe { core::ptr::read_volatile(usage_addr as *const u32) };
    unsafe { core::ptr::write_volatile(usage_addr as *mut u32, 0x0000_0200) };
    let after = unsafe { core::ptr::read_volatile(usage_addr as *const u32) };
    unsafe { core::ptr::write_volatile(usage_addr as *mut u32, before) };
    let restored = unsafe { core::ptr::read_volatile(usage_addr as *const u32) };
    defmt::info!("bare write test: before=0x{:08x} after=0x{:08x} restored=0x{:08x}", before, after, restored);

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 完整 bring-up（对齐上游 esp_wifi_start 路径）：
    // WifiController::new = esp_wifi_init；set_config 触发 esp_wifi_start（MAC 启动）；
    // set_channel 对应 esp_wifi_set_channel。STA 不连接，仅点亮 MAC。
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_config(&esp_radio::wifi::Config::Station(Default::default()))
        .unwrap();
    controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None).unwrap();
    controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None).unwrap();

    let mut radio = Radio::new();
    radio.set_frequency(2412).unwrap();
    radio.set_gain_manual(40).unwrap();
    defmt::info!("probe ready: wifi started, tuned, gain locked");
    drop(radio);
    drop(controller);

    spawner.spawn(probe_task().unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
