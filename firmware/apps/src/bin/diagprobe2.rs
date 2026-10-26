//! C4 验证探针：三嫌疑裁定 + DSP 汇编等价性 + 性能基准。
//!
//! 阶段：
//! A. ① DMA 描述符完成间隔直方图（1 秒窗口，轮询差分 + rdcycle）；
//! B. ③ GDMA 中断触发观测（ISR 计数，回答 C4 早期"ISR 未触发"悬案）；
//! C. ② CPU 份额对照：信道 1（繁忙 2.4G）vs 信道 36（清静 5G）；
//! D. DSP 汇编等价性自检（scalar vs asm 逐位）；
//! E. cycles/byte 基准（两路径）。
//!
//! 结论入档 docs/c5-registers.md §10.4。

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_sdr_mmio::DiagStream;
use esp_sdr_radio::Radio;
use esp_sdr_shared::dsp::{Cic, DIAG_BOX};

esp_bootloader_esp_idf::esp_app_desc!();

const CONSUME_CHUNK: usize = 4096;
const BENCH_BATCHES: usize = 4000;
/// 描述符大小（间隔直方图按它折算完成数）。
const NODE_BYTES: u64 = 3584;

#[inline(always)]
fn cycle_now() -> u32 {
    let value: u32;
    unsafe { core::arch::asm!("rdcycle {}", out(reg) value, options(nostack)) };
    value
}

/// 纯 CPU 份额指数：1 秒（240M cycles）内 tight loop 迭代数。
/// rdcycle 调用+比较约 10 cycles/迭代 → 无干扰时约 24M。
fn cpu_share_1s() -> u32 {
    let start = cycle_now();
    let mut iterations = 0u32;
    while cycle_now().wrapping_sub(start) < 240_000_000 {
        iterations = iterations.wrapping_add(1);
    }
    iterations
}

/// GDMA in_ch0 中断观测 ISR（③ 裁定）：只计数 + 清中断，不动 head
/// （head 仍由 poll 推进——观测不影响主逻辑）。
static ISR_FIRED: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
extern "C" fn diagprobe_isr_counter() {
    let dma = unsafe { esp32c5::DMA::steal() };
    dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
    ISR_FIRED.fetch_add(1, Ordering::Relaxed);
}

/// scalar 与 asm 在给定字节块上的输出逐位对比；返回是否一致（并打印首差）。
fn equivalence_check(bytes: &[u8], rate: u32) -> bool {
    let mut scalar = Cic::new(rate);
    let mut asm = Cic::new(rate);
    let mut out_scalar = [0i32; 512];
    let mut out_asm = [0i32; 512];
    let produced_scalar = scalar.run(bytes, &mut out_scalar);
    let produced_asm = asm.run_asm(bytes, &mut out_asm);
    if produced_scalar != produced_asm {
        defmt::error!(
            "R={}: produced scalar={} asm={}",
            rate,
            produced_scalar,
            produced_asm
        );
        return false;
    }
    for index in 0..2 * produced_scalar {
        if out_scalar[index] != out_asm[index] {
            defmt::error!(
                "R={}: out[{}] scalar={} asm={}",
                rate,
                index,
                out_scalar[index],
                out_asm[index]
            );
            return false;
        }
    }
    // 状态也必须逐位一致（后续块行为依赖它）
    for index in 0..15u32 {
        let offset = index * 4;
        let sv = unsafe {
            core::ptr::read_volatile(
                (&scalar as *const Cic as *const u8).add(offset as usize) as *const u32,
            )
        };
        let av = unsafe {
            core::ptr::read_volatile(
                (&asm as *const Cic as *const u8).add(offset as usize) as *const u32,
            )
        };
        if sv != av {
            defmt::error!("R={}: state offset {} scalar=0x{:08x} asm=0x{:08x}", rate, offset, sv, av);
            return false;
        }
    }
    true
}

#[embassy_executor::task]
async fn diag_probe_task(
    mut controller: esp_radio::wifi::WifiController<'static>,
    mut radio: Radio,
) {
    defmt::info!("diagprobe2: three-suspect verdicts + asm equivalence + bench");
    let mut stream = DiagStream::start(0);
    let mut acc = [0i32; 2 * (CONSUME_CHUNK / DIAG_BOX / 8 + 4)];

    // 预热：等环里积攒数据（每轮必须 poll 才能推进生产游标）
    let mut consumed_offset = 0usize;
    let mut scratch = [0u8; CONSUME_CHUNK];
    let mut warm_batches = 0;
    while warm_batches < 200 {
        stream.poll_producer();
        let available = stream.available_at(consumed_offset);
        if available >= CONSUME_CHUNK {
            consumed_offset = (consumed_offset + CONSUME_CHUNK) % esp_sdr_mmio::RING_BYTES;
            warm_batches += 1;
        }
    }

    // ---- A) 嫌疑①：DMA 描述符完成间隔直方图（1 秒窗口）----
    // 轮询差分 + rdcycle 时间戳。期望（40 MB/s）：3584 B / 89.6 µs
    // = 21504 cycles@240MHz，min/max 应接近（连续无停顿）。
    {
        let mut last_bytes = stream.producer_bytes();
        let mut last_cycle = cycle_now();
        let start = last_cycle;
        let mut count: u32 = 0;
        let mut min_gap: u32 = u32::MAX;
        let mut max_gap: u32 = 0;
        let mut sum_gap: u64 = 0;
        while cycle_now().wrapping_sub(start) < 240_000_000 {
            let bytes = stream.poll_producer();
            if bytes != last_bytes {
                let now = cycle_now();
                let descriptors = (bytes - last_bytes) / NODE_BYTES;
                if descriptors == 1 {
                    let gap = now.wrapping_sub(last_cycle);
                    count += 1;
                    min_gap = min_gap.min(gap);
                    max_gap = max_gap.max(gap);
                    sum_gap += u64::from(gap);
                }
                last_bytes = bytes;
                last_cycle = now;
            }
        }
        let average = if count > 0 { sum_gap / u64::from(count) } else { 0 };
        // 期望间隔 = NODE_BYTES / (40 MB/s) × 240 MHz = 21504 cycles
        defmt::info!(
            "① DMA interval: n={} avg={} min={} max={} (expect 21504 cycles@40MB/s)",
            count,
            average,
            min_gap,
            max_gap
        );
    }

    // ---- B) 嫌疑③：GDMA 中断触发观测（1 秒）----
    // 修正实验：除 CPU 侧 bind+enable 外，还需开 DMA 控制器侧 ena
    // （in_int_ch(0).ena）；且观察期内不调 poll（poll 的 clr 会压制
    // pending）。期望 ~11161 次/秒（40 MB/s / 3584 B）。
    {
        ISR_FIRED.store(0, Ordering::Relaxed);
        let dma = unsafe { esp32c5::DMA::steal() };
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
        dma.in_int_ch(0).ena().modify(|_, w| unsafe {
            // in_done（bit1）+ in_suc_eof（bit0）+ 错误位；排除保留位
            w.bits(u32::MAX & !(1 << 5))
        });
        let handler = esp_hal::interrupt::InterruptHandler::new(
            diagprobe_isr_counter,
            esp_hal::interrupt::Priority::Priority1,
        );
        esp_hal::interrupt::bind_handler(esp32c5::Interrupt::DMA_IN_CH0, handler);
        esp_hal::interrupt::enable(
            esp32c5::Interrupt::DMA_IN_CH0,
            esp_hal::interrupt::Priority::Priority1,
        );
        // 观察期：不 poll（让中断自由运行；环被未归还描述符堵死前
        // 足够产生大量中断——堵死后 poll 的兜底扫描会自愈）
        let start = cycle_now();
        while cycle_now().wrapping_sub(start) < 240_000_000 {
            core::hint::spin_loop();
        }
        let fired = ISR_FIRED.load(Ordering::Relaxed);
        // 关闭 ena 恢复轮询模式
        dma.in_int_ch(0).ena().write(|w| unsafe { w.bits(0) });
        dma.in_int_ch(0).clr().write(|w| unsafe { w.bits(u32::MAX) });
        defmt::info!("③ ISR fired in 1s: {} (expect ~11161)", fired);
    }

    // ---- C) 嫌疑②：CPU 份额对照（信道 1 繁忙 vs 信道 14 静默，均 2.4G）----
    {
        let busy = cpu_share_1s();
        defmt::info!("② busy(ch1)={} iterations", busy);
        // ch14（2484 MHz，中国/日本专用、实际几乎无人使用）；5G 切换
        // 实测有死锁风险故不采用
        match controller.set_channel(14, esp_radio::wifi::SecondaryChannel::None) {
            Ok(()) => {
                let _ = radio.set_frequency(2484);
                let quiet = cpu_share_1s();
                defmt::info!(
                    "② CPU share: ch1(busy)={} ch14(quiet)={} (ratio {}%)",
                    busy,
                    quiet,
                    quiet * 100 / busy.max(1)
                );
                let _ = controller.set_channel(1, esp_radio::wifi::SecondaryChannel::None);
                let _ = radio.set_frequency(2412);
            }
            Err(_error) => {
                defmt::warn!("set_channel(14) failed; fallback: sniffer off");
                let sniffer = controller.sniffer();
                let _ = sniffer.set_promiscuous_mode(false);
                let quiet = cpu_share_1s();
                defmt::info!(
                    "② CPU share: sniffer on {} vs off {} (ratio {}%)",
                    busy,
                    quiet,
                    quiet * 100 / busy.max(1)
                );
                let _ = sniffer.set_promiscuous_mode(true);
            }
        }
    }

    // 1) 等价性自检：三块真实数据 × 两个抽取率（含 settle 消耗后状态）
    let mut equal_all = true;
    for round in 0..3u32 {
        while stream.available_at(consumed_offset) < CONSUME_CHUNK {
            stream.poll_producer();
        }
        let bytes = stream.ring_slice_at(consumed_offset, CONSUME_CHUNK);
        scratch.copy_from_slice(bytes);
        // 前两轮 R=16（生产参数），第三轮 R=8
        let rate = if round < 2 { 16 } else { 8 };
        if equivalence_check(&scratch, rate) {
            defmt::info!("equivalence round {} R={}: EQUAL", round, rate);
        } else {
            equal_all = false;
        }
        consumed_offset = (consumed_offset + CONSUME_CHUNK) % esp_sdr_mmio::RING_BYTES;
    }
    // 跨 phase 边界：不同初始相位也对（两个新实例相位都从 0 开始，
    // 连续喂第二块即覆盖非零相位状态）——上轮已隐含（同实例连续调用）？
    // equivalence_check 每次新实例——补一轮同实例二次喂入。
    {
        let mut scalar = Cic::new(16);
        let mut asm = Cic::new(16);
        let mut out_a = [0i32; 512];
        let mut out_b = [0i32; 512];
        let mut mismatch = false;
        for _ in 0..3 {
            while stream.available_at(consumed_offset) < CONSUME_CHUNK {
                stream.poll_producer();
            }
            let bytes = stream.ring_slice_at(consumed_offset, CONSUME_CHUNK);
            scratch.copy_from_slice(bytes);
            let pa = scalar.run(&scratch, &mut out_a);
            let pb = asm.run_asm(&scratch, &mut out_b);
            if pa != pb || out_a[..2 * pa] != out_b[..2 * pb] {
                mismatch = true;
                break;
            }
            consumed_offset = (consumed_offset + CONSUME_CHUNK) % esp_sdr_mmio::RING_BYTES;
        }
        if mismatch {
            equal_all = false;
            defmt::error!("chained equivalence: MISMATCH");
        } else {
            defmt::info!("chained equivalence (same state, 3 chunks): EQUAL");
        }
    }
    defmt::info!(
        "EQUIVALENCE: {}",
        if equal_all { "PASS" } else { "FAIL" }
    );

    // 2) cycles/byte：两路径各 4000 批（交替，环境一致）
    let mut cic_scalar = Cic::new(16);
    let mut cic_asm = Cic::new(16);
    let mut scalar_cycles: u64 = 0;
    let mut asm_cycles: u64 = 0;
    let mut batches = 0usize;
    defmt::info!(
        "bench start: offset={} produced={}",
        consumed_offset,
        stream.producer_bytes()
    );
    while batches < BENCH_BATCHES {
        let mut spin_guard: u32 = 0;
        while stream.available_at(consumed_offset) < CONSUME_CHUNK {
            stream.poll_producer();
            spin_guard += 1;
            if spin_guard > 2_000_000 {
                defmt::error!(
                    "spin stuck: offset={} produced={} available={}",
                    consumed_offset,
                    stream.producer_bytes(),
                    stream.available_at(consumed_offset)
                );
                // 描述符快照（DMA 停在哪、owner 分布）+ DMA 状态
                let dma = unsafe { esp32c5::DMA::steal() };
                defmt::error!(
                    "dma: int_raw=0x{:08x} state=0x{:08x} link_addr=0x{:08x}",
                    dma.in_int_ch(0).raw().read().bits(),
                    dma.ch(0).in_state().read().bits(),
                    dma.in_link_addr_ch(0).read().bits()
                );
                for index in 0..16 {
                    let desc = 0x4082_E000u32 + index * 12;
                    let dw0 = unsafe { core::ptr::read_volatile(desc as *const u32) };
                    if dw0 != 0x8000_0E00 {
                        defmt::error!(
                            "desc[{}] dw0=0x{:08x} next=0x{:08x}",
                            index,
                            dw0,
                            unsafe {
                                core::ptr::read_volatile((desc + 8) as *const u32)
                            }
                        );
                    }
                }
                defmt::error!("(仅打印非 0x80000E00 常态的描述符)");
                break;
            }
        }
        let bytes = stream.ring_slice_at(consumed_offset, CONSUME_CHUNK);
        scratch.copy_from_slice(bytes);
        let t0 = cycle_now();
        let _ = cic_scalar.run(&scratch, &mut acc);
        let t1 = cycle_now();
        let _ = cic_asm.run_asm(&scratch, &mut acc);
        let t2 = cycle_now();
        scalar_cycles += u64::from(t1.wrapping_sub(t0));
        asm_cycles += u64::from(t2.wrapping_sub(t1));
        consumed_offset = (consumed_offset + CONSUME_CHUNK) % esp_sdr_mmio::RING_BYTES;
        batches += 1;
        if batches <= 4 || batches % 500 == 0 {
            defmt::info!(
                "batch {}: scalar={} asm={} cycles",
                batches,
                t1.wrapping_sub(t0),
                t2.wrapping_sub(t1)
            );
        }
    }
    let count = batches as u64;
    let scalar_per_byte = scalar_cycles / count / CONSUME_CHUNK as u64;
    let asm_per_byte_x100 = asm_cycles * 100 / count / CONSUME_CHUNK as u64;
    defmt::info!(
        "CYCLES/BYTE: scalar={} asm={}.{} (40MB/s needs scalar {}MHz asm {}MHz)",
        scalar_per_byte,
        asm_per_byte_x100 / 100,
        asm_per_byte_x100 % 100,
        scalar_per_byte * 40,
        asm_per_byte_x100 * 40 / 100
    );

    stream.stop();
    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    // 显式 240 MHz（esp-hal C5 默认 80 MHz）
    let config = esp_hal::Config::default()
        .with_cpu_clock(esp_hal::clock::CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

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
    drop(sniffer);

    defmt::info!("diagprobe2 ready");
    // controller/radio 保留给嫌疑②的信道切换
    spawner.spawn(diag_probe_task(controller, radio).unwrap());

    loop {
        Timer::after(embassy_time::Duration::from_secs(60)).await;
    }
}