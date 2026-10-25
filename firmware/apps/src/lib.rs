//! 固件 app 共享的初始化逻辑（供各 bin 复用）。

#![no_std]

/// 堆布局：块 2 的 dram2_seg（0x4084E5A0..0x4085E5A0，64 KiB，二阶段 bootloader
/// 加载器遗留区）放主体；.bss 内 24 KiB 作余量。esp-rtos 任务栈从堆分配，
/// 因此全部任务栈物理上避开块 1（dump bank，0x40820000..0x40840000）。
///
/// 链接脚本断言（ld/esp_sdr_guard.ld）保证静态内存不越过块 1 下沿。
pub fn init_heap() {
    #[unsafe(link_section = ".dram2_uninit")]
    static mut HEAP_DRAM2: core::mem::MaybeUninit<[u8; 65_536]> = core::mem::MaybeUninit::uninit();
    static mut HEAP_INTERNAL: core::mem::MaybeUninit<[u8; 24_576]> =
        core::mem::MaybeUninit::uninit();
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            core::ptr::addr_of_mut!(HEAP_DRAM2).cast::<u8>(),
            65_536,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            core::ptr::addr_of_mut!(HEAP_INTERNAL).cast::<u8>(),
            24_576,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}
