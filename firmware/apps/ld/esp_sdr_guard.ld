/* C2：SRAM 块 1（0x40820000..0x40840000）整体让渡给 WiFi dump engine。
 * 固件静态内存必须在块 1 之下结束；启动栈锚定在 RAM 顶端 0x4084E5A0（块 2），
 * 其运行时范围由固件断言（esp_sdr_mmio::assert_stack_clear_of_dump_bank）。
 * 任务栈全部来自堆，堆在 .dram2_uninit（dram2_seg，块 2）。 */
ASSERT(_data_end <= 0x40820000, "C2: .data crosses dump bank 1")
ASSERT(_bss_end <= 0x40820000, "C2: .bss crosses dump bank 1")
ASSERT(_noinit_end <= 0x40820000, "C2: .noinit crosses dump bank 1")
