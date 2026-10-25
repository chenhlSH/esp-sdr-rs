# ESP32-C5 SDR 寄存器与调用序列（C0 抄录）

来源：一手 = 本地 `reference/esp-sdr`（commit `f86da81807b7f6e850742f6b789d9468c190ad41`）源码逐行核实；
二手 = esp-sdr-zephyr（commit `58db280a11c0c12760064af647794c25d81fa967`）`doc/hardware-quirks.md`。
未在本文件出现的"通用"结论不得直接采信，实现前回源码复核。

## 1. SRAM 让渡与链接断言

- 捕获内存：`0x40830000`（capture buffer 起始，`targets/esp32c5/chip.h`）
- 让渡区间：`0x40820000 .. 0x40840000`（整个块 1，128 KiB，`chip.h` 中
  `SOC_RESERVE_MEMORY_REGION`；C5 的属主粒度 = 整个 128 KiB bank）
- 链接脚本断言（`targets/esp32c5/sram_guard.ld`，C2 落地为 Rust 链接脚本）：
  - `_iram_end <= 0x40820000`
  - `_data_end <= 0x40820000`
  - `_bss_end <= 0x40820000`
- SRAM 属主寄存器：`0x60095004`；采集时写 `(owner & ~0xf00) | 0x10200`
  （bits 11:8 = 2 + bit 16 = MAC_DUMP_ALLOC），结束后恢复原值
  （`families/c5_c6_c61/receiver.c` acquire_iq 原始 probe 路径）

## 2. RX 突发采集

### 2.1 上游主线（stock 路径，vendor 函数封装）

```c
adctrig(n - 1, 0, 0, divider * 2, 0, 0, 0, 0, 0);   // n ≤ 16380，9 个 uint32 参数
```

- `adctrig` 在 vendor/librftest.a 中（已 nm 验证）；符号在 `chip.h` 声明
- divider 取值 0..5（速率档，见 §5）；`BURST_MAX_RATE = 5`（C5，receiver.c）

### 2.2 原始寄存器路径（receiver.c 的 C5 SAMPLE_RATE_PROBE 分支，揭示 stock 函数内部行为）

1. 先用 stock 参数播种一次：`adctrig(255, 0, 0, clock*2, 0,0,0,0,0)`
2. 源选择：`0x600a9008` bits 20:17 ← `(源选择号 - 1) << 17`（vendor 选择号 1 = 原始源 15；
   掩码 `~0x001e0000`；读改写）
3. SRAM 属主：`0x60095004` ← `(owner & ~0xf00) | 0x10200`
4. dump 控制 `0x600a9004`：`ctrl = 0x80000000 | n`（bit 31 = 使能，低位 = 计数）；
   写 `ctrl` → 写 `ctrl | (1 << 19)`（bit 19 触发脉冲）→ 写回 `ctrl`
5. 轮询完成：`(REG_READ(0x600a9004) & (1 << 22))`，超时 20 ms
   —— **C5 的 DONE 位是 bit 22**（通用文档写 bit 18，不适用于 C5）
6. 收尾：`0x600a9004 ← 0`，恢复 `0x600a9008` 原值

### 2.3 校验（receiver.c acquire_iq）

- 采集前：buffer[0..n) 全部填充哨兵 `0xa5a0055a`；buffer[n..n+4) 填守卫 `0x5a5aa5a5 ^ j`
- 采集后：
  - 任一 buffer 字仍等于 `0xa5a0055a` → 采集未完成（capture_timeout）
  - 任一守卫字被改写 → 过跑（capture_overrun）

## 3. RX 链路准备序列（receiver.c prepare_rx，每次换频后执行）

```c
rx_recalibrate(frequency_mhz);                    // 按需重校准（rx_recalibration.c）
phy_chip_set_chan(frequency_mhz, rx_channel_mode); // = phy_set_chanfreq(mhz, mode)，C5 直接收 MHz
phy_stop_tx_tone(1);
phy_pbus_workmode();
phy_pbus_xpd_tx_off();
phy_pbus_xpd_rx_on(1);
phy_set_rxclk_en(1);
// 可选：phy_rx_filter_mode(filter)（C6 不用）
gain_apply();                                     // 增益锁定（§4）
```

半双工恢复（zephyr 二手）：TX 后调 `phy_pbus_xpd_rx_on()` + `phy_set_rxclk_en()` 恢复 RX。

## 4. 增益控制（common/burst_gain.h）

- C5：`force_rx_gain(manual, gain_code, 0)`——**三个 unsigned 参数**（librftest.a）
- 手动增益上限：`(REG_READ(0x600a702c) >> 8) & 127`（AGCPWR_CTRL7 bits 8..14，
  调谐时由 PHY 填入最大已校准低表索引）；C5 上限：`< 90` 才有效，否则视为 0
- GAIN_MIN = 0（非 H2）；增益码是 PHY 增益索引，不是 dB
- 状态位：`(REG_READ(0x600a702c) >> 23) & 1`
- zephyr 二手：WiFi AGC 会周期性夺回强制增益，需约 20 ms 重锁一次

## 5. 调谐与速率

- `phy_set_chanfreq(mhz, mode)`（libphy.a）：C5 直接收 MHz；软件限幅
  `RX_FREQ_MIN=100 .. RX_FREQ_MAX=6000`（common/rx_tuning.h）
  （zephyr 二手：任意 MHz > 14 可调，5 GHz 自 4500 起，<1100 死区）
- `mode` 参数来自带宽选择（rx_bandwidth.h 的 `rx_bandwidth_phy_mode`，C2 复核）
- C5 速率表（LIMITS，common/burst_limits.h，与 zephyr 一致）：
  `80000000, 40000000, 20000000, 10000000, 8000000, 4000000`（即 80/40/20/10/8/4 MS/s，
  divider 0..5，adctrig 第 4 参数 = divider*2）

## 6. WiFi/射频启动序列（receiver.c app_main，上游实证路径）

```c
nvs_flash_init();
esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT());
esp_wifi_set_storage(WIFI_STORAGE_RAM);
esp_wifi_set_mode(WIFI_MODE_NULL);      // 空模式：只点亮射频，不关联
esp_wifi_start();
esp_wifi_set_ps(WIFI_PS_NONE);
esp_wifi_set_promiscuous(true);
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
prepare_rx();
```

→ 上游用 esp_wifi 空模式做射频 bring-up。Rust 侧对应 esp-radio 的
未关联 controller（或 esp-phy 最小路径），C1 检查点实测后二选一。

## 7. TX（二手：esp-sdr-zephyr doc/hardware-quirks.md，C5 检查点回源码复核）

- DAC 回放引擎：`0x600a900c`，≤16383 字（I bits 9:0 / Q bits 19:10），
  80 MS/s；bit 17 = 40 MS/s；repeat 计数 bits 27:20 + LOOP 位无缝回放；
  重启已结束的 pass 会留下 35–37 样本空洞（~0.45 µs）
- DAC 输出刻度：`0x600a0c04` bits 23:8（每支路一字节，库设 15）；
  幅度×刻度 >1000–1500 起进入模拟压缩区
- CPU 节拍 TX：`0x600a0c08` 单字存储（store 15 周期；轮询 ~1.35 MS/s；预打包单 store 循环 8 MS/s）
- TX 前需 vendor CW 先打开发射链；半双工：TX 期间 RX 冻结（§3 恢复）

## 8. C2 实测结论（rev v1.2 本板，含与参考项目的差异）

- **采集必须走 vendor adctrig**（librftest.a + libbttestmode.a，esp-phy-lib
  commit 20f1db05）。手写寄存器序列（zephyr 路径）在 esp_wifi 启动后的运行
  环境里，写 SRAM 属主寄存器会触发 CPU 锁死——让渡瞬间有未知上下文访问块 1；
  裸机早期同写法可恢复（ROM 打印 RV_STK_EPC）。vendor 代码内部规避了该问题。
- **librftest.a 的 shell 依赖 libbttestmode.a**（ble_rx_start / bt_testmode_init /
  esp_ble_rx_func / zb_rx 所在），该库 Rust 生态与 esp-phy-lib 文档均未提及。
- **混杂模式必须开启**：未连接 STA 的接收链会下电，ADC 输出冻结为常量
  （dump 总线保持末值，全帧同一字）。esp_radio sniffer 的
  set_promiscuous_mode(true) 使接收链持续在线后样本恢复正常。
  对齐上游 app_main 的 esp_wifi_set_promiscuous(true)——此前认为"用不到"是误判。
- **DONE 位**：bit 18（对齐 zephyr）；上游 probe 用的 bit 22 在本板也能观测到
  置位，但以 bit 18 + 哨兵校验为准。
- **I/Q 位序**：维持上游文档口径（I bits 9:0，Q bits 19:10）；zephyr 主张相反。
  C4 换 DIAG 总线路径时以实测裁定。
- **采集耗时**：16380 字实测 ~636 µs（vendor adctrig 含内部校准开销），
  高于理论 205 µs（80 MS/s）约 3 倍；引擎写速率实测 ~340 Mwords/s（原始寄存器
  路径探针），速率字段在 rev v1.2 上的实际效果待 C4 一并厘清。
- **AGCPWR_CTRL7 bit 23**：手动增益锁定后仍读 0，不能当"已强制"指示。
- **SRAM 属主寄存器**（0x60095004）：复位值 0；位域与 PAC 一致
  （SRAM_USAGE bits 11:8 一热 + MAC_DUMP_ALLOC bit 16）。

## 9. USB-Serial/JTAG 主机侧语义（Windows 实测）

- **DTR = IO0（BOOT 引脚）、RTS = 复位**（espflash connection/reset.rs 同结论）。
  DTR=true 会进 ROM 下载监视器（打印 ESP-ROM 横幅后卡死等同步）。
- 数据流与 DTR/RTS 电平无关；espflash reset_after_flash 的 USB-SJ 分支
  （DTR=false + RTS 脉冲）是安全的复位进运行模式序列，host 工具已照抄。
- **设备侧 TX 在无人读取时会卡死端点**：固件侧用有界非阻塞发送
  （write_byte_nb + 500 ms 预算，超时丢帧下周期重试）规避。
- **serialport-rs 的 read 与 USB-SJ CDC 存在卡死兼容问题**（两案复现，
  bridge 会话 5 s 后无数据）；.NET SerialPort 读同链路完全可靠——
  桥的热路径用 Add-Type C#（含 .NET SerialPort）实现。
- 主机读取用 .NET SerialPort 或 serialport 均可；抓包诊断工具：
  tools/serial_capture.ps1（连续抓 → 文件）、tools/serial_echo_test.ps1（回显）、
  tools/probe_run.ps1（烧写+限时监控）。

## 10. C4 中期实测（进行中，截至本 commit）

已验证通路：
- **DSP 全套（主机单测 13/13 绿）**：box16（nibble 半边和）→ 2.5 MS/s；
  三阶 CIC（R=8..255，I/Q 双路 u32 回绕积分 + 输出率梳状）；
  cic_inverse/normalize（2^(32+lsb)/R³ 定点倒数归一，量纲=10bit×16）。
- **dump engine 自由运行序列**（无尽计数 0x1FFFF + 速率清零 + 触发后
  usage 立即复原）：触发后诊断总线有数据。
- **GPIO 矩阵回环**（8 pad×（IO_MUX mcu_sel=0 + 矩阵出 MODEM_DIAG0+lane
  + 入 PARL_RX_DATA0+k））+ **PARLIO 40 MHz 采样**（PLL_F240M÷6、8 位、
  LSB、软分隔符 rx_bitlen=环总位长、EOF=RX_FULL）+ **GDMA AHB ch0
  （触发 9）环形 16×3584B 描述符**：首轮实测 desc[0..2] 完成且 length
  正确写回（0xE00=3584），证明 PARLIO→GDMA→SRAM 数据通路连通。

当前卡点（顺序排查中）：
1. **生产持续 ~270µs（2-3 个描述符）后停**：PARLIO int_raw=RX_FIFO_WOVF
   （bit1，FIFO 写溢出）+ GDMA int_raw=in_done|in_dscr_empty。DMA 不再
   搬运，PARLIO 溢出。已排除：中断状态粘滞（轮询清状态无效）、通道
   优先级/加权仲裁/inlink_auto_ret（组合实验无效）、触发号错误
   （SOC_GDMA_TRIG_PERIPH_PARLIO0=9 已核对）。
2. **ISR 绑定未生效**：esp_hal::interrupt bind_handler+enable(DMA_IN_CH0,
   P1) 后 ISR 从未执行（ISR_BYTES 恒 0，尽管裸轮询同期能看到完成的
   描述符）。怀疑与 esp-rtos 向量/优先级交互，未定位。

下一步候选（按优先级）：
- 换 esp-hal 官方 GDMA 驱动路径（ParlIoRx + DmaLoopBuf/DmaRxStreamBuf）
  替代裸寄存器——驱动内处理了未文档化时序；
- 逐项 diff hal_espressif(54d64a1) 的 ahb_dma_ll.h 与本实现的寄存器
  序列（尤其 reset/burst/rsp 相关）；
- 向 ESPsoup/esp-sdr-zephyr 社区提问（C5 GDMA 环停走的已知坑）。

- **C5 SoftAP 在 esp-radio 1.0.0-beta.1 上不发射信标**：官方未改动的
  embassy_access_point 示例（同 tag、同工具链）在本板同样扫不到
  （PC AX200 全段扫描）。与 Espressif esp-idf#19267（C5 SoftAP 信标
  问题）呼应。固件侧 "rtltcp" bin（AP+TCP）代码保留，待上游修复后重测。
- **混杂模式与 AP 冲突**：同控制器开 sniffer 会卡死网络栈（wait_config_up
  不返回）；AP 模式本身让接收链持续在线，无需混杂模式。
- **AP 模式下禁用 prepare_receive**：其 phy_pbus_xpd_tx_off 会停发信标
  （AP 的信标是 TX）。AP 的接收增益由 PHY AGC 管理，不做手动锁。
- **交付路线（C3 验收通过）**：capburst（USB 流，10 Hz 突发）→ PC 桥
  （tools/rtl_tcp_bridge.ps1，C# 热路径）→ 本机 rtl_tcp 服务端 →
  SDR++ 直连 127.0.0.1:1234。60 秒验收：9.37 MB、78 kS/s IQ8、
  最大间隔 246 ms、零 CRC 错误。
- USB-Serial/JTAG 有效吞吐实测 ~160-200 kB/s（IQ8 78 kS/s × 2 B），
  低于理想 330 kB/s 峰值（突发间隙+USB 全速协议开销）。

