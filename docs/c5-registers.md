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
- **GPIO 矩阵回环** + **PARLIO 40 MHz 采样**（PLL_F240M÷6、8 位、LSB、
  软分隔符 rx_bitlen=环总位长、EOF=RX_FULL）+ **GDMA AHB ch0（触发 9）
  环形 16×3584B 描述符**：实测 desc[0..2] 完成且 length 正确写回
  （0xE00=3584），证明 PARLIO→GDMA→SRAM 数据通路连通。

当前卡点：**生产持续 2-3 个描述符（~180-270µs）后停**。已排除：
owner 不归还（已加 zephyr 同款归还）、总线复位脉冲、通道优先级/
加权仲裁/inlink_auto_ret、中断状态粘滞。每轮实测有随机性
（3584/7168/10752 字节后停）。

### 10.1 espsoup-firmware 源码考古结论（第二个独立实现，活答案）

读毕 `main/families/c5_c6_c61/parlio_cap.h` 与 `c5_engine.h`（GPL-3.0，
只读参考）：

1. **ESPsoup 的 PARLIO 走 IDF 官方驱动**（`driver/parlio_rx.h`：
   `parlio_new_rx_unit`（dma_burst_size=32, trans_queue_depth=2,
   max_recv_size=环长）+ `parlio_new_rx_soft_delimiter`（eof_data_len=
   环长）+ `parlio_rx_unit_receive`（partial_rx_en=true））——**非硬件
   环形链**；IDF 驱动内部管理 GDMA 重建链。**硬件环形链只有 zephyr 一家
   在用（且工作）**。
2. **owner bit16 = 半区选择**（0=下半区 0x40820000，1=上半区
   0x40830000；espsoup 实测语义）——zephyr 命名 MAC_DUMP_ALLOC 是
   同一位的另一理解。写位置 = 半区基址 + 4×(position mod 16384)。
3. **DUMP_MODE bits 0-16 = 实时写位置**（可无中断跟踪生产进度）。
4. **0x600a20b4 = source-15 门控**：bit0 置位则源被 gate；访问一次
   停总线 0.82ms；espsoup 仅 boot 检查。**本板（rev v1.2）实测：读并
   清零后反而完全无数据（0 字节）**——gate 语义在 rev v1.2 上可能相反
   或该读操作有副作用，已回滚（不碰此寄存器）。vendor adctrig 每次
   采集 RMW 它（stock 路径 1.7ms 开销来源）。
5. **I/Q nibble 布局与 zephyr 相反**：espsoup 实测 byte =
   I[lsb+3:lsb]（低 nibble）| Q[lsb+3:lsb]<<4（高 nibble）；MODEM_DIAG
   [0:9]=I、[10:19]=Q（10bit 有符号）。zephyr 主张 Q 低 I 高。真值待
   C4 通流后以已知信号裁定（box16 已按可交换设计）。
6. **lsb 默认 6**（I[9:6]/Q[9:6] 高位窗），非 0；AUTO 用 2×16KB 快照
   99.9% 幅度点选窗；VFM 模式用中值×2.2（抗突发）。
7. **WiFi 混杂过滤器运行期清零**（pe_quiet_wifi，想法源自 C5VRX）：
   驱动任务收帧会抢占消费任务足够久导致环溢出（繁忙信道）。
8. **每 run 前 DC 校准**（dc_cal）：DC 会吃掉 4bit 窗（实测 30-43
   ADC8 counts @5GHz/gain60）。
9. espsoup box16 输出直接打包回 dump 32 位字格式：
   w[j] = (4*si-480)&1023 | ((4*sq-480)&1023)<<10。
10. dump_arm 顺序：CTRL=0 → 速率 → 时钟强制 → 计数预写（无 RUN）→
    lanes → owner 让渡 → fence → 源 15 → go（RUN+trigger 脉冲）。
    与 zephyr 差异：源 15 在让渡后。**实测采用 zephyr 顺序（让渡前），
    两者均可产数据。**
11. DMA 侧无额外魔法：GDMA 全部经 IDF gdma 驱动（申请通道、默认
    内存范围、owner check off、RX_DONE 中断）。

### 10.2 option A 实录（esp-hal 驱动路径，16 轮实验，2026-10-25 破案）

**结论先行：C4 无缝流数据面已打通**——diagprobe2 实测 **40.0 MB/s 持续
5 秒（200 MB）零重启**，与 espsoup PRUN 完全对齐。根因两个，均与
"环形链纯软件回环"的假设有关：

**根因 1（命脉）：rx_bitlen 软分隔符 EOF 是环形无限流的引擎。**
正确机制 = `rx_bitlen = 环总位长（bits9:27，19 位，单位 bit）` +
`rx_eof_gen_sel = 0（RX_FULL：bitlen 处生成 EOF）` +
`inlink_auto_ret = 1（IN_LINK 复位默认 0x11 的 bit0，EOF 后 GDMA
自动回链头重启）`。每圈 PARLIO 生成 EOF → GDMA 收 suc_eof 结束当圈
→ auto_ret 回 desc[0] 开下一圈。**纯软件环（无 EOF 让 DMA 沿 next
无限走）在本板（rev v1.2）不成立**：DMA 预取队列耗尽后停（表现为
随机生产 0-4 个描述符后 in_dscr_err/in_dscr_empty 粘滞死，
inlink_restart 无效）。zephyr（rev1.0 板）的环与之表面相同，但其
bitlen 写入成功故有 EOF 引擎；我们此前 rx_data_cfg 的 bitlen 一直
没写进去（readback bits9:27=0）——**根因 2**。
IDF 驱动（espsoup PCAP/PRUN）的 partial_rx_en + eof_data_len=ring
正是同一机制（ISR 只推进软件游标，不碰 DMA 寄存器）。

**根因 2（陷阱）：rx_data_cfg 的 bitlen 写入依赖时序。** bitlen
写不进并非值越界（458752 < 19 位上限 524287），而是**写入时序**：
必须在 PCR rx 时钟域复位脉冲 + FIFO 复位 + `parl_clk_rx_en=0` 之后
写 rx_data_cfg，再 reg_update（对齐 zephyr parlio_start 顺序）。
满足时序后同一 svd2rust 写法 readback bitlen=0x70000 ✓。

**esp-hal 1.2.2 在 C5 上的三个缺口（该版本官方流式路径实际不可用；
详细上报口径见 docs/upstream-issues.md，其中原"缺口 4"经复核撤销）：
1. `ParlIoRx::read` → `do_prepare` 强制 `indscr_burst_en(true)`，
   软件链（DmaRxStreamBuf 链尾 next=NULL）启动即报 in_dscr_err
   （C5 的 AHB-GDMA 把 NULL next 判为描述符错误，不是"链尾暂停"）。
   inlink_restart 救不回（错误态需 in_rst 通道复位）。
2. `ahb_v2 setup()` 的 DMA 可访问内存窗口配置整段被注释（TODO），
   `INTR_MEM_START/END_ADDR` PAC 标注复位 0/0。注意：B 轮实测未写
   窗口时 DMA 仍能完成描述符（硅默认行为与 PAC 复位值矛盾，另行
   核对）；IDF/zephyr 每次初始化都显式写 0x40800000/0x44000000。
3. `apply_clock_divider` off-by-one：直分频语义写 div_num=source/freq，
   C5 实际语义 integer-1（hal fork parlio_ll）。请求 40 MHz 得
   div_num=6 → 实际 34.29 MB/s（实测裁定）；要 40 MHz 须 div_num=5
   （PAC 直改——请求 48 MHz 会撞 esp-hal 上限检查）。
（撤销的"缺口 4"：rx_timeout_en 其实由 RxConfig::default 的
timeout_ticks=None 经 apply_config 正确关闭，只影响裸路径。）

**espsoup 0.7.1 预编译固件（rev1.2 板实测，烧写三件套
reference/espsoup-bin/）**：PDIAG 2000 读 → MODEM_DIAG[19:0] 每信号
~1000 翻转/占空比 ~50%（总线活跃铁证）；PCAP 4096/32768/65532 全成功
（1/9/17 nodes）；**PRUN 8 秒 40.0 MB/s、overrun 1 次、时钟偏差
ppm=1.5、guard 0/18432（dump 引擎不踩 CPU 拥有的块 1）**。espsoup
的连续流 = PCAP 式快照 + PRUN 无限环（IDF partial 模式），另外它每
run 前做 DC 校准、run 期间清 WiFi 混杂过滤器（防驱动任务抢占过跑）。

**16 轮实验时间线**（负结果同样入档）：A read() 直接调 →
DescriptorError panic（burst+NULL）。B 加诊断 → armed 但 0 字节、
int=0x9。C 补窗口寄存器 → read 内部检查抓 dscr_err。D 手工启动+闭环
+关 burst → 14336 后 empty。E 闭环+view.pop+重连 → 57344 卡死（view
顺序扫描与 DMA park 失步）。F 顺序链（尾 NULL）→ 0 字节+dscr_err
（NULL=错误实锤）。G 裸 owner 消费 → 7168。H 关 check_owner →
dscr_err 消失但吞吐被 dump 日志污染。I 去日志 → 10752 死。J 关超时 →
3584 死。K 补 zephyr 全序列（bitlen/EOF/FIFO/PCR 复位）→ 0 死
（bitlen 没写进）。L 顺序重排 → 10752 死。M dump_pos 监测 → 源活着。
N espsoup 烧板裁判（PDIAG/PCAP/PRUN）→ 硅无罪。O bitlen readback
验证+写入成功 → **34.3 MB/s 零重启**。P div_num 直改 5 → **40.0 MB/s**。

**对 diag.rs 裸路径的回灌（C4 后续）**：diag.rs 序列与时序要求一致，
但需验证其 bitlen readback（diagprobe 旧轮显示 0——同为时序受害者）；
start() 里补 intr_mem 窗口写入；确认 auto_ret 未被显式清零。

### 10.3 C4 管线落地实录（回灌后，60 秒上板验证）

- **mmio 回灌**：DiagStream::start 重写为验证过的 zephyr 顺序（PARLIO
  配置在前 → GDMA armed → 启动三连）+ bitlen 写入断言（时序不满足时
  响亮崩溃）。回灌踩两坑（如实入档）：(a) 时钟源 sel 须在 en=true
  状态下建立（对齐 esp-hal PeripheralGuard 行为），en=false 下写入
  实测不生效 → 0 字节；(b) 裸路径必须显式开 PCR `gdma_conf.gdma_clk_en`
  （手工路径由 esp-hal PeripheralGuard 代办，缺它 DMA 寄存器写入
  全部无效——诊断特征：conf/link/addr 读回全零）。
- **diagstream 交付 bin**（最终形态双任务版）：lsb 自动选窗（burst
  快照直方图 + robust 中值，3 次重试抗 async USB 中断插入 adctrig
  临界窗的 CaptureOverrun）→ WiFi 混杂过滤器清零（FFI
  `esp_wifi_set_promiscuous_filter`）→ 无缝流（40 MHz 视界，终态）
  → box16+CIC(R=32，汇编路径) → IQ8 流帧（type 2，512 对/帧，CRC
  覆盖头+载荷）→ embassy Channel 交接 → usb_task async 中断驱动写出。
  终验见 §10.4：40.0 MB/s produced==consumed（精确相等）60 秒。
- **DC 校准**（espsoup dc_cal：PBUS debugmode + Newton 步进）暂缓：
  DC 仅占窗 ~1/16，lsb 留裕量即可；正式版前再上。

### 10.4 消费性能完整裁定（2026-10-26 终局：80MHz 误设 + 汇编 + 死锁 + 三嫌疑）

**根因一（决定性）：CPU 时钟默认 80 MHz。** esp-hal C5 的 `CpuClock`
枚举 `#[default] _80MHz`，C0 起全部固件用 `esp_hal::Config::default()`
从未显式设置——所有"消费钉死"现象（8.5 MB/s、"43.5% 比率"、rdcycle
时间账）在 80 MHz 下完全自洽：Rust 标量 CIC 8.27 c/B × 80 MHz ≈
9.7 MB/s 上限。**修复：`with_cpu_clock(CpuClock::_240MHz)`。**
修复后 20 MB/s 视界验收：produced == consumed == 1,200,002,048 字节
（精确相等）、零 skipped。教训：性能排查第一步核时钟配置，显式优于
隐式——委托默认值会以"物理现象"面貌掩盖配置疏漏。

**根因二（鲁棒性）：确认游标落后一圈的 DMA 死锁。** 轮询确认落后
DMA 写入整整一圈时：DMA 停在 head 身后"已完成未归还"的描述符上
（快照证据：DMA 物理游标 desc[2] owner=0、软件 head=12 length=0），
head 又在等 DMA 写入前方格子——循环等待永久僵死（bench 实测复现，
大停顿消费场景）。**修复：poll_producer 在 head 处无新数据时全环
兜底扫描**，归还任何已完成格（语义：确认落后一圈 ⇔ 生产超前消费
一圈 ⇔ 过跑，归还/覆盖符合丢最旧约定）；`orphaned_reclaimed` 计数
观测。生产 40 MB/s 60 秒 orphaned=0——正常节拍零动用，纯防御。

**DSP 内联汇编**（shared/src/dsp.rs `run_asm` + global_asm）：
- **3.38 cycles/byte**（板上 rdcycle，4000 批完全确定性；scalar 8.27
  c/B，zephyr C 4.1-4.4 参考对照——寄存器全程驻留 + 单循环体省去
  迭代器/边界开销）；
- 等价性自检 PASS×4（实环数据 R=16/16/8 + 同状态链式 3 块，输出与
  15 字状态逐位一致）——汇编路径的板上金标准验证；
- 40 MB/s 全消费 = 135 MHz = **CPU 240MHz 的 56%**，预算内。
- 实现要点：`#[repr(C)]` Cic 状态字节布局（偏移表在源码注释）；
  s4-s9 六积分器全程驻留；梳状每 R 次触达内存；注释限 ASCII
  （LLVM 汇编器拒绝中文注释）。

**40 MHz 视界终验**（diagstream 双任务 + asm + 兜底扫描 + 240MHz）：
**produced == consumed == 2,400,029,184 字节（精确相等）= 40.0 MB/s**、
skipped=0、dropped 仅 USB 侧（无主机）、orphaned=0，60 秒零错误。

**三嫌疑裁定**（diagprobe2，全部实测数据）：
1. **DMA 供数占空比——否定**。描述符完成间隔直方图（1 秒窗口轮询
   差分 + rdcycle）：n=11160/s、avg=21503 cycles（理论 21504，
   3584 B @40MB/s = 89.6 µs）、min/max=21318/21767（±1% 抖动）。
   DMA 连续均匀，无停顿。
2. **WiFi/调度 CPU 抢占——否定**。CPU 份额指数（1 秒 tight loop
   迭代数）：sniffer on 34,285,592 vs off 34,285,714，
   **ratio 100.0004%**——混杂模式帧处理 CPU 占用实测为零。
   （ch14 切换被 esp-radio 拒绝（WifiError）；5G ch36 切换实测有
   死锁风险，弃用——单纯为对照实验不值得。）
3. **ISR 驱动生产推进——结案（机制可用，非必需）**。GDMA ISR 实测
   触发 11161 次/秒（期望 11161，分毫不差）。**完整触发条件 = CPU 侧
   `bind_handler` + `interrupt::enable` + DMA 控制器侧
   `in_int_ch(0).ena`（缺一不可）+ 观察期无 poll 的 clr 压制**——
   C4 早期"ISR bind_handler 未触发"悬案以此结案（当年紧轮询的
   `in_int_ch(0).clr()` 把 pending 清掉了）。当前生产路径保持
   轮询+兜底（已验证 40 MB/s + 自愈）；ISR 路径为可选加固，
   条件已明确写入本节。

**性能预算定稿**：40 MB/s 视界全消费 = asm 56% CPU + poll/framing/
USB ~15% ≈ 71%，实测富余（60 秒零跳块零孤儿）。若未来上 80 MS/s
视界（需要 2× 消费 = 112%），则需要双核（C5 单核）/更宽 SIMD/
或 espsoup 式低算力消费设计。

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

