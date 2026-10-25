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

## 8. 待实测确认（C2 检查点）

- divider → 速率的精确映射（LIMITS 表 6 档 vs adctrig divider*2 编码）
- `rx_recalibration.c` 的重校准触发条件与移植必要性
- 带宽 mode 参数取值（rx_bandwidth.h）
- AGCPWR_CTRL7 bit 23 的语义（C1 实测：手动增益 40 锁定后仍读 0；
  上游从未标注该位，不能当作"已强制"指示）
