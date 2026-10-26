# esp-sdr-rs

ESP32-C5 上的开源 SDR 固件与主机工具：40 MS/s 基带采集、片上定点抽取、
USB 连续 IQ 流，以及 DAC 回放 / paced TX / raw 802.11 发射实验。
Rust（`no_std` + esp-hal/esp-rtos + embassy），代码独立实现。

## 当前状态

| 能力 | 状态 |
|---|---|
| RX 无缝流（40 MS/s → 78.125 kS/s IQ8） | **已验证**：60 s 零跳块，主机 60 s 收流通过 |
| rtl_tcp 输出到 SDR++ / GNU Radio | **已验证**：USB 桥 60 s 连续 |
| raw 802.11 Beacon 发射 | **已验证**：PC 扫描到 `ESP-SDR-TX`（信号 94%） |
| DAC 回放 / CPU paced TX | 内部时序已验证；外部接收机尚未确认收到 |
| AP 模式 + TCP rtl_tcp（固件直连） | 受上游 C5 SoftAP 信标缺陷阻塞 |

开发过程、寄存器结论与全部实测数字见 [docs/development-log.md](docs/development-log.md)。

## 特性

- **RX 无缝流**：片内 dump engine 40 MS/s → PARLIO → GDMA 环 → box16 → 三阶 CIC
  → IQ8 → USB-Serial/JTAG，零拷贝数据面、编译期固定缓冲。
- **rtl_tcp**：PC 侧桥把 USB 流转成标准 rtl_tcp，SDR++ / GNU Radio 直连
  `127.0.0.1:1234`。
- **定点 DSP**：自写 CIC / FFT / 多相整形；RISC-V 内联汇编 CIC 实测
  3.38 cycles/byte。
- **TX 实验**：DAC 回放（≤16383 字，有限/无尽）、CPU paced TX（实测上限
  ~730 kS/s）、QPSK 符号流、半双工 TX→RX 切换（4.9 ms）。
- **WiFi 帧发射**：用 `esp_wifi_80211_tx` 发送手工构造的 Beacon，普通 PC
  网卡可直接扫描到；这是目前唯一被外部设备确认收到的发射路径。
- **主机工具**：频谱 CSV、rtl_tcp 桥、连续性验收客户端。

## 硬件与软件要求

| 项目 | 要求 |
|---|---|
| 开发板 | ESP32-C5HR8（实测 rev v1.2）：16 MB flash、8 MB PSRAM、48 MHz 晶振 |
| 主机连接 | USB-Serial/JTAG（板卡仅此一路，无 USB-OTG） |
| 接线 | RX 无需外部接线（dump engine 片内回环）；TX 用板载天线 |
| 主机 | Windows 10/11 + PowerShell；串口以 `COMx` 表示 |
| 工具链 | rustup；`rust-toolchain.toml` 自动安装 Rust 1.99.1 与 RISC-V target |
| 烧写器 | 仓库自带 `tools/espflash/espflash.exe`（v4.6.0） |

TX 仅限当地合法频段与功率；示例使用 2.4 GHz 信道 1。

## 快速开始

### 1. 获取与准备

```powershell
git clone https://github.com/chenhlSH/esp-sdr-rs.git
cd esp-sdr-rs

# 主机侧单元测试（host + shared）
cargo test --locked

# 校验 vendor blob（文件已入库，脚本做 SHA-256 校验）
powershell -ExecutionPolicy Bypass -File tools\fetch_vendor_blobs.ps1
```

### 2. 构建与烧写

```powershell
cargo build --locked -p esp-sdr-apps --bin diagstream --release --target riscv32imac-unknown-none-elf
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\diagstream
```

把 `COM4` 换成实际端口。`flash` 与 `monitor` 建议分开执行（见「已知限制」）。

### 3. 接收：连续 IQ 流到 SDR++

实时使用：只启动桥，然后让 SDR++ 连接 `127.0.0.1:1234`，采样率
**78.125 kS/s**，格式 8 bit IQ。

```powershell
powershell -ExecutionPolicy Bypass -File tools\rtl_tcp_bridge.ps1 COM4 1234
```

60 秒自动验收（桥 + 客户端，会占用 1234 端口）：

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_acceptance.ps1 COM4 60
```

### 4. 频谱快照

```powershell
cargo build --locked -p esp-sdr-apps --bin capburst --release --target riscv32imac-unknown-none-elf
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\capburst

cargo build --release
powershell -ExecutionPolicy Bypass -File tools\serial_capture.ps1 40 target\capture.bin COM4
target\release\esp-sdr-host.exe --file:target\capture.bin 12
```

输出 `spectrum.csv`（频偏 Hz、功率 dB）。样例见
[docs/spectrum-example.csv](docs/spectrum-example.csv)。

### 5. 发射：WiFi Beacon

```powershell
cargo build --locked -p esp-sdr-apps --bin txwifi --release --target riscv32imac-unknown-none-elf
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\txwifi

# 不要打开串口监控；等待约 5-10 秒后在 PC 上扫描
netsh wlan show networks mode=bssid | findstr /i "ESP-SDR"
```

`txwifi` 每 100 ms 发一个 Beacon，持续 10 分钟，SSID 为 `ESP-SDR-TX`，
信道 1（2412 MHz），开放网络。

## 数据通路

```
RX:  RF → dump engine (40 MS/s) → PARLIO → GDMA 环 → box16 → CIC(32)
        → IQ8 环 → USB-Serial/JTAG → PC 桥 → rtl_tcp → SDR++

TX:  shared::tx 符号/波形 → CPU paced TXDC 注入（实测上限 ~730 kS/s）
     或 DAC 回放引擎（≤16383 字）
     或 vendor WiFi raw 802.11 帧（当前唯一外部确认可收的发射路径）
```

## 仓库结构

```
firmware/esp-sdr-mmio   寄存器层：dump engine / DiagStream、DAC replay、paced TX
firmware/esp-sdr-radio  射频控制：PHY 启动、调谐、增益锁、DC 校准、TX 链
firmware/apps/src/bin   固件入口
shared                  帧协议、定点 DSP、TX 整形、802.11 帧（固件与主机共用）
host                    PC 工具：频谱、rtl_tcp 桥、验收客户端
tools                   自举/验收脚本 + vendored espflash
docs                    开发记录、寄存器文档、上游问题台账
vendor                  Espressif 二进制 blob（librftest.a / libbttestmode.a）
```

## 可用固件

| 固件 | 用途 |
|---|---|
| `diagstream` | 主 RX 固件：40 MS/s → 78.125 kS/s IQ8 连续流 |
| `capburst` | 突发采集 + type 1 帧（配合频谱工具） |
| `txwifi` | raw 802.11 Beacon（PC 可扫描） |
| `txprobe` | DAC 回放标定、有限/无尽回放、RX 恢复 |
| `txsym` | paced QPSK 符号流 + 半双工时序 |
| `rtltcp` | AP + TCP rtl_tcp 固件路线（受上游缺陷阻塞） |

其余 `heartbeat`、`radio_check`、`capprobe`、`rxprobe`、`sweepprobe`、
`diagprobe2`、`beacon`、`cwbeacon`、`loopprobe` 为验证/诊断入口。
完整清单与实测状态见 [docs/development-log.md](docs/development-log.md)。

## 主机工具与脚本

| 工具 | 用途 |
|---|---|
| `esp-sdr-host` | 串口/抓包文件 → CRC → FFT → `spectrum.csv` |
| `rtl_tcp_bridge` | Rust 桥（capburst type 1 → rtl_tcp） |
| `rtltcp_client` | rtl_tcp 连续性验收客户端 |
| `tools\rtl_tcp_bridge.ps1` | 推荐桥（diagstream type 2，C# 热路径） |
| `tools\run_acceptance.ps1` | 一键 60 s 验收 |
| `tools\serial_capture.ps1` | 串口抓包到文件（`<秒> <输出> [COM口]`） |
| `tools\probe_run.ps1` | 烧写 + 限时 defmt 日志 |

## 已知限制

- **裸 DAC 波形路径外部不可见**：DAC 回放与 paced TX 的片上时序已验证，但外部
  接收机收不到；目前只有 vendor PHY 的 raw 802.11 路径被 PC 确认收到。
- **输出速率 78.125 kS/s**：受 USB-Serial/JTAG 有效吞吐（约 160–200 kB/s）限制。
- **C5 SoftAP 信标缺陷**：esp-radio 1.0.0-beta.1 下 SoftAP 不发射信标，`rtltcp`
  固件路线待上游修复。
- **Windows USB-SJ 注意事项**：`flash` 与 `monitor` 分开执行；发射期间不要打开
  串口监控（defmt 会阻塞固件）；桥必须“打开即发 `G`、之后只读”。
- **板卡版本**：只在 ESP32-C5HR8 rev v1.2 上验证；其他硅修订可能需要调整。
- **发射合规**：仅使用当地合法频段与功率。

更多细节（含每一项的实测数字与排查过程）见
[docs/development-log.md](docs/development-log.md)。

## 文档

- [docs/development-log.md](docs/development-log.md)：检查点过程、实测数据、排查记录
- [docs/c5-registers.md](docs/c5-registers.md)：C5 寄存器与调用序列（含出处）
- [docs/upstream-issues.md](docs/upstream-issues.md)：上游问题台账
- [docs/final-report.md](docs/final-report.md)：项目结论、根因与未解问题
- [REFERENCES.md](REFERENCES.md)：参考仓库与固定 commit

## 许可

代码以 **GPL-3.0-or-later** 发布，见 [LICENSE](LICENSE)。选择 GPL 是因为寄存器
序列与 RF 调用路径系统性参考了 GPL-3.0 的 esp-sdr / esp-sdr-zephyr /
espsoup-firmware（只读参考，不入库）。

`vendor/` 下的 Espressif 二进制 blob（librftest.a / libbttestmode.a）按 ESP-IDF
SDK 许可分发，不属于本项目许可范围。