# esp-sdr-rs

ESP32-C5 上的 SDR 固件与主机工具，Rust + esp-hal 1.2.2 + esp-rtos 0.4（embassy 异步）从零实现。
寄存器级结论参考 ESPARGOS/esp-sdr、esp-sdr-zephyr、espsoup-firmware（来源与 commit 见
[REFERENCES.md](REFERENCES.md)，逐条出处见 [docs/c5-registers.md](docs/c5-registers.md)），
代码独立编写。

- 目标板：**ESP32-C5HR8 rev v1.2**（16 MB flash / 8 MB PSRAM / 48 MHz 晶振 / 仅 USB-Serial-JTAG）
- 项目状态：**C0–C5 全部检查点完成并上板验证**（2026-10-26；功能基线 commit `48a9458`）
- 归档导读：**[docs/final-report.md](docs/final-report.md)** — 结论、实测数字、根因、未解问题

## 最终结果

| 检查点 | 交付 | 上板实测 |
|---|---|---|
| C0 | workspace、钉死工具链、可复现烧写 | `cargo build --locked` 全绿；heartbeat 日志正常 |
| C1 | PHY 启动 / 调谐 / 增益锁 | radio_check 设频回读与增益锁定正常 |
| C2 | dump engine 突发采集 + 主机频谱 | 12/12 帧 CRC 正确；频谱见真实空中信号（样例 [docs/spectrum-example.csv](docs/spectrum-example.csv)） |
| C3 | rtl_tcp 节拍流（USB 桥路线） | 60 s 连续 78 kS/s IQ8、9.37 MB、零 CRC 错误（WiFi 直连受 C5 SoftAP 上游缺陷阻塞） |
| C4 | 40 MS/s 无缝流：PARLIO→GDMA→box16→CIC→IQ8→USB | 60 s `produced == consumed == 2,400,029,184` 字节（40.0 MB/s）、零跳块；DSP 汇编 3.38 cycles/byte（预算 ≤6.0）；主机 60 s 收流 PASS（max gap 109 ms、seqGaps 0） |
| C5a | DAC 回放引擎（≤16383 字，有限/无尽） | 有限/无尽回放与 RX 恢复全验；HALF=0/1 实测 138.6 / 54.5 MS/s（与标称 80/40 不符，精确速率走 paced） |
| C5b | CPU paced TX + 多相 FIR + QPSK + 半双工 | 400 kS/s QPSK，late 0.33%（130/40000）；每样本 ~329 cycles（上限 ~730 kS/s）；TX→RX 可用 4.9 ms |
| C5c | raw 802.11 Beacon（vendor PHY 编码） | PC `netsh wlan show networks` 扫描到 **ESP-SDR-TX**（BSSID `02:45:53:50:44:52`、信号 94%、802.11g、信道 1、开放） |

## 仓库结构

```
firmware/esp-sdr-mmio    寄存器层：dump engine / DiagStream、DAC 回放（replay）、paced TX
firmware/esp-sdr-radio   射频控制：PHY 启动、调谐、增益锁、DC 校准、TX 链开关
firmware/apps/src/bin    固件入口（清单见下）
shared                   固件与主机共用纯逻辑：帧协议、定点 DSP、TX 整形、802.11 帧
host                     PC 端工具：收帧 + FFT、rtl_tcp 桥、验收客户端
docs                     c5-registers.md / upstream-issues.md / final-report.md
tools                    自举与验收脚本 + vendored espflash
vendor                   进仓库的二进制依赖（librftest.a / libbttestmode.a）
reference                参考仓库克隆区（不入库，脚本重建）
```

## 固件入口（firmware/apps/src/bin）

| 二进制 | 检查点 | 用途 | 状态 |
|---|---|---|---|
| `heartbeat` | C0 | embassy 定时器 + defmt 心跳 | 已验证 |
| `radio_check` | C1 | PHY 启动、调谐、增益锁回读 | 已验证 |
| `capburst` | C2 | dump engine 突发采集 → USB 帧（type 1） | 已验证 |
| `capprobe` | C2 诊断 | 逐步执行采集序列，定位 CPU_LOCKUP | 现场诊断工具 |
| `rxprobe` | C2 诊断 | USB-SJ RX 回显 | 现场诊断工具 |
| `sweepprobe` | C2 诊断 | vendor lane 与 zephyr IQ lane 采样活跃度对照 | 现场诊断工具 |
| `rtltcp` | C3 | AP 模式 + TCP rtl_tcp 服务器 | 保留：受 esp-radio C5 SoftAP 信标缺陷阻塞 |
| `diagprobe2` | C4 诊断 | DMA 间隔 / ISR / CPU 份额 + DSP 等价性与基准 | 已验证（§10.4） |
| `diagstream` | C4 | 无缝流交付：40 MS/s → 78.125 kS/s IQ8（type 2） | 已验证 |
| `txprobe` | C5a | DAC 回放标定、有限/无尽、RX 恢复 | 已验证 |
| `txsym` | C5b | paced QPSK、late 统计、半双工时序 | 已验证 |
| `beacon` | C5c | 裸 DAC 波形 QPSK 信标（三频轮换 5 min） | 外部未收到（见 §13） |
| `cwbeacon` | C5c | vendor CW 载波（三频轮换 5 min） | 诊断辅助 |
| `loopprobe` | C5c 诊断 | duplex 自环 A/B/C 对比 | 采集冻结，结论未定（见 §13） |
| `txwifi` | C5c | raw 802.11 Beacon（`esp_wifi_80211_tx`） | 已验证：PC 扫描到 SSID |

## 主机工具（host/src）

| 二进制 | 检查点 | 用途 |
|---|---|---|
| `esp-sdr-host` | C2 | 串口或抓包文件 → CRC 校验 → 定点 FFT → `spectrum.csv`；`--file:<path>` 解码抓包文件 |
| `rtl_tcp_bridge` | C3 | Rust 桥：capburst type 1 → rtl_tcp（早期实现；最终验收改用 PS 桥） |
| `rtltcp_client` | C3/C4 | rtl_tcp 客户端，统计吞吐、读间隔与连续性（验收用） |

## 脚本（tools）

| 脚本 | 用途 |
|---|---|
| `probe_run.ps1` | 烧写 + 限时监控，defmt 输出写入 `target\probe_monitor.log` |
| `serial_capture.ps1` | C2 连续抓串口到抓包文件 |
| `rtl_tcp_bridge.ps1` | 最终验收桥：USB-SJ → `127.0.0.1:1234`（C# 热路径；打开即发 `G`、16 KB drain 上限） |
| `run_acceptance.ps1` | 一键 60 s 验收（桥 + Rust client） |
| `check_dump_bank.ps1` | 链接 map 断言：固件无段落入 dump bank 1 |
| `gen_tx_fir.ps1` | 生成多相 Blackman-sinc FIR 表（interp 2/4/8，Q15） |
| `fetch_espflash.ps1` | 拉取钉版本 espflash（SHA-256 校验） |
| `fetch_vendor_blobs.ps1` | 拉取 librftest.a / libbttestmode.a（SHA-256 校验） |
| `fetch_references.ps1` | 克隆参考仓库到 `reference/`（不入库） |
| `serial_echo_test.ps1` | USB-SJ 回显诊断 |
| `join_ap.ps1` | 加入板卡开放热点（C3 WiFi 路线辅助） |

## 快速开始

```powershell
# 主机侧测试（default-members = host + shared）
cargo test --locked

# 固件构建（示例：WiFi 信标；换 --bin 即可构建其他入口）
cargo build --locked -p esp-sdr-apps --bin txwifi --release --target riscv32imac-unknown-none-elf

# 烧写 + 监控（COM4 按实际端口调整；flash 与 monitor 分开执行更稳）
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\txwifi
tools\espflash\espflash.exe monitor -S -p COM4 --elf target\riscv32imac-unknown-none-elf\release\txwifi -L defmt --non-interactive
```

主要验收流程：

```powershell
# C2 突发采集 → 频谱 CSV
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\capburst
powershell -ExecutionPolicy Bypass -File tools\serial_capture.ps1 40 target\capture.bin
target\release\esp-sdr-host.exe --file:target\capture.bin 12

# C4 无缝流 60 秒验收（板卡跑 diagstream；SDR++ 可连 127.0.0.1:1234，采样率 78.125 kS/s）
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\diagstream
powershell -ExecutionPolicy Bypass -File tools\run_acceptance.ps1 COM4 60

# C5c raw 802.11 Beacon：烧写后在 PC 侧扫描（发射期间不要开串口监控）
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\txwifi
netsh wlan show networks mode=bssid | findstr /i "ESP-SDR"
```

## 环境自举（去本地化）

- Rust 工具链：`rust-toolchain.toml` 钉死 `1.99.1` + `riscv32imac-unknown-none-elf`，
  任何装有 rustup 的机器首次构建自动安装；无需 nightly、无需 espup（C5 是 RISC-V）。
- 烧写器：`tools/espflash/espflash.exe`（v4.6.0，zip SHA-256 钉死），
  由 `tools/fetch_espflash.ps1` 拉取，可执行文件直接入库。
- vendor blob：`vendor/librftest.a`、`vendor/libbttestmode.a`（来源、commit 与 SHA-256
  见 [vendor/README.md](vendor/README.md)），由 `tools/fetch_vendor_blobs.ps1` 拉取，入库。
- 参考仓库不入库：`tools/fetch_references.ps1` 克隆到 `reference/`（已 gitignore），
  commit 记录在 REFERENCES.md。
- 依赖版本全部由根 Cargo.toml `[workspace.dependencies]` 集中钉定，提交 Cargo.lock，
  构建一律 `--locked`。

## 依赖版本组合（钉死）

esp-hal 1.2.2 + esp-rtos 0.4.0 + esp-radio **1.0.0-beta.1**（三者 2026-09 同期配套发布；
esp-radio 0.18.0 需要 esp-hal ~1.1，与 esp-rtos 0.4 互斥，不可混用）。
PAC 为 esp32c5 0.3.0（与 esp-hal 内部一致，避免类型分裂）。

## Windows USB-Serial/JTAG 实测注意事项

- `flash --monitor` 组合命令在监控接入阶段可能报 `os error 22`，分开执行 `flash` 与
  `monitor` 可稳定工作。
- 发射固件运行期间不要打开串口监控：无人读取时 defmt 会阻塞固件（beacon/txwifi/
  cwbeacon 发射期间零输出即为此）。
- 桥接程序必须**打开串口后立即发 `G`、之后只读**（读后写会永久阻塞）；单次 drain
  不超过 16 KB；8 KB 帧（4096 对）比小帧高帧率稳定。
- 有效吞吐实测约 160–200 kB/s（IQ8 78 kS/s × 2 B），低于 USB 全速理论峰值。

## 已知问题与未解疑点

详见 [docs/final-report.md](docs/final-report.md) §4 与 [docs/upstream-issues.md](docs/upstream-issues.md)。

- **裸 DAC 数据路径（replay / paced）外部始终收不到**，而 vendor PHY 路径（CW、
  raw 802.11）可被外部设备收到——怀疑 DAC 数据路径未完全配置，SDR 波形发射尚不可用。
- `loopprobe` 上下文的 vendor dump 采集冻结（checksum 多次相同，20/80 MS/s 同），
  原因未隔离；C2 时代同一 helper 有活数据。
- libphy PBUS force 通道在本 blob 版本无效（读回影子表、±64 码零响应）；
  当前用软件 DC 补偿，硬件校准代码保留待上游修复。
- esp-radio 1.0.0-beta.1 的 C5 SoftAP 不发射信标（官方示例复现），C3 WiFi 直连路线
  保留待上游修复。
- 回放 HALF 位速率与 zephyr 标称不符（138.6/54.5 vs 80/40 MS/s），精确速率一律走
  paced 路径；HALF=1 测量有 ±6% 抖动。
- TXDC 注入值读回为 0（硬件消费后清），不能作为注入是否生效的旁证。

## 文档索引

- [docs/c5-registers.md](docs/c5-registers.md)：C5 寄存器事实与全部检查点上板实测（§1–§13）
- [docs/upstream-issues.md](docs/upstream-issues.md)：上游问题台账（A1–A3 / B1–B3 / 撤销 C1）
- [docs/final-report.md](docs/final-report.md)：归档总结（结论、数字、根因、复现入口）
- [REFERENCES.md](REFERENCES.md)：参考仓库与固定 commit
- [vendor/README.md](vendor/README.md)：vendor blob 来源与符号验证

## 许可

本项目代码（firmware / host / shared / tools）以 **GPL-3.0-or-later** 发布，全文见
[LICENSE](LICENSE)。选择 GPL 的原因：寄存器序列与 RF 调用路径系统性参考了 GPL-3.0
的 esp-sdr / esp-sdr-zephyr / espsoup-firmware（只读参考，不入库）。

`vendor/` 下的 Espressif 二进制 blob（librftest.a / libbttestmode.a）不属于本项目，
按 ESP-IDF SDK 许可分发；`reference/` 为参考仓库克隆区，不入库。

## 归档信息

- 功能基线：commit `48a9458`（C5c raw 802.11 Beacon 成功）；本仓库结构与文档整理为其后提交。
- 日期：2026-10-26。板卡：ESP32-C5HR8 rev v1.2（仅此一块板验证）。
- 不入库内容：`target/`（构建产物）、`reference/`（可用脚本重建）、`.history/`。
- 打包归档时请排除 `target/`、`reference/`、`.history/`；如需完全匿名，另见
  `docs/final-report.md` 的隐私审计说明。