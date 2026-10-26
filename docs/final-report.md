# esp-sdr-rs 归档总结

- 日期：2026-10-26
- 功能基线：commit `48a9458`（C5c raw 802.11 Beacon 成功）
- 板卡：ESP32-C5HR8 **rev v1.2**（16 MB flash / 8 MB PSRAM / 48 MHz 晶振 / 仅 USB-Serial-JTAG）
- 工具链：Rust 1.99.1（`rust-toolchain.toml` 钉死）、espflash 4.6.0（vendored）
- 依赖：esp-hal 1.2.2 + esp-rtos 0.4.0 + esp-radio 1.0.0-beta.1 + PAC esp32c5 0.3.0

## 1. 结论

项目按 C0–C5 六个检查点完成，全部有上板实测：

- **RX 数据面打通**：dump engine → PARLIO 40 MS/s → GDMA 环 → 定点抽取 →
  IQ8 → USB-Serial/JTAG，60 秒零跳块、零孤儿描述符；主机侧 rtl_tcp 桥
  60 秒连续收流通过。
- **TX 数据面部分打通**：DAC 回放引擎、CPU paced TX、多相 FIR、QPSK 符号流
  与半双工切换均已上板验证；但裸 DAC 波形路径外部不可见（见 §4）。
- **WiFi 协议发射打通**：用 vendor PHY 的 raw 802.11 TX 周期发送 Beacon，
  本机 PC 扫描到 `ESP-SDR-TX`（信号 94%）。这是当前唯一被外部设备确认收到
  的发射路径。
- **工程目标达成**：可复现构建（`--locked`、零绝对路径）、零拷贝数据面、
  编译期固定缓冲、DSP 自写定点（内联汇编 3.38 cycles/byte）。

## 2. 实测数字与预算对照

| 指标 | 预算 / 目标 | 实测 | 结论 |
|---|---|---|---|
| 捕获输入带宽 | 40 MB/s（8 bit @ 40 MS/s） | **40.0 MB/s**，60 s `produced == consumed == 2,400,029,184 B` | 达标（精确相等，零跳块） |
| DSP 抽取开销 | ≤ 6.0 cycles/byte | **3.38 c/B**（汇编）/ 8.27 c/B（标量） | 达标（汇编） |
| CPU 占用（40 MB/s 全消费） | 有余量 | 汇编 56% + poll/组帧/USB ~15% ≈ **71%** | 达标 |
| USB 输出 | ~250 kS/s（早期预算） | **78.125 kS/s IQ8**（≈156 kB/s）；USB-SJ 实测上限 160–200 kB/s | 预算偏乐观：USB-SJ 实际只够 ~80–100 kS/s IQ8，当前 78.125 kS/s 留有余量 |
| 主机 60 s 连续性 | 无断流 | 最大读间隔 **109 ms**、seqGaps **0**、9.36 MB | 达标 |
| paced TX 每样本开销 | — | **~329 cycles** → 上限 **~730 kS/s** | 400 kS/s 实测 late **0.33%** |
| 半双工 TX→RX | 可用即可 | tx_chain_off 2.66 ms + 首采 2.26 ms = **4.9 ms** | 达标 |
| DAC 回放速率 | 标称 80/40 MS/s | HALF=0/1 实测 **138.6 / 54.5 MS/s**（±1–6%） | 与标称不符；精确速率一律走 paced |
| WiFi Beacon | 被 PC 扫描到 | SSID `ESP-SDR-TX`、**信号 94%**、802.11g、信道 1 | 达标 |

## 3. 关键根因（按影响排序）

1. **CPU 默认 80 MHz**：esp-hal C5 的 `CpuClock` 默认 `_80MHz`，C0 起所有固件
   从未显式设置，导致长期误判为"消费钉死"。显式 `with_cpu_clock(_240MHz)`
   后 40 MB/s 全消费成立。教训：性能排查第一步核时钟配置。
2. **`rx_bitlen` 是无限流引擎的软分隔符**：bitlen = 环总位数 + EOF 语义
   （RX_FULL + inlink_auto_ret 回链头）；rev v1.2 上纯软件环不成立。写入还
   依赖时序：PCR RX 域复位 + FIFO 复位 + 时钟关闭后写，再 `reg_update`。
3. **确认游标落后一圈的 DMA 死锁**：DMA park 在未归还描述符、head 等前方写入。
   兜底全环扫描（`orphaned_reclaimed`，正常为 0）自愈；40 MB/s 终验 orphaned=0。
4. **USB-Serial/JTAG 主机侧三坑**：读后写永久阻塞（打开即发 `G`、之后只读）；
   单次 256 KB drain 造成 1.7 s 断流（上限 16 KB）；小帧高帧率（153 fps）
   卡死 Windows 读（改 4096 对/帧即 8 KB 帧 + 零拷贝 payload + 同步逐字节）。
5. **WiFi raw TX 三个根因**：`set_config` 只在 mode 变化时 `esp_wifi_start`；
   `drop(controller)` 会 deinit WiFi；C5 SoftAP 模式 raw TX 返回 0 但外部不可见。
   正解 = STA 未连接 + controller 常驻 + 显式 `esp_wifi_start()`。
6. **libphy PBUS force 无效**：`phy_pbus_rd` 读 RAM 影子表、force 写硬件通道，
   两者不同步；±64 码实测零响应。放弃硬件 DC 校准，改软件直方图中值补偿。
7. **esp-hal 1.2.2 的 C5 缺口**：`read()` 强制 burst 破坏软件链；
   `ahb_v2 setup()` 内存窗口配置被注释；`apply_clock_divider` off-by-one
   （请求 40 MHz 实得 34.29 MHz）。均已记录到 upstream-issues.md。

## 4. 未解问题（后续调查入口）

1. **裸 DAC 数据路径外部不可见**：replay 与 paced TX 在 2412/2484/5180/5745 MHz、
   功率步 15、连续发射 5 分钟，外部接收机收不到；同板 vendor PHY 路径（CW /
   raw 802.11）可被收到。怀疑 DAC 数据路径有未配置的使能/选通位。TXDC 读回
   为 0（硬件消费后清），不能作为注入生效的旁证。
2. **loopprobe 采集冻结**：duplex 自环上下文中 vendor dump 采集 checksum 三次
   相同，20/80 MS/s 均如此；C2 时代同一 helper 有活数据。原因未隔离，导致
   自环无法判定发射是否有输出。
3. **回放 HALF 速率语义**：HALF=0/1 实测 138.6/54.5 MS/s，比例 ~2.5 而非 ÷2，
   与 zephyr 标称 80/40 不符；HALF=1 有 ±6% 抖动。精确速率走 paced。
4. **C5 SoftAP 信标不发射**：esp-radio 1.0.0-beta.1，官方未改动示例同板复现
   （与 Espressif esp-idf#19267 呼应）。`rtltcp` 固件保留待上游修复。
5. **libphy PBUS force 版本差异**：espsoup 在其 IDF 版本有效，本项目 blob 无效；
   硬件 DC 校准代码保留，修复后可零改动启用。
6. **rev v1.2 与参考文档差异**：DAC 速率、dump 采集行为均与 zephyr 标称有出入；
   本项目以实测为准，差异已入档。

## 5. 复现入口

```powershell
# 主机测试
cargo test --locked

# 固件构建（通用形式）
cargo build --locked -p esp-sdr-apps --bin <name> --release --target riscv32imac-unknown-none-elf

# C4 无缝流 60 s 验收
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\diagstream
powershell -ExecutionPolicy Bypass -File tools\run_acceptance.ps1 COM4 60

# C5c WiFi Beacon（烧写后 PC 扫描；发射期间不要开串口监控）
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\txwifi
netsh wlan show networks mode=bssid | findstr /i "ESP-SDR"
```

- 平台级注意事项（USB-SJ 死锁、drain 上限、defmt 阻塞）：development-log.md
  「Windows USB-Serial/JTAG 实测注意事项」。
- 寄存器与逐检查点实测：`docs/c5-registers.md` §1–§13。
- 上游问题台账：`docs/upstream-issues.md`。
- 固件/主机/脚本清单：development-log.md 对应表格。

## 6. 资产与出处

- 参考仓库（不入库）：`tools/fetch_references.ps1` + `REFERENCES.md`（URL + commit）。
- vendor blob（入库）：`vendor/librftest.a`、`vendor/libbttestmode.a`；
  来源 commit、SHA-256 与已验证符号见 `vendor/README.md`；可复现拉取脚本
  `tools/fetch_vendor_blobs.ps1`。
- 烧写器（入库）：`tools/espflash/espflash.exe` v4.6.0，SHA-256 钉死。
- 依赖锁定：`Cargo.lock` + 根 `Cargo.toml [workspace.dependencies]`。
- 归档排除：`target/`、`reference/`、`.history/`。
- 样例数据：`docs/spectrum-example.csv`（C2 频谱输出样例）。

## 7. 经验教训

- 默认值会以"物理现象"的面貌掩盖配置疏漏（80 MHz 时钟）；关键配置一律显式。
- 硬件文档与实测冲突时，以实测为准并记录差异（HALF 速率、rx_bitlen 语义）。
- 主机侧串口栈的坑与固件同等重要：USB-SJ 的行为要按"只能顺序读"设计。
- 任务书之外的"能不能直接发 WiFi"给了最高价值的对照实验：vendor PHY 通、
  裸 DAC 不通，直接把问题范围收敛到 DAC 数据路径配置。

## 8. 隐私审计与归档清单（2026-10-26）

扫描范围：全部 git 跟踪文件、git 元数据、本地配置。结论如下。

### 8.1 跟踪文件：无个人信息

- 无 `C:\Users\...`、`D:\<工作目录>\...`、`/home/...` 等本地绝对路径；
- 无用户名、邮箱、主机名、API key、密码或 token（模式扫描 0 命中）；
- 唯一 MAC 是 Beacon 中自造的本地管理地址 `02:45:53:50:44:52`（首字节
  0x02，不属于任何厂商）；`127.0.0.1` / `192.168.2.1` 是协议地址，非个人信息；
- `docs/spectrum-example.csv` 只是频谱功率数据，可能反映本地 RF 环境（信道
  占用），不含身份信息；如介意可删除；
- 仓库未配置 git remote；`.git/config` 无身份配置。

### 8.2 git 元数据：含作者身份（唯一泄露点）

- 全部提交的 author/committer 均为项目所有者的姓名与 163 邮箱（具体值可用
  `git log --format='%an <%ae>'` 查看）；`.git/logs/HEAD`（reflog）同样含该身份。
- 提交内容本身已确认无个人信息；若对外分享仓库，这是唯一需要处理的点。

### 8.3 本地生成物（未跟踪，但物理存在）

- `target/`：构建缓存。`.fingerprint/*.json` 实测含 `C:\Users\<用户名>\...`
  形式的本地路径（已命中）。**打包时必须排除**。
- `reference/`：参考仓库克隆，含各自 `.git` 与上游作者信息，**打包时排除**。
- `.git/`：含 8.2 的身份元数据。

### 8.4 建议

- **个人存档**：现状即可。推荐用 `git archive --format=zip -o esp-sdr-rs.zip HEAD`
  导出源码包（不含 `.git/`、`target/`、`reference/`，因此不含作者身份与本地路径）；
  若要保留 git 历史，则压缩整个目录并手动排除 `target/`、`reference/`。
- **对外分享且要匿名**：先在**副本**上重写作者身份（`git filter-repo --mail-map`
  或 `git filter-branch --env-filter`），再打包。重写会改变全部 commit hash，
  原始仓库不要动。
- `.gitignore` 已覆盖构建产物、参考克隆、本地抓包/频谱/日志、编辑器杂项与
  本地密钥文件；提交前用 `git status --short` 确认无新增未跟踪文件。