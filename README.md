# esp-sdr-rs

ESP32-C5 上的 SDR 固件，Rust + esp-hal/esp-rtos（embassy 异步）从零重写。
参考 ESPARGOS/esp-sdr 与 esp-sdr-zephyr 的寄存器级成果（见 REFERENCES.md 与
docs/c5-registers.md），代码独立实现。

目标板：ESP32-C5HR8（16MB 闪存 / 8MB PSRAM / USB-Serial-JTAG）。

## 检查点状态

| 检查点 | 内容 | 状态 |
|---|---|---|
| C0 | workspace 骨架、钉死工具链、heartbeat、参考与 vendor 落地 | **完成**（上板验证通过） |
| C1 | 射频点亮（PHY 启动、调谐、增益锁） | 未开始 |
| C2 | 突发采集 PoC + 主机频谱工具 | 未开始 |
| C3 | rtl_tcp 节拍流 | 未开始 |
| C4 | 无缝流（诊断总线→PARLIO→GDMA→定点抽取） | 未开始 |
| C5 | TX 发射 | 未开始 |

每个检查点完成后暂停，审核通过才继续。

## 环境自举（去本地化）

- Rust 工具链：`rust-toolchain.toml` 钉死 `1.99.1` + `riscv32imac-unknown-none-elf`，
  任何装有 rustup 的机器首次构建自动安装；无需 nightly、无需 espup（C5 是 RISC-V）。
- 烧写器：`tools/espflash/espflash.exe`（v4.6.0，zip SHA-256 钉死），
  由 `tools/fetch_espflash.ps1` 拉取，可执行文件直接入库。
- vendor blob：`vendor/librftest.a`（来源与符号验证见 vendor/README.md），
  由 `tools/fetch_vendor_blobs.ps1` 拉取，入库。
- 参考仓库不入库：`tools/fetch_references.ps1` 克隆到 `reference/`（已 gitignore），
  commit 记录在 REFERENCES.md。
- 依赖版本全部由根 Cargo.toml `[workspace.dependencies]` 集中钉定，提交 Cargo.lock，
  构建一律 `--locked`。

## 常用命令

```powershell
# 主机侧构建/测试（default-members：host + shared）
cargo test --locked

# 固件构建（显式指定包与目标；dev 与 release 均已验证）
cargo build --locked -p esp-sdr-apps --target riscv32imac-unknown-none-elf

# 烧写（已验证；COM4 为本机 USB-Serial/JTAG 端口，按实际端口调整）
tools\espflash\espflash.exe flash -S -p COM4 target\riscv32imac-unknown-none-elf\release\heartbeat

# 监控 defmt 日志（已验证；--elf 用于符号与地址解析）
tools\espflash\espflash.exe monitor -S -p COM4 --elf target\riscv32imac-unknown-none-elf\release\heartbeat -L defmt --non-interactive
```

注意事项（Windows USB-Serial/JTAG）：

- `flash --monitor` 组合命令在监控接入阶段可能报 `os error 22`，分开执行
  `flash` 与 `monitor` 可稳定工作。
- rustup 官方 CDN 若过慢，可用镜像加速安装工具链：
  `set RUSTUP_DIST_SERVER=https://mirror.sjtu.edu.cn/rust-static`（仅用户侧环境变量，
  本仓库构建不依赖任何镜像）。

## 目录结构

```
firmware/esp-sdr-mmio   寄存器层：dump engine / DAC 回放 / PARLIO+GDMA（C2 起）
firmware/esp-sdr-radio  射频控制层：PHY 启动、调谐、增益锁（C1 起）
firmware/apps           固件入口（heartbeat、capburst、rtltcp、tx）
shared                  固件与主机共用的纯逻辑：帧协议、位域、定点 DSP
host                    PC 端工具（收帧、FFT 频谱、rtl_tcp 校验）
docs/c5-registers.md    C5 寄存器与调用序列（含源码级出处）
vendor                  进仓库的二进制依赖（librftest.a）
tools                   环境自举脚本与 vendored 烧写器
reference               参考仓库克隆区（不入库）
```
