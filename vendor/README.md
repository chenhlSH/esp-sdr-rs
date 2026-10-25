# vendor 目录：进仓库的二进制依赖（可复现构建）

原则：小体积依赖直接进仓库；大体积依赖在 README 记录精确资源地址、资源名与校验值，由脚本拉取。

## librftest.a（ESP32-C5）

- 来源：Espressif 官方仓库 espressif/esp-phy-lib，commit `e37ecdc9c1e93fefda5b1a99f09bd979046d0720`
  （与 esp-sdr-zephyr 所用 hal libphy 同一提交，见其 `zephyr/module.yml`）
- 下载：`tools/fetch_vendor_blobs.ps1`（URL 与 SHA-256 已钉死并自动校验）
- SHA-256：`0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba`
- 用途：射频测试/增益控制闭源 blob，随 ESP-IDF SDK 许可分发

### 已验证符号（llvm-nm --defined-only，2026-10-25）

本项目 C1/C2 将用到的（`T` 全局函数）：

- `force_rx_gain` —— 手动锁定 RX 增益（C5 上 WiFi AGC 会周期性夺回，需 20ms 重锁）
- `adctrig` —— RX dump engine（基带 IQ 采集）触发序列的库函数源头
- `dactrig` —— TX DAC 回放引擎触发序列的库函数源头
- `esp_phy_rftest_init` / `rftest_init` —— RF 测试模式初始化
- `force_tx_gain` / `force_dig_gain` / `force_txtone` / `force_iq_set` —— TX 侧增益/单音控制

其余依赖符号（`phy_set_chanfreq`、`phy_txforce`、`g_phyFuns`、
`phy_pbus_xpd_rx_on`、`phy_set_rxclk_en` 等）在 **libphy.a** 内，
由 esp-wifi-sys-esp32c5 0.3.0 crate 附带，C1 构建时用 nm 复核。
