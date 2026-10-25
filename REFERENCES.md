# 参考仓库索引（只读参考，不入库；用 tools/fetch_references.ps1 克隆到 reference/）

| 仓库 | 用途 | commit |
|---|---|---|
| https://github.com/ESPARGOS/esp-sdr | RX dump engine 寄存器文档与 C 语言实现（GPL-3.0，只读参考） | `f86da81807b7f6e850742f6b789d9468c190ad41` |
| https://github.com/cyber-murmel/esp-sdr-zephyr | C5 无缝采集/TX 回放的寄存器级权威文档（GPL-3.0，只读参考） | `58db280a11c0c12760064af647794c25d81fa967` |
| https://github.com/pit711/espsoup-firmware | C5 无缝采集第二独立实现（IDF 驱动路径；GPL-3.0，只读参考） | 手动 `git clone --depth 1`（见 docs/c5-registers.md §10.1） |
| https://github.com/cyber-murmel/hal_espressif | esp-sdr-zephyr 的 HAL fork（parlio_ll/ahb_dma_ll 寄存器语义权威；只读参考） | `54d64a1289e8d0cddccef6359a385ffb4c68b213`（esp-sdr 分支，`git fetch --depth 1 origin <sha>`） |
| espsoup v0.7.1 预编译固件 | rev1.2 板跨实现裁判（PDIAG/PCAP/PRUN，见 §10.2）；放 reference/espsoup-bin/ 不入库 | https://github.com/pit711/espsoup-firmware/releases/tag/v0.7.1 |
| IDF components/esp_driver_parlio/src/parlio_rx.c | PARLIO RX 驱动层源码（partial 模式/gdma 策略；只读参考） | reference/espsoup-bin/parlio_rx.c（上游 esp-idf master 抓取） |

后续检查点按需追加：ESP32-DATV（C5 阶段，PolyForm-NC，仅参考原理不接触代码）。

寄存器事实的落地抄录见 docs/c5-registers.md（含来源文件与行级出处）。
