# 参考仓库索引（只读参考，不入库；用 tools/fetch_references.ps1 克隆到 reference/）

| 仓库 | 用途 | commit |
|---|---|---|
| https://github.com/ESPARGOS/esp-sdr | RX dump engine 寄存器文档与 C 语言实现（GPL-3.0，只读参考） | `f86da81807b7f6e850742f6b789d9468c190ad41` |
| https://github.com/cyber-murmel/esp-sdr-zephyr | C5 无缝采集/TX 回放的寄存器级权威文档（GPL-3.0，只读参考） | `58db280a11c0c12760064af647794c25d81fa967` |
| https://github.com/pit711/espsoup-firmware | C5 无缝采集第二独立实现（IDF 驱动路径；GPL-3.0，只读参考） | 手动 `git clone --depth 1`（见 docs/c5-registers.md §10.1） |

后续检查点按需追加：ESP32-DATV（C5 阶段，PolyForm-NC，仅参考原理不接触代码）。

寄存器事实的落地抄录见 docs/c5-registers.md（含来源文件与行级出处）。
