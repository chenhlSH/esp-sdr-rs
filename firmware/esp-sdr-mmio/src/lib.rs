//! ESP32-C5 SDR 底层寄存器访问层。
//!
//! 职责：WiFi 基带 dump engine（RX 突发采集）、DAC 回放引擎（TX）、
//! SRAM 块 1 让渡、诊断总线到 PARLIO+GDMA 的无缝采集路径。
//! 寄存器布局见 docs/c5-registers.md。
//!
//! 当前状态：C0 骨架，实现自 C2 检查点起逐项落地。
