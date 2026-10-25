# 上游问题与坑位台账

> 用途：汇总本项目发现的、值得回报上游（esp-hal / esp-radio / PAC 等）的缺陷，
> 以及虽不回报上游但必须在项目内长期记住的硬件坑。每条自包含：代码位置、症状、
> 根因、实测证据、建议修复与上报口径。新条目按文末模板追加。
>
> 约定：
> - 状态：`open`（待上报）/ `reported`（已提 issue，附链接）/ `fixed-upstream`
>   （上游已修，注明版本）/ `wontfix`（决定不上报，写明理由）。
> - 实测证据引用 docs/c5-registers.md §10.2 的实验轮次编号（A–P）与
>   commit `b0d5b71`（C4 option A 破案提交）。
> - 证据均可由 `firmware/apps/src/bin/diagprobe2.rs`（16 轮实验的最终形态）
>   在 ESP32-C5HR8 rev v1.2 上复现。

---

## A. 建议回报上游

### A1. [esp-hal] C5：`ParlIoRx::read()` 强制描述符突发预取，软件描述符链（尾 NULL）启动即 `in_dscr_err`

- **状态**：open
- **影响**：esp-hal 1.2.2，`esp32c5`（`dma_gdma_version = "2"`，AHB-GDMA v2）。
  `parl_io::ParlIoRx::read` + `dma::DmaRxStreamBuf` 的官方流式接收路径在 C5 上
  实际不可用。其他目标未测。
- **代码位置**：
  - `src/dma/mod.rs:1082`——`do_prepare()` 无条件 `self.rx_impl.set_descr_burst_mode(true)`；
  - `src/dma/buffers/mod.rs:1085`——`DmaRxStreamBuf::prepare` 明确建**非环**链
    （"Link up all the descriptors (but not in a circle)"），链尾 `next = NULL`；
    `consume()`（同文件 1202–1261 行）靠软件把消费完的描述符尾插回链维持流动；
  - `src/dma/engine/gdma/ahb_v2.rs:285-289`——`restart()`（inlink_restart）在
    整个 crate 内**没有任何调用者**：链耗尽后无恢复路径。
- **症状**：`read()` 返回 `DmaError(DescriptorError)`（或 armed 后 0 字节），
  GDMA `IN_INT_CHx.raw` = bit3（`in_dscr_err`）。实测（§10.2 F 轮）：即使手工
  关掉 `indscr_burst_en`、仅保留 NULL 尾的顺序链，C5 AHB-GDMA 走到链尾仍置
  `in_dscr_err`——**C5 把 NULL next 判为描述符错误，而非老 GDMA 的"暂停等续链"**。
  一旦置位，`inlink_restart` 无法恢复（需通道级 `in_rst`）。
- **根因**：`DmaRxStreamBuf` 的软件续链模型要求"DMA 到达链尾之前软件必须续上"，
  隐含 NULL 尾 = 可恢复暂停的硬件语义；C5 的语义是致命错误，两者冲突。突发预取
  （`indscr_burst_en=1`）进一步提前触发。
- **实测证据**：§10.2 轮次 A（read() 直接 DescriptorError）、C（armed 后 0 字节，
  int_raw=0x9）、F（burst 关 + NULL 尾仍 int_raw=0x8）。对照：闭环环 +
  `rx_bitlen` 软分隔符 EOF + `inlink_auto_ret`（复位默认开）可持续 40.0 MB/s
  （P 轮，199,997,952 字节/5 s 零重启）。
- **建议修复**（三选一或组合）：
  1. C5 上 `do_prepare` 不强制 `indscr_burst_en`，且 `DmaRxStreamBuf` 在
     `dma_gdma_version = "2"` 目标上改为环形链 + 软分隔符 EOF + auto_ret 语义
     （IDF `esp_driver_parlio` 的 partial 模式即此机制）；
  2. 文档明示 C5 不支持 `DmaRxStreamBuf`（编译期 `compile_error!` 或运行期报错），
     避免静默失败；
  3. 至少给 `ChannelRx` 暴露 `restart()` 公开 API 并在 `dscr_err` 恢复路径中使用。
- **建议上报标题（英文）**：
  `esp32c5: ParlIoRx::read() with DmaRxStreamBuf fails immediately with DmaError(DescriptorError) — AHB-GDMA v2 treats NULL-terminated descriptor chains as errors`

### A2. [esp-hal] C5：`apply_clock_divider` off-by-one——请求 40 MHz 实得 34.29 MHz

- **状态**：open
- **影响**：esp-hal 1.2.2 `src/parl_io.rs` `apply_clock_divider()`（427–465 行），
  TX/RX 共用（445–461 行），C5 实测裁定；其他基于 PCR 的目标（C6/H2 等）疑似
  同病，未测。后果：**无法通过公开 API 达到标称 40 MHz**——上限检查
  （441 行 `> 40_000_000` 拒绝）把"请求 48 MHz 换 div_num=5"的绕行也堵死了。
- **代码位置**：
  - esp-hal：`let divider = source_hz / frequency.as_hz();` 后直接
    `w.parl_clk_rx_div_num().bits(divider)`（直分频语义）；
  - 权威语义：hal_espressif（cyber-murmel fork，esp-sdr 分支
    `54d64a1289e8d0cddccef6359a385ffb4c68b213`）
    `components/esp_hal_parlio/esp32c5/include/hal/parlio_ll.h:123-128`——
    `parlio_ll_rx_set_clock_div` 写 `clk_div->integer - 1`（N-1 语义，即
    分频系数 = div_num + 1）。IDF `esp_driver_parlio` 同此（hal_utils_clk_div
    integer=6 → div_num=5）。
- **症状**：请求 40 MHz → div_num=6 → 实际 240/(6+1)=34.286 MHz。
- **实测证据**（吞吐即频率计）：
  - esp-hal 路径请求 40 MHz：34,284,544 B/s ≈ 34.29 MB/s（§10.2 O 轮）；
  - PAC 直改 `div_num=5`：39,997,952 B/s ≈ 40.0 MB/s（§10.2 P 轮）；
  - 旁证：espsoup 0.7.1 固件 `PRUN` 实测时钟 400,003,044 Hz，标称偏差 ppm=1.5
    （div_num=5 为正确值）。
- **建议修复**：`bits(divider - 1)`（divider≥1 由既有检查保证）；同时复核 40 MHz
  上限检查的语义（若 40 MHz 是外设额定上限，检查本身没问题，但修复后请求 40 MHz
  应得 div_num=5）。
- **建议上报标题（英文）**：
  `esp32c5: parl_io clock divider is off by one — with_frequency(40 MHz) yields 34.29 MHz (div_num written as N instead of N-1)`

### A3. [esp-hal] C5：AHB-GDMA 可访问内存窗口未配置（`ahb_v2 setup()` 整段注释 TODO）

- **状态**：open（诚实标注：单独必要性未隔离，见下）
- **影响**：esp-hal 1.2.2 `src/dma/engine/gdma/ahb_v2.rs:412-425`——`setup()`
  内 `INTR_MEM_START_ADDR`/`INTR_MEM_END_ADDR` 写入整段被注释
  （"TODO: configure accessible memory range with non-hardcoded data"）。
- **对照实现**：IDF / hal fork 的 `ahb_dma_ll_set_default_memory_range()`
  显式写 `0x40800000..0x44000000`（zephyr `dma_start()` 首步即调用）。
  PAC `esp32c5 0.3.0` 中两寄存器复位值标注为 0。
- **实测证据与疑点**（如实记录，避免过度归因）：
  - 我方最终工作配置包含显式窗口写入（0x40800000/0x44000000）；
  - 但 §10.2 B 轮显示：**未写窗口时 DMA 仍完成了至少一个描述符**
    （`in_done` 置位），说明 C5 硅的复位默认行为与 PAC 标注的复位值 0 不符
    （SVD 复位值可能有误，或窗口校验仅对特定条件生效）；
  - 因此该条的上报点不是"窗口不配必挂"，而是三件事：(a) setup() 是空实现 +
    TODO，应实现或移除并说明；(b) esp-hal 行为与 IDF ll 层分叉（IDF 每次初始化
    都写）；(c) PAC 标注的复位值与实测硅行为矛盾，值得向 PAC 上游核对。
- **建议上报标题（英文）**：
  `esp32c5: AHB-GDMA accessible memory range (INTR_MEM_START/END_ADDR) never configured — ahb_v2 setup() is a commented-out TODO; PAC reset value contradicts measured silicon behavior`

---

## B. 硬件/文档坑（项目内记录，暂不上报）

### B1. `rx_bitlen`（PARLIO RX_DATA_CFG bits 9:27，单位 bit）写入依赖时序

在 PCR RX 时钟域复位脉冲、FIFO 复位（`rx_fifo_srst`）、`parl_clk_rx_en=0`
**之后**写 `rx_data_cfg`，再 `rx_reg_update`，bitlen 才真正生效；不满足时序时
同一写法静默无效（readback 为 0）。字段 19 位（上限 524287 bit），57344 字节环
对应 458752 bit 装得下——不是值越界。zephyr `parlio_start()` 的顺序即安全序。
这也是 C4 卡了 16 轮的第二根因（§10.2 根因 2）。任何裸寄存器/新驱动路径都要
按此顺序自检 readback。

### B2. C5 AHB-GDMA 链尾 NULL 语义 = `in_dscr_err`（错误），非"暂停等续链"

见 A1 症状段。设计任何"软件续链"式 DMA 流（非硬件环）前先在此语义上对齐；
可恢复模型只剩：环形链 + 每圈 EOF（软分隔符 bitlen）+ `inlink_auto_ret`
（硬件自动回链头）。

### B3. 两处与直觉相反的复位默认值（PAC esp32c5 0.3.0）

- `DMA::CHx.IN_LINK` 复位值 `0x11`——bit0 `inlink_auto_ret` **默认开**
  （EOF 后硬件自动回当前描述符地址重试；对环形流是命脉，对错误恢复是死循环重试）；
- `PARL_IO.RX_GENRL_CFG` 复位值 `0x21FFE000`——bit29 `rx_timeout_en` **默认开**
  （RX 数据间隙触发超时 error EOF）。裸寄存器路径必须显式关闭；
  esp-hal 路径由 `RxConfig::default()`（timeout_ticks=None）经 `apply_config`
  自动关闭（见 C1）。

---

## C. 撤销记录

### C1. 原"缺口 4：read()/with_config 不覆盖 rx_timeout_en"——撤销

- **原结论**（docs/c5-registers.md §10.2，16 轮会话中期写入）：
  esp-hal 的 `read()`/`with_config` 不覆盖 `rx_timeout_en`，复位默认开会导致
  数据间隙触发超时 error EOF。
- **复核结论**：不成立。`src/parl_io.rs:2064-2071`（parl_io v2 实现）的
  `set_rx_timeout_ticks` 写 `rx_timeout_en().bit(value.is_some())`，而
  `RxConfig::default().timeout_ticks = None` 且 `with_config → apply_config`
  无条件调用它——**esp-hal 官方路径实际总是关闭超时**。`rx_timeout_en` 复位
  默认开仍是事实（保留在 B3），但只影响裸寄存器路径。
- **遗留**：docs/c5-registers.md §10.2"四个缺口"列表中的第 4 条应同步修正为
  三条 + 本撤销说明（待下次检查点 commit 一并处理）。
- **实验教训**：J 轮"关超时仍停滞"当时应立即触发对 with_config 是否已关超时的
  源码复核，而不是把超时保留为"缺口之一"。复合故障下逐变量归因要等总根因
  （bitlen）落定后再回头清洗。

---

## 附：新条目模板

```markdown
### A<n>. [目标 crate] 一句话症状
- **状态**：open | reported(<链接>) | fixed-upstream(<版本>) | wontfix(<理由>)
- **影响**：crate 与版本、芯片/目标、哪些 API 路径
- **代码位置**：文件:行（含本地 registry 路径或上游 commit）
- **症状**：可观察行为（寄存器/错误/日志）
- **根因**：
- **实测证据**：轮次编号 + 关键数字；复现入口（哪个 bin/工具）
- **建议修复**：
- **建议上报标题（英文）**：
```
