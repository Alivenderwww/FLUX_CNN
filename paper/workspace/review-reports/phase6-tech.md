# Phase 6 v3 图表设计技术评审报告

## 第 1 次评审

### 判定

FAIL

### 评审范围与抽样

- INDEX.md（全读）
- 抽样 4 张：图 4.2 mac-array、表 5.6 prior-art-compare、图 4.7 dma-subsystem、表 1.1 prior-art-comparison
- paper.md 全部图/表引用 grep（13 fig + 7 tab 编号完整覆盖）

### 通过项（PASS sub-checks）

| 项 | 状态 | 证据 |
|---|---|---|
| 正反向交叉引用 | ✅ | INDEX.md 列 13 图 + 7 表，paper.md grep 出图 1.1/2.1-2.2/3.1-3.3/4.1-4.8 + 表 1.1/5.1-5.6 编号齐全；无孤立图表 |
| 双语标题（抽 4 张）| ✅ | 图 4.2 / 图 4.7 / 表 5.6 / 表 1.1 头两行均为"图/表 X.Y 中文 / Figure/Table X.Y English"双标题 |
| 图 4.2 数据自洽 | ✅ | "16 列 × 16 PE = 256 INT8 MACs"、"WRF×16 (32 each)"、"PSUM 32 b" 全部与 CLAUDE.md 核心参数 (NUM_COL=NUM_PE=16, WRF=32, DATA=8, PSUM=32) 一致 |
| 图 4.7 数据自洽 | ✅ | "axi_dm IP (Xilinx, Vivado 2023.1)"、"4 类自研控制器 idma/wdma/rdma/odma_ctrl"、"mm2s_arb 3-to-1 round-robin"、"axi_m_mux 3-to-1"、"对外 1 AXI4 主 + 1 AXI-Lite 从" 全部与 CLAUDE.md DMA 子系统描述吻合 |
| image 提示词章节合规（抽 2 张图）| ✅ | 图 4.2 / 图 4.7 均有"中文版 + English version"双版；均含"1:1 (square) / 1:1.2"宽高比硬约束 + "避免 > 1.5:1" 警示；含 CMYK 配色（30/10/0/0、0/30/30/0 等）；含 Times New Roman 10pt + 思源黑体；含禁用清单（手写体/彩色渐变/阴影/3D/图标/卡通） |
| 表 image 提示词章节 | ✅ | 表 5.6、表 1.1 均填"无（表格直接以 markdown 形式嵌入论文）"，符合规范 |
| §5 数据 [CHECK] 覆盖（表 5.3/5.6 本工作行）| ✅ | 表 5.3 占位标 [CHECK: 功耗实测数据待 Vivado power 报告]；表 5.6 本工作功耗/能效列已标 [CHECK]；表 5.4 N=1 未启用空间到深度 cycles 已在 INDEX.md L39 列入 [CHECK] |
| 立项硬约束（避免说"FPGA 加速器"）| ✅ | 抽样图均把 FPGA 平台只用于本工作综合验证位置（表 5.6 "XC7K325T FPGA" 列），未在系统定位上说"本工作是 FPGA 加速器" |

### 未通过项（FAIL，需 Writer 修复）

| # | 严重度 | 位置 | 问题 | 期望修改方向 |
|---|---|---|---|---|
| 1 | 严重 | tab1-1-prior-art-comparison.md L44 vs tab5-6-prior-art-compare.md L48 | **VTA 频率自相矛盾**：表 1.1 给 VTA = 333 MHz（无 [CHECK]、无来源），表 5.6 给 VTA = 100 MHz（来源 literature.md L441 已确认）。同一文献 [27] 同一频率指标在两张表里不一致，且 333 MHz 没有来源标注。 | 表 1.1 VTA 频率改为 100 MHz（与 literature.md L441 锚定来源对齐），或给 333 MHz 补充确切来源；二选一。**禁止**两张表对同一文献同一指标给出不同数字。 |
| 2 | 严重 | tab1-1-prior-art-comparison.md L39-44（DianNao 452/485/0.92, TPU v1 92000/40000/2.30, Eyeriss 168/278/0.12, NVDLA 128/300/0.40, Eyeriss v2 153.6/606/0.96）vs tab5-6-prior-art-compare.md L43-50（这些工作同一指标全部 [CHECK: 数据待回查原论文]） | **数据可追溯性双重标准**：同一文献同一指标，表 1.1 写成"已确信的具体数字"（无 [CHECK] 无来源批注），表 5.6 全部标 [CHECK]。INDEX.md L39 已明确"表 1.1 / 表 5.6 共享部分文献条目"，但 Writer 未做一致化标注。这违反 chinese-thesis-spec §11.13 数据自洽硬约束——同一来源同一数据点不可在 A 表确信、在 B 表待查。 | 二选一：(a) 表 1.1 同步加 [CHECK: 数据待回查原论文]，与表 5.6 标注一致；(b) 表 5.6 在已被表 1.1 确信的指标上去除 [CHECK] 并把数据来源补到 literature.md（推荐 a，因为 INDEX.md 自己也把表 5.6 列为 [CHECK] 待补）。 |
| 3 | 中等 | tab1-1-prior-art-comparison.md L42 NVDLA 行 | NVDLA Power "~300 mW" 与 Efficiency "~0.40 TOPS/W" 标了 `~` 表示估值，但 L54 不确定项里也明确"为典型估值"。这种估值如果未来有人核对会发现"~"在论文表格里其实没在表注里说明置信度。 | 在表 1.1 注释（L46-49）补一行"NVDLA Power/Efficiency 为典型小型配置估值"，或直接改为 [CHECK]。 |
| 4 | 中等 | tab1-1-prior-art-comparison.md L48 VTA 行 Peak Perf = 170 GOPS | VTA 这一数字也无来源批注，且 literature.md L441 仅确认频率 100 MHz；如果 VTA Peak Perf 也无文献锚点，应同样标 [CHECK] 与表 5.6 一致。 | 给 VTA 170 GOPS 补 [CHECK: 数据待回查原论文]，或在 literature.md 中补锚点。 |

### 不影响判定但建议修订（轻微）

- INDEX.md L39 把"表 1.1 / 表 5.6 共享部分文献条目"作为 [CHECK] 数据待补提示，但只在表 5.6 落实标注，表 1.1 反而给出确信数字——INDEX.md 自己的指引未在表 1.1 落实，建议在 INDEX 状态汇总段补一句"表 1.1 同等共享条目同步标 [CHECK]"以避免未来再分裂。
- 图 4.7 提示词 L153-156 的 "key notes 右侧侧栏" 与 "整体 1:1 正方形" 在视觉上略有冲突（侧栏会拉宽 figure），建议提示词补一句"侧栏 key notes 字号缩小或改为底部水平排列以保持 1:1 宽高比"。

### 抽样验证记录

- 图 4.2 §"必含元素" + ASCII 示意稿 + image 提示词 → 与 docs/modules/mac_array.md（CLAUDE.md L17 引用）以及 CLAUDE.md L39 核心参数完全吻合
- 图 4.7 §"必含元素" + ASCII 示意稿 → 与 CLAUDE.md L11-14 DMA 子系统结构（idma_ctrl/wdma_ctrl/odma_ctrl/mm2s_arb/axi_dm/axi_m_mux/axi_lite_csr）完全吻合；rdma_ctrl 在 CLAUDE.md 顶层未列但在图 4.7 自洽性检查 L126 已自我标 [CHECK: 单核 vs 多核接入位置]，可接受
- 表 5.6 本工作行 (100 MHz 假设 51.2 GOPS / 68.4 MHz 实测 35.0 GOPS) → 算式 16×16 × 2 ops/MAC × Fmax 一致；功耗与能效已标 [CHECK]，符合规范
- paper.md grep 13 张图 + 7 张表编号全部出现，无孤立、无未引用

### 修复后重测路径

Writer 修复 #1 + #2 后，本评审重测可以仅复审两张 prior-art 表（tab1-1 与 tab5-6 数据一致性），≤2 工具调用即可结案。

---

## 第 1 轮重测（2026-05-07）

### 判定

PASS

### 4 问题修复情况

| # | 上次问题 | 当前状态 | 证据 |
|---|---|---|---|
| 1 | 严重：VTA 频率 tab1-1 (333) vs tab5-6 (100) 自相矛盾 | ✅ 已修 | tab1-1 L44 = `100`；tab5-6 L48 = `100`；tab1-1 L48 显式注"VTA 频率 100 MHz 来源于 literature.md L441（与表 5.6 对齐）" |
| 2 | 严重：tab1-1 给确信数字而 tab5-6 全部 [CHECK]，可追溯性双重标准 | ✅ 已修 | tab1-1 L39-44 6 行（DianNao/TPU v1/Eyeriss/NVDLA/Eyeriss v2/VTA）的所有 GOPS / Power / Efficiency 数字均加 `[CHECK: 数据待回查原论文]`；tab1-1 L50 显式注"本表与表 5.6 共享文献条目，所有 [CHECK: 数据待回查原论文] 标注与表 5.6 一致" |
| 3 | 中等：NVDLA `~` 标注未在表注中说明置信度 | ✅ 已修 | tab1-1 L47 表注新增"NVDLA 行 `*` 表示典型小型配置；Power 与 Efficiency 列的 `~` 标记表示典型小型配置估值，置信度低于 ASIC 实测数据" |
| 4 | 中等：VTA 170 GOPS 无来源 | ✅ 已修 | tab1-1 L44 = `170 [CHECK: 数据待回查原论文]`，与 tab5-6 同步 |

### 通过原因

- 2 严重问题（VTA 频率一致性、数据可追溯性双重标准）全部修复，tab1-1 与 tab5-6 在共享条目上完全对齐。
- 2 中等问题（NVDLA `~` 表注、VTA Peak Perf 标注）也已闭环。
- 抽查 tab1-1 ↔ tab5-6 共享条目数据一致性：DianNao 980/452、TPU v1 700/92000、VTA 100、Eyeriss 200、Eyeriss v2 200、NVDLA 1000，全部一致。
- 本工作行仅在 tab5-6 出现，符合 tab1-1 第 54 行设计原则（tab1-1 定位"研究现状"对比，本工作数据集中在 §5/§6）。
- 本轮无新增问题。Phase 6 v3 技术维度通过。
