# 写作评审报告 Phase 5 §7

### 判定：PASS

## 评审范围

paper.md §7 Evaluation 全章（line ~362-458），共 7 节、约 40 段。仅评写作（结构 / 衔接 / 风格 / 节奏 / 数据可读性），不评事实正确性。参考 §6 末段（line 360）作为衔接基线。

## 段落长度抽样（10 段）

| # | 位置 | 字数估计 | 词数估计 | 状态 |
|---|------|---------|---------|------|
| 1 | §7 章首 operating-point 提示框 (l.366) | ~110 | ~120 | OK（提示框允许略长，作为整章约定） |
| 2 | §7.1 p1 (l.370) | ~75 | ~85 | OK |
| 3 | §7.1 p2 22+24+20 case (l.372) | ~135 | ~155 | OK |
| 4 | §7.2 p1 starting-point (l.380) | ~95 | ~105 | OK |
| 5 | §7.3 p1 OOC 综合 (l.394) | ~70 | ~80 | OK（短而紧凑，全段是数据列举） |
| 6 | §7.3 末段 Fmax 根因 (l.402) | ~120 | ~135 | OK |
| 7 | §7.4 p3 86.6% 头条 (l.412) | ~95 | ~110 | OK |
| 8 | §7.5 p1 双尺度验证 (l.422) | ~115 | ~135 | OK |
| 9 | §7.5 1.45× 四部分分解 (l.428) | ~150 | ~165 | 略长但因为是分解结构 (a)(b)(c)(d) 列表化，可读性 OK |
| 10 | §7.6 86.6% vs 97% (l.440) | ~145 | ~160 | OK，但接近 200 字上限 |

无任何段超过 200 字 / 200 词上限，无任何段低于 50 字下限。**全部合规**。

## 主题句结构

逐段抽查，主题句均位于段首，后续句作为支持论据，未发现主题句埋在段中的情况。例如：
- §7.2 p1 主题句 "The starting point is unflattering" 直接给立场，后续给数据。
- §7.3 p1 主题句以"36,942 LUT ... Fmax = 68.4 MHz"为锚定数据，结尾"the design did not meet timing closure"作为评估结论。
- §7.5 p1 主题句"validated end-to-end at two scales"先给结论再给数据，符合 claim→evidence 范式。

**通过**。

## 段间承接

- **§6.5 末段 → §7 章首**：§6.5 末句 "The next chapter quantifies what that accelerator delivers." → §7 章首以 operating-point convention 引语开场，过渡顺畅。
- **§7.1 → §7.2**：§7.1 末段 "the next section opens the chapter's first quantitative narrative — single-layer PE utilization" — 承接显式且具体。
- **§7.2 → §7.3**：§7.2 末段 "the next section turns to the hardware cost side — synthesis resources and timing" — 承接显式。
- **§7.3 → §7.4**：§7.3 末段 "the next section turns to whole-network end-to-end latency" — 承接显式。
- **§7.4 → §7.5**：§7.4 末段 "the next section moves to the second system-level narrative ... how the same design scales across multiple cores" — 承接显式。
- **§7.5 → §7.6**：§7.5 末段 "the next section is the head-to-head comparison with prior art that defends both narratives" — 承接显式。
- **§7.6 → §7.7**：§7.6 末段 "the chapter closes with an explicit and honest accounting of known limitations" — 承接显式。
- **§7.7 → §8 占位**：§7.7 末段 "the next chapter closes the paper with a high-level summary" — 承接显式。

七节全部 explicit-bridge 句式，**全章承接无断裂**。

## 与 §1-§6 风格一致性

- 引用 short tag 格式：`[Kwon@ASPLOS'18]` / `[Liu-TNNLS'21]` / `[Gokhale@ISCAS'17]` 与 §3-§6 一致（注：`[Liu-TNNLS'21]` 与 `[Liu Full-Stack [TNNLS'21]]` 在 §6.4 / §7 中并存使用，两种 tag 实际指同一文献，建议 Phase 7 统一 — 见下"轻微建议"）。
- "we" 频率：每节 2-4 次，与 §3-§6 持平，未滥用。
- 学术腔调：claim 措辞（"we treat ... as a first-class deliverable"、"the honest reading is"）与 §6 一致，保持谦逊但不软弱。
- 句长节奏：与 §3-§6 持平，没有突兀拉长。
- contribution 标号 (C1.1 / C2.5 / C3.5) 在 §7 内未频繁出现（§7.2 提及 §3.3 / §5.4，§7.6 出现一次"C1.1, §4"），引用密度合理，比 §6 略低（合理：§7 是数据章，回扣靠数据而非 ID）。

**通过**。

## 数据章可读性

§7 最大风险是"数据淹没"。实际读起来：
- §7.2 PE 利用率：用粗体标 "12.5% / 25% / 50% / near 100%" 四个锚点，读者一眼抓得到。
- §7.3 综合数字：粗体标 "36,942 LUT / Fmax = 68.4 MHz / 128 BRAM"，密度高但因有粗体锚点不淹没。
- §7.4 头条："593K cycles / 86.6% / 5.95 ms / 8.69 ms / 168 fps / 115 fps" 集中在一段，但用并列结构组织，无堆叠感。
- §7.5 多核 cycles："9057 / 8808 / 3833 / 5569 / 1.45×" 是本章数字最密集处，但通过 N=2 / N=4 两个分句切开，读者能跟得上。

**粗体锚点使用得当、数据被分句切开、未发现淹没现象。通过**。

## §7.2 / §7.5 narrative 主战场可读性

**§7.2（compiler-side narrative A）层次**：starting-point unflattering → Table 3 总览 → mechanism §3.3 / §5.4 → 与整网 MAC% 区分 → MAERI / Eyeriss-v2 对照 → 收尾承接。**先总—后分—对照—结论** 四段式，逻辑清晰。读完能被说服"compile-time 换 hardware 是合理的设计点"。

**§7.5（system-level narrative B）层次**：双尺度验证 (N=2 + N=4) → Table 6 详细 sweep → 11-layer 估算（坦诚标注"projection 而非 measured"）→ 1.45× 四部分分解 (a)(b)(c)(d) → Simba / Liu / Kang 对照 → 收尾承接。**结构与 §7.2 对称**，narrative 主战场两节读起来"双轨平行"，审稿人能并列追踪 A+B。

**1.45× 短于理想 2× 这一可能引起质疑的数字**，作者用 (a)(b)(c)(d) 分解 + "we deliberately do not claim near-linear scaling" 兜住，**诚实但不软弱**，是 §7.5 最稳的处理。

**两节均通过**。

## §7.6 head-to-head 比较表

§7.6 通过散文+表格引用方式呈现：
- **Table 7**（性能比较）：列结构在散文中明确列出 "dataset, device, Fmax, GOPS, MAC%, end-to-end latency"，10 个 baseline 名字一一列出，列对位清晰。
- **Table 8**（设计轴比较）：5 维度 "dataflow / streaming granularity / multi-core partition / compiler scope / SDP fusion" 明确，FLUX_CNN 占据格的描述具体（"row-ring streaming + W-slice multi-core + compile-time Ky-fold/S2D + full SDP fusion"）。

注：Table 7 / Table 8 当前是**散文化描述其结构**，未在 paper.md 中实际渲染 markdown 表格——这一选择在 [CHECK] 阶段可接受（数字尚未完全核对），但 Phase 7 需补出实际表格 markdown。**这不影响当前 phase 判定**——§7.6 散文部分对每个 baseline 都有具体定位（Liu = 同范式异 scale；Snowflake = 分母异；Angel-Eye / Gemmini = 同器件异 control 哲学），逻辑清晰。

**通过（标注"Phase 7 表格补渲染"为非阻塞建议）**。

## §7.7 Limitations 语气

逐项检查：
- **(a) Fmax shortfall**：直接承认"the design did not meet timing closure"，但立刻给出"two contributors ... both with concrete fixes" + "estimated to reclaim ~17K LUT"——**诚实 + 有量化路径**。
- **(b) Cout<16 idling**：用"the design choice was to keep the array geometry simple"承认是设计选择而非疏忽，给出"empirical justification: contribute a negligible fraction of total MAC count"——**有据 + 不软弱**。
- **(c) Pooling / Depthwise / sparsity not implemented**：直接列为 future RTL work，**不 wave-hand**。
- Future-work plan：(1)(2)(3)(4) 排序按 ROI，每项有"completable pre-submission"或"1-2 days"或"medium-term"具体度量——**不空喊"as future work"**。

未发现"reviewer-bait"措辞（如"trivial extension"、"obvious"、"easy to add"）。语气标准的"诚实承认 + 有路径"学术写作。

**通过**。

## 学术英文腔调

- buzzword 检查：未发现 "novel"、"smart"、"intelligent"、"optimal"、"superior"、"groundbreaking"、"revolutionary" 等过强或营销化词汇。
- over-claim 检查：1.45× 处明确 "short of the ideal 2×"；86.6% vs 97% Liu 处明确 "we do not contest Liu's number"、"same direction, different scale point, not their number is better than ours"；§7.4 处 "we defer the precise scale-and-metric calibration ... rather than over-claim here"——**主动避免 over-claim 的措辞频繁出现**，是 §7 的稳健底色。
- 第一人称 "we"：每段最多 2-3 次，无滥用。
- cliché：未发现"to the best of our knowledge"、"a wealth of"、"shed light on"、"in this era of" 等套语。

**通过**。

## 句长与节奏

抽样 §7.5 1.45× 分解段（最长复合段）：四个 (a)(b)(c)(d) 子句长度均 25-35 词，无单从句超 50 词。
§7.6 86.6% vs 97% 段最长句 "FLUX_CNN at 36,942 LUT on a mid-range 7-series device versus Liu on Arria 10 GX1150 with roughly 1.15M ALMs (~1150K equivalent LUT-class units)" 约 35 词，未套娃。
未发现 60+ 词从句套娃。

**通过**。

## 术语一致性

- "PE utilization" / "MAC%" / "MAC efficiency" 三词在 §7 内的区分：作者明确在 §7.2 / §7.4 区分 "per-layer occupancy (mac_array duty cycle)" 与 "whole-network MAC efficiency"，并在 §7 章首 operating-point 提示框内提前声明，**消歧明确**。
- "Ky-fold" / "S2D" / "space-to-depth" 三词与 §3.3 / §5.4 一致。
- "row-ring" / "streaming row-ring" 与 §3-§6 一致。
- "AXI-Lite" / "axi_dm" / "DataMover" 与 §6 一致。
- "Shortcut Bank" 大小写与 §6.3 / §6.4 一致。

未发现术语别名混用。**通过**。

## [CHECK] 标记密度对阅读流畅性的影响

§7 共出现约 21 处 [CHECK] 和 2 处 [TBD]（[CHECK: ...] 形式），统计分布：
- §7.1：1 处
- §7.2：2 处
- §7.3：3 处
- §7.4：4 处
- §7.5：4 处
- §7.6：4 处
- §7.7：3 处

[CHECK] 全部以 `[CHECK: ...]` 形式置于句末，**未打断主句逻辑**。读者在跳过 [CHECK] 后，主文连贯性不受影响（实测略读：从 §7.1 到 §7.7 跳过所有 [CHECK]，主文逻辑链完整）。

[CHECK] 描述本身**清晰**（用户能看明白要补什么）：
- 例 `[CHECK: 22+24+20=66 case 总数最终核对 — 当前 STATUS 列出 22 chain + 26 单核 + 20 W-slice，需在投稿前重新统一口径]` — 描述具体到对账目标。
- 例 `[CHECK: 13% gap 各项占比的近似分解数字 — 需要从波形或 profile 计数器读出]` — 说明数据来源。

**[CHECK] 密度未损害可读性，描述质量合格**。Phase 7 润色阶段需做 (a) [CHECK] 全部清掉、(b) Table 3/4/5/6/7/8 实际渲染、(c) `[Liu-TNNLS'21]` vs `[Liu Full-Stack [TNNLS'21]]` 等 short tag 统一。这些是 Phase 7 例行工作，不影响当前 phase 判定。

## 通过-失败汇总

| 维度 | 状态 |
|------|------|
| 段落长度合规（10 段抽样） | PASS |
| 主题句结构 | PASS |
| 段间承接（§7 内 7 节 + §6.5→§7 + §7.7→§8） | PASS |
| 与 §1-§6 风格一致 | PASS |
| 数据章可读性 | PASS |
| §7.2 / §7.5 narrative 主战场可读性 | PASS |
| §7.6 head-to-head 比较 | PASS（散文部分；表格 Phase 7 渲染） |
| §7.7 Limitations 语气 | PASS |
| 学术腔调（无 buzzword / 无 over-claim） | PASS |
| 句长节奏 | PASS |
| 术语一致性 | PASS |
| [CHECK] 密度对可读性影响 | PASS |

**全部 12 项通过。**

## 修订建议（≤3 处轻微，不影响 PASS）

1. **§7.4 / §7.5 / §7.6 引用 tag 统一**：`[Liu-TNNLS'21]`（§6.4 用法）与 `Liu Full-Stack [TNNLS'21]`（§7.4 / §7.6 用法）指同一文献。建议 Phase 7 统一为 `[Liu-TNNLS'21]` 或 `[Liu@TNNLS'21]`（与 `[Kwon@ASPLOS'18]` / `[Gokhale@ISCAS'17]` `@` 符号约定对齐）。
2. **§7.6 Table 7 / Table 8**：当前以散文化描述列结构。Phase 7 需补出实际 markdown 表格渲染，并把 §7.6 散文中"Table 7 list ..."的列描述压缩，避免散文与表格重复。
3. **§7 章首 operating-point 提示框**：以 `> ` 引语形式呈现的整章约定建议 Phase 7 考虑改为 `**Operating-point convention.**` 加粗段首句的标准格式，与 §6 子节首段风格一致——当前 `> blockquote` 在长章首使用略显突兀（轻微视觉割裂，不影响理解）。

## 提示给其他评审

- §7.3 use_dsp 估算 "~17K LUT 节省" 与 "100+ MHz" 是 writing 上自洽的"已知问题 + 修复路径"叙述，但 **17K LUT 数字 + 100 MHz 投影的事实可信度** 建议 tech reviewer 审。
- §7.5 N=4 wslice1 的 1.45× 是否真为 N=2→N=4 比较口径（baseline 归一化）建议 tech reviewer 审。
- §7.6 86.6% vs 97% Liu 的"same paradigm at different scales" 这一定位是否过于谦让 / 是否应在 novelty 层面更主动 defend，建议 novelty reviewer 审。
