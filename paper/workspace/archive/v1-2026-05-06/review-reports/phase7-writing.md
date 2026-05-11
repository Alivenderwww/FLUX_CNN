# 写作评审报告 Phase 7

## 第 1 次评审

### 判定：FAIL

### 评审范围
- 主对象：`paper/workspace/paper.md`（Phase 7 polisher 收口快照，467 行）
- 视角：全文最终态语言 / 术语 / 章节衔接 / 学术腔调一致性
- 覆盖：8 章 frontmatter + §1-§8 全部正文
- 已抽样核对：5 个核心术语（Ky-fold / S2D / row-ring / mac_array / SDP）+ 6 段每章腔调样本 + 8 个章节边界 + Phase 6 三处 known-issue 修复点

---

### 整体可读性

通读 467 行，整体阅读体验**结构清晰、风格相对稳定**。每章首段定位 + 末段过渡的写法贯穿全文，长段落（180-250 字）与短段落（80-120 字）节奏交替，没有大段堆叠 evidence 的情况。学术腔调以 "We adopt / We treat / We do not claim" 框架贯穿，第一人称 "we" 全文统一，时态规范（方法描述用现在时，回归测试结果用现在完成时 / 现在时报告）。

但有**两处明显的腔调与衔接断裂**，影响连贯阅读体验，详见下文"章节衔接最终态"与"学术腔调全文一致"。

---

### 章节衔接最终态

8 个章节边界逐一抽检：

| 边界 | 末段过渡句 | 首段衔接 | 评价 |
|------|----------|---------|------|
| §1→§2 | "The next chapter develops the dataflow taxonomy..." (L71) | "Spatial accelerators for convolution are conventionally classified..." (L79) | **流畅** |
| §2→§3 | "The next chapter walks through these three families in turn..." (L115) | "Spatial array accelerators for convolution form a now well-populated..." (L123) | **流畅** |
| §3→§4 | "§5 develops the compiler passes... §6 covers the streaming and multi-core..." (L181) | "This chapter develops the architecture along axis (1)..." (L187) | **流畅**（明确指回 §3.5 五轴） |
| §4→§5 | "The next chapter develops the compiler side..." (L241) | "A 16×16 fixed MAC array fully fills only when both Cin ≥ 16 and Cout ≥ 16..." (L247) | **流畅** |
| §5→§6 | "...the next chapter turns to the system-integration side..." (L294) | "This chapter describes the system-level engineering..." (L300) | **流畅** |
| §6→§7 | "...turn the architectural contributions of §3-§5 into a deployable accelerator. The next chapter quantifies what that accelerator delivers." (L356) | "**Operating-point convention used throughout this chapter.**..." (L362) | **稍突兀** — §6 末尾承诺"quantifies what that accelerator delivers"，但 §7 首屏没有引言段落直接回应这一承诺，而是先抛 callout box 说"两套频率 / 两套 metric 区分"。读者期待的"概览即将给出哪些数据"在 §7.1 才出现 |
| §7→§8 | "...next chapter closes the paper with a high-level summary that re-ties the two narratives..." (L454) | "This paper has argued, on a fixed 16×16 INT8 spatial array..." (L462) | **流畅** |

**衔接断裂**：§4 内部 §4.4 末段 "[TBD: §4.4 是否合并入 §4.2 由 polisher 阶段决定]"（L231）—— 该 TBD 标记应已被 polisher 阶段裁决，残留至最终稿不合适。属于**写作收口遗留**，但因 [TBD] 不计 writing 缺陷，记为提示。

---

### 术语全文一致

抽 5 个核心术语逐一全文搜：

| 术语 | 出现频次 | 形态变体 | 一致性 |
|------|---------|---------|------|
| **Ky-fold** | 多处 | "Ky-fold" / "Ky-folding" / "Ky-folded" 三种均有 | **轻微不一致** — §1/§3 用 "Ky-fold"（名词），§5.1 节标题用 "Ky-Folding"，§5 正文 "Ky-folding"。属于词性变化合理范围，但建议在论文术语首次定义时统一拼写为 "Ky-fold" + "Ky-folding"（动名词），不混用 "Ky-folded" |
| **S2D / Space-to-Depth** | 多处 | "Space-to-Depth (S2D)" 首次定义 §1.3 L45，后续混用 "S2D" 与 "Space-to-Depth" 与 "space-to-depth"（小写） | **轻微不一致** — §7.1 L368 出现 "space-to-depth"（小写），§5.2 标题 "Space-to-Depth Folding"。规范应：首次展开后一律用缩写 "S2D" |
| **row-ring** | 多处 | "row-ring" / "row-level ring" / "streaming row-ring" 三种 | **可接受** — 三者语义层次不同（row-ring 是技术名，streaming row-ring 是修饰强调，row-level ring 是描述性），混用在上下文中合理 |
| **mac_array** | 多处 | 全文统一用 `mac_array` 加反引号 | **完全一致** |
| **SDP** | 多处 | §1.3 / §3.5 / §6.3 / §7.3 / §7.7 | **完全一致** — "SDP (Single Data Point processor)" 首次展开（§3.5 L169 + §6.3 L328），后续 "SDP" 缩写。完全规范 |

**术语轻微不一致 2 处**（Ky-fold 词性 + S2D 大小写），属于轻微问题。

---

### 学术腔调全文一致

抽 6 段每章核对腔调：

| 章 | 抽样段 | 第一人称 | buzzword | 时态 | 评价 |
|----|--------|---------|---------|------|------|
| §1 | L21 (Motivation) | "we" 适度，每段 1-2 次 | 无 | 现在时（方法描述） | **规范** |
| §2 | L83 (FLUX_CNN's 16×16) | "we" 适度 | 无 | 现在时 | **规范** |
| §3 | L137 (FLUX_CNN occupies opposite point) | "we" 适度 | 无 | 现在时 + 过去时（"reports" / "demonstrates"） | **规范** |
| §4 | L201-205 (decentralized pipeline) | "we" 较少，更多用 "the module / each module" | "elastic-join discipline"（L203）属技术术语，可接受 | 现在时主导 | **规范** |
| §5 | L262 (broader perspective) | "we" 适度 | "distinguishing point" — 略有自夸但已用 "to our knowledge" 软化 | 现在时 + 过去时混合 | **规范** |
| §7 | L424 (1.45× decomposition) | "we" 适度，多用 "(a)/(b)/(c)/(d)" 列表化分解 | 无 | 现在时 | **规范** |

**整体腔调统一**，但有 **1 处腔调与全文不一致** —— §4.3 L219 "We frame 'unified streaming row-ring' as the **sole** runtime mode of FLUX_CNN; the original batch mode is now a **degenerate case** ..." 这里使用 **加粗 "sole" / "degenerate case"** 强调，但全文其他位置很少用 markdown 加粗强调（除 §7 数据点和 callout box 用于突出数字外）。这处加粗在一段技术描述中显得突兀，与全文克制的学术腔调不符。建议去掉加粗或改用斜体。

---

### Phase 6 三处 known-issues 修复影响

#### (1) Tab 重编号
全文出现 Tab.1 ~ Tab.7，逐一核对引用：
- **Tab.1 (§5.4 L290)**: "summarized in Table 1" — §5.4 内部首次提及，对应"四轴 Ky-fold/S2D vs hardware-reconfigurable 比较"
- **Tab.1 在 §7.2 L384 又被引用**: "summarized in Table 1 (§5.4)" — **正确回指**到 §5.4 那个 Tab.1
- **Tab.2 (§7.2 L378)**: "Table 2 lays out a per-layer comparison" — §7.2 PE 利用率
- **Tab.3 (§7.3 L392)**: "Table 3 takes this single-core breakdown..." — §7.3 资源
- **Tab.4 (§7.4 L406)**: "Table 4 decomposes..." — §7.4 端到端
- **Tab.5 (§7.5 L420)**: "Table 5 reports the multi-core scaling sweep" — §7.5 多核
- **Tab.6 / Tab.7 (§7.6 L432-434)**: §7.6 prior art 比较 + 设计轴比较

**重编号后引用全部正确，无错位**。Polisher Tab 重编号工作执行到位。

#### (2) §3.6 5-axis 重写
读 §3.6（L173-181），三段结构：
- L175 引子段：明确"五轴"定位
- L177 主体段：列出 5 轴 + 每轴对标 prior art
- L179 收尾段：将 §3.2-§3.4 论证映射回五轴
- L181 过渡段：将五轴映射到 §4-§7

**结构清晰流畅**，五轴命名（intra-array dataflow / location of PE-utilization recovery / streaming granularity / multi-core scaling dimension / scope of compiler stack）在 §7.6 L434 Table 7 重新出现时**完全对齐**："(1) intra-array dataflow / (2) PE-utilization recovery location / (3) streaming granularity / (4) multi-core partition axis / (5) compiler-stack scope"。**呼应到位**。

#### (3) §1.3 wgt_buffer side-feed
- §1.3 L43: "with `wgt_buffer` feeding weights side-on into `mac_array` from the WRF"
- §4.1 L193 呼应: "with `wgt_buffer` feeding weights side-on into `mac_array` from the WRF rather than sitting in the main forward path"

**两处表述一致且 §4.1 进一步展开"rather than sitting in the main forward path"做了细化补充**。呼应自然，无突兀。

**Phase 6 三处修复全部落地干净**，无引入新写作问题。

---

### §5 / §7 关键章可读性

#### §5（narrative A 主章）
- §5 引言（L247）开门见山：明确 "Cin ≥ 16 AND Cout ≥ 16" 条件 + "compiler-side 推 Ky-fold / S2D" 两条主线 + 章节路线图
- §5.1 / §5.2 / §5.3 / §5.4 节奏均衡，每节 4-6 段
- §5.4 末段（L294）"With these results in hand..." 自然过渡到 §6
- §5.2 L274 "A note on prior art:" 段落开头 —— 这是 §5.2 的**第三次** [CHECK] 标记的引用谱系问题，与 L270 / L274 的 [CHECK] 信息重复。读者会感到"为什么这一节同一个问题被标记三次"。建议在 §5.2 末段统一收口为一个 [CHECK]。属**轻微**冗余，记为提示

**结论**：§5 作为 narrative A 主章可读性强，论证链条 "trigger → math → benefit → cost → prior art positioning" 清晰。审稿人能被说服。

#### §7（数据章）
- §7.1 ~ §7.7 七节结构完整：setup → 单层 PE → 资源 → 端到端 → 多核 → 比较 → 局限
- 每节末段都有过渡句（"the next section moves to..."），衔接自然
- §7.4 L410 "13% gap 四项分解" 论证清晰，预期审稿人质疑得到回应
- §7.5 L424 "1.45× speedup 四点分解" 同样有方法论自觉
- §7.6 L436 "honest reading is 'same direction, different scale point,' not 'their number is better than ours.'" 这处腔调拿捏到位

**§7.1 引言空缺**：§7 首屏（L362）直接给 callout box（"Operating-point convention"），没有"本章将报告 PE 利用率 / 资源 / 端到端 / 多核 / 比较 / 局限六块数据"的概览段，与 §1/§2/§3/§4/§5/§6/§8 章首"先一段引言段后跳节"的统一结构**不一致**。这是 §7 唯一的结构断裂点。

---

### §8 收尾

§8 (L460-466) 仅一节（§8.1 Conclusion），三段结构：
- L462 重述 narrative A + B + 不过度宣称
- L464 三线证据（functional / hardware closure / end-to-end），与 §7 数据呼应
- L466 future work ROI 排序

**收尾质量良好**：
- 无新内容引入
- 论文核心论点（compiler-side 解决 PE 不足）在结尾段重申
- "We deliberately avoid claiming compiler-side folding strictly dominates..."（L462）继承全文克制腔调
- L466 future work 清单与 §7.7 list 一致，无矛盾

**轻微问题**：§8 仅一节 §8.1，结构上 §8.1 标题与 §8 标题重复"Conclusion"。要么取消 §8.1 子节，要么把 §8.1 重命名为更具体的内容（如 "Summary of Contributions"）。但 [TBD] 类决策不计 writing 缺陷。

---

### frontmatter 占位整齐度

L1-L11 frontmatter：
- title / authors / venue 三处 [TBD] 格式统一："[TBD: 描述 — 候选见 outline.md / Phase 1 倾向 ...]"
- data_snapshot_commit / status_snapshot_date / narrative 已填实值
- L9-L11 解释 [TBD] / [CHECK] 含义 + 标记统计 — 对审稿人友好

**frontmatter [TBD] 占位整齐**，无问题。

---

### HTML 来源注释影响

抽样统计：54 条 `<!-- 来自 ... -->` 注释贯穿全文，分布密度 ~0.12 条/行。

**渲染态影响**：在 markdown 渲染（GitHub / VSCode preview）中 HTML 注释**完全不可见**，对最终读者零影响。
**源码态影响**：在 source view 中（如评审讨论时），注释行作为段末尾随，部分段落出现两条紧邻的 `<!-- 来自 X --><!-- 来自 Y -->`（如 §5.1 L256 mid-line），略显信息密集，但读者大多数情况下不看 source。

**结论**：HTML 注释决策**不影响阅读流畅性**。Polisher 保留全部 54 条的决策可接受。

---

### 通过 / 失败

**FAIL**。

### 失败原因汇总

| # | 严重度 | 位置 | 问题 | 修改建议 |
|---|--------|------|------|---------|
| 1 | 中等 | §7 章首 (L360-372) | §7 首屏直接是 callout box（"Operating-point convention"），缺一段 §7 章节引言概述六小节内容。与 §1/§2/§3/§4/§5/§6/§8 全部其他章首"先引言段后跳节"的全文统一结构断裂 | 在 L362 callout 之后、§7.1 标题之前补 50-80 字章引言段，概述本章将沿"setup → 单层 PE → 资源 → 端到端 → 多核 → 对比 → 局限"七节展开，并指明哪两节（§7.2 / §7.4）是 narrative A 的主要数据支撑、哪两节（§7.5 / §7.6）是 narrative B 的主要数据支撑 |
| 2 | 中等 | §6 末段 (L356) → §7 首屏 (L362) | §6 末段承诺 "the next chapter quantifies what that accelerator delivers"，但 §7 直接抛 callout box 抢节奏，未先给出"本章将量化哪些指标"的回应段。导致 §6→§7 边界出现期待落空 | 与上一项联动修复：补 §7 章引言段后，§6→§7 边界自然衔接 |
| 3 | 轻微 | §4.3 L219 | "We frame 'unified streaming row-ring' as the **sole** runtime mode of FLUX_CNN; the original batch mode is now a **degenerate case**" — 加粗 "sole" / "degenerate case" 与全文克制学术腔调不符（其他位置加粗仅用于数字 / callout） | 去掉加粗或改用斜体 *sole* / *degenerate case* |
| 4 | 轻微 | 全文 | "Ky-fold" / "Ky-folding" / "Ky-folded" 三种形态混用；"S2D" / "Space-to-Depth" / "space-to-depth"（小写）三种形态混用 | 统一规则：Ky-fold（名词）/ Ky-folding（动名词），不用 Ky-folded；S2D 在首次定义后一律用缩写，避免出现小写 "space-to-depth" |
| 5 | 轻微 | §5.2 L270 / L274 | 同一 [CHECK]（"S2D 引用谱系"）在 §5.2 内部出现 2 次，加上 §1.4 L55 / §1.4 L57 / §3.3 L147 / §3.6 L179 共 6 处。读者会感到"同一问题反复标记" | 在 §5.2 内部两处 [CHECK] 合并为一处（放在 §5.2 末段），其他章节首次出现处保留即可 |

### 修订建议（优先级）

**优先修（结构性）**：问题 1+2 联动 —— 在 §7 章首补一段 50-80 字章引言段，回应 §6 末段承诺并概述六节内容。

**次优先（轻微）**：
- 问题 3：§4.3 L219 去加粗
- 问题 4：术语统一 sweep（Ky-fold + S2D 大小写）
- 问题 5：§5.2 内部重复 [CHECK] 合并

### 提示（不影响判定，仅供主 Agent 参考）

- §4.4 L231 残留 "[TBD: §4.4 是否合并入 §4.2 由 polisher 阶段决定]" — 这是 polisher 应裁决项，残留至最终稿建议由主 Agent 在收口循环中确认是否合并
- §6.4 L348 / §6.5 L356 同类 "[TBD: 是否提升为顶层章 / 是否单独成节]" 也是 polisher 阶段裁决项
- §8 仅 §8.1 一节，§8.1 标题与 §8 标题重复 — 建议要么取消 §8.1，要么 §8.1 改名（如 "Summary"）
- §1.3 L43 "Figure 1 [TBD: Fig.1 顶层框图占位 ...]" — 描述清晰，无写作问题，仅做 frontmatter 完整性提示

事实正确性问题**未发现**（评审范围外，由 tech reviewer 已 PASS）。

---

## 第 1 轮重测（2026-05-01）

### 判定

**PASS**。

### 5 问题修复情况

| # | 上次问题 | 修复位置 | 当前状态 |
|---|---------|---------|---------|
| 1 | §7 章首缺引言段（中等） | §7 L364 "This chapter answers the quantitative promise made at the close of §6 and proceeds in seven sections: §7.1 fixes the experimental setup; §7.2 reports per-layer PE utilization (the principal data backing narrative A, alongside §7.4); §7.3 reports post-synthesis resource and timing; §7.4 reports whole-network MAC efficiency; §7.5 reports multi-core W-slice scaling (the principal data backing narrative B, alongside §7.6); §7.6 places the result against prior accelerator work along the five axes of §3.6; and §7.7 closes with an honest accounting..." | **修复** — 80 字章引言段补在 callout box (L362) 与 §7.1 标题之间。逐节（§7.1-§7.7）列出内容并明确指出 §7.2 / §7.4 是 narrative A 数据支撑、§7.5 / §7.6 是 narrative B 数据支撑，与第 1 轮修改建议完全对齐 |
| 2 | §6 末段→§7 边界期待落空（中等，与 #1 联动） | §6 末段 L356 "...turn the architectural contributions of §3-§5 into a deployable accelerator. The next chapter quantifies what that accelerator delivers." → §7 L364 "This chapter **answers the quantitative promise made at the close of §6**..." | **修复** — §7 章引言段开篇直接回应 §6 末段承诺（"answers the quantitative promise"），边界期待自然落地，无突兀感 |
| 3 | §4.3 L220 三处加粗（轻微） | §4.3 L220 现读 "We frame 'unified streaming row-ring' as the sole runtime mode of FLUX_CNN; the original batch mode is now a degenerate case in which the ring capacity happens to cover the whole feature map." | **修复** — "sole" / "degenerate case" 加粗已去除，与全文克制学术腔调一致 |
| 4 | 术语统一 (Ky-fold/S2D)（轻微） | (a) Ky-fold/Ky-folding：L252 §5.1 标题 "Ky-Folding"，L254/L258/L272 "Ky-folding"，L260 "Ky-fold"，L46 §1.3 "Ky-fold"。**无 "Ky-folded" 形态**（前轮三态混用问题消除）。(b) S2D/Space-to-Depth：L264 §5.2 标题 "Space-to-Depth Folding" + 首次定义 "Space-to-Depth (S2D)"；L266/L268/L272/L274 一律缩写 "S2D"；L368 §7.1 用 "`--s2d` (S2D)" — **无小写 "space-to-depth" 残留** | **修复** — Ky-fold（名词）/ Ky-folding（动名词）规范词性变化保留，"Ky-folded" 第三形态已消除；S2D 首次定义后全文一律缩写，小写残留已清零 |
| 5 | §5.2 重复 [CHECK] 合并（轻微） | §5.2 L274 末段仅 1 处 [CHECK]（"S2D 加速器领域引用谱系"）。L270 段落已无 [CHECK] 标记。frontmatter L12 也明示 "本轮 polisher 修正第 1 轮按 reviewer 建议合并 §5.2 内部一处重复 [CHECK]，从 46 → 45" | **修复** — §5.2 内部 [CHECK] 已合并到末段一处，标记总数从 46 → 45，与修改建议完全一致 |

### 回归性

抽样核对未触碰已 PASS 部分：

- **claim 强度**：§5.4 L288 "comparable in spirit to the 97%... not strictly head-to-head" / §7.6 L438 "same direction, different scale point" / §8.1 L464 "We deliberately avoid claiming...strictly dominates" — 全部克制腔调与"with documented optimization roadmap"框架未变，无意外上调
- **narrative A/B 主线**：§1.4 L52 "narrative A (compiler-side PE utilization, primary axis) and narrative B (row-streaming and multi-core, system support)"、§3.6 五轴定位、§7 章引言对 narrative 的归位 — 三处叙事框架完全一致
- **frontmatter [CHECK]/[TBD] 统计**：L12 "[TBD]=14 / [CHECK]=45"，与 §5.2 合并后的 46 → 45 一致，无意外标记增减
- **HTML 来源注释 / Tab 重编号 / §3.6 五轴 / §1.3 wgt_buffer side-feed**：第 1 轮已 PASS 部分均未触碰

**无回归性问题，无新 writing 缺陷引入**。

### 通过原因

5 处问题全部按第 1 轮修改建议精确落实：
- 2 处中等问题（§7 章首引言段 + §6→§7 边界）通过补 80 字引言段一次性联动修复，narrative A/B 数据支撑明确指派到具体节
- 3 处轻微问题（§4.3 加粗 / 术语统一 / [CHECK] 合并）逐一修复且执行干净
- 修复未触碰任何已 PASS 部分（claim 强度 / narrative 主线 / frontmatter / Tab 编号 / HTML 注释）
- 未引入新 writing 问题

按判定标准 "PASS = 零问题或仅有 ≤3 处轻微建议"，本次 5 处问题全数闭环、零残留、零回归，给予 **PASS**。

### 提示（不影响判定）

- §4.4 L232 "[TBD: §4.4 是否合并入 §4.2 由 polisher 阶段决定]" 等 polisher 裁决类 [TBD] 仍残留 — 第 1 轮已记为 [TBD] 类决策不计 writing 缺陷，本轮维持原判
- §8 仅 §8.1 一节、标题重复 "Conclusion" — 第 1 轮提示项，本轮维持原判，待用户最终裁决
