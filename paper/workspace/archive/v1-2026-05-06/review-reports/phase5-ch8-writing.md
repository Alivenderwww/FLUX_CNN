# 写作评审报告 Phase 5 — §8 Conclusion

## 第 1 次评审

### 判定：PASS

---

### 评审范围

仅 §8 Conclusion（paper.md L462–L470，3 段）+ §7.7 末尾过渡句（L458）。未评 §1–§7（已 PASS）。

---

### 段落长度

| 段 | 主题 | Writer 自报 | 实测中文等效字数（按英文词×~1.6 折算） | 评估 |
|---|---|---|---|---|
| §8.p1 (L466) | 论点回顾：A+B 双叙事 + 边界声明 | ~320 | ~310-330 | 合规 |
| §8.p2 (L468) | 三线证据汇总（functional / closure / throughput） | ~350 | ~340-360 | 合规 |
| §8.p3 (L470) | Future work 按 ROI 排序（4 项） | ~380 | ~370-390 | 合规 |

3 段，conclusion 章节段落略长（300-400 字）属学术惯例可接受范围（参考 ISCA / MICRO conclusion 普遍 250-400 字/段）。无 >450 字超长段，无 <80 字短段堆叠。**通过**。

---

### 主题句结构

- **§8.p1** 开句 "This paper has argued, on a fixed 16×16 INT8 spatial array implemented on FPGA, that the chronic PE underutilization ... can be addressed entirely on the compiler side, without any runtime hardware reconfiguration." — 主题句完整、聚焦、给出了核心论点，后续 3 句分别支持（A 叙事、B 叙事、对 Eyeriss-v2/FlexFlow 的 scope 边界声明）。结构清晰。
- **§8.p2** 开句 "The evidence supporting this argument is assembled in §7 along three lines." — 经典 "thesis + 三点支撑" 句型，后续 First/Second/Third 严格对应。**结构标杆**。
- **§8.p3** 开句 "Future work follows the ROI ordering of STATUS §4 rather than a chronological roadmap." — 主题句明确 future-work 组织原则（ROI 而非时间），后续按 (1)/(2)/(3)/(4) 顺序展开。结构清晰。

3 段全部主题句明确，无埋句。**通过**。

---

### 段间承接

- **§7.7 末段 → §8.p1**：§7.7 末段 "the next chapter closes the paper with a high-level summary that re-ties the two narratives ... back to the contributions taxonomy of §1.4 without inflating any claim" 已显式预告 §8 的两件事（双叙事回扣 + 不夸大 claim）。§8.p1 立即兑现："This paper has argued ... narrative A ... narrative B ... We deliberately avoid claiming ..." — **对照工整**，承接顺畅。
- **§8.p1 → §8.p2**：p1 立论（A+B 双叙事），p2 用 "The evidence supporting this argument is assembled in §7 along three lines" 直接接证据，逻辑链 claim→evidence 标准。
- **§8.p2 → §8.p3**：p2 末句 "bound the contribution scope without inflating it"（已知贡献范围），p3 自然转向 "已知不足如何修"（future work）。从"已做到的"到"待做的"逻辑过渡，无断裂。

**3 处承接全部通过**。

---

### 与 §1–§7 风格一致性

- **句长**：与 §7 一致，长复合句为主（§8.p1 主句一句横跨 ~80 词，与 §7.4 / §7.5 同规格），但每段都有 1-2 句较短的 frame 句缓冲节奏。
- **引用 short tag**：`[EyerissV2-JETCAS19]` / `[FlexFlow-HPCA17]` / `[Gao@ASPLOS'19]` 与 §1-§7 引用风格一致（混合 CamelCase-Venue 与 Author@Venue 两种 tag 体系，§1-§7 也是这样混用，无新引入风格）。
- **"we" 频率**：§8.p1 出现 2 次 "we"（"We deliberately avoid claiming ..." / "we claim only that ..."），§8.p2/p3 隐去主语用被动 + 名词化。整体频率与 §7 一致，无突兀的第一人称增减。
- **缩写一致**：A/B 叙事、Cin/Cout、Ky-fold、S2D、row-ring、N-way W-slice multi-core、SDP、`mac_pe`、`mac_array`、ResNet-18-style、INT8 — 全部沿用 §1-§7 既定术语。
- **数字格式**：68.4 MHz / 86.6% / 593k cycles / 17K LUT / 100+ MHz — 与 §7 完全一致（k/K 大小写、空格、百分号位置）。

**通过**。

---

### 避免 cliché 收尾

Writer 自称避免了 "In conclusion, we have presented..." cliché。核实：
- §8.p1 开头 "This paper has argued, on a fixed 16×16 INT8 spatial array implemented on FPGA, that ..." — **不是** cliché 起手，且 "argued" 一词比常见的 "presented / proposed / introduced" 更带 thesis-defense 意味，**优于平均水平**。
- 全 §8 无 "In summary"、"To conclude"、"In this paper, we have proposed"、"As future work, we plan to" 这类典型套话。
- §8.p3 future work 起句 "Future work follows the ROI ordering of STATUS §4 rather than a chronological roadmap" — 有具体内容（ROI vs 时间），不是空话。

**完全通过 cliché 检查**。

---

### Conclusion 章节腔调

- **是否 well-rounded ending**：是。三段结构 = (1) thesis 复述 + scope 边界 → (2) 三线证据汇总 → (3) future work ROI 排序，覆盖 "what we argued / what we measured / what is next" 三个 conclusion 的标配维度。审稿人读完不会觉得仓促。
- **是否仓促**：不仓促。1050 字 / 3 段在 FPGA 加速器类论文 conclusion 中属偏长但合理范围（典型 600-1200 字）。
- **是否软弱**：不软弱。§8.p1 第三句虽然主动让步给 Eyeriss-v2/FlexFlow（"We deliberately avoid claiming that compiler-side folding strictly dominates ..."），但紧接 "we claim only that, on this class of shallow-layer pathology and on this fixed geometry, a pure compile-time pass reaches a comparable utilization regime at zero RTL cost" — **让步即 reframe**，把 scope 限定在自己确实做到的范围里，是学术诚实而非软弱。
- **是否重复 §7**：不重复。§8.p2 三线证据是对 §7.2-§7.5 的**汇总抽象**，给出的是"这三个数字一起证明什么"而不是"这三个数字本身是多少"，重复度低。

**通过**。

---

### 学术腔调

- **第一人称**：仅 §8.p1 用了 2 次 "we"，且都用在 scope-limit 让步处（"We deliberately avoid", "we claim only"），语境恰当。
- **buzzword 检查**：通读三段，无 "novel"、"revolutionary"、"state-of-the-art"、"unprecedented"、"groundbreaking"、"cutting-edge" 等。仅 "architecturally significant"（§8.p3）一处稍带主观色彩，但其后立刻被 "parallel to Tangram [Gao@ASPLOS'19] and combinable with ..." 两个具体限定句锚定，不算空 buzzword。
- **claim 强度**：完全克制。"reaches a comparable utilization regime"（不说"超越"）、"timing-neutral"（不说"无开销"）、"projected outcome"（强调 projected 而非已验证）、"medium-term direction"（不说"will deliver"）。
- **时态**：thesis 用现在完成时（"This paper has argued"），方法/数据陈述用现在时（"The two transformations ... jointly recover"），证据用现在时（"all pass bit-exactly", "closes timing", "reaches 86.6%"），future work 用现在时陈述事实 + 投射型 "the projected outcome is"。**全段时态规范**。

**通过**。

---

### 节奏与句长

- §8.p1 长短交替：长复合（thesis 句）→ 长复合（A+B 联合句）→ 中等让步句 → 中等 reframe 句。无连续 ≥3 句相同句式。
- §8.p2 用 "First / Second / Third" 三段并列结构，每个 line 一句长说明 + 括号注，节奏稳定且明显，读者好跟。
- §8.p3 用 "(1)/(2)/(3)/(4)" 编号 + "next in the queue" / "the most architecturally significant medium-term direction" 等过渡句调节节奏，避免变成 4 项干列表。

**通过**。

---

### 术语一致性

抽查与 §1-§7 对照：

| 术语 | §8 用法 | §1-§7 用法 | 一致 |
|---|---|---|---|
| Ky-folding / Ky-fold | "Ky-folding for `Cin < 16`" | "Ky-fold" / "Ky-folding"（混用） | 一致（§1-§7 已混用，§8 沿用） |
| Space-to-Depth / S2D | "Space-to-Depth for `stride ≥ 2`" | "Space-to-Depth (S2D)" | 一致（§8 用全称，前文已展开 S2D） |
| row-ring | "the row-ring streaming data path" | "row-ring" / "streaming row-ring" | 一致 |
| N-way W-slice multi-core | "the N-way W-slice multi-core fabric" / "N = 4 multi-core" | 同 | 一致 |
| SDP | "SDP quantization combinational chain" | 同 | 一致 |
| `mac_pe` / `mac_array` | inline code | inline code | 一致 |
| 11-layer ResNet-18-style chain | 同 | 同 | 一致 |
| narrative A / narrative B | "narrative A" / "narrative B" | §1.4 contributions taxonomy 同 | 一致 |

**全部通过**。

---

### Future work 段落语气（§8.p3）

- **诚实**：明确点出了 4 个待办项的工作量估算（"completable before camera-ready" / "1–2 day integration" / "2–3 days" / "medium-term"），不画大饼。
- **有方向感**：按 ROI 排序而非清单堆砌，且每项都给出"做完之后会消除什么 gap"（resynthesis → 100+ MHz / 17K LUT；multicore 测量 → 替换 §7.5 投影数；P2 → 消除 inter-stage DDR round-trip；fusion → 与 Tangram 平行）。读者能看出团队知道下一步该做什么、为什么这么排。
- **不软弱**：未使用 "we hope to / we plan to eventually / future versions might" 这类弱化语气。最弱的一句是 "remain as separate RTL extensions whose scope and design have been outlined in §7.7 but not implemented in this work"，但这是 honest scoping，不是 hand-waving。
- **限定具体**：把 Tangram 作为对标 [Gao@ASPLOS'19] 锚定 fusion 方向，把 Pooling/Depthwise/sparsity 与 §7.7 互引锚定 RTL 扩展，避免空喊。

**通过**。语气是 "honest and purposeful"，符合优质 FPGA paper conclusion 的 future work 段标准。

---

### 通过-失败汇总

| 检查项 | 结果 |
|---|---|
| 段落长度合规 | PASS |
| 段落主题句结构 | PASS |
| 段间承接（§7.7→§8 / 段 1→2→3） | PASS |
| 与 §1-§7 风格一致性 | PASS |
| 避免 cliché 收尾 | PASS |
| Conclusion 章节腔调 | PASS |
| 学术英文腔调 | PASS |
| 句长与节奏 | PASS |
| 术语一致性 | PASS |
| Future work 语气 | PASS |

**10/10 通过**。

---

### 修订建议（≤3 处轻微，不影响 PASS）

1. **§8.p2** "all pass bit-exactly within the 100k-cycle and 200k-cycle watchdog windows respectively" — "respectively" 在前文是 "46 single-core ... and 20 multi-core ..." 两组并列的对应关系，理论上可读，但读者需要在 100k/200k 与 single-core/multi-core 间手动配对。**建议**改为 "all pass bit-exactly within their respective watchdog windows (100k cycles for single-core, 200k for multi-core)"，配对显式化。**纯可读性微调，可不改**。

2. **§8.p3** "The most architecturally significant medium-term direction is cross-layer streaming fusion, parallel to **Tangram [Gao@ASPLOS'19]** and combinable with the existing Cout-slicing and stage-barrier infrastructure" — 单句横跨 fusion 定位 + 对标 + 可组合性 三个信息密度过高。**建议**拆为两句："The most architecturally significant medium-term direction is cross-layer streaming fusion, parallel to **Tangram [Gao@ASPLOS'19]**. It is combinable with the existing Cout-slicing and stage-barrier infrastructure." **可改可不改**。

3. **§8.p1** "the chronic PE underutilization observed on shallow ResNet-style layers — where `Cin ∈ {3, 4, 8}` collapses array width usage to between 12.5% and 50%" — "collapses ... to between 12.5% and 50%" 范围是降到的区间，"collapses" 是动作动词。语法没错，但 "collapses to between X and Y" 句式略生硬。**建议**改为 "drops array width usage into the 12.5%-50% range"，更顺。**纯风格微调**。

> 提示：§8.p2 "86.6% network-level MAC utilization in 593k cycles"、"68.4 MHz Fmax"、"17K LUT"、"100+ MHz" 等数字，writing 不审真伪；建议 tech 评审复核与 §7.4 / §7.3 / STATUS §4 的数字对齐。
