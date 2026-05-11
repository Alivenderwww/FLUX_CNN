# 新颖性评审报告 Phase 7（最终全文）

### 判定：PASS

---

## 第 1 次评审

### 评审范围
- 主产出：`paper/workspace/paper.md`（润色后全文 467 行 / 8 章 + frontmatter）
- 上一阶段参考：`literature.md` / `contributions.md` / `outline.md`（已 PASS，未重读）
- 视角：最终新颖性 — claim 强度全文一致 / prior art 应对最终态 / narrative A+B 一致性 / 整体定位强度 / Reject 风险预判
- 假想审稿人：
  - **Reviewer A（系统派）**：会问"工程贡献够 architecture novelty 吗？"
  - **Reviewer B（算法派 / 数据流）**：会问"你的 OS-with-column-broadcast 跟现有 OS 有何区别？S2D 早就被人做过吧？"
  - **Reviewer C（FPGA 派）**：会问"Liu 97% MAC% 在 Arria 10 上做到，你 86.6% 在 7-series 上做到，到底是 scale 不同还是你方法弱？"

---

### 1. claim 强度全文一致性（Polisher 自报已 hedge ~10 处）

通读全文清点 "novel / first / to our knowledge / distinguishing" 类强词出现位置与 hedge 状态：

| 位置 | 原 claim 强度 | 当前态 | 评估 |
|------|--------------|--------|------|
| §1.4 C(1) Ky-fold | "Our literature search ... did not return a direct prior art" | 加 [CHECK: 引用谱系待 reviewer 阶段补查] | 合理 hedge — 用"未检索到"代替"首次"，且明示待复核 |
| §1.4 C(2) S2D | "to our knowledge, distinct from the training-time use" | 限定到"applying it as a compiler pass over an arbitrary pre-trained convolution — without retraining" | 合理 hedge — 把新颖性收缩到"compile-time + 不重训"这一具体动作上 |
| §3.3 末尾 | "to our knowledge, not directly named as such in the surveyed works" | 同上 + [CHECK] | 合理 |
| §3.6 末尾 | "principal threat to narrative A on axis (2), by contrast, is the absence of a directly comparable prior compiler-side Ky-fold + S2D work, which we have flagged for follow-up rather than asserted as proof of novelty" | 主动声明"未做 proof of novelty" | 这是非常诚实的 hedge — 不打"独有"牌而打"待复核"牌，反而更经得起审稿 |
| §5.1 末尾 | "To our knowledge, packaging this Ky-only fold as an ahead-of-time pass on a fixed 2-D MAC array, with no runtime configuration and no per-layer microcode, is a distinguishing point of this work" | 限定到"Ky-only + ahead-of-time + fixed array + no runtime config + no microcode" 五重定语 | 合理 — 任意一个限定条件被 reviewer 找出反例都可以局部 retreat |
| §5.2 末尾 | "appears to be less standard in the published accelerator literature" | "appears to" + [CHECK] | 合理 hedge |
| §5.4 §1 句 | "comparable in spirit to the 97% reported by Liu" + "not strictly head-to-head" | 同句立即自我软化 | 合理 |
| §5.4 §3 | "we do not claim the compiler-only route dominates" | 明示不做 dominance claim | 关键诚实立场 — 见下文"整体定位强度"段 |
| §7.6 §3 | "We do not contest Liu's number; ... 'same direction, different scale point,' not 'their number is better than ours.'" | 明示放弃"打败 SOTA"叙述 | 合理 — 把 86.6 vs 97 转成"不同 Pareto 点"框架 |
| §8.1 §1 | "We deliberately avoid claiming that compiler-side folding strictly dominates the hardware-reconfigurable route" | 明示 | 合理 |

**结论**：全文 ~10 处强 claim 都已 hedge 到具体限定条件 + [CHECK] 或主动放弃 dominance。**未发现遗漏的过强 claim**。

唯一边缘案例 — §4.3 "We frame 'unified streaming row-ring' as the **sole** runtime mode of FLUX_CNN" — 这里"sole"是描述自家设计而非 vs 文献的强 claim，OK。

---

### 2. prior art 三大威胁应对最终态（Alwani / Kang / Liu）

#### Alwani [Alwani-MICRO16]（layer-fused 多层共驻）
- §1.2 出现 → §2.3 三段对比 → §3.4 §2 段（"clearest threat"明示）→ §3.6 axis (3) 编码
- 差异化论证：**granularity + resource regime** — Alwani 多层共驻要求 BRAM 随融合层数 scale，FLUX_CNN 单层 + DDR 跨层
- 关键句："On a 1.6 MB BRAM budget for VGA-scale inputs, the Alwani regime does not fit; FLUX_CNN's row-level granularity is therefore not a strict improvement over fused-layer execution but a different point on the streaming-vs-fusion design space"
- **评估**：诚实 + 清晰 — 不打 dominance 牌而打 design-space 牌，Reviewer A 难以 reject

#### Kang [Kang-AoCStream-arXiv22 / Kang-Sensors23]
- §1.2 出现 → §2.3 同段 → §3.4 §3 段（专段处理）→ §3.6 编码
- 差异化论证：**layer-pipelined-multi-block vs layer-serial-single-core + on-chip-only vs DDR-as-inter-layer-buffer**
- 关键句："The 'all-on-chip' assumption is well-suited to the small detection networks Kang targets but does not extend to the larger networks at VGA-scale resolution"
- **评估**：差异化清晰，明确指出 Kang 假设的边界

#### Liu [Liu-FullStack-TNNLS21]（86.6 vs 97 焦点比较）
- §1.2 / §2.3 / §3.3 末尾 / §3.4 §4 段 / §5.4 §1 / §6.3 / §7.4 末尾 / §7.6 §3（专段）
- 差异化论证：**scale point** — Arria 10 GX1150 vs XC7K325T，~10× BRAM/DSP class
- §5.4 + §7.6 一致性核：
  - §5.4 §1：86.6% / Arria10 / 97% / "not strictly head-to-head" / "different devices, different workloads, different metric definitions"
  - §7.6 §3：86.6% / Arria10 GX1150 / 97% / "same paradigm at very different scales" / "same direction, different scale point"
  - 两处叙述完全一致，无矛盾。
- **评估**：86.6 vs 97 论证经过 4 处反复说明 + 2 处"not head-to-head" hedge，Reviewer C 找不到漏洞

#### 三大威胁外加：MAERI / Eyeriss-v2（hardware-reconfigurable）
- §1.2 / §3.2 / §5.1 / §5.4 / §7.2 末尾 / §8.1
- 差异化论证：**compiler-side vs hardware-side** — 一致使用"opposite point on same design space"框架
- §3.2 末尾："The two stances are not mutually exclusive — one could in principle combine them" — 这是漂亮的 hedge

---

### 3. narrative A+B 主线全文一致性

主线追踪 §1 → §5 → §7 → §8：

| 阶段 | A (compiler-side PE utilization) | B (row-streaming + multi-core) | C (decentralized handshake) |
|------|----------------------------------|-------------------------------|------------------------------|
| §1.1 motivation | 12.5%-50% PE 利用率 pathology | 4.9MB image vs 1.6MB BRAM | — |
| §1.3 approach | Ky-fold + S2D | row-ring + multi-core W-slice | "organizing principle, not top-line claim" |
| §1.4 contributions | C(1) + C(2) | C(3) + C(4) + C(5) | (内含 §4.2 但不算入 5 条 contributions — 一致) |
| §3 related work | §3.2 + §3.3 + §3.6 axis (2)(5) | §3.4 + §3.6 axis (3)(4) | §3.6 axis (1) |
| §4 architecture | — | §4.3 row-ring | §4.2 |
| §5 compiler | §5.1 + §5.2 + §5.3 + §5.4 主章 | — | — |
| §6 system | §6.1 PyTorch | §6.4 multi-core + §6.2 DMA + §6.3 SDP | — |
| §7 eval | §7.2 PE util by mode | §7.3 / §7.5 / §7.4 | §7.6 末尾 vs Gemmini 中央 FSM |
| §8 conclusion | §8.1 §1 提 A | §8.1 §1 提 B | §8.1 §1 未提（与 §1.3 "not top-line claim" 一致） |

**A+B 闭环**：§1 promise → §5/§6 实现 → §7 量化 → §8 回顾 — 完全一气呵成。
**C 的处理**：§1.3 主动声明"organizing principle rather than a top-line claim"，§8.1 不在 closing 提，**完全一致**，反而符合诚实立场。
**评估**：narrative A+B 主线无切换混乱，C 作为支撑而不掺杂主轴，编排合理。

---

### 4. 整体定位强度（Polisher 担心"作者太谦虚"）

Polisher 自报 "deliberately avoid claiming compiler-side folding strictly dominates" — 担心削弱定位。

**审稿人视角分析**：

- **Reviewer A（系统派）**会读 §5.4 §3 + §8.1 §1 的"not dominates"声明 → 反应：这是诚实工程立场，不会因此判 reject。系统派审稿人对"打败一切"的论文反而警惕。
- **Reviewer B（算法派）**会读 §5.2 + §3.3 末尾对 Shi sub-pixel 的 ack → 反应：S2D 数学不是新的本来就是事实，作者主动 ack 加分而非减分。
- **Reviewer C（FPGA 派）**会读 §7.6 §3 "same direction, different scale point" → 反应：相比硬打"我们打败 Liu"被 desk reject 强一万倍。

**定位强度自查**：
- 是否仍有可销售的核心 claim？✅ 有 — §1.4 五条 contributions，**特别是 C(1) + C(2)**（compiler-side fold on fixed array, no RTL change, bit-exact regression 46+20=66 cases pass）+ C(4)（W-slice multi-core 20/20 pass + N=4 close timing）这两块是工程级别经得起 reproducibility 审查的硬贡献
- 是否落入"无贡献"嫌疑？❌ 否 — §1.4 contributions 列表清晰，§7 数据章 5 个 Tab 支撑（即便 [CHECK] 多）
- 整体定位框架："a different point on the Pareto frontier" — 这是被 ISCA/MICRO/FPGA 共识接受的合法 framing，不弱

**评估**：Polisher 担心的"过度谦虚"并未发生 — hedge 都集中在"vs 文献的相对强度"上，自家工程贡献的 claim 仍然清晰可量化。**整体定位强度合适**。

---

### 5. Reject 风险预判（最终态）

| 风险类型 | Polisher 担心 | 实际审稿人会怎么反应 | 严重度 |
|---------|--------------|---------------------|--------|
| 贡献分散（5 条） | 是否过多 | §1.4 末"alternative organizations are equally defensible [TBD]" 主动 ack — Reviewer 不会因此 reject | 低 |
| 缺少 SOTA 比较 | §7.6 head-to-head 是否够 | Tab.6 + Tab.7 列出 10 篇 baseline + 5 axis 对比，结构完整。86.6 vs 97 已用"different Pareto point"重 framing | 中（Tab.6 数字未实测，依赖 [CHECK] 闭环） |
| [CHECK]=46 影响可信度 | 主要在 §7 数据章 | Reviewer 会要求 camera-ready 前补全，但不至于 reject — 关键是 [CHECK] 都附"待哪个文档/重综合补"具体可执行清单。**风险主要在 §7.4 86.6% MAC% 和 §7.5 1.45× speedup 这两个核心数字未实测时是否会被 reject** — 但 §7.5 已主动声明 scheduler 投影 vs 实测的边界 | 中 |
| narrative 切换太多 | A+B 混合定位 | 见上节追踪表，A+B 主线一气呵成，无切换 | 低 |
| Cout<16 不优化 | §5.3 / §7.7 主动声明 | 主动 ack 限制 + workload 论证 negligible — 合理 trade-off claim | 低 |
| 单核 + 单器件评估 | 未对比多器件 / 多 workload | §7.7 (c) 列出 Pooling/DW/Sparsity 未做 + ResNet-18-style 单一 workload — 这是 *真正* 的潜在 reject 风险，但 paper 主动声明 future work | 中 |

**总体 Reject 风险评估**：中等偏低。论文主动声明所有限制 + 用 [CHECK] 标注待补数字 + 不打 SOTA dominance 牌 — 是 FPGA / TRETS / TVLSI 类期刊接受度较高的写法。最大风险是 [CHECK] 数据未在 camera-ready 前闭环（这是 Phase 8 的事，不是 novelty 评审的事）。

---

### 6. §7.7 Limitations 诚实立场是否削弱论文？

§7.7 列三条限制 (a) Fmax 68.4MHz < 100MHz target / (b) Cout<16 column idling / (c) Pooling/DW/Sparsity 未做。

**审稿人反应预判**：
- (a) Fmax 短缺：附两条具体修复路径（use_dsp attribute + SDP 流水化）+ STATUS §4 列为 highest ROI 项 → Reviewer 会理解为"已诊断未修"而非"不知道问题"，**比假装没问题强**
- (b) Cout idling：附"affected layers contribute negligible MAC%" workload 论证 → 合理 trade-off
- (c) RTL extension 未做：明示 future work + 给出 (c) 的难度（"depthwise inverts Cin/Cout broadcast pattern"）→ 显示理解深度

**评估**：§7.7 是论文加分项而非减分项。FPGA 圈 reviewer 普遍欣赏"诚实工程报告"。Polisher 担心多余。

---

### 7. frontmatter [TBD] 合理性

| 项 | 当前态 | 评估 |
|---|--------|------|
| title | "[TBD: 标题最终待用户定 — 候选见 outline.md，建议 'FLUX_CNN: A Streaming Row-Ring CNN Accelerator with Compiler-Driven PE Utilization on Xilinx 7-Series FPGAs']" | 合理 — 给出建议候选，user 决定 |
| authors | [TBD] | 合理 — 用户必填项 |
| venue | "[TBD: 目标会议/期刊 — Phase 1 倾向 FPGA / FCCM / TCAD / TVLSI / TRETS]" | 合理 — 列出候选会议梯度 |

**评估**：3 处 frontmatter [TBD] 都是 user-decision 类，不是评审期可闭环的事项。合理。

---

### 8. 通过-失败要点

**通过依据**：
1. ~10 处强 claim 全部 hedge 到具体限定条件 + [CHECK] 或主动放弃 dominance — **无遗漏强 claim**
2. Alwani / Kang / Liu 三大威胁各自有专段差异化论证 + axis (3) 框架编码，无漏应对
3. §5.4 vs §7.6 关于 Liu 86.6 vs 97 的论证两处叙述完全一致，无矛盾
4. narrative A+B 主线 §1→§5→§7→§8 完整闭环，C 作为支撑不混入主轴
5. 整体定位"a different Pareto point"是合法 framing，不削弱
6. §7.7 limitations 主动声明所有限制 — 加分项
7. frontmatter [TBD] 都是 user-decision，合理

**未触发任何 FAIL 条件**：
- 无过度声明（"SOTA / 独有 / 首次"无支撑）— ✅
- 无贡献无法与文献区分 — ✅
- 无漏掉近期同领域工作（Alwani / Kang / Liu / MAERI / Eyeriss-v2 / Tangram / Simba / Snowflake / Angel-Eye / Aydonat / Lu / fpgaConvNet / Gemmini / NVDLA / TPU / Eyeriss / ShiDianNao / VTA / TVM / Timeloop / Interstellar / Ma / Buffets / Shi / Jacob / He / EIE / ESE / cuDNN — 26+ 篇覆盖完整）— ✅
- Intro promise 全部在正文兑现 — ✅
- 无 over-hedging 削弱整体定位（hedge 集中在"vs 文献"，自家工程贡献仍硬）— ✅

---

### 9. 修订建议

无 — 评审通过。如 user 在 camera-ready 前希望进一步加固定位，可考虑（**非 FAIL 项，仅可选**）：
- 在 §1.4 contributions 列表前加一句"these contributions are quantified by 66 bit-exact regression cases (46 single-core + 20 multi-core)"，把"reproducibility"作为隐性新颖性要素（许多 FPGA 论文不开源、不附回归套件）
- 在 §3.6 五轴定位段增加一行"FLUX_CNN's distinctive cell"摘要句（Tab.7 已做，此处仅为可视化加强）

但以上都不是 FAIL 项，paper 当前态可直接进入 Phase 8 数据/引用复核。

---

### 定位摘要（≤3 句）
- **本工作定位**：fixed 16×16 INT8 array + compiler-only PE utilization 恢复（Ky-fold + S2D） + row-ring streaming + W-slice multi-core，目标 mid-range edge FPGA（XC7K325T 类）
- **与主要对手差异**：vs MAERI/Eyeriss-v2 = compiler-side vs hardware-reconfigurable（不同设计哲学）；vs Alwani/Kang = row-level single-layer-at-a-time vs multi-layer-resident（不同 BRAM 体量）；vs Liu = same paradigm at different scale point（86.6% on 7-series vs 97% on Arria 10 GX1150）
- **主要风险点（仍可接受）**：[CHECK]=46 核心数据未在 Phase 7 完成实测闭环（待 Phase 8）；ResNet-18-style 单一 workload 评估（已主动声明）；Fmax 68.4MHz < 100MHz target（已诊断 + 列 highest-ROI 修复路径）
