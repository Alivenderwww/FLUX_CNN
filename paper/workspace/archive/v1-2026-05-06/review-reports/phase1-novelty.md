# 新颖性评审报告 Phase 1

## 第 1 次评审

### 判定
PASS（中等强度通过——18 条贡献的强度自评谨慎、所有"偏强"claim 都已对照 literature.md 排查、3 处主要 prior art 威胁均显式列出并附差异化角度。存在 4 处中等问题、3 处轻微问题，建议 Writer 在 Phase 2 outline 阶段顺手收敛，但都不到 FAIL 阈值。）

---

### 评审范围

**评的是**：
- 18 条贡献的 claim 强度（自评偏强 1 / 中-强 2 / 中 1 / 弱-中 3 / 工程化 11）逐条对照 literature.md
- 3 处已识别 prior art 威胁（Alwani / Kang / Liu）的差异化角度是否经得起审稿挑战
- literature.md A–G 七类中是否还有"已列出但被 Writer 忽略"的潜在威胁
- narrative A（compiler）+ B（streaming+multicore）混合定位的可行性
- 18 条主角/配角分布是否合理
- 弱-中 claim 是该保留为工程化叙述还是砍掉
- 16 [CHECK] + 7 [TBD] 中哪些是必须在 Phase 2/3 前消除的、哪些可推到 reviewer 阶段

**不评**：
- 数据真实性（PE 利用率实测、综合数字、cycles 等——tech reviewer 的活）
- 文献是否真实存在（Phase 0 已用 Elicit 核完）
- 句式 / 段落结构（writing reviewer 的活）

---

### claim 强度评估

逐条核 18 条贡献，按"过强 / 合理 / 偏弱"分类。**重点检查每个"独有 / 首次 / 显著 / SOTA"等强词的证据支撑**。

#### 偏强声明（只有 1 条，符合自评）

| # | 贡献 | 自评 | 评审判定 | 备注 |
|---|------|------|---------|------|
| C2.1 | Ky-fold | "claim 偏强" | **合理** | Writer 用"literature.md §D 检索未命中"作为措辞——这是合规的 phrasing（不是"首次"，是"已检索未命中"）。MAERI / Eyeriss-v2 / cuDNN im2col 作为对比对象列得清晰。**风险**：reviewer 可能拿出某篇 ASPLOS/HPCA architecture-side fold 的 prior art。**缓解**：Writer 已在 contributions §8.1 #1 把 PE 利用率实测列为最先消除项；只要实测数字撑得住，这个 claim 即使有 prior art 露头也可改 framing 为 "extends X to FPGA + INT8 + compiler-only"。 |

#### 中-强声明（2 条，符合自评）

| # | 贡献 | 自评 | 评审判定 | 备注 |
|---|------|------|---------|------|
| C2.2 | S2D | "claim 中等偏强" | **合理但需补查** | Writer 自己已 flag "S2D 引用谱系 reviewer 阶段需补查"（contributions §C2.2 + §8.1 #15），且引用 Shi@CVPR'16（算法侧）作为参考。**风险**：本 claim 是 narrative A 的第二支柱，若 reviewer 找出 prior art 会同时削弱 C2.1+C2.2 两条。**缓解**：建议 Phase 2 outline 完成前由主 Agent 用 Elicit 跑 1 次 "space to depth FPGA accelerator compiler"+"channel rearrangement stride convolution accelerator" 关键词（约 1 次预算）做"反向检索"，把 [CHECK #15] 从 reviewer 阶段提前到 Phase 2。 |
| C3.7 | W slice multicore | "claim 偏强" | **合理** | "computed redundancy halo + asymmetric pad + 整层 stride 标记 + DDR row_stride 解耦 cmd_btt" 是具体的、组合式的 mechanism-level claim，literature.md 中确无完全对位 prior art。Simba（channel-切分）是对照点，方向不同。**轻微提示**：建议在差异化句中显式对照 Liu Full-Stack@TNNLS'21（也是 streaming+多并行）和 Aydonat DLA@FPGA'17（Arria 10 上 1D 数据流），表明 W-slice 与他们的 multi-block 路线正交。 |

#### 中等声明（1 条）

| # | 贡献 | 自评 | 评审判定 | 备注 |
|---|------|------|---------|------|
| C1.2 | row-ring streaming | "claim 偏弱" (Writer 自评) | **判定合理偏弱即可** | Writer 在差异化段已诚实承认 Alwani / Kang / Liu 是"最近邻 prior art"并列出 3 条差异点（row-level vs layer-level granularity / layer-serial vs layer-pipelined / 承认外存 vs 全片上）。这种 framing 是经得起审稿的。**没有发现过度声明**。 |

#### 偏弱声明（3 条）

| # | 贡献 | 自评 | 评审判定 | 备注 |
|---|------|------|---------|------|
| C1.1 | 去中心化握手 | "工程化贡献，并非首次" | **合理** | Writer 显式承认 Buffets@ASPLOS'19 提供了形式化语言、自身只是 RTL 落地。这是诚实的 framing。**保留作为工程化叙述**（详见后段"弱-中 claim 处理建议"）。 |
| C1.4 | SDP residual fusion | "偏增量" | **合理** | NVDLA-inspired + Liu 也支持，已显式承认。R.1 bias 重定位 + R.2 shortcut_mult/shift 是有价值的具体优化，但不撑得起主 claim。Writer 在 §7.3 已建议作为 "system 完整性" 出现，**保留**。 |
| C3.5 | multicore wrapper | "工程化贡献" | **合理** | N=2/4 综合 + 仿真验证有 system completeness 价值。**注意**：若主 Agent 选 narrative B（streaming+multicore），此条会从"配角"升为"主角"，届时强度自评应同步修订（升为"中"）。 |

#### 工程化贡献（C1.3 / C1.5 / C2.3 / C2.4 / C2.5 / C3.1 / C3.2 / C3.3 / C3.4 / C3.6）— 共 10 条
均为系统集成 / 实现细节 / 验证基础设施类，Writer 一律标"工程化贡献"，**无过度声明**。其中 C3.6 (params.py) Writer 已在 §7.3 建议"不写入 contribution"，是恰当的。

---

#### 总结：claim 强度无 FAIL 级问题

- **没有"首次 / 独有 / SOTA / 显著优于"等无证据强词**——所有强 claim 都用"literature.md 检索未命中"的合规 phrasing
- **数字 claim 都按"target vs Fmax"两口径同时报告**——避免选择性披露（[CHECK #4] 在 §8.1 已 flag 主 Agent 决定文中口径）
- **PE 利用率"接近 100%" Writer 也加了 [CHECK]**——表明实测百分比待补，没有把估算当事实

---

### prior art 威胁评估完整性

#### Writer 已识别的 3 处威胁（§5.3 表）

| Prior art | Writer 评级 | 评审判定 | 备注 |
|-----------|-------------|---------|------|
| Alwani Fused-layer @ MICRO'16 | 🔴 高 | **同意** | 差异点（row-level vs layer-level + layer-serial 单核共用硬件）经得起审稿挑战 |
| Kang AoCStream @ arXiv'22 / Sensors'23 | 🔴 高 | **同意** | 差异点（承认外存必然 + 优化 DDR 流量；layer-serial vs layer-pipelined）合理 |
| Liu Full-Stack @ TNNLS'21 | 🟡 中 | **同意** | 单核 vs 多 block + 规模差 + 97% vs 86.6% MAC% 是同思路不同规模——这是经典审稿应对 |

#### 评审发现的额外潜在威胁

**【中等】FINN（Umuroglu et al.@FPGA'17）/ FINN-R（Blott et al.@TRETS'18）漏列延续 Phase 0**

- Phase 0 novelty review (`phase0-novelty.md`) 已经标记 FINN 是 §C 的"中等补强点"——但 contributions.md 仍未引入。
- **审稿人风险**：投 FCCM / FPL / TRETS 时 FINN 几乎是必引；漏掉会被审稿人圈"作者似乎不熟悉 streaming 加速器主流文献"。投 FPGA 也大概率被引到。
- **威胁等级**：FINN 偏向"HLS 自动生成 + per-layer streaming"路线，与 FLUX_CNN 的"手写 RTL + 单核 row-streaming"虽不正交但容易被 reviewer 当成"同类"工作做差异化追问。
- **建议处理**：Phase 2 outline 阶段由 Writer 在 Related Work 章占位时**显式列入 FINN 对照**（无需 Phase 0 重跑核验，只需主 Agent 查 BibTeX 入条）。本评审不据此 FAIL，因为 (a) 是 Phase 0 残留延续问题、(b) Alwani / Kang / Liu 已覆盖 streaming 主线索的核心威胁、(c) FINN 的不正交性使其威胁度可控。

**【轻微】Tangram@ASPLOS'19 在 C1.2 差异化中未被点到**

- Tangram 在 literature.md §B 而非 §C，但 Tangram 也做"cross-layer fine-grained forwarding"减小 buffer 需求，与 row-ring streaming 的"按行前向传"概念有相似性。
- Writer 在 contributions §C1.2 差异化句中未引 Tangram；但在 literature.md L223-228 已经列出 Tangram 作为"将来工作的直接对照"。
- **建议处理**：Phase 2 outline 中把 Tangram 作为"row-level cross-layer pipelining 是 FLUX_CNN row-ring streaming 的自然扩展、Tangram 已在 ASIC tile 上验证可行"的 framing 工具。**不影响 Phase 1 PASS**。

**【轻微】Snowflake@ISCAS'17 / Angel-Eye@TCAD'18 在贡献差异化中未被显式对位**

- 两者都在 §5.1 / §5.2 对比矩阵中，但作为最直接的 Xilinx 7-series + INT8 + line-buffer / streaming 对照，**应当在 C1.2 row-ring 或 C2.1/C2.2 PE 利用率的差异化句中点名**——尤其 Angel-Eye 与 FLUX_CNN 同 Zynq XC7Z045 量级器件、同 INT8、文献中是 fpgaConvNet 的姐妹工作。
- **建议处理**：Phase 2 outline / Phase 7 paper.md 写 Related Work 章时显式列对照。**Phase 1 contributions.md 此处不构成 FAIL**——因为对比矩阵已包含两者，差异化论证可在正文展开。

#### 总结：威胁评估基本完整

3 处主要威胁（Alwani / Kang / Liu）已显式处理；FINN 是 Phase 0 延续遗漏，Phase 2 补即可；Tangram / Snowflake / Angel-Eye 已在对比矩阵中、缺的是贡献级差异化句。**未达 FAIL（漏 ≥2 篇近期同领域工作）阈值**。

---

### 整体定位连贯性

#### narrative A + B 混合可行性分析

Writer 在 §7.1 列了 4 个 narrative 候选并在 §7.2 建议"A 主 + 部分 B 作为 system context"。本评审独立判断：

**narrative A（compiler-side PE utilization）单独成文的难点**：
- 主推 C2.1 + C2.2 + C2.4 + C2.5，**只有 4 条主贡献**，加 C3.x 工程化作为支撑
- Cout<16 不优化是 Writer 自己点出的弱点（§7.2 提到）——若 reviewer 强追"为什么 Cout 也不能 fold？"会显被动
- Ky-fold + S2D 都需要硬件支持任意 Cin / 任意 stride=1 + ARF reuse_en=1 这些 RTL 特性——硬件部分（C1.x）不能完全砍掉

**narrative B（row-streaming + multi-core）单独成文的难点**：
- Alwani / Kang / Liu 的 prior art 威胁高，必须精细差异化（Writer 已 flag）
- 单核 row-ring 是 §1.2，multi-core W slice 是 §3.7——两者实际是不同 mechanism，论文需要论证"这两件事其实是同一思想的两个尺度"
- N=2/4 实测但 ResNet 完整 multicore chain 还没跑（§8.1 #8）——若不在 Phase 2 前补，narrative B 的 multi-core 论据会显薄弱

**A + B 混合（Writer 推荐）的连贯论证**：
- 可能的统一 framing："**fixed 16×16 INT8 array with compiler-driven PE utilization and streaming row-ring scaling to multi-core**"——硬件保持简洁、编译器 + streaming 共同填补"任意层形状 + 任意图像 + 多核扩展"三个维度
- 这个 framing 是**站得住的**，因为 C2.1/C2.2 (compiler 填 PE) 和 C1.2/C3.7 (streaming + multicore) 共享同一个硬件 + 同一个 OS dataflow + 同一个 valid-ready 抽象
- **风险**：reviewer 仍可能挑"两件事"——但只要 paper.md 中 §1 Introduction 把统一 framing 摆在 contributions 之前，"X+Y 拼凑" 的 reject 把柄会消解大半

**评审建议**：A+B 混合是**可行的**，但需要 paper.md 阶段做好统一 framing。Writer 在 §7.1 [TBD] 把决策权交给主 Agent 是合理的，**Phase 1 此处 PASS**。

#### 主角 / 配角分布合理性

| 角色 | 贡献编号 | 数量 |
|------|---------|------|
| 主角候选 | C2.1 Ky-fold / C2.2 S2D / C1.2 row-ring / C3.7 W slice | 4 |
| 二级（system context） | C1.4 SDP fusion / C2.5 PyTorch 编译 / C3.5 multicore / C3.1 axi_dm 集成 | 4 |
| 配角（实现细节 / 工程化） | C1.1 / C1.3 / C1.5 / C2.3 / C2.4 / C3.2 / C3.3 / C3.4 / C3.6 | 9 |
| 不写入 contribution | params.py | 1 |

- **4 主角 + 4 二级 + 9 配角的分布对一篇会议论文是合理的**（典型 FPGA / FCCM 长文 3-5 主贡献）
- 主角中 4 条覆盖 (a) Ky-fold (b) S2D (c) row-ring (d) W slice，**形成"compiler + streaming + multi-core"三角**，narrative A+B 混合时支柱够
- **轻微提示**：narrative A 单独走时主角降到 2-3 条 (C2.1+C2.2+可能 C2.4)；narrative B 单独走时主角降到 2-3 条 (C1.2+C3.5+C3.7)。混合走能保住 4 条。这是 A+B 混合的另一支撑论据。

#### 总结：定位连贯性 PASS（中等风险）

A+B 混合可行，但需在 paper.md §1 Introduction 找到"**fixed array + compiler PE utilization + streaming multi-core scaling**"的统一 framing。Writer 在 §7.2 已经预见这一点。

---

### 弱-中 claim 处理建议

#### C1.1 去中心化 valid-ready 流水

**保留**，作为工程化叙述。**位置**：paper.md 中应作为 §3.1 Architecture Overview 的"design philosophy"出现，**不**作为 §1 Introduction 列出的 contribution。Writer 在 §7.3 已建议作为"system 完整性"出现，吻合。**审稿人风险**：低——Buffets framing 工具已就位。

#### C1.4 SDP residual fusion

**保留**，作为工程化叙述。**位置**：paper.md 中作为 §3.x SDP 后处理的描述，举 R.1 bias 重定位 + R.2 shortcut_mult/shift 作为具体优化。**不**列入 §1 contributions。Writer 在 §7.3 建议吻合。**审稿人风险**：低——NVDLA-inspired 已显式承认。

#### C3.5 multicore N=2/4

**根据 narrative 决定**：
- narrative A（compiler）走：作为 system context 二级出现，强度保持"工程化"
- narrative B（streaming+multi-core）走：升为主角，强度修订为"中"或"中-强"，并在差异化中显式对照 Simba（channel-切分）+ Liu Full-Stack（同时 multi-block 但 Arria 10 大规模）+ Aydonat DLA（Arria 10）

**评审建议**：**保留**，待主 Agent 确定 narrative 后再修订自评强度。

#### 总结：3 条弱-中 claim 全部保留为工程化叙述

无需砍掉。**审稿人不会因为"工程化贡献"水量挑论文**——前提是它们出现在正文系统描述章节而非 §1 contributions 列表中。

---

### Reject 风险预判

| 风险 | 等级 | 缓解方案 | Writer 已做 / 待做 |
|------|------|---------|-------------------|
| **"是 X+Y 拼凑"**（compiler + streaming/multi-core 双线 narrative） | 中 | paper.md §1 Introduction 找到统一 framing："fixed 16×16 INT8 array with compiler-driven PE utilization and streaming row-ring multi-core scaling"，把 contributions 列表放到统一 framing 之后 | Writer 在 §7.1 [TBD] 已交主 Agent 决定 narrative；建议 Phase 2 outline 时优先确定 |
| **"性能没跟 SOTA 比"**（FLUX_CNN 51.2 GOPS @ target / 35 GOPS @ Fmax，远低于 Aydonat 1382 GFLOPS / Lu 854.6 GOPS） | 中 | (a) 不主张 SOTA peak GOPS，主张 PE 利用率 + 系统简洁性 + 任意 H×W；(b) 用"GOPS / DSP" 或 "GOPS / W" 归一化对比；(c) 文中显式声明"我们的 design point 是中型 7-series + INT8 + 单核 256 MAC 的最佳设计，不是 high-end FPGA peak GOPS 比拼" | Writer 在 §5.1 已 flag "量级差异显著，不应直接绝对比较"；Phase 2 outline 时需进一步在 Introduction 中把 design space 说清 |
| **"作者自己都说 [CHECK]/[TBD] 这么多"**（16 [CHECK] + 7 [TBD]） | **必须消除** | 详见下方"必消除 [CHECK]"清单 | Writer 在 §8.3 已列修订路径（最先做 PE 利用率实测） |
| **"prior art 已经做过 row-streaming"**（Alwani / Kang / Liu） | 高，但 Writer 已应对 | Related Work 章详细差异化（不要回避） | Writer 在 §5.3 已列"必谈" |
| **"FINN 漏引"** | 中 | Phase 2 outline 时 Related Work 章补 FINN | Phase 1 不致 FAIL，Phase 2 补即可 |
| **"Cout<16 不优化"** | 低-中 | 在 Limitations / Future Work 节诚实声明 | Writer 在 §4.4 已显式列出 |
| **"Fmax 68.4 MHz 未达 100 MHz target"** | 低 | Limitations 节诚实声明 + 修复路径已知（加 use_dsp 属性 + SDP 流水化） | Writer 在 §4.6 / §5.1 / C1.5 已多处 flag |

#### 必消除 [CHECK] 清单（Phase 2 outline 完成前必须解决）

| # | 项 | 来源 | 紧迫度 | 行动 |
|---|----|------|--------|------|
| 1 | Layer 1 / 3 / 4 启用 Ky-fold 后实测 PE 利用率百分比 | §8.1 #1 #2 | **最高** | narrative A 的核心数字。无此则 Phase 7 写 §1 Introduction 强 claim 无法落地。请 project-analyst 跑 |
| 2 | "100 MHz target → 168 fps" vs "68.4 MHz Fmax → 115 fps" 文中口径统一 | §8.1 #4 | **高** | 用户决定。本评审建议**统一用 Fmax 实测口径**——审稿人会更接受，且 Future Work 中可承诺加 use_dsp 后重综合 |
| 3 | S2D 在加速器领域引用谱系反向检索 | §8.1 #15 | 高 | Phase 2 outline 前由主 Agent 跑 1 次 Elicit "space to depth FPGA accelerator compiler"——若仍未命中，C2.2 claim 强度可保持；若命中近邻 prior art，需在 contributions.md 补差异化 |
| 4 | NVDLA / Xilinx PG / cuDNN / ARM Ethos vendor doc 引用方式决策 | §8.1 #16 | 中 | Phase 2 outline 时由主 Agent 决定（直接引 vendor doc 还是引衍生 academic paper） |

其余 [CHECK] / [TBD] 可推到 Phase 7 paper.md / reviewer 阶段。

#### 总体 reject 风险等级：**中**

主要风险来自"两线 narrative 拼凑"+"算力数字弱"两点，但都可通过 Phase 2 outline 阶段的统一 framing + 文中 design space 声明缓解。**没有发现致命风险**（无过度声明、无关键 prior art 漏列）。

---

### 通过原因

1. **claim 强度自评谨慎**：只有 1 条"偏强"声明，且用"literature.md 检索未命中"的合规 phrasing 而非"首次/独创"
2. **3 处主要 prior art 威胁均显式列出**（Alwani / Kang / Liu）并附差异化角度
3. **18 条贡献的主角/配角分布合理**：4 主角 + 4 二级 + 9 配角 + 1 不写入，对一篇会议论文是工业标准
4. **弱-中 claim Writer 已自评且建议作为工程化叙述**——避免顶层 contributions 列表被水量稀释
5. **数据口径双报告**（target vs Fmax / 单层 vs 整网）+ 实测缺口诚实标注（[CHECK]）
6. **narrative A+B 混合的统一 framing 存在**（fixed array + compiler + streaming + multi-core），可由 Phase 2 outline 落地

---

### 假想审稿人模拟

- **Reviewer A（系统派）**："工程贡献够 architecture novelty 吗？"
  - 答：narrative A 主推 compiler PE 利用率（C2.1+C2.2），是 mechanism-level 创新；narrative B 主推 row-ring streaming + W slice multi-core，是 architecture-level 创新。两者结合能撑起 architecture novelty。**通过**
- **Reviewer B（FPGA streaming 派）**："和 Alwani / Kang / Liu / FINN 怎么比？"
  - 答：Writer 已对 Alwani / Kang / Liu 显式差异化；FINN 是 Phase 2 outline 待补。**轻度待修**
- **Reviewer C（端侧应用派）**："PE 利用率 100% 实测在哪？功耗呢？"
  - 答：PE 利用率实测是 [CHECK #1]，Phase 2 前必消除；功耗未在 contributions.md 中提到，**轻度风险**——若投 FCCM / FPGA 系会议建议补 board-level 功耗或 Vivado 估算
- **Reviewer D（compiler 派）**："Ky-fold 和 im2col / Winograd 的本质区别是什么？S2D 跟 sub-pixel CNN 的引用谱系是什么？"
  - 答：Writer 在 §C2.1 已对 cuDNN im2col 显式差异化（"只折 Ky 不展开 Kx"）；S2D 引用谱系是 [CHECK #15]，**Phase 2 前必消除以防 reviewer 突袭**

四类审稿人都不会因致命问题给 reject——只在 Phase 2 outline / Phase 7 paper.md 中补充论证即可。

---

### 给 Writer 的修订建议（优先级排序，**非 FAIL 项，可 Phase 2 顺手收敛**）

1. **【高】Phase 2 outline 前消除 [CHECK #1 #2 #15]**：PE 利用率实测 + S2D 反向检索。前者由 project-analyst，后者由主 Agent 跑 Elicit
2. **【中】Phase 2 outline / Related Work 章占位时补 FINN（Umuroglu@FPGA'17）/ FINN-R（Blott@TRETS'18）**：Phase 0 已 flag，本评审延续
3. **【中】narrative 决策 [TBD #2]**：建议 A+B 混合，统一 framing 为 "fixed 16×16 INT8 array with compiler-driven PE utilization and streaming row-ring multi-core scaling"。该决策影响 C3.5 multicore 强度自评修订（A 走则保持工程化，B 走则升为"中"）
4. **【中】口径统一 [CHECK #4]**：建议全文统一用 **Fmax 实测口径**（68.4 MHz / 35 GOPS / 115 fps），把 100 MHz target 放进 Future Work
5. **【轻】C1.2 row-ring 差异化句中显式对照 Tangram + Snowflake + Angel-Eye**：三者都在对比矩阵但贡献级差异化句缺
6. **【轻】C3.7 W slice 差异化句中显式对照 Liu Full-Stack（multi-block streaming）+ Aydonat DLA（Arria 10 1D streaming）**：表明 W-slice 与他们的多并行路线正交

以上修订都不致 FAIL，可在 Phase 2 outline 同步进行。

---

### 评审输出格式声明

本报告不修改 contributions.md，仅作为评审记录。Writer 决定是否在 Phase 2 outline 时引用本报告的修订建议。
