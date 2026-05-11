# 新颖性评审报告 Phase 2

## 第 1 次评审

### 判定
PASS（with caveats — 7 处中等风险已在 outline 内显式标注且应对策略可行；建议 Writer 在 Phase 3 起草时按"修订建议"项收紧 §5.4 / §3.4 / §7.6 三处的具体差异化句式）

### 评审范围

- 评审对象：`paper/workspace/outline.md`（8 章 + 元信息 + 章节清单 + 贡献-章节映射 + 文献-章节映射 + 章节依赖图 + 待决清单）
- 上下文资料：`paper/workspace/contributions.md`（17 条 C；§5.3 三处 prior art 威胁；§7.1 narrative 候选 4 条；§7.3 不建议主 claim 项；§8.1/§8.2 两类 [CHECK]/[TBD]）+ `paper/workspace/literature.md`（35 篇，A-G 七大类，§C 三处近邻 prior art = Alwani/Kang/Liu）
- 视角：扮演 3 类假想审稿人（Reviewer A 系统派 / Reviewer B 算法派 / Reviewer C 应用派）+ MAERI / Eyeriss-v2 / Tangram / Liu / Kang / Alwani 6 篇近邻 prior art 的"被代理"视角

### narrative 选择合理性

**判定：合理。混合 A+B（A 主 + B 支撑）是当前 contributions 池下的最优选择。**

- **不应单走 A**：narrative A（compiler-side PE utilization）单独成篇会暴露三个空洞——(a) §1 立 motivation 时如果不给"streaming row-ring 是端侧大图必要"的 system context，PE 利用率优化就失去硬件约束的支撑（"为什么不直接用更大阵列？"）；(b) §7 Eval 缺 11-layer chain 整网 MAC% / multi-core scaling 这两组数据，会让"compiler-only PE filling"的 claim 显得 narrow；(c) Cout<16 不优化的弱点在窄 narrative 下暴露更醒目。混合 A+B 让 §7.4 / §7.5 的整网/多核数据为 §7.2 的 PE 利用率数据"做背书"。
- **不应单走 B**：narrative B（row-streaming + multi-core）单独成篇直接撞到 Alwani / Kang / Liu 三处 🔴/🟡 威胁，且 contributions.md §5.4 已自评 claim 1 (streaming) 为"中等"——单走 B 主 claim 强度不够。
- **混合方案的具体取舍**已在 outline §"Narrative 选择" 段显式陈述（line 27–38）：narrative C（去中心握手）作为 §4.2 实现细节不主推，narrative D（开源）作为 §6 二级 system completeness——这两个降级判断与 contributions.md §7.1 + §7.3 的"不建议作为主 claim"列表一致，不存在 narrative 内部矛盾。
- **审稿人快读路径设计**（outline line 325–328：架构/编译器/system reviewer 各自 4-5 节路径）显示 Writer 已经预想了三类 reviewer 视角，这是混合 narrative 能 deliver 的前提。

**剩余风险（低）**：8 章配 ~10 页 IEEE 双栏，§5（2 页）+ §7（2.5–3 页）= 4.5–5 页是数据 + 主 claim 章节；§3（1.5 页）+ §6（2 页）= 3.5 页是 prior art 防御 + system 支撑。剩余 §1+§2+§4+§8 = 4.5 页，紧但不超。**风险点**：§3.4 是 3 处威胁防御主战场，但只占 §3 一节，且 §3 全章仅 1.5 页——如果 Writer 在 Phase 3 不把 §3.4 单独占足篇幅（建议 ≥ 0.6 页），3 个差异化论证会被压缩成 2-3 句话，失去防御力度。outline.md 的 §3 章描述里 6 个小节并列且 §3.6 还要做 5-维度 placement 表，§3.4 的实际可用篇幅可能不足 0.5 页。**建议见后**。

### §5 主章定位强度

**判定：定位足够强，但章描述里的差异化论证骨架还需要"拳头一句"。**

- **是 narrative A 的核心证据章**这一点 outline 表达清晰——预估篇幅 2 页（全文最长，与 §7 Evaluation 并列），与"全文最强 claim"地位匹配。
- **小节布局合理**：§5.1 Ky-fold / §5.2 S2D / §5.3 Joint Trigger / §5.4 Comparison 四节覆盖"机制 → 决策 → 对位"完整论证链；§5.4 单独成节做与 MAERI / Eyeriss-v2 / cuDNN im2col 的对位是关键决策，避免与 §3.2 / §3.3 重复——这是合理的"summary table at the end of contribution chapter"模式。
- **claim 强度对位 contributions.md**：C2.1 (强) → §5.1，C2.2 (中-强) → §5.2，C2.3 (工程化) → §5.3，全部一致；§5.4 不直接 claim 新内容，仅做对位——符合"主章不夸大"原则。
- **章描述中的防御预案到位**（line 165–167）：(a) Cout<16 老实承认；(b) S2D prior art 主动揭示 Sub-pixel CVPR'16；(c) PE 利用率口径区分。这三条都是 contributions.md §8.1 / §5.3 已识别的风险，Writer 已转写到 outline。

**剩余风险（中）**：
1. **章描述里没有给出 §5.4 对位表的"拳头一句"**——line 156 仅说"对位表"，但具体的"对位维度"是什么没在大纲里钉死（应该是 4 维：硬件改动量 / 编译器改动量 / Cin<16 时的 PE util 上限 / 是否需要重训）。Phase 3 起草时如果 Writer 没想清楚这 4 维就直接写表，会被审稿人挑"对位维度 cherry-picked"。
2. **C2.1 "强 claim"的支撑数据全部 [CHECK]**——§5.1 / §5.2 的实测 PE 利用率百分比 + DDR 带宽节省 全部待 project-analyst 跑实测。outline 已识别这是 §8.1 #1, #2, #3 三项 [CHECK]。这不是 outline 的问题，但 Writer 在 Phase 3 起草前必须等到这些数据；如果起草时数据未到，就会出现"narrative A 主章数据空洞"的失败模式。
3. **"零 RTL 改动"是 Ky-fold/S2D 的关键卖点，但章描述没有显式 claim**——outline §5.1 / §5.2 章描述只说"硬件按普通 conv 跑 / 编译器侧重排不复制"，没有把"zero RTL change vs MAERI ART hardware reconfig vs Eyeriss-v2 NoC reconfig"这条比较直接 claim 出来。这是与 narrative A 主轴 ("硬件保持简洁固定 16×16，编译器侧填满 PE") 最对应的一句。建议 Phase 3 在 §5 章首段或 §5.4 首段显式 claim。

### §3 prior art 威胁应对充分性

**判定：覆盖到位，但篇幅分配存在压迫风险。**

**对每条威胁的应对追溯**：

| Prior art | outline 应对位置 | 论证骨架（outline line 113–117） | 评估 |
|---|---|---|---|
| Alwani Fused-layer @ MICRO'16 🔴 | §3.4 + 文献-章节映射表 | "row-level vs layer-level granularity；FLUX_CNN 单核 layer-serial 共用硬件 vs Alwani 跨层同时驻留计算" | ✅ 论证维度具体（granularity + 硬件共用方式两维），可挡 |
| Kang AoCStream 🔴 | §3.4 + 文献-章节映射表 | "FLUX_CNN 承认外存必然存在并优化 DDR 流量 vs Kang 强调全片上；FLUX_CNN layer-serial 共用硬件 vs Kang layer-pipelined 多 block" | ✅ 论证维度具体（外存假设 + 硬件复用方式两维），可挡 |
| Liu Full-Stack 🟡 | §3.4 + §7.6 二次出现 | "FLUX_CNN 单核 layer-serial 不做 layer fusion；规模差（中型 7-series 86.6% vs 大型 Arria 10 GX1150 97%）是同思路在不同硬件规模下的对照" | ⚠️ 见下条专节 |
| MAERI / Eyeriss-v2 中 | §3.2 + §5.4 | "硬件复杂度 vs 编译复杂度的不同 trade-off，不互斥" | ✅ 与 narrative A "compiler-only" 对位是 outline 主张的核心 |

**Reviewer A（系统派）模拟挑战**：
- 问："Alwani 是 layer-fused，能省 95% off-chip 流量；FLUX_CNN row-level 只缓几行，还在写 DDR——为何 row-level 是 advance 而不是 retreat？"
- outline 现有应对：line 304 "row-level granularity（不是 layer-fused 也不是 fully-on-chip）+ layer-serial 单核共用硬件（不是 layer-pipelined 多 block）"
- **缺口**：outline 没有把"为何选择 row-level 而非 layer-level"的 design rationale 显式回答（应是 BRAM 容量约束 + 硬件复用 + 编译器复杂度三因素）。Phase 3 必须在 §3.4 论证 row-level 是端侧中型 FPGA + 单核硬件复用约束下的合理点，不是 layer-level fusion 的"弱化版"。

**Reviewer B（算法派）模拟挑战**：
- 问："Eyeriss-v2 NoC reconfig 不仅解 Cin<16，还解稀疏；MAERI ART 解任意层形状。FLUX_CNN 的 Ky-fold/S2D 只解 Cin<16 这一种 case，泛化性是否不足？"
- outline 现有应对：line 117 "硬件复杂度 vs 编译复杂度的不同 trade-off，不互斥"
- **缺口**：太弱。"不互斥"是闪避，不是回答。建议 §5.4 明确说"FLUX_CNN target 的是常见 ResNet-style 浅层 (Cin∈{4,8})—— model_analysis.md 14 层中 5 层 Cin<16，覆盖关键 motivation case；不 target 任意层形状"。这是 contributions.md C2.1 自评 "claim 偏强" 的真实 scope，需如实陈述。

**Reviewer C（应用派）模拟挑战**：
- 问："VGA 480×640 单 start 的 motivation case 在 §1 / §2.3 抛出，但 §3.4 的对照工作 Snowflake/Angel-Eye 是否也支持任意 H×W？"
- outline 现有应对：line 295 表头维度"任意 H × W 输入"已列入 5.2 表，但 outline 的 §3.4 章描述（line 103）是 "fpgaConvNet / DnnWeaver / Snowflake / Angel-Eye / Aydonat / Lu Winograd / Alwani / Kang / Liu" 的并列，没说哪几个支持任意 H×W。
- **缺口**：Phase 3 在 §3.4 必须给出一个 mini-table（或一段文字）明确各工作的 max H/W 支持情况——否则"任意 H×W single start"看起来像 universal 但其实多数 line-buffer 工作都支持。差异化要落到 row-level credit backpressure + IDMA/计算/ODMA 三阶段并发上。

**篇幅压迫风险**：§3 总 1.5 页，6 节 + 1 个 placement 表。按 IEEE 双栏估算，1.5 页 ≈ 1500–1700 字（中文等价）。6 节平均 250–280 字 + 1 张表。§3.4 是防御主战场，需要至少 2 段（350–400 字）才能把 Alwani / Kang / Liu / Snowflake / Angel-Eye 五个对照都"有具体差异化句"。outline 没有显式给 §3.4 切单独篇幅。建议 Phase 3 起草时让 §3.4 占 §3 总篇幅的 ≥ 40%。

### §7.6 head-to-head Liu 论证可信度

**判定：论证立得住，但 outline 内的句式还需要更稳的二次防御。**

- **核心论证**：FLUX_CNN 86.6% 整网 MAC% (中型 7-series XC7K325T) vs Liu 97% MAC% (大型 Arria 10 GX1150) = "same paradigm at different scale"
- **outline 立场**（line 116, 224, 311, 325 多处）：承认 Liu 更高，承认是同思路（streaming + residual），论证差异在硬件规模而非方法

**审稿人挑刺路径模拟**：

1. **挑战 1（最常见）**："你不能用'规模不同'来解释 10 个百分点的 MAC% 差距。规模大反而更容易出现 idle（更多 stall），中型规模 86.6% 应该比大型规模 97% 更差才对，你的论证方向错了。"
   - **outline 现有应对**：line 224 提到"86.6% 整网 MAC% vs Snowflake 91% 平均计算效率：口径不完全等同（FLUX_CNN 含 IDMA/ODMA stall），需在表脚注说明"。
   - **可信度评估**：✅ 这条预判正确——Liu 的 97% 大概率是"compute efficiency on conv layer"而非 outline 上 FLUX_CNN 的"整网 MAC% 含 DMA stall"，这是口径差。建议 Phase 3 §7.6 表脚注**明确写出 Liu 的 MAC% 口径定义** ([CHECK: 查 Liu 原文 §X 的 efficiency 定义]——这是 contributions.md §8.1 #6 + #10 的延伸)。如果 Liu 真的是"端到端 MAC%"也是 97%，那"规模不同"论证会更弱，必须落到 layer fusion 复杂度 vs simplicity 这条线（contributions.md §5.3 已识别，line 311）。
   - **缺口**：outline 把"规模差"放在主防御位 (line 116, 311)，把"layer fusion 复杂度差异"放二级。如果 reviewer 攻击规模差论证，二级防御能否顶住取决于 §7.6 文字能否切换论证轴。Phase 3 起草建议**双轨防御**——首先用 layer fusion 取舍说明（"FLUX_CNN 单核 layer-serial 不做 cross-layer fusion"），再用规模差异作为辅助。
2. **挑战 2（精明）**："那为什么不就报 conv layers 的 MAC% 而非整网？这样跟 Liu 同口径就能直接比。"
   - **outline 现有应对**：line 167 / line 224 已识别"单层 PE 利用率 vs 整网 86.6% MAC%"两口径区分，line 218 把这条列为 §7.4 必论证项 ([CHECK: 单层 vs 整网 MAC% 口径论证])。
   - **可信度评估**：✅ outline 已意识到必须双口径报告，方向正确；但具体怎么写还看 Phase 3。建议 §7.4 + §7.6 同时报"端到端 MAC% (含 stall) + conv-only PE 平均利用率 (excl. DMA stall)"，这样跟 Liu 双口径都能比。
3. **挑战 3（致命）**："Liu 用了 layer fusion，你不用，所以你慢——这不就证明你比 Liu 差吗？为什么这是'同思路在不同规模'？"
   - **outline 现有应对**：line 116 "FLUX_CNN 单核 layer-serial 不做 layer fusion；规模差是同思路在不同硬件规模下的对照"
   - **可信度评估**：⚠️ outline 把"不做 layer fusion"和"规模差"两条并列陈述，但没有论证为何"不做 layer fusion"是可接受的取舍。审稿人会问"为什么不做"——必须回答"BRAM 受限 (single-core 128 BRAM 已 28.8%) + 硬件简洁优先 + 编译器复杂度可控"。outline 没有把这条 design rationale 写出来。**这是 Phase 3 必补的论证**。

**整体评估**：outline §7.6 的 Liu 论证预判到位，但句式偏"防御性闪避"而非"主动论证"。Phase 3 起草建议把"layer fusion 取舍 → BRAM/硬件复杂度约束 → 86.6% 是该取舍下的合理点 → 规模差异作为辅助 framing"四步论证写清楚，而不是把"规模差"放在主防御位。

### claim 强度一致性（contributions → outline）

**全文逐条核对（17 条 C）**：

| C | contributions.md 自评强度 | outline.md 主章节描述强度 | 一致性 |
|---|---|---|---|
| C1.1 | 弱-中 (工程化) | line 65 "弱-中 claim ... 不进 contribution 列表" | ✅ 一致；outline 显式不主推 |
| C1.2 | 中（differentiation 工程取舍） | line 256 "中（与 Alwani/Kang/Liu 差异化）" | ✅ 一致 |
| C1.3 | 工程化 (implementation detail) | line 257 "工程化（implementation detail）"+ line 142 "C1.3 per-col PARF 作为'单元级 implementation detail'出现，不作顶层贡献" | ✅ 一致 |
| C1.4 | 偏增量 | line 258 "偏增量"+ line 192 "不当主贡献，仅作 system 完整性" | ✅ 一致 |
| C1.5 | 实现状态披露 | line 259 "实现状态" + line 66 "老老实实承认 Cout<16 不优化 + Fmax 68.4 MHz 未达 100 MHz target" | ✅ 一致 |
| C2.1 | 强（无直接 prior art） | line 260 "**强**（无直接 prior art）" | ✅ 一致 |
| C2.2 | 中-强（S2D 谱系待补查） | line 261 "中-强（S2D 谱系待补查）" | ✅ 一致 |
| C2.3 | 工程化 | line 262 "工程化" | ✅ 一致 |
| C2.4 | 偏增量 (loop tiling 已公开) | line 263 "偏增量（loop tiling 已公开）" | ✅ 一致 |
| C2.5 | 工程化 | line 264 "工程化" | ✅ 一致 |
| C3.1 | 工程化 | line 265 "工程化" | ✅ 一致 |
| C3.2 | 工程化 | line 266 "工程化" | ✅ 一致 |
| C3.3 | 工程化 (验证基础设施) | line 267 "工程化（验证基础设施）" | ✅ 一致 |
| C3.4 | 机制级 (小) | line 268 "机制级（小）" | ✅ 一致 |
| C3.5 | 工程化 + 系统集成 | line 269 "工程化 + 系统集成" | ✅ 一致 |
| C3.6 | 工程化 (辅助) | line 270 "工程化（辅助）" + line 273 "在 §6.5 单节或合并 §6.2 末尾，二选一" | ✅ 一致 |
| C3.7 | 中-强（多核切分维度） | line 271 "中-强（多核切分维度）" | ✅ 一致 |

**结论**：claim 强度全文一致。**没有发现 outline 内出现 contributions.md 没标的过强 claim**。

**亮点**：
- outline §1.4 (line 66) 主动列出"弱 claim 不进 contribution 列表"，C1.1 / C1.4 不上 §1.4 主清单——这与 contributions.md §7.3 "不建议作为主 claim 的项" 完全对齐
- outline §IV.2 把 C1.1 描述为 "narrative A 硬件简洁性论据" 而非顶层贡献，再次降级到位
- outline §6.3 把 C1.4 标注为 "NVDLA-inspired，新增点仅在 R.1 bias 重定位 + R.2 shortcut_mult/shift 可编程量化因子 — 不当主贡献" (line 192)——主动揭示 NVDLA-inspired 这条最容易被审稿人挑的点

**唯一需要 Writer 注意的细节**：outline line 25 一句话摘要里的 "PyTorch nn.Sequential 端到端 bit-exact 编译" 是工程化 (C2.5)。摘要把它放在三个核心卖点（compiler-side / row-ring / multi-core）后面是正确的；但如果 Phase 3 起草 Abstract 时把这条提前，会被审稿人挑"为什么是 contribution"。建议保持 outline 当前顺序。

### 整体定位连贯性

**判定：定位链条清晰，§1 → §7 闭环成立。**

**主张-方法-验证三段式追溯**：
- **主张**（§1.1 + §1.3 + §1.4）：fixed-array CNN accelerator on edge FPGA，硬件保持简洁固定 16×16，编译器侧填满 PE，单 start 跑完任意 H×W
- **方法**（§4 + §5 + §6）：
  - §4 Architecture: 5 模块去中心 valid-ready + row-ring streaming + per-col PARF
  - §5 Compiler: Ky-fold + S2D + 自动决策
  - §6 System: PyTorch 编译栈 + DMA + SDP + multi-core
- **验证**（§7）：
  - §7.2 PE util (浅层 12.5%-50% → 接近 100%) → 兑现 §1.3/§5
  - §7.3 资源 + Fmax → 配 §4 硬件简洁性
  - §7.4 整网 MAC% 86.6% / 593K cycles → 兑现 §1.3 system-level
  - §7.5 multi-core scaling → 兑现 §6.4
  - §7.6 head-to-head → 兑现 §3 prior art 对照
  - §7.7 limitations → 兑现 C1.5 诚实标注

**章节依赖图**（line 312–321）显式画出"§1 → §7 主验证场"和"§3 是 §5/§6/§7 的 prior art 防御铺垫"两条链路，与论证三段式吻合。

**剩余风险（低）**：
1. **§8 Conclusion 仅 0.5 页**——line 233 描述"一段话回顾 narrative A 主轴 + 一段话指出 future work"。**风险**：如果 Writer 在 Phase 3 起草时让 Conclusion 只回顾不闭环，会失去 §1 promise → §8 兑现的"环"。建议 Conclusion 显式提"§5 PE util / §7.4 86.6% / §7.5 1.45× speedup 三组关键数字"对应 §1.4 的 contribution 列表，不重复但回应。outline 已识别 line 244 "不在 conclusion 翻新 claim / 不夸大 contribution 强度"，方向正确。
2. **§2 Background 的 motivation 数据全部 [CHECK]**——line 85 "§2.2 各浅层 PE util 实测百分比表（Layer1/3/4/5a/5c 启用 fold 前后）需 project-analyst 跑实测填入"。如果 §2.2 数据不到位，§1 motivation → §5 method → §7 validation 整条 narrative A 主轴的"浅层 PE 利用率痛点"立不住。这不是 outline 的问题，但 Writer 在 Phase 3 起草前必须等数据。

### Reject 风险预判

**整体风险：中-低**

#### 高风险（无）

经核查，无高风险触发条件：
- 无过度声明 ("SOTA / 独有 / 首次" 等强词在 outline 内没有出现，contributions 主清单已主动剔除 C1.1/C1.4 弱 claim)
- 无关键文献漏（35 篇 + 3 处近邻 prior art 已覆盖；Alwani/Kang/Liu 在 Phase 0 §批 8 主动检索补入；MAERI/Eyeriss-v2/Tangram 三大可重构对位齐全）
- 无 narrative 不闭环（§1 → §7 → §8 链路清晰）

#### 中风险（4 项）

1. **§3.4 篇幅压迫**：1.5 页 / 6 小节 / 1 个 placement 表 → §3.4 实际可用篇幅可能 < 0.5 页，挤压 3 处 prior art 威胁的差异化论证。**缓解**：Phase 3 起草时把 §3.4 占 §3 总篇幅的 ≥ 40%（约 0.6 页），其他 5 节合计 0.9 页；或牺牲 §3.6 placement 表（合并到 §3.4 末尾）。
2. **§7.6 Liu 双轨防御未在 outline 明确**：当前论证主轴是"规模差"，二级是"layer fusion 取舍"。**缓解**：Phase 3 起草建议"layer fusion 取舍 (BRAM/复杂度约束) → 规模差异 (作为 framing)"双层论证，避免 reviewer 攻击规模差论证时无二级防御。
3. **§5.4 对位维度未在 outline 钉死**：outline 仅说"对位表"，未给具体 4 维（硬件改动 / 编译器改动 / Cin<16 PE util 上限 / 是否需要重训）。**缓解**：Phase 3 起草前 Writer 与 user 商定对位 4 维。
4. **大量 [CHECK] 待填**：§5.1/§5.2 PE 利用率实测、§5.2 DDR 节省、§7.4 Wall_us、§7.5 ResNet 11-layer multicore 实测、§7.6 baseline 整网 MAC% 等 13 项 [CHECK]。**缓解**：Phase 3 起草前必须等 project-analyst 完成实测（contributions.md §8.3 修订路径已排序）；如果起草时数据未到，narrative A / B 会出现"主章数据空洞"失败模式。

#### 低风险（3 项）

1. C2.2 S2D 在加速器领域的引用谱系不清（reviewer 阶段补查 ASPLOS/HPCA） — outline 已主动揭示 line 110, 163, 287
2. C1.5 Fmax 68.4 MHz 未达 100 MHz target — outline 已在 §7.7 老实标注 + 给出修复路径
3. Cout<16 不优化 — outline 已在 §5.3 末尾或 §5.4 末尾承认

### 通过原因

1. **narrative 选择有依据且 outline 显式陈述取舍**：A+B 混合是 contributions 池下的最优；放弃 C/D 的理由 (line 34, 36) 写得清晰，与 contributions.md §7.1 一致
2. **17 条贡献 100% 归位**，强度全文一致，无过强 claim 走漏
3. **3 处 prior art 威胁全部覆盖**：Alwani/Kang/Liu 在 §3.4 + §7.6 双层应对；MAERI/Eyeriss-v2 在 §3.2 + §5.4 对位；Liu 二次出现在 §7.6 head-to-head
4. **诚实标注弱点到位**：Cout<16 不优化、Fmax 未达、SDP fusion NVDLA-inspired、S2D 谱系待补查 四项主动揭示
5. **§1 → §7 → §8 论证闭环**：主张-方法-验证三段式清晰；章节依赖图 + 审稿人快读路径设计配套
6. **claim 强度自评对位 contributions.md 100% 一致**——这是 outline 最大优点，避免了"大纲偷偷加强 claim"的常见失败模式

### 给 Writer 的修订建议（PASS 但建议在 Phase 3 起草前/中处理）

以下为非阻断性建议，按优先级排：

1. **[Phase 3 主章起草前]** §5.4 对位维度 4 维钉死：硬件改动量 / 编译器改动量 / Cin<16 时的 PE util 上限 / 是否需要重训。建议在 outline.md "§5 章节描述" 段落小幅补充该 4 维，避免 Phase 3 起草时临时定维度
2. **[Phase 3 主章起草前]** §7.6 Liu 论证切换到双轨防御：layer fusion 取舍 (主) + 规模差异 (辅)。建议在 contributions.md §5.3 表格 Liu 行的"FLUX_CNN 差异点"列里把"layer fusion 取舍"提到首位
3. **[Phase 3 起草中]** §3.4 篇幅守底：占 §3 总篇幅 ≥ 40%；考虑把 §3.6 placement 表合并到 §3.4 末尾以节省篇幅
4. **[Phase 3 起草中]** §5 章首段或 §5.4 首段显式 claim "zero RTL change vs MAERI ART hardware reconfig vs Eyeriss-v2 NoC reconfig"——这是 narrative A 主轴最关键一句，outline 章描述里没显式列
5. **[Phase 3 起草前]** 等 project-analyst 完成 §8.1 #1, #2, #3, #5, #8 五项实测——narrative A/B 主章数据基础。建议主 Agent 在调度 Phase 3 前 gate 这五项
6. **[Phase 3 起草中]** §3.4 内增一段 "max H/W support" 对照（mini-paragraph 或 mini-table），让"任意 H×W single start"差异化落到 row-level credit backpressure 而非 universal 卖点

---

### 评审小结

outline.md 在 narrative 选择 / claim 强度 / prior art 应对 / 整体闭环四个核心维度都达到"假想审稿人难以一击致命"的水平。本次评审未发现严重过度声明、关键文献遗漏、或 promise 未兑现的 FAIL 触发条件。**判定 PASS**。

剩余 4 项中等风险均为"Phase 3 起草时具体兑现"层面的细节，Writer 在 outline 内已显式标注了 [CHECK]/[TBD]/未决问题，处理路径清晰。建议主 Agent 把上述 6 条修订建议同步给 Phase 3 Writer，作为起草前/中的注意清单。
