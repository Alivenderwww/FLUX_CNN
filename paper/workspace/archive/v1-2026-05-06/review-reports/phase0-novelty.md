# 新颖性评审报告 Phase 0

## 第 1 次评审

### 判定
PASS（中等强度通过——文献覆盖充分、新增 prior art 标注得当；定位段存在 1 处中等问题、3 处轻微问题，但都不到 FAIL 阈值，可在 Phase 1 contributions 阶段顺手收敛）

---

### 评审范围

**评的是**：
- 文献覆盖度（A–G 七大类是否覆盖到 FLUX_CNN 的真实对比面）
- 新增 3 处 prior art（Alwani / Kang / Liu）的威胁度判定与差异化角度
- "本工作定位预判"段（literature.md L60-69）的 claim 强度
- "可量化优势 candidate"（L503-510）的措辞强度
- 是否有遗漏的近期（2022–2025）同领域工作可能成为审稿人 reject 把柄

**不评**：
- DOI / 会议归属 / 数据准确性（tech reviewer 的事，已并发评审）
- 文献条目内的句式/段落结构（writing 评审的事）
- 5 处 vendor doc 的 [CHECK] 残留（已合理标注，不算缺陷）

---

### 覆盖度评估

按主题轴系统检查：

#### A. 系统脉动阵列（ASIC）
**覆盖：充分**。TPU v1 / Eyeriss ISCA'16 + JSSC'17 拆分 / Eyeriss-v2 / Gemmini / NVDLA / Simba 全到位，足以撑起"FLUX_CNN 与 ASIC 系统脉动阵列对比"那一面。Gemmini 被点名为"最贴近对比对象"，对位准确（同 16×16、同 INT8、Gemmini 中心 controller vs FLUX_CNN 去中心化）。

**潜在补强（轻微）**：
- TPU v4 / TPU v5（Jouppi 2023）的多 die 互联、Cerebras WSE（2021）属于"远端规格参考"，在 ASIC 阵列对比里不强求；不是 reject 把柄。

#### B. 数据流体系与 PE 利用率优化（编排）
**覆盖：充分**。MAERI / ShiDianNao / Buffets / Timeloop / Tangram 全到位。Tangram 的会议归属已修正为 ASPLOS'19（之前错记 MICRO'19）。MAERI 作为 FLUX_CNN "硬件可重构 vs 编译器侧"对比的 anchor，定位精准。

**潜在补强（轻微）**：
- SIGMA@HPCA'20（Qin et al., flexible NoC for sparse tensor）— 与 MAERI/Eyeriss-v2 同主题但更新；不强求但若 Phase 1 要打"硬件可重构 PE 利用率"这一面更全可补。
- HERALD@MICRO'20（Kwon et al., heterogeneous accelerators for irregular workloads）— 同方向但属于偏门；不补也行。

#### C. Streaming / line-buffer 风格 CNN 加速器（FPGA）⭐ FLUX_CNN 主对照集
**覆盖：基本充分，但有 1 个值得注意的补强点（中等）**。
- fpgaConvNet (FCCM'16 + TNNLS'19) ✓
- DnnWeaver MICRO'16 ✓
- Zhang FPGA'15 ✓
- Aydonat DLA FPGA'17 ✓
- Snowflake ISCAS'17 ✓
- Angel-Eye TCAD'18 ✓
- **新增**：Alwani Fused-layer MICRO'16 / Kang AoCStream FCCM'22 / Liu Full-Stack TNNLS'21 ✓

**【中等补强】FINN（Umuroglu et al.@FPGA'17）和 FINN-R / Blott et al.@TRETS'18** 未列入。FINN 是 Xilinx Research 主推的 streaming-style FPGA CNN inference 框架，在审稿人视野里是 fpgaConvNet 的孪生工作（同样 streaming + per-layer dataflow + HLS 生成）。FINN-R 还专门讨论了不同精度（含 INT1/INT2/INT4/INT8）的 streaming dataflow 实现。
- **审稿人风险**：投 FCCM/FPL/TRETS 时 FINN 几乎是必引；漏掉会被审稿人直接圈出"作者似乎不熟悉 streaming 加速器主流文献"。
- **建议**：Phase 1 时由 Writer 视实际定位决定是否补；本评审记录在此但不作为 FAIL 项（因为 Alwani / Kang / Liu 已经覆盖了 streaming 主线索的核心威胁，FINN 偏向"HLS 自动生成"那一支，与 FLUX_CNN 手写 RTL 不正交也不竞争）。

#### D. PE 利用率优化技巧（kernel folding / S2D / 通道复用）
**覆盖：基本充分**。cuDNN（im2col 起点）+ Sub-Pixel CVPR'16（S2D 算法谱系）+ Lu Winograd FCCM'17 + Ma FPGA'17 loopopt 都到位。

**【轻微】Sub-pixel S2D 在硬件加速器领域的"近邻 prior art"**：
- literature.md L518 已标 [CHECK] 留待 Phase 1+ 补查 ASPLOS/HPCA 中是否有 architecture-side S2D 的更早出处。这个 [CHECK] 处理得合理。
- 一个潜在的近邻：Sze et al.@PIEEE'17 "Efficient Processing of Deep Neural Networks: A Tutorial and Survey"（10.1109/JPROC.2017.2761740）—— 综述性论文，会涉及 channel folding / depth packing 思路。如果 reviewer 问 "Ky-fold 与 channel-folded conv 比如何"，这是一个好的引锚。
- **不作 FAIL 项**：因为 Writer 的 [CHECK] 标记本身就承认这条不确定性，且方向论证（"compiler-pass 不重训"）已经站住脚。

#### E. AXI / DMA / DataMover
**覆盖：可接受**。Xilinx PG022 / PG338 / ARM Ethos 全标 [CHECK: vendor doc]，处理方式合理。

**【轻微】DPU 对照点的可引学术替代**：literature.md L370 已经给出 Jia FPL'22 XVDPU 作为可引学术替代，处理得当。

#### F. 编译器 / ISA-level 切片机制
**覆盖：基本充分**。TVM / Interstellar / VTA 全到位。

**【轻微】MLIR 走向**：HLS 走向的最新代表如 ScaleHLS@HPCA'22（Ye et al.）/ AutoSA@FPGA'21（Wang et al., systolic array auto-generation）未提及。FLUX_CNN 是手写 RTL 不走 HLS，可以不深入这条线，但 Related Work 提一句"区别于 HLS-generated streaming accelerator"会让定位更紧。**不作 FAIL 项**——属于"可加分但不必"。

#### G. 量化 + Bias / Residual fusion
**覆盖：充分**。Jacob CVPR'18 / He ResNet / EIE / ESE 都到位，足以支撑 SDP fusion 那一段的 framing。

---

### 新增 prior art 威胁度评估

#### ⚠ Alwani et al.@MICRO'16（Fused-layer CNN）
**威胁等级：中**

- **是不是同方向**：**部分重叠但角度明显不同**。Alwani 强调"**跨层 fusion**"——多层 conv 在 on-chip pyramid buffer 中同时驻留，把多层中间 feature map 留在片上不写 DRAM。FLUX_CNN row-ring 是"**层内 row-streaming + layer-serial 共用同一组硬件**"，不是把多层叠在片上同时计算。
- **审稿人最可能的攻击**：
  > "你的 row-ring 不就是 Alwani fused-layer 的简化版？只缓存几行而不是几层 pyramid，新颖性在哪？"
- **FLUX_CNN 仍可主张的差异化角度**（足以撑住）：
  1. **粒度不同**：Alwani 是 layer-pyramid（多层中间 fmap 同驻留），FLUX_CNN 是 row-ring（同层内仅几行 ifm + 几行 ofm 留 SRAM）。BRAM 占用差一个数量级。
  2. **调度模式不同**：Alwani 是"多层同时计算 + on-chip pyramid"，需要在编译器里规划 pyramid 形状；FLUX_CNN 是"layer-serial + 单核共用硬件"，单次 start 跑完任意 H×W 单层，**不依赖跨层 fusion 也能做 streaming**。
  3. **反压机制**：FLUX_CNN 的 forward-pressure（"`rows_available ≥ yout·stride + Ky` 时发射"）+ 双向 row-credit 反压是握手原语，Alwani 缺少这条 layer-internal 的 elastic 机制（pyramid 调度是预先编译规划的）。

  → 这三条差异化角度足够清晰，但需要 Phase 1 在 contributions.md 里**显式写出**而不是"和 Alwani 不同"一笔带过。

#### ⚠ Kang@FCCM'22（AoCStream）
**威胁等级：高**

- **是不是同方向**：**最近邻 prior art**。Kang 的核心宣称——"**stream-based line-buffer 架构（取代传统 frame-based）**；line buffer 大小**线性于图宽**而非平方于图大小"——与 FLUX_CNN 的"row-ring 仅缓几行 line buffer"几乎是同一句话。FCCM'22 是审稿人最可能首先扔出来的对比对象。
- **审稿人最可能的攻击**：
  > "AoCStream 在 FCCM'22 已经做了 stream-based line-buffer 任意 H×W single-start 在 low-end FPGA 上跑物体检测，FLUX_CNN 的 streaming row-ring 是不是只是 AoCStream 的 derived work？"
- **FLUX_CNN 仍可主张的差异化角度**（必须显式论证）：
  1. **AoCStream = layer-pipelined 多 block**（每层一个 dedicated hardware block，多层并行同时跑，但**全部驻留片上不能溢出**）；**FLUX_CNN = layer-serial 单核共用硬件**（一组 256 MAC 跑所有层）。这是最本质的差异——不同架构哲学。
  2. **AoCStream 假设全图 + 全权重 + 全 fmap 都装片上**（low-end FPGA 跑小物体检测的前提）；**FLUX_CNN 承认 DDR 必然存在**（VGA 480×640 单图 4.9 MB，IFB 容不下），并通过 row-ring + IDMA/计算/ODMA 三阶段并发优化 DDR 流量。两条假设链路完全不同。
  3. **AoCStream 强调 accelerator-aware pruning**（稀疏剪枝协同），FLUX_CNN 是 dense INT8。
  4. **AoCStream 是 layer-pipelined**（layer 级流水，不能复用 PE），意味着每层 PE 利用率一旦低就永久浪费；**FLUX_CNN 的 Ky-fold + S2D 在 layer-serial 共用硬件下能**让所有层都填满 256 MAC**（因为编译器侧把不同层映射到统一的 16×16 上）。这是 FLUX_CNN PE 利用率优化方法**与 AoCStream 不可替代**的关键论据。

  → **这是最危险的 prior art**。**Phase 1 的 contributions.md 必须给 AoCStream 一段独立的差异化论述**，不能合并到"和其他 streaming 工作不同"的笼统句子里。Writer 在 L284 已经给出这个差异化论述，但还需要在 contributions 段重复一次确保审稿人不漏读。

#### ⚠ Liu, Fan et al.@TNNLS'21（Full-Stack Streaming CNN）
**威胁等级：中**

- **是不是同方向**：**部分重叠**。Liu 的 streaming + residual fusion + 97% MAC efficiency 与 FLUX_CNN 的 SDP residual + 86.6% MAC% 同方向但**规模不同（Arria 10 GX1150 是 high-end）**。
- **审稿人最可能的攻击**：
  > "Liu TNNLS'21 已经在 streaming + residual fusion 拿到了 97% MAC efficiency，FLUX_CNN 的 86.6% 不是更差吗？"
- **FLUX_CNN 仍可主张的差异化角度**（要小心）：
  1. **资源平台差异**：Liu 是 Arria 10 GX1150（427K LUT, 1518 DSP），FLUX_CNN 是 Kintex-7 XC7K325T（203K LUT, 840 DSP）—— 半个量级；Liu 的 97% 是 high-end DSP-rich 平台，FLUX_CNN 的 86.6% 是 mid-range BRAM-bound 平台，**单纯比 MAC% 数字不公平**。
  2. **Liu 用 layer fusion**（多层中间数据合并），**FLUX_CNN 不用 layer fusion 也能拿到 86.6%**。这本身是个 framing 优势——"在更弱平台 + 更简洁架构（无 layer fusion）下仍然拿到接近的整网 MAC%"，需要在 contributions 里这样表述。
  3. **PE 利用率优化方式**：Liu 偏重"layer fusion + 多级 parallelism + 充分 DSP 利用"（**硬件路线**）；FLUX_CNN 偏重"Ky-fold + S2D 编译器侧"（**编译器路线**）。这条差异 Writer 已经在 §C 末尾点出。

  → **威胁中等**。Phase 1 contributions 写时**禁止单独 claim "我们 PE 利用率最高"**，因为 Liu 97% > 86.6%。改为 claim "在固定阵列 + layer-serial + 中端 FPGA 平台下做到 ..."。

#### 整体威胁度小结
| Prior art | 威胁 | FLUX_CNN 差异化是否站住 | Phase 1 必做动作 |
|---|---|---|---|
| Alwani'16 | 中 | 是（粒度 + 调度 + 反压三轴）| Related Work 显式三轴对比 |
| **Kang'22 AoCStream** | **高** | 是（layer-serial vs layer-pipelined + DDR-aware）| **contributions.md 给独立段，禁止合并** |
| Liu'21 Full-Stack | 中 | 是（编译器路线 + 中端平台）| 禁止单独 claim "PE 利用率最高"，改为限定语境 |

---

### 整体定位评估

#### "横跨 ASIC + FPGA streaming 两条线" 是否清晰？
**基本清晰，但有 1 处中等问题**。

literature.md L62 写："**横跨两条线**：硬件层面是 ASIC 风格的 16×16 spatial array...；系统集成层面是 FPGA streaming 风格..."

- **优点**：明确了"硬件形态属 A 类、系统集成属 C 类"，不是空泛的"既 X 又 Y"。
- **风险（中等）**：审稿人会问"那么本工作的 fundamental contribution 到底属于 A 还是 C？"
  - 如果投 FCCM/FPGA/TCAD（C 类阵地），审稿人是 FPGA streaming 圈，会认为"16×16 spatial array 是已知的，你的 contribution 必须在 streaming/系统/编译器侧"——那么定位就要往 C 类压重。
  - 如果投 ISCA/MICRO/ASPLOS（A 类阵地），审稿人是 ASIC 体系结构圈，会认为"row-ring streaming 在 FPGA 圈里早有人做（fpgaConvNet/AoCStream），你的 contribution 必须在 dataflow/PE 利用率优化算法上"——那么定位就要往编译器侧（D 类、F 类）压重。
- **建议（不作 FAIL 项）**：Phase 1 决定目标会议时，让 Writer 根据会议性质把"horizontal positioning"段重写——不是"横跨"而是**主轴在哪**。L66 列的"三条对比轴"是好的工具，但要选 1 条做主轴、其他 2 条做支轴。

#### "三条对比轴" 是否能撑起一篇完整论文？
**能撑，但被 prior art 削弱后变化如下**：

| 对比轴 | 削弱前强度 | 削弱后实际 claim |
|---|---|---|
| 轴 1：PE 利用率优化的位置（编译器 vs 硬件）| 强 | **依然强**——Ky-fold + S2D 在 streaming + layer-serial 场景下无近邻 prior art（MAERI 是硬件路线，sub-pixel CVPR'16 是 ML 算法路线）。**这是最强 claim 候选**。 |
| 轴 2：任意 H×W streaming（row-ring）| 中（被 Kang/Alwani 削弱）| **削弱为"row-level granularity + layer-serial 共用硬件 + DDR-aware"**——可作为支轴，单独立轴风险高 |
| 轴 3：去中心化握手 vs 中心 FSM | 弱（"工程美学"）| **不能单独作为 architectural contribution**，需借 Buffets ASPLOS'19 的形式化语言包装。属于支轴中的支轴。|

**结论**：**轴 1（PE 利用率编译器优化）应作主 contribution**。轴 2 和轴 3 作为 "在 streaming row-ring 这个具体载体上的支撑机制"。这样的论文骨架才能站住——不能把 3 条都当独立贡献等量并列。

**这一点 Writer 在 L508 "可量化优势 candidate" 中其实已经默认排序了（claim 2 强度高于 claim 1 / claim 3），但还没有明确标注"主 / 支"。Phase 1 应明确化。**

#### "可量化优势 candidate" 措辞强度评估

| Claim | 措辞 | 评估 |
|---|---|---|
| 1. 流式任意 H×W | "审稿人威胁" 已标，差异点已点 | **OK，措辞克制** |
| 2. 编译器侧 PE 利用率优化 | "**审稿人威胁较弱**" + "**较强 claim 候选**" | **OK，措辞克制；只要 Phase 1 实测数字到位（12.5% → ~99%）就站得住** |
| 3. 去中心化握手 | "属于'工程美学'claim" + "需评估是否被审稿人认可" | **OK，自我定位准确** |
| 4. SDP residual fusion | "需要审视是否足够新颖" | **OK，承认有 prior art (Liu/NVDLA)** |

**整体没有 overclaim**。L62-69 的"潜在对比轴"段也用了"潜在"二字，措辞克制；L66"任意 H×W 的处理方式"明确写了"在文献中**有近邻 prior art**"——这是负责任的写法。

---

### 通过原因

1. **覆盖度过关**：A–G 七大类、35 篇文献，对应 FLUX_CNN 的硬件层（A）、数据流（B）、FPGA streaming（C，主对照集）、PE 利用率技巧（D）、AXI/DMA（E）、编译器（F）、量化/残差（G）全部覆盖。
2. **新增 prior art 处理负责任**：Alwani / Kang / Liu 三处近邻 prior art 主动加入而不是回避；每条都标注了"威胁度 + 差异点"。
3. **"自我威胁分析"做得到位**：L507-510 的 "可量化优势 candidate" 段落主动给每条 claim 写出 "审稿人威胁"，自我审视质量比一般 literature.md 高出一档。
4. **没有过度声明**：L66 的 "任意 H×W ... 有近邻 prior art" + L508 的 "审稿人威胁：Alwani / Kang ..."，措辞克制；没有出现 "首次"、"独有"、"SOTA" 等需要全面对比支撑的强词。
5. **5 处 [CHECK] 残留全部为 vendor doc**，处理方式合理（Elicit 已检索确认无学术索引，留待 Phase 1 决定引用形式）。
6. **会议归属与 DOI 错误已修正**（Tangram MICRO→ASPLOS、Snowflake CVPRW→ISCAS、Eyeriss 拆为 ISCA + JSSC 两条等），文献基本盘可信。

---

### 给 Writer 的建议（非阻塞，PASS 已下，建议项 Phase 1 顺手收敛）

> 这些不是 FAIL 项，**不需要重做 Phase 0**。Writer 在 Phase 1 起草 contributions.md 时若顺手处理，可让定位更紧。

#### 建议 1：FINN / FINN-R 评估（中等优先级）
- **位置**：§C 末尾或单独一条
- **动作**：Phase 1 决定目标会议后由 Writer 评估是否补 FINN（Umuroglu FPGA'17）+ FINN-R（Blott TRETS'18）。
- **理由**：投 FCCM/FPL/TRETS 时 FINN 几乎必引；漏掉会被审稿人视为"不熟悉 streaming 加速器主流"。
- **预计开销**：补 2 条文献 + 2 段差异化论述（FINN 是 HLS 自动生成走多精度路线，FLUX_CNN 是手写 RTL + INT8）

#### 建议 2：AoCStream 差异化论述独立段（高优先级）
- **位置**：Phase 1 contributions.md
- **动作**：给 Kang AoCStream 一段独立的差异化论述，**不要合并进"和其他 streaming 工作不同"的笼统句子**。重点论证：
  1. layer-serial 共用硬件 vs layer-pipelined 多 block
  2. DDR-aware 大模型 vs 全片上 low-end FPGA
  3. 编译器侧 Ky-fold/S2D 让 layer-serial 单核也能填满 PE
- **理由**：AoCStream 是当前最危险的 prior art，差异化必须显式且独立。

#### 建议 3：明确"主对比轴" vs "支撑轴"（高优先级）
- **位置**：Phase 1 contributions.md / 后续 Phase 2 outline.md
- **动作**：选定主对比轴（建议轴 1：编译器侧 PE 利用率优化），把轴 2、轴 3 降为"支撑机制"。论文骨架 = 1 主 + 2 支，而不是 3 条等量贡献并列。
- **理由**：3 条等量并列容易被审稿人认为"工作不聚焦"；1 主 + 2 支结构能让 Related Work 与 Eval 段更紧。

#### 建议 4：Liu TNNLS'21 的措辞约束（中等优先级）
- **位置**：Phase 1 contributions.md / Eval 段
- **动作**：禁止单独 claim "FLUX_CNN PE 利用率/MAC% 最高"。改为限定语境："在 mid-range Kintex-7 + layer-serial + 无 layer fusion 条件下，FLUX_CNN 整网 MAC% = 86.6%"。
- **理由**：Liu 97% > 86.6%，单纯比数字会输；限定语境后能体现差异化。

#### 建议 5：S2D 在硬件加速器领域的引用谱系（低优先级）
- **位置**：§D Sub-pixel CVPR'16 条 [CHECK] 段
- **动作**：Phase 1 由 reviewer-tech 或主 Agent 在 Elicit 上单跑一次 query "space-to-depth FPGA accelerator PE utilization" / "channel folding stride-2 conv hardware"，确认是否有更近邻 prior art。
- **理由**：现在 [CHECK] 留着，Phase 1 之前最好关掉，否则审稿人也会问。

---

### 最终判定原因（一句话）

**PASS** —— 文献覆盖充分、新增 prior art 处理得当、措辞克制无过度声明、自我威胁分析到位；Kang AoCStream 是高威胁 prior art 但差异化角度已基本论证，留 Phase 1 contributions 阶段强化即可，不构成 Phase 0 阻塞性缺陷。
