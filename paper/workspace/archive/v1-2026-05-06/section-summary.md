# 小节摘要 (Phase 3)

> Phase 3 小节级别简介。每节 100-200 字说清"这节讲什么、为什么放在这、与前后小节如何衔接"，
> 让段落骨架阶段可直接细化。**不展开到段落级骨架**——那是 Phase 4 的工作。

---

## 元信息

- **完成时间**：2026-04-30
- **章节总数**：8 章（Introduction / Background / Related Work / Architecture / Compiler / System Integration / Evaluation / Conclusion）
- **小节总数**：35 节（§1×5 + §2×4 + §3×6 + §4×5 + §5×4 + §6×5 + §7×7 + §8×0=单节不分小节，按 1 节计入则 36；本文件按 35 节展开 §1-§7，§8 不分节单独列）
- **数据快照**：commit `b158cab`，STATUS.md 时间戳 2026-04-30
- **依赖输入**：`outline.md` / `contributions.md` / `literature.md` / `lessons-learned.md`
- **narrative**：A 主轴（compiler-side PE utilization）+ B 系统侧支撑（row-streaming + multi-core）

---

## 第 1 章 Introduction

### 1.1 Motivation: PE Underutilization in Fixed Arrays + Streaming Constraints on Edge FPGAs

**核心命题**：固定 16×16 INT8 阵列在 ResNet-18 风格网络的浅层（Cin<16 / stride≥2）PE 利用率仅 12.5%-50%；端侧 FPGA SRAM 受限，VGA-级输入图无法整图驻留——这两条约束是 FLUX_CNN 的设计起点。

**关键内容**：用 model_analysis 的 5 层 quantitative breakdown（Layer 1 Cin=4 → 12.5%；Layer 3/4 Cin=8 → 25%；Layer 5a/c Cin=8 → 50%）作为 motivation 数字；点出 VGA 480×640 单图 4.9 MB vs 端侧 BRAM 容量的矛盾，引出"streaming row-ring 是必要选择"的预告。

**承上启下**：作为开篇章首节，下一节 §1.2 转向"现有解法为什么不够"。

**依赖**：
- 贡献：C2.1 / C2.2（PE 利用率核心 motivation）/ C1.2（streaming 必要性）
- 文献：model_analysis.md §2 表（FLUX_CNN 内部数据）；Eyeriss ISCA'16 / TPU ISCA'17（fixed-array 谱系铺垫）
- 不确定项：[CHECK: Layer 1/3/4 启用 Ky-fold 后实测 PE 利用率百分比]（contributions.md §8.1 #1, #2）

### 1.2 Limitations of Existing Approaches

**核心命题**：当前三条主流路线——hardware-reconfigurable 阵列（MAERI ART / Eyeriss-v2 NoC）、layer-pipelined fusion（Alwani / Kang）、im2col 全展开（cuDNN）——各自有硬件复杂度、片上容量假设、内存炸裂等局限。

**关键内容**：用三段话精准点出三条路线的代价：(a) hardware-reconfigurable 增加 RTL 复杂度；(b) layer-pipelined 假设跨层 feature map 同时驻留片上；(c) im2col 把 K×K Cin 全展开，端侧 BRAM 受不了。每条都点出 FLUX_CNN 的不同选择，但不展开（细节留 §3 / §5）。

**承上启下**：承接 §1.1 的两条约束，引出 §1.3 介绍 FLUX_CNN 的"compiler-only + row-streaming"组合方案。

**依赖**：
- 贡献：无直接 claim（仅作 prior art 对位铺垫）
- 文献：MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / Alwani MICRO'16 / Kang AoCStream / cuDNN arXiv'14
- 不确定项：[TBD: 三条路线点到即止还是展开到一句话各自一个具体数字 — 由段落骨架决定]

### 1.3 Our Approach

**核心命题**：FLUX_CNN 的核心设计选择是"硬件保持简洁固定 16×16 阵列 + 编译器侧 Ky-fold/S2D 填满 PE + row-ring streaming 单次 start 跑完任意 H×W"。

**关键内容**：把 narrative A 主轴（compiler-side PE utilization）+ narrative B 系统侧（streaming row-ring + multi-core W slice）+ narrative C 元素（去中心化 valid-ready 流水）三层 framing 用一段话讲清。引一张顶层框图占位 Fig.1（5 模块核流水 + DMA 子系统 + 多核 wrapper）。

**承上启下**：承接 §1.2 的三条路线局限，引出本工作的具体取舍；为 §1.4 列贡献做铺垫。

**依赖**：
- 贡献：C2.1 / C2.2（编译器主轴）/ C1.2（streaming）/ C3.7（多核 W slice）/ C1.1（去中心化握手作为支撑）
- 文献：Buffets ASPLOS'19（C1.1 framing）/ Snowflake ISCAS'17（line-buffer 命名传承）
- 图表占位：Fig.1（顶层框图，§4.1 首次详细引入）

### 1.4 Contributions

**核心命题**：列 5 条贡献，每条带 quantified results。

**关键内容**：(1) Ky-fold 编译器变换，浅层 PE 利用率 12.5%→[CHECK]；(2) Space-to-Depth 编译器变换，stride≥2 时 4 相位重排零 RTL 改动；(3) Streaming row-ring + 双向 row-credit 反压，单次 start 跑 VGA 480×640；(4) Multi-core W slice with computed redundancy halo，N=4 综合通过 + 20/20 W slice case bit-exact；(5) PyTorch nn.Sequential 端到端编译，11 层 ResNet-18 风格 chain 86.6% MAC%。每条贡献括号内标"with documented optimization roadmap"防御 over-claim。

**承上启下**：承接 §1.3 approach overview，落实到具体可数贡献；为 §1.5 organization 收束。

**依赖**：
- 贡献：C2.1 / C2.2 / C1.2 / C3.7 / C2.5（5 条主推）；C1.5（critical path 已知问题）以"with roadmap"形式隐式提及
- 文献：无（贡献声明不引文献）
- 图表占位：可附 Tab.1 Contribution-Section Map（可选）
- 不确定项：[TBD: 5 条 vs 6 条由段落骨架定]；[CHECK: PE 利用率提升数字 12.5%→实测百分比]；[TBD: fps 用 100 MHz target 168fps 还是 68.4 MHz Fmax 115 fps 口径]

### 1.5 Paper Organization

**核心命题**：用一段话给出本文 8 章的阅读路径。

**关键内容**：§2 Background → §3 Related Work（prior art 防御）→ §4 Architecture → §5 Compiler（narrative A 主章）→ §6 System Integration（system completeness）→ §7 Evaluation → §8 Conclusion。点明 §4 / §5 / §6 任意单读都能闭合（弱依赖前章），方便不同 reviewer 类型快速定位。

**承上启下**：作为 Introduction 收束节，引出 §2 Background。

**依赖**：
- 贡献：无
- 文献：无
- 图表占位：无

---

## 第 2 章 Background and Motivation

### 2.1 Spatial Array Dataflows: WS / OS / RS Taxonomy

**核心命题**：用 weight-stationary / output-stationary / row-stationary 三分类把 FLUX_CNN 的 16×16 阵列定位为"列广播激活 + 列独立 cout 输出"的 OS-like 数据流，为后文 §4.4 per-col PARF 与 §5 编译器变换的可行性提供数据流前提。

**关键内容**：用一段话给三类 dataflow 各自一行特征定义（TPU=WS / Eyeriss=RS / ShiDianNao=OS），然后说 FLUX_CNN 选 OS 的工程理由——配合 row-ring streaming 时输出在列内累加最自然。简短铺垫 NVDLA 的 CDMA→CMAC→CACC→SDP 段命名脉络。

**承上启下**：作为 Background 首节给术语；为 §2.2 量化分析、§3.1 spatial array 谱系、§4.5 loop nest 形式化对位铺垫。

**依赖**：
- 贡献：C1.3（per-col PARF 数据流前提）、C2.4（loop nest 形式化对位）
- 文献：TPU ISCA'17（WS）/ Eyeriss ISCA'16/JSSC'17（RS）/ ShiDianNao ISCA'15（OS）/ NVDLA（命名传承）
- 图表占位：无（仅术语表）

### 2.2 PE Utilization Pathology in Shallow Layers

**核心命题**：用 ResNet-18 风格 5 层 quantitative breakdown 论证 Cin<16 在浅层是 chronically PE-underutilized——这是 narrative A 的核心 motivation 数字。

**关键内容**：列 model_analysis §2 表的 5 层（Layer 1 Cin=4 / Layer 3-4 Cin=8 / Layer 5a-c Cin=8 / Layer 5b 起 Cin=16 / FC_xy Cout=2）的 PE 利用率（12.5% / 25% / 50% / 100% / 12.5%）。明确"单层 PE 利用率"vs"整网 86.6% MAC%"的口径区分（占位文字论证）。点出 Cout<16 不优化的 design rationale（硬件保持简洁 + 影响 MAC%-of-total 较小）。引 Tab. PE-Util-Breakdown。

**承上启下**：承 §2.1 OS dataflow 给出 16×16 阵列的几何前提，引出 §2.3 streaming 必要性（与 PE 利用率正交但同样源自端侧约束）。

**依赖**：
- 贡献：C2.1 / C2.2（PE 利用率 motivation 数字 → 编译器解法预告）/ C2.4（多 slice 切片）
- 文献：model_analysis.md §2（FLUX_CNN 内部数据）
- 图表占位：Tab.PE-Util-Breakdown（5 层 × {无 fold / Ky-fold / S2D} 三模式）
- 不确定项：[CHECK: Layer 1/3/4/5a/5c 启用 Ky-fold 后实测 PE 利用率百分比]（contributions.md §8.1 #1, #2）

### 2.3 Streaming vs Tiled: Memory Footprint of Large-Image Inference

**核心命题**：端侧 FPGA SRAM 容量（XC7K325T 445 BRAM ≈ 1.6 MB）vs VGA 480×640 单图 4.9 MB 的硬约束论证 row-streaming 是必要选择，而非 layer-fusion 替代品。

**关键内容**：一段话点出"片上 SRAM 占比 vs 输入图大小"的数字对照——row-ring 仅需 strip_rows×W = 8×640 ≈ 10 KB SRAM 即可承载 VGA 480×640 单图。简要预告"为什么不用 Alwani-style layer fusion 解决端侧 large-image 推理"——layer fusion 假设跨层 FM 同时驻留片上，端侧 BRAM 受不了；细节留 §3.4。[TBD: 是否配"SRAM 占比 vs 输入图大小"曲线图]。

**承上启下**：承 §2.2 PE 利用率约束，引出第二条 motivation 约束（streaming 必要性）；为 §2.4 design goals 收束 + §3.4 streaming prior art 对照铺垫。

**依赖**：
- 贡献：C1.2（row-ring streaming 必要性）
- 文献：Alwani MICRO'16（layer fusion 对位预告）/ Kang AoCStream（全片上对位预告）
- 图表占位：[TBD: Fig. SRAM-vs-Image-Size 曲线图，Phase 4 决定是否值得占图位]

### 2.4 Design Goals

**核心命题**：把 §2.2 + §2.3 两条 motivation 约束转化为本工作的三条 design goals——(a) 硬件简洁固定不引入 reconfigurable PE 互联，(b) 编译器侧填满 PE 应对 Cin<16，(c) row-ring streaming 单次 start 跑完任意 H×W。

**关键内容**：一段话陈述三条 design goals + 对应贡献编号；点明这三条 goals 在文献中分别对应 fixed-array / compiler-only PE-utilization / row-level streaming 三个谱系定位。简短预告 §3 Related Work 将逐条对位 prior art。

**承上启下**：作为 Background 收束节，把 motivation 收为 design goals，自然过渡到 §3 Related Work 对每条 goal 的 prior art 对位。

**依赖**：
- 贡献：C1.1 / C1.2 / C2.1 / C2.2（三条 design goals 对应贡献）
- 文献：Eyeriss ISCA'16 / TPU ISCA'17（fixed-array 谱系）/ MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19（hardware-reconfigurable 反例）
- 图表占位：无

---

## 第 3 章 Related Work

### 3.1 Spatial Array Accelerators

**核心命题**：把 FLUX_CNN 的 16×16 INT8 阵列定位在 spatial array 谱系中——与 TPU v1（256×256 数据中心）/ Eyeriss（14×12 RS）/ Gemmini（16×16 同尺度但中心 RoCC scheduler）/ NVDLA（端侧 IP）/ Simba（多 chiplet）形成尺度与控制范式对照。

**关键内容**：一段话给出 5 篇代表作的尺度 / 数据流 / 平台一行表格化描述；强调 FLUX_CNN 与 Gemmini 同尺度（16×16 INT8）但去中心化握手，是 §1.3 narrative 的硬件谱系锚点。NVDLA 的 CDMA→CMAC→CACC→SDP 段命名传承到 FLUX_CNN 的 line_buffer→mac_array→parf_accum→sdp。

**承上启下**：作为 Related Work 首节给硬件谱系坐标，引出 §3.2 hardware-reconfigurable 路线对位（narrative A 的反向 alternative）。

**依赖**：
- 贡献：C1.1（去中心化 vs Gemmini 中心 RoCC 对位）
- 文献：TPU v1 ISCA'17 / Eyeriss ISCA'16/JSSC'17 / Gemmini DAC'21 / NVDLA / Simba MICRO'19
- 图表占位：可选 Tab. Spatial-Array-Comparison（5 维度 placement）

### 3.2 Hardware-Reconfigurable Approaches to PE Utilization

**核心命题**：对位 narrative A 的反向 alternative——MAERI（ART 互联网络）/ Eyeriss-v2（NoC reconfiguration）/ Tangram（chiplet array reconfig）三条 hardware-reconfigurable 路线用增加 RTL 复杂度换 PE 利用率，FLUX_CNN 选反方向：硬件保持简洁固定，编译器侧重映射。

**关键内容**：一段话点出三条工作如何分别在 dataflow 上加 reconfigurable PE 互联；强调"硬件复杂度 vs 编译复杂度" trade-off 不互斥；为 §5.4 narrative A 与 MAERI/Eyeriss-v2 的具体对位预留差异化论证空间。

**承上启下**：承 §3.1 spatial array 硬件谱系坐标，转向"如何应对 PE 利用率"的两大路线选择——本节讲硬件路线，§3.3 讲编译器路线（即 narrative A 的同语言谱系）。

**依赖**：
- 贡献：C2.1 / C2.2（compiler-only vs hardware-reconfigurable 对位前哨）
- 文献：MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / Tangram ASPLOS'19

### 3.3 Compiler / Loop-Nest Co-Design

**核心命题**：Timeloop / Interstellar / TVM / VTA / Ma FPGA'17 / cuDNN im2col 是 narrative A 的同语言谱系——他们用编译器 / loop tiling / 形式化建模处理 spatial array 的映射问题，但**没有命中"FPGA 上把 Ky 折到 cin 作为 PE 利用率优化"或"compile-pass S2D"这两点**——这是 narrative A 防御点。

**关键内容**：一段话给六篇代表作的语言定位（Timeloop=mapping search / Interstellar=7-loop 形式化 / TVM+VTA=端到端编译栈 / Ma=4-6 维 loop systematic / cuDNN=im2col baseline）；明确 FLUX_CNN 的 Ky-fold + S2D 在该语言体系内是新点。引 [CHECK] 标 S2D 在加速器领域的引用谱系待 reviewer 阶段补查。

**承上启下**：承 §3.2 硬件路线 alternative，给 narrative A 同语言文献坐标；为 §3.4 streaming 谱系（narrative B 主对照集）让位。

**依赖**：
- 贡献：C2.1 / C2.2 / C2.4（narrative A 同语言谱系）
- 文献：Timeloop ISPASS'19 / Interstellar ASPLOS'20 / TVM OSDI'18 / VTA Micro'19 / Ma FPGA'17 / cuDNN arXiv'14
- 不确定项：[CHECK: S2D 在加速器领域的引用谱系不清晰，reviewer 阶段须补查 ASPLOS/HPCA 是否有 architecture-side S2D 更早出处]（contributions.md §8.1 #15）

### 3.4 Streaming / Line-Buffer FPGA Accelerators

**核心命题**：streaming 谱系是 narrative B 的主对照集——fpgaConvNet / DnnWeaver / Snowflake / Angel-Eye / Aydonat / Lu Winograd / **Alwani / Kang / Liu** 三处 streaming 近邻 prior art 在此精细差异化是 narrative B 防御主战场。

**关键内容**：一段话给 streaming 谱系的"line-buffer + 行级触发"共性；然后用三段话精准点出三处威胁：(a) **Alwani Fused-layer @ MICRO'16**——FLUX_CNN row-level vs Alwani layer-level granularity；FLUX_CNN 单核 layer-serial 共用硬件 vs Alwani 跨层同时驻留计算。(b) **Kang AoCStream @ arXiv'22 / Sensors'23**——FLUX_CNN 承认外存必然 + 优化 DDR 流量 vs Kang 强调全片上；FLUX_CNN layer-serial vs Kang layer-pipelined 多 block。(c) **Liu Full-Stack @ TNNLS'21**——FLUX_CNN 单核 layer-serial 不做 layer fusion；规模差是同思路在不同硬件规模下的对照（中型 7-series 86.6% vs 大型 Arria 10 GX1150 97% MAC%）。Snowflake / Angel-Eye 作为 §7.6 同器件对照点铺垫。

**承上启下**：承 §3.3 编译器谱系，转向 narrative B 主对照集——本节是 Related Work 章篇幅最长节、防御最关键节，为 §7.6 head-to-head 对照表预留所有引用。

**依赖**：
- 贡献：C1.2（row-ring streaming）/ C3.5 / C3.7（multi-core W slice）
- 文献：fpgaConvNet TNNLS'19 / DnnWeaver MICRO'16 / Snowflake ISCAS'17 / Angel-Eye TCAD'18 / Aydonat FPGA'17 / Lu Winograd FCCM'17 / **Alwani MICRO'16 / Kang AoCStream Sensors'23 / Liu TNNLS'21**

### 3.5 Quantization and Fusion

**核心命题**：简短 framing 节——把 Jacob / ResNet / EIE / ESE / NVDLA SDP 的 quantization + residual fusion 谱系点到，作为 §6.3 SDP 详细论述的前哨。

**关键内容**：一段话给 INT8 量化（Jacob CVPR'18）+ residual 算法侧必要性（He CVPR'16 ResNet）+ 工业 SDP 段（NVDLA）+ 稀疏量化（EIE/ESE）的文献坐标；强调 FLUX_CNN 的 SDP fusion 偏增量贡献，仅在 §6.3 system 完整性论据中出现。[TBD: 是否单独成节 vs 并入 §6.3 SDP 段落，视篇幅压力定]。

**承上启下**：承 §3.4 streaming 主对照集，简短补 quantization 谱系 framing；为 §3.6 整体 placement 收束 + §6.3 SDP 详述铺垫。

**依赖**：
- 贡献：C1.4（SDP fusion，仅作 system 完整性）
- 文献：Jacob CVPR'18 / He ResNet CVPR'16 / NVDLA / EIE ISCA'16 / ESE FPGA'17
- 不确定项：[TBD: §3.5 是否单独成节 vs 并入 §6.3 SDP，视篇幅压力定]

### 3.6 Positioning of FLUX_CNN

**核心命题**：用一段话 + 一张 5 维度 placement 表把 FLUX_CNN 在 dataflow / PE-utilization 路线 / streaming granularity / multi-core 切分维度 / 编译栈 scope 五个轴上钉死位置——作为 Related Work 收束节防御性总结。

**关键内容**：5 维度 placement：(1) dataflow=OS-like + per-col PARF；(2) PE-utilization=compiler-only Ky-fold/S2D（对位 MAERI/Eyeriss-v2 hardware-reconfig）；(3) streaming granularity=row-level（对位 Alwani layer-level / Kang layer-pipelined / Liu layer-fusion）；(4) multi-core 切分维度=W slice + halo（对位 Simba channel slice）；(5) 编译栈 scope=hand-rolled mini-compiler（对位 TVM/VTA 通用栈）。引 Tab.Positioning（5 维度对照）。

**承上启下**：作为 Related Work 收束节给 narrative 防御总结表；自然过渡到 §4 Architecture 详细架构论证。

**依赖**：
- 贡献：所有 C1-C3 在此被 prior art 校准
- 文献：本章涉及全部 32 篇综合
- 图表占位：Tab.Positioning（5 维度 × 9 工作对照）

---

## 第 4 章 Architecture

### 4.1 Top-Level Architecture

**核心命题**：FLUX_CNN 顶层由 5 模块 core pipeline（`line_buffer → wgt_buffer → mac_array → parf_accum → ofb_writer`）+ DMA 子系统（`idma_ctrl / wdma_ctrl / odma_ctrl + mm2s_arb + axi_dm IP + axi_m_mux + axi_lite_csr`）两层构成；外部仅 1 个 AXI4 Master + 1 个 AXI-Lite Slave。

**关键内容**：用 Fig.1（系统框图）展示 core 与 DMA 子系统的边界；点出 NVDLA 类比命名 `line_buffer→mac_array→parf_accum→sdp` ↔ NVDLA `CDMA→CMAC→CACC→SDP` 的对位关系；简短预告 IDMA / 计算 / ODMA 三阶段并发的执行模型，详细 timing 留 §4.3。

**承上启下**：作为 §4 章首节给出整体形态视图，承 §3.6 防御总结收束、为 §4.2 起的"模块级独立机制"细化打底。

**依赖**：
- 贡献：C1.1 / C1.2 主推；C3.1（DMA 子系统形态）作为 system 上下文
- 文献：NVDLA（命名传承）/ Eyeriss ISCA'16（OS dataflow 对位）/ Snowflake ISCAS'17（line-buffer 命名传承）
- 图表占位：Fig.1（顶层框图，core + DMA 子系统）/ [TBD: 是否配 timing diagram 展示三阶段并发，Phase 4 决定]

### 4.2 Decentralized Valid-Ready Pipeline

**核心命题**：核流水内 5 模块各自维护独立 counter，模块间用 valid-ready 双向握手串联，**无中心 FSM / scheduler**；`elastic join` 保证 stall 下 in-flight 数据不丢，`FILL/DRAIN overlap` 让 parf_accum tile N 的 drain 隐藏在 tile N+1 fill 的 first_round 里。

**关键内容**：用一段话给握手 invariant（每模块边界点出"counter + ready 反压"模式）；点出 sequencer 仅做 cross-block 启动同步而非中心调度的边界；以 22-case 链式回归全 PASS 作为该 narrative 的工程证据；Buffets ASPLOS'19 的"buffer + counter + handshake"形式化语言作为该设计的同语谱系。

**承上启下**：承 §4.1 整体形态，进入"模块级机制"细化；为 §4.3 row-ring streaming 的 forward-pressure 提供握手语义基础。

**依赖**：
- 贡献：C1.1（去中心化 valid-ready 流水，工程化非主推）
- 文献：Buffets ASPLOS'19（formal framing）/ Gemmini DAC'21（中心 RoCC scheduler 反例）/ NVDLA（显式 controller 反例）/ VTA Micro'19（4 段 fetch/load/compute/store 反例）

### 4.3 Streaming Row-Ring Datapath with Bidirectional Credit Backpressure

**核心命题**：IFB / OFB 作 row-level ring buffer 按 `strip_rows` 取模；line_buffer 仅在 `rows_available ≥ yout·stride + Ky` 时发射（forward-pressure），ODMA 排空行 → 送 ofb_writer credit；sequencer 一次 dispatch 同启 IDMA / 计算 / ODMA 三阶段并发。这让整图不必装进片上 SRAM——VGA 480×640 / 4.9 MB 单图只用 ~10 KB ring。

**关键内容**：用一段话给 ring 数学（wptr/rptr 模运算永不 reset）+ 触发条件 + credit 回送；用 Fig.2（row-ring + 三阶段并发 timing）可视化；点出"统一 streaming row-ring 模式覆盖原 batch（环容量 ≥ 整图的退化情形）"；引 22+24=46 case 全 bit-exact PASS 作为鲁棒性证据；narrative B 的精细差异化（vs Alwani layer-level / Kang layer-pipelined / Liu layer-fusion）已在 §3.4 详述，本节只点 row-level granularity 的硬件落地。

**承上启下**：承 §4.2 握手语义，给出 streaming 数据路径核心机制；为 §4.4 per-col PARF 累加细节让位（PARF 是 ring 内累加单元）。

**依赖**：
- 贡献：C1.2（row-ring streaming，narrative B 系统侧主推）
- 文献：Snowflake ISCAS'17（line-buffer 命名）/ Alwani MICRO'16（layer-level 对照）/ Kang AoCStream Sensors'23（layer-pipelined 对照）/ Liu TNNLS'21（layer-fusion 对照）
- 图表占位：Fig.2（row-ring 数据路径 + 三阶段并发 timing diagram）

### 4.4 Per-Column PARF Accumulator

**核心命题**：`parf_accum` 内部不是单一 PSUM SRAM，而是 `parf_col × 16` 列独立 SRAM 共享 wr_addr/we/rd_addr 外壳——每列可用 single-port BRAM，匹配"列广播激活 + 列独立 cout 输出"的数据流。

**关键内容**：用一段话给 parf_col 单列结构 + 外壳共享逻辑的拆分；点出该选择的端口压力释放收益（vs 单一大 SRAM 需要多端口 / port arbitration）；强调这是"实现细节"而非顶层架构 claim，仅作单元级机制记录。[CHECK: per-col PARF 的 BRAM 数量贡献单独估算需查综合报告 cell utilization]

**承上启下**：承 §4.3 ring 内累加场景，给出累加器单元的具体实现；为 §4.5 loop-nest realization 的 cin_slices 时间维累加提供硬件存储基础。

**依赖**：
- 贡献：C1.3（per-col PARF，机制级 / 单元级实现细节）
- 文献：NVDLA / Gemmini DAC'21（单一 PSUM 大 SRAM 反例对照）
- 不确定项：[CHECK: parf BRAM 数量贡献单独估算待补]（contributions.md §8.1，C1.3 量化数据条）
- [TBD: §4.4 是否与 §4.2 valid-ready 合并，取决于篇幅]（outline.md line 137）

### 4.5 Loop-Nest Realization

**核心命题**：物理 16×16 阵列处理任意 Cin/Cout 靠两层切片（`cin_slices = ⌈Cin/16⌉` 时间维 PARF 累加 + `cout_slices = ⌈Cout/16⌉` 时间维 OFB 行内段 NHWC 拼接），整层硬件循环是 `for yout > for cs > for tile > for cins > for ky > for kx > for iss_pos` 7 层嵌套；超容量自动切 strip（H 方向）+ tile（W 方向）。

**关键内容**：用一段话给 7 层嵌套 + 容量校验自动切片机制；点出 Interstellar ASPLOS'20（7 nested loops over DNN）的形式化天然同构 + Ma FPGA'17（4-6 维 loop systematic）的早期同语言对照；作为 narrative A "硬件保持简洁，由编译器映射" 的硬件侧支撑论据；为 §5 Compiler 章铺垫——loop nest 是 fold 变换的载体。

**承上启下**：承 §4.4 累加单元，给出整层循环嵌套作为硬件 / 编译器边界；自然过渡到 §5 Compiler Optimizations——本节告诉读者"硬件按普通 conv 跑这 7 层循环"，§5 告诉读者"编译器侧如何把 Ky-fold/S2D 映射进这 7 层"。

**依赖**：
- 贡献：C2.4（7+1 层硬件循环嵌套 + cin/cout 多 slice 切片，作为 implementation context）
- 文献：Interstellar ASPLOS'20（7-loop 形式化）/ Ma FPGA'17（loop nest 对位）

---

## 第 5 章 Compiler Optimizations for PE Utilization

> **章首段必述（口径声明）**：本章用"单层 PE 利用率"指标，与 §7.4 整网 86.6% MAC% 不是同一口径——前者描述单层 mac_array 占空，后者含 IDMA/ODMA stall + 多层切换；两者关系在 §7.2 末尾文字论证。

### 5.1 Ky-Folding

**核心命题**：当 Cin<16 让 PE 行不能填满时，编译器侧把 Ky 维按 `groups_y = HW_PE/Cin` 折到 cin 维，定义 `cin_fake = groups_y · Cin` / `kyper = ⌈K/groups_y⌉`，**输入做 y-方向偏移复制 + 权重 reshape**，硬件按普通 conv 跑（K_new=kyper, Cin=cin_fake），完全无感。代价是 IFB 占用 × groups_y。

**关键内容**：用一段话给 fold 数学（`compute_fold_params(K, Cin, HW_PE)` / `fold_input` / `fold_weights`）；用 Layer 1（K=7, Cin=4 → 12.5%）作为 motivation case，启用 Ky-fold 后接近 100%；与 cuDNN im2col 同源但 **更轻量**（只折 Ky 不展 Kx，保留 Cin 通道并行，避免内存炸裂）；与 MAERI ART / Eyeriss-v2 NoC 形成"compiler-only vs hardware-reconfigurable"对比，详细对位留 §5.4。[CHECK: Layer 1/3/4 启用 Ky-fold 后实测 PE 利用率百分比待 project-analyst 跑 fold 模式 case 反算]

**承上启下**：承 §4.5 7 层硬件 loop nest，给出 narrative A 主轴第一变换；为 §5.2 S2D 引入"另一种触发条件下的 fold"。

**依赖**：
- 贡献：C2.1（Ky-fold，编译器机制级，narrative A 核心）
- 文献：cuDNN arXiv'14（im2col 对位）/ MAERI ASPLOS'18（hardware-reconfigurable 反例）/ Eyeriss-v2 JETCAS'19（NoC reconfig 反例）/ Ma FPGA'17（loop unrolling 形式化）
- 不确定项：[CHECK: Layer 1/3/4 实测 PE 利用率百分比]（contributions.md §8.1 #1, #2）

### 5.2 Space-to-Depth Folding

**核心命题**：stride≥2 卷积按 `(kx%stride, ky%stride)` 把 stride² 个相位折到 cin 维，等价为 stride=1, K_new=⌈K/stride⌉, Cin_new=stride²·Cin 的卷积；**编译器侧只重排不复制**——DDR 友好，启用后 stride=1 + ARF reuse_en=1 滑动窗口复用让 IFB 读次数大幅下降；K 被 stride 整除时 pad_waste=0（如 K=8 stride=2，4×4 sub-kernel 完美对齐）。

**关键内容**：用一段话给 S2D 数学（`compute_s2d_params(K, Cin, stride)` / `s2d_input` / `s2d_weights`）；点出 vs Ky-fold 的关键差异——S2D 等量重排不复制，多核场景 DDR 带宽节省更显著；算法侧出处来自 Shi et al.@CVPR'16 Sub-Pixel（train-time sub-pixel layer），FLUX_CNN 的贡献是把它**作为 compile-pass 应用到任意预训练 stride≥2 conv**——不需要重训。[CHECK: 启用 S2D 后 DDR 带宽节省比例需对照 run 统计 IDMA 读次数] / [CHECK: S2D 在加速器领域的引用谱系——reviewer 阶段补查 ASPLOS/HPCA architecture-side S2D 更早出处]

**承上启下**：承 §5.1 Ky-fold（"Cin<16 触发"路径），给出"stride≥2 触发"的另一变换；为 §5.3 联合触发逻辑铺垫（两变换可叠加）。

**依赖**：
- 贡献：C2.2（S2D，编译器机制级，narrative A 核心）
- 文献：Shi CVPR'16 Sub-Pixel（算法侧出处）/ MAERI ASPLOS'18（hardware-reconfigurable 反例）/ Interstellar ASPLOS'20（compiler-hardware co-design framing）
- 不确定项：[CHECK: DDR 带宽节省比例] / [CHECK: S2D 在加速器领域的引用谱系]（literature.md / contributions.md §8.1 #15）

### 5.3 Joint Trigger Logic and Decision Automation

**核心命题**：编译器根据 layer 几何自动选 fold：Ky-fold 触发于 `K>1 AND Cin<16`；S2D 触发于 `stride≥2 AND K≥stride`；S2D 启用后 Cin 变为 stride²·Cin，**重新判定 Ky-fold**（多数情况 S2D 后 Cin'≥16 不再需 Ky-fold）。这让 ResNet-18 风格网络上的 fold 选择无需人工标注。

**关键内容**：用一段话给三种模式（无 fold / `--fold` / `--fold --s2d`）的回归对照框架（`run_regression.py` 22 case × 3 mode 全 PASS）；点出 S2D-after-fold 的重判逻辑作为"组合变换的工程闭环"；强调本节是 narrative A 的"自动化论据"而非"新算法"——避免被审稿人挑 "trigger heuristic 是 ad-hoc"。**Cout<16 不优化**的诚实标注放在本节末尾："Cout<16 layers (e.g., FC_xy with Cout=2) result in PE column idling, util=Cout/16; design rationale: hardware kept simple, MAC%-of-total negligible in ResNet-18-style"。

**承上启下**：承 §5.1+§5.2 两变换，给出"组合 + 自动化"工程闭环；为 §5.4 与 hardware-reconfigurable alternatives 的对位让位。

**依赖**：
- 贡献：C2.3（自动决策 + 受益层判定逻辑，编译器策略）
- 文献：Interstellar ASPLOS'20 / Timeloop ISPASS'19（mapping search 同语言对照——但 FLUX_CNN 的策略是 rule-based 而非 search-based）

### 5.4 Comparison with Hardware-Reconfigurable Alternatives

**核心命题**：用一张表（Tab.2）把 FLUX_CNN compiler-only Ky-fold/S2D 与 MAERI ART / Eyeriss-v2 NoC reconfig / cuDNN im2col 在四个维度（硬件复杂度 / 编译复杂度 / 内存代价 / 适用场景）做对位——核心 claim："硬件保持简洁固定 16×16 阵列时，编译器侧 fold 变换可以达成 hardware-reconfigurable 等价的 PE 利用率收益"。

**关键内容**：表的四维度对位 + 一段话文字论证 trade-off 不互斥（"两条路线各有适用域"）；对 S2D 的 prior art 防御点占位（如 reviewer 阶段发现 architecture-side S2D 更早出处，本节预留修订空间）；引 §7.2 实测 PE 利用率数据作为支撑。这是 narrative A 章的收束节，承担"防御审稿人 'compiler-only 有什么新意' 挑战"的责任。

**承上启下**：作为 §5 narrative A 主章收束节给 hardware-reconfigurable 对位防御；自然过渡到 §6 System Integration——把 narrative A 论据落到完整系统。

**依赖**：
- 贡献：C2.1 / C2.2 / C2.3 在此被 prior art 校准
- 文献：MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / cuDNN arXiv'14 / Sub-pixel CVPR'16 / Timeloop ISPASS'19
- 图表占位：Tab.2（compiler-only vs hardware-reconfigurable 4 维度对位表）
- 不确定项：[CHECK: S2D 加速器领域引用谱系待补]（与 §5.2 同源）

---

## 第 6 章 System Integration and Toolchain

### 6.1 PyTorch nn.Sequential End-to-End Compilation

**核心命题**：FLUX_CNN 提供 `compile_layer.py`（PyTorch Conv2d → 硬件 cfg + 数据文件）+ `compile_model.py`（多层链式编译，`_plan_ddr` 分配 FM-shared DDR 区）+ `run_regression.py` DSL builder（`Chain / _Node / resnet_block` 7 行写完 11 层 ResNet block）的端到端编译栈，整网 bit-exact 通过。

**关键内容**：用一段话给编译栈三层结构（Conv2d 单层 / 多层链 / DSL builder）；引 11-case ResNet-like chain 全 PASS @ 593K cycles, 86.6% MAC%, 5.95 ms @ 100 MHz target 作为 system completeness 论据；与 TVM OSDI'18 / VTA Micro'19 / fpgaConvNet TNNLS'19 的更通用编译栈做对比，承认 FLUX_CNN 是 hand-rolled mini-compiler 而非通用栈，但 22-case 链式回归提供完整可复现验证。[CHECK: 5.95 ms target vs 68.4 MHz Fmax 实际 fps 应为 168×0.684 ≈ 115 fps，论文须统一两口径]

**承上启下**：作为 §6 章首节给系统侧"工具链入口"视图，承 §5 narrative A 主章收束、为 §6.2 DMA 子系统硬件落地铺垫——编译产物最终通过 DMA 流入硬件。

**依赖**：
- 贡献：C2.5（PyTorch 多层端到端编译，工程化贡献）/ C3.3（链式 CASES DSL + 墙钟报告，间接关联）
- 文献：TVM OSDI'18 / VTA Micro'19 / fpgaConvNet TNNLS'19
- 不确定项：[CHECK: target 100 MHz vs Fmax 68.4 MHz 两口径在论文中如何统一]（contributions.md §C2.5 量化数据条）

### 6.2 AXI / DMA Subsystem with Vendor IP Integration

**核心命题**：原项目自写 IDMA/WDMA/ODMA RTL（~3000 行）替换为 Xilinx `axi_dm` IP + 轻量 `idma_ctrl/wdma_ctrl/odma_ctrl/rdma_ctrl`（仅做 cmd 生成 + done 检测）+ `mm2s_arb`（IDMA/WDMA 共享 MM2S 串行仲裁）；外部看是 1 个 AXI4 Master + 1 个 AXI-Lite Slave。CFG_WRITE descriptor + DFE 让 host AXI-Lite 写从 ~50/层 降到 4/层。

**关键内容**：用一段话给 DMA 子系统形态 + vendor IP 集成边界（Xilinx PG022 引用方式）；用一段话给 CFG_WRITE descriptor 流（TYPE_CFG=0x3 → DFE 拉 list → cfg_regs 双端写口）；点出 done sticky + 双端 cfg 写口（C3.4）解 race；DataMover 性能与原版自写 DMA 持平 (+0.5%) / burst_size 16→256 调优证明 vendor IP 不牺牲吞吐。NVDLA 类似有"register list" descriptor 模式，FLUX_CNN 把 descriptor 类型扩展到 CFG_WRITE 而非仅 buffer 描述符。

**承上启下**：承 §6.1 编译产物如何流入硬件，给出 host ↔ DMA ↔ 核 三方协作的硬件落地；为 §6.3 SDP 后处理（也属 system completeness）让位。

**依赖**：
- 贡献：C3.1（vendor IP 集成）/ C3.2（CFG_WRITE descriptor）/ C3.3（链式 CASES，间接）/ C3.4（done sticky + 双端 cfg 写口）
- 文献：Xilinx PG022（vendor IP 引用，[CHECK: vendor doc 引用方式]）/ NVDLA（register list descriptor 类比）

### 6.3 SDP Post-Processing: Bias / Residual / Quantization Fusion

**核心命题**：Single Data Point processor 把 5 步后处理融合在一段组合链 `pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc`；R.1 把 bias 从 mac_array 内移到 SDP 段（解 MAX_COUT_SLICES=32 限制 + 解放 bias_rf 容量），R.2 加 `shortcut_mult/shortcut_shift` 可编程量化因子做残差融合，Shortcut Bank（8192×128 SRAM = 32 BRAM）专用驻留残差源。

**关键内容**：用一段话给 SDP 5 步链 + R.1/R.2 重构动机；NVDLA 的 SDP 段命名/分段法直接借用，老老实实标 "NVDLA-inspired"；He CVPR'16 ResNet 是该 fusion 算法侧动机；与 Liu Full-Stack TNNLS'21 streaming + residual fusion 做参照——但 FLUX_CNN 不做跨层 layer fusion，仅做 SDP 内部融合。**新颖点仅在 R.1 bias 重定位 + R.2 shortcut_mult/shift 可编程量化因子**——不当主贡献，仅作 system 完整性论据。

**承上启下**：承 §6.2 DMA 子系统，给出核内最后一段后处理细节；为 §6.4 multi-core scaling（更系统层面）让位。

**依赖**：
- 贡献：C1.4（SDP 后处理融合，机制级 / system 完整性）
- 文献：NVDLA（SDP 命名传承）/ He CVPR'16 ResNet（算法侧动机）/ Liu TNNLS'21（streaming + residual fusion 对照）/ Jacob CVPR'18（INT8 量化对位）

### 6.4 Multi-Core Scaling: W-Slice with Computed Redundancy Halo

**核心命题**：`multicore_top.sv` 参数化 N 核 wrapper，N=2/4 综合通过且 N=4 W slice 20/20 case 全 bit-exact PASS；W slice (Mode C) 让 N 核处理同一图的不同 W 段，**computed redundancy halo + asymmetric pad** 机制（如 N=2 K=3 pad=1 W=32：Core 0 处理 W[0..17) pad_l=1 pad_r=0，Core 1 处理 W[15..32) pad_l=0 pad_r=1，重叠 2 列 halo）；M2 阶段升级 `axi_2to3 / axi_4to5` 实现跨核 SRAM 直送（producer ODMA → crossbar → consumer ifb_axi_slave 经 ring 反压写 IFB SRAM）。

**关键内容**：用一段话给 multicore wrapper + W slice 几何 + halo 机制；用一段话给跨核 SRAM 直送的 AXI crossbar 路由；引 N=4 wslice1 1.45× speedup vs N=2 5569 cycles 作为 scaling 证据；与 Simba MICRO'19 chiplet × 36（channel 切）形成"FPGA 多核 W slice vs ASIC chiplet channel slice"对照——多核切分维度选择不同；Liu TNNLS'21 / Kang AoCStream 同属 streaming 谱系但不做 W slice。BRAM-bound 而非通信 bound 是 FPGA 多核版的 scaling 域。[CHECK: N=4 wslice1 1.45× 是否近线性需 baseline 归一化] / [CHECK: ResNet 11-layer multicore N=2/4 实测 cycles vs 估算 1.7×/3.64× 待跑]

**承上启下**：承 §6.3 核内 SDP，跳到系统层面给 multi-core scaling 路径；为 §6.5 参数 single source of truth 的工程实践收束让位。

**依赖**：
- 贡献：C3.5（multi-core wrapper + 跨核 SRAM 直送，N=4 综合通过）/ C3.7（W slice + computed redundancy halo + DDR row_stride 解耦 cmd_btt，narrative B 系统侧主推）
- 文献：Simba MICRO'19（channel slice 反例对照）/ Liu TNNLS'21 / Kang AoCStream Sensors'23（streaming 谱系切分维度对照）
- 不确定项：[CHECK: N=4 wslice1 1.45× 接近线性论证]（contributions.md §8.1 #9）/ [CHECK: ResNet 11-layer multicore 实测]（contributions.md §8.1 #8）/ [TBD: §6.4 是否升格为独立章以提升 narrative B 比重]（outline.md line 187）

### 6.5 Single Source of Truth: Parameter Management

**核心命题**：`params.py`（项目根）作为 RTL/Python 唯一参数源，64 个 `\`FLUX_*` 宏覆盖核心尺寸 + SRAM 容量 + AXI/CSR + 全局地址映射 + 58 CSR addr；`python params.py` 自动生成 `RTL/flux_cnn_params.svh`；RTL `\`include`，Python `from params import *`——避免参数 drift。

**关键内容**：用一段话给参数源 → 自动生成 → RTL 与 Python 双侧消费的工具链闭环；强调这是工程实践细节，作为 system 完整性收束节出现；不当 contribution 主推，仅在评审 reproducibility 维度有价值。[TBD: §6.5 是否值得占节位——篇幅紧时合并到 §6.2 末尾 1 段]

**承上启下**：承 §6.4 multi-core 系统层面，给 §6 章工程实践收束节；自然过渡到 §7 Evaluation——本章给"系统能跑"，下一章给"系统跑得多好"的 quantified data。

**依赖**：
- 贡献：C3.6（参数 single source of truth，工程化贡献）
- 文献：无直接对照
- 不确定项：[TBD: 是否值得占节位]（outline.md line 190）

---

## 第 7 章 Evaluation

> **章首段必述（口径声明）**：(a) 100 MHz target 是设计目标，(b) 68.4 MHz Fmax 是 XC7K325T-2 OOC 综合实测；本章 latency / GOPS 数字以 (a)(b) 双 operating point 报告。**单层 PE 利用率（§7.2）vs 整网 MAC%（§7.4）口径不同**——前者描述 mac_array 占空，后者含 IDMA/ODMA stall + 多层切换。

### 7.1 Experimental Setup

**核心命题**：实验平台 XC7K325T-FFG900-2 / Vivado 2023.1 / ModelSim 仿真；测试套件含 22-case ResNet-18 风格链（11 layer × 3 mode：无 fold / `--fold` / `--fold --s2d`）+ 24-case 鲁棒性 corner（K∈{1..7} × stride∈{1..4} × pad∈{0..3} × Cin/Cout/H/W 多组合）+ 20-case multi-core W slice（K∈{1,3,5,7} × stride∈{1,2} × W∈{8,32,33}）。

**关键内容**：用一段话给硬件平台 + EDA 工具链 + 三个测试套件结构 + 22+24+20=66 case 全 bit-exact PASS 总览；点出 OOC（out-of-context）综合作为 IP-style 资源占用基准；引 commit `b158cab` 数据快照作为 reproducibility 锚点。

**承上启下**：作为 §7 章首节给评测平台 / 测试套件 / 数据快照三件套，承 §6 系统能跑、为 §7.2 起的 quantified data 章节铺垫。

**依赖**：
- 贡献：C2.5 / C3.3（验证基础设施）作为 setup context
- 文献：无直接对照（setup 节）

### 7.2 PE Utilization: Ky-Folding + S2D Improvement

**核心命题**：narrative A 核心数据节——浅层 Layer 1 (K=7, Cin=4 → 12.5%) / Layer 3/4 (Cin=8 → 25%) / Layer 5a/c (Cin=8 → 50%) 启用 Ky-fold + S2D 后接近 100% PE 利用率，**零 RTL 改动**；与 hardware-reconfigurable 路线（MAERI / Eyeriss-v2）比硬件复杂度 vs 编译复杂度的不同 trade-off。

**关键内容**：用一张表（Tab.3 PE-util-by-layer）给 model_analysis 5 层基线 + Ky-fold 启用后 + S2D 启用后三列对比；用一段文字论证"单层 PE 利用率 vs 整网 86.6% MAC%"口径区分（前者描述 mac_array 占空，后者含 IDMA/ODMA stall + 多层切换）；引 §5.4 Tab.2 hardware-reconfigurable 对位作为防御。**Cout<16 不优化**的诚实标注作为表脚注。[CHECK: Layer 1/3/4 启用 Ky-fold 后实测 PE 利用率百分比待 project-analyst 跑 fold 模式 case 取 cycles 反算]

**承上启下**：承 §7.1 实验 setup，进入 narrative A 主数据章；为 §7.3 综合资源 + Fmax 让位（narrative C 元素出现）。

**依赖**：
- 贡献：C2.1 / C2.2 / C2.3（PE 利用率核心数据）
- 文献：MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / Sub-pixel CVPR'16
- 图表占位：Tab.3（PE-util-by-layer 三列对照）
- 不确定项：[CHECK: Layer 1/3/4 实测 PE 利用率]（contributions.md §8.1 #1, #2）/ [CHECK: 单层 vs 整网 MAC% 口径论证]（contributions.md §8.1 #6）

### 7.3 Resource Utilization and Fmax

**核心命题**：单核 36.9k LUT (18.1%) / 13.2k FF (3.2%) / 128 BRAM (28.8%) / 82 DSP (9.8%) / Fmax 68.4 MHz @ XC7K325T-2 OOC（WNS=-4.618 ns，未达 100 MHz target）；多核 N=2 74.4k LUT (36.5%) / 256 BRAM (57.5%) / N=3 推算 BRAM 86%（边缘）/ N=4 BRAM 超容需缩 shortcut bank 8192→2048。

**关键内容**：用 Tab.4 综合资源对照表（单核 / N=2/3/4 推算 / 总容量）；点出 BRAM 明细（WB 57 / IFB 32 / Shortcut 32 / OFB 7+1）；3 核是 XC7K325T 不动 SRAM 的硬上限（BRAM-bound）作为 narrative C 元素；Fmax 68.4 MHz 未达 target 的 critical path 已知问题（C1.5）老实标注、修复路径放 §7.7。[TBD: 是否在评估前给 mac_pe `(* use_dsp = "yes" *)` 重综合 + SDP 流水线化重综合更新数字]

**承上启下**：承 §7.2 narrative A 数据，给硬件成本侧的 quantified evidence；为 §7.4 端到端 latency + 整网 MAC% 让位（narrative C 收束）。

**依赖**：
- 贡献：C1.5（critical path 已知问题，诚实标注）/ C3.5（多核综合资源）
- 文献：无直接对照（资源数字以 FLUX_CNN 内部为准）
- 图表占位：Tab.4（综合资源对照表，单核 + N=2/3/4 + 容量）
- 不确定项：[TBD: use_dsp + SDP 流水化重综合是否在投稿前完成]（contributions.md §8.2 #4, #5）

### 7.4 End-to-End Latency and MAC Efficiency

**核心命题**：11-case ResNet-18 风格 chain 整网 593K cycles / 86.6% MAC% / 5.95 ms @ 100 MHz target / 8.69 ms @ 68.4 MHz Fmax；端到端 throughput 约 168 fps @ target / 115 fps @ Fmax。

**关键内容**：用 Tab.5 端到端 latency 表（11 layer 逐层 cycles + 整网总和 + Wall_us 列 + 两 operating point 对照）；用一段文字论证 86.6% MAC% 含 IDMA/ODMA stall + 多层切换 + descriptor 拉取的实际开销；引 §5.4 trade-off 表 + §7.2 单层 PE 利用率作为支撑——"单层接近 100%, 整网 86.6%, gap 来源于系统级 stall"。[CHECK: Wall_us 端到端实测数字待重跑回归取]

**承上启下**：承 §7.3 资源 + Fmax，给整网 quantified end-to-end；为 §7.5 multi-core scaling 让位（narrative B 系统侧数据）。

**依赖**：
- 贡献：C1.2（streaming 端到端） / C2.5（多层端到端编译）/ C3.3（链式 CASES + Wall_us）
- 文献：无直接对照（FLUX_CNN 内部数据）
- 图表占位：Tab.5（端到端 latency 表，11 layer + 整网 + Wall_us + 两 operating point）
- 不确定项：[CHECK: Wall_us 实测]（contributions.md §8.1 #5）

### 7.5 Multi-Core Scaling

**核心命题**：N=2 multicore DDR-mode TB PASS @ 9057 cycles vs 单核 8808 cycles（+2.8% AXI 仲裁开销）；N=4 W slice 20/20 case 全 bit-exact PASS，N=4 wslice1 3833 cycles vs N=2 5569 cycles = 1.45× speedup；ResNet 11-layer multicore N=2/4 估算 1.7×/3.64× speedup（实测待跑）。

**关键内容**：用 Tab.6 multi-core scaling 表（单核 / N=2 / N=4 W slice 20 case 子集 + ResNet 11-layer 估算）；用一段文字论证 N=4 wslice1 1.45× 不是近线性的原因（baseline 归一化 + AXI 仲裁开销 + W slice halo 重叠开销）；与 Simba MICRO'19 chiplet 36× scaling 对比时点出"FPGA BRAM-bound vs ASIC chiplet 通信 bound 不同 scaling 域"。[CHECK: ResNet 11-layer multicore N=2/4 实测 cycles 待跑] / [CHECK: 1.45× 接近线性论证待 baseline 归一化]

**承上启下**：承 §7.4 单核整网，给多核 scaling quantified evidence（narrative B 系统侧主数据）；为 §7.6 head-to-head prior art 对位让位（narrative B 防御主战场）。

**依赖**：
- 贡献：C3.5（multi-core wrapper + 跨核 SRAM 直送）/ C3.7（W slice + computed redundancy halo）
- 文献：Simba MICRO'19（chiplet scaling 对照）/ Liu TNNLS'21 / Kang AoCStream（streaming 谱系切分维度对照）
- 图表占位：Tab.6（multi-core scaling 表）
- 不确定项：[CHECK: ResNet 11-layer multicore 实测 cycles]（contributions.md §8.1 #8）/ [CHECK: 1.45× 近线性论证]（contributions.md §8.1 #9）

### 7.6 Comparison with Prior Art

**核心命题**：narrative A/B 防御主战场——用 head-to-head 比较表（Tab.7 性能维度 + Tab.8 功能 / 设计取舍）把 FLUX_CNN 与 TPU / Eyeriss / Gemmini / Snowflake / Angel-Eye / Aydonat / Lu Winograd / Liu Full-Stack / VTA / fpgaConvNet 的同器件 / 同任务 / 同口径数字对位；尤其 86.6% 整网 MAC% vs Liu 97% MAC%（Arria 10 GX1150 大型器件）/ vs Snowflake 91% 平均计算效率（口径不完全等同）的精细文字论证。

**关键内容**：Tab.7 性能维度（dataset / 器件 / Fmax / GOPS / MAC% / latency）+ Tab.8 功能 / 设计取舍维度（dataflow / streaming granularity / multi-core 切分维度 / 编译栈 scope / SDP 支持）；一段文字论证 "86.6% vs Liu 97% = same paradigm at different scale"——中型 7-series vs 大型 Arria 10 GX1150 不否认 Liu 更高，承认是同思路在不同规模下的可比数字；Snowflake 口径需脚注说明 FLUX_CNN 含 IDMA/ODMA stall。[CHECK: 各 baseline 整网 MAC% / 同器件 Fmax / Angel-Eye SDP 支持 / 同器件资源占用 reviewer 阶段查原文]

**承上启下**：承 §7.5 多核 scaling，给 narrative B 防御主对位 + 5 维度功能取舍表；为 §7.7 已知局限 + future work 让位（诚实标注章末收束）。

**依赖**：
- 贡献：所有 C1-C3 在此被 quantified 校准；尤其 C1.2 / C2.1 / C2.2 / C3.5 / C3.7 是数据集中处
- 文献：TPU ISCA'17 / Eyeriss ISCA'16 / Gemmini DAC'21 / Snowflake ISCAS'17 / Angel-Eye TCAD'18 / Aydonat FPGA'17 / Lu Winograd FCCM'17 / Liu TNNLS'21 / VTA Micro'19 / fpgaConvNet TNNLS'19
- 图表占位：Tab.7（性能维度对位）/ Tab.8（功能 / 设计取舍维度对位，可与 §3.6 Tab.Positioning 整合）
- 不确定项：[CHECK: 各 baseline 数字 reviewer 阶段查原文]（contributions.md §8.1 #10, #11, #12, #13）/ [TBD: §7.6 是否在投稿前 use_dsp + SDP 流水化重综合更新数字]（contributions.md §8.2 #4, #5）

### 7.7 Discussion: Known Limitations and Future Work

**核心命题**：诚实标注三类已知局限——(a) Fmax 68.4 MHz 未达 100 MHz target（C1.5，修复路径已知：SDP 流水线化 + use_dsp 属性 → 100+ MHz 估省 17K LUT）；(b) Cout<16 layers PE 列空转（design rationale：硬件保持简洁）；(c) Pooling / Depthwise Conv / 稀疏未做。

**关键内容**：用一段话给 (a) 修复路径具体性（contributions.md C1.5 + STATUS §4 ROI 排序）+ (b) Cout<16 design rationale（"affects negligible MAC%-of-total in ResNet-18-style networks"）+ (c) future work 列表（cross-layer streaming fusion 对照 Tangram / ResNet 11-layer multicore chain 适配 1-2 天 / 片上 push 链 P2 完成态 2-3 天 / cout slice + stage barrier）；不试图掩饰，强调"目标会议受众接受 work-in-progress 程度"。

**承上启下**：作为 §7 Evaluation 章收束节给 limitations 诚实标注；自然过渡到 §8 Conclusion——本节给"已知缺口 + 修复路径"，下一章 high-level 总结。

**依赖**：
- 贡献：C1.5（critical path 已知问题）；其他 C1-C3 future work 维度引用
- 文献：Tangram ASPLOS'19（cross-layer streaming fusion 未来工作对照）

---

## 第 8 章 Conclusion

> 单章不分小节。内容仅做收束 + 简短 future work 一句话。

### 8.1 Conclusion

**核心命题**：FLUX_CNN 论证了在固定 16×16 INT8 阵列 FPGA 加速器上，**编译器侧 Ky-fold + S2D 变换可以替代 hardware-reconfigurable 路线达成接近 100% PE 利用率**（narrative A 主轴），并通过 row-ring streaming + N=4 W slice multi-core 验证了 system-level 完整性（narrative B 系统侧）；46+20 case 全 bit-exact PASS + XC7K325T N=4 综合通过 + 11-layer ResNet-18 风格 chain 86.6% 整网 MAC% 作为完整证据。

**关键内容**：用一段话回顾 narrative A 主轴（compiler-side PE utilization on fixed array，C2.1/C2.2/C2.3）+ narrative B 系统侧（C1.2 row-streaming + C3.5/C3.7 multi-core）；用一段话指出 future work（按 STATUS §4 ROI 排序：use_dsp 属性 + SDP 流水线化先做 / Pooling DW 稀疏 / cross-layer streaming fusion 对照 Tangram / ResNet multi-core chain 适配 / 片上 push 链 P2 完成态）；不在 conclusion 翻新 claim，不夸大 contribution 强度（保持与 §1.4 一致）。[TBD: 是否点名 future work 具体里程碑，取决于是否要给 roadmap timestamp]

**承上启下**：承 §7.7 已知局限 + 修复路径，全文收束；不引出后续章节（论文最后一节）。

**依赖**：
- 贡献：高 level 总结 5 条左右（C1.2 / C2.1 / C2.2 / C3.5 / C3.7 主线）
- 文献：仅引 Tangram ASPLOS'19（cross-layer streaming fusion 未来工作对照）；可能再引 Eyeriss ISCA'16 或 NVDLA 做 motivation 回环
- 不确定项：[TBD: 是否点名 future work 具体里程碑]（outline.md line 241）

---

## 跨小节一致性记录

> 写完 §1-§8 共 36 节后做跨节一致性检查。本节列出已发现的潜在重复 / 未引用依赖 / 图表唯一性管理。

### 重复风险（已识别）

- **PE 利用率 motivation 数字**：§1.1 motivation + §5.1 Ky-fold + §7.2 PE-util-by-layer 三处都引 Layer 1 12.5% / Layer 3/4 25% / Layer 5a/c 50%；分工：§1.1 只做 motivation 一句话点出，§5.1 详述 Ky-fold 数学 + 启用后接近 100%，§7.2 给三列对比表（基线 / Ky-fold / S2D）。
- **streaming row-ring 描述**：§1.3 our approach + §3.4 Related Work narrative B + §4.3 hardware 落地 三处提及；分工：§1.3 一句话定位，§3.4 prior art 精细差异化（Alwani / Kang / Liu），§4.3 ring 数学 + Fig.2 timing。
- **vs Liu Full-Stack 论证**：§3.4 + §7.6 两处；分工：§3.4 narrative B 主对照集精细差异化，§7.6 head-to-head 数字对位（86.6% vs 97% MAC% 文字论证）。
- **C1.4 SDP residual fusion**：§3.5 + §6.3 两处；分工：§3.5 仅做 framing 一段话点到 quantization 谱系，§6.3 详述 R.1/R.2 重构 + Shortcut Bank。[TBD: §3.5 是否单独成节 vs 并入 §6.3 视篇幅压力定]
- **C2.4 7 层硬件循环嵌套**：§4.5 + §5.1/5.2（implicit）两处；分工：§4.5 给 7 层结构 + 切片机制作为 implementation context，§5.1/5.2 把 fold 变换映射进 loop nest。

### 未引用依赖检查

**贡献清单覆盖**（17 条 C1.1-C3.7，校验每条至少一节明确引用）：
- C1.1 去中心化 valid-ready：§4.2 主推
- C1.2 row-ring streaming：§1.1, §1.3, §3.4, §4.3, §7.4 多节支撑
- C1.3 per-col PARF：§4.4 主推
- C1.4 SDP fusion：§3.5, §6.3 主推
- C1.5 critical path 已知：§7.3, §7.7 诚实标注
- C2.1 Ky-fold：§1.1, §5.1, §7.2 主推
- C2.2 S2D：§5.2, §7.2 主推
- C2.3 自动决策：§5.3 主推
- C2.4 7 层 loop nest + cin/cout slice：§4.5 主推
- C2.5 PyTorch 端到端：§6.1 主推, §7.4 间接
- C3.1 vendor IP 集成：§6.2 主推
- C3.2 CFG_WRITE descriptor：§6.2 主推
- C3.3 链式 CASES + Wall_us：§6.1 间接, §7.1, §7.4 主推
- C3.4 done sticky + 双端 cfg：§6.2 主推
- C3.5 multi-core wrapper：§6.4, §7.5 主推
- C3.6 params single source：§6.5 主推
- C3.7 W slice + halo：§3.6, §6.4, §7.5 主推

**结论**：17 条贡献全部至少一节明确引用，无遗漏。

**文献覆盖关键条**（literature.md 32 篇全覆盖在 §3 已确认；§4-§8 重要引用复核）：
- Buffets ASPLOS'19：§4.2
- Snowflake ISCAS'17：§3.4, §4.1, §4.3, §7.6
- NVDLA：§3.5, §4.1, §6.2, §6.3
- Alwani MICRO'16 / Kang AoCStream / Liu TNNLS'21：§3.4 主, §4.3, §6.4, §7.5, §7.6
- Simba MICRO'19：§3.6, §6.4, §7.5
- Tangram ASPLOS'19：§3.6, §7.7, §8.1
- Sub-pixel CVPR'16：§5.2, §7.2
- MAERI / Eyeriss-v2：§3.2, §3.6, §5.1, §5.2, §5.4, §7.2
- Interstellar ASPLOS'20 / Ma FPGA'17：§3.3, §4.5, §5.1, §5.2
- Xilinx PG022：§6.2

### 图表占位汇总

- **Fig.1**：§4.1 顶层框图（core + DMA 子系统）首次引入
- **Fig.2**：§4.3 row-ring 数据路径 + 三阶段并发 timing diagram 首次引入
- **Tab.Positioning**：§3.6 5 维度 placement 表首次引入；§7.6 Tab.8 可考虑整合
- **Tab.2**：§5.4 compiler-only vs hardware-reconfigurable 4 维度对位表首次引入
- **Tab.3**：§7.2 PE-util-by-layer 三列对照（基线 / Ky-fold / S2D）首次引入
- **Tab.4**：§7.3 综合资源对照表（单核 + N=2/3/4 + 容量）首次引入
- **Tab.5**：§7.4 端到端 latency 表（11 layer + 整网 + Wall_us + 两 operating point）首次引入
- **Tab.6**：§7.5 multi-core scaling 表首次引入
- **Tab.7**：§7.6 性能维度 prior art head-to-head 对位首次引入
- **Tab.8**：§7.6 功能 / 设计取舍维度对位（可与 Tab.Positioning 整合）

### 承接连贯性

按章逐对核查 N.M 的"承上启下"与 N.(M+1) 的"承上"对位：
- §4.1 → §4.2：整体形态 → 模块级握手机制 ✓
- §4.2 → §4.3：握手语义 → ring forward-pressure ✓
- §4.3 → §4.4：ring 内累加场景 → 累加单元拆分 ✓
- §4.4 → §4.5：累加器单元 → loop nest 切片 ✓
- §4.5 → §5.1：硬件 7 层 loop nest → 编译器把 Ky-fold 映射进 ✓
- §5.1 → §5.2：Cin<16 触发 fold → stride≥2 触发 fold ✓
- §5.2 → §5.3：两变换 → 联合触发逻辑 ✓
- §5.3 → §5.4：组合自动化 → 与 hardware-reconfigurable 对位 ✓
- §5.4 → §6.1：narrative A 收束 → 系统工具链入口 ✓
- §6.1 → §6.2：编译产物 → DMA 子系统硬件落地 ✓
- §6.2 → §6.3：DMA 子系统 → 核内 SDP ✓
- §6.3 → §6.4：核内细节 → 系统多核 ✓
- §6.4 → §6.5：multi-core → 工程实践收束 ✓
- §6.5 → §7.1：系统能跑 → 跑得多好 ✓
- §7.1 → §7.2：实验 setup → narrative A 数据 ✓
- §7.2 → §7.3：narrative A → 资源 + Fmax (narrative C) ✓
- §7.3 → §7.4：硬件成本 → 端到端 latency ✓
- §7.4 → §7.5：单核整网 → 多核 scaling ✓
- §7.5 → §7.6：narrative B 数据 → prior art 对位 ✓
- §7.6 → §7.7：对位 → 已知局限 ✓
- §7.7 → §8.1：缺口 + 修复路径 → 全文收束 ✓

---

## 待决清单（[TBD] / [CHECK]）

### [TBD]（写作选择待用户决定）

1. **§1.2**：三条 prior art 路线点到即止 vs 一句话各自一个具体数字（outline 段落骨架决定）
2. **§3.5**：是否单独成节 vs 并入 §6.3 SDP 段落（视篇幅压力）
3. **§4.4**：是否与 §4.2 valid-ready 合并（取决于篇幅）
4. **§4.1**：是否在框图旁配 timing diagram（Phase 4 决定）
5. **§6.4**：是否升格为独立章以提升 narrative B 比重（当前选节内）
6. **§6.5**：params single source 是否值得占节位（篇幅紧时合并到 §6.2 末尾）
7. **§7.3 / §7.6**：是否在投稿前 mac_pe `(* use_dsp = "yes" *)` + SDP 流水线化重综合更新数字
8. **§8.1**：是否点名 future work 具体里程碑（roadmap timestamp）

### [CHECK]（涉及实测 / 实现状态待 project-analyst 跑或 reviewer 阶段补查）

1. **§1.1 / §5.1 / §7.2**：Layer 1/3/4 启用 Ky-fold 后实测 PE 利用率百分比（contributions.md §8.1 #1, #2，跑 `run_regression.py --fold` 取 cycles 反算）
2. **§4.4**：per-col PARF 的 BRAM 数量贡献单独估算（contributions.md §C1.3，查综合报告 cell utilization）
3. **§5.2**：启用 S2D 后 DDR 带宽节省比例（对照 run 统计 IDMA 读次数）
4. **§5.2 / §5.4**：S2D 在加速器领域的引用谱系（reviewer 阶段补查 ASPLOS/HPCA architecture-side S2D 更早出处）
5. **§6.1 / §7.4**：5.95 ms target vs 68.4 MHz Fmax 实际 fps 两口径在论文中如何统一（contributions.md §C2.5）
6. **§6.2**：Xilinx PG022 vendor doc 引用方式（contributions.md §C3.1）
7. **§6.4 / §7.5**：N=4 wslice1 1.45× 接近线性论证 baseline 归一化（contributions.md §8.1 #9）
8. **§6.4 / §7.5**：ResNet 11-layer multicore N=2/4 实测 cycles vs 估算 1.7×/3.64×（contributions.md §8.1 #8，STATUS line 254 估 1-2 天）
9. **§7.2 / §7.4**：单层 PE 利用率 vs 整网 MAC% 口径论证文字（contributions.md §8.1 #6）
10. **§7.4**：Wall_us 端到端实测数字重跑回归（contributions.md §8.1 #5）
11. **§7.6**：各 baseline 整网 MAC% / 同器件 Fmax / Angel-Eye SDP 支持 / 同器件资源占用（contributions.md §8.1 #10, #11, #12, #13，reviewer 阶段查原文）

---


