# 段落骨架 (Phase 4)

> Phase 4 段落级骨架。每段一句话主题句（topic sentence），明确该段在小节中的位置 / 论证作用 / 与相邻段的承接关系。
> **每段一句话**——不展开为完整段落（那是 Phase 5 的工作）。

---

## 元信息

- **完成时间**：2026-04-30
- **章节总数**：8 章 / 35 节（§1×5 + §2×4 + §3×6 + §4×5 + §5×4 + §6×5 + §7×7 + §8×1）
- **总段数**：填充后统计
- **数据快照**：commit `b158cab`，STATUS.md 时间戳 2026-04-30
- **依赖输入**：`section-summary.md` / `outline.md` / `contributions.md` / `literature.md` / `lessons-learned.md`
- **narrative**：A 主轴（compiler-side PE utilization）+ B 系统侧支撑（row-streaming + multi-core）

### 类型标签约定（每段一个）

- **setup**：背景 / 定义 / 术语铺垫
- **claim**：提论断
- **evidence**：数据 / 例子 / 推导支持上一段的 claim
- **comparison**：与文献 / baseline 对位
- **transition**：承上启下、桥段

### 标记约定

- `[CHECK: 描述]` — 涉及实测 / 实现状态 / 引用谱系不确定
- `[TBD: 描述]` — 涉及方向 / 写作选择待用户决定

---

## 第 1 章 Introduction

### §1.1 Motivation: PE Underutilization in Fixed Arrays + Streaming Constraints on Edge FPGAs

- 段 1 (setup): 固定空间阵列 CNN 加速器（如 16×16 INT8）已成为端侧 FPGA 推理的主流形态，但其 PE 利用率与片上 SRAM 容量两条约束共同决定了实际可用算力。
- 段 2 (claim): 在 ResNet-18 风格网络的浅层，固定 16×16 阵列的 PE 利用率仅 12.5%-50%，是 narrative A 的核心 motivation。
- 段 3 (evidence): model_analysis 5 层 quantitative breakdown 显示 Layer 1 (Cin=4) 12.5%、Layer 3/4 (Cin=8) 25%、Layer 5a/c (Cin=8) 50%——浅层 Cin<16 是 chronically underutilized。 [依赖: model_analysis.md §2 / [CHECK: Layer 1/3/4/5a/c 启用 Ky-fold 后实测 PE 利用率]]
- 段 4 (claim): 端侧 FPGA SRAM 受限（XC7K325T 445 BRAM ≈ 1.6 MB）与 VGA 480×640 单图 4.9 MB 的容量矛盾，是 narrative B streaming 必要性的硬约束。
- 段 5 (transition): 这两条约束（PE 利用率 + streaming 必要性）共同构成 FLUX_CNN 设计起点，下一节回顾现有解法为何不够。

### §1.2 Limitations of Existing Approaches

- 段 1 (setup): 应对固定阵列 PE 利用率不足，文献中存在三条主流路线，分别在硬件、跨层数据流与算子展开三个层次给出不同答案。
- 段 2 (claim): hardware-reconfigurable 阵列（MAERI ART / Eyeriss-v2 NoC）以增加 RTL 复杂度换 PE 利用率，但 reconfigurable 互联本身带来综合面积与时序压力。
- 段 3 (claim): layer-pipelined fusion（Alwani / Kang AoCStream）假设跨层 feature map 同时驻留片上，端侧 BRAM 容量难以承载多层 FM 同驻。
- 段 4 (claim): im2col 全展开（cuDNN 风格）把 K×K·Cin 全展为列向量，端侧 BRAM 受不了内存炸裂。
- 段 5 (transition): FLUX_CNN 在这三条路线之外选第四条——硬件保持简洁固定、由编译器侧重映射填满 PE，下一节给出整体方案。

### §1.3 Our Approach

- 段 1 (claim): FLUX_CNN 的核心设计选择是"硬件保持简洁固定 16×16 阵列 + 编译器侧 Ky-fold/S2D 填满 PE + row-ring streaming 单次 start 跑完任意 H×W"。
- 段 2 (setup): narrative A 主轴落在编译器侧 PE utilization——Ky-fold 把 Ky 折到 cin、S2D 把 stride 相位折到 cin，零 RTL 改动。
- 段 3 (setup): narrative B 系统侧由 row-ring streaming 与 multi-core W slice 支撑——VGA 480×640 单图仅需 ~10 KB ring，N=4 W slice 在 XC7K325T 综合通过。
- 段 4 (transition): narrative C 工程化元素（去中心化 valid-ready 流水）作为核内组织形式支撑 A/B 两轴；下一节列出可量化贡献。 [依赖: Fig.1 顶层框图，§4.1 详述]

### §1.4 Contributions

- 段 1 (setup): 本文贡献围绕 narrative A 主轴 + narrative B 系统侧组织，每条带 quantified evidence。
- 段 2 (claim): (1) Ky-fold 编译器变换在 Cin<16 浅层把 PE 利用率从 12.5% 提升至接近 100%，零 RTL 改动。 [依赖: C2.1 / [CHECK: 实测 PE 利用率百分比]]
- 段 3 (claim): (2) Space-to-Depth 编译器变换在 stride≥2 把 stride² 相位折到 cin，等量重排不复制——DDR 友好。 [依赖: C2.2]
- 段 4 (claim): (3) Streaming row-ring 数据路径 + 双向 row-credit 反压让 VGA 480×640 单图仅需 ~10 KB 片上 ring，22+24=46 case 全 bit-exact PASS。 [依赖: C1.2]
- 段 5 (claim): (4) Multi-core W slice with computed redundancy halo 在 XC7K325T 上 N=4 综合通过 + 20/20 W slice case bit-exact。 [依赖: C3.5 / C3.7]
- 段 6 (claim): (5) PyTorch nn.Sequential 端到端编译让 11 层 ResNet-18 风格 chain 整网 86.6% MAC%，含已识别但未修复的 critical path（with documented optimization roadmap）。 [依赖: C2.5 / C1.5]
- 段 7 (transition): 上述 5 条贡献在 §4-§7 详述；下一节给章节阅读路径。 [TBD: 5 条 vs 6 条由用户最终裁决]

### §1.5 Paper Organization

- 段 1 (setup): 全文 8 章按 Background → Related Work → Architecture → Compiler → System Integration → Evaluation → Conclusion 组织。
- 段 2 (transition): §4 / §5 / §6 任意单读都能闭合，方便不同 reviewer 类型快速定位关心的部分；§7 含 narrative A 与 B 的主数据节。



## 第 2 章 Background and Motivation

### §2.1 Spatial Array Dataflows: WS / OS / RS Taxonomy

- 段 1 (setup): 空间阵列加速器按数据驻留维度分为三类——weight-stationary (WS) / output-stationary (OS) / row-stationary (RS)，是后续讨论的术语前提。
- 段 2 (evidence): TPU v1 是 WS 代表（256×256 systolic）、Eyeriss 是 RS 代表（14×12 PE row reuse）、ShiDianNao 是 OS 代表（cout 列内累加）。 [依赖: TPU ISCA'17 / Eyeriss ISCA'16 / ShiDianNao ISCA'15]
- 段 3 (claim): FLUX_CNN 的 16×16 阵列采用"列广播激活 + 列独立 cout 输出"的 OS-like 数据流，配合 row-ring streaming 时输出在列内累加最自然。
- 段 4 (transition): NVDLA 的 CDMA→CMAC→CACC→SDP 段命名脉络为 FLUX_CNN 后续模块命名提供谱系参照；下一节量化 PE 利用率病理。 [依赖: NVDLA]

### §2.2 PE Utilization Pathology in Shallow Layers

- 段 1 (claim): ResNet-18 风格 5 层在 16×16 阵列上 chronically PE-underutilized，是 narrative A 的核心 motivation 数字。
- 段 2 (evidence): Layer 1 Cin=4 → 12.5%、Layer 3-4 Cin=8 → 25%、Layer 5a-c Cin=8 → 50%、Layer 5b 起 Cin=16 → 100%、FC_xy Cout=2 → 12.5%。 [依赖: Tab.PE-Util-Breakdown / model_analysis.md §2 / [CHECK: 启用 Ky-fold 后实测百分比]]
- 段 3 (setup): "单层 PE 利用率"（描述 mac_array 占空）vs"整网 86.6% MAC%"（含 IDMA/ODMA stall + 多层切换）是两个不同口径，本文 §7.4 末尾文字论证关系。
- 段 4 (claim): Cout<16 的层（如 FC_xy Cout=2）当前不优化——硬件保持简洁，且 MAC%-of-total 在 ResNet-18-style 中 negligible。
- 段 5 (transition): 浅层 PE 利用率约束已建立，下一节转向同源于端侧硬约束的第二个挑战——streaming 必要性。 [依赖: C2.1 / C2.2 / C2.4]

### §2.3 Streaming vs Tiled: Memory Footprint of Large-Image Inference

- 段 1 (setup): 端侧 FPGA SRAM 容量与现代视觉任务输入分辨率之间存在硬约束——XC7K325T 445 BRAM ≈ 1.6 MB vs VGA 480×640 单图 4.9 MB。
- 段 2 (claim): row-ring streaming 是必要选择而非可选优化——只缓 strip_rows×W ≈ 8×640 = 10 KB 即可承载整图。 [依赖: C1.2]
- 段 3 (comparison): 这与 Alwani layer-fusion / Kang AoCStream 全片上路线形成对照——后者假设跨层 FM 同驻，端侧 BRAM 受不了；细节留 §3.4。 [依赖: Alwani MICRO'16 / Kang Sensors'23]
- 段 4 (transition): 与 §2.2 PE 利用率约束正交但同源于端侧硬约束，下一节把两者收为 design goals。 [TBD: 是否配 SRAM-vs-Image-Size 曲线图]

### §2.4 Design Goals

- 段 1 (setup): 把 §2.2 + §2.3 两条 motivation 约束转化为本工作的设计目标。
- 段 2 (claim): 三条 design goals：(a) 硬件简洁固定不引入 reconfigurable PE 互联，(b) 编译器侧填满 PE 应对 Cin<16，(c) row-ring streaming 单次 start 跑完任意 H×W。 [依赖: C1.1 / C2.1+C2.2 / C1.2]
- 段 3 (transition): 这三条 goals 在文献中分别对应 fixed-array / compiler-only PE-utilization / row-level streaming 三个谱系定位，下一章逐条对位 prior art。



## 第 3 章 Related Work

### §3.1 Spatial Array Accelerators

- 段 1 (setup): 空间阵列加速器在尺度（PE 数）、数据流（WS/OS/RS）、平台（ASIC/FPGA）三个维度上形成谱系，是定位 FLUX_CNN 的硬件坐标。
- 段 2 (evidence): TPU v1（256×256 ASIC 数据中心）/ Eyeriss（14×12 RS ASIC）/ Gemmini（16×16 同尺度 ASIC）/ NVDLA（端侧 IP）/ Simba（多 chiplet）覆盖该谱系主要节点。 [依赖: 5 篇代表作 / Tab.Spatial-Array-Comparison]
- 段 3 (claim): FLUX_CNN 与 Gemmini 同尺度（16×16 INT8）但去中心化握手，是 narrative C 的硬件谱系锚点。 [依赖: C1.1]
- 段 4 (transition): NVDLA CDMA→CMAC→CACC→SDP 命名传承到 FLUX_CNN line_buffer→mac_array→parf_accum→sdp，下一节进入"应对 PE 利用率"的两大路线选择。

### §3.2 Hardware-Reconfigurable Approaches to PE Utilization

- 段 1 (claim): hardware-reconfigurable 路线用增加 RTL 复杂度换 PE 利用率，是 narrative A 的反向 alternative。
- 段 2 (evidence): MAERI 用 ART 互联网络重构 dataflow、Eyeriss-v2 用 NoC reconfiguration、Tangram 用 chiplet array reconfig 三条工作分别在不同粒度上引入硬件 reconfigurable PE 互联。 [依赖: MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / Tangram ASPLOS'19]
- 段 3 (comparison): FLUX_CNN 选反方向——硬件保持简洁固定 16×16 阵列，编译器侧重映射；硬件复杂度 vs 编译复杂度 trade-off 不互斥。 [依赖: C2.1 / C2.2]
- 段 4 (transition): 下一节进入 narrative A 的同语言谱系——编译器 / loop-nest co-design。

### §3.3 Compiler / Loop-Nest Co-Design

- 段 1 (setup): 编译器 / loop-nest co-design 是 narrative A 的同语言谱系——用 mapping search、loop tiling、形式化建模处理 spatial array 映射问题。
- 段 2 (evidence): Timeloop（mapping search）/ Interstellar（7-loop 形式化）/ TVM+VTA（端到端编译栈）/ Ma FPGA'17（4-6 维 loop systematic）/ cuDNN（im2col baseline）覆盖该语言体系主要工作。 [依赖: Timeloop ISPASS'19 / Interstellar ASPLOS'20 / TVM OSDI'18 / VTA Micro'19 / Ma FPGA'17 / cuDNN]
- 段 3 (claim): 该语言体系内"FPGA 上把 Ky 折到 cin 作为 PE 利用率优化"和"compile-pass S2D"两点未被命中，是 narrative A 的防御点。 [依赖: C2.1 / C2.2]
- 段 4 (transition): S2D 在加速器领域的引用谱系待 reviewer 阶段补查；下一节进入 narrative B 主对照集。 [依赖: [CHECK: S2D 引用谱系] / contributions.md §8.1 #15]

### §3.4 Streaming / Line-Buffer FPGA Accelerators

- 段 1 (setup): streaming 谱系是 narrative B 的主对照集，共性是 line-buffer + 行级触发，关键差异在 streaming 粒度。
- 段 2 (evidence): fpgaConvNet / DnnWeaver / Snowflake / Angel-Eye / Aydonat / Lu Winograd 共同奠定该谱系的"line-buffer + 行级触发"硬件模式。 [依赖: 6 篇 streaming 文献]
- 段 3 (comparison): Alwani Fused-layer（MICRO'16）做 layer-level granularity——FLUX_CNN row-level 只缓几行而非几层 FM，且单核 layer-serial 共用硬件而非跨层同时驻留。 [依赖: Alwani MICRO'16]
- 段 4 (comparison): Kang AoCStream（Sensors'23）强调全片上——FLUX_CNN 承认外存必然存在并优化 DDR 流量；layer-serial vs Kang layer-pipelined 多 block。 [依赖: Kang Sensors'23]
- 段 5 (comparison): Liu Full-Stack（TNNLS'21）支持 streaming + residual fusion——FLUX_CNN 单核 layer-serial 不做 layer fusion，规模差是同思路在不同硬件规模下的对照（中型 7-series 86.6% vs Arria 10 GX1150 97% MAC%）。 [依赖: Liu TNNLS'21]
- 段 6 (transition): Snowflake / Angel-Eye 作为 §7.6 同器件对照点；下一节简短 framing quantization 谱系。 [依赖: C1.2 / C3.5 / C3.7]

### §3.5 Quantization and Fusion

- 段 1 (setup): INT8 量化 + residual fusion + 工业 SDP 段是 system 完整性的算法侧背景。
- 段 2 (evidence): Jacob CVPR'18（INT8 量化）/ He CVPR'16（ResNet residual 算法侧必要性）/ NVDLA SDP（工业段命名）/ EIE+ESE（稀疏量化）覆盖该谱系。 [依赖: 5 篇文献]
- 段 3 (claim): FLUX_CNN 的 SDP fusion 偏增量贡献——R.1 bias 重定位 + R.2 shortcut_mult/shift 可编程量化因子——仅作为 system 完整性论据出现。 [依赖: C1.4 / [TBD: §3.5 是否单独成节 vs 并入 §6.3]]

### §3.6 Positioning of FLUX_CNN

- 段 1 (claim): 用 5 维度 placement 表把 FLUX_CNN 在文献中的位置钉死，作为 Related Work 收束节防御性总结。
- 段 2 (evidence): 5 维度——(1) dataflow=OS-like + per-col PARF / (2) PE-utilization=compiler-only Ky-fold/S2D / (3) streaming granularity=row-level / (4) multi-core 切分维度=W slice + halo / (5) 编译栈 scope=hand-rolled mini-compiler。 [依赖: Tab.Positioning（5 维度 × 9 工作对照）/ 所有 C1-C3 在此被校准]
- 段 3 (comparison): 五个轴上的对位反例分别是 MAERI/Eyeriss-v2（hardware-reconfig）、Alwani/Kang/Liu（layer-level streaming）、Simba（channel slice 多核）、TVM/VTA（通用编译栈）。
- 段 4 (transition): 这张表为 §4 详细架构论证铺好坐标系，下章进入 Architecture。



## 第 4 章 Architecture

### §4.1 Top-Level Architecture

- 段 1 (claim): FLUX_CNN 顶层由 5 模块 core pipeline + DMA 子系统两层构成；外部仅 1 个 AXI4 Master + 1 个 AXI-Lite Slave。
- 段 2 (evidence): core pipeline 是 line_buffer → wgt_buffer → mac_array → parf_accum → ofb_writer 五个模块；DMA 子系统是 idma_ctrl/wdma_ctrl/odma_ctrl + mm2s_arb + axi_dm IP + axi_m_mux + axi_lite_csr。 [依赖: Fig.1 顶层框图 / C1.1 / C3.1]
- 段 3 (comparison): line_buffer→mac_array→parf_accum→sdp 命名延续 NVDLA CDMA→CMAC→CACC→SDP 谱系，便于读者跨工程理解。 [依赖: NVDLA / Snowflake ISCAS'17]
- 段 4 (transition): IDMA / 计算 / ODMA 三阶段并发的执行模型留 §4.3 详述；下一节先进入握手机制。 [依赖: C1.2 / [TBD: 是否配 timing diagram]]

### §4.2 Decentralized Valid-Ready Pipeline

- 段 1 (claim): 核流水内 5 模块各自维护独立 counter、模块间用 valid-ready 双向握手串联，无中心 FSM / scheduler。 [依赖: C1.1]
- 段 2 (evidence): 每模块边界采用"counter + ready 反压"模式，elastic join 保证 stall 下 in-flight 数据不丢；FILL/DRAIN overlap 让 parf_accum tile N 的 drain 隐藏在 tile N+1 fill 的 first_round 里。
- 段 3 (evidence): sequencer 仅做 cross-block 启动同步而非中心调度；22-case 链式回归全 PASS 是该 narrative 的工程证据。
- 段 4 (comparison): 与 Gemmini 中心 RoCC scheduler / NVDLA 显式 controller / VTA 4 段 fetch-load-compute-store 形成对照；Buffets ASPLOS'19 提供"buffer + counter + handshake"的形式化语言。 [依赖: Gemmini DAC'21 / NVDLA / VTA Micro'19 / Buffets ASPLOS'19]
- 段 5 (transition): 握手语义是下一节 row-ring forward-pressure 的语义基础。

### §4.3 Streaming Row-Ring Datapath with Bidirectional Credit Backpressure

- 段 1 (claim): IFB / OFB 作 row-level ring buffer 按 strip_rows 取模；line_buffer 仅在 rows_available ≥ yout·stride + Ky 时发射（forward-pressure），ODMA 排空行送 ofb_writer credit；sequencer 一次 dispatch 同启 IDMA / 计算 / ODMA 三阶段并发。 [依赖: C1.2]
- 段 2 (evidence): ring 数学上 wptr/rptr 模运算永不 reset；VGA 480×640 / 4.9 MB 单图只用 ~10 KB ring（strip_rows=8 × W=640）。 [依赖: Fig.2 row-ring + 三阶段并发 timing diagram]
- 段 3 (evidence): 22+24=46 case 全 bit-exact PASS（含 H/W 1×1 FC、15×17 奇数、120×68 大图）作为鲁棒性证据。
- 段 4 (claim): "统一 streaming row-ring"模式覆盖原 batch（环容量 ≥ 整图的退化情形）；narrative B 精细差异化（vs Alwani / Kang / Liu）已在 §3.4 详述，此处只点 row-level granularity 的硬件落地。 [依赖: Alwani MICRO'16 / Kang Sensors'23 / Liu TNNLS'21]
- 段 5 (transition): ring 内累加单元的具体实现留下一节。

### §4.4 Per-Column PARF Accumulator

- 段 1 (claim): parf_accum 内部不是单一 PSUM SRAM，而是 parf_col × 16 列独立 SRAM 共享 wr_addr/we/rd_addr 外壳。 [依赖: C1.3]
- 段 2 (evidence): 每列独立寻址但共享外壳逻辑，让每列可用 single-port BRAM，匹配"列广播激活 + 列独立 cout 输出"数据流。
- 段 3 (comparison): 与 NVDLA / Gemmini 单一 PSUM 大 SRAM（需多端口 / port arbitration）形成对照；该选择释放端口压力。 [依赖: NVDLA / Gemmini]
- 段 4 (transition): 这是单元级实现细节而非顶层 claim；下一节给整层 loop nest 作为编译器 / 硬件边界。 [依赖: [CHECK: per-col PARF BRAM 数量贡献] / [TBD: §4.4 是否与 §4.2 合并]]

### §4.5 Loop-Nest Realization

- 段 1 (claim): 物理 16×16 阵列处理任意 Cin/Cout 靠两层切片——cin_slices = ⌈Cin/16⌉ 时间维 PARF 累加 + cout_slices = ⌈Cout/16⌉ 时间维 OFB 行内段 NHWC 拼接。 [依赖: C2.4]
- 段 2 (evidence): 整层硬件循环是 for yout > for cs > for tile > for cins > for ky > for kx > for iss_pos 七层嵌套；超容量自动切 strip（H 方向）+ tile（W 方向）。
- 段 3 (comparison): Interstellar ASPLOS'20 的 7 nested loops 形式化与本文嵌套天然同构；Ma FPGA'17 的 4-6 维 loop systematic study 是早期同语言对照。 [依赖: Interstellar ASPLOS'20 / Ma FPGA'17]
- 段 4 (transition): 本节告诉读者"硬件按普通 conv 跑这 7 层循环"，下章告诉读者"编译器侧如何把 Ky-fold/S2D 映射进这 7 层"。



## 第 5 章 Compiler Optimizations for PE Utilization

> **章首段必述（口径声明）**：本章用"单层 PE 利用率"指标，与 §7.4 整网 86.6% MAC% 不是同一口径——前者描述单层 mac_array 占空，后者含 IDMA/ODMA stall + 多层切换；两者关系在 §7.4 末尾文字论证。

### §5.1 Ky-Folding

- 段 1 (claim): 当 Cin<16 让 PE 行不能填满时，编译器侧把 Ky 维按 groups_y = HW_PE/Cin 折到 cin 维，硬件按普通 conv 跑（K_new=kyper, Cin=cin_fake），完全无感。 [依赖: C2.1]
- 段 2 (evidence): 数学上定义 cin_fake = groups_y · Cin / kyper = ⌈K/groups_y⌉，输入做 y-方向偏移复制 + 权重 reshape；compute_fold_params / fold_input / fold_weights 实现 in toolchain。 [依赖: docs/pe-fold.md §1]
- 段 3 (evidence): Layer 1（K=7, Cin=4 → 12.5%）启用 Ky-fold 后接近 100% PE 利用率；代价是 IFB 占用 × groups_y。 [依赖: [CHECK: Layer 1/3/4 实测 PE 利用率百分比]]
- 段 4 (comparison): 与 cuDNN im2col 同源但更轻量——只折 Ky 不展开 Kx，保留 Cin 通道并行，避免 im2col 全展开内存炸裂。 [依赖: cuDNN]
- 段 5 (comparison): 与 MAERI ART / Eyeriss-v2 NoC reconfig 形成"compiler-only vs hardware-reconfigurable"对比，详细对位留 §5.4。 [依赖: MAERI / Eyeriss-v2]

### §5.2 Space-to-Depth Folding

- 段 1 (claim): stride≥2 的卷积按 (kx%stride, ky%stride) 把 stride² 个相位折到 cin 维，等价为 stride=1 / K_new=⌈K/stride⌉ / Cin_new=stride²·Cin 的卷积。 [依赖: C2.2]
- 段 2 (evidence): 编译器侧只重排不复制——DDR 友好；启用后 stride=1 + ARF reuse_en=1 滑动窗口复用让 IFB 读次数大幅下降；K 被 stride 整除时 pad_waste=0。 [依赖: docs/pe-fold.md §2]
- 段 3 (comparison): 算法侧出处来自 Shi et al. CVPR'16 Sub-Pixel（train-time sub-pixel layer），FLUX_CNN 的贡献是把它作为 compile-pass 应用到任意预训练 stride≥2 conv——不需要重训。 [依赖: Shi CVPR'16]
- 段 4 (claim): 相比 Ky-fold 的 groups_y 倍 IFB inflation，S2D 等量重排不复制——多核场景 DDR 带宽节省更显著。 [依赖: [CHECK: DDR 带宽节省比例待对照 run]]
- 段 5 (transition): S2D 在加速器领域引用谱系待 reviewer 阶段补查；下一节给两变换的联合触发逻辑。 [依赖: [CHECK: S2D 加速器领域引用谱系]]

### §5.3 Joint Trigger Logic and Decision Automation

- 段 1 (claim): 编译器根据 layer 几何自动选 fold——Ky-fold 触发于 K>1 AND Cin<16；S2D 触发于 stride≥2 AND K≥stride；S2D 启用后 Cin 变为 stride²·Cin，重新判定 Ky-fold（多数情况 S2D 后 Cin'≥16 不再需 Ky-fold）。 [依赖: C2.3]
- 段 2 (evidence): run_regression.py 22 case × 三种模式（无 fold / --fold / --fold --s2d）全 PASS 是自动决策的回归证据。
- 段 3 (claim): Cout<16 layers (e.g., FC_xy with Cout=2) 当前不优化——结果是 PE 列空转，util=Cout/16；design rationale: 硬件保持简洁，MAC%-of-total negligible in ResNet-18-style。
- 段 4 (transition): 本节是 narrative A 的"自动化论据"，下一节进入与 hardware-reconfigurable alternatives 的对位。 [依赖: Interstellar ASPLOS'20 / Timeloop ISPASS'19]

### §5.4 Comparison with Hardware-Reconfigurable Alternatives

- 段 1 (claim): 硬件保持简洁固定 16×16 阵列时，编译器侧 fold 变换可以达成 hardware-reconfigurable 等价的 PE 利用率收益。 [依赖: Tab.2（4 维度对位）]
- 段 2 (evidence): 4 维度对位——硬件复杂度 / 编译复杂度 / 内存代价 / 适用场景——FLUX_CNN compiler-only Ky-fold/S2D vs MAERI ART / Eyeriss-v2 NoC / cuDNN im2col。 [依赖: MAERI / Eyeriss-v2 / cuDNN]
- 段 3 (comparison): trade-off 不互斥——两条路线各有适用域；hardware-reconfigurable 适合频繁切换不同形状的工作负载，compiler-only 适合形状已知的 ahead-of-time 编译。
- 段 4 (transition): 引 §7.2 实测 PE 利用率数据作为支撑；本节作为 narrative A 章收束节，下章进入 System Integration。 [依赖: [CHECK: S2D 加速器领域引用谱系]]



## 第 6 章 System Integration and Toolchain

### §6.1 PyTorch nn.Sequential End-to-End Compilation

- 段 1 (claim): FLUX_CNN 提供完整 PyTorch → 硬件的端到端编译栈，整网 bit-exact 通过。 [依赖: C2.5]
- 段 2 (evidence): 编译栈三层结构——compile_layer.py（PyTorch Conv2d → 硬件 cfg + 数据文件）+ compile_model.py（多层链式编译，_plan_ddr 分配 FM-shared DDR 区）+ run_regression.py DSL builder（Chain / _Node / resnet_block 7 行写完 11 层 ResNet block）。
- 段 3 (evidence): 11-case ResNet-like chain 全 PASS @ 593K cycles, 86.6% MAC%, 5.95 ms @ 100 MHz target / 8.69 ms @ 68.4 MHz Fmax。 [依赖: [CHECK: target vs Fmax fps 口径统一]]
- 段 4 (comparison): 与 TVM OSDI'18 / VTA Micro'19 / fpgaConvNet TNNLS'19 通用编译栈对照——FLUX_CNN 是 hand-rolled mini-compiler scope，但 22-case 链式回归提供完整可复现验证。 [依赖: TVM / VTA / fpgaConvNet]
- 段 5 (transition): 编译产物最终通过 DMA 流入硬件，下一节给 DMA 子系统的硬件落地。 [依赖: C3.3 链式 CASES]

### §6.2 AXI / DMA Subsystem with Vendor IP Integration

- 段 1 (claim): 原项目自写 IDMA/WDMA/ODMA RTL（~3000 行）替换为 Xilinx axi_dm IP + 轻量 *_ctrl 控制器，外部仅 1 个 AXI4 Master + 1 个 AXI-Lite Slave。 [依赖: C3.1]
- 段 2 (evidence): idma_ctrl/wdma_ctrl/odma_ctrl/rdma_ctrl 仅做 cmd 生成 + done 检测；mm2s_arb 让 IDMA/WDMA 共享 axi_dm 的 MM2S 通道串行仲裁；axi_m_mux 聚合 axi_dm.MM2S/S2MM + DFE 到外部单 master 口。 [依赖: Xilinx PG022 / [CHECK: vendor doc 引用方式]]
- 段 3 (claim): CFG_WRITE descriptor (TYPE_CFG=0x3) + DFE 让 host AXI-Lite 写从 ~50/层 降到 4/层；done sticky + 双端 cfg 写口（C3.4）解 race。 [依赖: C3.2 / C3.4]
- 段 4 (evidence): DataMover 性能与原版自写 DMA 持平 (+0.5%)；burst_size 16→256 调优证明 vendor IP 不牺牲吞吐。
- 段 5 (comparison): NVDLA 类似有"register list" descriptor 模式——FLUX_CNN 把 descriptor 类型扩展到 CFG_WRITE 而非仅 buffer 描述符。 [依赖: NVDLA]

### §6.3 SDP Post-Processing: Bias / Residual / Quantization Fusion

- 段 1 (claim): SDP（Single Data Point processor）把 5 步后处理融合在一段组合链：pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc。 [依赖: C1.4]
- 段 2 (evidence): R.1 把 bias 从 mac_array 内移到 SDP 段（解 MAX_COUT_SLICES=32 限制 + 解放 bias_rf 容量）；R.2 加 shortcut_mult/shortcut_shift 可编程量化因子做残差融合；Shortcut Bank（8192×128 SRAM = 32 BRAM）专用驻留残差源。 [依赖: memory/sdp_residual_fusion.md]
- 段 3 (comparison): NVDLA SDP 段命名 / 分段法直接借用——老老实实标 "NVDLA-inspired"；He CVPR'16 ResNet 是该 fusion 算法侧动机。 [依赖: NVDLA / He CVPR'16]
- 段 4 (comparison): 与 Liu Full-Stack TNNLS'21 streaming + residual fusion 对照——FLUX_CNN 不做跨层 layer fusion，仅做 SDP 内部融合。 [依赖: Liu TNNLS'21 / Jacob CVPR'18]
- 段 5 (transition): 新颖点仅在 R.1 bias 重定位 + R.2 可编程量化因子，不当主贡献，仅作 system 完整性论据；下一节进入 system 层面的多核 scaling。

### §6.4 Multi-Core Scaling: W-Slice with Computed Redundancy Halo

- 段 1 (claim): multicore_top.sv 参数化 N 核 wrapper 在 XC7K325T 上 N=2/4 综合通过且 N=4 W slice 20/20 case 全 bit-exact PASS。 [依赖: C3.5 / C3.7]
- 段 2 (evidence): W slice (Mode C) 让 N 核处理同一图的不同 W 段，computed redundancy halo + asymmetric pad 机制（如 N=2 K=3 pad=1 W=32：Core 0 处理 W[0..17) pad_l=1 pad_r=0，Core 1 处理 W[15..32) pad_l=0 pad_r=1，重叠 2 列 halo）。
- 段 3 (evidence): M2 阶段升级 axi_2to3 / axi_4to5 实现跨核 SRAM 直送——producer ODMA → crossbar → consumer ifb_axi_slave 经 ring 反压写 IFB SRAM。 [依赖: memory/m2_cross_core_pipeline.md]
- 段 4 (evidence): N=4 wslice1 3833 cycles vs N=2 5569 cycles = 1.45× speedup。 [依赖: [CHECK: 1.45× 接近线性论证 baseline 归一化] / [CHECK: ResNet 11-layer multicore 实测]]
- 段 5 (comparison): 与 Simba MICRO'19 chiplet × 36（channel slice）形成"FPGA 多核 W slice vs ASIC chiplet channel slice"对照——多核切分维度选择不同；BRAM-bound 而非通信 bound 是 FPGA 多核版的 scaling 域。 [依赖: Simba MICRO'19 / Liu TNNLS'21 / Kang Sensors'23 / [TBD: §6.4 是否升格独立章]]

### §6.5 Single Source of Truth: Parameter Management

- 段 1 (claim): params.py 作为 RTL/Python 唯一参数源，64 个 \`FLUX_*\` 宏覆盖核心尺寸 + SRAM 容量 + AXI/CSR + 全局地址映射 + 58 CSR addr，避免参数 drift。 [依赖: C3.6]
- 段 2 (evidence): python params.py 自动生成 RTL/flux_cnn_params.svh；RTL \`include\`，Python from params import *——RTL 与 Python 双侧消费同一参数源。
- 段 3 (transition): 这是工程实践细节而非主贡献，作为 system 完整性收束节出现；下章进入 quantified evaluation。 [依赖: [TBD: §6.5 是否值得占节位]]



## 第 7 章 Evaluation

> **章首口径声明**：本章所有 latency / GOPS 数字以 (a) 100 MHz target 与 (b) 68.4 MHz XC7K325T-2 OOC 实测 Fmax 双 operating point 报告；**单层 PE 利用率（§7.2）vs 整网 MAC%（§7.4）口径不同**——前者描述 mac_array 占空，后者含 IDMA/ODMA stall + 多层切换开销。

### §7.1 Experimental Setup

- 段 1 (setup): 评测平台为 Xilinx XC7K325T-FFG900-2 / Vivado 2023.1 综合（OOC IP-style 资源基准）+ ModelSim 仿真，commit `b158cab` 作为 reproducibility 数据快照。 [依赖: C3.3 / STATUS.md]
- 段 2 (setup): 测试套件三件套——22-case ResNet-18 风格 chain（11 layer × 3 mode：无 fold / `--fold` / `--fold --s2d`）+ 24-case corner 鲁棒性（K∈{1..7} × stride∈{1..4} × pad∈{0..3} × Cin/Cout/H/W 多组合）+ 20-case multi-core W slice（K∈{1,3,5,7} × stride∈{1,2} × W∈{8,32,33}）。 [依赖: C2.5 / C3.3]
- 段 3 (claim): 66 case 全 bit-exact PASS 是后续 §7.2-§7.5 quantified data 的可信度锚点；run_regression.py 链式 CASES + 墙钟 Wall_us 报告作为完整可复现验证基础设施。 [依赖: C3.3 / [CHECK: 22+24+20=66 case 总数最终核对]]
- 段 4 (transition): 给出 setup 三件套后，下节进入 narrative A 核心数据——单层 PE 利用率三列对比。 [依赖: §7.2]

### §7.2 PE Utilization: Ky-Folding + S2D Improvement

- 段 1 (claim): 浅层 PE 利用率基线低源于 Cin<HW_PE——Layer 1 (K=7, Cin=4) 12.5% / Layer 3-4 (Cin=8) 25% / Layer 5a/c (Cin=8, stride=2) 50%；**编译器侧 Ky-fold + S2D 启用后接近 100%，零 RTL 改动**。 [依赖: C2.1 / C2.2 / C2.3 / Tab.3]
- 段 2 (evidence): Tab.3 PE-util-by-layer 三列对照——基线 / `--fold` / `--fold --s2d`——把每层占空和整网平均都给齐；Cout<16 layers 在表脚注诚实标注"PE 列空转，util=Cout/16，design rationale 见 §5.3"。 [依赖: Tab.3 / [CHECK: Layer 1/3/4/5a/5c 三列实测 PE 利用率百分比]]
- 段 3 (evidence): Layer 1 (K=7, Cin=4) 启用 Ky-fold 后 groups_y=4 把 Ky 折到 cin_fake=16，PE 利用率从 12.5% 升到接近 100%；Layer 5a/c stride=2 启用 S2D 后 Cin' = 4·8 = 32 ≥ HW_PE 即可不再触发 Ky-fold。 [依赖: docs/pe-fold.md]
- 段 4 (claim): 单层 PE 利用率（mac_array 占空率）与整网 MAC%（§7.4 86.6%）口径不同——后者含 IDMA/ODMA stall + descriptor 拉取 + 多层切换开销，gap 来源在 §7.4 文字论证。 [依赖: [CHECK: 单层 vs 整网 MAC% 口径论证文字]]
- 段 5 (comparison): 与 MAERI ASPLOS'18 ART 网络 / Eyeriss-v2 JETCAS'19 NoC reconfig 形成"compiler-only vs hardware-reconfigurable"路线对比——同等 PE 利用率收益的不同 trade-off 取舍（详对位表见 §5.4 Tab.2）。 [依赖: MAERI / Eyeriss-v2 / Tab.2]
- 段 6 (transition): narrative A 单层数据已落地，下节给出硬件成本侧的 quantified evidence——综合资源 + Fmax。 [依赖: §7.3]

### §7.3 Resource Utilization and Fmax

- 段 1 (claim): 单核 XC7K325T-2 OOC 综合资源——36.9k LUT (18.1%) / 13.2k FF (3.2%) / 128 BRAM (28.8%) / 82 DSP (9.8%)，Fmax 68.4 MHz（WNS=-4.618 ns，未达 100 MHz target）。 [依赖: C1.5 / Tab.4 / [CHECK: Fmax / WNS / 资源数字最终重综合更新]]
- 段 2 (evidence): BRAM 明细（WB 57 / IFB 32 / Shortcut 32 / OFB 7+1 ≈ 128 BRAM）映射到 §4-§5 各模块容量配置；Tab.4 给单核 / N=2 / N=3 / N=4 推算与 XC7K325T 总容量 (445 BRAM) 的对照。 [依赖: Tab.4 / params.py / [CHECK: BRAM 明细 57/32/32/7+1 实际综合占用]]
- 段 3 (evidence): N=2 综合通过 74.4k LUT (36.5%) / 256 BRAM (57.5%)；N=3 推算 BRAM 86%（边缘）；N=4 BRAM 超容需缩 shortcut bank 8192→2048。 [依赖: C3.5 / [CHECK: N=2/3/4 多核综合 LUT/BRAM 数字]]
- 段 4 (claim): 3 核是 XC7K325T 不动 SRAM 配置的硬上限，scaling 域是 BRAM-bound 而非 LUT-bound 或通信 bound——这是 narrative C 中型 FPGA 落地的核心约束。 [依赖: C1.5 / [CHECK: 3 核 BRAM 86% 边缘判定数字]]
- 段 5 (comparison): Fmax 68.4 MHz 未达 100 MHz target 的 critical path 已知问题（mac_pe 缺 `(* use_dsp = "yes" *)` + SDP 组合链未流水）作为诚实标注；修复路径具体性见 §7.7。 [依赖: C1.5 / [TBD: use_dsp + SDP 流水化重综合是否在投稿前完成]]
- 段 6 (transition): 给完资源 + Fmax 双侧硬件成本，下节给整网 quantified end-to-end latency。 [依赖: §7.4]

### §7.4 End-to-End Latency and MAC Efficiency

- 段 1 (claim): 11-layer ResNet-18 风格 chain 整网 593K cycles / 86.6% MAC% / 5.95 ms @ 100 MHz target / 8.69 ms @ 68.4 MHz Fmax；端到端 throughput 168 fps @ target / 115 fps @ Fmax。 [依赖: C1.2 / C2.5 / C3.3 / Tab.5 / [CHECK: 593K cycles / 86.6% MAC% / Wall_us 实测最终核对]]
- 段 2 (evidence): Tab.5 端到端 latency 表逐层给 cycles + 占比 + Wall_us 列 + 两 operating point 对照；最大耗时层（如 Layer 1 K=7 大 H×W）作为 latency 主导项标注。 [依赖: Tab.5 / [CHECK: 11 layer 逐层 cycles 实测]]
- 段 3 (claim): 整网 86.6% MAC% 含 IDMA/ODMA stall + 多层切换 + descriptor 拉取的实际开销；这是 streaming row-ring 在 layer-serial 模式下能达到的实际可见占空率。 [依赖: C1.2 / [CHECK: 86.6% MAC% 实测重跑]]
- 段 4 (evidence): 单层 PE 利用率接近 100%（§7.2）vs 整网 86.6% MAC% 的 gap≈13% 可分解为：descriptor 链拉取 + 跨层 cfg 写入 + IDMA 跨 layer 切换 + ODMA backpressure 末端排空——逐项归因（粗）。 [依赖: [CHECK: 13% gap 分解项各占比]]
- 段 5 (comparison): 86.6% 与 Liu Full-Stack TNNLS'21 97% MAC% 同思路不同规模（中型 7-series vs 大型 Arria 10 GX1150），与 Snowflake ISCAS'17 91% 平均计算效率口径不完全等同——精细文字论证留 §7.6 head-to-head。 [依赖: Liu TNNLS'21 / Snowflake ISCAS'17 / §7.6]
- 段 6 (transition): 单核整网 quantified 完成，下节进入 narrative B 系统侧主数据——多核 scaling。 [依赖: §7.5]

### §7.5 Multi-Core Scaling

- 段 1 (claim): N=2 multicore DDR-mode TB PASS @ 9057 cycles vs 单核 8808 cycles（仅 +2.8% AXI 仲裁开销）；N=4 W slice 20/20 case 全 bit-exact PASS @ wslice1 3833 cycles vs N=2 5569 cycles = 1.45× speedup。 [依赖: C3.5 / C3.7 / Tab.6 / [CHECK: N=2 9057 / 单核 8808 / N=4 wslice1 3833 / N=2 5569 cycles 实测]]
- 段 2 (evidence): Tab.6 multi-core scaling 表三列——单核 / N=2 DDR-mode / N=4 W slice 20-case 子集——给 cycles + speedup + AXI 仲裁开销百分比 + halo 重叠开销。 [依赖: Tab.6]
- 段 3 (evidence): ResNet 11-layer multicore N=2/N=4 估算 1.7×/3.64× speedup（基于 W slice halo 模型 + 单核 593K cycles 推算），实测 chain 适配 1-2 天工作量待跑。 [依赖: C3.5 / C3.7 / [CHECK: ResNet 11-layer multicore N=2/4 实测 cycles] / STATUS §4]
- 段 4 (claim): N=4 wslice1 1.45× 不到线性 2× 的原因可分解为——(a) baseline 归一化（W=8 小图 startup overhead 占比大）+ (b) AXI 仲裁开销 + (c) W slice halo 重叠计算冗余 + (d) ODMA 末端排空。 [依赖: [CHECK: 1.45× 近线性论证 baseline 归一化]]
- 段 5 (comparison): 与 Simba MICRO'19 chiplet × 36 channel slice 形成"FPGA W slice vs ASIC chiplet channel slice"对位——多核切分维度选择不同；FPGA 多核版的 scaling 域是 BRAM-bound 而非通信 bound。 [依赖: Simba MICRO'19 / Liu TNNLS'21 / Kang Sensors'23]
- 段 6 (transition): narrative B 系统侧多核 quantified 完成，下节进入 prior art head-to-head 防御主战场。 [依赖: §7.6]

### §7.6 Comparison with Prior Art

- 段 1 (claim): Tab.7 性能维度对位 + Tab.8 功能 / 设计取舍维度对位是 narrative A/B 防御主战场——把 FLUX_CNN 与 TPU / Eyeriss / Gemmini / Snowflake / Angel-Eye / Aydonat / Lu Winograd / Liu Full-Stack / VTA / fpgaConvNet 同器件 / 同任务 / 同口径数字对位。 [依赖: Tab.7 / Tab.8 / literature.md]
- 段 2 (evidence): Tab.7 性能维度六列——dataset / 器件 / Fmax / GOPS / MAC% / latency——FLUX_CNN 数据从 §7.3-§7.5 直接搬入；baseline 数字 reviewer 阶段查原文最终对齐口径。 [依赖: Tab.7 / [CHECK: 各 baseline 整网 MAC% / 同器件 Fmax / 同器件资源占用 reviewer 阶段查原文]]
- 段 3 (evidence): Tab.8 功能 / 设计取舍五维度——dataflow / streaming granularity / multi-core 切分维度 / 编译栈 scope / SDP 支持——可与 §3.6 Tab.Positioning 整合避免重复；FLUX_CNN 在 row-ring streaming + W slice + compile-time fold 三维独占落点。 [依赖: Tab.8 / Tab.Positioning §3.6 / [CHECK: Angel-Eye SDP 支持具体形态]]
- 段 4 (claim): 86.6% 整网 MAC% vs Liu TNNLS'21 97% MAC% 的精细论证——同 paradigm 不同 scale（中型 7-series 36.9k LUT vs 大型 Arria 10 GX1150 ≈ 1150k LUT），不否认 Liu 更高，承认是同思路在不同规模下的可比数字。 [依赖: Liu TNNLS'21 / [CHECK: Arria 10 GX1150 LUT 数字]]
- 段 5 (claim): 86.6% vs Snowflake 91% 平均计算效率口径不完全等同——FLUX_CNN 含 IDMA/ODMA stall + 多层切换，Snowflake 91% 口径含义需在表脚注说明。 [依赖: Snowflake ISCAS'17 / [CHECK: Snowflake 91% 计算效率口径]]
- 段 6 (comparison): 与 Angel-Eye TCAD'18 同 7-series + INT8 最直接对照——controller-driven vs handshake-driven 是去中心化 valid-ready 流水（C1.1）的差异化；与 Gemmini DAC'21 同 16×16 INT8 但 Gemmini 中心 FSM vs FLUX_CNN 去中心化。 [依赖: Angel-Eye TCAD'18 / Gemmini DAC'21]
- 段 7 (transition): head-to-head 对位完成，下节进入诚实标注的章末收束——已知局限与 future work。 [依赖: §7.7]

### §7.7 Discussion: Known Limitations and Future Work

- 段 1 (claim): 诚实标注三类已知局限——(a) Fmax 68.4 MHz 未达 100 MHz target；(b) Cout<16 layers PE 列空转 design rationale；(c) Pooling / Depthwise Conv / 稀疏未做。 [依赖: C1.5]
- 段 2 (evidence): (a) Fmax 修复路径具体性——mac_pe 加 `(* use_dsp = "yes" *)` 属性 + SDP 5 步组合链流水线化预估 100+ MHz 同时省 17K LUT，按 STATUS §4 ROI 排序优先做。 [依赖: C1.5 / STATUS §4 / [CHECK: SDP 流水化 + use_dsp 重综合 100+ MHz 与省 17K LUT 估算]]
- 段 3 (evidence): (b) Cout<16 design rationale——硬件保持简洁，受影响层在 ResNet-18 风格网络中 MAC%-of-total negligible（如 FC_xy Cout=2 仅占总 MAC<1%）；(c) Pooling / DW / 稀疏 future work 列表。 [依赖: C2.3 / model_analysis.md / [CHECK: Cout<16 layers MAC%-of-total 数字]]
- 段 4 (claim): future work 按 STATUS §4 ROI 排序——use_dsp + SDP 流水化优先（投稿前可完成）/ ResNet 11-layer multicore chain 适配 1-2 天 / 片上 push 链 P2 完成态 2-3 天 / cross-layer streaming fusion 对照 Tangram ASPLOS'19 + cout slice + stage barrier 中长期。 [依赖: C1.5 / Tangram ASPLOS'19 / STATUS §4]
- 段 5 (transition): 章末诚实标注收束 + future work 已给出，自然过渡到 §8 Conclusion 全文 high-level 总结。 [依赖: §8.1]

## 第 8 章 Conclusion

> 单章不分小节；仅做收束 + 简短 future work 一句话；不在 conclusion 翻新 claim，不夸大 contribution 强度（保持与 §1.4 一致）。

### §8.1 Conclusion

- 段 1 (claim): FLUX_CNN 论证了在固定 16×16 INT8 阵列 FPGA 加速器上，**编译器侧 Ky-fold + S2D 变换可以替代 hardware-reconfigurable 路线达成接近 100% PE 利用率**（narrative A 主轴），并通过 row-ring streaming + N=4 W slice multi-core 验证了 system-level 完整性（narrative B 系统侧）。 [依赖: C1.2 / C2.1 / C2.2 / C3.5 / C3.7]
- 段 2 (evidence): 完整证据三件套——46+20 case 全 bit-exact PASS（22-case ResNet chain × 3 mode + 24-case corner + 20-case multi-core W slice）+ XC7K325T-2 OOC 综合 N=4 通过 + 11-layer ResNet-18 风格 chain 86.6% 整网 MAC%。 [依赖: C2.5 / C3.3 / C3.5 / [CHECK: 46+20 case 总数最终核对]]
- 段 3 (transition): future work 按 STATUS §4 ROI 排序——use_dsp + SDP 流水线化重综合更新 Fmax / Pooling / Depthwise Conv / 稀疏 / cross-layer streaming fusion 对照 Tangram ASPLOS'19 / ResNet 11-layer multicore chain 适配 / 片上 push 链 P2 完成态。 [依赖: C1.5 / Tangram ASPLOS'19 / STATUS §4 / [TBD: 是否点名 future work 具体里程碑]]

---

## 全文统计

> 填充完所有章节后回填。
