# 论文大纲 (Phase 2)

> Phase 2 章节级大纲。每章给出定位 / 目标 / 关键内容（100-300 字）/ 与贡献-文献的对齐 / 预估篇幅。
> 不展开到 sub-section 摘要——那是 Phase 3 的工作。

---

## 元信息

- **目标会议/期刊**：[TBD: 用户决定] — Phase 0/1 倾向 **FPGA / FCCM / TCAD / TVLSI / TRETS** 优先（FLUX_CNN 工程偏重 + Xilinx 7-series 平台导向）；按"中等篇幅 IEEE 会议风格"起草，预估 8-10 页双栏正文 + 0.5-1 页 references
- **总章节数**：8 章（Introduction / Background / Related Work / Architecture / Compiler Optimizations / System Integration / Evaluation / Conclusion）
- **预估总页数**：~10 页双栏（IEEE 会议风格） / [TBD: 期刊版可扩到 14-16 页]
- **完成时间**：2026-04-30
- **数据快照**：commit `b158cab`，STATUS.md 时间戳 2026-04-30

## 论文标题（草拟）

- **候选 1（narrative A 主推）**：*"Compiler-Side PE Utilization for Fixed 16×16 INT8 CNN Accelerators: Ky-Folding and Space-to-Depth on a Streaming FPGA Pipeline"*
- **候选 2（narrative A+B 平衡）**：*"FLUX_CNN: A Streaming Row-Ring CNN Accelerator with Compiler-Driven PE Utilization on Xilinx 7-Series FPGAs"*
- **候选 3（narrative D 完整性）**：*"An Open-Source 16×16 INT8 CNN Accelerator with Decentralized Handshake Pipeline, Row-Ring Streaming, and PyTorch-to-RTL Compiler"*
- [TBD: 标题最终待用户定；候选 2 在 narrative 平衡上最稳妥]

## 一句话摘要

我们提出 FLUX_CNN——一个 16×16 INT8 CNN 加速器，**硬件保持简洁固定（去中心化 valid-ready 流水 + row-ring streaming 数据路径）**，**编译器侧（Ky-fold + Space-to-Depth）填满 PE**，单次 `start` 跑完任意 H×W 输入，PyTorch nn.Sequential 端到端 bit-exact 编译。在 Xilinx XC7K325T 上单核综合 36.9 k LUT / 128 BRAM / 82 DSP，22-case ResNet-18 风格回归整网 86.6% MAC% / 593K cycles，多核 N=4 W slice 20/20 case bit-exact 通过。

## Narrative 选择（显式陈述）

**采用 narrative：混合 A+B（A 为主轴，B 为系统侧支撑）**

- **主轴 A：compiler-side PE utilization** — 把 Ky-fold（C2.1）+ Space-to-Depth（C2.2）作为顶层 contribution，**核心论点 = "硬件保持简洁固定 16×16 阵列，编译器侧重映射填满 PE"**。该 claim 在 literature.md 检索中**未命中具体 prior art**（强度强）；与 MAERI（硬件 ART 网络）/ Eyeriss-v2（NoC 重构）形成 "compiler-only vs hardware-reconfigurable" 对位
- **二级 B：row-streaming + multi-core W slice** — 作为 **system context** 出现，论证 "compiler 优化在一个完整的 streaming 系统里 deliver"。Alwani / Kang / Liu 三个近邻 prior art 在 Related Work 章精细差异化（row-level vs layer-level granularity；layer-serial vs layer-pipelined；外存承认 vs 全片上）

**为什么不选 C（去中心化握手）作为主轴**：工程美学 claim 弱，Buffets@ASPLOS'19 已提供形式化语言，FLUX_CNN 仅是该思想的 RTL 落地；保留为 §IV Architecture 的实现细节论据，不主推

**为什么不选 D（端到端开源）作为主轴**：高度依赖开源决策（[TBD]）；scope 过宽，单篇会议论文 9-10 页装不下完整端到端论证

**取舍**：narrative C 元素（去中心化握手）在 §IV 作为 "Why this design pays off" 论据出现；narrative D 元素（开源 + PyTorch 编译栈）在 §VI System Integration 作为 system completeness 二级出现

---

## 章节清单

### 第 1 章：Introduction

- **章节定位**：开篇——立问题、立 claim、列贡献
- **目标**：
  - 让审稿人在 1-2 页内理解 (a) 为什么 fixed-array 加速器在浅层 (Cin<16) 上 PE 利用率低 / (b) 为什么 row-streaming 是端侧 large-image 推理的必要选择 / (c) 我们提供什么
  - 读者读完应能复述 4-5 条贡献
- **包含小节**：
  - 1.1 Motivation: PE Underutilization in Fixed Arrays + Streaming Constraints on Edge FPGAs
  - 1.2 Limitations of Existing Approaches（一段话点出 hardware-reconfigurable / layer-pipelined / im2col 三条主流路线的局限）
  - 1.3 Our Approach（compiler-side PE utilization + row-ring streaming + decentralized pipeline）
  - 1.4 Contributions（5 条左右，带 quantified results）
  - 1.5 Paper Organization
- **依赖项**：
  - 贡献：C2.1 Ky-fold（核心）、C2.2 S2D（核心）、C1.2 row-streaming（system context）、C2.5 PyTorch 编译栈（system completeness）、C3.5/C3.7 multi-core（system scaling 论据）
  - 文献：MAERI（对位 hardware-reconfigurable）、Eyeriss-v2（对位 NoC 重构）、cuDNN im2col（对位 全展开）、Alwani/Kang/Liu（streaming 谱系）、TPU/Gemmini/Eyeriss（fixed array 基线）
- **未决问题**：
  - [TBD: contribution 列表 5 vs 6 条由 Phase 3 sub-section 起草时定]
  - [CHECK: §1.4 PE 利用率提升数字（12.5%→~99%）需 project-analyst 跑 fold mode 取实测填入]
  - [TBD: §1.4 fps 用 100 MHz target 168fps 还是 68.4 MHz Fmax 115 fps 口径——按 contributions.md §4.6 建议统一用 Fmax 实测口径]
- **如何防御审稿挑战**：
  - 弱 claim C1.4 (SDP residual fusion) 不进 contribution 列表，仅作 system 完整性论据
  - 弱-中 claim C1.1 (去中心化握手) 不进 contribution 列表
  - 老老实实承认 Cout<16 不优化 + Fmax 68.4 MHz 未达 100 MHz target，避免审稿人挑 "over-claim"
- **预估篇幅**：1.5 页（IEEE 会议双栏）

### 第 2 章：Background and Motivation

- **章节定位**：奠定术语 / 数据 / motivation
- **目标**：
  - 用一组 quantitative breakdown（model_analysis.md §2 表）说服审稿人 "Cin<16 在 ResNet-18 风格网络的浅层是 chronically PE-underutilized"
  - 引入 row-ring streaming 对端侧 large-image 推理的必要性（VGA 480×640 = 4.9 MB 单图 vs 端侧 FPGA BRAM 容量）
  - 简要回顾 spatial array dataflow taxonomy（WS / OS / RS）以便后文对位
- **包含小节**：
  - 2.1 Spatial Array Dataflows: WS / OS / RS Taxonomy
  - 2.2 PE Utilization Pathology in Shallow Layers（Cin/Cout 失衡定量分析）
  - 2.3 Streaming vs Tiled: Memory Footprint of Large-Image Inference
  - 2.4 Design Goals（汇总 motivation → 本工作 design constraints）
- **依赖项**：
  - 贡献：C2.4 多 slice 切片（loop nest taxonomy）、C1.2 row-ring（streaming 必要性）；motivation 章不直接 claim 贡献，只引向后文
  - 文献：Eyeriss ISCA'16（RS 数据流）、ShiDianNao ISCA'15（OS）、TPU ISCA'17（WS）、Ma FPGA'17（loop nest）、Interstellar ASPLOS'20（7 nested loops 形式化）、Zhang FPGA'15（Roofline）；motivation 数据来自 model_analysis.md
- **未决问题**：
  - [CHECK: §2.2 各浅层 PE util 实测百分比表（Layer1/3/4/5a/5c 启用 fold 前后）需 project-analyst 跑实测填入]
  - [TBD: §2.3 是否要画"片上 SRAM 占比 vs 输入图大小"的曲线图说明 row-streaming 必要性 — 由 Phase 3 段落骨架决定是否值得占图位]
- **如何防御审稿挑战**：
  - §2.2 motivation 数字一定要用 ResNet-18 风格而不是单层捏造的极端 case，避免被挑 "cherry-picked"
  - §2.3 要点明"为什么不用 Alwani-style layer fusion 解决 streaming 问题"——预告 §3 Related Work 会展开
- **预估篇幅**：1 页

### 第 3 章：Related Work

- **章节定位**：与 prior art 对位的关键章节，3 处 prior art 威胁的主要应对场所
- **目标**：
  - 系统化论证 FLUX_CNN 在 spatial array dataflow / streaming / compiler co-design 三条线上的位置
  - **重点应对**：Alwani Fused-layer (MICRO'16) / Kang AoCStream (arXiv'22 + Sensors'23) / Liu Full-Stack (TNNLS'21) 三处 streaming 近邻 prior art
  - 论证 Ky-fold + S2D 在 compiler-side PE utilization 文献中**无直接 prior art** ——这是 narrative A 的关键防御点
- **包含小节**：
  - 3.1 Spatial Array Accelerators (TPU / Eyeriss / Gemmini / NVDLA / Simba) — 尺度对照
  - 3.2 Hardware-Reconfigurable Approaches to PE Utilization (MAERI / Eyeriss-v2 / Tangram) — 对位 narrative A 的"compiler-only" alternative
  - 3.3 Compiler / Loop-Nest Co-Design (Timeloop / Interstellar / TVM / VTA / Ma FPGA'17 / cuDNN im2col) — narrative A 的同语言谱系
  - 3.4 Streaming / Line-Buffer FPGA Accelerators (fpgaConvNet / DnnWeaver / Snowflake / Angel-Eye / Aydonat / Lu Winograd / Alwani / Kang / Liu) — narrative B 的主对照集；**3 处 prior art 威胁在此精细差异化**
  - 3.5 Quantization and Fusion (Jacob / ResNet / EIE / ESE / NVDLA SDP) — 简短，作为 §VI 的 framing
  - 3.6 Positioning of FLUX_CNN（一段总结表，把 5 维度 placement 钉死：dataflow / PE-utilization 路线 / streaming granularity / multi-core 切分维度 / 编译栈 scope）
- **依赖项**：
  - 贡献：所有 C1-C3 都在此章被 prior art 校准；尤其 C2.1 / C2.2（narrative A 防御点）/ C1.2 / C3.5 / C3.7（narrative B 防御点）
  - 文献：literature.md A-G 全部 32 篇；§3.4 重点引 Alwani / Kang / Liu / fpgaConvNet / Snowflake / Angel-Eye；§3.2 重点引 MAERI / Eyeriss-v2 / Tangram；§3.3 重点引 Timeloop / Interstellar / Ma / cuDNN
- **未决问题**：
  - [CHECK: S2D 在加速器领域的引用谱系不清晰（literature.md §D Shi 2016 条目）— reviewer 阶段需补查 ASPLOS/HPCA 是否有 architecture-side S2D 更早出处；本章须主动揭示并标注差异点（compiler-pass vs train-time）]
  - [TBD: §3.5 是否单独成节 vs 并入 §VI System Integration 的 SDP 段落——视篇幅压力定]
  - [TBD: NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report 的引用方式（5 处 vendor doc）— 由主 Agent 决定引 vendor doc 还是衍生 paper]
- **如何防御审稿挑战**（针对 contributions.md §5.3 三处威胁）：
  - **Alwani**: 明确说"FLUX_CNN row-level vs Alwani layer-level granularity；FLUX_CNN 单核 layer-serial 共用硬件 vs Alwani 跨层同时驻留计算"
  - **Kang AoCStream**: 明确说 "FLUX_CNN 承认外存必然存在并优化 DDR 流量 vs Kang 强调全片上；FLUX_CNN layer-serial 共用硬件 vs Kang layer-pipelined 多 block"
  - **Liu Full-Stack**: 明确说 "FLUX_CNN 单核 layer-serial 不做 layer fusion；规模差（中型 7-series 86.6% MAC% vs 大型 Arria 10 GX1150 97% MAC%）是同思路在不同硬件规模下的对照"
  - **MAERI / Eyeriss-v2**: 明确说"硬件复杂度 vs 编译复杂度的不同 trade-off"，不互斥
- **预估篇幅**：1.5 页

### 第 4 章：Architecture

- **章节定位**：硬件架构总览（不深入 mechanism）；narrative A 的硬件支撑论据；narrative C 元素出现处
- **目标**：
  - 让审稿人理解 FLUX_CNN 的 5 模块核流水 + DMA 子系统的整体形态
  - 论证去中心化 valid-ready 握手（C1.1）+ row-ring streaming（C1.2）+ per-col PARF（C1.3）这些设计如何共同支撑 narrative A "硬件保持简洁固定" 的 claim
  - **不深入 SDP 细节**——SDP 留到 §VI System Integration（system 完整性论据）
- **包含小节**：
  - 4.1 Top-Level Architecture（5-module core pipeline + DMA subsystem 框图）
  - 4.2 Decentralized Valid-Ready Pipeline（C1.1 — 每模块独立 counter，无中心 FSM；FILL/DRAIN overlap）
  - 4.3 Streaming Row-Ring Datapath with Bidirectional Credit Backpressure（C1.2 — strip_rows × W ring，rows_available ≥ yout·stride+Ky 触发，IDMA/计算/ODMA 三阶段并发）
  - 4.4 Per-Column PARF Accumulator（C1.3 — parf_col × 16 列独立 SRAM）
  - 4.5 Loop-Nest Realization（C2.4 — 7 层硬件循环嵌套与 Interstellar/Ma FPGA'17 形式化对位）
- **依赖项**：
  - 贡献：C1.1 / C1.2 / C1.3 / C2.4 主推；C1.5（critical path 已知问题）作为 §VII Evaluation 的诚实标注前置
  - 文献：Buffets ASPLOS'19（C1.1 framing）、Eyeriss ISCA'16（OS dataflow 对位）、Snowflake ISCAS'17（line-buffer 命名传承）、Interstellar ASPLOS'20（C2.4 形式化）、Ma FPGA'17（loop nest 对位）；NVDLA（架构粒度命名 CDMA→CMAC→CACC→SDP 对位 line_buffer→mac_array→parf_accum→sdp）
- **未决问题**：
  - [TBD: §4.4 per-col PARF 是否与 §IV.2 valid-ready 合并 — 取决于篇幅]
  - [TBD: 是否在 §4.1 框图旁配 timing diagram 展示 IDMA/计算/ODMA 三阶段并发 — Phase 3 决定]
  - [CHECK: §4.4 parf BRAM 数量贡献单独估算（per-col 16 SRAM 各占多少 BRAM）需查综合报告 cell utilization]
- **如何防御审稿挑战**：
  - C1.1 去中心化握手不主推为顶层贡献——避免被挑 "工程美学，无 quantified value"；放在 §4.2 作为 architecture 论据，配 22-case 全 PASS 数据支撑
  - C1.3 per-col PARF 作为"单元级 implementation detail"出现，不作顶层贡献
- **预估篇幅**：1.5 页

### 第 5 章：Compiler Optimizations for PE Utilization（**narrative A 主章**）

- **章节定位**：本论文核心贡献章；narrative A 的核心证据
- **目标**：
  - 用具体数学推导 + 代码定位 + 实测 PE 利用率数据论证 Ky-fold + S2D 在 Cin<16 / stride≥2 的浅层上把 PE 利用率从 12.5%-50% 提升到接近 100%，**零 RTL 改动**
  - 与 cuDNN im2col / MAERI ART / Eyeriss-v2 NoC / Sub-pixel CVPR'16 形成清晰对位
  - 论证 trigger condition + 自动决策（C2.3）确保编译器在通用 ResNet 风格网络上能自动选择最佳变换组合
- **包含小节**：
  - 5.1 Ky-Folding（C2.1 — Cin<16 时把 Ky 折到 cin_fake，输入 y-方向偏移复制 + 权重 reshape；硬件按普通 conv 跑）
  - 5.2 Space-to-Depth Folding（C2.2 — stride≥2 时把 stride² 个相位折到 cin，编译器侧重排不复制；DDR 友好）
  - 5.3 Joint Trigger Logic and Decision Automation（C2.3 — Ky-fold 触发于 K>1 AND Cin<16；S2D 触发于 stride≥2 AND K≥stride；S2D 启用后重判 Ky-fold）
  - 5.4 Comparison with Hardware-Reconfigurable Alternatives（与 MAERI / Eyeriss-v2 / cuDNN im2col 对位表）
- **依赖项**：
  - 贡献：C2.1 / C2.2 / C2.3（核心）；C2.4 多 slice 切片作为 implementation context（§4.5 已铺垫）
  - 文献：cuDNN arXiv'14（im2col 对位）、Sub-pixel CVPR'16（S2D 算法侧出处）、MAERI ASPLOS'18（hardware-reconfigurable 对位）、Eyeriss-v2 JETCAS'19（NoC reconfig 对位）、Ma FPGA'17（loop unrolling 形式化）、Interstellar ASPLOS'20（compiler-hardware co-design framing）
- **未决问题**：
  - [CHECK: §5.1 Layer 1/3/4 启用 Ky-fold 后实测 PE 利用率百分比（12.5%/25% → ?）— project-analyst 跑 `run_regression.py --fold` 取 cycles 反算]
  - [CHECK: §5.2 启用 S2D 后 DDR 带宽节省比例 — 需对照 run 统计 IDMA 读次数]
  - [CHECK: §5.4 S2D 在加速器领域的引用谱系（literature.md 残留 [CHECK] 之一）— reviewer 阶段补查 ASPLOS/HPCA 是否有 architecture-side S2D 更早出处]
- **如何防御审稿挑战**：
  - **Cout<16 不优化**——必须在 §5.3 末尾或 §5.4 末尾老老实实承认 "Cout<16 layers (e.g., FC_xy with Cout=2) result in PE column idling, util=Cout/16"，并说明 design rationale ("hardware kept simple; affects negligible MAC%-of-total in ResNet-18-style networks")
  - **S2D prior art 可能突袭**——主动揭示 Sub-pixel CVPR'16 是 train-time sub-pixel layer，FLUX_CNN 是 compile-pass space-to-depth；reviewer 阶段如发现 architecture-side S2D 更早出处，提前在 §5.4 占好位
  - **PE 利用率口径**——必须在本章首段明确"单层 PE 利用率"vs"整网 MAC%"的区分，避免 §VII Evaluation 的 86.6% 整网 MAC% 与 §5 单层数字之间的口径冲突
- **预估篇幅**：2 页（narrative A 主章，篇幅最长）

### 第 6 章：System Integration and Toolchain（**narrative D 元素 + system 完整性**）

- **章节定位**：把 §IV 硬件 + §V 编译器 落到完整可工作系统；system completeness 论据；narrative B 部分要素（multi-core）出现处
- **目标**：
  - 论证 FLUX_CNN 不只是 RTL 实验品，而是有完整 PyTorch 编译栈 + DMA 子系统 + descriptor-driven 配置流 + 多核 wrapper 的端到端系统
  - **multi-core W slice (C3.7) 在此章作为 narrative B 系统侧支撑**（不当 narrative B 主章；narrative A 主轴下 multi-core 是 scaling 论据，非主推）
  - SDP residual fusion (C1.4) 在此作为 system 完整性论据
- **包含小节**：
  - 6.1 PyTorch nn.Sequential End-to-End Compilation（C2.5 — compile_layer / compile_model / DSL builder Chain/_Node/resnet_block）
  - 6.2 AXI / DMA Subsystem with Vendor IP Integration（C3.1 — Xilinx axi_dm IP + 轻量 *_ctrl + mm2s_arb 仲裁；C3.2 CFG_WRITE descriptor + descriptor-driven 配置流）
  - 6.3 SDP Post-Processing: Bias / Residual / Quantization Fusion（C1.4 — R.1 bias→SDP 重构 + R.2 SDP residual fusion w/ programmable shortcut_mult/shift；shortcut bank 8192×128 SRAM）
  - 6.4 Multi-Core Scaling: W-Slice with Computed Redundancy Halo（C3.5 + C3.7 — multicore_top wrapper + axi_2to3/4to5 + ifb_axi_slave 跨核 SRAM 直送 + DDR row_stride 解耦 cmd_btt + computed redundancy halo + asymmetric pad）
  - 6.5 Single Source of Truth: Parameter Management（C3.6 — params.py → flux_cnn_params.svh，简略）
- **依赖项**：
  - 贡献：C1.4 / C2.5 / C3.1 / C3.2 / C3.3 / C3.4 / C3.5 / C3.6 / C3.7
  - 文献：NVDLA（C1.4 SDP 命名传承 + 工业 IP 对位）、Xilinx PG022（C3.1 vendor IP 对位）、TVM OSDI'18 / VTA Micro'19 / fpgaConvNet TNNLS'19（C2.5 编译栈对位）、Simba MICRO'19（C3.5 multi-core 对位）、Liu TNNLS'21 / Kang AoCStream（C3.7 streaming + multi-core 切分维度对位）、ResNet CVPR'16（C1.4 residual 必要性算法侧动机）
- **未决问题**：
  - [TBD: §6.4 是否单独成章（提升 narrative B 比重）vs 维持 §6 节内（narrative A 主轴下作为 scaling 论据） — 当前选后者；用户可决定升格]
  - [CHECK: §6.4 N=4 wslice1 1.45× speedup 是否近线性需 baseline 归一化（contributions.md §8.1 #9）]
  - [CHECK: §6.4 ResNet 11-layer multicore N=2/4 实测 cycles vs 估算（STATUS line 254 估 1-2 天，contributions.md §8.1 #8）]
  - [TBD: §6.5 params.py 是否值得占节位 — 篇幅紧时合并到 §6.2 末尾 1 段]
- **如何防御审稿挑战**：
  - SDP residual fusion (C1.4) 老老实实标 "NVDLA-inspired"，新增点仅在 R.1 bias 重定位 + R.2 shortcut_mult/shift 可编程量化因子 — 不当主贡献，仅作 system 完整性
  - axi_dm vendor IP 集成不夸大 ("vendor-grade throughput with ~3000-line RTL saving")，强调"system 集成实用性"而非"原创架构贡献"
  - C3.2 CFG_WRITE descriptor 类似 NVDLA register list 模式，不夸大新颖性
- **预估篇幅**：2 页

### 第 7 章：Evaluation

- **章节定位**：把 §IV/§V/§VI 的 claim 用 quantified data 钉死；narrative A/B 防御主战场
- **目标**：
  - 4 类 quantified evidence: (a) PE 利用率提升（narrative A 核心）/ (b) 综合资源 + Fmax / (c) 端到端 latency + 整网 MAC% / (d) 多核 scaling
  - 与 prior art 对位的 head-to-head 比较表（contributions.md §5.1 + §5.2 已起骨架）
  - **诚实标注弱点**：Fmax 68.4 MHz 未达 100 MHz target / Cout<16 不优化 / Pooling/DW 未做（contributions.md §4.6）
- **包含小节**：
  - 7.1 Experimental Setup（XC7K325T-FFG900-2 OOC / Vivado 2023.1 / ModelSim / 22-case ResNet-18 风格 chain + 24-case smoke + 20-case multi-core W slice）
  - 7.2 PE Utilization: Ky-Folding + S2D Improvement（narrative A 核心数据；浅层 12.5%-50% → 接近 100%）
  - 7.3 Resource Utilization and Fmax（4.1 表 — 单核 36.9k LUT / 128 BRAM / 82 DSP / 68.4 MHz Fmax；2/3/4 核 scaling；BRAM-bound 分析）
  - 7.4 End-to-End Latency and MAC Efficiency（4.3 表 — 11 层 ResNet-18 风格 chain 593K cycles / 86.6% MAC% / 5.95 ms @ 100 MHz target / 8.69 ms @ 68.4 MHz Fmax）
  - 7.5 Multi-Core Scaling（4.5 表 — W slice 20/20 PASS, N=4 wslice1 1.45× speedup; ResNet 11-layer N=2/4 估算 1.7×/3.64×）
  - 7.6 Comparison with Prior Art（contributions.md §5.1 性能维度表 + §5.2 功能/设计取舍表）
  - 7.7 Discussion: Known Limitations and Future Work（4.6 表 — Fmax 100 MHz 未达 / use_dsp 属性未加 / Pooling DW 稀疏未做 / ResNet multi-core chain 适配中）
- **依赖项**：
  - 贡献：所有 C1-C3 在此被 quantified validate；尤其 C2.1/C2.2 (PE util) / C1.2 (streaming) / C3.5/C3.7 (multi-core) 是数据集中处
  - 文献：所有 baseline 工作（TPU / Eyeriss / Gemmini / Snowflake / Angel-Eye / Aydonat / Lu / Liu / VTA / fpgaConvNet）
- **未决问题**：
  - [CHECK: §7.2 浅层 PE 利用率实测百分比（contributions.md §8.1 #1, #2）— project-analyst 跑 fold mode 取实测]
  - [CHECK: §7.4 Wall_us 端到端实测数字（contributions.md §8.1 #5）— 重跑回归取 Wall_us 列]
  - [CHECK: §7.4 单层 vs 整网 MAC% 口径论证（contributions.md §8.1 #6）— 文中文字论证]
  - [CHECK: §7.5 ResNet 11-layer multicore N=2/4 实测（contributions.md §8.1 #8）— STATUS line 254 估 1-2 天]
  - [CHECK: §7.6 各 baseline 整网 MAC% / 同器件 Fmax / Angel-Eye SDP 支持 / 同器件资源占用（contributions.md §8.1 #10, #11, #12, #13）— reviewer 阶段查原文]
  - [TBD: §7.6 是否同步把 mac_pe `(* use_dsp = "yes" *)` 加上重综合更新数字 + SDP 流水线化重综合更新 Fmax — 影响 §7.3/§7.4 数据（contributions.md §8.2 #4, #5）]
- **如何防御审稿挑战**：
  - **数据口径全文统一**：§7 首段明确 "We report two clock-frequency operating points: (a) 100 MHz target with critical-path optimization roadmap, (b) 68.4 MHz current Fmax. Latency / GOPS numbers are accompanied by both points where they differ."
  - **86.6% 整网 MAC% vs Liu 97% MAC%**：在 §7.6 文字论证 "same paradigm at different scale"——中型 7-series vs 大型 Arria 10 GX1150；不否认 Liu 更高，承认是同思路在不同规模下的可比数字
  - **86.6% 整网 MAC% vs Snowflake 91% 平均计算效率**：口径不完全等同（FLUX_CNN 含 IDMA/ODMA stall），需在表脚注说明
  - **Fmax 68.4 MHz 修复路径已知**（contributions.md C1.5）：在 §7.7 老实标注，给出修复路径（SDP 流水线化 + use_dsp 属性 → 100+ MHz 估省 17K LUT）
  - **Pooling/DW/稀疏不做**：在 §7.7 简短列入 future work，不试图掩饰
- **预估篇幅**：2.5-3 页（数据章，篇幅次长）

### 第 8 章：Conclusion

- **章节定位**：收束；不引入新内容，不重复贡献清单
- **目标**：
  - 一段话回顾 narrative A 主轴（compiler-side PE utilization on fixed array） + narrative B 系统侧（row-streaming + multi-core）
  - 一段话指出 future work（contributions.md §4.6 + STATUS §4 ROI 排序：use_dsp 属性 / SDP 流水线 / Pooling DW / cross-layer streaming fusion / ResNet multi-core chain 适配）
- **包含小节**：单节，不分小节
- **依赖项**：
  - 贡献：高 level 总结 5 条左右
  - 文献：仅引 Tangram (cross-layer streaming fusion 未来工作对照) / 可能再引 1-2 篇 motivation 文献回环
- **未决问题**：
  - [TBD: 是否点名 future work 的具体里程碑（如 P2 片上 push 链 / cout slice / stage barrier） — 取决于 conclusion 是否要给出 roadmap timestamp]
- **如何防御审稿挑战**：
  - 不在 conclusion 翻新 claim
  - 不夸大 contribution 强度（保持与 §1.4 一致）
- **预估篇幅**：0.5 页

---

## 叙事一致性检查

### 贡献-章节映射表

| 贡献编号 | 强度自评（contributions.md） | 主章节 | 二级章节 | narrative 角色 |
|---------|------------------------------|--------|---------|----------------|
| C1.1 去中心化 valid-ready 流水 | 弱-中（工程美学） | §4.2 | §1.3, §7.7 | narrative A 硬件简洁性论据 |
| C1.2 Streaming row-ring + 双向反压 | 中（与 Alwani/Kang/Liu 差异化） | §4.3 | §1.1, §1.3, §7.4 | narrative B 主推 + narrative A 系统侧 |
| C1.3 PARF per-col SRAM | 工程化（implementation detail） | §4.4 | — | narrative A 实现细节，不主推 |
| C1.4 SDP fusion (Bias/Residual/Quant) | 偏增量 | §6.3 | §3.5, §7.7 | narrative D system 完整性 |
| C1.5 Critical path 已知问题诚实披露 | 实现状态 | §7.7 | §1.4 (隐式) | 防御性论据 |
| C2.1 Ky-fold | **强**（无直接 prior art） | §5.1 | §1.3, §7.2 | **narrative A 核心** |
| C2.2 Space-to-Depth | 中-强（S2D 谱系待补查） | §5.2 | §1.3, §7.2 | **narrative A 核心** |
| C2.3 自动决策 + 受益层判定 | 工程化 | §5.3 | — | narrative A implementation |
| C2.4 7 层硬件循环嵌套 + cin/cout 多 slice | 偏增量（loop tiling 已公开） | §4.5 | §2.1, §3.3 | narrative A 形式化对位 |
| C2.5 PyTorch nn.Sequential 多层编译 | 工程化 | §6.1 | §1.3 | system completeness |
| C3.1 Xilinx axi_dm IP + 轻量 *_ctrl | 工程化 | §6.2 | — | system completeness |
| C3.2 CFG_WRITE descriptor + descriptor-driven 配置 | 工程化 | §6.2 | — | system completeness |
| C3.3 链式 CASES + DSL builder + 墙钟报告 | 工程化（验证基础设施） | §7.1 (Experimental Setup) | §6.1 | 验证完整性 |
| C3.4 Done sticky + 双端 cfg 写口 | 机制级（小） | §6.2（一段话）| — | implementation detail |
| C3.5 Multi-core wrapper N=4 | 工程化 + 系统集成 | §6.4 | §1.3, §7.5 | narrative B 系统侧 |
| C3.6 params.py single source | 工程化（辅助） | §6.5（合并到 §6.2 候选）| — | implementation aid |
| C3.7 W slice (Mode C) | 中-强（多核切分维度） | §6.4 | §1.3, §7.5 | **narrative B 主推** |

**覆盖率**：17 条 contribution **全部归位**（grep `^### C[0-9]\.[0-9]` contributions.md 实际为 17 条 = C1.1-C1.5 / C2.1-C2.5 / C3.1-C3.7；主 Agent 提示词的"18 条"为 off-by-one，本大纲按实际 17 条对齐）。其中 C3.6 在 §6.5 单节或合并 §6.2 末尾，二选一。
**未出现的 contribution**：0 条。
**最薄归位**：C1.5（critical path 已知问题）仅在 §7.7 出现，作为防御性论据；这是 contributions.md 自评 "实现状态披露" 应有的位置，无需补强。

### 文献-章节映射（重点 prior art）

| Prior art | 威胁度 | 主对位章节 | 应对策略 |
|-----------|-------|-----------|---------|
| Alwani Fused-layer @ MICRO'16 | 🔴 高 | §3.4 | row-level vs layer-level granularity；layer-serial 单核 vs 多层同时驻留 |
| Kang AoCStream @ arXiv'22 / Sensors'23 | 🔴 高 | §3.4 | 承认外存 vs 全片上；layer-serial vs layer-pipelined 多 block |
| Liu Full-Stack @ TNNLS'21 | 🟡 中 | §3.4 + §7.6 | 单核 layer-serial 不做 layer fusion；规模差（中型 7-series 86.6% vs 大型 Arria 10 97%）是同思路不同规模 |
| MAERI @ ASPLOS'18 | 中（narrative A 主对位） | §3.2 + §5.4 | 硬件 ART 网络 vs 编译器 Ky-fold/S2D，"硬件复杂度 vs 编译复杂度"trade-off |
| Eyeriss-v2 @ JETCAS'19 | 中（narrative A 主对位） | §3.2 + §5.4 | NoC reconfig vs compiler-only |
| cuDNN im2col @ arXiv'14 | 弱（narrative A 同源） | §3.3 + §5.4 | im2col 全展开 vs Ky-fold 仅折 Ky 维 |
| Sub-pixel CVPR'16 | 中（S2D 算法侧出处） | §5.4 | train-time sub-pixel layer vs compile-pass S2D |
| NVDLA | 弱-中（架构命名传承） | §4.1 + §6.3 | CDMA→CMAC→CACC→SDP 命名脉络致敬；C1.4 SDP fusion 在此对位 |
| TPU v1 ISCA'17 | 弱（尺度对照） | §3.1 + §7.6 | 数据中心 vs 端侧规格差异 |
| Eyeriss ISCA'16/JSSC'17 | 弱（dataflow 对位） | §2.1 + §3.1 | RS vs OS dataflow taxonomy |
| Gemmini DAC'21 | 中（最贴近对照） | §3.1 + §7.6 | 16×16 INT8 同尺度但中心 FSM vs 去中心化 |
| Buffets ASPLOS'19 | 弱（C1.1 framing） | §4.2 | 形式化语言提供者；FLUX_CNN 是 RTL 落地 |
| Snowflake ISCAS'17 | 中（line-buffer 命名传承 + 91% 对照点） | §3.4 + §7.6 | line-buffer 模式继承；row-ring 反压扩展 |
| Angel-Eye TCAD'18 | 高（同器件最直接对照） | §3.4 + §7.6 | 同 7-series + INT8；controller-driven vs handshake-driven |

**3 处 prior art 威胁应对覆盖率**：✅ 全部在 §3.4 中精细差异化；其中 Liu 在 §7.6 head-to-head 比较表二次出现（86.6% vs 97% MAC% 同思路不同规模论证）。
**核心 prior art 引用覆盖率**：literature.md 35 篇全部在某一章出现；A 类（spatial array）/ C 类（streaming，含 3 近邻）/ D 类（PE utilization 优化）覆盖最密。

### 章节间一致性 ✅/⚠️

- ✅ **Intro 贡献声明 ↔ Evaluation 实验**：§1.4 列的 5 条贡献全部映射到 §7.2-§7.5 quantified data；§5 narrative A 主章数据（浅层 PE 利用率）映射到 §7.2；§6.4 multi-core data 映射到 §7.5
- ✅ **Related Work ↔ Eval head-to-head**：§3.4 列的 streaming prior art (Alwani/Kang/Liu/Snowflake/Angel-Eye) 在 §7.6 比较表全部对位；§3.2 hardware-reconfigurable (MAERI/Eyeriss-v2) 不列 head-to-head（不可直接比较，仅 §5.4 文字对位）
- ⚠️ **§4.4 PARF per-col vs §IV.2 valid-ready 是否合并节**：[TBD] 取决于 §4 篇幅
- ⚠️ **§3.5 Quantization fusion 章节归属 vs §6.3 SDP**：[TBD] 取决于 §3 篇幅；当前选 §3.5 简短 framing + §6.3 详细
- ⚠️ **§6.4 Multi-core 是否升格为独立章节（narrative B 比重）**：[TBD] 用户可决定升格
- ✅ **18 条 contribution 全部归位**，无 [TBD: 是否要写] 标记
- ⚠️ **C1.5 critical path 已知问题 ↔ §7.7 / §1.4**：必须在 §1.4 contributions 列表（提及"with documented optimization roadmap"）+ §7.7 详细标注，避免审稿人挑 over-claim

---

## 章节依赖图（文字版）

```
§1 Intro ──┐
           ├──> §7 Evaluation (主验证场)
§2 Background ──> §3 Related Work ──> §4 Architecture ──┐
                                       │                │
                                       └──> §5 Compiler ──> §6 System Integration
                                                                   │
                                                                   └──> §7 Evaluation
§7 ──> §8 Conclusion
```

**阅读路径**：
- **审稿人快读**：§1 Intro → §7.6 Comparison → §8 Conclusion（5-10 分钟，能抓主 claim）
- **架构 reviewer**：§1 → §3.1-§3.2 → §4 → §7.3 / §7.4 / §7.7 (resource + Fmax + limitations)
- **编译器 reviewer**：§1 → §3.3 → §5 → §7.2 (PE util)
- **system reviewer**：§1 → §3.4 → §6 → §7.5 (multi-core scaling) + §7.7 (limitations)
- **可独立阅读章节**：§4 Architecture / §5 Compiler / §6 System Integration 任意单读都能闭合（不强依赖前章）

**强依赖关系**：
- §5 Compiler 强依赖 §4.5 Loop-Nest Realization（提供形式化框架）
- §6.4 Multi-Core 强依赖 §4.3 Streaming Row-Ring（W slice 是 row-ring 的多核延伸）
- §7 Evaluation 强依赖 §1 Contributions（claim → quantified validate 一一对应）
- §3 Related Work 是 §5/§6/§7 的 prior art 防御铺垫

**弱依赖**：
- §2 Background 仅为 §3+§4+§5 提供共享术语；不读 §2 也能理解后续章节
- §6.3 SDP / §6.5 params.py 与主线弱关联，可作"system 完整性附录式"段落

---

## 待决清单

### [CHECK]：可通过实测/查文献解决的数据缺口

> 全文 grep `[CHECK:]` 共 **14 处**（章节内嵌 [CHECK] 与下表条目去重后归并）。下表列 13 项行动条目（其中 1 处 §4.4 parf BRAM 与 §7.3 的资源数据可视为同一行动）。

| # | 项 | 关联章节 | 来源 | 行动 |
|---|----|---------|------|------|
| 1 | Layer 1 (Cin=4) 启用 Ky-fold 后实测 PE 利用率百分比 | §1.4 / §5.1 / §7.2 / §2.2 | contributions.md §8.1 #1 | project-analyst 跑 `--fold` 取 cycles 反算 |
| 2 | Layer 3/4 (Cin=8) 启用 Ky-fold 后实测 PE 利用率 | §5.1 / §7.2 / §2.2 | contributions.md §8.1 #2 | 同上 |
| 3 | 启用 S2D 后 DDR 带宽节省比例 | §5.2 / §7.2 | contributions.md §8.1 #3 | 跑对照 run，统计 IDMA 读次数 |
| 4 | Wall_us 端到端实测数字 | §7.4 | contributions.md §8.1 #5 | 重跑回归取 Wall_us 列 |
| 5 | 单层 PE 利用率 vs 整网 86.6% MAC% 关系论证 | §5 首段 / §7.4 | contributions.md §8.1 #6 | 文字论证 + 表脚注 |
| 6 | parf BRAM 数量贡献单独估算 | §4.4 / §7.3 | contributions.md §8.1 #7 | 查综合报告 cell utilization |
| 7 | ResNet 11-layer multicore N=2/4 实测 cycles vs 估算 | §6.4 / §7.5 | contributions.md §8.1 #8 + STATUS line 254 | 估 1-2 天 ResNet 多核适配后跑 |
| 8 | N=4 wslice1 1.45× speedup 是否近线性的 baseline 归一化 | §7.5 | contributions.md §8.1 #9 | 与 N=1/N=2 对比时分子分母统一 |
| 9 | 各 baseline 整网 MAC% (Snowflake 91%/Liu 97% 已知；Angel-Eye / Aydonat / Lu / VTA 未知) | §7.6 | contributions.md §8.1 #10 | reviewer 阶段查原文 |
| 10 | 同器件 (XC7K325T) 其他工作 Fmax | §7.6 | contributions.md §8.1 #11 | reviewer 阶段查原文 |
| 11 | Angel-Eye 是否原生支持 SDP residual fusion | §6.3 / §7.6 | contributions.md §8.1 #12 | 查 Guo TCAD'18 |
| 12 | 同器件 Angel-Eye / DPU / VTA 资源占用 | §7.3 / §7.6 | contributions.md §8.1 #13 | reviewer 阶段查 |
| 13 | S2D 在加速器领域的引用谱系（reviewer 风险） | §5.4 / §3.3 | contributions.md §8.1 #15 + literature.md §D | reviewer 阶段在 ASPLOS / HPCA / MICRO 中补查 architecture-side S2D |

### [TBD]：需要用户决定的事项

> 全文 grep `[TBD:]` 共 **15 处**（含元信息处目标会议/期刊 + 篇幅占位 + Phase 3 起草前可决定的 sub-section 取舍 + 章节内嵌 [TBD]）。下表列 10 项独立决策事项（已合并意义重复项）。

| # | 项 | 影响章节 | 备注 |
|---|----|---------|------|
| 1 | 目标会议（FPGA / FCCM / TCAD / TVLSI / TRETS / ASPLOS / MICRO） | 全文篇幅 + 风格 | contributions.md §8.2 #1；建议 FCCM 优先 |
| 2 | 论文标题最终选择（候选 1/2/3） | 全文 | 本大纲三候选，建议候选 2 |
| 3 | 写作时是否同步把 mac_pe `(* use_dsp = "yes" *)` 加上重综合 | §7.3 / §7.4 数据 | contributions.md §8.2 #4；省 17K LUT |
| 4 | 是否同步把 SDP 流水线化重综合更新 Fmax | §7.3 / §7.4 / §7.6 | contributions.md §8.2 #5；100+ MHz 拉到 |
| 5 | 是否单列 axi_dm IP 集成方案（升格为独立小节）| §6.2 节内 vs 升格 | contributions.md §8.2 #6 |
| 6 | PE-fold 是否单独成章（已选成章 §V）vs 仅作为 §6 一节 | §V 章节归属 | contributions.md §8.2 #7；本大纲已选成章 |
| 7 | §6.4 Multi-core 是否升格为独立章节（narrative B 比重） | §6.4 节内 vs §VII 章 | 当前选 §6 节内；用户可决定升格 |
| 8 | §6.5 params.py 是否值得占节位 | §6.5 节位 | 当前选独立节；篇幅紧时合并 §6.2 末尾 1 段 |
| 9 | §3.5 Quantization fusion 单独成节 vs 并入 §6.3 SDP | §3 篇幅 | 当前选 §3.5 简短 framing + §6.3 详细 |
| 10 | FLUX_CNN 是否开源 | 全文 ethos + §7.1 statement | contributions.md §8.2 #3；narrative D 高度依赖 |

### 其他

- **fps 口径**（100 MHz target 168 fps vs 68.4 MHz Fmax 115 fps）：[CHECK→TBD] 取决于 [TBD #4] 是否同步综合更新 Fmax 数据；本大纲建议全文统一用 Fmax 实测口径并标注修复路径在 §7.7 / §VIII Future Work
- **vendor doc 引用方式**（NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report 5 处）：[TBD] 由主 Agent 在 Phase 3 sub-section 起草前决定（contributions.md §8.1 #16 + literature.md §C/§E）

---

## 附录：与 Phase 0/1 输出的追溯关系

- **literature.md 35 篇** → **outline 文献-章节映射** 已对位 14 篇核心 prior art；剩余 21 篇分布在 §3.1-§3.5 各节作为 framing 引用
- **contributions.md 17 条 C**（实际 grep 数；主 Agent 提示词写 18 条系 off-by-one）→ **outline 贡献-章节映射** 已 100% 归位（其中 C3.6 在 §6.5 单节或合并 §6.2 末尾，二选一）
- **contributions.md §5.3 三处 prior art 威胁** → **outline §3.4 + §7.6** 全部精细差异化
- **contributions.md §7.1 推荐 narrative**：A 主轴 + B 支撑（混合）→ **outline 已显式陈述**并给出取舍理由
- **contributions.md §8.1 16 项 [CHECK] / §8.2 7 项 [TBD]** → **outline 待决清单 13+10 项**全部对位（部分合并）
- **lessons-learned.md 第 8 条 "[CHECK] 总数声明禁分类裁剪"** → **outline 待决清单分类计数 + 全文 grep 出 [CHECK]/[TBD] 出现位置一一对应**
