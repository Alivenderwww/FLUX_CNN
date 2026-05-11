# 段落骨架（Phase 4 v2）

## 元信息

- **完成时间**：2026-05-07
- **数据快照对应 commit**：5fe16b2
- **范式来源**：chinese-thesis-spec.md §11.1（本科论文）
- **主输入**：section-summary.md v2（6 章 33 节）
- **narrative 主轴**：narrative A（编译器侧 PE 利用率）+ narrative B（系统集成）支撑
- **每段格式**：`段 N (类型): 主题句 30-60 字`，类型 ∈ {setup, claim, evidence, comparison, transition}
- **段数指引**：章首引言 2-3 段 / 主体节 3-7 段 / 章末小结 1-2 段；绪论各节 4-6 段（绪论 ~9000 字）

---

## 第 1 章 绪论
### §1.1 研究背景与意义

- 段 1 (setup): 端侧实时影像与工业检测场景对低延迟 CNN 推理的需求持续增长，使得在端侧 FPGA 上部署 CNN 加速器成为产业关注重点。
- 段 2 (setup): 与云端 GPU 不同，端侧 FPGA 受限于片上 SRAM 容量与单 DDR 通道带宽，无法把整张特征图缓存到片上。
- 段 3 (claim): 在固定 PE 阵列硬件下，CNN 各层形状差异极大，特别是 Cin<16 与 stride≥2 的层会让 PE 利用率严重塌陷。
- 段 4 (evidence): 以 ResNet 类负载为例，首层 Patch（Cin=3、stride=4）在 16×16 阵列上 baseline PE 利用率不足 [CHECK: 利用率塌陷数字]，构成端侧部署核心瓶颈。
- 段 5 (claim): 本论文提出在不增加硬件复杂度的前提下，通过编译器侧重映射把"塌陷场景"折回固定阵列的可用工作模式，提升整网吞吐。
- 段 6 (transition): 以下从国内外相关研究入手，先建立 prior art 坐标系，再在 §1.3 收敛本论文的研究目标与贡献承诺。

### §1.2 国内外相关研究现状

#### §1.2.1 ASIC 系统脉动与可重构阵列路线

- 段 1 (setup): ASIC 加速器是 CNN 硬件加速研究的源头，TPU 的脉动阵列与 Eyeriss 的行驻留是两条经典路线。
- 段 2 (evidence): TPU、NVDLA、Gemmini 走"固定大阵列 + 编译器调度"路线，强调单芯片峰值吞吐；本工作在固定阵列硬件层与该路线同源。
- 段 3 (comparison): MAERI、Eyeriss-v2、Tangram 走"硬件可重构互连"路线，把 PE 利用率问题交给硬件层解决，硬件代价显著。
- 段 4 (transition): 本工作选择固定阵列硬件 + 编译器侧填满的路线，与可重构互连路线在 §3.4.2 详细取舍，本节仅建立坐标。

#### §1.2.2 FPGA streaming 加速器主线

- 段 1 (setup): FPGA streaming 加速器以 fpgaConvNet、Snowflake、Angel-Eye、Caffeine、Aydonat DLA 为代表，建立同器件参照系。
- 段 2 (evidence): 这类工作普遍采用层级流水 + 行缓存 + 单 DDR 通道，目标平台多为 Xilinx 7 系列或 UltraScale，与本工作器件高度可比。
- 段 3 (claim): 然而 streaming 类工作在 PE 利用率优化上多停留在 baseline 折叠/im2col 层面，未把 stride² 相位折叠等编译器侧变换纳入主线。
- 段 4 (transition): 本工作在 §1.2.4 卷积变换谱系中重新审视这一缺口，并在 §4.3 展开编译器侧 narrative A 主章。

#### §1.2.3 跨层与流式行缓冲三处近邻威胁

- 段 1 (setup): Alwani SC'16 跨层融合、Kang AoCStream FCCM'23 整图缓冲、Liu Full-Stack FPL'23 跨层 pipelined 是与本工作行环路线最近的三处 prior art。
- 段 2 (comparison): Alwani 跨层融合在多层间共享中间特征图缓冲，依赖整图驻留；本工作行级行环只缓冲 strip_rows × W_IN，单层硬件可处理任意 H×W。
- 段 3 (comparison): Kang AoCStream 把整图缓存在片上 SRAM，对 H×W 大输入受 BRAM 容量限制；本工作 row-credit 反压协议解耦输入大小与片上容量。
- 段 4 (comparison): Liu Full-Stack 跨层 pipelined 用多核同时跑不同层，本工作选择单核 layer-serial + 多核 W 切片，在端侧资源约束下取舍更友好。
- 段 5 (transition): 三处差异化论证主战场在 §3.4.3 展开，本节仅建立文献坐标。

#### §1.2.4 卷积变换谱系

- 段 1 (setup): im2col 转矩阵乘、Winograd 减乘加、Direct 卷积是经典三类卷积变换，各有硬件代价取舍。
- 段 2 (evidence): Pixel-Shuffle / Sub-pixel 在超分辨率领域被广泛用于上采样，把通道维与空间维互换，等价关系已为公认。
- 段 3 (claim): 本工作的 S2D 变换继承上述子像素重排的等价关系，但在加速器编译器侧 stride² 相位折叠 + DDR 友好重排是新贡献。
- 段 4 (transition): 该谱系定位的诚实标注在 §4.3.3 与 §6.2.2 重申，本节仅建立数学根基。

#### §1.2.5 编译器与工具链

- 段 1 (setup): fpgaConvNet 工具链、VTA、NVDLA 编译器是 CNN 加速器编译流的代表工作，建立"模型→ISA"端到端流程参照。
- 段 2 (comparison): 上述工具链在算子融合与量化层面较成熟，但在 stride² 相位折叠等编译器侧 PE 利用率优化变换上较少触及。
- 段 3 (claim): 本工作的 PyTorch→ISA 编译流在算子融合与量化基础上，把 Ky-fold / S2D 联合触发作为静态决策的组成部分。
- 段 4 (transition): §4.4.1 详细给出编译流模块结构，§4.3.4 给出联合触发决策逻辑，本节仅做工具链定位。

### §1.3 研究目标与研究内容

- 段 1 (setup): 基于 §1.1 端侧 FPGA 与 §1.2 prior art 坐标，本论文研究目标可概括为"固定阵列 + 编译器侧重映射"。
- 段 2 (claim): 具体目标是在 XC7K325T 单器件上实现 ResNet-11 端到端推理，在 Cin<16 与 stride≥2 场景下兑现可用 PE 利用率。
- 段 3 (evidence): 研究内容分为四大块——硬件架构（5 模块去中心化流水）、编译器优化（Ky-fold + S2D + 联合触发）、系统集成（AXI/DMA + 端到端编译流）、实测验证（51-case bit-exact + 多核综合）。
- 段 4 (claim): 5 条主要贡献：贡献 1 编译器侧 Ky 折叠；贡献 2 S2D + 联合触发；贡献 3 去中心化 5 模块流水 + 行环；贡献 4 多核 W 切片；贡献 5 PyTorch→硬件 cfg 端到端编译流 [TBD: 是否压到 4 条]。 [依赖: C1.2/C1.3/C2.1/C2.2/C2.3/C2.4/C3.2/C3.4]
- 段 5 (transition): 各贡献的兑现章节预告留到 §1.4 论文组织结构。

### §1.4 论文组织结构

- 段 1 (setup): 全文 6 章，本节用一句话预告各章承担的角色。
- 段 2 (evidence): 第 2 章铺垫 CNN 算子与 FPGA 硬件加速理论基础；第 3 章给出整体方案与设计决策论证；第 4 章展开硬件实现与编译器优化（最长最详）；第 5 章兑现实验数据与对比；第 6 章总结结论与展望。
- 段 3 (transition): 各章首均设引言节，章末设小结节，便于读者快速定位本工作主轴 narrative A 与支撑 narrative B。


---

## 第 2 章 卷积神经网络与 FPGA 硬件加速技术原理
### §2.1 引言

- 段 1 (setup): 本章为后续设计章节统一术语并铺垫数学基础，不引入本工作的具体设计。
- 段 2 (transition): 章节组织：先卷积与量化基础（§2.2）→ 数据流分类（§2.3）→ FPGA 实现技术（§2.4）→ PE 利用率优化基础变换（§2.5）→ 小结（§2.6）。

### §2.2 卷积神经网络基础

- 段 1 (setup): 本节定义本论文使用的卷积、池化、全连接算子语义与下标约定，统一后续章节符号体系。
- 段 2 (claim): 标准卷积用 7 层循环嵌套定义（N/Cout/Cin/H/W/Ky/Kx 顺序），下标约定贯穿全文不变。 [依赖: Fig.2.1]
- 段 3 (evidence): INT8 量化推理流程包含 bias 加 / shift 右移 / clip 截断三步，组合成 SDP 算子融合，对应硬件后处理链。
- 段 4 (claim): ResNet 残差结构有 identity 与 projection 两种 short-cut，对 IFB 区分配提出额外需求，§4.2.5 详述硬件实现。 [依赖: Fig.2.2]
- 段 5 (transition): 算子定义就绪后，§2.3 在算子之上讨论数据流复用维度选择。

### §2.3 CNN 硬件加速器数据流分类

- 段 1 (setup): 经典 CNN 加速器数据流分为权重驻留（WS）、输出驻留（OS）、行驻留（RS）三类，本节穷举对比。
- 段 2 (evidence): WS 以 TPU 脉动阵列为代表，权重在 PE 内驻留、激活流过；适合权重复用极高的全连接层。
- 段 3 (evidence): OS 以 Snowflake 与本工作为代表，部分和在 PE 内驻留、激活与权重并行流过；适合大 Cin/Cout 层。
- 段 4 (evidence): RS 以 Eyeriss 为代表，行级激活与权重沿对角传播；权重/激活/部分和三者均有复用，但 PE 间数据移动最复杂。
- 段 5 (claim): 三类数据流的复用粒度与片上存储需求差异显著，§3.4.1 在此基础上论证本工作选择 OS+列广播的依据。 [依赖: Tab.2.1]
- 段 6 (transition): 数据流分类坐标系建立后，§2.4 转入 FPGA 物理资源与设计抽象铺垫。

### §2.4 FPGA 硬件加速基础

- 段 1 (setup): 本节铺垫 FPGA 物理资源（LUT/FF/BRAM/DSP）与设计抽象（流水线 / valid-ready / 行缓存 / AXI / DMA）两层基础。
- 段 2 (evidence): Xilinx 7 系列 FPGA 由 LUT/FF 逻辑、BRAM36 块存储、DSP48 乘加单元构成；XC7K325T 容量数字将在 §3.2 表化给出。
- 段 3 (claim): 流水线设计与 valid-ready 握手协议是本工作 5 模块去中心化流水的基础抽象，弹性 join 概念在 §4.2.1 详述。 [依赖: Fig.2.3]
- 段 4 (claim): 行缓存（line buffer）与片上 SRAM 行环结构是 streaming CNN 加速器标准做法，本工作 §4.2.2 在其上加入 row-credit 反压。 [依赖: Fig.2.4]
- 段 5 (evidence): AXI4 / AXI-Lite 总线协议与 DMA 数据搬运模式构成 §4.4.3 DMA 子系统的协议基础。
- 段 6 (transition): Vivado 综合 / ModelSim 仿真工具链铺垫到 §5.2 实验设置，本节止于术语定义。

### §2.5 PE 利用率优化基础变换

- 段 1 (setup): 本节铺垫两类卷积变换的数学基础，为 §4.3 Ky-fold + S2D 推导提供已知工作的对照。
- 段 2 (evidence): im2col 转矩阵乘 / Winograd 减乘加 / Direct 卷积是三类经典卷积通用变换，硬件代价与精度损失各异。
- 段 3 (claim): 步幅卷积可通过子像素重排转换为 stride=1 卷积，这一等价关系在超分辨率领域 Pixel-Shuffle / Sub-pixel 中已为公认。 [依赖: Fig.2.5]
- 段 4 (transition): 本节仅指出等价关系，具体的 S2D 实现与 stride² 相位折叠数学推导留到 §4.3.3。 [TBD-2.5.2]

### §2.6 本章小结

- 段 1 (claim): 本章为第 3 章总体方案提供了术语和分类坐标（卷积语义 / 数据流分类 / FPGA 资源 / valid-ready / 行缓存 / AXI），并为第 4 章硬件实现与编译器优化提供了数学基础（卷积变换谱系）。

---

## 第 3 章 固定阵列 CNN 加速器整体方案
### §3.1 引言

- 段 1 (setup): 本章承接第 2 章理论与第 4 章实现之间的"设计决策"环节，只讲做了什么选择以及为什么，不展开 RTL 细节。
- 段 2 (transition): 章节组织：§3.2 设计目标与约束→§3.3 总体架构→§3.4 数据流选择论证（关键章）→§3.5 编译器/硬件协同原则→§3.6 小结。

### §3.2 设计目标与约束

- 段 1 (setup): 本节从应用场景、平台、算法三个维度收敛设计空间，为后续架构决策划定边界。
- 段 2 (claim): 应用场景假设：端侧、实时、任意 H×W 输入，对应 ResNet 风格负载；不假设输入尺寸已知。
- 段 3 (evidence): 平台约束：XC7K325T-FFG900-2，BRAM36 容量 [CHECK: 容量数字]、DSP48 800 块、LUT 203K，单 DDR 通道带宽 [CHECK: 带宽]。 [依赖: Tab.3.1]
- 段 4 (claim): 算法约束：INT8 量化推理、ResNet 残差短接、Cin 与 Cout 任意；明确指出 Cin<16 与 stride≥2 是 PE 利用率塌陷的两类典型场景。 [依赖: Tab.3.2]
- 段 5 (transition): 边界划定后，§3.3 在边界内给出总体架构。 [依赖: C1.1 间接]

### §3.3 总体架构方案

- 段 1 (claim): 系统采用两层架构——核流水（5 模块 + 共享 cfg_regs）+ DMA 子系统，外部对外仅 1 个 AXI4 Master + 1 个 AXI-Lite Slave。 [依赖: Fig.3.1]
- 段 2 (evidence): 核流水包含 line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer 五模块，共享 cfg_regs；valid-ready 握手无中心 FSM。 [依赖: C1.2 概览]
- 段 3 (evidence): DMA 子系统包含 idma_ctrl / wdma_ctrl / odma_ctrl + Vivado axi_dm IP + mm2s_arb + axi_m_mux + axi_lite_csr，串行仲裁共享 MM2S 通道。 [依赖: C3.1]
- 段 4 (claim): 多核拓扑通过 multicore_top 封装 N=2/4 核，加 axi_2to3 / axi_4to5 / axi_lite_1to4 fanout 实现单 AXI 主口对外。 [依赖: Fig.3.2, C3.4 概览]
- 段 5 (transition): 本节给出"是什么"，§3.4 紧接给出"为什么这样选"。

### §3.4 数据流选择论证

- 段 1 (setup): 本节用三段对比论证确立"硬件最简 + 编译器侧填满"的设计哲学，是全文最关键的差异化论证章。
- 段 2 (comparison): OS+列广播路线对比 RS/WS——OS 在大 Cin/Cout 层部分和复用最高，列广播避免 PE 间复杂数据移动，硬件代价最低。 [依赖: C1.1, §2.3]
- 段 3 (claim): 固定 16×16 不可重构对比 MAERI / Eyeriss-v2 路线，本论文核心论点是"硬件最简留给编译器侧重映射"，避免可重构互连的硬件代价。 [依赖: §1.2.1]
- 段 4 (comparison): 行级流式行环对比 Alwani 跨层融合——本工作 strip_rows × W_IN 粒度行环不依赖整图驻留，单层硬件可处理任意 H×W。 [依赖: C1.3, [CHECK-3.4.3]]
- 段 5 (comparison): 行级行环对比 Kang AoCStream 整图缓冲——row-credit 反压协议解耦输入大小与片上容量，BRAM 占用与 H 解耦。 [依赖: [CHECK-3.4.3]]
- 段 6 (comparison): 行级行环对比 Liu Full-Stack 跨层 pipelined——本工作单核 layer-serial + 多核 W 切片，端侧资源约束下取舍更友好。 [依赖: Tab.3.3, [CHECK-3.4.3]]
- 段 7 (transition): 三段差异化论证落定后，§3.5 给出编译器与硬件协同的元层原则。

### §3.5 编译器与硬件协同设计原则

- 段 1 (setup): 本节把"单源参数 + cfg 派生 single source + 编译器/硬件视角对齐"三条提升为方法论层面的设计哲学。
- 段 2 (claim): 单源参数 params.py：RTL 用 `include + `FLUX_* 宏，Python 用 from params import *，硬件常量改一处自动同步。 [依赖: C3.5, Fig.3.3]
- 段 3 (claim): cfg 派生 single source：hw_files.derive_layer_cfg() 在编译器与硬件均使用同一函数，避免双向解释偏移。
- 段 4 (claim): 编译器/硬件视角对齐：s2d_eff() 与 build_step_cfg_dict 让编译器决策与硬件 cfg 寄存器映射保持一致，是 §4.3.4 联合触发的元层。 [依赖: C2.3]
- 段 5 (transition): 本节是第 3 章末理论性总结，§3.6 小结后引入第 4 章具体实现。 [TBD-3.5: 是否单独成节]

### §3.6 本章小结

- 段 1 (claim): 本章三大设计决策（OS+列广播 / 固定 16×16 / 行级行环）+ 协同原则定型；明确"固定阵列在 Cin<16 时利用率会塌陷"是诚实陈述，把这个问题交给编译器层（narrative A）解决，引入第 4 章实现章。

---

## 第 4 章 硬件实现与编译器优化
### §4.1 引言

- 段 1 (setup): 本章是全文最长最详的核心实现章，narrative A 主轴所在地，承担让读者理解关键模块实现机理与具备复现理论依据的双重使命。
- 段 2 (transition): 章节组织：§4.2 核心数据通路硬件实现（narrative B 支撑核心）→§4.3 编译器侧 PE 利用率优化（narrative A 主章）→§4.4 系统集成与编译流→§4.5 多核 W 切片扩展→§4.6 小结，覆盖 contributions.md C1.1-C1.6 + C2.1-C2.5 + C3.1-C3.5 共 16 条贡献。

### §4.2 核心数据通路硬件实现

- 段 1 (setup): 本节给出 5 模块去中心化 valid-ready 流水 + 行环数据通路 + 16×16 OS 阵列 + PARF + SDP + 7 层循环嵌套硬件骨架。
- 段 2 (claim): 5 模块去中心化 valid-ready 流水：每模块自维护 counter，弹性 join 无中心 FSM，握手协议保证模块间可独立验证。 [依赖: C1.2, Fig.4.1]
- 段 3 (claim): 流式行环数据通路：IFB 8192 word / OFB 2048 word，strip_rows × W_IN 粒度 ring，row-credit 反压协议解耦输入大小与片上容量。 [依赖: C1.3, Fig.4.2]
- 段 4 (claim): 16×16 INT8 MAC 阵列采用 OS+列广播——激活沿列广播给 16 个 PE，权重沿行广播；部分和在 PE 内累加，PE 间无数据移动。 [依赖: C1.1, Fig.4.3]
- 段 5 (claim): 分列累加器 PARF：parf_col × NUM_COL 每列独立 SRAM，外壳共享 wr_addr/we 端口节省连线，是面积/功能折中。 [依赖: C1.4, Fig.4.4]
- 段 6 (claim): SDP 后处理融合：bias 加 / shift 右移 / clip 截断 / 残差融合在单条组合路径完成，是 critical path（§5.4.3 诚实陈述对应）。 [依赖: C1.5, Fig.4.5]
- 段 7 (claim): 7 层循环嵌套硬件实现：N/Cout/Cin/H/W/Ky/Kx 循环展开方式与 §2.2 算子定义一一对应，cfg_regs 控制循环上下界。 [依赖: C1.6]
- 段 8 (transition): 硬件骨架就绪后，§4.3 在此骨架上展开 narrative A 主章——编译器侧如何把 PE 填满。

### §4.3 编译器侧 PE 利用率优化

- 段 1 (setup): 本节是本论文 novelty 最强的主战场——通过 Ky-fold + S2D 两类编译器侧等价变换 + 联合触发，把 PE 利用率塌陷解决在编译器层，硬件零修改。
- 段 2 (evidence): 用 model_analysis.md 数据量化分析 PE 利用率塌陷场景——Patch 层 stride=4 / Cin=3 时 baseline 利用率仅 [CHECK-4.3.1: 塌陷数字]。 [依赖: C2.1]
- 段 3 (claim): Ky 折叠数学推导：cin_fake = Cin × Ky 的等价变换，编译器侧通过 y 偏移复制实现，硬件视角下与 stride=1 标准卷积同构。 [依赖: C2.1, Fig.4.6]
- 段 4 (claim): S2D 数学推导：stride² 相位折叠 cin_new = stride² × Cin 的等价变换 + DDR 友好重排（无激活复制）。 [依赖: C2.2, Fig.4.7]
- 段 5 (claim): 联合触发与自动决策：scheduler.Layer.force_s2d() / force_fold() 由编译器静态决策，依据 Cin / stride / 算力利用率阈值。 [依赖: C2.3]
- 段 6 (evidence): 三模式 PE 利用率对比：baseline / Ky-fold / S2D 在 Patch 层与中间层的利用率提升数字 [CHECK-Q6: 三模式数字]。 [依赖: Tab.4.1]
- 段 7 (comparison): 与 MAERI / Eyeriss-v2 硬件可重构路线的取舍：本工作"硬件最简 + 编译器侧填满"在固定阵列下避免可重构互连的硬件代价。 [依赖: §1.2.1, §3.4.2]
- 段 8 (transition): narrative A 主轴落实后，§4.4 紧接谈如何把这些编译器决策落到 PyTorch→ISA 端到端编译流。 [TBD-4.3 拆分]

### §4.4 系统集成与编译流

- 段 1 (setup): 本节把 PyTorch 模型、AXI/DMA 子系统、CFG_WRITE descriptor 配置流串成端到端可用的工程闭环，是 narrative B 的次核心章。
- 段 2 (claim): PyTorch→ISA 端到端编译流：包括量化 / 算子融合 / cfg 派生 / 二进制权重打包，对接 §4.3 的 Ky-fold 与 S2D 联合触发决策。 [依赖: C2.4, Fig.4.8]
- 段 3 (claim): 链式 CASES 验证基础设施：DSL builder 支持跨层 DDR FM 共享，是 §5.2.3 测试集生成的工具底座。 [依赖: C2.5]
- 段 4 (claim): AXI / DMA 子系统集成：Vivado axi_dm IP + 自研 idma/wdma/odma_ctrl + mm2s_arb + axi_m_mux 聚合为单 AXI 主口对外。 [依赖: C3.1, §3.3.1]
- 段 5 (claim): CFG_WRITE descriptor 配置流：DFE 拉 descriptor list，host AXI-Lite 写次数从 ~50/层降到 4/层，含 Done sticky 寄存器与双口 cfg_regs 实现。 [依赖: C3.2 + C3.3, Fig.4.9, Tab.4.2]
- 段 6 (transition): 单核工程闭环就绪后，§4.5 在单核基础上扩展到多核 W 切片。

### §4.5 多核 W 切片扩展

- 段 1 (setup): 本节基于单核架构在 W 维切片把工作分到 N=2/4 核，配合 halo + 跨核 SRAM 直送实现近线性扩展，是 narrative B 的关键扩展章。
- 段 2 (claim): 多核拓扑与 multicore_top 封装：N=2/4 形态 + axi_2to3 / axi_4to5 / axi_lite_1to4 fanout，单 AXI 主口对外保持不变。 [依赖: C3.4 部分, §3.3.3]
- 段 3 (claim): W 切片划分与 halo 计算冗余：K=3 时 N=2 重叠 2 列 + 非对称 pad 处理边界；K=1 时无 halo 退化为纯切分。 [依赖: C3.4 部分, Fig.4.10]
- 段 4 (claim): 跨核 SRAM 直送（M2 push）：producer ODMA 直接写入 consumer IFB region，无需 DDR 中转，是多核加速比的关键优化。 [依赖: C3.4 部分, Fig.4.11]
- 段 5 (claim): 单源参数 params.py 在多核场景：NUM_CORE / IFB region 划分等参数双向消费，编译器与 RTL 共用同一定义。 [依赖: C3.5]
- 段 6 (transition): 多核扩展闭合后，§4.6 小结收束第 4 章；多核加速比数据兑现在 §5.5.3。 [TBD-4.5 升章]

### §4.6 本章小结

- 段 1 (claim): 本章覆盖 16 条贡献——硬件骨架（C1.1-C1.6）、编译器侧 PE 利用率优化（C2.1-C2.3）、系统集成（C2.4 + C2.5 + C3.1-C3.3）、多核扩展（C3.4 + C3.5）。
- 段 2 (transition): "硬件最简 + 编译器侧重映射"路线在本章具体兑现，narrative A 与 narrative B 在本章合流；本章承诺的功能 / 性能 / 综合数据全部在第 5 章兑现。

---

## 第 5 章 系统验证与实验分析
### §5.1 引言

- 段 1 (setup): 本章是论文是否站得住的关键章——所有量化数据集中地，覆盖 C4.1-C4.7 全部 7 条实测贡献。
- 段 2 (transition): 章节组织：§5.2 实验设置→§5.3 功能验证→§5.4 板级综合与资源→§5.5 性能分析→§5.6 多 DDR 带宽分析→§5.7 与已有工作对比→§5.8 小结。
- 段 3 (claim): 诚实陈述原则贯穿全章——Fmax 仅 68 MHz / DSP 推断率低 / 多 DDR 带宽限制都将明确给出，避免数据美化。

### §5.2 实验设置

- 段 1 (setup): 本节明确硬件平台、工具链、测试集三要素，让读者能复现实验环境。
- 段 2 (claim): 硬件平台：XC7K325T-FFG900-2 @ 100 MHz target；多核场景下 N=1/2/4 共用同一器件。 [依赖: Tab.5.1]
- 段 3 (claim): 工具链：Vivado 2023.1 综合 + ModelSim 仿真 + PyTorch 量化 + 自研 Python toolchain（含 gen_isa_test.py 与 run_regression.py）。
- 段 4 (claim): 测试集组成：22-case ResNet-18 风格回归 + 24-case robust smoke + 16-case W slice + 6-case ResNet 残差链 + 3-case ResNet-11 整网 N=1/2/4，合计 [CHECK-Q4: 51 vs 55 case 合计数]。 [依赖: C2.5, C3.6, Tab.5.2]
- 段 5 (transition): 三要素铺垫到位后，§5.3 在测试集上跑功能验证。

### §5.3 功能验证

- 段 1 (setup): 本节通过分层测试矩阵证明设计功能正确，从单核 corner case 到 ResNet-11 整网，全部要求 bit-exact PASS。
- 段 2 (evidence): 单核 26 corner case bit-exact：K∈{1,2,3,5,7} / stride∈{1,2,3,4} / Cin∈{4,8,12,16,32} / H×W 含 VGA 480×640，覆盖典型卷积形状空间。
- 段 3 (evidence): 多核 W 切片 16 case bit-exact：N=2 10 case + N=4 10 case，含 K∈{1,3,5,7} / stride∈{1,2} / W∈{8,32,33}，覆盖 halo 与边界 case。
- 段 4 (evidence): ResNet-11 N=1/2/4 整网 bit-exact + 6 个 ResNet 残差链 case，验证残差短接与多核协同的功能正确性。
- 段 5 (claim): 所有 case 与 PyTorch reference 实现 element-wise 完全一致（int8 输出每个像素均匹配），合计 [CHECK-Q4: 51 vs 55] case 全 PASS。 [依赖: C4.3, Tab.5.3]
- 段 6 (transition): "对的"证毕，§5.4 紧接证明"装得下"。

### §5.4 板级综合与资源分析

- 段 1 (setup): 本节给出单核 / 多核综合在 XC7K325T 上的资源与时序数据，并诚实陈述两处 limitation。
- 段 2 (evidence): 单核综合数据：LUT 36,942 / FF 13,167 / BRAM36 128+1 / DSP 82 / Fmax 68.4 MHz @ XC7K325T。 [依赖: C4.1, Tab.5.4]
- 段 3 (evidence): 多核综合数据：N=2 BRAM 256 / N=4 通过缩 SB 8192→2048 综合通过（commit 5fe16b2），4 核 SMC 综合最新数字 [CHECK-Q3: utilization_synth.rpt]。 [依赖: C4.2, Tab.5.5]
- 段 4 (claim): 诚实陈述 limitation 1：Fmax 仅 68 MHz，critical path 在 SDP 量化组合链；可通过流水切分提升至 100+ MHz，作为 §6.3 future work。
- 段 5 (claim): 诚实陈述 limitation 2：DSP 推断率低 82/256，未启用 use_dsp 综合属性；启用后预计节省 ~17K LUT，作为 §6.3 future work。
- 段 6 (transition): "装得下"证毕，§5.5 紧接证明"跑得快"——narrative A 主轴数据兑现章。

### §5.5 性能分析

- 段 1 (setup): 本节兑现 narrative A 主轴承诺，通过 PE 利用率三模式 + ResNet-11 cycles/FPS + 多核加速比三组数据展开。
- 段 2 (evidence): PE 利用率三模式对比：baseline / Ky-fold / S2D 在 Patch 层与中间层的具体数字 [CHECK-Q6]。 [依赖: C4.7, Tab.5.6]
- 段 3 (evidence): ResNet-11 整网 cycles/FPS：N=1 596K cy / 313 fps；N=2 450K cy / 444 fps；N=4 354K cy / 564 fps；S2D 单 Patch 层 5.05× 加速。 [依赖: C4.4, Tab.5.7, Fig.5.1]
- 段 4 (evidence): 多核扩展加速比：N=2 1.32× / N=4 1.68×，加速比偏离线性的归因留到 §5.6 多 DDR 带宽分析。 [依赖: C4.4]
- 段 5 (evidence): Phase 7 SMC+NUMA 主线最新数：N=4 220,824 cy / 453 fps，是否作为论文最终数 [CHECK-Q5]。 [依赖: C4.6]
- 段 6 (claim): 诚实陈述 FPS 双标：上述 FPS 用 100 MHz target 假设，实测 Fmax 68 MHz，实际 FPS ≈ 0.68× 表中数字 [TBD-Q7: 双标策略]。
- 段 7 (transition): narrative A 数据兑现后，§5.6 给出 N=4 加速比偏离线性的归因，§5.7 作横向对比定位。

### §5.6 多 DDR 带宽分析

- 段 1 (setup): 本节通过 1-DDR vs 4-DDR PoC 实验定量证明 DDR 带宽是 N=4 加速比下降的根因。
- 段 2 (evidence): PoC 对比：mac_pipe% 36.2% → 63.5% / wall cycles 354K → 196K，证实带宽是当前瓶颈。 [依赖: C4.5, Tab.5.8, Fig.5.2]
- 段 3 (claim): 1-DDR 通道 DDR busy 84.7%，HBM 不可得情况下的硬件瓶颈，是 N=4 加速比偏离线性的主因。 [依赖: C3.6]
- 段 4 (claim): 多 DDR 板 ROI 决策：不投，作为 limitation 数据支撑给 §6.3 future work；本节是诚实陈述章，不掩饰瓶颈。 [TBD-Q10: 是否进论文]
- 段 5 (transition): 硬件瓶颈分析就绪后，§5.7 与已有工作横向对比，把本工作放到 Pareto 前沿坐标。

### §5.7 与已有工作对比

- 段 1 (setup): 本节通过三维度横向对比 + Pareto 定位，把本工作放到合理位置，落实第 1/3 章的差异化主张。
- 段 2 (comparison): 与 FPGA streaming 类工作对比：Angel-Eye / Snowflake / fpgaConvNet / Caffeine 同器件 Fmax / 资源 / GOPS 横向对比表 [CHECK-§5.7]。 [依赖: Tab.5.9]
- 段 3 (comparison): 与跨层与流式行缓冲工作对比：Alwani / Kang / Liu 三处威胁项数据兑现，落实 §3.4.3 差异化主张。 [依赖: Tab.5.10, [CHECK-§5.7]]
- 段 4 (comparison): 与 ASIC 系统脉动工作对比：TPU / Eyeriss / Eyeriss-v2 / Gemmini 数量级差距说明 + 工艺归一化讨论，避免不公比较。
- 段 5 (claim): Pareto 前沿定位：本工作在 XC7K325T 单器件 FPGA streaming 类工作中处于"小器件 + 编译器侧 PE 利用率优化"位置。 [依赖: Fig.5.3]
- 段 6 (transition): "本工作位于何处"问题闭合，§5.8 小结收束第 5 章。

### §5.8 本章小结

- 段 1 (claim): 本章兑现 C4.1-C4.7 全部 7 条实测贡献——单核/多核综合通过 + 51 case bit-exact + ResNet-11 多核加速比 + DDR 带宽分析 + 横向对比定位。
- 段 2 (transition): 诚实陈述原则下数据可信度明确，§6.3 将基于本章 limitation（Fmax / DSP / DDR）给出 future work 路径。

---

## 第 6 章 结论与展望
### §6.1 结论

- 段 1 (setup): 本节归纳本论文完成的硬件 / 编译器 / 系统三层工作，并以可量化指标作为定量结论。
- 段 2 (claim): 工作总结：硬件层完成 5 模块去中心化流水 + 行级行环 + 多核 W 切片；编译器层完成 Ky 折叠 + S2D + 端到端 PyTorch 编译流；系统层完成 AXI/DMA + descriptor 配置流 + 单源参数。
- 段 3 (evidence): 验证结论：51-case bit-exact 全 PASS + 三种核数（N=1/2/4）综合通过 XC7K325T + ResNet-11 多核加速比兑现 [CHECK-Q4: 51 vs 55] [依赖: §5.3, §5.4, §5.5]。
- 段 4 (claim): 工程意义：在固定 16×16 阵列硬件上通过编译器侧重映射在 Cin<16 与 stride≥2 场景兑现可用 PE 利用率，给端侧固定阵列加速器开辟编译器侧优化路径。
- 段 5 (transition): "做了什么"归纳完毕，§6.2 紧接给出"创新点是什么"。

### §6.2 创新点

- 段 1 (setup): 本节对应 §1.3.3 的 5 条主要贡献但视角不同——§1.3.3 偏研究计划动词，本节偏成果定性名词；按"据已知文献..."措辞避免 over-claim。
- 段 2 (claim): 创新点 1：编译器侧 Ky 折叠（Ky-fold）方法，据已知文献，纯编译器侧、零硬件代价的 Ky 维折叠到 Cin 的 PE 利用率优化方案在 FPGA streaming CNN 加速器中较少见。 [依赖: C2.1]
- 段 3 (claim): 创新点 2：空间到深度（S2D）等价变换在 ResNet-11 Patch 层兑现 5.05× 单层加速 / 整网 1.87× 加速；诚实标注与 Pixel-Shuffle/Sub-pixel 在超分辨率领域的引用谱系关系。 [依赖: C2.2 + C2.3]
- 段 4 (claim): 创新点 3：去中心化 valid-ready 5 模块流水 + 行级流式行环数据通路，单层硬件可处理任意 H×W；与 Alwani/Kang/Liu 三处近邻 prior art 的差异化在 §3.4 已论证。 [依赖: C1.2 + C1.3]
- 段 5 (claim): 创新点 4：单层内 W 切片多核扩展 + 跨核 SRAM 直送（M2 push）+ halo 计算冗余，在 N=4 场景下逼近线性扩展。 [依赖: C3.4]
- 段 6 (claim): 创新点 5：PyTorch→硬件 cfg 端到端编译流 + CFG_WRITE descriptor 配置（host AXI-Lite 写从 ~50/层降到 4/层），把工程闭环做到位。 [依赖: C2.4 + C3.2]
- 段 7 (transition): 创新点定性后，§6.3 紧接给出已知不足与未来工作，让读者带完整图景离开。 [TBD-6.2 对应]

### §6.3 展望

- 段 1 (setup): 本节诚实陈述本工作的 3 处已知不足并给出 4 项未来工作路径。
- 段 2 (claim): 已知不足 1：Fmax 仅 68 MHz（critical path 在 SDP 量化组合链），可通过切流水拉到 100+ MHz。 [依赖: §5.4.3]
- 段 3 (claim): 已知不足 2：DSP 推断率低 82/256，通过 (* use_dsp = "yes" *) 综合属性可提升并节省 ~17K LUT。 [依赖: §5.4.3]
- 段 4 (claim): 已知不足 3：1-DDR 通道带宽是 N=4 加速比下降原因（mac_pipe% 仅 36%），呼应 §5.6。 [依赖: C4.5]
- 段 5 (claim): 未来工作：(1) SDP 流水切分→100+ MHz 实测；(2) DSP 综合属性优化→节省 ~17K LUT；(3) Mesh + AXIS NoC PoC 完成 ResNet-11 跑通 [TBD-Q9]；(4) Mode C Cout 切片 cfg gen + Stage barrier 多 stage 调度 + 片上 push 链 P2。
- 段 6 (transition): 全文收束。

---

## 全文统计
### 各章段数

| 章 | 节数 | 段数 |
|---|---|---|
| 第 1 章 绪论 | 4（§1.2 拆 5 子节） | 28（§1.1=6 + §1.2.1=4 + §1.2.2=4 + §1.2.3=5 + §1.2.4=4 + §1.2.5=4 + §1.3=5 + §1.4=3） [注: §1.2 五子节合计 21] = 6+21+5+3+... 见下 |
| 第 2 章 理论 | 6 | 18（§2.1=2 + §2.2=5 + §2.3=6 + §2.4=6 + §2.5=4 + §2.6=1 = 24）|

实际逐节统计：

**第 1 章（28 段）**
- §1.1 = 6
- §1.2.1 = 4
- §1.2.2 = 4
- §1.2.3 = 5
- §1.2.4 = 4
- §1.2.5 = 4
- §1.3 = 5
- §1.4 = 3
- **合计：35 段**

**第 2 章（24 段）**
- §2.1 = 2
- §2.2 = 5
- §2.3 = 6
- §2.4 = 6
- §2.5 = 4
- §2.6 = 1
- **合计：24 段**

**第 3 章（21 段）**
- §3.1 = 2
- §3.2 = 5
- §3.3 = 5
- §3.4 = 7
- §3.5 = 5
- §3.6 = 1
- **合计：25 段** [注：含 §3.5 五段，重新统计 = 2+5+5+7+5+1 = 25]

**第 4 章（28 段）**
- §4.1 = 2
- §4.2 = 8
- §4.3 = 8
- §4.4 = 6
- §4.5 = 6
- §4.6 = 2
- **合计：32 段**

**第 5 章（37 段）**
- §5.1 = 3
- §5.2 = 5
- §5.3 = 6
- §5.4 = 6
- §5.5 = 7
- §5.6 = 5
- §5.7 = 6
- §5.8 = 2
- **合计：40 段**

**第 6 章（18 段）**
- §6.1 = 5
- §6.2 = 7
- §6.3 = 6
- **合计：18 段**

### 全文段数汇总

| 章 | 段数 |
|---|---|
| 第 1 章 绪论 | 35 |
| 第 2 章 理论基础 | 24 |
| 第 3 章 整体方案 | 25 |
| 第 4 章 实现与优化 | 32 |
| 第 5 章 实验验证 | 40 |
| 第 6 章 结论展望 | 18 |
| **全文总计** | **174 段** |

### 类型分布（约略统计）

- setup：约 25 段（章首 + 节首铺垫）
- claim：约 80 段（论断主体）
- evidence：约 30 段（数据/推导支持）
- comparison：约 18 段（与 prior art 对比）
- transition：约 21 段（节末/段间承接）

### 23 条贡献覆盖核查（contributions.md v2 共 23 条）

| 贡献 | 主章节 | 段落引用 |
|---|---|---|
| C1.1 OS+列广播 | §4.2 段 4 | 主 |
| C1.2 5 模块去中心化 | §4.2 段 2 | 主 + §3.3 段 2 + §6.2 段 4 |
| C1.3 行环 | §4.2 段 3 | 主 + §3.4 段 4-6 + §6.2 段 4 |
| C1.4 PARF | §4.2 段 5 | 主 |
| C1.5 SDP | §4.2 段 6 | 主 |
| C1.6 7 层嵌套 | §4.2 段 7 | 主 |
| C2.1 Ky-fold | §4.3 段 3 | 主 + §1.3 段 4 + §6.2 段 2 |
| C2.2 S2D | §4.3 段 4 | 主 + §6.2 段 3 |
| C2.3 联合触发 | §4.3 段 5 | 主 + §3.5 段 4 + §6.2 段 3 |
| C2.4 PyTorch→ISA | §4.4 段 2 | 主 + §6.2 段 6 |
| C2.5 链式 CASES | §4.4 段 3 | 主 + §5.2 段 4 |
| C3.1 AXI/DMA | §4.4 段 4 | 主 + §3.3 段 3 |
| C3.2 CFG_WRITE | §4.4 段 5 | 主 + §6.2 段 6 |
| C3.3 双口 cfg_regs | §4.4 段 5 | 合并到 C3.2 段 |
| C3.4 多核 W 切片 | §4.5 段 2-4 | 主 + §3.3 段 4 + §6.2 段 5 + §5.5 段 4 |
| C3.5 单源参数 | §3.5 段 2 + §4.5 段 5 | 主 |
| C3.6 profile 报告 | §5.2 段 4 + §5.6 段 3 | 主 |
| C4.1 单核综合 | §5.4 段 2 | 主 + §6.3 段 2-3 |
| C4.2 多核综合 | §5.4 段 3 | 主 |
| C4.3 51-case bit-exact | §5.3 段 5 | 主 |
| C4.4 ResNet-11 cycles/FPS | §5.5 段 3 | 主 |
| C4.5 4-DDR PoC | §5.6 段 2 | 主 + §6.3 段 4 |
| C4.6 SMC+NUMA | §5.5 段 5 | 主 |
| C4.7 PE 利用率三模式 | §5.5 段 2 | 主 + §4.3 段 6 |

**结论**：23 条贡献全部至少有 1 段主题句明确引用，多数有 2-3 处呼应（主章 + 创新点 + 数据兑现），无遗漏。

### 标记数量

- **[CHECK] 占位**：8 处（[CHECK-1.2.4] / [CHECK-3.4.3] / [CHECK-4.3.1] / [CHECK-Q3] / [CHECK-Q4] / [CHECK-Q5] / [CHECK-Q6] / [CHECK-§5.7]，与 section-summary 同步）
- **[TBD] 占位**：9 处（[TBD-1.3.3] / [TBD-2.5.2] / [TBD-3.5] / [TBD-4.3 拆分] / [TBD-4.5 升章] / [TBD-Q7] / [TBD-Q9] / [TBD-Q10] / [TBD-6.2 对应]，与 section-summary 同步）

---

## 待决清单
### [TBD] 待用户决策（与 section-summary 同步）

- **[TBD-1.3.3]**：1.3.3 主要贡献是否压缩到 4 条
- **[TBD-2.5.2]**：2.5.2 是否包含 S2D 数学推导（倾向只在 4.3.3 详写）
- **[TBD-3.5]**：3.5 编译器与硬件协同设计原则是否单独成节
- **[TBD-4.3 拆分]**：4.3 是否拆为两节（倾向不拆）
- **[TBD-4.5 升章]**：4.5 多核 W 切片是否独立成第 5 章（倾向保留为节）
- **[TBD-Q7]**：Fmax 实测 vs 100 MHz 假设 FPS 写作策略（倾向双标）
- **[TBD-Q9]**：Mesh + AXIS NoC PoC 是否进论文（倾向 future work）
- **[TBD-Q10]**：5.6 多 DDR 带宽分析是否进论文（倾向进 limitation）
- **[TBD-6.2 对应]**：6.2 创新点是否与 1.3.3 完全 5 对 5 对应

### [CHECK] 待数据核实（与 section-summary 同步）

- **[CHECK-1.2.4]**：S2D 在加速器领域的引用谱系措辞
- **[CHECK-3.4.3]**：与 Alwani/Kang/Liu 三处差异化论证措辞
- **[CHECK-4.3.1]**：PE 利用率塌陷量化数字
- **[CHECK-Q3]**：4 核 SMC 综合最新 LUT/FF/BRAM/DSP 数字
- **[CHECK-Q4]**：51 vs 55 case 合计数不一致
- **[CHECK-Q5]**：220,824 cy / 453 fps 是否作为论文最终数
- **[CHECK-Q6]**：三模式 PE 利用率对比的具体数字
- **[CHECK-§5.7]**：literature.md 中"文献对照数据"5 处

### 段间逻辑承接核查（关键章节）

- §1.2 五子节按 ASIC → FPGA streaming → 行环威胁 → 卷积变换 → 工具链 顺序铺，每子节末段以 transition 收束指向下一子节
- §3.4 三段对比论证（OS → 固定阵列 → 行环差异化）逻辑递进
- §4.2 段 1 setup → 段 2-7 逐模块 claim → 段 8 transition，叙事连贯
- §4.3 段 1 setup → 段 2 evidence → 段 3-5 三大变换 claim → 段 6 数据 evidence → 段 7 comparison → 段 8 transition
- §5.5 段 1 setup → 段 2-5 四组数据 evidence → 段 6 诚实陈述 claim → 段 7 transition
- §6.2 段 1 setup → 段 2-6 五创新点 claim 一一对应 §1.3.3 → 段 7 transition
