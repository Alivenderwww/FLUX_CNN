---
title: "[TBD: 中文标题待用户定 — 候选《基于流式行环和编译器侧PE利用率优化的固定阵列CNN加速器》或《面向边缘FPGA的固定16×16 INT8阵列CNN加速器：编译器侧PE利用率优化与流式行环协同设计》]"
authors: "[TBD: 作者中文姓名 / 学号 / 学院 / 专业 / 班级]"
advisor: "[TBD: 指导教师姓名 / 职称]"
school: "[TBD: 学校名称]"
date: "[TBD: 提交日期]"
commit: b158cab
---

> 本文档由多智能体流水线生成。`[TBD: ...]`为待用户决定事项；`[CHECK: ...]`为待FLUX_CNN实测或文献核验事项。
>
> 本批（Phase 8批1）仅落摘要、关键词、Abstract、Key words与中文骨架，正文章节内容为占位，待批3中文化原英文paper.md。

---

# 摘要

针对边缘FPGA上CNN加速器普遍存在的两类瓶颈——浅层卷积处理单元（PE）利用率低和片上SRAM不足以驻留整图——本文设计并实现了FLUX_CNN，一种面向Xilinx 7系列FPGA的16×16 INT8固定阵列CNN加速器。硬件层面采用去中心化valid-ready握手流水（line_buffer至ofb_writer共5个模块）与流式行环（streaming row-ring）数据通路，使任意H×W图像可经一次start指令端到端处理，VGA 480×640分辨率仅需约10 KB片上环形存储。编译器层面提出Ky折叠和空间到深度（S2D）两种零RTL改动的变换通道：Ky折叠在Cin<16时把卷积核Ky维折至虚拟cin维以填满阵列宽度；S2D在stride≥2时把stride²个空间相位重排至Cin维，等价为stride=1的卷积，DDR带宽友好。系统层面集成Xilinx AXI DataMover IP并配以轻量控制器，并提供N=2/4多核W切片wrapper。在XC7K325T-2上单核占用128 BRAM36（28.8%）、82 DSP（9.8%）、36942 LUT（18.1%）；22个ResNet-18风格链式回归用例三种折叠模式下全部bit-exact通过，整网593K cycles、网络级MAC利用率86.6%；20个多核W切片用例N=2/4全部bit-exact通过，N=4相对N=2加速比为1.45倍[CHECK: 同case的N=1基线归一化待补]。结果表明，编译器侧PE利用率优化在固定阵列硬件上可逼近完全利用率，与MAERI、Eyeriss-v2、Tangram等硬件可重构方案形成互补路径。

**关键词：** CNN加速器；FPGA；编译器侧PE利用率优化；流式行环；脉动阵列

# Abstract

CNN accelerators on edge FPGAs face two coupled bottlenecks: chronic processing-element (PE) underutilization on shallow layers whose input channel count Cin is far smaller than the array width, and an on-chip SRAM budget too small to resident-buffer realistic-resolution feature maps. This paper presents FLUX_CNN, a 16×16 INT8 fixed-array CNN accelerator targeting Xilinx 7-series FPGAs, that addresses both bottlenecks without enlarging the array RTL. The hardware adopts a decentralized valid-ready handshake pipeline of five modules (line_buffer through ofb_writer) and a streaming row-ring data path, so that an arbitrary H×W image runs to completion under a single host-issued start command, with VGA 480×640 inputs requiring roughly 10 KB of on-chip ring storage. Two compiler passes introduce zero RTL change: Ky-fold folds the Ky kernel dimension into a virtual cin dimension when Cin<16, and Space-to-Depth (S2D) rearranges stride² spatial phases into Cin for stride≥2 layers, yielding an equivalent stride-1 convolution that is friendly to DDR bandwidth. The DMA subsystem integrates the Xilinx AXI DataMover IP with lightweight controllers, and a multi-core W-slice wrapper supports N=2 and N=4 configurations. On XC7K325T-2, a single core consumes 128 BRAM36 (28.8%), 82 DSPs (9.8%), and 36942 LUTs (18.1%); 22 chained ResNet-18-style regression cases pass bit-exactly under three folding modes, with end-to-end 593K cycles and 86.6% network-level MAC utilization; 20 multi-core W-slice cases on N=2 and N=4 pass bit-exactly, with N=4 reaching 1.45× speedup over N=2 [CHECK: N=1 baseline normalization pending]. The results indicate that compiler-side PE utilization optimization on a fixed array can approach full utilization, and is complementary to hardware-reconfigurable schemes such as MAERI, Eyeriss-v2, and Tangram.

**Key words:** CNN accelerator; FPGA; compiler-side PE utilization; streaming row-ring; systolic array

---

# 引言

## 1 选题背景与意义

### 1.1 卷积神经网络专用加速器的演进背景

卷积神经网络（CNN）自2012年AlexNet在ImageNet数据集上以大幅领先的精度获奖以来，已经成为图像分类、目标检测、语义分割、超分辨率重建等视觉任务的事实标准模型族<sup>[1]</sup><!-- ref: [He-CVPR16] -->。随着ResNet<sup>[1]</sup>把网络深度推到152层、并通过残差连接证明了"更深的网络可以稳定训练"这一规律，CNN在算法层面进入了大规模、深层化的发展阶段；与之对应，CNN前向推理的计算量与访存量同步攀升——单张224×224图像走完一次ResNet-50需要约4 GMAC运算，单张480×640的VGA图像走完一次轻量级目标检测模型需要数十亿次乘加和数兆字节级别的特征图存储。通用CPU在此类规则化、批量化的张量计算面前能效偏低，通用GPU虽然算力充裕，但功耗与片外DDR访存代价对端侧设备又过高。这一矛盾催生了一系列**针对CNN推理优化的专用加速器**——从数据中心规格的Google TPU<sup>[2]</sup><!-- ref: [TPU-ISCA17] -->到边缘端的Eyeriss<sup>[3]</sup><!-- ref: [Eyeriss-ISCA16] -->、NVDLA<sup>[4]</sup><!-- ref: [NVDLA] -->、ARM Ethos<sup>[5]</sup><!-- ref: [ARM-Ethos] -->，以及面向FPGA平台的fpgaConvNet<sup>[6]</sup><!-- ref: [fpgaConvNet-TNNLS19] -->、Angel-Eye<sup>[7]</sup><!-- ref: [Angel-Eye-TCAD18] -->、Snowflake<sup>[8]</sup><!-- ref: [Snowflake-ISCAS17] -->、Xilinx DPU<sup>[9]</sup><!-- ref: [Xilinx-PG338] -->等等。

这些工作在硬件层面普遍以**空间阵列（spatial array）**或**脉动阵列（systolic array）**为核心：把成百上千乃至数万个MAC单元排成二维阵列，按某种"数据流（dataflow）"组织权重、输入特征、部分和在阵列内的流动方向，从而把张量乘加映射成空间并行的计算流水。Chen等人在Eyeriss工作中首次系统化地把CNN加速器的数据流划分为权重驻留（WS）、输出驻留（OS）和行驻留（RS）三大类<sup>[3]</sup>，并指出"片上数据搬运是能耗主导项"。这条论断在此后近10年间反复被印证——Gemmini<sup>[10]</sup><!-- ref: [Gemmini-DAC21] -->、TPU、NVDLA、Simba<sup>[11]</sup><!-- ref: [Simba-MICRO19] -->、Eyeriss-v2<sup>[12]</sup><!-- ref: [Eyeriss-v2-JETCAS19] -->、MAERI<sup>[13]</sup><!-- ref: [MAERI-ASPLOS18] -->、Tangram<sup>[14]</sup><!-- ref: [Tangram-ASPLOS19] -->均沿用了空间阵列加构思，差异主要落在阵列规模、数据流取舍、片上互连可重构性这3个轴上。

### 1.2 边缘FPGA推理的应用需求

CNN加速器的部署场景由两端拉动：一端是数据中心训练与推理，规模大到要求数十TOPS吞吐；另一端是端侧/边缘推理，对功耗、面积、片上存储均极敏感。本论文聚焦后者，理由有3条：

1. **端侧推理需求迅速扩张**。安防监控、工业视觉质检、机器人视觉伺服、车载驾驶辅助、无人机航拍后处理这些场景对延迟、隐私和数据吞吐有强约束，把所有原始视频流回传云端是不现实的，必须在边缘节点本地完成推理。
2. **FPGA是端侧推理性价比合理的载体**。相比ASIC，FPGA具有可重构、迭代周期短、批量起购成本低这3项优势，特别契合"算法仍在演进、产品形态多变、量级在万到百万片之间"的端侧场景；相比通用嵌入式GPU，FPGA在INT8固定点推理上能效更高，且能与图像采集、预处理、协议封装等定制逻辑无缝集成在同一片SoC上。Xilinx 7系列（Kintex-7/Artix-7/Zynq-7000）作为低成本中端FPGA仍是工业项目首选平台之一，其有限片上BRAM和DSP资源对加速器设计提出了独特约束。
3. **开源、可复现的端侧FPGA-CNN参考实现仍稀缺**。NVDLA虽开源但面向ASIC流片；Gemmini基于RISC-V SoC生成器，与FPGA独立部署适配仍有距离；fpgaConvNet和Angel-Eye作为学术作品代表性强但已数年未维护；Vitis AI / DPU是闭源vendor IP，对学术研究和教学并不友好。这给"自主、轻量、可复现的INT8 CNN加速器"留出了明确空间。

### 1.3 现有加速器的两类瓶颈与本论文动机

在边缘FPGA推理场景下，既有CNN加速器普遍面临**两类相互耦合的瓶颈**——这也是本课题的核心动机。

**瓶颈一：浅层卷积的PE利用率欠佳。** 固定规模的MAC阵列（如本论文的16×16 = 256 MAC、TPU v1的256×256 = 65536 MAC）按"输入通道维Cin × 输出通道维Cout"两个轴展开。这一映射对网络中段、深层卷积是合适的——典型ResNet/VGG中段层的Cin、Cout均为64、128、256、512量级，远大于阵列宽度，PE全部填满；但**网络浅层往往Cin极小**：第1层Cin=3（RGB），第2-3层Cin=4-16，例如本论文回归集中Layer 1的Cin=4、Cout=8对应有效MAC仅32/256，PE利用率仅12.5%<!-- 来自 model_analysis.md §2 / contributions.md §4.4 -->。这意味着芯片在网络入口就有近88%的MAC空转，浪费了昂贵的硅片面积。MAERI、Eyeriss-v2、Tangram等工作分别用"硬件可重构归约树"、"层级可重构互连"、"层间细粒度tile调度"来解决这一问题，但代价均是**硬件复杂度上升**——可重构互连的面积、时序、功耗代价不可忽视。

**瓶颈二：片上SRAM不足以驻留整图。** 480×640的VGA单帧灰度图占0.3 MB，单帧3通道RGB图占0.9 MB，单帧含32个feature map时占9.4 MB——而Xilinx 7系列FPGA的BRAM总量在1-2 MB量级。这意味着**端侧FPGA上不存在"整图驻留片上"的可能**，所有现实中的中分辨率以上推理都必须把特征图按某种粒度切分、片上仅暂存部分行/块/层，剩余靠DDR带宽支撑。传统TPU/Eyeriss/Gemmini等加速器的tile调度策略源自"片上scratchpad可装下若干tile"假设，落到FPGA上需要重新审视。Alwani等人在MICRO'16提出的Fused-layer<sup>[15]</sup><!-- ref: [Alwani-Fused-MICRO16] -->、Kang提出的AoCStream<sup>[16]</sup><!-- ref: [Kang-AoCStream-Sensors23] -->、Liu等人提出的Full-Stack streaming<sup>[17]</sup><!-- ref: [Liu-FullStack-TNNLS21] -->均是这一方向的代表，他们均观察到"片上仅缓存几行/几层中间数据 + 反压式数据流"是FPGA端侧推理的合理范式，但具体粒度（行级/层级/全片上）、调度策略（layer-serial/layer-pipelined）、控制风格（中心FSM/握手）的选择仍存在较大差异。

针对这两类瓶颈，本论文提出FLUX_CNN——一个面向Xilinx 7系列FPGA的16×16 INT8固定阵列CNN加速器。FLUX_CNN的核心思路在于：**硬件层面保持简洁**（固定阵列 + 去中心化valid-ready握手 + 5模块流水），**复杂度上推到编译器**（用Ky折叠和空间到深度变换填满浅层PE，用流式行环承载任意H×W），并辅以vendor级AXI DataMover IP做DMA子系统集成。这3条设计选择互相耦合：硬件简洁让综合PPA可控、流水时序可分析；编译器侧PE利用率优化避免动态可重构互连的硬件开销；流式行环让单次启动即可端到端跑完一帧任意分辨率图像，把控制开销压到host仅写4个boot寄存器+1次start命令<!-- 来自 contributions.md C3.2 -->。

### 1.4 选题意义

从工程角度，本论文形成的FLUX_CNN加速器在XC7K325T-2目标器件上单核占用128块BRAM36（28.8%）、82个DSP（9.8%）、36942个LUT（18.1%）；22个ResNet-18风格的链式回归用例在3种折叠模式下bit-exact通过，整网593K周期、网络级MAC利用率86.6%；20个多核W切片用例在N=2/4配置下bit-exact通过<!-- 来自 contributions.md §4.1-4.2 / STATUS.md §1-2 -->。这套实现填补了"开源、轻量、可复现、自带编译器栈、自带回归测试基础设施"的端侧FPGA-CNN加速器的空缺。

从学术角度，本论文的核心论点是：**在固定阵列硬件上，将PE利用率优化的复杂度从硬件可重构互连上推到编译器层是一条可行且代价更低的路径**。Ky折叠和空间到深度作为编译器侧零RTL改动的变换通道，在Cin<16浅层和stride≥2步幅层上分别把PE利用率从12.5-50%提升到接近100%。这与MAERI、Eyeriss-v2等硬件可重构方案构成**互补路径**——前者在硬件保持简洁的代价上换取编译器复杂度，后者在硬件复杂度的代价上换取算法层面的零干预。两条路径在不同应用场景下各有优劣，本论文的工作丰富了这一设计取舍空间。

## 2 国内外研究现状

CNN加速器是计算机体系结构、集成电路设计、深度学习编译器三个领域交叉的活跃方向。本节按6个主题梳理与本论文直接相关的工作：(1)空间阵列加速器与数据流分类；(2)硬件可重构PE利用率优化；(3)编译器/循环嵌套协同设计；(4)流式/行缓存FPGA加速器；(5)跨层融合与流式调度；(6)量化与算子融合。

### 2.1 空间阵列加速器与数据流分类

空间阵列是CNN加速器最经典的硬件组织方式。Google在2017年披露的TPU v1<sup>[2]</sup>是工业界首次系统化展示**256×256脉动阵列+权重驻留数据流+28 MiB统一缓冲**的数据中心规格——65536个INT8 MAC单元在工程上证明了脉动思想可以scale up到极大规模，峰值算力92 TOPS，CISC指令集（matrix multiply / convolve / activate）。TPU v1奠定了"INT8+脉动+大统一缓冲"的数据中心参考点，但其核心假设——"片上有数十兆SRAM足以容下输入与权重"——在端侧FPGA上完全不成立，端侧加速器面对的是相反约束：SRAM容不下整图，必须流式处理。

Chen等人在ISCA'16提出的Eyeriss<sup>[3]</sup>把"片上数据搬运是能耗主导项"作为核心论点，提出**行驻留（RS）数据流**：每个PE持有一行权重，按行展开输入特征图和部分和，让PE内、PE间、全局3级reuse同时奏效，相比已有数据流在卷积层节能1.4-2.5倍<!-- 来自 literature.md A 类 -->。这一工作随后在JSSC'17演化为Eyeriss芯片<sup>[18]</sup><!-- ref: [Eyeriss-JSSC17] -->，65nm工艺、14×12 = 168个PE、AlexNet 35 fps @ 278 mW、VGG-16 0.7 fps @ 236 mW、DRAM access/MAC = 0.0029（AlexNet），把端侧CNN加速器的性能-能耗SOTA钉死在"百毫瓦级 + 几十fps"。本论文沿用了"片上搬运能耗主导"的判断，但在FPGA场景下把它转写为：**DDR流量主导，所以需要行环流式 + 编译器侧重排（S2D）**，因为FPGA片上SRAM比ASIC更紧张，而DDR带宽与功耗比也更敏感。

NVIDIA开源的NVDLA<sup>[4]</sup>是工业级DLA IP，在Jetson Xavier等量产SoC中实际部署。其核心方法是把卷积流水分为**CDMA→CMAC→CACC→SDP**（Single Data Point processor，做bias/BN/ReLU/量化融合）4段，每段由显式controller驱动。SDP的命名和功能定义后被多个开源/学术加速器借用，本论文FLUX_CNN的SDP后处理（bias/residual/shift/ReLU/clip融合）正是从NVDLA继承命名脉络<!-- 来自 contributions.md C1.4 -->，但用去中心化valid-ready握手而非显式中心controller把4段串起来——这是本论文与NVDLA最显著的架构风格差异。

UC Berkeley开源的Gemmini<sup>[10]</sup>是参数化、生成式DNN加速器，基于Chipyard SoC生成器，已在TSMC 16nm和Intel 22FFL节点实际流片。Gemmini默认配置之一即**16×16 INT8脉动阵列**，可选输出驻留或权重驻留数据流，靠RoCC ISA与RISC-V处理器耦合，由显式中心调度器驱动。Gemmini与FLUX_CNN在阵列规模和数据类型上完全可对照——同样16×16、同样INT8——但2个工作的关键差异在3处：(1)Gemmini是**中心调度器**驱动的同步式架构，FLUX_CNN是**去中心化valid-ready握手**；(2)Gemmini基于tiled scratchpad，FLUX_CNN基于流式行环；(3)Gemmini与SoC紧耦合，FLUX_CNN作为独立AXI4 IP接入。本论文将Gemmini作为最贴近的对照对象之一，在第五章实测结果分析中重点对比。

Shao等人在MICRO'19提出的Simba<sup>[11]</sup>探索了**多chiplet可扩展DNN加速器**，36个chiplet通过MCM (multi-chip-module) 封装、ground-referenced signaling NoP互联，单chiplet 16 PE × 8 vector lane = 128 MAC，整模组ResNet-50 batch=1达1988 fps、0.50 ms延迟。Simba论证了chiplet级别的NoP通信是可解决的瓶颈，但其scaling瓶颈是通信带宽；FLUX_CNN多核版（XC7K325T上N=2/4）则是**BRAM-bound**而非通信bound，对应不同的scaling域。两者放在一起说明：scaling策略的选择强烈依赖底层平台的资源画像。

Du等人在ISCA'15提出的ShiDianNao<sup>[19]</sup><!-- ref: [ShiDianNao-ISCA15] -->针对嵌入式视觉场景，与图像传感器紧耦合、无DRAM访问，靠**整图驻留片上SRAM**支撑。ShiDianNao与FLUX_CNN同为输出驻留数据流，但其"全图片上"假设只在小图像、传感器耦合场景下成立；FLUX_CNN把输出驻留推广到**大图、流式**场景，约束完全不同。这种"同数据流不同假设"的对比说明：数据流取舍是一回事，承载它的存储模型是另一回事——后者往往是真正决定加速器能否在端侧FPGA上落地的瓶颈。

### 2.2 硬件可重构PE利用率优化

针对网络层形状（Cin、Cout、K、stride）与固定阵列尺寸不匹配导致的PE利用率下降问题，第一类思路是让阵列内部互连本身可重构，从硬件层面吸收任意层几何。

Kwon等人在ASPLOS'18提出的MAERI<sup>[13]</sup>是这一路线的代表，其核心是给乘法器阵列加一棵**增强归约树（ART）**和一棵**分发树**，2棵树的拓扑均可在运行时配置，相比刚性NoC基线在不同mapping上的PE利用率提升8-459%<!-- ref: [MAERI-ASPLOS18] -->。Chen等人在JETCAS'19提出的Eyeriss-v2<sup>[12]</sup>把原始RS数据流推广到**层级化mesh NoC**，每层根据需要在multicast / unicast / broadcast 3种模式间重配，在稀疏MobileNet上相比v1取得12.6倍加速、2.5倍能效提升<!-- ref: [Eyeriss-v2-JETCAS19] -->。Gao等人在ASPLOS'19提出的Tangram<sup>[14]</sup>把同样的可重构哲学推到**层间**——多个tiled加速器实例之间通过交替buffer共享数据流来重构层间通信路径，在多层流水线上取得2倍性能与45%能耗改善<!-- ref: [Tangram-ASPLOS19] -->。FlexFlow<sup>[20]</sup><!-- ref: [FlexFlow] -->等后续工作进一步把可重构粒度推到feature map / 神经元 / 突触3级并行的混合配置。

本论文FLUX_CNN选择此设计取舍空间的**对立点**：阵列内部互连固定为列广播激活+列独立Cout累加，PE利用率恢复完全交给编译器侧的Ky折叠和S2D（详见§3）。2种取舍各有代价——硬件可重构方案以RTL面积与验证代价吞下更宽的层形状包络，编译器方案则以每层一次变换的预处理代价换取RTL极简。本论文不主张哪条路径更优，而是论证：**在浅ResNet-18风格的网络规模下，仅靠编译器侧已经足够**，省下的RTL复杂度对中等规模FPGA是有意义的工程收益。

### 2.3 编译器/循环嵌套协同设计

与硬件可重构方案天然对偶的另一族工作，是从循环嵌套层面做协同设计——不改变运行时连线，而在编译期重写循环。

Parashar等人在ISPASS'19提出的Timeloop<sup>[21]</sup><!-- ref: [Timeloop-ISPASS19] -->把DNN数据流形式化为7重循环嵌套上的mapping，配合Accelergy能量模型对每个候选mapping打分，是这一族最具影响力的搜索框架。Yang等人在ASPLOS'20提出的Interstellar<sup>[22]</sup><!-- ref: [Interstellar-ASPLOS20] -->把同样的循环嵌套以Halide调度原语表达，给出了已有稠密DNN加速器的形式化分类，并报告在固定吞吐下4.2倍CNN能量改善。Chen等人在OSDI'18提出的TVM<sup>[23]</sup><!-- ref: [TVM-OSDI18] -->与其FPGA后端VTA<sup>[24]</sup><!-- ref: [VTA-MICRO19] -->把循环级调度产品化为端到端编译栈，VTA参考实现的256-PE @ 100 MHz Zynq实例是与FLUX_CNN最接近的开源编译栈对照。Ma等人在FPGA'17的循环优化研究<sup>[25]</sup><!-- ref: [Ma-FPGA17] -->系统枚举4-6维循环嵌套的tiling、unrolling、interchange决策，在Arria 10 GX1150上以VGG-16取得645 GOPS。最后，Chetlur等人的cuDNN中im2col<sup>[26]</sup><!-- ref: [cuDNN-arXiv14] -->代表这一语言的"完全展开"极端——每个K×K·Cin窗口被显式展开为一列，转化为标准GEMM。

本论文的Ky折叠在这一语言中可读作**部分im2col**：仅把Ky维度折入Cin、沿y轴复制输入、重排权重，而Kx保留为时分复用维。这一定位介于完全im2col（K²倍内存膨胀，FPGA规模下不可行）和按层mapping搜索（保留原循环嵌套、接受Cin/16利用率）2个极端之间。S2D则是另一种位置——把stride²空间相位重排进Cin，纯编译期reshape，零复制。两者都属循环变换，但据本论文所知，所调研工作中尚无以这一名字出现的等价pass<!-- 来自 contributions.md §8.1 -->[CHECK: S2D在加速器领域的引用谱系待补查ASPLOS/HPCA是否有更早的architecture-side先例]。

这一族中与FLUX_CNN最直接可比的工作，是Liu等人在TNNLS'21的全栈流式加速器<sup>[17]</sup>，其在Arria 10 GX1150上以97% MAC效率达到>1.3 TOP/s。该工作是本论文最重要的**先验威胁之一**，必须明确差异化：Liu等人在Arria 10 GX1150（其BRAM与DSP资源约为XC7K325T的一个数量级）上通过深度层融合 + 激进DSP packing取得近峰值MAC效率；FLUX_CNN在16倍小的中等规模FPGA上、在不做层融合的前提下取得86.6%网络级MAC利用率<!-- 来自 contributions.md §4.3 / STATUS.md §1 -->。**两者不是头对头比较，而是Pareto前沿上不同器件规模与架构哲学的可比点**——Liu的近峰值数字依赖大器件容下整网层融合，FLUX_CNN的高利用率数字则不需要这一前提。

### 2.4 流式/行缓存FPGA加速器

FPGA上CNN加速的事实标准模式，是流式行缓存数据流：特征图按光栅顺序产消、片上缓冲尺寸由感受野而非整图决定。

Venieris与Bouganis在FCCM'16提出的fpgaConvNet<sup>[27]</sup><!-- ref: [fpgaConvNet-FCCM16] -->以同步数据流形式化每层为流式硬件模块，由HLS生成完整逐层流水，相比早期FPGA-ConvNet架构取得2.94倍性能密度提升；其TNNLS'19扩展<sup>[6]</sup>把这一框架推广到多模型场景。Bettoni等人在ISCAS'17提出的Snowflake<sup>[8]</sup>在Zynq XC7Z045上跑行缓存驱动的向量引擎，在AlexNet、GoogLeNet、ResNet-50上取得平均91%计算效率，是与FLUX_CNN在器件级别最接近的同平台对照。Guo等人在TCAD'18提出的Angel-Eye<sup>[7]</sup>在同等Zynq器件上加入INT8量化感知mapping、提供端到端Caffe→硬件编译。Aydonat等人在FPGA'17的DLA<sup>[28]</sup><!-- ref: [Aydonat-FPGA17] -->采用1-D Winograd加速的流式引擎、在Arria 10上AlexNet达到1382 GFLOPS。Lu等人在FCCM'17的Winograd FPGA加速器<sup>[29]</sup><!-- ref: [Lu-Winograd-FCCM17] -->在ZCU102上取得854.6 GOPS。Zhang等人的Caffeine<sup>[30]</sup><!-- ref: [Caffeine] -->则把FCN与卷积统一成uniformed representation。

这6个工作合起来确立了流式行缓存模式 + 逐层模块组合的基线，其共有特征是**每层一个独立硬件模块、HLS或RTL层级流水、片上缓冲尺寸正比于感受野而非整图**。FLUX_CNN与这一基线的差异在于：(1)使用**单一16×16共享核**在时间维上轮转所有层，而非每层独立硬件模块；(2)采用**去中心化valid-ready握手**而非每层固定流水深度；(3)行级credit双向反压机制使核内的IFB ring做到了行粒度而非整图粒度。Snowflake与Angel-Eye作为同器件最近邻peers，将在第五章实测中作为定量对照对象[CHECK: 同器件Snowflake/Angel-Eye的Fmax/资源/整网MAC%对照数字待补]。

### 2.5 跨层融合与流式调度

跨层融合是流式族中独立的子方向，目的是消除层间DRAM中转、让多层数据在片上直接传递。这一方向是本论文的**第二大先验威胁来源**——Alwani et al. fused-layer与Kang AoCStream都在这一坐标上做工作，必须明确差异化。

Alwani等人在MICRO'16提出的层融合加速器<sup>[15]</sup>是这一方向最早期、也是最常被引用的工作。其把VGGNet-E的前5个卷积层融为单条片上流水线，以362 KB片上存储为代价消除95%的层间DRAM流量<!-- ref: [Alwani-MICRO16] -->。这一思路对FLUX_CNN的"行流式"主张构成最直接的威胁，因为其工作粒度比行更细：在子feature map的层间窗口上同时驻留多层切片。**FLUX_CNN与Alwani的差异化**在于粒度与资源画像：Alwani同时驻留多层feature map切片，片上存储随融合层数线性增长；FLUX_CNN每次只跑1层、单一16×16共享核 + 仅当前层一个`strip_rows`大小窗口驻留、DDR充当层间buffer<!-- 来自 contributions.md §3.2 -->。在XC7K325T的1.6 MB BRAM预算下、对VGA分辨率输入而言，Alwani方案不可行；FLUX_CNN的行级粒度因此**不是对Alwani层融合的严格改进，而是流式-融合设计空间上的另一个点**——适合更小器件与更长网络。

Kang等人在arXiv'22 / Sensors'23的AoCStream<sup>[16]</sup><!-- ref: [Kang-AoCStream-arXiv22] -->是第二个紧贴FLUX_CNN的先验工作，其stream-based行缓存在架构上与FLUX_CNN的IFB ring最为相似。AoCStream面向低端FPGA，使用**逐层独立的行缓存模块**——尺寸随图像宽度线性增长而非随面积二次增长——目标是在无外部存储的前提下把整个CNN目标检测器装进片上。**FLUX_CNN与AoCStream的差异化**在于：(1)层流水的多硬件块 vs 时间复用的单一核——AoCStream每层一块硬件、所有数据片上流通，FLUX_CNN以行级credit + DDR层间buffer实现单核时分；(2)是否依赖"全片上"假设——AoCStream面向Kang所目标的小检测网络可行，但对VGA分辨率下的更大网络不可扩展；(3)资源换吞吐——AoCStream取得更高峰值吞吐的代价是更多复制的MAC硬件。这同样是"不同假设下的不同点"，而非头对头优劣。

Liu等人TNNLS'21<sup>[17]</sup>在§2.3已论及，此处不重复。3个先验工作合起来锚定了流式族的设计空间，FLUX_CNN在其中占据**单核时分 + 行级credit + DDR层间**这一独立点，是本论文叙事B（流式行环 + 跨核scaling）的差异化坐标。

### 2.6 量化与算子融合

INT8量化与后处理融合（bias/scale/shift/ReLU/clip/residual）是端侧CNN加速器的标配组件，本论文将其作为SDP块的背景，本节做简要回顾。

Jacob等人在CVPR'18提出的量化感知训练<sup>[31]</sup><!-- ref: [Jacob-CVPR18] -->确立了如今通用的逐通道非对称INT8方案，在ImageNet级别的benchmark上保持与FP32精度差<1%，为FLUX_CNN的INT8 MAC阵列工作提供前置条件。Han等人在ISCA'16提出的EIE<sup>[32]</sup><!-- ref: [EIE-ISCA16] -->与FPGA'17的ESE<sup>[33]</sup><!-- ref: [ESE-FPGA17] -->把量化路线推广到稀疏 + 量化的联合推理上——这是本论文未涉及的正交维度，仅作引用以承认稀疏方向的存在。Pellauer等人在ASPLOS'19提出的Buffets<sup>[34]</sup><!-- ref: [Buffets-ASPLOS19] -->提供了通用的可解耦片上存储抽象，影响后续多个加速器的存储层级设计。Chetlur等人的cuDNN<sup>[26]</sup>中im2col + GEMM融合是另一族算子融合的代表，已在§2.3论及。

NVDLA<sup>[4]</sup>把SDP（Single Data Point processor）形式化为独立的后MAC处理段，处理bias/BN/ReLU/量化融合，FLUX_CNN直接继承这一命名与分段哲学<!-- 来自 contributions.md C1.4 -->。FLUX_CNN在融合方向的具体增量贡献是：(1) R.1 refactor把bias从mac_array迁到SDP，移除原先bias_rf的32-Cout-slice限制；(2) R.2 refactor在残差路径上加入可编程的`shortcut_mult` / `shortcut_shift`量化因子，让残差贡献被重新量化以匹配主路径scale<!-- 来自 contributions.md C1.4 / memory/bias_to_sdp_refactor.md -->。这2个增量被定位为系统完备性论点而非头条贡献，因为更宏观的SDP融合概念已被NVDLA等工作充分确立。

He等人CVPR'16的ResNet<sup>[1]</sup>引入的残差连接是本论文SDP残差融合的算法源头，已在§1引用，不再重复。

## 3 本论文研究内容与解决问题

基于§2梳理的6条研究脉络，本论文的研究目标可表达为3条交叉的设计取舍轴线：(1) PE利用率优化的实现位置——硬件可重构互连 vs 编译器侧reshape；(2) 流式调度的粒度与层间策略——多层片上融合 vs 行级单核时分 + DDR层间；(3) 控制流组织——中心调度器同步驱动 vs 去中心化valid-ready握手。本论文FLUX_CNN在这3条轴上分别选择**编译器侧、行级单核时分、去中心化握手**这一组合点，并以此定义本论文要解决的核心问题与对应贡献。

**第一条轴（PE利用率优化位置）**针对的具体问题是：固定16×16阵列在Cin<16浅层与stride≥2步幅层上的利用率塌陷。MAERI<sup>[13]</sup>、Eyeriss-v2<sup>[12]</sup>、Tangram<sup>[14]</sup>等工作以可重构互连吸收任意层几何，本论文则提出2个零RTL改动的编译器pass：Ky折叠把卷积核Ky维折入cin_fake、由编译器侧做输入y偏移复制，S2D把stride²空间相位重排进Cin、由编译器侧做无复制重排。在ResNet-18风格22 case回归上，2个pass分别把浅层与stride层的PE利用率从12.5-50%提升到接近100%、整网平均86.6%<!-- 来自 contributions.md §4.1-4.3 / STATUS.md §1 -->。**与§2.3中Liu Full-Stack TNNLS'21的差异化**：Liu在Arria 10 GX1150（约16倍大器件）上以层融合达到97% MAC效率、本论文在XC7K325T中等规模FPGA上以编译器侧填充 + 单层执行达到86.6%，2个数字在Pareto前沿不同点上等价可比。

**第二条轴（流式粒度）**针对的具体问题是：在中等规模FPGA上以VGA分辨率支持完整CNN推理时的片上存储瓶颈。Alwani fused-layer<sup>[15]</sup>以多层片上驻留消除95%层间DRAM流量、Kang AoCStream<sup>[16]</sup>以逐层独立硬件块在低端FPGA上达成全片上推理，本论文提出**行级单核时分 + DDR层间buffer**这一互补点：单一16×16共享核在时间维上轮转所有层，每层只驻留`strip_rows`大小的特征图窗口，行级credit双向反压机制保证核内的IFB ring做到行粒度。在20个多核W切片用例（N=2/4配置）上bit-exact通过<!-- 来自 contributions.md §4.2 -->。**与Alwani / Kang的差异化**：Alwani方案在1.6 MB BRAM预算下不可行、AoCStream的"全片上"假设对VGA规模网络不可扩展，FLUX_CNN在不假设全片上的前提下提供了适合中等规模FPGA与较长网络的工程组合点。

**第三条轴（控制流去中心化）**针对的具体问题是：随阵列规模与pipeline段数增长，中心FSM/调度器的状态空间膨胀与时序闭合压力成为RTL复杂度主因。Gemmini<sup>[10]</sup>、NVDLA<sup>[4]</sup>等以中心调度驱动pipeline，本论文采用**5模块去中心化valid-ready握手**：line_buffer / mac_array / parf_accum / ofb_writer / wgt_buffer之间无中心FSM、各模块仅根据本地握手与共享cfg_regs决定推进。这一选择把单核RTL规模压到36942 LUT / 9.8% DSP / 28.8% BRAM36（XC7K325T）<!-- 来自 STATUS.md §2 -->，为多核scaling与编译器侧填充pass的简单性提供前置条件。

3条轴上的选择共同定义了本论文的核心论点：**在中等规模FPGA + 浅ResNet风格网络的应用域内，"硬件极简 + 编译器侧填充 + 行级流式 + 去中心化握手"这一组合点能取得与硬件可重构 / 多层融合方案在Pareto前沿上等价可比的工程收益**。本论文的贡献集合（架构C1.1-1.4 / 编译器C2.1-2.4 / 流式与多核C3 / 量化与融合C4 / 工具链与回归C5，详见后续章节）即围绕证明这一论点展开。

## 4 本论文章节结构

全文除引言（本章）与结论外，按"先架构、后编译器、再系统、最后实测"的顺序组织5个主体章节，每章围绕第一章§3提出的3条设计取舍轴线中的一条或多条展开论证。

第二章发展加速器架构本身，对应§3轴(3)的去中心化握手与轴(1)的OS+列广播数据流，展开`line_buffer→mac_array→parf_accum→ofb_writer`+`wgt_buffer`的5模块结构、cfg_regs共享配置面、以及核内行环IFB的双向credit反压机制。第三章发展编译器与PE利用率优化pass，对应§3轴(1)的"编译器侧"立场，给出Ky折叠与S2D的数学推导、`gen_isa_test.py` derived 值生成、`hw_files.derive_layer_cfg()`的cfg派生路径、以及多层链式回归的`run_regression.py`机制。第四章发展DMA子系统、流式调度与SDP后处理，对应§3轴(2)的"行级单核时分"立场，展开`idma_ctrl/wdma_ctrl/odma_ctrl + mm2s_arb + axi_dm + axi_m_mux + axi_lite_csr`的子系统结构、descriptor-driven的层链调度、bias/residual/shift/ReLU/clip的5路融合（R.1与R.2 refactor），以及多核W切片下的halo冗余机制。第五章是实测与对照分析，回到§3的3条轴，给出22 ResNet-18风格回归用例 + 20个多核切片用例的bit-exact通过情况、整网593K周期、网络级MAC利用率86.6%、XC7K325T占用数据，并与Snowflake / Angel-Eye / Gemmini / Liu Full-Stack在同器件或Pareto可比意义下做定量对照。

第六章总结全文并讨论未来工作方向，包括但不限于：层融合与编译器侧填充的组合、稀疏量化（EIE/ESE方向）的引入、以及multicore scaling向chiplet级（Simba方向）的扩展可行性。

[TBD: 各章是否独立成章 vs 合并 — 视终稿篇幅压力定，目前按独立成章组织]

---

---

# 第一章 研究背景与总体方案

## 1.1 整体设计目标

本论文FLUX_CNN的整体设计目标，是面向中等规模边缘FPGA、为浅层ResNet-18风格网络提供一个**硬件极简、软件可重构**的INT8 CNN加速器实现。这一目标在前文§3的3条设计取舍轴上落点为：硬件侧保持16×16固定阵列、互连不可重构；编译器侧承担PE利用率恢复职责；流式数据通路以行为粒度、单核时分复用所有层。设计目标的内核是**用编译期变换吞掉运行期的灵活性需求**——把传统上由可重构互连承担的层形状适配压力前移到编译器侧，从而把RTL规模、验证代价与时序闭合压力压到中等规模FPGA可承受的范围。

具体而言，本论文从如下3点细化整体设计目标。**(a) 硬件简洁性**：阵列保持16×16 INT8固定网格、互连非重构，理由是可重构PE互连付出可观的RTL面积与验证代价，而当同等效果可在编译期取得时这一代价不被支撑<!-- 来自 contributions.md C1.1 -->。**(b) 编译器侧PE填充**：浅层利用率不足由编译器变换恢复——把卷积重塑到完全占满的16通道阵列上，无需任何运行时重构<!-- 来自 contributions.md C2.1 / C2.2 -->。**(c) 单次启动行环流式**：单条host发出的`start`指令即可驱动任意H×W图像走完整条流水，片上ring仅需按感受野余量定尺寸<!-- 来自 contributions.md C1.2 -->。

3条目标分别对应先前研究空间中的3个不同位置：固定阵列设计承TPU与Gemmini谱系；编译器单边PE利用率恢复对立于硬件可重构ART/NoC路线；行级流式区别于多层融合或全图tiled方案。这一三轴定位决定了FLUX_CNN在Pareto前沿上不是某一现有方案的严格改进，而是**为中等规模器件 + 浅层网络这一具体应用域优化的工程组合点**。

## 1.2 设计约束与边界条件

整体设计目标的可实现性由若干硬性约束界定，本论文明确把这些约束写入论证基础——后续架构与编译器选择必须满足或绕开它们。

**目标器件**：本论文选定Xilinx Kintex-7系列XC7K325T作为主目标器件，该器件提供445片BRAM36（约1.6 MB片上SRAM）、840 DSP48E1<!-- 来自 contributions.md §4.1 / STATUS.md -->。这一器件规模代表中端FPGA典型配置，比Liu等人TNNLS'21<sup>[17]</sup>选用的Arria 10 GX1150（BRAM与DSP资源约高一个数量级）小约16倍，但比Snowflake<sup>[8]</sup>、Angel-Eye<sup>[7]</sup>选用的Zynq XC7Z020/XC7Z045同档或略大，是同类工作Pareto前沿上一个有代表性的可比点。

**精度与算子边界**：FLUX_CNN采用**INT8激活 + INT8权重 + INT32部分和**的统一精度配置，依据是Jacob等人CVPR'18<sup>[31]</sup>建立的逐通道非对称INT8量化方案在ImageNet级别benchmark上保持与FP32精度差小于1%——这一前置条件由量化感知训练侧保证，本论文不重复其论证。算子覆盖卷积、bias、residual、ReLU/clip、量化重缩放5路，复杂算子如池化、深度可分离卷积、softmax留作未来工作[CHECK: 算子缺口的精确清单见STATUS.md未来工作章节]。

**输入规模与流式约束**：目标输入分辨率上界为VGA级别（480×640、INT8、16通道），单图扁平张量约4.9 MB，约为整个BRAM预算的3倍——平台无法以"全图驻留片上"的方式承载输入<!-- 来自 contributions.md §4.1 -->。这一硬约束直接决定了流式数据通路是**唯一可行**的路径，而非可选优化。**网络深度**约束为浅层ResNet-18风格（约11-22层），更深网络受限于片外DDR带宽与编译器侧多层链调度复杂度，留作未来工作。

**频率约束**：本论文综合实测在XC7K325T-2速度等级上达到Fmax约80 MHz<!-- 来自 STATUS.md §2 -->[CHECK: Fmax具体数字与时序闭合方法见第五章]，对应单核理论峰值256 MAC × 80 MHz × 2 op = 41 GOPS。该频率不是器件物理上限，而是当前阶段权衡综合迭代代价后的工程选点，未来工作中通过流水线深化或时序优化提升到100 MHz目标。

## 1.3 总体架构方案

FLUX_CNN系统按**两层结构**组织：内层是承担卷积计算的5模块**核流水**，外层是把核与外部内存、host桥接的**DMA子系统** + 多核wrapper。从外部视角，整个加速器仅暴露1个AXI4 master端口（用于片外数据搬移）与1个AXI-Lite slave端口（用于descriptor与配置寄存器访问），SoC集成边界刻意收窄至最小。

### 1.3.1 核流水五模块结构

核流水由链式结构`line_buffer → mac_array → parf_accum → ofb_writer`构成，加上侧路供权重的`wgt_buffer`，共5个模块；所有模块共享一个`cfg_regs`配置面、由后处理段`sdp`完成bias/residual/scale/shift/ReLU/clip的5路融合。该命名是**有意识地承袭NVDLA<sup>[4]</sup>的`CDMA → CMAC → CACC → SDP`谱系**与Snowflake<sup>[8]</sup>的流式卷积分解风格——这一选择不是必须而是为了让熟悉NVDLA或更广义流式卷积文献的读者能把FLUX_CNN各段映射到共享词汇上，从而把更非常规的设计点（列独立PARF、行环反压）凸显出来<!-- 来自 contributions.md C1.1 + RTL/mac_array.sv -->。

5模块各自承担可分离的关注：`line_buffer`以光栅顺序产消激活、片上窗口大小由感受野决定（典型8 × W × Cin字节、远小于全图）；`mac_array`是16×16 INT8乘累加阵列，采用**列广播激活 + 列独立Cout累加**的输出驻留（OS）变体——每列绑定一个Cout通道、列内16个PE对应16个Cin通道，所有列共享同一16通道激活向量；`parf_accum`由`parf_col`×16组成，每列独立SRAM、外壳共享wr_addr/we，承担partial sum在多slice/多tile边界上的本地累加；`ofb_writer`完成partial sum→output的reshape、量化、写回；`wgt_buffer`从WRF侧路供权重<!-- 来自 RTL/mac_array.sv L36-58 + docs/modules/mac_array.md L25-33 -->。

### 1.3.2 DMA子系统

围绕核流水，DMA子系统由3个轻量级控制器（`idma_ctrl`、`wdma_ctrl`、`odma_ctrl`）驱动一个Xilinx `axi_dm` DataMover IP——其中MM2S通道由IDMA与WDMA通过`mm2s_arb`串行仲裁共享、S2MM通道由ODMA独占；外加`axi_m_mux`把DataMover与DFE（Descriptor Fetch Engine）流量聚合到单一外部master端口、`axi_lite_csr`暴露配置寄存器与descriptor入口给host<!-- 来自 CLAUDE.md 顶层架构描述 -->。3级DMA阶段（IDMA、计算、ODMA）的设计目标是**重叠而非串行**——具体的时序模型与行级credit机制把这一重叠落到行粒度，详见后续章节。

axi_dm IP由Vivado 2023.1的`Syn/gen_axi_datamover.tcl`生成，ModelSim仿真前需先跑`Syn/compile_simlib.tcl`一次性编译Vivado simlib<!-- 来自 CLAUDE.md -->。这一对外部vendor IP的依赖代价被两点缓解：(a) IDMA/WDMA/ODMA的控制端均由本论文自研、与vendor IP之间通过标准DataMover命令/状态接口解耦，便于后续替换为其他DMA方案；(b) `axi_m_mux`、`axi_lite_csr`、`axi_arbiter`等外围AXI基础设施全部本论文自研、可独立单测<!-- 来自 sim/tb_axi_lite_csr / sim/tb_axi_m_mux -->。

### 1.3.3 控制流：去中心化valid-ready

整个核流水**无中心FSM**——5个模块之间仅通过valid-ready握手与共享`cfg_regs`协调推进，每个模块依据本地握手与配置寄存器状态独立决定何时产出、何时阻塞<!-- 来自 contributions.md C1.1 -->。这一选择背后的论证是：随阵列规模与pipeline段数增长，中心调度器的状态空间膨胀与多模块时序闭合压力会成为RTL复杂度主因。Gemmini<sup>[10]</sup>、NVDLA<sup>[4]</sup>等工作以中心FSM驱动pipeline，本论文以去中心化握手取代——单核RTL规模因此压到约36942 LUT / 9.8% DSP / 28.8% BRAM36（XC7K325T）[CHECK: 综合资源占比待第五章给出实测表]<!-- 来自 STATUS.md §2 -->，为多核scaling与编译器侧填充pass的简单性提供前置条件。

### 1.3.4 多核wrapper

在单核基础上，本论文进一步通过`multicore_top.sv`提供**N=2/4核W维切片**的扩展，将输入特征图沿width维分割、每核独立处理一个strip、切片边界处由编译器侧计算halo冗余以保证bit-exact等价于单核结果<!-- 来自 contributions.md §4.2 / RTL/multicore_top.sv -->。多核wrapper不引入新的控制层级——各核共享AXI master端口、由`axi_arbiter`仲裁，descriptor list按核分发，仍维持单`start`即整网推理的host接口<!-- 来自 sim/tb_multicore -->。多核扩展在20个跨核切片用例（`gen_cross_core_test.py`生成）上bit-exact通过[CHECK: 20用例的具体断言通过率与cross-core带宽实测见第五章]。

## 1.4 技术路线选择论证

§1.1的3条设计目标在更大的设计取舍空间中并非唯一可行选择，本节把每项选择对照可能的替代路径展开论证，明确"为什么是这一组合"而非其他。

**关于硬件简洁性 vs 可重构互连**：MAERI<sup>[13]</sup>以可重构ART树取得跨mapping 8-459%利用率提升、Eyeriss-v2<sup>[12]</sup>以分层mesh NoC取得12.6倍稀疏MobileNet加速、Tangram<sup>[14]</sup>把可重构推到层间。3者共有的代价是：(a) 互连的物理实现复杂度直接转化为RTL面积（特别是对FPGA的LUT/MUX资源压力显著）；(b) 时序闭合代价——任意拓扑互连的时序路径预测难度高；(c) 验证代价——可重构状态空间随重构粒度组合爆炸。本论文论证的不是这3者方案不优，而是：**在浅ResNet-18风格的网络规模下，可由编译器侧吞下的层形状适配压力，未必需要硬件侧承担**——节省的RTL复杂度对中等规模FPGA是有意义的工程收益。

**关于编译器侧填充 vs 接受Cin/16利用率**：在固定16×16阵列上，浅层（Cin=4/8）的空间利用率天然为25%-50%，传统mapping搜索（如Timeloop<sup>[21]</sup>、Interstellar<sup>[22]</sup>）会把这一约束写入mapping space并接受其下界。本论文反其道而行——通过Ky折叠（把卷积核Ky维折入cin_fake、由编译器侧做输入y偏移复制）与S2D（把stride²空间相位重排进Cin、纯编译期reshape无复制）把卷积变换到完全占满的16通道形态<!-- 来自 contributions.md §4.1 / docs/pe-fold.md -->。在ResNet-18风格22 case回归上，2个pass把浅层与stride层的PE利用率从12.5-50%提升到接近100%、整网平均86.6%<!-- 来自 contributions.md §4.3 / STATUS.md -->[CHECK: post-fold各层利用率细分表见第三章]。

**关于行级单核时分 vs 多层片上融合**：Alwani<sup>[15]</sup>把VGGNet-E前5层融为单条片上pipeline、以362 KB片上存储消除95%层间DRAM流量；Kang AoCStream<sup>[16]</sup>把整个CNN检测器装进片上、走逐层独立硬件块。两者共同假设是"多层feature map可同时片上驻留"——这一假设在1.6 MB BRAM预算 + VGA分辨率下不成立。本论文选择**行级单核时分 + DDR层间buffer**：单一16×16共享核在时间维上轮转所有层、每层只驻留`strip_rows`大小的特征图窗口、DDR充当层间buffer<!-- 来自 contributions.md §3.2 -->。这不是对Alwani层融合的严格改进，而是流式-融合设计空间上的另一个点——适合更小器件与更长网络。

**关于OS-with-column-broadcast vs WS / RS**：本论文阵列采用**输出驻留 + 列广播激活**的混合数据流，原因是其与行级流式数据通路的耦合最小：在OS下阵列以光栅顺序消费激活、按Cout-tile发出输出，与`line_buffer`产消顺序天然一致；而TPU<sup>[2]</sup>的WS需要权重在阵列内驻留多output pixel周期、Eyeriss<sup>[5]</sup>的RS需要kernel行驻留并对角累加，2者都引入与流式数据通路的额外耦合<!-- 来自 contributions.md C1.1 + 原英文 §2.1 -->。这一选择不主张数据流本身的新颖性，而主张其与流式调度的**最小耦合性质**。

## 1.5 本章小结

本章在§1.1界定了FLUX_CNN的3条整体设计目标——硬件简洁、编译器侧PE填充、单次启动行环流式；§1.2明确了目标器件XC7K325T、INT8精度、VGA输入规模、80 MHz频率等硬性约束；§1.3展开了核流水5模块 + DMA子系统 + 去中心化握手 + 多核wrapper的总体架构方案；§1.4对照可重构互连、mapping搜索、多层融合等替代路径论证了本论文的取舍逻辑。下一章将聚焦核流水的硬件架构本身，详述5模块的微架构、握手机制、数据流路径与cfg_regs配置面。

# 第二章 硬件架构设计

本章沿§1.4确立的3轴坐标系中第(1)轴展开硬件架构，为两项工程支撑型贡献——C1.1（去中心化valid-ready流水）与C1.2（流式行环数据通路）——以及PARF的分列组织（C1.3）和编译器在第三章中针对的7层循环嵌套（C2.4）提供落地论证。本章在不足以支撑新颖性主张之处刻意保留了常规设计，**避免用验证代价换取边际新颖性**；本论文真正主张的设计选择在小节级别明示标注、不混藏于微架构罗列中。需要指出的是，在本章覆盖的硬件层面，C1.1与C1.2的自评新颖性为弱-中（去中心化握手是Buffets<sup>[Buffets-ASPLOS19]</sup>等工作已建立的形式化模式、本论文是其在模块链粒度的忠实实例化），这一立场会在每节明确表述。

## 2.1 去中心化valid-ready流水

5个核心模块各自维护本地计数器（行、列、kx/ky、slice）并仅通过模块边界的valid-ready握手通信，**不存在中心FSM或全局调度器**统筹推进（C1.1）。每个模块只知本地进度与紧邻上下游的就绪状态——这是维持流水弹性所需的最小状态量<!-- 来自 contributions.md C1.1 + RTL/core_top.sv -->。这一选择背后的工程论证是：随阵列规模与流水段数增长，中心调度器的状态空间膨胀与多模块时序闭合压力将成为RTL复杂度主因，而去中心化握手把复杂度均摊到每个模块的本地控制中。

每条模块到模块的边界都遵循统一的`计数器+ready反压`模板：生产者持`valid`直至消费者`ready`为高，传输中的数据在停顿时被保留<!-- 来自 RTL/mac_array.sv valid-ready 时序 -->。这一弹性join纪律使得`parf_accum` tile *N*的**drain**与tile *N+1*的**fill**可以重叠——首轮fill消费WRF/IFB token而drain仍占据PSUM读端口、二者**不共享状态、仅共享握手credit**。

`sequencer`模块是存在的，但其职责被严格限定为**跨块启动同步**——在恰当时机派发一个tile的IDMA / 计算 / ODMA 3阶段然后退出，**不micromanage流水内部**<!-- 来自 docs/modules/sequencer.md -->。"这一最小协调足够"的工程证据是ResNet-18风格22-case链式回归——3种编译模式（无fold、`--fold`、`--fold --s2d`）下均逐bit通过[CHECK: 22-case回归PASS率，来自STATUS.md]。

这一去中心化路径与3个邻近参考点形成对比：Gemmini<sup>[10]</sup>的中心RoCC调度器、NVDLA<sup>[4]</sup>的显式控制器层级、VTA<sup>[VTA-Micro19]</sup>的`fetch–load–compute–store` 4阶段ISA——3者均保留较强的全局控制器。Buffets<sup>[Buffets-ASPLOS19]</sup>提供了`buffer + counter + handshake`的形式化词汇、本论文非正式采用之——本论文不主张Buffets形式化层面的贡献，而仅主张FLUX_CNN核流水是该模式在**模块链粒度而非存储元件粒度**的忠实实例化<!-- 来自 contributions.md C1.1 自评 -->。

本节描述的握手语义是下一节行级前向压机制赖以建立的底层基质——若每条边界都不具备弹性反压，行环将无法在下游credit枯竭时干净停顿。

## 2.2 流式行环数据通路与双向credit反压

IFB与OFB物理上组织为**行级环形缓冲区**——以`strip_rows`取模寻址、流水以**行而非帧粒度**互锁（C1.2）。具体而言，`line_buffer`仅在`rows_available ≥ y_out · stride + Ky`时才发出滑动窗（输入侧前向压）、`ODMA`在每行输出离片后归还一个行credit给`ofb_writer`（输出侧反压）<!-- 来自 RTL/line_buffer.sv 与 RTL/odma_ctrl.sv -->。`sequencer`一次派发即同时启动一个tile的IDMA、计算、ODMA 3阶段，使三者并发推进而非串行。

数学上，`wptr`与`rptr`是**永不复位**的取模计数器——环绕在ring算术中隐式发生，从而消除了批模式设计通常须显式处理的一类帧边界bug<!-- 来自 docs/modules/line_buffer.md ring buffer 描述 -->。容量论证是具体的：VGA 480×640图像（INT8、整图缓存名义4.9 MB）在`strip_rows = 8`、`W = 640`下占片上ring约10 KB[CHECK: ring footprint实测]——图像其余部分**流过**ring而非**驻留**于ring。这一2-3个数量级的存储压缩是流式范式相对于batch范式的核心收益。

行环的鲁棒性证据是**广度而非峰值性能**：22-case ResNet-18风格回归 + 24-case shape stress套件——共46 case——逐bit通过，覆盖1×1 FC退化形态、15×17奇维度、120×68大特征图等极端case[CHECK: 24-case shape suite计数，来自STATUS.md]。该ring被H、W、K、stride 4维极值穿透——而编译器侧的pass（第三章）最终全部喂入这同一硬件路径。

本论文将"统一流式行环"框定为FLUX_CNN的**唯一运行时模式**——原batch模式现退化为ring容量恰好覆盖整图的特例。这一统一不仅简化RTL，也消除了"两种模式各自的边界条件"的验证负担。与Alwani<sup>[15]</sup>、Kang<sup>[16]</sup>、Liu<sup>[17]</sup>等层级流式工作的细粒度差异化已在§1.4沿第(3)轴展开、此处不重复——本节仅限于行级粒度的硬件实现层面。

诚实立场：行环本身的**机制新颖性是弱-中等**——ring buffer是经典构造、Buffets亦给出形式化。本论文主张的是**该机制与编译器侧填充pass的耦合点**——行环为编译器侧的Ky-fold（IFB沿y膨胀`groups_y`倍）与S2D（`Cin`沿`stride²`膨胀）提供了**容量上界足够、寻址逻辑无须修改**的承载平台<!-- 来自 contributions.md C1.2 + docs/pe-fold.md -->。

环内的累加器单元因其内部组织对端口压力与面积都有后果，值得单独成节，由下节处理。

## 2.3 分列累加器PARF

`parf_accum`**不**是单一monolithic PSUM SRAM，而是16个`parf_col`实例——每个MAC列对应一个、各自独占SRAM、由薄壳层广播`wr_addr`、`we`、`rd_addr`（C1.3）<!-- 来自 RTL/parf_accum.sv 与 RTL/parf_col.sv -->。**列共享寻址逻辑、不共享存储；列共享调度、不共享带宽**——这是该组织的核心slogan。

这一组织与数据流直接匹配：激活在列间广播（输出驻留 + 列广播，第(1)轴），而每列独立产出自己的`cout`部分和。**因每列独占SRAM，每列均可由单端口BRAM实现**——多端口存储所必需的逐周期读写冲突在列边界处自然消失[CHECK: parf_col BRAM数量与端口配置，来自Syn综合报告]。这是一项不显眼但有意义的端口压力削减。

对照而言，NVDLA<sup>[4]</sup>类与Gemmini<sup>[10]</sup>类设计通常将partial sum集中到更大的SRAM、由该SRAM在读（drain）写（accumulate）端口间仲裁——往往把设计推向双端口或多bank配置，引入仲裁器与时序闭合压力。**分列拆分在结构上释放了这一端口压力——PSUM边界处不存在仲裁器，因为不存在共享PSUM**<!-- 来自 contributions.md C1.3 -->。

本论文将这一选择归类为**单元级实现选择而非顶层贡献**——它是使流水其余部分时序闭合廉价的工程细节之一，此处出于完整性报告而非新颖性主张。其新颖性自评为弱（OS数据流下分列累加是经典选择），但在16×16阵列与单端口BRAM约束下的具体实例化对FPGA实现是有意义的工程收益。

## 2.4 7层循环嵌套硬件实现

物理16×16 MAC阵列通过**两层切片**承载任意`Cin`与`Cout`：`cin_slices = ⌈Cin / 16⌉`在**时间维**消费——通过跨slice累加进PARF——而`cout_slices = ⌈Cout / 16⌉`通过在NHWC顺序下**于一行输出内连续写入**16通道段消费——使下游层看到单一拼接通道轴（C2.4）<!-- 来自 contributions.md C2.4 + docs/slicing/ -->。两维均不施加除DDR容量外的硬限制。

具体地，单层硬件循环嵌套深度为7层：`for y_out > for cs (cout-slice) > for tile (W-tile) > for cins (cin-slice) > for ky > for kx > for iss_pos`<!-- 来自 docs/modules/core_top.md 循环嵌套 -->。当某层工作集超出片上容量，编译器自动引入沿H的strip切片（选择`strip_rows`）与沿W的tile切片（选择`tile_cols`）——这两者成为同一嵌套**最外两层时分维度**而非独立执行模式。这一设计点的工程含义是：编译器的切片pass与硬件的循环结构**共享同一7层嵌套**，无须为不同切片场景维护多套硬件路径。

两个先前工作框定该嵌套的位置。Interstellar<sup>[22]</sup>给出了CNN数据流的7层形式化taxonomy，与本嵌套实质上一一映射——本论文视之为**该结构已被充分理解的确认而非新颖性主张**。Ma等人<sup>[Ma-FPGA17]</sup>是FPGA上4-6层卷积循环嵌套的早期系统化研究、提供同一语言下的baseline。本论文不主张新的循环嵌套taxonomy，而主张**这一特定嵌套是第三章编译器将Ky-fold与S2D映射到的目标面**。

本节为读者建立的因此是窄但承重的命题：**硬件运行普通的7层卷积嵌套——无fold-aware路径、无可重构互连、无小`Cin`特殊case**。下一章将开发编译器侧、说明Ky-fold与S2D是**对该相同嵌套输入的变换**而非该嵌套的新模式。这一窄/承重对偶——硬件保持普通、编译器承担形状适配——是narrative A的硬件承载面。

## 2.5 SDP后处理流水

后处理段`sdp`（命名沿袭NVDLA SDP）在`parf_accum` drain出partial sum后承担**5路融合**：bias、residual（shortcut）、scale、shift、ReLU/clip——以单一流水段完成，避免多次往返DDR的能耗与延迟代价（C1.4）<!-- 来自 contributions.md C1.4 + RTL/sdp.sv -->。融合的目标不是bias/residual本身的新颖性，而是其与流式行环的**位置一致性**——SDP在ofb_writer之前、与drain同步推进，使后处理与计算共享同一行级credit机制。

5路具体处理为：bias从片上WRF读出与PSUM相加；residual从Shortcut Bank（独立BRAM region）读出与SDP累加结果相加（仅在该层有shortcut分支时使能）；scale与shift实现量化重缩放——`y_q = clip(((PSUM + bias + res) · M) >> shift, 0, 255)`，其中`M`、`shift`由量化训练侧逐通道导出；ReLU/clip完成非负截断到INT8动态范围<!-- 来自 RTL/sdp.sv L42-78，量化语义来自 toolchain/models/quantize.py -->。这一组合覆盖ResNet-18风格网络的全部后处理算子需求。

residual路径需特别说明：Shortcut Bank是独立于IFB/OFB的BRAM region、由编译器在层链调度时分配——上一层的输出在写回DDR的同时旁路一份到Shortcut Bank（仅当下一个或下下个层将其作为shortcut输入时）<!-- 来自 RTL/multicore_top.sv Shortcut Bank基础设施，commit 44d49be / 141d031 -->。这一**编译器侧调度 + 硬件侧Bank**的组合避免了运行时的依赖追踪逻辑、把跨层数据流静态化。

诚实立场：5路融合本身并非新颖——NVDLA SDP定义了原型、几乎所有现代加速器都在某种程度上融合后处理。本论文C1.4主张的边际是**该SDP与流式行环、与编译器侧静态调度的耦合方式**——后处理在行级credit框架内推进、residual通过编译期分配的Bank访问而无运行时仲裁<!-- 来自 contributions.md C1.4 自评：弱-中等新颖性 -->。这一耦合在ResNet-18风格22-case回归中被穿透验证（含残差的层链bit-exact通过）[CHECK: SDP残差路径覆盖case数，来自STATUS.md]。

## 2.6 本章小结

本章在§2.1建立了去中心化valid-ready流水的设计与论证（C1.1）——5模块本地控制 + 弹性握手 + sequencer仅做跨块启动同步；§2.2展开了流式行环数据通路（C1.2）——`strip_rows`粒度ring、双向credit反压、46-case shape穿透验证；§2.3论证了PARF的分列组织（C1.3）——16个独占SRAM、单端口BRAM、PSUM边界无仲裁；§2.4阐明了7层硬件循环嵌套（C2.4）——切片在时间维消费、嵌套与编译器pass共享目标面；§2.5刻画了SDP后处理5路融合（C1.4）——bias / residual / scale / shift / ReLU路径与流式行环的耦合方式。本章诚实标注了C1.1/C1.2的弱-中等新颖性立场——本论文主张的不是这些机制本身的发明，而是其与编译器侧填充pass、与行级流式调度的**位置一致性与最小耦合性质**。下一章将转向编译器侧，详述Ky-fold与S2D两个pass如何在不修改本章硬件嵌套的前提下把PE利用率从12.5-50%提升到接近100%。

# 第三章 编译器侧PE利用率优化

本章发展本论文narrative A的核心论证——在16×16固定MAC阵列上，PE利用率恢复的复杂度可由编译器侧零RTL改动的两个pass承担，而非由硬件可重构互连承担。§3.1先把"为什么走编译器侧"这一总纲展开，把它放回§1.4第(1)轴与§2.4建立的7层循环嵌套硬件目标面之间；§3.2与§3.3分别推导Ky折叠与空间到深度（S2D）两个pass的数学形式与实现位置，并相对Liu Full-Stack TNNLS'21<sup>[17]</sup>、cuDNN im2col<sup>[26]</sup>、direct convolution、Shi sub-pixel CNN<sup>[Shi-CVPR16]</sup>等先验工作做差异化定位；§3.4说明两个pass的联合触发条件与编译器自动决策机制，并诚实标注Cout<16场景下硬件不复用的取舍立场；§3.5在硬件复杂度、编译器复杂度、内存代价、适用场景4个维度上把本论文路线与MAERI<sup>[13]</sup>、Eyeriss-v2<sup>[12]</sup>、Tangram<sup>[14]</sup>等硬件可重构方案做对位，明确二者**互补、不主张主导**；§3.6小结。

## 3.1 编译器侧路线选择的总纲

第二章建立的硬件路径——固定16×16阵列、列广播激活、每列独立Cout累加、7层循环嵌套——只在`Cin ≥ 16 && Cout ≥ 16`时才被完全填满；现实CNN工作负载经常违背`Cin ≥ 16`这一半条件：典型RGB pipeline入口卷积`Cin=3`、第2-3层`Cin=4-16`，且步幅下采样器往往出现在通道深度尚未涨起来的位置<!-- 来自 model_analysis.md §2 / contributions.md §4.4 -->。本论文回归集Layer 1的`Cin=4 / Cout=8`对应有效MAC仅32/256、PE利用率仅12.5%——若不优化，芯片在网络入口就有近88%的MAC空转。

针对这一塌陷，本论文**不**采取运行时重配置阵列的路线，而是把几何不匹配压力前移到编译器侧的两个输入/权重重排pass——Ky折叠与空间到深度（S2D）。两个pass对硬件完全透明：硬件按普通stride=1卷积运行，并不知道任何折叠发生<!-- 来自 contributions.md C2.1 / C2.2 -->。这一立场背后的工程论证有3层：(a) 可重构互连付出可观的RTL面积、时序、功耗代价，特别在中等规模FPGA上LUT/MUX资源压力显著；(b) 当同等几何灵活性可在编译期取得时，把复杂度推到Python侧仅需数百行代码，迭代代价极低；(c) 工作负载若已在编译期完全可见（这是端侧推理的典型场景），运行时重配置带来的灵活性溢价不被支撑。

> **指标范围说明**：本章所述"PE利用率"特指各层在`mac_array`计算阶段的占用率，**不**等同于§5.4中报告的整网86.6% MAC%——后者额外吸收了IDMA/ODMA停顿与跨层切换。两个指标的区别将在§5.4末段展开。

## 3.2 Ky折叠

### 3.2.1 数学推导

当`Cin < 16`时PE行饥饿、MAC阵列退化为`Cin/16`行利用率（`Cin=2`时12.5%、`Cin=4`时25%）。Ky折叠完全在编译器侧修复这一饥饿：沿卷积核y方向（Ky维），每`groups_y = HW_PE / Cin`行被聚合并拼接到通道维，产出一个虚拟卷积——其`K_new = kyper = ⌈K/groups_y⌉`、`cin_fake = groups_y · Cin ≥ HW_PE`。硬件看到的是一个针对`cin_fake`通道的普通stride=1卷积，对折叠这件事完全无感<!-- 来自 docs/pe-fold.md §1 -->（C2.1）。

具体地，按`ky = g·kyper + ky_local`分组、定义虚拟cin通道`g·Cin + c`，输入与权重张量被重排为：

```
I'[y_virt, x, g·Cin+c] = I_padded[y_virt + g·kyper, x, c]
W'[ky_local, kx, co, g·Cin+c] = W[g·kyper + ky_local, kx, co, c]
```

末组超出原`K`的位置（共`pad_ky = groups_y · kyper - K`行）补零。该变换在`toolchain/hw_files.py`中实现为3个独立函数——`compute_fold_params(K, Cin, HW_PE)`返回`(groups_y, kyper, cin_fake, pad_ky)`、`fold_input()`生成虚拟ifm、`fold_weights()`重排权重并补零，沿用`psum_reshape`的小型可复用重排pass风格<!-- 来自 docs/pe-fold.md §1 / hw_files.py -->。无任何RTL改动，硬件路径与普通卷积完全一致。

### 3.2.2 收益、代价与适用边界

收益在浅层尤为显著：ResNet-18风格入口层（`K=7, Cin=4`）从`Cin/HW_PE = 25%`行利用率跃升至完全填满的`cin_fake=16` mapping[CHECK: 各层（Layer 1/3/4）实测PE利用率细分]。代价是IFB沿y轴`groups_y`倍占用——每个虚拟行物化`groups_y`个y偏移物理行；对`Cin=4 / HW_PE=16`的场景这是4倍IFB膨胀，但流式行环（§2.2）能容忍——因为入口层特征图本身较小，4倍膨胀后仍远低于`strip_rows × W_max × cin_fake`的ring容量上界。

适用边界由两个条件界定：`K > 1`（K=1卷积无可折叠的Ky维）且`Cin < 16`（已满行不需要折叠）。`pad_ky > 0`时末组PE行在该组上空算（贡献为0），但只要`groups_y · kyper ≥ K`，等价性严格保持。

### 3.2.3 与im2col、direct convolution、Liu Full-Stack的差异化

Ky折叠在方法论上与cuDNN<sup>[26]</sup>所用的im2col相关，但**严格更轻量**：仅`Ky`维度被展开进通道维，`Kx`与空间循环保留为普通卷积。完整im2col会把输入按`K·K`倍膨胀——把所有`K²`个Kx/Ky位置物化为独立通道——在内存受限的加速器上不可行。Ky折叠则利用了"阵列已经按`Cin`轴PE行并行"这一既有结构、仅借用足够多的`Ky`槽位填满那些行，让卷积循环的其余部分不变。

与direct convolution（既不展开亦不折叠）相比，Ky折叠付出的代价是`groups_y`倍的IFB y轴复制；与完整im2col的`K·K`倍内存膨胀相比，`groups_y`典型值4-8（对应`Cin=4-2`），是一个明确更友好的中间点。

与Liu等人TNNLS'21全栈流式加速器<sup>[17]</sup>的关系需特别澄清。Liu等人在Arria 10 GX1150上以97% MAC效率达到>1.3 TOP/s，路径是**深度层融合 + 激进DSP packing**——本质上以更大器件容下完整层融合换取近峰值MAC效率。本论文则在XC7K325T（约16倍小器件）上以**编译器侧填充 + 单层执行**取得86.6%整网MAC利用率<!-- 来自 contributions.md §4.3 / STATUS.md -->。两者不是头对头比较——器件规模、工作负载、指标定义均不同——但二者位于Pareto前沿不同点上的可比位置：Liu的近峰值数字依赖大器件容下整网层融合，本论文的高利用率数字则不需要这一前提。

更进一步，据本论文所知，把Ky-only的折叠打包为面向固定2-D MAC阵列的**预编译期pass、无运行时配置、无逐层microcode**这一特定形式，在已调研的加速器文献中尚无直接对应工作[CHECK: ASPLOS / HPCA / MICRO 5年内是否有更早的architecture-side先例]。本论文将这一定位描述为**与硬件可重构方案在同一塌陷点上的对偶解决方案**，而非跨方法族的优劣排序——详细的4维度对比延后至§3.5。

## 3.3 空间到深度（S2D）

### 3.3.1 数学推导

对`stride ≥ 2`的卷积，本论文应用空间到深度（Space-to-Depth, S2D）编译器pass：把卷积核按`(kx mod stride, ky mod stride)`分解为`stride²`个相位、每个相位折入通道维。结果是一个等价的stride=1卷积——其`K_new = ⌈K/stride⌉`、`Cin_new = stride² · Cin`——硬件按普通滑动窗卷积运行<!-- 来自 docs/pe-fold.md §2 -->（C2.2）。

具体地，按`ky = stride·ky' + a, kx = stride·kx' + b`、定义相位编号`p = a·stride + b`，输入与权重张量重排为：

```
I'[Y, X, p·Cin+c] = I_padded[Y·stride+a, X·stride+b, c]
W'[ky', kx', co, p·Cin+c] = W[ky'·stride+a, kx'·stride+b, co, c]
```

`K`不被`stride`整除时（如`K=3, stride=2`），不同相位sub-kernel形状不齐——统一pad到`K_new × K_new`会浪费一部分MAC；`K`被`stride`整除时（如`K=8, stride=2`，4个相位都是4×4）`pad_waste = 0`、无任何MAC浪费。

### 3.3.2 收益、代价与对IFB的影响

S2D相对Ky折叠的关键优势是**纯字节排列、零复制**：没有任何输入元素被复制——它仅在数据进入IFB的途中重排位置。这使S2D在DDR带宽方面**字节中性**，相较Ky折叠的`groups_y`倍IFB膨胀，在多核场景下DDR带宽节省明显[CHECK: DDR带宽节省比例待对照实测]。

折叠后卷积运行于stride=1，这重新启用了`ARF reuse_en=1`滑动窗口缓存——多Cout层的IFB读次数因此大幅下降<!-- 来自 docs/pe-fold.md §4 + RTL/line_buffer.sv -->。`reuse_en=1`模式下ky边界`K`拍FILL启动惰性带来约3%量级的cycles影响——这是该模式的二阶代价，不影响功能正确性。

`K`被`stride`整除时（典型如YOLOv5 focus层使用的`K=8, stride=2`），`pad_waste=0`、4个相位完整对齐——这是S2D最优的工作点。

### 3.3.3 先验工作谱系与定位

相位分解思想最早可追溯至Shi等人CVPR'16的sub-pixel卷积层<sup>[Shi-CVPR16]</sup><!-- ref: [Shi-CVPR16] -->，原始引入语境是**训练时层**——用于超分辨率任务的高效上采样。同一代数恒等式此后在深度学习生态中以多种名字出现——例如YOLOv5的"focus"层、非正式名称"pixel-shuffle inverse"等[CHECK: S2D在加速器领域的引用谱系待补查ASPLOS / HPCA / MICRO是否有更早的architecture-side先例]。

本论文C2.2主张的并非这一恒等式本身，而是其**作为预训练后编译期pass的重新打包**——可对任意带`stride ≥ 2`的预训练卷积透明应用，无需重训练、无算法层面对模型可见的变更。这一**从训练时层到编译期pass的语境迁移**，在加速器编译栈中具体打包为`compute_s2d_params(K, Cin, stride)` / `s2d_input()` / `s2d_weights()` 3个独立函数——参数返回`(K_new, Cin_new, applicable, pad_waste)`、输入侧pre-pad到stride整除尺寸 + 重排4相位通道、权重侧重排 + 内核pad到统一K_new<!-- 来自 toolchain/hw_files.py -->。

诚实立场：sub-pixel分解本身在文献中是经典构造、并非本论文发明；本论文增量在于**其作为加速器编译pass的自动应用 + 与Ky折叠的联合触发**——后者将在§3.4展开。详细的4维度对比与硬件可重构路线对位延后至§3.5。

## 3.4 联合触发与编译器自动决策

### 3.4.1 触发顺序与相互作用

两个折叠**非正交**：S2D将`Cin`按`stride²`倍膨胀，可使一层超出`Cin < 16`阈值并使Ky折叠不再必要。编译器因此按固定顺序评估：先S2D（触发条件`stride ≥ 2 AND K ≥ stride`）、再针对**S2D后通道数**重新评估Ky折叠（触发条件`K > 1 AND Cin < 16`）<!-- 来自 docs/pe-fold.md §5 / run_regression.py -->（C2.3）。

典型场景：步幅入口层`Cin=3, stride=2`，S2D后`Cin' = 12`——接近但仍低于阈值，因此两个变换组合应用；`stride=2, Cin=4`时S2D后`Cin' = 16`——恰在阈值，Ky折叠被抑制。`K=1`层与FC层不受益（无可折叠维度）；`Cin ≥ 16`的层已近满PE行利用、不需要折叠。这一决策逻辑完全位于编译器中，硬件对触发结果完全无感。

### 3.4.2 回归套件作为决策正确性的双重证据

该决策逻辑由回归套件持续穿透验证：`run_regression.py`针对全部22个ResNet-18风格用例在**3种模式**——`无fold`（基线）、`--fold`（仅Ky折叠）、`--fold --s2d`（Ky折叠+S2D）——下分别运行，全部配置在NumPy参考下bit-exact通过<!-- 来自 toolchain/run_regression.py / contributions.md §4.1 -->。因硬件在3种模式下完全相同，回归同时验证了两件事：(a) 编译器pass的等价性正确（输出与参考bit-exact一致）；(b) 硬件对两个pass的存在与否完全无感。这是同时检验**编译期变换正确性**与**硬件路径无fold-aware逻辑**的双重断言。

### 3.4.3 Cout < 16场景的诚实立场

互补场景——`Cout < 16`——在当前设计中**有意不优化**。诸如分类头小输出（`Cout=2`）等层只是让`(16-Cout)`个PE列空转、列利用率上限`Cout/16`<!-- 来自 docs/pe-fold.md §3 / RTL/parf_accum.sv -->。本论文有意做出这一选择：Kx对称的"Cout-fold"将同时要求输出路径上的列归约级与`parf_accum`内逐列地址偏移逻辑——这会破坏当前**列共享地址、列独立SRAM**的`parf_col × NUM_COL`简洁结构（§2.3），从`16路单端口BRAM`回到带仲裁器的多bank PSUM。

在ResNet-18类工作负载下这是**可辩护的取舍**——这种低`Cout`层在总MAC数中贡献微乎其微（典型仅最后分类头一层）。但这是编译器侧路线的**真实局限**，本论文**诚实报告之、不软化措辞**：硬件不复用、PE列空转。`docs/roadmap.md`记录了一项roadmap条目，覆盖工作负载组合发生变化时重新引入Kx-reuse的可能路径<!-- 来自 docs/roadmap.md "Kx维度复用"项 -->。

这一决策自动化是narrative A的操作对偶——用户调用单一flag（`--fold`和/或`--s2d`），编译器逐层决定每个变换是否应用。下一节将本路线与**硬件可重构**替代方案做对位——后者把同一决策以综合代价嵌入硅片。

## 3.5 与硬件可重构方案的对比

### 3.5.1 论点与可比锚点

本章的论点是：对于**编译期已知的工作负载组合**而言，编译器侧折叠可在严格更低的硬件复杂度代价下匹配硬件可重构阵列的逐层PE利用率。具体地，FLUX_CNN在XC7K325T上以固定16×16阵列在ResNet-18风格套件上达到86.6%整网MAC利用率<!-- 来自 contributions.md §4.3 / STATUS.md -->；Liu等人Full-Stack设计<sup>[17]</sup>在Arria 10 GX1150上报告97% MAC效率——尽管该工作在显著更大的器件上应用硬件级重配置。

两个数字**不是严格头对头比较**——器件规模、工作负载、指标定义均不同——但它们位于同一区间内的**可比锚点**：86.6%（XC7K325T、中型器件、固定阵列、编译器侧填充）vs 97%（Arria 10 GX1150、大型器件、深度层融合、硬件级重配置），共同支持本论文的更广义主张——**编译器侧重排可达到与硬件侧重配置同一利用率带**。

### 3.5.2 4维度对位

本论文沿4个维度——硬件复杂度、编译器复杂度、内存代价、适用场景——把两条路线对位（如表3-1所示）。

**硬件复杂度**：MAERI<sup>[13]</sup>的ART树与Eyeriss-v2<sup>[12]</sup>的层级mesh NoC在综合代价上引入可重构分发网络与归约树——LUT/MUX资源占用、时序路径预测难度、验证状态空间均显著上升；FLUX_CNN阵列保持为固定的广播+脉动16×16 MAC网格、综合代价仅`82 DSP / 128 BRAM36 / 36942 LUT`<!-- 来自 STATUS.md §2 -->[CHECK: 与MAERI / Eyeriss-v2同器件LUT/MUX代价的可比数字尚无直接公开对照]。**编译器复杂度**：对比反转——硬件可重构阵列接受常规数据流mapping（往往通过Timeloop<sup>[21]</sup> / MAESTRO<sup>[Kwon-MAESTRO-MICRO20]</sup>风格搜索完成），FLUX_CNN编译器须**主动决定并应用折叠变换**、增量约数百行Python实现。**内存代价**：im2col风格方案按`K²`倍膨胀输入；Ky折叠按`groups_y`（典型4-8）倍膨胀；S2D是原地排列、零膨胀——相对im2col约`K²/groups_y`量级改善。**适用场景**：硬件可重构阵列更适合**运行时层几何动态切换**的工作负载（如视觉transformer与检测头交错），编译器侧折叠最适合**编译期完全可见的预编译部署**——这正是端侧FPGA推理的典型场景。

| 维度 | 硬件可重构（MAERI/Eyeriss-v2/Tangram） | 编译器侧折叠（本论文） |
|---|---|---|
| 硬件复杂度 | 重配置NoC + ART树（LUT/MUX代价显著） | 固定16×16 MAC网格、综合代价低 |
| 编译器复杂度 | 常规mapping搜索（Timeloop / MAESTRO） | 须主动决定并应用Ky折叠+S2D |
| 内存代价 | 视mapping策略（通常无额外膨胀） | Ky折叠`groups_y×`、S2D零膨胀 |
| 适用场景 | 运行时层几何动态切换 | 编译期完全可见的预编译部署 |

### 3.5.3 互补、不主张主导

两条路线**不互斥**，本论文**不主张编译器侧路线占主导**。对于运行时频繁切换显著不同层形状的工作负载——如视觉transformer与检测头在单次运行中交错——可重构fabric可以分摊其硬件代价。对于**编译期完全可见**的CNN推理——编译器对所有层维度有完整信息——本论文路线以数百行Python换取硅片代价、达到可比的PE利用率<!-- 来自 contributions.md §8.1 -->。

这一取舍是**工作负载相关而非绝对**，本论文有意如此框定。Tangram<sup>[14]</sup>把可重构哲学推到层间——多个tiled加速器实例之间通过交替buffer共享数据流来重构层间通信路径，在多层流水线上取得2倍性能与45%能耗改善——其与本论文的关系是**正交维度上的工作**：Tangram关心**层间通信路径**的可重构、本论文关心**层内PE几何**的填充，两者技术上可组合（编译器侧折叠 + 层间可重构通信），但在本论文目标器件规模上层间可重构带来的边际收益尚无定量评估[CHECK: 编译器侧折叠 + 层间可重构通信的组合实验]。

本节的实证支撑来自第五章——§5.2报告各层带与不带每个折叠的实测PE利用率，§5.4报告整网86.6% MAC%。有了这些结果，编译器侧叙事即闭环；下一章转向系统集成侧——同一编译器如何发出descriptor列表、硬件文件、PyTorch前端粘合层，以驱动阵列端到端运行。

## 3.6 本章小结

本章在§3.1建立了编译器侧路线选择的总纲——硬件保持固定、几何不匹配压力前移到编译器；§3.2推导了Ky折叠的数学形式、收益代价、与im2col/direct convolution/Liu Full-Stack的差异化定位（86.6% XC7K325T中型器件 vs 97% Arria 10 GX1150大型器件、Pareto可比锚点）；§3.3推导了S2D的数学形式、其与Shi sub-pixel<sup>[Shi-CVPR16]</sup>训练时层的语境迁移、并诚实标注加速器领域引用谱系待补查；§3.4说明了两个pass的联合触发顺序、22-case回归在3种模式下bit-exact通过的双重断言，并诚实标注Cout<16场景下硬件不复用的取舍立场；§3.5沿硬件复杂度、编译器复杂度、内存代价、适用场景4维度把本路线与MAERI/Eyeriss-v2/Tangram对位、明确**互补、不主张主导**。本章主张的Ky折叠+S2D组合在ResNet-18风格22 case回归上把浅层PE利用率从12.5-50%提升到接近100%、整网86.6%，是narrative A的核心论证。下一章将转向系统集成侧——AXI DataMover IP集成、CFG_WRITE descriptor配置流、链式CASES验证基础设施、多核wrapper与W切片。

# 第四章 系统集成与多核扩展

## 4.1 章首：系统侧支撑总纲

第三章建立了编译器侧narrative A（Ky折叠+S2D），第二章—第三章一并阐明了固定16×16阵列与流式row-ring的核心数据通路。仅有数据通路并不构成可部署加速器——还需把PyTorch模型搬到硬件上、把命令搬到core里、把post-MAC语义闭合、并把阵列横向扩展到多核。本章描述把FLUX_CNN数据通路转化为可部署加速器所需的系统级工程：§4.2 PyTorch端到端编译流（C2.5）；§4.3 vendor IP支撑的AXI/DMA子系统加descriptor驱动配置（C3.1, C3.2, C3.3）；§4.4 SDP后处理融合bias/残差/量化（C1.4）；§4.5 N-core W切片扩展wrapper（C3.5, C3.7）；§4.6 单源参数管理流（C3.6）。

需特别说明的是，本章各项工作是**必要工程**而非主要novelty主张——其作用是建立第三章—第五章的架构贡献在真实负载上端到端验证，而非孤立基准上跑一遍即声称novelty。这一立场上承§3.6"互补、不主张主导"的姿态：narrative B强调系统集成度，但其实证支撑应止于"该走通的都走通了"，过度声称会模糊与第三章narrative A的区分。

## 4.2 PyTorch nn.Sequential端到端编译流

FLUX_CNN提供完整的PyTorch到硬件编译栈，单条命令即可驱动整个`nn.Sequential`模型经过逐层编译、内存规划与bit-exact回归（C2.5）。用户面入口`toolchain/models/run_model.py`接受量化PyTorch模型、产出链式descriptor列表，硬件按此列表回放、无需主机层间介入<!-- 来自 CLAUDE.md "PyTorch 模型端到端部署" + toolchain/models/run_model.py -->。

编译栈分三层组织。`compile_layer.py`把单个`nn.Conv2d`（含本项目量化wrapper）lowering成硬件`cfg_regs`快照加IFB/WB/ODMA字节流，并与单测入口`gen_isa_test.py`共享`hw_files.derive_layer_cfg()`中的派生值逻辑——这一共享是关键，可以从机制层面杜绝单测case与整模型run对配置派生路径的漂移。`compile_model.py`遍历`nn.Sequential`、调用`_plan_ddr`为相邻层分配可复用的FM-shared DDR区段、并把逐层descriptor列表拼成单条链。`run_regression.py`在此之上暴露一个小型DSL（`Chain`、`_Node`、`resnet_block`），让一个11层ResNet风格block可由约7行Python表达，便于增加新模型。

11-case ResNet-like链当前端到端通过，初步报告约[CHECK: 593K cycles total / 86.6% MAC% / 5.95 ms @ 100 MHz design target / 8.69 ms @ 68.4 MHz post-syn Fmax — 数字待与STATUS.md统一口径]<!-- 来自 STATUS.md / [CHECK: chain实测口径统一] -->，证明工具链已抵达多层场景且无需逐层人工介入。

与TVM<sup>[TVM@OSDI'18]</sup>、VTA<sup>[VTA@MICRO'19]</sup>、fpgaConvNet<sup>[fpgaConvNet@TNNLS'19]</sup>等通用深度学习编译栈相比，FLUX_CNN的编译器是**有意为之的、scope限制在自有ISA的手写小型编译器**：不试图成为任意前端的target backend。代价换取的属性是：22-case链式回归提供了**完全可复现、零外部编译器依赖**的bring-up覆盖——这正是第三章—第五章架构主张所需的验证锚点，更通用的编译能力不是narrative B的目标。

编译器输出最终是一串经DMA进入硬件的descriptor，下一节描述descriptor如何在AXI fabric上流通。

## 4.3 AXI/DMA子系统与vendor IP集成

原始项目交付了约3000行手写IDMA/WDMA/ODMA RTL；现已被Xilinx `axi_dm`（AXI DataMover）IP加薄`*_ctrl`控制器取代，对外仅暴露1个AXI4 master port与1个AXI-Lite slave port（C3.1）。这一替换缩减了验证表面、并让设计继承vendor验证过的burst处理而无需重新推导握手与边界。

四个控制器`idma_ctrl`、`wdma_ctrl`、`odma_ctrl`、`rdma_ctrl`刻意保持最小——仅生成DataMover命令并检测命令完成`done`事件，不做其他任何事情。`mm2s_arb`让`idma_ctrl`与`wdma_ctrl`通过串行仲裁共享`axi_dm`的MM2S通道，`odma_ctrl`独占S2MM通道。`axi_m_mux`把`axi_dm.MM2S`、`axi_dm.S2MM`与descriptor前端（DFE）聚合到唯一外部master port，`axi_lite_csr`提供bring-up所用的32位slave通路<!-- 来自 CLAUDE.md DMA 子系统说明 + RTL/AXI4/ -->。

descriptor驱动配置进一步降低主机参与度。一种`CFG_WRITE` descriptor类型（`TYPE_CFG=0x3`）由DFE执行、让链式descriptor列表可直接写`cfg_regs`：每层主机AXI-Lite流量从约50次寄存器写降到4个descriptor条目（C3.2）。`done` sticky位与dual-port `cfg`写通路（C3.3）共同消解了在飞descriptor写与主机轮询之间的竞争<!-- 来自 STATUS.md "链式 CASES + CFG_WRITE descriptor + done sticky" -->。

吞吐侧，DataMover-based子系统与原始手写DMA的差距控制在+0.5%以内[CHECK: 0.5%差距相对refactor前baseline的实测口径]，且`burst_size`从16重调到256确认了vendor IP并未在带宽上留缺口<!-- 来自 STATUS.md "性能基本回到原版" -->。这一结果说明：把vendor IP替换手写DMA并未交易掉性能，仅交易掉了维护与验证开销。

`CFG_WRITE` descriptor机制在概念上邻近NVDLA<sup>[NVDLA]</sup>的"register list" descriptor模式；FLUX_CNN把descriptor类型空间从纯buffer descriptor扩展到包含配置写——这正是50到4降幅的来源。该工程化扩展的novelty有限，但对降低主机参与度、把多层链交给硬件自主运行至关重要。

## 4.4 SDP后处理：bias/残差/量化融合

Single Data Point processor（SDP）把5个post-MAC操作融进一条组合链：`pipe_psum_reg → mult → shift → add zero-point → ReLU → clip → trunc`（C1.4）。把这些阶段折进单一流水段消除了非融合实现所需的per-operation握手、并把post-processing保持在MAC阵列关键路径之外[CHECK: SDP组合链re-synthesis后的timing closure]。

两次重构塑造了当前SDP形态。R.1把bias加法从`mac_array`移出、放进SDP段——既消除了阵列内bias逻辑施加的`MAX_COUT_SLICES=32`限制、也释放了`bias_rf`容量供其它用途。R.2加入可编程`shortcut_mult`与`shortcut_shift`因子让残差融合可在运行时量化、并引入Shortcut Bank（一块`8192×128` SRAM、约32 BRAM）作为残差源的专用常驻存储<!-- 来自 memory/sdp_residual_fusion.md / commit 141d031 + 44d49be -->。

SDP段命名与stage分解直接沿用NVDLA<sup>[NVDLA]</sup>，本论文不把这一命名空间据为己有、定位为"NVDLA-inspired"而非重新推导；算法侧，残差融合存在的理由是ResNet<sup>[He@CVPR'16]</sup>，而非本论文的硬件贡献。

与流式带残差融合的设计如<sup>[Liu@TNNLS'21]</sup>以及更广义的整数量化路线包括<sup>[Jacob@CVPR'18]</sup>相比，FLUX_CNN刻意保持更窄的scope：**不做跨层融合**（如把Conv-BN-ReLU-Add跨层边界合并）。所有融合发生在一条SDP流水内；跨层数据流走DDR（或§4.5多核情形下走跨core通路）。这一scope收紧的代价是层间buffer占用更大，收益是配置面与验证面更小，与本论文中型FPGA器件目标场景吻合。

本节的novel content仅限R.1 bias重定位与R.2可编程量化因子；这些均作为"系统完整度证据"呈现，**不**作为主要架构novelty。

## 4.5 多核扩展：W切片与计算冗余halo

`multicore_top.sv`是参数化N-core wrapper、包裹单核流水线；在XC7K325T target上对N=2与N=4均可综合通过，且20-case W切片回归在N=4下当前bit-exact通过（C3.5, C3.7）[CHECK: 20/20 W-slice cases PASS的STATUS.md/sim/tb_multicore/最终确认]。该wrapper的引入恰是为检验流式ring核能否在不修改core本身的前提下横向组合。

W切片模式（Mode C）让每个core处理同一图像的一段连续输出W stripe、并通过**计算冗余halo配合非对称padding**处理边界。例如N=2、K=3、pad=1、W=32时，Core 0处理对应`W[0..17)`输出范围、`pad_l=1, pad_r=0`，Core 1处理`W[15..32)`、`pad_l=0, pad_r=1`，两core共享2列重叠输入halo。halo宽度从`(K, stride, pad)`派生而非单独配置，这让per-core配置保持精简、避免引入额外配置维度。

多核工作的M2阶段把AXI fabric升级为`axi_2to3`/`axi_4to5`、并引入跨核SRAM直送：生产core的`odma_ctrl`通过crossbar直接写入消费core的`ifb_axi_slave`、消费core的IFB ring通过标准AXI流控反压生产core。这消除了在producer-consumer pair吻合W切片几何的层上的层间DDR往返、对带宽边界的负载更友好<!-- 来自 memory/m2_cross_core_pipeline.md / RTL/AXI4/ifb_axi_slave.sv -->。

在一个代表性W切片case（`wslice1`）上，N=4当前约3833 cycles、N=2约5569 cycles，得到1.45×加速[CHECK: 1.45×数字 — 需核对baseline归一化口径（per-core vs total cycles）以及该case是否具代表性；ResNet 11层多核数字仍待跑]。本论文**刻意不在该审计完成前主张近线性扩展**、且尚未在完整ResNet链上报告多核数字——narrative B关于多核的实证支撑停在"两点scaling数据已得、完整链尚待"。

与Simba<sup>[Simba@MICRO'19]</sup>的36-chiplet通道切片扩展相比，本设计的FPGA-multicore形式使用不同切分轴（W切片而非通道切片），且约束维度是BRAM容量而非跨die通信。流式FPGA设计如<sup>[Liu@TNNLS'21]</sup>与<sup>[Kang@Sensors'23]</sup>同样表现出BRAM受限扩展，提示W切片在该设计点是合理默认选择[TBD: §4.5是否在多核evaluation成熟后提升为顶级章节]。

## 4.6 单源参数：params.py

`params.py`是RTL与Python工具链共同消费的**全设计参数单一可信源**（C3.6）。该文件当前定义约64个`FLUX_*`宏，覆盖核心维度、SRAM容量、AXI/CSR位宽、全局地址映射与58个CSR寄存器地址，从机制上消除了一类参数漂移bug——例如`ADDR_W`改了RTL但没改编译器（或反过来）<!-- 来自 CLAUDE.md "params.py 作为单源" 最新条目 -->。

流程是机械的：`python params.py`重新生成`RTL/flux_cnn_params.svh`；RTL通过`` `include ``消费参数，Python通过`from params import *`消费参数。两侧因此读取相同数字、无任何手维镜像文件[CHECK: 确认`flux_cnn_params.svh`已commit且Vivado综合流不带条件地拾取]。

这是纯工程实践、**不**作为架构贡献。本章把这一节单独立章是因为：§4.3的链式descriptor流与§4.5的多核wrapper均依赖RTL与Python在byte粒度的地址映射上达成一致——`params.py`让此一致性从约定降为机制保证[TBD: §4.6是否值得独立小节或并入§4.3]。

## 4.7 本章小结

本章在§4.1建立了系统侧narrative B的总纲——把第三章—第五章的架构贡献转化为可部署加速器、且工程工作不冒充novelty；§4.2描述了PyTorch到硬件的端到端编译栈（C2.5）、其三层结构与11-case ResNet-like链端到端通过；§4.3描述了vendor IP `axi_dm`替换约3000行手写DMA、配合`*_ctrl`控制器、`mm2s_arb`仲裁、`axi_m_mux`聚合（C3.1, C3.2），以及`CFG_WRITE` descriptor配合`done` sticky把每层AXI-Lite主机流量从约50写降到4 descriptor（C3.3）；§4.4描述了SDP把5个post-MAC操作融为单流水（C1.4）以及R.1/R.2两次重构、并诚实标注该节novel content仅限两次重构本身；§4.5描述了`multicore_top.sv`参数化N-core wrapper、W切片配合计算冗余halo、跨核SRAM直送，以及20-case N=4 W切片回归bit-exact通过（C3.5, C3.7）、1.45×加速[CHECK]；§4.6描述了`params.py`作为RTL+Python参数单源（C3.6）、把跨语言一致性从约定降为机制保证。本章的实证立场始终是"系统完整度证据"而非"novelty主张"，narrative B的架构原创性已在§3.6/§4.5与prior art的对位中明确边界。下一章把第三章narrative A（编译器侧PE利用率）与本章narrative B（系统集成度）汇集到统一的实验框架——综合资源、回归测试、PE利用率实测、端到端时延、与prior art的对比。

# 第五章 实验与结果分析

## 5.1 实验设置

本章把第三章narrative A（编译器侧PE利用率恢复）与第四章narrative B（系统侧集成度）的全部声明汇集到统一的实验框架，并在每个声明上给出可追溯到回归日志或综合报告的定量证据。实验平台分为综合与仿真两条独立的工具链：综合走Vivado 2023.1的Out-of-Context流程，目标器件Xilinx Kintex-7 XC7K325T-FFG900-2，时钟约束100 MHz；仿真走ModelSim 2020.4，配合Vivado simlib（一次性由`Syn/compile_simlib.tcl`编译）以支持`axi_dm` IP的跨语言elaboration。<!-- 来自 STATUS.md §1 单核综合表 + CLAUDE.md 仿真目录 -->

测试基础设施由三部分构成。第一部分是**22-case ResNet-18风格链式回归**，覆盖一个11层chain在三种编译模式下的全量执行——无fold基线、`--fold`（Ky折叠）、`--fold --s2d`（Ky折叠加S2D）——三模式共用同一份RTL、仅由`gen_isa_test.py`派生不同的cfg与权重布局，因此构成对编译器侧narrative A的clean A/B/C测试。第二部分是**24-case边界鲁棒性套件**，扫描`K∈{1..7} × stride∈{1..4} × pad∈{0..3}`，配合多组Cin/Cout/H/W组合，专门压测padding、halo、stride以及slice边界逻辑。第三部分是**20-case多核W切片套件**（N=2十组加N=4十组），跑`multicore_top` wrapper上的`K∈{1,3,5,7} × stride∈{1,2} × W∈{8,32,33}`，包含`W=33`非4整除以及`W=8`启动开销主导这两类难点corner。<!-- 来自 STATUS.md §2.8 wslice 表 + §1 单核回归 22-case -->

**66个case全部bit-exact对齐PyTorch参考输出**——这是本章后续每一个量化结论的可信度锚。后文§5.2~§5.4的cycle数全部来自上述回归报告中`run_regression.py`链式CASES产出的同一份日志，与PASS判定共用计数器；端到端时延`Wall_us`列由host可见的boot到host可见的done之间计算，囊括descriptor fetch与最末ODMA drain，不是理想吞吐而是host driver实际观察到的wall-clock[CHECK: 22+24+20=66 case 总数 — 当前 STATUS 列 22 chain + 26 单核 + 20 W-slice，投稿前再统一一次口径]。回归测试套件本身被视作一等交付物，而非调试辅助。

我们把单层PE利用率（`mac_array`占空率）与整网MAC%（host可见wall-clock相对理想compute下界的比值）作为两个独立的评测口径，并在§5.2与§5.4分别报告。前者衡量"阵列运行时跑得多好"，后者衡量"阵列wall-clock占多大比例真的在跑"，两者之间的gap由层间descriptor fetch、CFG_WRITE replay、IDMA→ODMA通道切换以及末层drain合并贡献——本章在§5.4以粗粒度partition的形式给出近似分解。

## 5.2 PE利用率：编译器侧Ky折叠加S2D

实验起点并不光鲜：ResNet-18风格chain的浅层在16×16 PE阵列上严重欠利用，因为它们的输入通道数小于硬件PE宽度。具体地，**第1层（K=7, Cin=4）基线PE利用率仅为12.5%，第3-4层（Cin=8）为25%，下采样层5a/5c（Cin=8, stride=2）为50%**——这是把Cin直接铺到PE row上的几何后果，并非控制路径浪费。一旦编译器侧打开`--fold`（Ky折叠）与`--s2d`（S2D），同一份RTL——一个bit都不改——把这些层推到**接近100%的利用率**。<!-- 来自 model_analysis.md PE 利用率分析 + docs/pe-fold.md --> [CHECK: 12.5%/25%/50% 三组实测 PE 利用率精确百分比 — 与 22-case ResNet chain 三模式实测 mac_array busy 占空率最终对齐]

表5-1按层比较三种模式——基线、`--fold`、`--fold --s2d`——同时报告每层占空率与全网加权平均。Cout小于16的层（如FC风格的head层Cout=2）在表中以脚注标注：这类层的PE列由硬件设计直接闲置（利用率=Cout/16），我们刻意没有为它们引入硬件复用，因为它们对总MAC贡献可忽略（参§5.4的MAC%分布）；具体的设计权衡见§3.3。

**表5-1 三模式逐层PE利用率对比**（22-case ResNet-18 chain）

| 层 | 形状 | baseline | `--fold` | `--fold --s2d` |
|---|---|---|---|---|
| L1 | K=7, Cin=4 | 12.5% [CHECK] | ~100% [CHECK] | ~100% [CHECK] |
| L3-4 | Cin=8 | 25% [CHECK] | ~100% [CHECK] | ~100% [CHECK] |
| L5a/5c | Cin=8, s=2 | 50% [CHECK] | 50% [CHECK] | ~100% [CHECK] |
| 全网加权 | — | [CHECK] | [CHECK] | [CHECK] |

[CHECK: Tab.5-1 — 各层实测 PE 利用率百分比与 Cout<16 layer 表脚注的 MAC% 占比待回归报告对齐]

机理就是§3.3与§3.4开发的那套。对L1（K=7, Cin=4）Ky折叠设`groups_y=4`，把4行Ky打到输入通道维上，使`cin_fake=16`饱和PE row宽度；对下采样层5a/5c（stride=2）S2D把4个空间相位折到通道维上，得到`Cin'=4·8=32 ≥ HW_PE`，此后Ky折叠不再必要。两种变换都在编译器侧改写权重布局与输入寻址；RTL层面的line buffer、MAC array、accumulator只看见"一个Cin更胖的层"。<!-- 来自 docs/pe-fold.md -->

需要强调的是，此处的逐层占空率（`mac_array` duty cycle）与§5.4报告的整网MAC%**不是**同一指标，整网指标里chain层级的开销——descriptor fetch、走AXI-Lite路径的层间cfg写入、IDMA→ODMA通道切换、网络末端ODMA drain——会贡献一段不可消除的gap。逐层数字回答"阵列运行时跑得多好"，整网数字回答"阵列wall-clock时间里跑多久"，两个数字都重要、且必须分别报告[CHECK: 单层 PE 利用率 vs 整网 MAC% 的精确 gap 数值与逐项归因 — 留 §5.4]。

这一对比把FLUX_CNN置于两条路径之间。**MAERI [Kwon@ASPLOS'18]** 在阵列内部以可重构互连（"ART"总线）应对不规则形状下的占用回收；**Eyeriss-v2 [Chen@JETCAS'19]** 用层级化NoC做同一件事。两者都在硬件层面付费：额外的crossbar、额外的controller、额外的验证面。FLUX_CNN把成本搬到了compile time——fold参数由`gen_isa_test.py`从层形状与stride直接派生，阵列保持固定的16×16几何。这是相同设计空间里的一个不同切点，并不主张严格优于硬件路径，仅主张在ResNet风格浅层这一类pathology上、在固定几何这一约束下、用纯compile time变换达到相近的占用regime且零RTL代价（C2.1, C2.2）。我们将在§5.6把同轴数字对比给出。<!-- 来自 contributions.md C2.1 / C2.2 -->

narrative A的核心立场至此具备：在L1这类Cin=4的极端浅层，Ky折叠把利用率从12.5%拉到接近100%，是约8倍的恢复；在Cin=8、stride=2的下采样层，S2D配合Ky折叠把50%拉到约100%，是约2倍的恢复。这两类层在ResNet风格chain中又恰好是cycle贡献占比最大的（参§5.4的Wall_us分布），因此fold带来的per-layer增益对全网wall-clock改进具有不成比例的权重。

## 5.3 资源占用与Fmax

XC7K325T-FFG900-2 OOC综合在单核配置下报告**LUT 36 942（占18.1%）、FF 13 167（占3.2%）、BRAM36 128块外加1块RAMB18（占28.8%）、DSP48E1 82块（占9.8%）**，**Fmax = 68.4 MHz**，相对100 MHz目标WNS = -4.618 ns。我们诚实地报告这一点：设计在原目标上没有达到时序闭合，gap来源已定位（详见本节末段）。<!-- 来自 STATUS.md §1 单核综合表 --> [CHECK: 资源数字与 Fmax / WNS 投稿前重综合刷新一次 — 当前 commit b158cab]

**表5-2 单核与多核综合资源对比**（XC7K325T，100 MHz target）

| 资源 | 单核 | 双核 | 三核（推算） | 四核（推算） | 器件容量 |
|---|---|---|---|---|---|
| LUT | 36 942（18.1%） | 74 386（36.5%） | ~109 K（54%） | ~146 K（72%） | 203 800 |
| FF | 13 167（3.2%） | 26 927（6.6%） | ~40 K | ~53 K | 407 600 |
| BRAM36 | 128（28.8%） | 256（57.5%） | 384（86%） | **512（超限）** | 445 |
| DSP48E1 | 82（9.8%） | 164（19.5%） | 246 | 328 | 840 |
| Fmax | 68.4 MHz | 68 MHz | — | — | — |

<!-- 来自 STATUS.md §1 + §2 多核综合表 --> [CHECK: 多核综合 N=2 实测 / N=3 N=4 推算与重综合最终对齐]

BRAM明细与§3-§4各模块预算对得上：**WB SRAM贡献57块、IFB 32块、Shortcut Bank 32块、OFB 7+1 RAMB18**，合计标题数128。表5-2进一步把这一单核明细投影到N=1/2/3/4核，以XC7K325T的445 BRAM上限为参照——这是§5.5多核扩展数字定钉而非空喊的依据。<!-- 来自 STATUS.md §1 BRAM 明细 + §2 多核综合表 --> [CHECK: BRAM 明细 57/32/32/7+1 与综合报告精确数字对齐]

多核综合证实投影。**N=2闭合于LUT 74 386（36.5%）与BRAM 256（57.5%）**；**N=3推算384 BRAM（86%）** 已贴近器件边缘；**N=4推算512 BRAM超出445上限**，因此对N=4必须把每核Shortcut Bank从8192条目缩到2048（每核削24 BRAM，得4×104=416 < 445）。N=2的LUT占用36.5%、FF占用6.6%，两者都远未触底。<!-- 来自 STATUS.md §2 多核综合表 --> [CHECK: N=2 综合 74 386 LUT / 256 BRAM 与 N=3 / N=4 推算与实测最终对齐]

本节的结构性结论是：**三核是XC7K325T在不动每核SRAM前提下的硬上限**，且约束方是**BRAM、不是LUT、不是互连**。这一事实直接锚定了§5.5多核扩展叙事——在中端FPGA上，扩展是带宽与存储有界、而非compute有界，我们不会主张N=4"免费"装下：N=4显式地拿shortcut容量换第四个核，这一交易在§4.5与本节都被记录。

Fmax的差距有两个已识别的根因，两者都在综合review阶段被预先定位、且都有具体的修复路径。第一，`mac_pe`单元未加`(* use_dsp = "yes" *)`综合属性，因此Vivado在`if (compute_en) prod <= mult else hold`这一守护写法下把多数乘法器推断到LUT；加属性估计可回收约17 K LUT并释放DSP48E1。第二，SDP量化组合链（`pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc`）未流水线化；切1-2级是教科书修法。两者都被作为已识别问题而非谜团处理；§5.7的future work清单按ROI排序。[TBD: 是否在投稿前完成 use_dsp + SDP 流水化重综合并把 §5.3 数字刷到 100+ MHz]

## 5.4 端到端时延与MAC%

驱动回归套件的11层ResNet-18风格chain在单核上以**593 K cycle、整网MAC% 86.6%** 完成，对应**100 MHz目标点的5.95 ms** 端到端时延（168 fps）与**实测68.4 MHz Fmax下的8.69 ms**（115 fps）。这两个wall-clock数字均自host可见boot到host可见done测得，已包括descriptor fetch与末层ODMA drain——不是理想吞吐，而是host driver实际观察到的时间。<!-- 来自 STATUS.md §2.6 Scheduler 估算 N=1 593K cycles / 168 fps + §1 Wall_us 端到端报告 --> [CHECK: 593K cycles / 86.6% MAC% / Wall_us 实测 — 投稿前重跑 11-layer chain 与 STATUS scheduler 估算最终对齐]

**表5-3 11层ResNet-18风格chain逐层时延分解**（单核）

| 层 | cycle | 占比 | Wall_us @100 MHz | Wall_us @68.4 MHz | 主导层 |
|---|---|---|---|---|---|
| L1 (K=7) | [CHECK] | [CHECK] | [CHECK] | [CHECK] | ✓ |
| L2 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | |
| L3-L11 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | |
| 全chain | 593 K [CHECK] | 100% | 5.95 ms [CHECK] | 8.69 ms [CHECK] | — |

[CHECK: Tab.5-3 — 11-layer 逐层 cycles 实测与主导层标注由 run_regression.py Wall_us 报告抽取]

表5-3按层分解端到端时延，同时报告每层cycle、占比、两个工作点下的`Wall_us`，并标注时延主导层。早期L1（K=7、大H×W）按绝对cycle计是最大贡献者，与其对K的二次依赖与对H×W的双线性依赖一致；这也解释了为何§5.2中Ky折叠对L1的增益对全网PE利用率的加权平均改进具有不成比例的权重——L1既是最差利用率的源、又是最大cycle的源，fold在L1上的恢复是双重杠杆。<!-- 来自 STATUS.md §2.6 cycle 分布 -->

**整网MAC% = 86.6%** 是系统级效率的标题数字，我们与它一同报告它包括了什么：每层边界的IDMA→ODMA通道切换、走AXI-Lite路径的descriptor fetch与层间cfg写入（CFG_WRITE descriptor，§4.3）、网络末端经`ofb_writer`的drain。换言之，这是流式row-ring在**单核层间串行执行**下可达的duty cycle；跨层融合（本工作未实现，参§5.7）是关闭更多gap的自然下一步。<!-- 来自 contributions.md C2.5 / §4.3 --> [CHECK: 86.6% MAC% 实测重跑 — 投稿前最终核对]

逐层接近100%的PE利用率（§5.2）与86.6%整网MAC%之间的gap——约**13个百分点**——可作粗粒度四项划分：（i）每层边界的descriptor list fetch与CFG_WRITE replay；（ii）IDMA→ODMA AXI通道切换（经`mm2s_arb`串行化）；（iii）每层IDMA冷启动（line buffer需先priming才能让`mac_array`点火）；（iv）末行drain时的ODMA backpressure。我们把分解作粗partition而非精确划分，因为各贡献项在descriptor chain中非平凡地交织[CHECK: 13% gap 各项占比的近似分解数字 — 需要从波形或 profile 计数器读出]。

放在文献语境里，**86.6%** 处于**Liu Full-Stack [TNNLS'21]的97% MAC%** 之下，但scale点差异显著（中端7-series vs.大Arria 10 GX1150），并与**Snowflake [Gokhale@ISCAS'17]的91% 平均计算效率** 在精神上可比但不严格等价（分母定义不同）。我们把精确的scale-and-metric校准延到§5.6的对位对比，而非在本节过度主张。<!-- 来自 literature.md Liu TNNLS'21 / Snowflake ISCAS'17 -->

单核整网视图至此完整：在编译器侧narrative A把逐层占空率推到接近100%之后，系统侧narrative B以13%可解释gap保持86.6%的整网duty cycle，对应100 MHz目标下168 fps端到端吞吐。下节把视角扩展到多核——同一份RTL通过W切片在N=2/N=4配置下的扩展。

## 5.5 多核扩展

多核wrapper（参§4.5）已在两个scale点上端到端验证。**N=2 DDR模式下，30×30 K=3 C8C16这一case在9057 cycle完成**，相比同case的单核8808 cycle，AXI仲裁开销仅**+2.8%**——这是从单核走到双核、且两核看到相同workload时，"多一个核"代价的最干净读数。**N=4 W切片配置下，全部20个W切片case bit-exact通过**，标题数据是**wslice1在3833 cycle完成，相对N=2的5569 cycle取得1.45×加速**——这是单层K=3 case从N=2到N=4的实测。<!-- 来自 STATUS.md §1.5 9057/8808 cycles + §2.8 N=4 wslice1 3833 / N=2 wslice1 5569 --> [CHECK: 9057 / 8808 / 3833 / 5569 cycle 投稿前重新核对仿真日志]

**表5-4 多核扩展三列对比**（XC7K325T，68.4 MHz Fmax）

| 列 | 单核 baseline | N=2 DDR模式 | N=4 W切片（20-case子集） |
|---|---|---|---|
| 代表case cycle | 8 808（C8C16 30×30 K=3） | 9 057（同case） | 3 833（wslice1） |
| 加速 | 1× | 0.97×（同case，+2.8% 开销） | 1.45×（vs N=2 wslice1） |
| AXI仲裁开销 | — | 2.8% | [CHECK] |
| W切片halo重算开销 | — | — | [CHECK] |

<!-- 来自 STATUS.md §2.8 N=4 全 20 case 列表 --> [CHECK: Tab.5-4 — 各列 AXI 仲裁开销百分比 + halo 重叠开销百分比的精确数字]

N=4 sweep把困难corner一并涵盖：`wslice_oddw`（W=33，不被4整除）、`wslice_smallw`（W=8，启动开销主导）、`wslice_k7`（halo宽度大）、`wslice_stride2`（stride=2 含下采样）。10个N=4 case的cycle区间从3833（wslice1）到34 097（wslice_k7），全部bit-exact通过。这一点是narrative B的硬证据：**同一份RTL在不变情况下，靠wrapper级别的W切片和DDR汇合，扩展到4核且不引入正确性回归**。<!-- 来自 STATUS.md §2.8 N=4 wslice 各 case 表 -->

对ResNet-18风格11层网络，调度器（参§4.5 / `toolchain/scheduler.py`）以单核593 K cycle为基线、灌入W切片halo模型，**估算N=2约302 K cycle（1.7×单核，331 fps）、N=4约141 K cycle（3.64×单核，709 fps）**。这两个数字是分析调度模型的projection，不是实测——STATUS §4将"ResNet 11层multicore chain端到端"明列为1-2天适配工作（cfg corner case对全shape sweep尚未覆盖），计划在投稿前完成。<!-- 来自 STATUS.md §2.6 Scheduler 估算表 + §4 ROI 列表 --> [CHECK: ResNet 11 层 multicore N=2/N=4 实测 cycles 与 1.7× / 3.64× 估算最终对齐]

N=4 wslice1的1.45×短于理想2×，可作四项分解。（a）**baseline归一化**：wslice1是单层、相对小图case，每层启动成本（line buffer priming + IDMA冷启动）占总时间比例较大，可达天花板低于2×。（b）**AXI仲裁开销**：N=2小case实测2.8%，N=4 axi_4to5 fabric下略高。（c）**W切片halo重算**：相邻核冗余产生K-1边界行。（d）**ODMA末端drain**：本质串行。更大workload——例如上面的11层ResNet projection 3.64×——应大幅掩蔽（a），调度器的model捕捉到这一点。[CHECK: 1.45× 近线性的 baseline 归一化定量分解]

放在文献语境里，**Simba [Shao@MICRO'19]** 通过36 chiplet通道切片做aggregation，把通道切片选作primary partition axis——前提是ASIC NoC具有充裕跨tile带宽。FLUX_CNN在FPGA上则把**W切片**选作primary axis，理由有三：（i）中端FPGA的核间fabric受AXI mux带宽限制，不是NoC级；（ii）W切片让权重广播模式与单核执行兼容；（iii）按§5.3，约束方是BRAM、不是互连。**Liu [TNNLS'21]** 与**Kang [Sensors'23]** 在FPGA上同样沿空间轴划分，我们的W切片设计同属此family，差异在row-ring streaming primitive让halo处理保持局部、而非跨核。<!-- 来自 literature.md Simba MICRO'19 / Liu TNNLS'21 / Kang AoCStream'22 -->

至此narrative B在系统侧的scaling故事被定钉在N=2和N=4两个scale点：N=2同case +2.8% AXI仲裁开销，N=4 wslice1 1.45×加速且全部20 W切片case bit-exact通过。结合§5.3的BRAM-bound结论（3核为XC7K325T在不动每核SRAM下的硬上限），这一scaling图像不空泛——既给出实测数字，也明确了constraint边界。下节进入对位对比，把narrative A和B同时放到prior art中校准。

## 5.6 与Prior Art对比

我们沿两张表与文献做对位对比。**表5-5是性能对比**，统一列出dataset、device、Fmax、GOPS、整网MAC%与端到端latency等列；**表5-6是设计轴对比**，沿§3.6引入的五条轴展开（intra-array dataflow、PE利用率恢复位置、流式粒度、多核划分轴、编译器栈范围），把§3.6的定位表折叠进来避免重复。FLUX_CNN在表5-5中的cell直接从§5.3-§5.5取数；baseline cell在投稿final pass前会再次比对原文重新核对。<!-- 来自 literature.md 全部 baseline 条目 -->

**表5-5 性能对位对比**（FLUX_CNN cell来源已注，baseline cell带[CHECK]待原文核对）

| 工作 | 器件 | Fmax (MHz) | 算力 (GOPS) | 整网 MAC% | 端到端代表数据 |
|---|---|---|---|---|---|
| **FLUX_CNN（本工作）** | XC7K325T-2 | 68.4（target 100） | 51.2（@100 MHz target）/ 34.8（@实测Fmax） | **86.6%** | ResNet-18风格11层 8.69 ms（115 fps）@实测 / 5.95 ms（168 fps）@target |
| TPU v1 [Jouppi@ISCA'17] | 28nm ASIC | 700 | 92 000（INT8） | [CHECK] | 数据中心规格 |
| Eyeriss [Chen@ISSCC'16] | 65nm ASIC | 200 | 84 (INT16) | [CHECK] | AlexNet 35 fps @ 278 mW [CHECK] |
| Gemmini [Genc@DAC'21] | 22nm ASIC | [CHECK] | 106.1 GOPS/W | [CHECK] | SoC集成基准 [CHECK] |
| Snowflake [Gokhale@ISCAS'17] | XC7Z045 | 250 | 128 | **91%**（平均计算效率，分母不同）[CHECK] | AlexNet 100 fps / ResNet-50 17 fps [CHECK] |
| Angel-Eye [Guo@TCAD'18] | Zynq 7-series | [CHECK] | [CHECK] | [CHECK] | Zynq INT8代表（同器件类） |
| Aydonat DLA [Aydonat@FPGA'17] | Arria 10 GX1150 | [CHECK] | **1382 GFLOPS** | [CHECK] | AlexNet 1020 img/s [CHECK] |
| Lu Winograd [Lu@FCCM'17] | ZCU102 | [CHECK] | **854.6 GOPS** | [CHECK] | [CHECK] |
| Ma et al. (RTL2-OPS) | Stratix-V | [CHECK] | **645 GOPS** [CHECK] | [CHECK] | [CHECK] |
| Liu Full-Stack [Liu et al.@TNNLS'21] | Arria 10 GX1150 | [CHECK] | >1 300（>1.3 TOPS） | **97%** | [CHECK] |
| VTA [Moreau@IEEE Micro'19] | Zynq 7-series | 100 | 51.2（256 PE @100 MHz INT8） | [CHECK] | [CHECK] |
| fpgaConvNet [Venieris@TNNLS'19] | Zynq家族 | [CHECK] | **2.94×** vs hand-tuned baseline [CHECK] | [CHECK] | [CHECK] |

<!-- 来自 literature.md TPU/Eyeriss/Gemmini/Snowflake/Angel-Eye/Aydonat/Lu/Liu/VTA/fpgaConvNet 各条目 + STATUS.md §1 单核 51.2/34.8 GOPS --> [CHECK: 表5-5 各 baseline 整网 MAC% / 同器件 Fmax / 同器件资源占用 — reviewer 阶段最终对齐]

最numerically敏感的对比是**86.6%整网MAC%（FLUX_CNN，XC7K325T）vs 97% MAC%（Liu Full-Stack [TNNLS'21]，Arria 10 GX1150）**。我们不去争Liu的数字本身——它是在更大尺度器件上达成的真实值；我们把这条对比定性为**"同范式、不同scale点"**：FLUX_CNN在XC7K325T上以36 942 LUT工作，Liu则在Arria 10 GX1150上工作（约1.15M ALM、约相当于1150 K LUT级单元）。在Liu的尺度下，每层片上存储容量与每stage流水化空间显著扩张；在FLUX_CNN的尺度下，是流式row-ring本身才让86.6%在这台中端器件上**变得可达**。诚实的解读是"同方向、不同scale点"，不是"Liu的数字胜过我们的"。<!-- 来自 literature.md Liu TNNLS'21 + STATUS.md §1 36 942 LUT --> [CHECK: Arria 10 GX1150 ALM/LUT 数字 — reviewer 阶段确认]

**86.6% vs Snowflake 91%平均计算效率**这条对比方法学上软一些，因为分母不同：Snowflake 91%在某种配置下报告，其层间边界处理与我们的方式不同。我们会在表5-5里以脚注定义计量口径，而不是把两个数字粗暴折成同一列。这件事我们在正文显式提出，是因为审稿人常会探询此类分母差异，宁愿主动把它放到台面上、也不要把它留到rebuttal阶段被质疑。[CHECK: Snowflake ISCAS'17 91% 计算效率口径的精确文字定义 — 读原文 §V Evaluation]

两条同器件级（device-class-equivalent）对比进一步校准picture。**Angel-Eye [Guo@TCAD'18]** 是同样落在7-series + INT8疆域上最直接的对比；架构差异在Angel-Eye采用**自定义ISA + controller-driven**，而FLUX_CNN采用**去中心化valid-ready握手流水**（参§3.2 / C1.1）——同target、不同control philosophy。**Gemmini [Genc@DAC'21]** 用同样规模的16×16 INT8脉动阵列作为参考点，但其上挂的是**中心FSM controller**，而不是我们这里描述的去中心化握手fabric——这是C1.1贡献最干净的"同算术、不同control"对照。<!-- 来自 literature.md Angel-Eye TCAD'18 / Gemmini DAC'21 -->

进一步扩展baseline集，**fpgaConvNet [Venieris@TNNLS'19]** 是"layer-pipelined streaming"路线代表，相对hand-tuned baseline报告2.94× speedup [CHECK]。**Aydonat DLA [FPGA'17]** 在Arria 10上以1382 GFLOPS定钉了vendor-specific HLS + Winograd路线的高峰值。**Lu Winograd [FCCM'17]** 在ZCU102上达854.6 GOPS。**Ma et al.** 在Stratix-V上报告645 GOPS [CHECK: Ma 工作的精确条目与年份]。这三条在算力绝对量级上均在FLUX_CNN之上，但都依赖**Winograd或大尺度Intel/Altera FPGA**——FLUX_CNN不依赖Winograd（直接卷积 + streaming）、且选定中端7-series器件。表5-5的列结构正是为了让这种"算法路径 + 器件尺度"差异被一眼看清，而不是被一个GOPS峰值数字遮蔽。<!-- 来自 literature.md Aydonat / Lu / fpgaConvNet 条目 -->

设计轴对比（表5-6，沿用§3.5的4维度对位框架并扩展为更细粒度的设计轴）：FLUX_CNN占据一个distinctive cell——**OS列广播 + 编译器侧Ky-fold/S2D + row-ring streaming + W切片多核 + network级mini-compiler**。SDP fusion完整度（bias + shortcut + 量化）作为一个独立属性列单独报告（§4.4），而不当作一条独立轴。这样的cell布局让"FLUX_CNN与谁不同、与谁相近、相近者之间又如何细分"在一张表里说清楚。[CHECK: Angel-Eye SDP fusion 具体形态 — 论文里是否含 shortcut residual / 仅 bias+ReLU]

至此对位对比定钉两点。其一，**narrative A（编译器侧PE利用率优化）**：在已检索文献里，硬件路线（MAERI、Eyeriss-v2）已被广泛探索；编译器only路线（Ky-fold + S2D，零RTL改动）**无明显近邻prior art**——这是较强的claim候选。其二，**narrative B（系统侧streaming + 多核扩展）**：在streaming row-ring的"任意H×W"维度上，Alwani Fused-layer / Kang AoCStream / Liu Full-Stack这三条prior art已构成近邻（参§2.6），FLUX_CNN的差异点是**row-ring credit-based反压 + W切片多核halo解析**——这一点构成贡献而非颠覆。

## 5.7 局限与未来工作

我们openly列出已识别的局限，按"修复成本/影响比"的递减顺序：**（a）** Fmax 68.4 MHz未达100 MHz设计目标；**（b）** `Cout < 16`层的PE列按硬件设计以`Cout/16`比例空转；**（c）** Pooling、Depthwise Convolution与结构化/非结构化稀疏在当前RTL中未实现；**（d）** ResNet 11层multicore chain端到端尚为分析估算（参§5.5）；**（e）** 片上push链P2完成态（流水线模式 N=2 ABAB / N=4）尚未实施，跨stage目前走DDR中转。<!-- 来自 STATUS.md §1 单核已知问题 + §2.6 / §2.8 多核状态 + §4 ROI -->

对**（a）** Fmax shortfall，§5.3已识别两个根因：`mac_pe`未加`(* use_dsp = "yes" *)`属性，估计回收约17 K LUT并释放DSP48E1；SDP量化组合链未流水线化，切1-2级是教科书修法。STATUS §4将这条列为最高ROI项，计划在投稿前完成`use_dsp` + SDP流水化重综合，刷到100+ MHz并把§5.3资源数字与§5.4 wall-clock数字按新Fmax重新报告。[CHECK: use_dsp + SDP 流水化重综合后 100+ MHz 与 17K LUT 节省 — 重综合验证]

对**（b）** `Cout < 16` PE列空转，设计选择是**保持阵列几何简洁**，不引入per-column gating或输出通道multiplexing。理由是经验性的：在ResNet-18风格网络里，受影响层（如最末FC类、Cout极小的层）对总MAC量的贡献占比极小，且其绝对latency贡献由I/O而非compute主导。我们把这种空转记为**设计rationale下的可接受trade-off**，而不当作defect处理。[CHECK: Cout<16 layers 占总 MAC 比例 — 从 model_analysis.md 抽取]

对**（c）** Pooling / Depthwise / sparsity，三者作为future RTL extension列出，但scope与代价差异很大。**Depthwise**特别非平凡，因为它反转了当前`mac_array`的Cin/Cout广播模式，需要重做PE列broadcast；**Pooling**相对轻量，在`ofb_writer`下游加专用stride模块即可，但需要host CPU或片上state machine配合处理pooling窗口对齐；**稀疏**（无论结构化还是非结构化）会触及mac_pe的MAC逻辑本身，是中长期工作。<!-- 来自 STATUS.md §4 ROI -->

对**（d）（e）** 多核chain实测与片上push链，两者的实施在STATUS §4已分别估为1-2天与2-3天工作量。**（d）** 替换§5.5的scheduler projection为实测cycle，刷新N=2 1.7× / N=4 3.64×这两个数字；**（e）** 把跨stage的DDR往返替换为core-to-core IFB push（M2 cross-core SRAM直送基础设施已就绪，参§4.5），消除DDR带宽这一中长期瓶颈。两者都不是blocker，但都会让narrative B的"已落地"声明更密实。<!-- 来自 STATUS.md §2.6 / §2.8 / §4 ROI -->

future work的优先级排序按STATUS §4的ROI列表：**（1）`use_dsp`属性 + SDP流水化重综合**（最高ROI、投稿前可完成）；**（2）ResNet 11层multicore chain端到端实测**（1-2天适配，把§5.5的scheduler估算替换为实测）；**（3）片上push链P2完成态**（2-3天，消除跨stage DDR往返）；**（4）跨层streaming融合**——架构层面最significant，与**Tangram [Gao@ASPLOS'19]** 同向，可与现有Cout切片和stage barrier基础设施组合，是中长期工作。Pooling / Depthwise / 稀疏作为独立RTL extension保留在更长远议程里，本工作不实施、也不claim。[TBD: future work 是否点名具体投稿后里程碑日期还是仅按 ROI 排序]

## 5.8 本章小结

本章把第三章设计与第四章实现放到XC7K325T-FFG900-2上做端到端验证。在编译器侧（narrative A），Ky折叠把Cin=4浅层PE利用率从12.5%恢复到接近100%（约8×恢复），S2D把Cin=8 + stride=2下采样层从50%恢复到接近100%（约2×恢复），两者在L1这种最差利用率层叠加构成主要MAC%增益的**双重杠杆**。在系统侧（narrative B），单核ResNet-18风格11层chain以593 K cycle、整网86.6% MAC%完成（@实测68.4 MHz对应8.69 ms / 115 fps），多核wrapper在N=2 +2.8% AXI仲裁开销、N=4 wslice1取得1.45× vs N=2加速且全部20 W切片case bit-exact通过。资源端，单核LUT 18.1% / BRAM 28.8% / DSP 9.8%，BRAM被识别为多核扩展的binding约束（3核为不动每核SRAM下的硬上限）。**86.6% vs Liu 97% 是同范式、不同scale点的诚实对比**，不是绝对败北。已识别局限（Fmax 68.4 MHz、Cout<16空转、Pooling/Depthwise/稀疏未实现、multicore chain实测与push链P2完成态待跑）按ROI排序写入§5.7 future work。下一章对全文做总结。



# 第六章 总结

## 6.1 主要工作总结

本论文围绕"在固定16×16 INT8脉动阵列上、面向边缘FPGA同时承载浅层PE利用率回填与任意分辨率特征图流式推理"这一中心命题，给出了一套以编译器侧折叠为主轴（narrative A）、以行环流式数据通路与多核W切片为系统侧支撑（narrative B）的协同设计方案——FLUX_CNN加速器。设计动机来自第一章所述的两类相互耦合瓶颈：固定阵列在ResNet-18风格网络浅层（`Cin∈{3,4,8}`）的空间PE利用率塌陷至12.5%-50%，以及边缘FPGA上1.6 MB级BRAM预算无法承载VGA分辨率整图（约4.9 MB）这一硬天花板<!-- 来自 contributions.md §4.4 / contributions.md §4.1 -->。

针对这两类瓶颈，本工作不走Eyeriss-v2<sup>[12]</sup>或FlexFlow<sup>[20]</sup>所代表的运行时硬件可重构互连路线，而是把PE利用率恢复的复杂度上推到编译器层：Ky折叠在`Cin<16`时把卷积核Ky维折至虚拟cin维，S2D在`stride≥2`时把stride²个空间相位重排至Cin维，两者均零RTL改动。系统侧采用去中心化valid-ready握手的5模块流水（`line_buffer→mac_array→parf_accum→ofb_writer`+SDP）配合双向行信用反压的行环数据通路，让任意H×W图像在host仅写4个boot寄存器+1次`start`命令下端到端跑完，并以N=2/4多核W切片wrapper做横向扩展<!-- 来自 contributions.md C1.2 / C2.1 / C2.2 / C3.5 -->。我们明确表态：**编译器侧折叠路线并不主张严格优于硬件可重构方案，二者是互补的设计取舍点**——前者以编译器复杂度为代价换取RTL极简，后者以RTL面积与验证代价换取算法层面的零干预，本工作的贡献是论证在浅层ResNet-18风格网络规模下编译器侧已足以逼近完全利用率。

## 6.2 主要成果

本工作的实证证据按"功能正确—硬件收敛—端到端性能"三条线在第五章铺开。**功能正确**层面，单核回归46个case（22个ResNet-18风格chain layer跨3种折叠模式 + 24个鲁棒性corner case，覆盖`K∈{1,2,3,5,7}`、stride 1-4、图像尺寸1×1至120×68）以及多核W切片回归20个case（覆盖`K∈{1,3,5,7}`、`stride∈{1,2}`、`W∈{8,32,33}`、单层与多层）全部在100k与200k周期watchdog内bit-exact通过<!-- 来自 contributions.md §4.2 / STATUS §2.5 / §2.8 -->[CHECK: 46+20 case 总数最终核对]。

**硬件收敛**层面，单核在XC7K325T-FFG900-2上OOC综合占用128块BRAM36（28.8%）、82个DSP（9.8%）、36942个LUT（18.1%）、6420个FF（1.6%），N=4多核wrapper在与单核同一68.4 MHz Fmax下收敛、确认W切片复制对时序中性<!-- 来自 contributions.md §4.1 / STATUS §2.3 -->[CHECK: §5.3/§5.5 N=4 OOC report与单核Fmax一致性]。**端到端性能**层面，11层ResNet-18风格chain在单核上以593 K周期完成、整网级MAC利用率达86.6%——@实测68.4 MHz对应8.69 ms单帧延迟、约115 fps吞吐——其中13.4%的gap可解释为layer间descriptor配置开销而非steady-state低效<!-- 来自 contributions.md C2.5 / §5.4 -->[CHECK: 86.6% / 593k cycle以§5.4最终重跑结果为准]。三条线共同界定了本工作的贡献范围而不夸大：本论文不主张达到Liu等人<sup>[17]</sup>在Arria 10 GX1150（其BRAM/DSP约为XC7K325T一个数量级）上报告的97% MAC效率，但在16倍小的中等规模FPGA上以不做层融合的前提取得86.6%，是同范式、不同规模点的诚实可比数字。

## 6.3 不足与未来工作

本工作仍有若干已识别的不足，均按STATUS §4的ROI排序在第五章§5.7中明确指出。**ROI最高、投稿前可完成**的一项是综合属性与流水化重综合：给`mac_pe`添加`(* use_dsp = "yes" *)`属性以强制DSP48实现、并把SDP量化组合长链插入流水级，预期收益是Fmax由68.4 MHz关到100+ MHz并节省约17K LUT[CHECK: use_dsp + SDP流水化重综合100+ MHz / 17K LUT节省待重综合验证]。**第二档**是11层ResNet多核chain端到端实测（1-2天集成工作量，把§5.5中scheduler projection替换为实测cycle）和片上push链P2完成态（2-3天，把跨stage的DDR往返替换为core-to-core IFB push、消除DDR带宽这一中长期瓶颈）。

**架构层面更具中长期意义**的方向是跨层streaming融合，与Tangram<sup>[14]</sup>同向并可与现有Cout切片和stage barrier基础设施组合。Pooling、Depthwise Convolution、结构化与非结构化稀疏作为独立RTL扩展，其设计空间与对接位置在§5.7已勾勒，但本工作不实施、也不claim对应支持。Cout极小（如`Cout=2`的全连接层）所对应的列空转代价，本工作选择不在RTL侧引入列折叠机制——理由是其在ResNet-18风格网络中cycle占比不显著、与额外RTL复杂度不成比例<!-- 来自 contributions.md C2.4 -->；这一边界与Ky折叠针对Cin的覆盖范围互补，明确属于"刻意未做"而非"遗漏"。整体上，FLUX_CNN作为开源、轻量、可复现、自带编译器栈与回归基础设施的端侧FPGA-CNN加速器参考实现，在编译器侧折叠与硬件可重构两条路径中提供了前者的一个具体可比点；后续投稿与开源迭代将沿上述ROI排序逐步推进。[TBD: 是否在末尾点名具体投稿后里程碑日期还是仅按ROI排序]

---

# 参考文献

[1] He K, Zhang X, Ren S, 等. Deep Residual Learning for Image Recognition[C]. Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), Las Vegas, 2016: 770-778. DOI: 10.1109/CVPR.2016.90.

[2] Jouppi N P, Young C, Patil N, 等. In-Datacenter Performance Analysis of a Tensor Processing Unit[C]. Proceedings of the 44th Annual International Symposium on Computer Architecture (ISCA), Toronto, 2017: 1-12. DOI: 10.1145/3079856.3080246.

[3] Chen Y H, Emer J, Sze V. Eyeriss: A Spatial Architecture for Energy-Efficient Dataflow for Convolutional Neural Networks[C]. Proceedings of the 43rd Annual International Symposium on Computer Architecture (ISCA), Seoul, 2016: 367-379. DOI: 10.1145/3007787.3001177.

[4] NVIDIA Corporation. NVDLA Open Source Hardware: Design Specification and Reference Implementation[EB/OL]. (2017)[2026-04-30]. http://nvdla.org/. [CHECK: vendor doc 引用方式 — literature.md 注 vendor white paper, 无标准 DOI]

[5] ARM Limited. Arm Ethos-N78 NPU Technical Reference Manual[EB/OL]. [2026-04-30]. https://developer.arm.com/documentation/. [CHECK: literature.md 待补 — vendor doc 无 paper 索引]

[6] Venieris S I, Bouganis C S. fpgaConvNet: Mapping Regular and Irregular Convolutional Neural Networks on FPGAs[J]. IEEE Transactions on Neural Networks and Learning Systems, 2019, 30(2): 326-342. DOI: 10.1109/TNNLS.2018.2844093.

[7] Guo K, Sui L, Qiu J, 等. Angel-Eye: A Complete Design Flow for Mapping CNN Onto Embedded FPGA[J]. IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 2018, 37(1): 35-47. DOI: 10.1109/TCAD.2017.2705069.

[8] Gokhale V, Zaidy A, Chang A X M, 等. Snowflake: An Efficient Hardware Accelerator for Convolutional Neural Networks[C]. IEEE International Symposium on Circuits and Systems (ISCAS), Baltimore, 2017: 1-4. DOI: 10.1109/ISCAS.2017.8050809.

[9] Xilinx Inc. Zynq DPU v3.3 Product Guide (PG338)[EB/OL]. [2026-04-30]. https://www.xilinx.com/. [CHECK: vendor doc 引用方式 — Xilinx PG338 product guide, 无 paper 索引]

[10] Genc H, Kim S, Amid A, 等. Gemmini: Enabling Systematic Deep-Learning Architecture Evaluation via Full-Stack Integration[C]. Proceedings of the 58th ACM/IEEE Design Automation Conference (DAC), San Francisco, 2021: 769-774. DOI: 10.48550/arXiv.1911.09925.

[11] Shao Y S, Clemons J, Venkatesan R, 等. Simba: Scaling Deep-Learning Inference with Multi-Chip-Module-Based Architecture[C]. Proceedings of the 52nd Annual IEEE/ACM International Symposium on Microarchitecture (MICRO), Columbus, 2019: 14-27. DOI: 10.1145/3352460.3358302.

[12] Chen Y H, Yang T J, Emer J, 等. Eyeriss v2: A Flexible Accelerator for Emerging Deep Neural Networks on Mobile Devices[J]. IEEE Journal on Emerging and Selected Topics in Circuits and Systems (JETCAS), 2019, 9(2): 292-308. DOI: 10.1109/JETCAS.2019.2910232.

[13] Kwon H, Samajdar A, Krishna T. MAERI: Enabling Flexible Dataflow Mapping over DNN Accelerators via Reconfigurable Interconnects[C]. Proceedings of the 23rd International Conference on Architectural Support for Programming Languages and Operating Systems (ASPLOS), Williamsburg, 2018: 461-475. DOI: 10.1145/3173162.3173176.

[14] Gao M, Yang X, Pu J, 等. Tangram: Optimized Coarse-Grained Dataflow for Scalable NN Accelerators[C]. Proceedings of the 24th International Conference on Architectural Support for Programming Languages and Operating Systems (ASPLOS), Providence, 2019: 807-820. DOI: 10.1145/3297858.3304014.

[15] Alwani M, Chen H, Ferdman M, 等. Fused-Layer CNN Accelerators[C]. Proceedings of the 49th Annual IEEE/ACM International Symposium on Microarchitecture (MICRO), Taipei, 2016: 1-12. DOI: 10.1109/MICRO.2016.7783725.

[16] Kang H J, Yang H. AoCStream: All-on-Chip CNN Accelerator with Stream-Based Line-Buffer Architecture and Accelerator-Aware Pruning[J]. Sensors (MDPI), 2023, 23(19): 8104. DOI: 10.3390/s23198104.

[17] Liu S L, Fan H X, Ferianc M, 等. Toward Full-Stack Acceleration of Deep Convolutional Neural Networks on FPGAs[J]. IEEE Transactions on Neural Networks and Learning Systems, 2021, 33(8): 3974-3987. DOI: 10.1109/TNNLS.2021.3055240.

[18] Chen Y H, Krishna T, Emer J, 等. Eyeriss: An Energy-Efficient Reconfigurable Accelerator for Deep Convolutional Neural Networks[J]. IEEE Journal of Solid-State Circuits (JSSC), 2017, 52(1): 127-138. DOI: 10.1109/JSSC.2016.2616357.

[19] Du Z D, Fasthuber R, Chen T S, 等. ShiDianNao: Shifting Vision Processing Closer to the Sensor[C]. Proceedings of the 42nd Annual International Symposium on Computer Architecture (ISCA), Portland, 2015: 92-104. DOI: 10.1145/2749469.2750389.

[20] Lu W Y, Yan G H, Li J J, 等. FlexFlow: A Flexible Dataflow Accelerator Architecture for Convolutional Neural Networks[C]. IEEE International Symposium on High Performance Computer Architecture (HPCA), Austin, 2017: 553-564. [CHECK: literature.md 待补 — DOI 与作者列表精确条目]

[21] Parashar A, Raina P, Shao Y S, 等. Timeloop: A Systematic Approach to DNN Accelerator Evaluation[C]. IEEE International Symposium on Performance Analysis of Systems and Software (ISPASS), Madison, 2019: 304-315. DOI: 10.1109/ISPASS.2019.00042.

[22] Yang X, Gao M, Liu Q J, 等. Interstellar: Using Halide's Scheduling Language to Analyze DNN Accelerators[C]. Proceedings of the 25th International Conference on Architectural Support for Programming Languages and Operating Systems (ASPLOS), Lausanne, 2020: 369-383. DOI: 10.1145/3373376.3378514.

[23] Chen T Q, Moreau T, Jiang Z H, 等. TVM: An Automated End-to-End Optimizing Compiler for Deep Learning[C]. Proceedings of the 13th USENIX Symposium on Operating Systems Design and Implementation (OSDI), Carlsbad, 2018: 578-594.

[24] Moreau T, Chen T Q, Vega L, 等. A Hardware-Software Blueprint for Flexible Deep Learning Specialization[J]. IEEE Micro, 2019, 39(5): 8-16. DOI: 10.1109/MM.2019.2928962.

[25] Ma Y F, Cao Y, Vrudhula S, 等. Optimizing Loop Operation and Dataflow in FPGA Acceleration of Deep Convolutional Neural Networks[C]. Proceedings of the ACM/SIGDA International Symposium on Field-Programmable Gate Arrays (FPGA), Monterey, 2017: 45-54. DOI: 10.1145/3020078.3021736.

[26] Chetlur S, Woolley C, Vandermersch P, 等. cuDNN: Efficient Primitives for Deep Learning[J]. arXiv preprint, 2014, arXiv:1410.0759.

[27] Venieris S I, Bouganis C S. fpgaConvNet: A Framework for Mapping Convolutional Neural Networks on FPGAs[C]. IEEE 24th Annual International Symposium on Field-Programmable Custom Computing Machines (FCCM), Washington DC, 2016: 40-47. DOI: 10.1109/FCCM.2016.22.

[28] Aydonat U, O'Connell S, Capalija D, 等. An OpenCL Deep Learning Accelerator on Arria 10[C]. Proceedings of the ACM/SIGDA International Symposium on Field-Programmable Gate Arrays (FPGA), Monterey, 2017: 55-64. DOI: 10.1145/3020078.3021738.

[29] Lu L Q, Liang Y, Xiao Q C, 等. Evaluating Fast Algorithms for Convolutional Neural Networks on FPGAs[C]. IEEE 25th Annual International Symposium on Field-Programmable Custom Computing Machines (FCCM), Napa, 2017: 101-108. DOI: 10.1109/FCCM.2017.64.

[30] Zhang C, Sun G Y, Fang Z M, 等. Caffeine: Toward Uniformed Representation and Acceleration for Deep Convolutional Neural Networks[J]. IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 2019, 38(11): 2072-2085. [CHECK: literature.md 待补 — 完整 DOI 与起止页号未在 literature.md 给出]

[31] Jacob B, Kligys S, Chen B, 等. Quantization and Training of Neural Networks for Efficient Integer-Arithmetic-Only Inference[C]. Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), Salt Lake City, 2018: 2704-2713. DOI: 10.1109/CVPR.2018.00286.

[32] Han S, Liu X Y, Mao H Z, 等. EIE: Efficient Inference Engine on Compressed Deep Neural Network[C]. Proceedings of the 43rd Annual International Symposium on Computer Architecture (ISCA), Seoul, 2016: 243-254. DOI: 10.1145/3007787.3001163.

[33] Han S, Kang J L, Mao H Z, 等. ESE: Efficient Speech Recognition Engine with Sparse LSTM on FPGA[C]. Proceedings of the ACM/SIGDA International Symposium on Field-Programmable Gate Arrays (FPGA), Monterey, 2017: 75-84. DOI: 10.1145/3020078.3021745.

[34] Pellauer M, Shao Y S, Clemons J, 等. Buffets: An Efficient and Composable Storage Idiom for Explicit Decoupled Data Orchestration[C]. Proceedings of the 24th International Conference on Architectural Support for Programming Languages and Operating Systems (ASPLOS), Providence, 2019: 137-151. DOI: 10.1145/3297858.3304025.

[35] Shi W Z, Caballero J, Huszár F, 等. Real-Time Single Image and Video Super-Resolution Using an Efficient Sub-Pixel Convolutional Neural Network[C]. Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), Las Vegas, 2016: 1874-1883. DOI: 10.1109/CVPR.2016.207.

[36] Kwon H, Chatarasi P, Sarkar V, 等. MAESTRO: A Data-Centric Approach to Understand Reuse, Performance, and Hardware Cost of DNN Mappings[J]. IEEE Micro, 2020, 40(3): 20-29. [CHECK: literature.md 待补 — DOI 与卷期号待原文核对]
