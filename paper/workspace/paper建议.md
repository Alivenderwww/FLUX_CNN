
从写成一篇论文的角度来考虑：
1、研究场景和动机不太清晰，且相关研究没有围绕动机进行分析。
文章中提到的场景是端侧，动机包括
①“然而固定的硬件规模与单一数据流策略难以覆盖多种网络结构与层间差异，限制了加速器的长期可用生命周期”，解决方案是“本文 FLUX_CNN 采用的二维 mesh 多核互联” ，“基于可重构阵列的 CNN 加速器设计涵盖数据流编排、阵列结构、存储层次、软硬协同调度等多个层面”
②“但面对任意 K 卷积核、可变步长（Stride）、多分支残差等现代网络结构时，硬件逻辑复杂度急剧上升，难以通用化”，尤其可以提到stride带来的问题，这在之前的论文中也有提到，然后可以说他们的解决方案有什么问题。

这两个动机与端侧的相关性不够强。
例如 
如果针对端侧场景，重点考虑的指标是延迟/吞吐优化。动机应该是当前加速器在实时处理高清图像方面的不足之处（单batch场景下推理利用率低）。
如果针对大分辨率，单batch，高吞吐低延迟，real-time的推理场景，那就需要用对应的算法来测试，看看与现有研究的差距在哪里，再去做针对性的优化。


2、本文的核心创新点需要与场景和动机对应：
①计算核支持多维度复用，对不同卷积参数的利用率保持较高，因此有较高的灵活性（适合的场景是做asic而不是FPGA，例如端侧场景）。
②支持多核并行调度，便于拓展。
③流式计算，保证高吞吐？。
④软件工具链，不同网络均可快速部署。

3、相关研究部分，没有突出当前研究的问题
例如1.2.1 端侧卷积神经网络硬件加速器架构研究进展 部分，介绍了很多文献的性能指标，可以作为一个小综述，但是并没有提到关键问题。


方向 1：多任务与空间复用 (Multi-tenancy & Spatial Multiplexing)
1. 故事线 (The Story)
现状： 现有的加速器（如 TPU, NVDLA）通常是为单模型、大吞吐量设计的。但在实际应用（如自动驾驶、AR/VR）中，硬件需要同时运行多个模型（检测、分割、跟踪）。 
痛点[MZ1.1]：
•	资源碎片化： 如果模型小，大阵列填不满，利用率低。
•	干扰与拥塞： 多个模型抢占总线和片上 SRAM，导致帧率（Latency）大幅抖动，这在实时系统中是致命的。 你的创新点： 利用你的“微观 FSM + 宏观 Task Descriptor”。你可以主打“硬件级虚拟化与弹性分片”。
•	故事叙述： “我们提出了一种基于描述符驱动的弹性加速阵列。通过 6 层 FSM 实现核内零开销调度，同时通过宏指令流在空间上动态切分 MAC 阵列。当运行多个异构模型时，我们的架构能通过‘细粒度切片策略’消除资源争用，并实现 70%+ 的系统级综合利用率。”
2. 建议测试算法[MZ2.1]
你需要测试一个“模型组合（Model Suite）”：
•	组合 A (AR/VR)： MobileNetV2 (特征提取) + RepVGG (实时渲染) + 一个小的全连接网络 (传感器融合)。
•	组合 B (自动驾驶)： YOLOv5s (检测) + 简单的 Segmentation 网络 + PointNet (如果能支持)。
•	重点： 必须展示在并发运行这些模型时，你的架构表现。
3. 核心指标与目标
•	Throughput (FPS/Normalized)： 在运行多任务时，系统总吞吐量应比“串行执行”提升 1.5x - 2x。
•	Tail Latency (P99)： 证明在高负载下，关键任务的延迟抖动极小（对比 Baseline 降低 30%-50%）。
•	Baseline： 至少对比 NVDLA (多实例模式) 或 PREMA (HPCA'20) 这种专门做多任务调度的架构。
4. 相关论文参考
•	2020 MICRO-Planaria: Dynamic Architecture Fission for Spatial Multi-Tenant Acceleration of Deep Neural Networks：Planaria可以在运行时动态分裂（分裂）成多个更小但功能齐全的DNN引擎。这种微架构能力使多个DNN推理服务能够在同一硬件上空间共存，提供多租户DNN加速。为了实现这种动态可重构性，我们首先设计了用于DNN加速的可分裂全向收缩阵列，允许数据全向流动。其次，它利用这种能力以及芯片上内存、互连和计算资源的独特组织，在基于收缩阵列的DNN加速器中实现分裂。架构分裂及其相关的灵活性为任务调度提供了额外的自由度，甚至允许根据服务器负载、DNN拓扑和任务优先级来分裂加速器。
•	2023 IEEE Transactions on Computers-Enabling Fine-Grained Spatial Multitasking on Systolic-Array NPUs Using Dataflow Mirroring：在本文中，我们提出了数据流镜像NPU（DM-NPU），一种支持细粒度收缩阵列分布的新型空间多任务NPU架构。DM-NPU的关键思想是在水平和/或垂直方向上反转共位置DNN的数据流。DM-NPU可以在收缩阵列的任何相邻处理单元之间放置分配边界，无论是水平还是垂直。
•	2023 ISCA-V10: Hardware-assisted npu multi-tenancy for improved resource utilization and fairness： 在这篇论文中，我们提出了V10，一个硬件辅助的NPU多租户框架，以提高资源利用率，同时确保不同ML服务的公平性。我们重新设计了NPU架构以支持多租户。V10采用算子调度器，以在收缩阵列和矢量单元上实现并发算子执行，并为执行基于优先级的资源共享机制提供灵活性。V10还支持NPU中的细粒度算子抢占和轻量级上下文切换。为了进一步提高NPU的利用率，V10还开发了一种基于聚类的负载分配机制，用于识别在共享NPU上最佳匹配的ML服务。。
•	2021 ASAP-How to Reach Real-Time AI on Consumer Devices? Solutions for Programmable and Custom Architectures中有一部分介绍了Multi-Tenant AI Systems的相关研究。

________________________________________
方向 2：极致能效与无指令调度的 TinyML 引擎 (Ultra-low Power & Instruction-less TinyML Engine)
1. 故事线 (The Story)
现状： 在极低功耗（mW 级）设备上，取指、译码和复杂的总线协议消耗了 40%-60% 的非计算功耗。 
痛点： 现有的“通用 AI 加速器”为了灵活性保留了太多的指令控制逻辑，对 Tiny 级别的网络来说太重了。 
你的创新点： 利用你的“6 层嵌套自循环 FSM”和“工具链后端地址生成”。
•	故事叙述： “我们挑战了‘灵活性必须以控制开销为代价’的传统观念。通过将复杂的卷积循环嵌套直接映射到硬件 FSM 中，并利用离线工具链预计算地址步进，我们实现了‘近乎固定逻辑（Fixed-function）’的能效，同时保留了‘软件定义’的灵活性。这是一种‘无指令（Instruction-less）’的计算范式。”
2. 建议测试算法
必须跑 MLPerf Tiny 基准测试集：
•	Keyword Spotting (KWS)： 语音唤醒。
•	Visual Wake Words (VWW)： 人脸/人身检测。
•	Image Classification： 小尺寸的 MobileNet/ShuffleNet。
•	Anomaly Detection： 异常检测。
3. 核心指标与目标
•	Energy per Inference (uJ/Inf)： 每一帧推断消耗的微焦耳数。这是 TinyML 的第一指标。
•	Peak Power： 峰值功耗是否能压在 10mW - 50mW 以下（取决于工艺）。
•	Area Efficiency： 在极小面积下提供高算力。
•	目标水平：
o	能效比（Energy Efficiency）需要达到 10 - 20 TOPS/W (INT8, 28nm)。
o	对比基线：ARM Ethos-U55, GreenWaves GAP9, 或者顶会论文 Ultra-Low Power 系列（如 Eyeriss v2 的边缘模式）。
4. 相关论文参考
•	2025  IEEE Transactions on Computers- Stream: Design Space Exploration of Layer-Fused DNNs on Heterogeneous Dataflow Accelerators：（源自 KU Leuven 的顶级低功耗芯片团队 Marian Verhelst 组，Marian Verhelst有很多关于tiny ML的文章）迄今为止，这些系统通过一次在多个核心上粗略映射单个层来利用并行性，这会导致频繁昂贵的片外内存访问，或者通过流水线输入批处理，这无法满足延迟关键应用的需求。为了缓解这些瓶颈，这项工作探索了在异构数据流加速器上的一种新的细粒度映射范式，称为层融合，通过一个名为Stream的新型设计空间探索框架。Stream捕捉了各种异构数据流架构和映射粒度，并实现了经过三种不同的最先进的硬件实现验证的记忆和通信感知延迟和能耗分析。因此，它通过约束优化策略性地分配工作负载，促进了架构和映射的整体探索。
•	2024 DATE-DeepFrack: A Comprehensive Framework for Layer Fusion, Face Tiling, and Efficient Mapping in DNN Hardware Accelerators：重点研究如何在给定的硬件加速器上自动进行“层融合、分块（Tiling）和映射（Mapping）”
•	2021 -ZigZag: Enlarging Joint Architecture-Mapping Design Space Exploration for DNN Accelerators：（gemini）这是目前学术界做 Dataflow 分析的黄金标准。建议将加速器参数输入到 ZigZag 工具（开源的）中进行测试，如果在某些特定 workload（如小尺寸卷积或深度可分离卷积）上，实际利用率击败了常规设计的理论上限，你的体系结构 Paper 也就基本成型了。

________________________________________

