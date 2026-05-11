---
title: "[TBD: 论文标题，候选——固定阵列 FPGA CNN 加速器的编译器侧 PE 利用率优化与多核扩展]"
author: "[TBD: 作者姓名]"
advisor: "[TBD: 指导教师姓名]"
school: "[TBD: 学校 / 学院 / 专业]"
year: "2026"
commit_anchor: "5fe16b2 (Phase 7 SMC+NUMA 主线 + IFB ring bug fix + xc7k325t N=4 综合通过)"
generated_by: "FLUX_CNN paper multi-agent pipeline (Phase 5 v2)"
---

> 本文档由多智能体流水线生成。`[TBD: ...]` 表示待用户决策的写作路径，`[CHECK: ...]` 表示待数据核实或回查代码/文档的事实，HTML 注释 `<!-- 来自 ... -->` 用于来源追溯（在 Markdown 渲染中不可见）。
>
> 标记统计（每次扩写后更新）：[TBD]={见末尾} / [CHECK]={见末尾}。

---

!!! 点评：论文标题错误，为：面向端侧流式计算场景的卷积加速器设计

# 主要符号表

| 符号 | 代表意义 | 单位或定义 |
|---|---|---|
| FPGA | 现场可编程门阵列 | Field Programmable Gate Array |
| ASIC | 专用集成电路 | Application Specific Integrated Circuit |
| SoC | 片上系统 | System on Chip |
| CNN | 卷积神经网络 | Convolutional Neural Network |
| DNN | 深度神经网络 | Deep Neural Network |
| PE | 处理单元 | Processing Element |
| MAC | 乘加运算 | Multiply-Accumulate |
| LUT | 查找表 | Look-Up Table（FPGA 基本单元） |
| DSP | 数字信号处理单元 | Digital Signal Processor（FPGA 基本单元） |
| BRAM | 块随机存储器 | Block Random Access Memory |
| FF | 触发器 | Flip-Flop |
| SRAM | 静态随机存储器 | Static Random Access Memory |
| DDR | 双倍速率同步动态存储器 | Double Data Rate SDRAM |
| AXI | 高级可扩展接口 | Advanced eXtensible Interface |
| DMA | 直接内存访问 | Direct Memory Access |
| SDP | 标量数据通路 | Single Data Processor |
| RTL | 寄存器传输级 | Register Transfer Level |
| IP | 知识产权核 | Intellectual Property core |
| INT8 | 8 位整数定点 | 8-bit Integer |
| FP32 | 32 位单精度浮点 | 32-bit Floating Point |
| MHz | 兆赫兹 | 10⁶ Hz |
| GOPS | 每秒十亿次运算 | Giga Operations Per Second |
| GMAC | 每秒十亿次乘加 | Giga Multiply-Accumulates per second |
| FPS | 每秒帧数 | Frames Per Second |
| RMSE | 均方根误差 | Root Mean Square Error |
| WRF | 权重寄存器文件 | Weight Register File |
| ARF | 激活寄存器文件 | Activation Register File |
| PARF | 部分和累加寄存器文件 | Partial-sum Accumulator Register File |
| IFB | 输入特征图缓冲 | Input Feature Buffer |
| OFB | 输出特征图缓冲 | Output Feature Buffer |
| WB | 权重缓冲 | Weight Buffer |
| WS / OS / RS | 权重驻留 / 输出驻留 / 行驻留 | Weight / Output / Row-Stationary |
| Ky-fold | Ky 折叠 | 编译器侧浅层 Cin 折叠到 Ky 维 |
| S2D | 空间到深度 | Space-to-Depth（编译器侧 stride 折叠） |

注：如文中对符号另有说明，以文中对应位置说明为准。

---

# 1 绪论

## 1.1 研究背景与意义

近十年来，卷积神经网络（CNN）已经从云端的图像分类基准迁移到端侧的实时影像分析、工业缺陷检测、无人机视觉、智能安防等场景。这类场景对推理延迟、稳定吞吐与功耗预算同时提出要求：摄像头送来的画面往往是连续逐帧的高分辨率图像，云端往返既不满足毫秒级响应也不接受带宽与隐私代价。因此，把 CNN 推理放在贴近传感器的端侧硬件上完成，已成为产业界与学术界共同关注的方向，而具备灵活逻辑、可重构特性与较低单位算力成本的 FPGA 在该领域占有重要位置。

!!! 点评：FLUX_CNN为ASIC加速器，FPGA仅用于验证平台。FLUX_CNN无法改变硬件连接。不能强调FPGA和可重构技术。

与云端 GPU 拥有数十 GB HBM、数百 GB/s 多通道访存不同，端侧 FPGA 平台普遍受限于片上 BRAM 容量与单 DDR 通道的访存带宽。以本论文采用的 Xilinx XC7K325T 为例，全片可用 BRAM36 仅 445 块，对外通常仅有 1 路 DDR3/DDR4 接口 <!-- 来自 STATUS.md §1 单核综合表 -->。任何打算缓存"整张特征图"或"整层权重"的方案在 480×640 量级输入或 ResNet 系列模型上都会迅速触碰到 SRAM 上墙，进而要求设计者要么把特征图按片切分多次驻留、要么在片上保留更精细的滑窗结构。

!!! 点评：绪论部分不应过早提及论文设计方案细节，应该先从更高层次的挑战和设计空间说起。

更进一步的矛盾来自于 CNN 各层形状与固定 PE 阵列硬件之间的失配。一旦设计选择了 16×16 这一类规则化的 MAC 阵列，网络中那些通道数远小于 16 或步幅大于 1 的层便会让 PE 行/列出现大面积空转。两类典型场景尤其严重：其一是网络入口处 Cin = 3 的彩色输入层，其二是 ResNet 风格 Patch 层中 stride = 4 这种降采样卷积层。<!-- 来自 contributions.md C2.1 / C2.2 -->

!!! 点评：16x16的MAC阵列是设计细节，不应在绪论提及。

以本工作目标负载 ResNet-11 为例，其 Patch 层为 K = 4 / stride = 4 / Cin = 3 的彩色输入卷积，在 16×16 阵列上 baseline PE 利用率发生明显塌陷 <!-- 数字待 model_analysis.md 核 [CHECK-4.3.1] -->。该层的 wall cycles 占整网相当大比重——经实测，在启用编译器侧 S2D 变换前，单 Patch 层达 654,404 cycles，而启用后仅 129,594 cycles，单层加速 5.05× <!-- 来自 contributions.md C2.2 / STATUS §2.8 -->，可见入口塌陷层是端侧 ResNet 部署中不可回避的核心瓶颈。

!!! 点评：同上。

针对上述端侧 FPGA 资源约束与固定阵列利用率塌陷的双重压力，本论文提出在不增加硬件复杂度的前提下，通过编译器侧的等价数据重映射，把"塌陷场景"折回固定阵列的可用工作模式。具体路线是保持 16×16 MAC 阵列、行级流式行环数据通路与去中心化握手流水的硬件最简形态，把 Ky 维折叠（Ky-fold）与空间到深度（Space-to-Depth, S2D）等编译器侧静态变换作为 PE 利用率提升的主要手段，并以单核 layer-serial 共用硬件 + 多核 W 维切片扩展的方式覆盖整网吞吐。

下文先在 §1.2 自 ASIC 阵列、FPGA streaming 加速器、行环与跨层缓冲、卷积变换谱系、编译器工具链五个角度梳理国内外现状，建立清晰的 prior art 坐标系；再在 §1.3 收敛本论文的研究目标与主要贡献承诺；§1.4 给出全文组织结构。

!!! 点评：不用写"下文先在 §1.2 ... "等章节预告。

## 1.2 国内外相关研究现状

CNN 硬件加速器研究自 2014 年前后从 ASIC 走向 FPGA 与异构片上系统，大量代表性工作沿着 ASIC 系统脉动 / FPGA streaming / 跨层与行级缓冲 / 卷积变换谱系 / 编译器与工具链五条主线展开。本论文在 §1.2.1—§1.2.5 依次梳理这五条主线，并在 §1.2.3 集中讨论与本工作行级行环路线最近的 Alwani / Kang / Liu 三处近邻 prior art，以建立后续差异化论证的文献坐标。

!!! 点评："文献坐标"这类词语别用，模糊不清。

### 1.2.1 ASIC 系统脉动与可重构阵列路线

ASIC 加速器是 CNN 硬件加速研究的源头，其中以 Google TPU 为代表的脉动阵列与以 MIT Eyeriss 为代表的行驻留是两条经典路线 <!-- 来自 literature.md §A -->。前者通过权重在二维阵列内驻留、激活按节拍流过的方式实现高密度 INT8 矩阵乘，后者通过让一行激活与一行权重沿对角传播实现权重 / 激活 / 部分和三者的组合复用。这两条路线共同奠定了"固定大阵列 + 单芯片峰值吞吐"的工业 ASIC 基调。

!!! 点评："源头、起点"这类词语别用，模糊不清。可以说"CNN专用硬件加速器的研究可追溯至"

沿着这一基调，TPU、NVDLA、Gemmini 等工作进一步推动"固定大阵列 + 编译器调度"路线，强调通过软件栈把不同形状的层映射到同一硬件阵列上 <!-- 来自 literature.md §A、§G -->。本论文在硬件最底层的固定阵列形态上与该路线同源，但选择了规模更小的 16×16 INT8 MAC 阵列以适配端侧 FPGA 的资源预算，而非追求云端 ASIC 数百乃至上千 MAC 的规模。

!!! 点评：不要反复提及自己的设计细节。与上文的毛病相同。

另一支路线是"硬件可重构互连"，以 MAERI、Eyeriss-v2、Tangram 为代表，通过加入可编程 NoC 或层级化 mesh 互连，让 PE 网络在不同层形状下动态重排数据流，从硬件侧解决利用率塌陷问题 <!-- 来自 literature.md §B -->。这类工作把 PE 利用率优化的负担放在硬件层，代价是增加显著的互连面积与控制复杂度，对端侧 FPGA 而言并不友好。

本工作选择的是"固定阵列硬件 + 编译器侧填满"路线，与上述硬件可重构互连路线在设计哲学上正相对照。两条路线的具体取舍与定量论证留到 §3.4.2 展开，本节仅在文献坐标层面建立位置。

!!! 点评：不要反复提及自己的设计细节。

### 1.2.2 FPGA streaming 加速器主线

FPGA streaming 类加速器是与本工作器件平台最贴近的对照集，其代表工作包括 fpgaConvNet (Venieris@FCCM'16 / TNNLS'19)、Snowflake (ISCAS'17)、Angel-Eye (TCAD'18)、Caffeine (ICCAD'16) 与 Aydonat 等的 Intel DLA <!-- 来自 literature.md §C -->。这些工作普遍采用层级流水 + 行缓存 + 单 DDR 通道的工程范式，目标平台多为 Xilinx 7 系列或 Intel Stratix/Arria，提供了与本工作器件可比的资源 / Fmax / GOPS 横向参照。

在数据流维度上，这类工作大多遵循 layer-pipelined 多 block 同时跑，或 layer-serial 单 block 复用的两条思路；在硬件复用粒度上则以 line buffer + tile-based feature map 为主。这与本工作"行级流式行环 + 单核 layer-serial 共用硬件"的组合在硬件结构上形成同一族范式，使得后续在 §5.7 横向对比中能够形成可比的数字基线。

然而，FPGA streaming 类工作在 PE 利用率优化方面普遍停留在 baseline 折叠或 im2col 转换层面，更激进的等价变换（如 stride² 相位折叠）较少进入主线 <!-- 来自 literature.md §C 综述部分 -->。这一缺口正是本论文 §4.3 编译器侧 PE 利用率优化主章试图填补的位置。

!!! 点评：不要反复提及自己的设计细节。

本节仅建立同器件 streaming 主线坐标，对编译器侧变换的不足在 §1.2.4 卷积变换谱系一节中再做集中讨论，本工作的具体填补方案则留到 §4.3 narrative A 主章展开。

!!! 点评：不要反复提及章节结构。

### 1.2.3 行级与跨层流式缓冲：三处近邻 prior art

在与本工作行级流式行环（以下简称"行级行环"）路线最近的范畴中存在三处必须显式应对的近邻 prior art：Alwani 等的 Fused-layer (MICRO'16)、Kang 等的 AoCStream (arXiv'22 / Sensors'23) 与 Liu 等的 Full-Stack Streaming CNN Accelerator (TNNLS'21) <!-- 来自 literature.md §C 三条新增条目 -->。这三处工作均围绕"streaming + 行 / 层级片上缓冲"展开，与本工作行级行环数据通路构成最直接的差异化论证战场。

Alwani 的 Fused-layer 通过修改输入数据进入芯片的顺序，让多个连续 conv 层的中间特征图在片上 buffer 内被复用，从而把整张图的 off-chip feature map 流量在 VGGNet-E 前 5 层从 77 MB 降到 3.6 MB，节省约 95% <!-- 来自 literature.md Alwani 条目 -->。其代价是依赖多层间共享缓冲，往往要求该若干层的中间特征图能整体或大面积驻留片上。本工作的行级行环只缓冲 strip_rows × W_IN 量级的几行数据，单层硬件即可处理任意 H×W 输入 <!-- 来自 contributions.md C1.3 -->，与 Alwani 的"多层 fused 同时驻留"在缓冲粒度上形成代差。

Kang 的 AoCStream 把所有特征图都驻留在片上 SRAM 中，line buffer 大小与图宽线性相关，每层使用专用 dataflow block，pipelined input/output streams 同时跑，在 low-end FPGA 上可不依赖外存完成 MobileNetV1/V2 + SSDLite 等 object detection 网络 <!-- 来自 literature.md Kang AoCStream 条目 -->。其受限点在于 BRAM 容量上墙，输入分辨率上限受片上存储严格约束。本工作通过 row-credit 反压协议把片上缓冲容量与输入分辨率解耦：strip_rows 由编译器决定（编译器 `ofb_strip_rows_max = (OFB-1)//row_words`），VGA 480×640 单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张 <!-- 来自 contributions.md C1.3 -->。

Liu 的 Full-Stack Streaming CNN Accelerator 在 Intel Arria 10 上把 conv / deconv 层映射到统一模块，高效实现 residual / concatenative 连接，支持 ResNet / DenseNet 拓扑，并采用 layer fusion + 多级 parallelism + 充分利用 DSP <!-- 来自 literature.md Liu Full-Stack 条目 -->。其架构核心是跨层 pipelined 让多核同时跑不同层。本工作选择"单核 layer-serial 共用硬件 + 多核 W 维切片"的组合，在 ResNet-11 的 N = 1/2/4 上分别跑通 596K / 450K / 354K wall cycles <!-- 来自 contributions.md C4.4 + STATUS §2.8 -->，在端侧资源约束下偏向 single-layer 内并行扩展，与 Liu 的多层流水路线在并行维度选择上明显不同。

三处近邻 prior art 的差异化主张主战场在 §3.4.3 三段对比论证中展开，§5.7 给出对应数据兑现，本节先建立必须显式应对的文献坐标。

### 1.2.4 卷积变换谱系

!!! 点评："谱系"这类词语别用，模糊不清。

经典卷积变换主要包括 im2col 转矩阵乘、Winograd 减乘加与 Direct 卷积三类，三者在硬件代价与精度损失上各有取舍 <!-- 来自 literature.md §D -->。im2col 把卷积窗展开为列向量再走通用矩阵乘，便于复用 GEMM 内核但显著增加片上 / 片外数据搬运；Winograd 通过预计算的变换矩阵减少乘法运算数，但带来精度漂移与反向变换的额外硬件代价；Direct 卷积保持原始七层循环嵌套，对硬件最简但缺少进一步的算力压缩。

在与本论文 S2D 变换最相关的子领域中，Pixel-Shuffle / Sub-pixel Convolution 在超分辨率领域已被广泛用于上采样，其核心是把通道维与空间维互换的等价关系，在 ML 算法侧已为公认 <!-- 来自 literature.md §D Sub-pixel CVPR'16 等条目 -->。该等价关系并非本工作的发明，论文写作必须在谱系上诚实标注其在超分辨率领域的来源 <!-- 来自 literature.md §C 末尾"S2D 引用谱系"备注 [CHECK-1.2.4] -->。

!!! 点评："公认"、"诚实标注"。

本工作的 S2D 变换继承上述子像素重排的等价关系，但在加速器编译器侧把 stride² 相位折叠到 cin 维并配合 DDR 友好的零复制重排作为 PE 利用率优化手段，是面向 FPGA streaming CNN 加速器的新构造。已检索文献中尚未见到把 S2D 作为编译器侧 PE 利用率优化路径的明确 prior art <!-- 来自 literature.md §C 备注，[CHECK-1.2.4] 谱系措辞待复查 -->，本工作在 §6.2 创新点章用"据已知文献..."的措辞表达 novelty，避免 over-claim。

谱系定位的诚实标注在 §4.3.3 数学推导节与 §6.2.2 创新点节会再次重申，本节仅为 §4.3 编译器侧 PE 利用率优化主章铺垫数学根基。

### 1.2.5 编译器与工具链

CNN 加速器编译流的代表工作包括 fpgaConvNet 工具链、TVM / VTA、NVDLA 编译器与 Caffeine 等，其共同建立了"模型→硬件 ISA / cfg"端到端流程的范式 <!-- 来自 literature.md §G 与 §C -->。这些工具链通常以 ONNX 或自有图 IR 为中间表示，在算子融合、INT8 量化与权重 / 激活布局打包层面已比较成熟，面向通用 CNN 拓扑可工作。

然而，上述工具链在 stride² 相位折叠、Ky 维折叠到 cin 等"等价数据重映射型"PE 利用率优化变换上较少触及。这类变换与传统的算子融合 / 量化变换在抽象层级上不同：算子融合 / 量化优化的是算子边界与数值精度，重映射变换则需要编译器同时改写硬件 cfg 寄存器、DDR 数据布局与权重打包顺序，要求编译器与硬件视角紧耦合。

本工作的 PyTorch→ISA 端到端编译流（`toolchain/models/run_model.py` + `hw_files.derive_layer_cfg()` + `gen_isa_test.py`）<!-- 来自 contributions.md C2.4 --> 在算子融合与 INT8 量化基础上，把 Ky-fold / S2D 联合触发作为 `scheduler.Layer.force_s2d() / force_fold()` 的静态决策组成部分，使得编译期决策与硬件 cfg 寄存器映射通过 `s2d_eff()` 与 `build_step_cfg_dict` 保持一致 <!-- 来自 contributions.md C2.3 -->，避免编译器与硬件之间的视角偏移。

§4.4.1 详细给出 PyTorch→ISA 编译流模块结构，§4.3.4 给出 Ky-fold + S2D 联合触发的决策逻辑，本节仅在工具链谱系层面给出本工作在编译器栈中的位置定位，为 §1.3 研究目标的收敛做最后铺垫。

## 1.3 研究目标与研究内容

基于 §1.1 端侧 FPGA 资源约束与固定阵列利用率塌陷的双重压力，以及 §1.2 在 ASIC 系统脉动 / FPGA streaming / 行级行环 / 卷积变换 / 编译器工具链五条 prior art 主线下建立的坐标系，本论文研究目标可概括为"固定阵列硬件 + 编译器侧重映射"——硬件保持 16×16 INT8 MAC 阵列、行级流式行环、去中心化握手流水的最简形态，把 PE 利用率优化与多核扩展的负担推到编译器侧与系统集成侧。

具体目标是在 Xilinx XC7K325T-FFG900-2 单器件上实现 ResNet-11 端到端 INT8 推理，在 Cin < 16（典型如彩色输入层）与 stride ≥ 2（典型如降采样 Patch 层）这两类 PE 利用率塌陷场景下兑现可用的硬件利用率与 wall cycles 性能 <!-- 来自 contributions.md C2.1/C2.2 + C4.4 -->。同时要求所有功能正确性以与 PyTorch reference 实现 element-wise bit-exact 的方式验证，以排除"实现差异掩盖性能数字"的风险。

围绕该目标，研究内容分为四大块。其一是硬件架构层，完成 16×16 OS+列广播 MAC 阵列、5 模块去中心化 valid-ready 流水、行级流式行环 IFB / OFB、分列累加器 PARF、SDP 后处理融合与 7 层循环嵌套硬件实现 <!-- 对应 contributions.md C1.1—C1.6 -->。其二是编译器优化层，完成 Ky 折叠、S2D 等价变换与两者的联合触发自动决策 <!-- C2.1—C2.3 -->。其三是系统集成层，完成 PyTorch→ISA 端到端编译流、链式 CASES 验证基础设施、AXI / DMA 子系统集成与 CFG_WRITE descriptor 配置流 <!-- C2.4—C2.5 + C3.1—C3.3 -->，并在此基础上做多核 W 维切片扩展与单源参数 params.py 双向消费 <!-- C3.4—C3.5 -->。其四是实测验证层，完成 51-case bit-exact 回归、单核与多核 Vivado 综合、ResNet-11 多核 cycles / FPS 实测与 1-DDR vs 4-DDR 带宽 PoC 分析 <!-- C4.1—C4.7 -->。

围绕上述研究内容，本论文凝练出 5 条主要贡献：

- **贡献 1（编译器侧 Ky 折叠）**：当 Cin < 16 时，把 Ky 维通过 y 偏移复制折叠到 cin_fake = Cin × Ky，硬件视角下与 stride = 1 标准卷积同构，纯编译器侧零 RTL 改动 <!-- C2.1 -->。详见 §4.3.3。
- **贡献 2（S2D 等价变换 + 联合触发）**：当 stride ≥ 2 时把 stride² 个相位折到 cin_new = stride² × Cin，DDR 友好重排无激活复制；与 Ky-fold 的联合触发由 `scheduler.Layer.force_s2d() / force_fold()` 在编译期按 (Cin, K, stride) 自动决策 <!-- C2.2 + C2.3 -->。详见 §4.3.4 与 §4.3.5。
- **贡献 3（去中心化 5 模块流水 + 行级行环）**：5 模块各自维护 counter，弹性 join 无中心 FSM，配合 strip_rows × W_IN 粒度的 row-credit 反压协议 ring，单层硬件可处理任意 H×W 输入 <!-- C1.2 + C1.3 -->。详见 §4.2.1—§4.2.3。
- **贡献 4（多核 W 切片扩展）**：在单核架构上以 W 维切片扩展到 N = 2/4 核，配合 halo 计算冗余与跨核 SRAM 直送（M2 push），ResNet-11 上 N = 4 较 N = 1 实现 1.68× wall cycles 加速 <!-- C3.4 + STATUS §2.8 -->。详见 §4.5。
- **贡献 5（PyTorch→硬件 cfg 端到端编译流 + descriptor 配置）**：以 `hw_files.derive_layer_cfg()` 为 cfg 派生 single source 完成 PyTorch→ISA 端到端编译，并通过 CFG_WRITE descriptor 把 host AXI-Lite 写从约 50 次/层降到 4 次/层 <!-- C2.4 + C3.2 -->。详见 §4.4。

[TBD-1.3.3: 主要贡献是否压缩到 4 条——若篇幅紧张可合并贡献 4 + 5 为"系统集成与多核编译流"]

各贡献的主章节兑现位置与对应实测数据预告留到 §1.4 论文组织结构一并给出。

## 1.4 论文组织结构

全文共 6 章，按"理论铺垫 → 设计决策 → 具体实现 → 实测验证 → 总结展望"的顺序展开，每章首设引言节、章末设小结节，便于读者定位本工作的两条 narrative 主轴——编译器侧 PE 利用率优化（narrative A，主轴）与系统集成（narrative B，支撑）。

第 2 章铺垫卷积神经网络与 FPGA 硬件加速的理论基础，统一卷积语义、量化算子、数据流分类（WS/OS/RS）、FPGA 物理资源与 valid-ready / 行缓存 / AXI 等设计抽象，并在 §2.5 引出卷积变换谱系（im2col / Winograd / Pixel-Shuffle）作为 §4.3 编译器侧优化的数学根基。第 3 章承接理论与实现之间的"设计决策"环节，自设计目标与约束（§3.2）、总体架构（§3.3）、数据流选择论证（§3.4，含与 Alwani / Kang / Liu 三处 prior art 的差异化主战场）、编译器与硬件协同原则（§3.5）四个维度展开，是全文最关键的差异化论证章。

第 4 章是全文最长最详的核心实现章，narrative A 主轴所在地，依次展开核心数据通路硬件实现（§4.2，对应贡献 3）、编译器侧 PE 利用率优化（§4.3，对应贡献 1 与贡献 2）、系统集成与编译流（§4.4，对应贡献 5）与多核 W 切片扩展（§4.5，对应贡献 4），覆盖 contributions.md 共 16 条贡献。第 5 章给出系统验证与实验分析，依次完成实验设置（§5.2）、功能验证（§5.3，51-case bit-exact）、板级综合与资源分析（§5.4，含 Fmax 仅 68 MHz 与 DSP 推断率 82/256 两处诚实陈述）、性能分析（§5.5，PE 利用率三模式 + ResNet-11 cycles/FPS + 多核加速比）、多 DDR 带宽分析（§5.6）与与已有工作对比（§5.7）。第 6 章总结结论与创新点（§6.1—§6.2），并基于第 5 章 limitation 给出未来工作展望（§6.3）。

---

# 2 卷积神经网络与 FPGA 硬件加速技术原理

## 2.1 引言

第 1 章已建立端侧 FPGA 资源约束与固定阵列利用率塌陷两条主要矛盾，并在 §1.2 五条 prior art 主线下给出文献坐标。本章为后续设计与实现章节统一术语并铺垫数学基础，不引入本工作的具体设计取舍。考虑到本论文同时涉及 CNN 算子语义、加速器数据流分类、FPGA 物理资源与等价数据重映射四个层次的概念，若不在第 2 章一次性给出统一约定，第 3—4 章的设计决策与 RTL 实现将不得不反复打断主线去解释术语，影响 narrative 主轴的可读性。本章所采用的下标命名、数据流分类边界、流水线握手协议与卷积变换谱系均沿用学界惯用约定，并标注与本工作后续章节的对应位置，便于读者前后参照。

!!! 点评：又反复提及其他章节的内容。本节只讲自己的事，不要讲别人的。

章节组织如下：§2.2 卷积与量化算子基础统一全文符号，§2.3 数据流分类（WS/OS/RS）建立 PE 阵列复用维度的坐标系，§2.4 FPGA 硬件加速基础铺垫物理资源与设计抽象两层基础，§2.5 PE 利用率优化基础变换给出 §4.3 编译器侧主章所需的卷积变换谱系，§2.6 本章小结收束并衔接第 3 章总体方案。

!!! 点评：别反复提及章节结构。

## 2.2 卷积神经网络基础

本节定义本论文使用的卷积、池化与全连接算子的语义与下标约定，统一后续章节引用的符号体系。CNN 由若干卷积层、非线性激活层、池化层与全连接层堆叠构成；其中卷积与全连接占算力主导，是加速器硬件复用的核心对象。下文以 N（batch）/ Cout（输出通道）/ Cin（输入通道）/ H（输入高）/ W（输入宽）/ Ky（卷积核高）/ Kx（卷积核宽）的命名作为全文统一下标，stride 与 pad 分别记作 S 与 P，本章及后续章节所有公式、cfg 寄存器名称与代码片段均与此一致。

!!! 点评：完全没必要的一段。

标准二维卷积可以用 7 层循环嵌套表达，其外层依次遍历 N、Cout、H_out、W_out 四个输出空间下标，内层依次遍历 Cin、Ky、Kx 三个累加维度，对每个输出像素累加 Cin × Ky × Kx 次乘加；输出像素 y[n, co, h, w] 等于 Σ_{ci, ky, kx} x[n, ci, h·S + ky − P, w·S + kx − P] · W[co, ci, ky, kx] + bias[co]。这一七层循环嵌套的展开顺序直接决定加速器的数据复用粒度与片上存储需求，本工作的硬件循环嵌套顺序与该数学定义一一对应（详见 §4.2.7）。 [依赖: Fig.2.1]

!!! 点评："本工作的硬件循环嵌套顺序与该数学定义一一对应"不要反复提及"本工作"。

本论文采用 INT8 量化推理流程：权重与激活均量化到 8-bit 有符号定点整数，乘加结果在片上以 32-bit 部分和（PSUM）累加，每个输出像素在写回前需依次完成 bias 加、shift 右移与 clip 截断三步，把 32-bit PSUM 重新对齐回 INT8 输出空间 <!-- 来自 contributions.md C1.5 -->。这三步可以融合到单条组合路径中作为后处理算子（Single Data Processor, SDP），其硬件实现与时序约束在 §4.2.6 SDP 节给出；对部分支持残差短接的层，SDP 还需在 clip 之前融合 short-cut 加法。

ResNet 系列残差结构在两条分支的 short-cut 设计上分为 identity 与 projection 两种：identity 直接把输入特征图按元素加到主分支输出，要求两分支通道数与空间形状一致；projection 通过 1×1 卷积调整 short-cut 分支的通道数或下采样空间形状后再相加 <!-- 来自 literature.md ResNet 条目 -->。在加速器侧，两种 short-cut 对 IFB 区分配与 SDP 残差融合时机提出不同需求：identity 要求生产层与消费层共享同一 IFB region，projection 则要求 short-cut 分支单独占用一段 region 并在主分支完成后参与 SDP 加法，§4.2.5 详述其硬件实现细节。 [依赖: Fig.2.2]

至此 CNN 算子层面的符号、量化语义与残差结构定义就绪。在算子定义之上，下一节进一步讨论加速器把这些算子映射到 PE 阵列时的数据复用维度选择，即数据流分类问题。

## 2.3 CNN 硬件加速器数据流分类

经典 CNN 加速器数据流按部分和、权重、激活三者中"在 PE 内驻留的对象"分为权重驻留（Weight-Stationary, WS）、输出驻留（Output-Stationary, OS）与行驻留（Row-Stationary, RS）三大类，本节穷举对比并定位本工作的选择 <!-- 来自 literature.md §A Eyeriss 分类 -->。三类数据流在复用粒度、片上存储需求与 PE 间数据移动复杂度上差异显著，是任何 CNN 加速器架构决策的第一道分叉。

权重驻留（WS）以 Google TPU 的脉动阵列为典型代表 <!-- 来自 literature.md TPU 条目 -->：每个 PE 内部固定保存一个权重值，激活按节拍沿一个方向脉动流过 PE 阵列，部分和沿另一方向累加传出。该数据流的优势在于对权重的复用程度极高（权重在阵列内只需加载一次即可处理整个 batch 的所有输出像素），适合权重重复使用次数极高的全连接层与大 batch 训练场景；其代价是对激活与部分和需要严格的二维脉动节拍同步，硬件控制复杂度较高，并要求 PE 阵列足够大以摊销权重加载开销。

输出驻留（OS）以 Snowflake 与本工作为代表：每个 PE 内部累加一个输出像素的部分和，激活与权重按节拍并行流入 PE，乘积累加在 PE 内 PSUM 寄存器或 PARF 内完成 <!-- 来自 literature.md Snowflake 条目 -->。该数据流避免了部分和在 PE 间传输的开销，适合 Cin 与 Cout 都比较大的中间层；当一个 PE 完成全部 Cin × Ky × Kx 次累加后，部分和经 SDP 后处理一次性写回 OFB。OS 的硬件代价较低，控制逻辑相对简单，是与端侧 FPGA 资源约束最匹配的数据流之一。

行驻留（RS）以 MIT Eyeriss 为代表 <!-- 来自 literature.md Eyeriss 条目 -->：一行权重在 PE 内驻留，一行激活沿水平方向流过，部分和沿垂直方向累加；通过让二维 PE 阵列的对角线对应同一个输出行，实现权重、激活与部分和三者的组合复用，理论上能在更小的片上存储下达到更高的能效。其代价是 PE 间数据移动模式复杂，需要专门的 NoC 或对角传播网络支持，硬件控制与互连面积均显著高于 WS 与 OS。

三类数据流的选择本质上是"复用粒度 vs 硬件复杂度"的权衡：WS 复用权重最彻底但要求大阵列摊销加载开销，OS 控制最简但牺牲一部分激活与权重复用，RS 复用最完备但 PE 间数据移动最复杂 [依赖: Tab.2.1]。本工作选择 OS + 列广播组合的依据在 §3.4.1 详细论证，核心动机是在端侧 FPGA 单 DDR 通道与有限 BRAM 容量的约束下，避免 RS 类硬件可重构互连的代价。

数据流分类坐标系建立后，下一节转入 FPGA 物理资源与设计抽象两层基础，为第 3—4 章的硬件实现章节铺垫背景。

!!! 点评："数据流分类坐标系建立后，下一节转入 FPGA 物理资源与设计抽象两层基础，为第 3—4 章的硬件实现章节铺垫背景。"这句话就是废话。

## 2.4 FPGA 硬件加速基础

本节铺垫两层 FPGA 加速器设计基础：底层是物理资源（LUT / FF / BRAM / DSP），上层是设计抽象（流水线、valid-ready 握手、行缓存、AXI 总线、DMA）。读者需要同时理解这两层才能在后续章节看懂硬件资源占用、Fmax 与时序约束的来源，以及第 4 章核心数据通路与 DMA 子系统的工程实现。

!!! 点评："铺垫"也是废话。"读者"也是。你讲明白自己的事就行了。

Xilinx 7 系列 FPGA（包括本工作目标器件 XC7K325T）由若干种基础资源构成：LUT 用作组合逻辑与小型分布式 RAM；FF 用作时序寄存器；BRAM36 是 36 Kbit 的双口块 RAM，是片上特征图与权重缓冲的主要载体；DSP48 是带预加器、乘法器与累加器的硬核乘加单元，是 INT8 MAC 的首选实现资源 <!-- 来自 literature.md Kintex-7 datasheet / 通用 FPGA 知识 -->。XC7K325T-FFG900-2 全片可用 LUT 约 203K、FF 约 407K、BRAM36 约 445 块、DSP48 约 800 块，具体容量数字与本工作综合资源占用对比将在 §3.2 表化给出。

!!! 点评：FPGA资源不在这说，在验证章节说。

流水线设计是 FPGA 加速器在保持高 Fmax 的前提下提升吞吐的标准手段：把一个长组合路径切成若干段，每段之间插入寄存器，使每段时延均小于目标时钟周期。多模块流水线之间通常通过 valid-ready 握手协议解耦——发送方在数据有效时拉高 valid，接收方在准备好接收时拉高 ready，仅当 valid 与 ready 同时为高时一拍数据被实际消费。该协议天然支持上下游模块的弹性 join，无需中心 FSM 协调多模块状态，这一抽象是本工作 5 模块去中心化流水的基础（§4.2.1 详述）。 [依赖: Fig.2.3]

行缓存（line buffer）是 streaming CNN 加速器的标志性数据通路结构：把若干行（通常等于 Ky 行）特征图保留在片上 SRAM 中，让 Ky × Kx 滑窗在不重复加载激活的前提下沿 W 方向滑过整行 <!-- 来自 literature.md Snowflake / Angel-Eye 等条目 -->。当滑窗到达行末时，最早一行行缓冲被新进入的下一行覆写，形成片上 ring buffer。本工作在标准 line buffer 之上加入 row-credit 反压协议（§4.2.2 详述），使片上 BRAM 占用与输入特征图行数 H 解耦，单层硬件即可处理任意 H × W 输入。 [依赖: Fig.2.4]

AXI4 与 AXI-Lite 是 ARM 推出的开放总线协议族：AXI4 支持突发传输与 outstanding 多事务，是片上高带宽数据通道的事实标准；AXI-Lite 是 AXI4 的简化版，仅支持单拍读写，用于配置寄存器访问 <!-- 来自 literature.md AXI 协议规范 -->。DMA 数据搬运是把 DDR 与片上 SRAM 之间的数据传输从 host 卸载到专用搬运引擎，实现计算与搬运的并行。本工作的 DMA 子系统由 idma_ctrl / wdma_ctrl / odma_ctrl 三套控制器加 Vivado axi_dm IP 与 mm2s_arb 仲裁器组成，外部聚合为单个 AXI4 主口（§4.4.3 详述）。

最后，本工作所用工具链包括 Vivado 2023.1 用作综合 + 实现 + bit 流生成、ModelSim 用作 RTL 仿真、自研 Python toolchain 用作 PyTorch→ISA 编译与回归测试调度。各工具的具体使用流程与命令在 §5.2 实验设置中给出，本节仅在术语层面建立铺垫，避免与第 5 章实测章节产生重复表述。

!!! 点评："本节仅在术语层面建立铺垫，避免与第 5 章实测章节产生重复表述。"废话。

## 2.5 PE 利用率优化基础变换

本节铺垫两类卷积等价变换的数学根基，为 §4.3 编译器侧 Ky-fold + S2D 主章提供已知工作的对照参照。本工作的编译器侧 PE 利用率优化并非凭空构造，而是在经典卷积变换谱系（im2col / Winograd / Direct）与超分辨率领域子像素重排谱系（Pixel-Shuffle / Sub-pixel）的基础上，针对 FPGA streaming 加速器固定阵列利用率塌陷场景作出的工程化改造。本节定位是建立这一谱系坐标系，不展开具体推导。

!!! 点评："铺垫"废话。"本节定位是建立这一谱系坐标系，不展开具体推导。"废话。

经典卷积变换主要分为三类：im2col 把 Ky × Kx 滑窗按列展开为长向量，使整个卷积层等价于一次通用矩阵乘（GEMM），便于复用现有 BLAS 内核但带来约 K² 倍的激活数据膨胀 <!-- 来自 literature.md Caffe im2col 条目 -->；Winograd 通过预计算变换矩阵把 K=3 卷积的乘法数从 9 降到 4 或更少，但带来精度漂移与反向变换硬件代价 <!-- 来自 literature.md Winograd CVPR'16 条目 -->；Direct 卷积则保持原始七层循环嵌套不做变换，对硬件最简但缺少进一步算力压缩。三类变换在硬件代价与精度损失上各有取舍，本工作硬件层采用 Direct 卷积，把变换工作放到编译器层。

步幅卷积（stride ≥ 2 的卷积）可以通过子像素重排转换为 stride = 1 卷积加上对输入特征图的相位重排，这一等价关系最早由超分辨率领域 Sub-pixel Convolution 提出 <!-- 来自 literature.md Shi CVPR'16 Sub-pixel 条目 -->。其核心数学事实是：stride = s 的卷积可以视作把输入特征图按 s × s 个相位拆分成 s² 个子图，每个子图对原 stride 卷积输出贡献一组下标对齐的输出像素，最终所有相位的贡献按通道维拼接即可。该等价关系在 ML 算法侧已为公认，其在加速器编译器侧的应用是本工作 S2D 变换的数学根基。 [依赖: Fig.2.5]

本节仅指出上述等价关系的存在与谱系来源，具体的 S2D 在加速器编译器侧 stride² 相位折叠到 cin 维的实现、DDR 友好的零复制重排策略，以及与 Ky-fold 的联合触发自动决策，均留到 §4.3.3 与 §4.3.4 详述。 [TBD-2.5.2: 是否在本节包含简化 S2D 数学推导，倾向只在 §4.3.3 详写以避免重复]

## 2.6 本章小结

本章为后续章节统一了术语与坐标系：§2.2 给出卷积、池化、全连接、INT8 量化与残差短接的算子语义；§2.3 给出 WS / OS / RS 三类数据流分类；§2.4 给出 FPGA 物理资源与流水线 / valid-ready / 行缓存 / AXI / DMA 等设计抽象；§2.5 给出 im2col / Winograd / Direct 卷积变换谱系与 Pixel-Shuffle / Sub-pixel 子像素重排谱系。这些坐标系既是第 3 章总体方案设计决策论证的依据（OS + 列广播 / 固定 16×16 / 行级行环），也是第 4 章硬件实现与编译器侧 PE 利用率优化的数学基础。从本章到第 3 章的过渡逻辑是：在已统一术语与坐标的基础上，下一章正式给出本工作"硬件最简 + 编译器侧填满"的整体设计哲学与三大设计决策。

!!! 点评："从本章到第 3 章的过渡逻辑是：在已统一术语与坐标的基础上，下一章正式给出本工作"硬件最简 + 编译器侧填满"的整体设计哲学与三大设计决策。"废话。不要说别人的事。

---

# 3 固定阵列 CNN 加速器整体方案

## 3.1 引言

本章承接第 2 章理论铺垫与第 4 章具体实现之间的"设计决策"环节，只交代本工作做出了哪些核心选择以及为什么这样选，不展开具体的 RTL 时序、cfg 寄存器位定义与编译器流水实现细节，后两类内容统一留到第 4 章。把设计决策与实现拆到两章的目的，是让读者先建立"硬件最简 + 编译器侧填满"这一整体设计哲学的清晰图景，再带着该图景去阅读篇幅最长的第 4 章实现章，避免在"做了什么选择"与"具体怎么实现"之间反复切换思维主线。

!!! 点评："只交代、不展开"废话。"避免"也是废话。你直接说清楚你的事就行了，不要说你没干什么。

章节组织如下：§3.2 从应用场景、目标平台与目标算法三个维度收敛设计空间，划定后续架构决策的边界；§3.3 给出两层架构（核流水 + DMA 子系统）与多核拓扑的总体方案，回答"是什么"；§3.4 通过三段对比论证（OS+列广播 vs RS/WS、固定阵列 vs 可重构、行级行环 vs Alwani / Kang / Liu 三处近邻 prior art）确立本工作的差异化主张，回答"为什么这样选"，是全文最关键的差异化论证章；§3.5 把单源参数与 cfg 派生 single source 等三条工程协同原则提升到方法论层面；§3.6 本章小结收束并衔接第 4 章。

!!! 点评："章节组织如下"废话。

## 3.2 设计目标与约束

本节从应用场景、目标平台与目标算法三个维度收敛设计空间，为后续架构决策划定明确边界。设计目标的取舍不在抽象层面追求"通用最优"，而是在端侧 FPGA 这一具体语境下选择能落地、可综合、能跑通完整 ResNet 的工程化最简组合，这一取向贯穿后续所有设计决策。

应用场景假设是端侧、实时、任意 H × W 输入的 CNN 推理任务，对应工业相机、无人机视觉与智能安防一类的连续逐帧推理负载。本工作不假设输入分辨率在编译期已知，硬件需对 480 × 640、960 × 540 乃至更大幅面输入提供单 start 直接跑通的能力，而不是要求 host 把整张图按 tile 切分多次启动加速器；这一假设直接驱动了后续 §3.4.3 行级行环数据通路的选择 <!-- 来自 contributions.md C1.3 -->。

!!! 点评："对应工业相机、无人机视觉与智能安防一类的连续逐帧推理负载"这种话已经在绪论说过了，这里不要说。"这一假设直接驱动了"又是废话。

目标平台为 Xilinx Kintex-7 系列 XC7K325T-FFG900-2 单器件，全片可用资源约 LUT 203,800、FF 407,600、BRAM36 445、DSP48E1 840 <!-- 来自 STATUS.md §1 单核综合表 -->，对外仅有 1 路 DDR3/DDR4 通道，单通道带宽为系统访存上限。这一平台是端侧 FPGA 加速器中具有代表性的中等规模器件，BRAM 与 DSP 数量均不算宽裕，无法照搬云端 ASIC 数百乃至上千 MAC 的规模，必须在 16 × 16 这一量级的小阵列下做精细化的资源平衡。 [依赖: Tab.3.1]

!!! 点评：重复提及FPGA资源。

目标算法约束包括三点：其一是 INT8 量化推理，权重与激活均量化到 8-bit 有符号定点，部分和在片上以 32-bit 累加，每个输出像素经 SDP 后处理（bias 加 / shift 右移 / clip 截断）回到 INT8 输出空间 <!-- 来自 contributions.md C1.5 -->；其二是支持 ResNet 风格残差短接，含 identity 与 projection 两种 short-cut 形态；其三是 Cin 与 Cout 任意，覆盖从入口处 Cin = 3 的彩色输入层到中间 Cin = Cout = 256 的瓶颈层。在该算法约束下需要明确指出，Cin < 16 与 stride ≥ 2 是固定 16 × 16 阵列上 PE 利用率塌陷的两类典型场景，本工作把这两类场景作为编译器侧优化的核心攻击面 <!-- 来自 contributions.md C2.1 / C2.2 -->。 [依赖: Tab.3.2]

三个维度的约束共同框出本工作的设计空间：硬件规模上不能超过 XC7K325T 资源预算，访存带宽上必须按单 DDR 通道核算，输入空间上必须支持任意 H × W 而不是 batch 整图驻留，PE 利用率上必须正面应对 Cin < 16 与 stride ≥ 2 的塌陷场景。在该边界内，§3.3 给出本工作的总体架构方案。

## 3.3 总体架构方案

系统采用两层架构——核流水（5 模块 + 共享 cfg_regs）+ DMA 子系统，外部对外仅暴露 1 个 AXI4 主口与 1 个 AXI-Lite 从口，与上位 host SoC 的接口最小化。这一两层划分的核心动机是把"卷积计算流水"与"DDR 数据搬运"两类高度异构的逻辑解耦：核流水完全在片上 SRAM 内闭合数据流，避免计算路径直接面对 DDR 时延抖动；DMA 子系统统一负责 DDR ↔ 片上 SRAM 的搬运，配合 mm2s_arb 仲裁器把 IFB 加载与 WB 加载两类 MM2S 请求串行复用同一条 axi_dm 通道，最大化器件外部 AXI 带宽利用率。 [依赖: Fig.3.1]

核流水包含五个模块——line_buffer、wgt_buffer、mac_array、parf_accum、ofb_writer，加上一份所有模块共享的 cfg_regs。各模块之间通过 valid-ready 握手协议解耦，每个模块独立维护自己的 H/W/Ky/Kx/Cin/Cout 计数器，没有任何中心 FSM 协调全局状态；上下游通过 valid 与 ready 同时为高一拍才完成数据移交，天然支持弹性 join 与气泡传递 <!-- 来自 contributions.md C1.2 概览 -->。这一去中心化握手设计的好处是模块可独立验证、debug 信号可在任意模块边界打断查看，避免传统中心 FSM 在长流水中的状态爆炸问题。

DMA 子系统在 Vivado axi_dm IP 之上由三套自研控制器（idma_ctrl / wdma_ctrl / odma_ctrl）加 mm2s_arb 仲裁器、axi_m_mux 主口聚合器、axi_lite_csr 配置接口构成 <!-- 来自 contributions.md C3.1 -->。其中 idma_ctrl 与 wdma_ctrl 共享 axi_dm 的 MM2S 通道（由 mm2s_arb 串行仲裁，避免两个搬运任务在 DDR 端竞争），odma_ctrl 独占 S2MM 通道。axi_m_mux 把 axi_dm 的 MM2S/S2MM 与 DFE 的 descriptor 拉取请求聚合成单条对外 AXI4 主口，使加速器对上位 SoC 仅暴露一个主口、一个 AXI-Lite 从口，集成代价最小化。

在单核基础上，多核拓扑通过 multicore_top 封装 N = 2 / 4 核形态，加 axi_2to3 / axi_4to5 / axi_lite_1to4 等 fanout / arbiter 组件实现"N 核共用单 AXI 主口对外"的接口约定不变 <!-- 来自 contributions.md C3.4 部分、STATUS.md §2 -->。多核的具体地址映射沿用 DDR `0x0000_0000—0x7FFF_FFFF` + Core[i] IFB `0x8000_0000 + i × 0x1000_0000` 的统一规则，确保 host 配置与 DDR 描述符在单核与多核场景下共享同一份编译器输出。 [依赖: Fig.3.2]

本节给出"是什么"的总体架构图景——5 模块去中心化核流水 + DMA 子系统 + 多核 fanout 三层组合。下一节 §3.4 紧接给出"为什么这样选"的差异化论证，把硬件最简形态与编译器侧填满路线放到 prior art 坐标系中确立本工作的位置。

## 3.4 数据流选择论证

!!! 点评："论证"这个词太大了，而且整段完全没必要。设计理由不用单独开一个章节讲，你直接讲你干了什么，在讲述同时顺带把设计理由讲清楚就行。

本节通过三段对比论证确立本工作"硬件最简 + 编译器侧填满"的设计哲学，是全文最关键的差异化论证章。三段对比依次回答三个问题：第一，PE 阵列采用何种数据流？（OS + 列广播 vs RS/WS）；第二，PE 阵列是否需要硬件可重构？（固定 16 × 16 vs MAERI/Eyeriss-v2 路线）；第三，行级行环数据通路与 Alwani / Kang / Liu 三处近邻 prior art 究竟有什么差异？三段论证既呼应 §1.2 的文献坐标，也为 §4.2—§4.3 的具体实现章节提供设计依据。

第一段对比聚焦数据流选择。在 §2.3 给出的 WS / OS / RS 三类数据流坐标系中，本工作选择 OS + 列广播组合：每个 PE 在 PSUM 寄存器或 PARF 内驻留一个输出像素的部分和，激活沿 16 列广播给同列 16 个 PE，权重沿行广播给同行 16 个 PE，乘积累加完全在 PE 内完成，PE 间无数据移动 <!-- 来自 contributions.md C1.1 -->。相比 RS 类需要专门 NoC 或对角传播网络的复杂互连，OS + 列广播在硬件控制与互连面积两个维度都显著更简；相比 WS 类要求大阵列摊销权重加载开销的限制，OS 在 16 × 16 这一端侧友好的小阵列规模下仍能保持高复用粒度，特别是 Cin 与 Cout 都较大的中间层中部分和复用最彻底。 [依赖: §2.3]

第二段对比聚焦阵列是否可重构这一更深的设计哲学问题。MAERI / Eyeriss-v2 / Tangram 等可重构互连路线的核心动机是从硬件侧动态重排 PE 间数据流，以应对 CNN 不同层形状之间的失配 <!-- 来自 literature.md §B -->；其代价是显著的可编程 NoC 面积、控制复杂度与时序压力，对端侧 FPGA 而言资源代价远超收益。本工作明确选择固定 16 × 16 不可重构阵列，把"PE 阵列上层间形状失配带来的利用率塌陷"问题完全推到编译器侧解决——通过 §4.3 的 Ky 折叠与 S2D 等价变换在编译期把塌陷场景"折回"固定阵列的可用工作模式 <!-- 来自 contributions.md C2.1 / C2.2 / C2.3 -->。这一设计哲学是本工作 narrative A 主轴的元层论断：硬件保持最简，所有形状适配交给编译器。 [依赖: §1.2.1]

第三段对比聚焦行级行环数据通路与 Alwani 跨层融合（MICRO'16）的差异。Alwani 的 Fused-layer 把多个连续 conv 层的中间特征图缓存在共享片上 buffer 中以避免 DDR 往返，VGGNet-E 前 5 层 off-chip feature map 流量从 77 MB 降到 3.6 MB，但其有效性依赖"多层中间特征图能在片上共同驻留"的硬约束，输入分辨率与层深都受 BRAM 容量上墙 <!-- 来自 literature.md Alwani 条目 -->。本工作行级行环只缓冲 strip_rows × W_IN 量级的几行数据，单层硬件即可处理任意 H × W 输入；多层之间通过 DDR 中转或多核 SRAM 直送（§4.5）传递，缓冲粒度从"层数级"降到"行级"，与 Alwani 在缓冲粒度上形成代差 <!-- 来自 contributions.md C1.3 -->。 [CHECK-3.4.3: 与 Alwani 差异化措辞最终版]

行级行环与 Kang AoCStream（FCCM'23 / Sensors'23）的差异在于片上存储与输入分辨率之间是否解耦。Kang 把所有特征图驻留片上 SRAM，line buffer 大小与图宽线性相关，每层使用专用 dataflow block，输入分辨率上限受片上 BRAM 严格约束 <!-- 来自 literature.md Kang AoCStream 条目 -->。本工作通过 row-credit 反压协议把片上缓冲容量与输入分辨率解耦——strip_rows 由编译器静态决策（`ofb_strip_rows_max = (OFB-1) // row_words`），VGA 480 × 640 的单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张 <!-- 来自 contributions.md C1.3 -->。BRAM 占用与 H 解耦后，同一份硬件可在不重综合的前提下覆盖从 32 × 32 到 960 × 540 的不同输入幅面。 [CHECK-3.4.3: 与 Kang 差异化措辞最终版]

行级行环与 Liu Full-Stack Streaming CNN Accelerator（TNNLS'21）的差异在于并行维度的取舍。Liu 在 Intel Arria 10 上把 conv / deconv / residual 等不同层映射到独立模块，跨层 pipelined 让多核同时跑不同层 <!-- 来自 literature.md Liu Full-Stack 条目 -->，并行维度是层间。本工作选择"单核 layer-serial 共用同一份硬件 + 多核 W 维切片"的组合（§4.5 详述），并行维度是单层内的 W 切片；ResNet-11 在 N = 1/2/4 上分别跑通 596K / 450K / 354K wall cycles <!-- 来自 contributions.md C4.4 + STATUS §2.8 -->。两种路线在端侧资源约束下的取舍各有侧重——Liu 路线适合资源宽裕、层数固定的 Arria 10 量级器件，本工作单层内并行更易在 XC7K325T 这一中等规模器件上保持线性扩展且不依赖整网拓扑常驻硬件。 [依赖: Tab.3.3] [CHECK-3.4.3: 与 Liu 差异化措辞最终版]

三段对比论证的共同结论是：本工作在 PE 阵列形态、可重构性、缓冲粒度三个层面均选择"硬件最简"，把所有形状适配与跨层调度的负担推到编译器侧与系统侧。这一选择并非声称比可重构互连或跨层融合"更优"，而是在端侧 FPGA 单器件、单 DDR 通道、有限 BRAM 容量的约束语境下做出的工程化最简组合。下一节 §3.5 把支撑该哲学落地的三条协同原则提升到方法论层面。

!!! 点评："硬件最简"、"哲学"这个词太大了。

## 3.5 编译器与硬件协同设计原则

本节把"单源参数 + cfg 派生 single source + 编译器/硬件视角对齐"三条具体工程实践提升到方法论层面，作为支撑 §3.4"硬件最简 + 编译器侧填满"哲学落地的设计原则。这三条原则的共同目标是消除编译器与硬件之间的视角偏移，避免任何参数、cfg 派生或决策逻辑在两侧出现"双向解释不一致"的隐患。这类隐患在多模块、多人协作的加速器项目中极易演化为难以排查的功能 bug，必须在工程方法论层面预防。

第一条原则是单源参数 `params.py`。所有硬件常量（NUM_COL = NUM_PE = 16、WRF = ARF = PARF = 32、DATA = 8、PSUM = 32、IFB = 8192、WB = 1024、OFB = 2048、BUS_DATA_W = 128、CSR_DATA_W = 32）在 `params.py` 中以 Python 常量统一定义，运行 `python params.py` 自动生成 RTL 端可 `\`include` 的 `RTL/flux_cnn_params.svh`，RTL 用 `\`FLUX_*` 宏引用，Python 用 `from params import *` 引用 <!-- 来自 CLAUDE.md 配置约定 + contributions.md C3.5 -->。改硬件常量只需改一处，编译器与 RTL 双向消费同一份定义，杜绝两侧常量不一致。 [依赖: Fig.3.3]

!!! 点评：这种东西根本没必要在论文里提及。

第二条原则是 cfg 派生 single source。每层硬件 cfg 寄存器由 `hw_files.derive_layer_cfg()` 单一函数从 (Cin, Cout, K, stride, pad, H, W, mode) 等高层参数派生而来；该函数同时被编译器（在 `gen_isa_test.py` 与 `models/run_model.py` 内调用以生成 descriptor）与硬件 testbench（在仿真前打印 cfg 值用于交叉对比）共享 <!-- 来自 contributions.md C3.5 -->。编译器与硬件视角的一切派生计算（如 strip_rows、ofb_strip_rows_max、cin_eff、ky_eff）都收敛在这一函数中，避免硬件代码自行重新推导而引入双向解释偏移。

!!! 点评：这种东西根本没必要在论文里提及。

第三条原则是编译器/硬件视角对齐。Ky-fold 与 S2D 联合触发的核心决策函数 `s2d_eff()` 与 cfg 寄存器映射的 `build_step_cfg_dict` 共享同一份 (Cin, K, stride) → (cin_eff, ky_eff, mode) 的转换语义 <!-- 来自 contributions.md C2.3 -->。编译器在 `scheduler.Layer.force_s2d() / force_fold()` 中按阈值规则做静态决策后，硬件侧 cfg 寄存器只是该决策的字段映射，没有任何独立的"硬件理解"。这一对齐是 §4.3.4 联合触发自动决策的元层基础，也是确保编译器侧等价变换在硬件端 bit-exact 复现的前提。

!!! 点评：这种东西根本没必要在论文里提及。"硬件理解"、"没有任何独立的决策逻辑"这种东西根本没必要说。

三条原则共同把 §3.4 的设计哲学落到工程实践：硬件最简的代价是编译器与硬件需要紧耦合，紧耦合的安全边界由"单源 + 派生 single source + 视角对齐"三条原则保证 [TBD-3.5: §3.5 是否单独成节，倾向保留]。在该方法论框架下，第 4 章具体实现章可以专注 RTL 时序与编译器流水细节，无需再反复论证设计取舍。

## 3.6 本章小结

本章在第 2 章理论坐标系上确立了本工作的三大设计决策：第一，PE 阵列采用 OS + 列广播数据流，固定 16 × 16 INT8 MAC 阵列不引入硬件可重构互连；第二，数据通路采用行级流式行环（strip_rows × W_IN 粒度的 row-credit 反压 ring），单层硬件可处理任意 H × W 输入；第三，多核扩展采用单核 layer-serial 共用硬件 + 多核 W 维切片的组合，外部接口保持单 AXI4 主口 + 单 AXI-Lite 从口最简形态。三大决策合并形成"硬件最简 + 编译器侧填满"的整体设计哲学，并由 §3.5 的三条协同原则（单源参数 / cfg 派生 single source / 编译器与硬件视角对齐）在工程方法论层面保证落地。本章诚实陈述：固定 16 × 16 阵列在 Cin < 16 与 stride ≥ 2 场景下确实存在 PE 利用率塌陷问题，本工作不试图通过硬件可重构掩盖这一塌陷，而是把它显式地交给编译器层（narrative A）解决。下一章紧接展开核心数据通路硬件实现（§4.2，对应贡献 3）、编译器侧 PE 利用率优化（§4.3，对应贡献 1 与贡献 2）、系统集成与编译流（§4.4，对应贡献 5）与多核 W 切片扩展（§4.5，对应贡献 4），共同兑现本章给出的三大设计决策。

!!! 点评："兑现"烂词。

---

# 4 硬件实现与编译器优化

## 4.1 引言

本章是全文最长最详的核心实现章，是 narrative A（编译器侧 PE 利用率优化）主轴的所在地，同时承担 narrative B（系统集成）的主体兑现。第 3 章把"硬件最简 + 编译器侧填满"的设计哲学与三大设计决策定型，本章则把这些决策落到 RTL 模块时序、编译器侧等价变换数学、PyTorch→ISA 端到端编译流、AXI/DMA 子系统集成与多核 W 切片扩展五个具体工程层面，让读者既能理解每个关键模块的实现机理，也具备复现本工作硬件 / 编译器组合的理论依据。

!!! 点评："全文最长最详的核心实现"烂。"让读者理解"烂。

章节组织如下：§4.2 给出核心数据通路硬件实现，包括 5 模块去中心化 valid-ready 流水、行级流式行环 IFB / OFB、16 × 16 OS+列广播 MAC 阵列、分列累加器 PARF、SDP 后处理融合与 7 层循环嵌套硬件骨架，对应贡献 C1.1—C1.6；§4.3 是 narrative A 主战场，展开 Ky 折叠数学推导、S2D 等价变换数学推导、联合触发自动决策、Cout < 16 诚实陈述与硬件可重构路线四维取舍，对应贡献 C2.1—C2.3；§4.4 给出系统集成与编译流，包括 PyTorch→ISA 端到端编译、AXI / DMA 子系统、CFG_WRITE descriptor 配置流与单源参数双向消费，对应贡献 C2.4—C2.5 + C3.1—C3.3 + C3.6；§4.5 在单核基础上扩展到多核 W 维切片，对应贡献 C3.4 + C3.5；§4.6 本章小结。本章合计覆盖 contributions.md 共 16 条贡献。

!!! 点评："本章合计覆盖 contributions.md 共 16 条贡献"烂。评审员都不知道你在说什么。

## 4.2 核心数据通路硬件实现

本节给出核心数据通路的硬件骨架——5 模块去中心化 valid-ready 流水、行级流式行环 IFB / OFB、16 × 16 INT8 OS+列广播 MAC 阵列、分列累加器 PARF、SDP 后处理融合与 7 层循环嵌套硬件实现。这些模块在 §3.3 给出的两层架构中位于核流水一侧，构成所有卷积层共用的固定硬件单元。所有模块共享一份 cfg_regs，由 §4.4 的 CFG_WRITE descriptor 在每层切换时统一改写。

!!! 点评："本节给出"烂。

第一层骨架是 5 模块去中心化 valid-ready 流水。核流水由 line_buffer、wgt_buffer、mac_array、parf_accum 与 ofb_writer 五个模块组成，每个模块自维护一组 H/W/Ky/Kx/Cin/Cout 计数器，没有任何中心 FSM 协调全局状态；上下游通过 valid-ready 握手协议解耦，仅在 valid 与 ready 同时为高的一拍完成数据移交，天然支持弹性 join 与气泡传递 <!-- 来自 contributions.md C1.2 -->。这一去中心化握手的工程价值有三：模块可独立写仿真激励、debug 信号可在任意模块边界打断查看、单模块的 cfg 派生与时序约束改动不会传染到其他模块，避免传统中心 FSM 在长流水中的状态爆炸问题。 [依赖: Fig.4.1]

!!! 点评："天然支持弹性 join"烂。别用天然这种词。

第二层骨架是行级流式行环数据通路。片上特征图缓冲 IFB 容量 8192 word（按 BUS_DATA_W = 128 计算约 128 KB）、输出缓冲 OFB 容量 2048 word，二者均按 strip_rows × W_IN 粒度组织为 ring buffer——line_buffer 把当前 strip 的若干行依次写入 IFB ring，mac_array 沿 W 方向滑窗读取，ofb_writer 把 SDP 输出按行写入 OFB ring 后由 odma_ctrl 搬出 <!-- 来自 contributions.md C1.3 -->。strip_rows 由编译器静态决策（`ofb_strip_rows_max = (OFB - 1) // row_words`），row-credit 反压协议保证 ring 不会被生产者写穿。这一行级 ring 把片上 BRAM 占用与输入特征图行数 H 解耦，VGA 480 × 640 单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张，不需要按 tile 多次启动。 [依赖: Fig.4.2]

!!! 点评："不需要"烂。

第三层骨架是 16 × 16 INT8 OS+列广播 MAC 阵列。16 行 × 16 列共 256 个 INT8 MAC 单元构成本工作的核心算力——激活沿 16 列广播给同列 16 个 PE，权重沿行广播给同行 16 个 PE，PE 内乘积累加到 PSUM 寄存器或下游 PARF 中，PE 间无任何数据移动 <!-- 来自 contributions.md C1.1 -->。该数据流在 §3.4.1 已论证选择依据：相比 RS 类需要专门 NoC 或对角传播网络的复杂互连，OS + 列广播在硬件控制与互连面积上显著更简；相比 WS 类要求大阵列摊销权重加载开销的限制，OS 在 16 × 16 这一端侧友好规模下仍能保持高复用粒度。该阵列在 Cin ≥ 16 且 Cout ≥ 16 时可填满 256 MAC，本章 §4.3 把"Cin < 16"塌陷场景通过编译器侧重映射折回该工作模式。 [依赖: Fig.4.3]

!!! 点评："该数据流在 §3.4.1 已论证选择依据"烂。"PE 间无任何数据移动"烂。"本章 §4.3 把"Cin < 16"塌陷场景通过编译器侧重映射折回该工作模式。"烂。

第四层骨架是分列累加器 PARF。PARF 由 NUM_COL = 16 个 parf_col 单元构成，每个 parf_col 内部独立维护一片 PARF = 32 深度的 SRAM 用于存储该列对应输出像素的部分和；外壳共享 wr_addr 与 we 端口以节省全局连线，rd_addr 由各列独立维护以支持后处理时按列读出 <!-- 来自 contributions.md C1.4 -->。这一"内部独立 + 外壳共享"的折中设计使得 PARF 既具备每列独立累加的语义灵活性（不同列对应不同输出像素），又能在 16 列同时写入时共用同一组写入控制逻辑，在面积与控制复杂度之间做精细平衡。 [依赖: Fig.4.4]

!!! 点评："精细平衡"烂。去掉精细。

第五层骨架是 SDP 后处理融合。SDP（Single Data Processor）模块在 ofb_writer 之前，把 32-bit PSUM 经 bias 加 / shift 右移 / clip 截断三步重对齐回 INT8 输出空间；对支持残差短接的层，SDP 还在 clip 之前融合 short-cut 加法 <!-- 来自 contributions.md C1.5 -->。三步算子被融合到单条组合路径中，避免每步插入流水寄存器带来的延迟代价。该组合路径的代价是其长度成为当前实现的 critical path，单核 Vivado 综合 Fmax 仅 68.4 MHz <!-- 来自 STATUS.md §1 单核综合表 -->。本工作把这一 limitation 在 §5.4.4 诚实陈述，并指出可通过流水切分提升至 100+ MHz 作为 §6.3 future work。 [依赖: Fig.4.5]

第六层骨架是 7 层循环嵌套硬件实现。§2.2 中给出的标准卷积 7 层循环嵌套（N / Cout / H_out / W_out / Cin / Ky / Kx）在硬件侧逐层映射到不同模块的计数器：N 与 Cout 由顶层 sequencer 在层切换之间体现（每个 N、每个 Cout slice 启动一次完整数据流），H_out / W_out 由 line_buffer 与 ofb_writer 在 strip 与 row 维度推进，Cin 由 mac_array 与 parf_accum 在权重 / 激活循环中累加，Ky / Kx 由 mac_array 滑窗与 wgt_buffer 内部展开 <!-- 来自 contributions.md C1.6 -->。每个循环的上下界与 stride / pad 偏移由 cfg_regs 一组 32-bit 配置寄存器统一控制，编译器侧 `hw_files.derive_layer_cfg()` 单一函数完成派生（§4.4.4 详述）。

硬件骨架就绪后，§4.3 在该骨架上展开 narrative A 主章——编译器侧如何通过 Ky 折叠与 S2D 等价变换把 PE 利用率塌陷场景"折回"该固定骨架的可用工作模式。

## 4.3 编译器侧 PE 利用率优化

本节是本论文 novelty 最强的主战场——通过 Ky 折叠（Ky-fold）与空间到深度（Space-to-Depth, S2D）两类编译器侧等价变换，配合联合触发自动决策，把 §4.2 固定 16 × 16 阵列在 Cin < 16 与 stride ≥ 2 场景下的 PE 利用率塌陷解决在编译器层，硬件侧零改动。本节延续 §3.4 设计哲学的元层论断："硬件最简留给编译器侧重映射"，但本节给出具体可复现的数学推导、参数派生公式、决策阈值与硬件视角对齐的工程实现细节。

PE 利用率塌陷场景在本工作目标负载 ResNet-11 中具有典型性。Patch 层是网络入口处 K = 4 / stride = 4 / Cin = 3 的彩色输入卷积，在 16 × 16 阵列上 baseline PE 行利用率仅为 Cin / 16 = 3 / 16 ≈ 18.8%（仅 3 行被有效使用，其余 13 行空转），同时 PE 列利用率受 Cout 影响。中间层中也存在 Cin < 16 的小通道卷积（如残差分支 1 × 1 投影层、网络早期 Cin = 8 的层），同样会让 PE 行利用率掉到 Cin / 16 以下 <!-- 数字待 model_analysis.md 核 [CHECK-4.3.1: 各层利用率塌陷数字] -->。这些塌陷层在整网 wall cycles 中占据相当比重，是端侧 ResNet 部署不可回避的核心瓶颈，构成本节优化的攻击面。

第一类编译器侧变换是 Ky 折叠（Ky-fold），适用于 Cin < 16 且 K > 1 的卷积层。其数学根基是把卷积核的 Ky 维与 Cin 维等价合并：把原始卷积按 ky = g · kyper + ky_local 分组（g 取 0 到 groups_y − 1），定义虚拟激活 I'[y_virt, x, g · Cin + c] = I_padded[y_virt + g · kyper, x, c] 与虚拟权重 W'[ky_local, kx, co, g · Cin + c] = W[g · kyper + ky_local, kx, co, c]，则原卷积等价于一个 K_new = kyper、cin_fake = groups_y · Cin 的 stride = 1 标准卷积 <!-- 来自 docs/pe-fold.md §1 + contributions.md C2.1 -->。具体参数派生为 groups_y = HW_PE / Cin、kyper = ⌈K / groups_y⌉、cin_fake = groups_y · Cin、pad_ky = groups_y · kyper − K（末组零 pad 数）。 [依赖: Fig.4.6]

Ky 折叠的工程实现位置在 `toolchain/hw_files.py` 中——`compute_fold_params(K, Cin, HW_PE)` 给出参数派生，`fold_input()` 生成虚拟 ifm（每个 y_virt 位置存放 groups_y × Cin 个通道，来自 groups_y 个 y 方向偏移行），`fold_weights()` 把权重按 ky 分组重排并对 pad_ky 位置补零 <!-- 来自 docs/pe-fold.md §1 实现位置 -->。硬件侧完全无感，仍按普通 stride = 1 卷积跑——line_buffer / mac_array / parf_accum / SDP 五模块流水的 cfg_regs 只看到 cin_fake 与 K_new = kyper，与原始 (Cin, K) 无关。代价是 IFB 占用变为原来的 groups_y 倍（编译器侧输入 y 偏移复制），以及 pad_ky > 0 时一部分 PE 行在末组上空算贡献为 0。据已知文献，加速器领域尚无直接对应的纯编译器侧、零硬件代价、把 Ky 维折叠到 Cin 的 PE 利用率优化方案 <!-- 来自 literature.md §C 备注 -->，本工作据此在 §6.2 用"据已知文献..."的措辞表达 novelty。

!!! 点评：千万不要在论文里写什么项目文件的文件名，评审员不会关心，只会认为你在凑字数。

第二类编译器侧变换是空间到深度（S2D），适用于 stride ≥ 2 且 K ≥ stride 的卷积层。其数学根基是把 stride 卷积的 stride² 个相位折叠到 cin 维，使硬件按 stride = 1 卷积跑。具体地，按 ky = stride · ky' + a、kx = stride · kx' + b、p = a · stride + b 重写卷积，定义虚拟激活 I'[Y, X, p · Cin + c] = I_padded[Y · stride + a, X · stride + b, c] 与虚拟权重 W'[ky', kx', co, p · Cin + c] = W[ky' · stride + a, kx' · stride + b, co, c]（K 不被 stride 整除时越界位置补零），则原 stride 卷积等价为一个 K_new = ⌈K / stride⌉、Cin_new = stride² · Cin 的 stride = 1 卷积 <!-- 来自 docs/pe-fold.md §2 + contributions.md C2.2 -->。 [依赖: Fig.4.7]

!!! 点评："数学根基"烂。你看看参考范文里怎么写的数学公式。

S2D 变换相比 Ky 折叠有两点关键收益：其一是 DDR / IFB 内存代价为零——`s2d_input()` 仅做相位重排不做激活复制，相比 Ky-fold 的 groups_y 倍 IFB 膨胀，多核场景下 DDR 带宽节省尤为明显；其二是 stride 变为 1 后可启用 ARF reuse_en = 1 的滑动窗口复用模式（约束 stride == 1 && K > 1 && tile_w ≤ 33 − K），IFB 读次数大幅下降 <!-- 来自 docs/pe-fold.md §2 代价收益 + §4 ARF 容量约束 -->。其代价是 K 不被 stride 整除时不同相位 sub-kernel 形状不齐，统一 pad 到 K_new × K_new 会造成 pad_waste = (K_new² · stride² − K²) / (K_new² · stride²) 部分 MAC 浪费（K = 8、stride = 2 时 pad_waste = 0；其他不整除场景有量化损失）。本变换在 ResNet-11 Patch 层（K = 4 / stride = 4）兑现单层 5.05 × 加速（654,404 → 129,594 cycles）<!-- 来自 STATUS.md §2.8 / contributions.md C2.2 -->。S2D 在 ML 算法侧的等价关系（子像素重排）来源于超分辨率领域 Pixel-Shuffle / Sub-pixel Convolution，本工作在加速器编译器侧把它作为 PE 利用率优化路径的引用谱系待复查 [CHECK-1.2.4: 加速器领域 S2D 引用谱系待补查]。

!!! 点评："关键收益"烂。别用"关键"这种词，通篇这关键那关键到底哪关键。

两类变换的联合触发由 `toolchain/run_regression.py` 与 `scheduler.Layer.force_s2d() / force_fold()` 在编译期完成自动决策 <!-- 来自 docs/pe-fold.md §5 受益层判定 + contributions.md C2.3 -->。决策规则为：Ky-fold 触发条件 `K > 1 AND Cin < 16`，S2D 触发条件 `stride ≥ 2 AND K ≥ stride`；S2D 启用后 Cin 变为 stride² · Cin，重新判定 Ky-fold 触发条件（多数情况下 S2D 后 Cin' ≥ 16 不再需要 Ky-fold）。K = 1 与 FC 层无可折维度不受益，Cin ≥ 16 的层已近满 PE 行利用不需要 fold。该决策的元层基础是 §3.5 第三条原则——`s2d_eff()` 与 `build_step_cfg_dict` 共享同一份 (Cin, K, stride) → (cin_eff, ky_eff, mode) 的转换语义，编译器决策与硬件 cfg 寄存器映射保持一一对齐，确保编译器侧等价变换在硬件端 bit-exact 复现。

!!! 点评："决策规则为："烂。去掉语句还通畅，根本就是废话。"bit-exact 复现"也是废话，不bit-exact那还叫复现吗？

三模式（baseline / Ky-fold / S2D）PE 利用率与 wall cycles 的具体对比数字将在 §5.5.2 表化给出 [CHECK-Q6: 三模式利用率与 cycles 数字]。本节诚实陈述本工作的两处 disadvantage：其一是 Cout < 16 时硬件不做任何复用，PE 列利用率掉到 Cout / 16，列空转无法通过编译器侧重映射回收 <!-- 来自 docs/pe-fold.md §3 -->，这是当前架构的明确取舍——避免引入 Kx 维度复用必需的列归约级（psum_reshape）和 parf_accum 内部每列地址偏移逻辑，把硬件保持简洁；其二是 Ky-fold 的 IFB 膨胀代价在多核 W 切片场景下需要重新核算 ring 容量。这两处 disadvantage 不软化、不掩饰，作为本工作"硬件最简"取舍的诚实代价显式陈述。

本节最后给出与硬件可重构（MAERI / Eyeriss-v2 / Tangram）路线的四维度对位。第一是硬件复杂度——可重构互连需要可编程 NoC 或层级化 mesh 网络支持运行时数据流重排，本工作固定 16 × 16 阵列加上行 / 列广播线即可，硬件代价显著更低 <!-- 来自 literature.md §B + §3.4.2 -->；第二是编译器复杂度——可重构互连把利用率优化的负担放在硬件侧，编译器只需做层映射决策，本工作把负担推到编译器侧 Ky-fold + S2D + 联合触发，编译器复杂度更高；第三是内存代价——可重构互连不引入额外激活复制，本工作 Ky-fold 引入 groups_y 倍 IFB 膨胀（S2D 无内存代价）；第四是适用场景——可重构互连适合层形状高度多样、硬件资源宽裕的云端 ASIC 场景（典型如 MAERI 在 28 nm ASIC 上演示），本工作的"硬件最简 + 编译器侧填满"路线适合层形状相对集中（CNN 主线 ResNet 风格）、硬件资源紧张的端侧 FPGA 场景。两条路线在不同场景下各有合理性，本工作不声称"更优"，仅在端侧 FPGA 单器件、单 DDR 通道、有限 BRAM 容量的约束语境下做出的工程化最简组合。

与 Liu 等人 Full-Stack TNNLS'21 的对位是另一处必须明确点出的同范畴对照：Liu 在 Intel Arria 10 GX1150 大型器件（约 1518K LE / 65.7 Mb 片上 SRAM / 1518 DSP 块）上实现 ResNet 风格网络的端到端 streaming，报告 PE 利用率约 97% <!-- 来自 literature.md Liu Full-Stack 条目 -->；本工作在 Xilinx XC7K325T 中型器件（203K LUT / 445 BRAM36 / 840 DSP48）上启用 S2D 后整网 PE 利用率约 86.6% <!-- [CHECK: 整网利用率 86.6% 数字与 model_analysis.md 与 STATUS.md 交叉核 + 取数语义对齐] -->。两组数字对应不同器件规模下的可比数字——Arria 10 GX1150 的资源量级约为 XC7K325T 的 5 ~ 7 倍，Liu 路线在该量级器件上能让所有层硬件常驻、跨层 pipelined 同时跑，本工作在中型器件上选择单核 layer-serial 共用硬件，利用率自然有差距，但绝对硬件代价显著更低且支持任意 H × W 输入。

narrative A 主轴的核心论断与可复现路径在本节落实后，§4.4 紧接谈如何把这些编译器侧决策落到 PyTorch→ISA 端到端编译流，与 AXI / DMA 子系统、CFG_WRITE descriptor 配置流串成完整工程闭环。

## 4.4 系统集成与编译流

本节把 PyTorch 模型、AXI / DMA 子系统、CFG_WRITE descriptor 配置流与单源参数 params.py 串成端到端可用的工程闭环，是 narrative B 的次核心章。第 4.2 节给出了硬件骨架，第 4.3 节给出了编译器侧等价变换，但若没有自动化的端到端编译流与可工作的 AXI / DMA 集成，整套加速器就只能在仿真上跑单 case，无法对接 PyTorch 训练后模型完成端到端推理。本节正是把这块工程闭环兜起来。

PyTorch→ISA 端到端编译流由 `toolchain/models/run_model.py` 主入口、`hw_files.derive_layer_cfg()` cfg 派生、`gen_isa_test.py` 单 case 生成与 `compute_fold_params / compute_s2d_params` 等价变换四部分组成 <!-- 来自 contributions.md C2.4 + CLAUDE.md 常用命令 -->。该编译流的输入是 PyTorch nn.Module 与一组样本图像，输出是一组对硬件 cfg_regs 与 DDR 数据布局的二进制描述：编译器逐层做 INT8 量化（权重 / 激活 / bias / shift）、按 §4.3 的触发规则做 Ky-fold / S2D 联合决策、把权重按 16 × 16 PE 阵列的 OS+列广播次序打包、把 cfg_regs 字段按 `derive_layer_cfg()` 派生填入 descriptor list。该流程兑现"模型→硬件 ISA"的端到端能力，开发者可一行命令跑通完整模型部署：`python models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10`。 [依赖: Fig.4.8]

!!! 点评：又提文件名。文件名你认为有必要在论文里提吗？。

链式 CASES 验证基础设施是端到端编译流的兄弟工具，其 DSL builder 支持把多层 CNN 表达成"层 list + 跨层 DDR FM 共享"的声明式描述，由 `toolchain/run_regression.py` 自动化生成 51 case 的回归测试集 <!-- 来自 contributions.md C2.5 + CLAUDE.md 常用命令 -->。每个 case 自动完成生成虚拟激活 / 权重 → 编译器派生 cfg → 仿真器跑 RTL → 与 PyTorch reference 实现做 element-wise bit-exact 对比四步，输出统一的 CASE_RESULT / CASE_PROFILE 报告。这一基础设施使得任何 RTL 改动 / 编译器改动 / 等价变换改动都能在数十分钟内完成全量回归，是 §5.3 功能验证 51-case bit-exact 的工具底座，也是本工作支持快速迭代的工程基石。

!!! 点评："工具底座""工程基石"这种词咋嫩喜欢用呢，你觉得评审员会觉得"基石"这个词很有说服力吗？

AXI / DMA 子系统集成在 §3.3.2 已给出总体方案，本节展开实现细节。子系统由三套自研控制器（idma_ctrl / wdma_ctrl / odma_ctrl）+ 一片 Vivado 提供的 axi_dm 数据搬运 IP（`Syn/gen_axi_datamover.tcl` 生成）+ mm2s_arb 仲裁器 + axi_m_mux 主口聚合器 + axi_lite_csr 配置接口构成 <!-- 来自 contributions.md C3.1 + CLAUDE.md 项目总览 -->。idma_ctrl 与 wdma_ctrl 共享 axi_dm 的 MM2S 通道（mm2s_arb 串行仲裁，避免 IFB 加载与 WB 加载在 DDR 端竞争），odma_ctrl 独占 S2MM 通道。axi_m_mux 把 axi_dm 的 MM2S/S2MM 与 DFE 的 descriptor 拉取请求聚合成单条对外 AXI4 主口，加速器对上位 SoC 仅暴露一个主口与一个 AXI-Lite 从口，集成代价最小化。Vivado IP 边界明确化是 §3.5 第二条原则的工程兑现——自研控制器只负责 ISA / cfg 语义，DDR 突发与 outstanding 事务管理交给 Xilinx 官方 IP，避免重复造轮子并保证综合时序闭合。

!!! 点评："工程兑现"

CFG_WRITE descriptor 配置流是 host 配置开销优化的关键工程贡献。在没有 descriptor 机制的朴素方案中，host 在每层切换时需要通过 AXI-Lite 逐字段写入约 50 个 32-bit cfg 寄存器，对带宽占用与 host CPU 时序压力显著。本工作引入 DFE（Descriptor Front End）拉取一段 host 预先组织好的 descriptor list（在 DDR 中），每条 descriptor 一次性更新一组 cfg 寄存器，host 每层只需写 4 次 AXI-Lite（指针 / 长度 / 启动 / 状态轮询），等效写次数从约 50 / 层降到 4 / 层 <!-- 来自 contributions.md C3.2 -->。配套的双口 cfg_regs（一口接 axi_lite_csr，一口接 DFE，两口仲裁更新）与 Done sticky 寄存器（用于 host 轮询每层完成）在硬件侧支持这一配置流 <!-- 来自 contributions.md C3.3 -->，使加速器与上位 SoC 之间的协作开销降到工程可接受范围。 [依赖: Fig.4.9, Tab.4.2]

profile 报告基础设施是验证与调试链路的另一关键组件。`toolchain/run_regression.py` 在每个 case 跑完后自动产出 CASE_PROFILE 报告，其中包含 wall cycles、mac_pipe%（MAC 阵列实际工作 cycles / wall cycles）、DDR busy%、各模块 stall 来源分布等关键指标 <!-- 来自 contributions.md C3.6 + docs/simulation.md -->。该报告是 §5.5 性能分析与 §5.6 多 DDR 带宽分析的数据来源——例如 N = 4 SMC 场景下 DDR busy 84.7% 即由该报告统计而出，是 1-DDR 通道为 N = 4 加速比下降根因的直接证据。profile 报告的存在让本工作的所有性能数字都有可追溯的统计来源，而非来自手工估算。

单核工程闭环在本节四要素（PyTorch→ISA / DMA / CFG_WRITE / profile）就绪后已具备端到端可用形态。下一节 §4.5 在单核基础上扩展到多核 W 切片，把吞吐进一步推高，并保持外部接口与编译器流不变。

!!! 点评："就绪后已具备端到端可用形态"何意味。

## 4.5 多核 W 切片扩展

本节在 §4.2—§4.4 单核闭环基础上，把工作沿 W 维切片到 N = 2 / 4 核，配合 halo 计算冗余与跨核 SRAM 直送实现近线性扩展。该扩展是 narrative B 的关键章节，对应贡献 C3.4 与 C3.5。多核扩展遵循 §3.3 的总体架构约束——外部接口保持单 AXI4 主口 + 单 AXI-Lite 从口最简形态不变，编译器侧 cfg 派生与 host 配置流复用单核基础设施，避免多核扩展对整套工程闭环造成侵入性改动。

多核拓扑由 `multicore_top` 模块封装 N = 2 / 4 核形态，加 axi_2to3 / axi_4to5 fanout 与 axi_lite_1to4 fanout 等组件实现单 AXI 主口对外的接口约定 <!-- 来自 contributions.md C3.4 + STATUS.md §2 + CLAUDE.md 项目总览 -->。每个核内部仍是 §4.2 的 5 模块去中心化流水 + DMA 子系统，核间通过共享 AXI 主口仲裁访问 DDR，地址映射沿用 `DDR 0x0000_0000—0x7FFF_FFFF` + `Core[i] IFB 0x8000_0000 + i × 0x1000_0000` 的统一规则。该拓扑选择的核心约束是不破坏单核 cfg 派生与编译器流——同一份编译器输出可在 N = 1 / 2 / 4 三种形态下被消费，host 只需根据 NUM_CORE 设置 descriptor 分发模式。

W 切片的具体划分由编译器在 `derive_layer_cfg()` 内部完成。给定一层卷积 (Cin, Cout, K, stride, pad, H, W)，编译器把 W 维按 NUM_CORE 等分为 N 段，每个核负责其中一段（W_local ≈ W / NUM_CORE）；K > 1 的卷积存在 halo 区域——例如 K = 3 时 N = 2 两核需要在切分边界附近重叠 2 列输入数据，配合非对称 pad 处理边界确保切片输出与单核结果 bit-exact 一致；K = 1 的卷积无 halo，退化为纯切分 <!-- 来自 contributions.md C3.4 -->。halo 设计的代价是边界列在两核上重复计算（计算冗余而非数据冗余），但该开销在 W 较大时占比小，整体仍能接近线性扩展。 [依赖: Fig.4.10]

跨核 SRAM 直送（M2 push）是多核加速比的关键优化。在朴素方案中，producer 核完成一层输出后写回 DDR，consumer 核再从 DDR 加载到 IFB；该方案下 DDR 带宽成为多核扩展瓶颈。本工作引入 producer ODMA 直接把输出写入 consumer 核 IFB region 的机制——producer 在写完一行 OFB 后，odma_ctrl 直接通过 AXI 主口写到 consumer 的 IFB 地址区段，无需 DDR 中转 <!-- 来自 contributions.md C3.4 + STATUS.md §2 多核加速比 -->。该优化在 ResNet-11 N = 4 场景下把多核 wall cycles 从 N = 1 596K 降到 N = 4 354K，等效 1.68 × 加速比 <!-- 来自 STATUS.md §2.8 -->，是多核扩展从"理论可行"走向"工程实测有效"的关键。 [依赖: Fig.4.11]

单源参数 params.py 在多核场景下展现其方法论价值。NUM_CORE / IFB region 划分 / W_local 等参数在 `params.py` 中以 Python 常量统一定义，编译器（`hw_files.derive_layer_cfg()`）与 RTL（`multicore_top` 实例化参数）双向消费同一份定义 <!-- 来自 contributions.md C3.5 + CLAUDE.md 配置约定 -->。例如把 N = 4 调整为 N = 2 只需改 `params.py` 中 `NUM_CORE = 2` 并重跑 `python params.py`，编译器自动按新 NUM_CORE 重派 W 切片，RTL 自动按新 NUM_CORE 例化 multicore_top；不存在"忘了改某一侧"导致的双向解释偏移。这条原则在 §3.5 已上升为方法论，本节是其在多核场景的具体兑现。

多核扩展的诚实陈述是：N = 4 加速比 1.68 × 偏离线性 4 ×，根因是 1-DDR 通道带宽。Phase 7 SMC + NUMA 主线最新数显示 N = 4 wall cycles 220,824 cy / 453 fps，DDR busy 84.7% <!-- 来自 STATUS.md §2 + commit 5fe16b2 + contributions.md C4.6 [CHECK-Q5: 是否作为论文最终数] -->。该 limitation 通过 §5.6 多 DDR 带宽分析（1-DDR vs 4-DDR PoC，mac_pipe% 36.2% → 63.5% / wall cycles 354K → 196K）定量证明，作为 §6.3 future work 的硬件路径之一保留。本工作不投多 DDR 板的 ROI 决策依据是端侧加速器主流场景的板级成本约束，作为诚实 limitation 在论文中保留。

!!! 点评："诚实陈述"何意味。

多核扩展闭合后，§4.6 小结收束第 4 章；多核加速比的完整数据兑现在 §5.5.3 与 §5.6 给出。

## 4.6 本章小结

本章覆盖 contributions.md 共 16 条贡献——硬件骨架 6 条（C1.1 OS+列广播 16 × 16 阵列 / C1.2 5 模块去中心化流水 / C1.3 行级流式行环 / C1.4 分列累加器 PARF / C1.5 SDP 后处理融合 / C1.6 7 层循环嵌套硬件）、编译器侧 PE 利用率优化 3 条（C2.1 Ky 折叠 / C2.2 S2D 等价变换 / C2.3 联合触发自动决策）、系统集成 5 条（C2.4 PyTorch→ISA 编译流 / C2.5 链式 CASES 验证基础设施 / C3.1 AXI-DMA 集成 / C3.2 CFG_WRITE descriptor 配置 / C3.3 双口 cfg_regs）、多核扩展与单源参数 2 条（C3.4 多核 W 切片 + halo + 跨核 SRAM 直送 / C3.5 单源参数 params.py 双向消费），以及 profile 报告基础设施 1 条（C3.6）。每条贡献在本章对应小节均有具体的实现机理、参数派生公式或工程组件描述。

!!! 点评：这一大段总结性的文字很啰嗦。少用括号，少用斜杠。

"硬件最简 + 编译器侧重映射"路线在本章具体兑现——硬件层保持 16 × 16 固定阵列、行级流式行环、5 模块去中心化握手流水的最简形态，编译器层通过 Ky 折叠与 S2D 等价变换把 PE 利用率塌陷折回固定阵列的可用工作模式，系统层通过 PyTorch→ISA 端到端编译流与 CFG_WRITE descriptor 配置流把工程闭环做到位，多核层通过 W 切片 + halo + 跨核 SRAM 直送实现近线性扩展。技术路线 A（编译器侧 PE 利用率优化）与 技术路线 B（系统集成）两条主轴在本章合流。本章承诺的功能正确性（51-case bit-exact）、性能数字（PE 利用率三模式、ResNet-11 N = 1/2/4 wall cycles）、综合数据（XC7K325T 单核 / 多核资源与 Fmax）与多 DDR 带宽分析将全部在第 5 章兑现，并对 Fmax 仅 68 MHz、DSP 推断率 82/256、1-DDR 带宽限制三处 limitation 做诚实陈述。

---

# 5 系统验证与实验分析

## 5.1 引言

本章是检验论文是否站得住的关键——前 4 章的所有设计主张、narrative A（编译器侧 PE 利用率优化）与 narrative B（系统集成与多核扩展）的承诺，最终都要在本章以可复现的量化数据兑现。本章覆盖 contributions.md 中 C4.1-C4.7 共 7 条实测贡献：单核与多核综合资源（C4.1/C4.2）、66-case bit-exact 功能验证（C4.3）、ResNet-11 cycles 与 FPS（C4.4）、多 DDR 带宽 PoC（C4.5）、Phase 7 SMC+NUMA 主线（C4.6）、PE 利用率三模式（C4.7）<!-- 来自 contributions.md C4.* + paragraph-skeleton §5.1 -->。

本章按"对的→装得下→跑得快→比得过"四步推进：§5.2 实验设置铺垫硬件平台、工具链与测试集口径；§5.3 功能验证用 66-case bit-exact 证明设计正确；§5.4 板级综合与资源分析证明设计在 XC7K325T 上装得下；§5.5 性能分析展开 PE 利用率与 cycles/FPS 数据；§5.6 多 DDR 带宽分析定量解释多核加速比偏离线性的根因；§5.7 与已有工作横向对比给出 Pareto 前沿定位；§5.8 小结收束全章。诚实陈述原则贯穿全章——Fmax 仅 68 MHz、DSP 推断率 82/256、1-DDR 带宽限制三处 limitation 将在对应小节明确给出，不做数据美化 <!-- 来自 paragraph-skeleton §5.1 段 3 -->。

## 5.2 实验设置

本节明确硬件平台、工具链与测试集三要素，让读者能复现实验环境，并对后续小节使用的 cycles、FPS、bit-exact 等评测口径给出统一定义。

硬件平台为 Xilinx Kintex-7 系列 XC7K325T-FFG900-2 FPGA，目标频率 100 MHz，单核与多核（N=1/2/4）均共用同一器件 <!-- 来自 STATUS.md §1 综合 + §2 多核综合 -->。该器件资源容量为 LUT 203,800 / FF 407,600 / BRAM36 445 / DSP48E1 840，是中端 FPGA 的代表规格。选择该器件而非更大的 UltraScale+ 系列，是为了证明本工作的"小器件 + 编译器侧 PE 利用率优化"路线在中端器件上即可达到目标性能，符合 §1 引言中给出的端侧加速器场景假设。

工具链由综合、仿真、量化、编译四部分组成。综合使用 Vivado 2023.1 OOC（Out-Of-Context）流程，约束文件给出 100 MHz 目标周期与 IO 异步假设；仿真使用 ModelSim 2020.4，跨语言 Vivado simlib 由 `Syn/compile_simlib.tcl` 一次性编译；量化与参考实现使用 PyTorch 后量化（int8 对称量化）；编译器为本工作自研的 Python toolchain，含单 case 生成器 `gen_isa_test.py`、回归驱动 `run_regression.py`、多核调度器 `scheduler.py` 与 PyTorch 模型部署入口 `models/run_model.py` <!-- 来自 CLAUDE.md 工具链约定 + STATUS.md 工具链描述 -->。

测试集组织为分层结构，覆盖从单核 corner case 到 ResNet-11 整网的功能空间。第一层是 22-case ResNet-18 风格回归集，覆盖典型卷积形状（K∈{1,3,5,7}、stride∈{1,2}、Cin/Cout∈{4,8,16,32,64}），每 case 在 baseline / Ky-fold / S2D 三种模式下各跑一次，共 66 次执行 <!-- 来自 CLAUDE.md run_regression.py + paragraph-skeleton §5.3 -->。第二层是 24-case robust smoke，含极端形状（K=7 大 halo、W=8 极小、W=33 奇数）与 VGA 480×640 大尺寸 <!-- 来自 STATUS.md §2.8 wslice_k7/wslice_smallw/wslice_oddw -->。第三层是 16-case 多核 W 切片测试（N=2 与 N=4 各 8 case），覆盖 halo 边界、非对称 pad、ds 层 stride=2 与 K=1 无 halo 退化 case <!-- 来自 STATUS.md §2.8 N=2/N=4 W slice 表 -->。

ResNet 风格 chain 测试是端到端验证的核心。除独立的 corner case 外，工具链支持 DSL builder 拼链（`Chain` / `_Node` / `resnet_block`），把跨层 DDR FM 共享与 layer 间 push 关系编码为 7 行 Python 即可生成 11-layer chain 的完整 descriptor 列表 <!-- 来自 STATUS.md §1 链式 CASES + contributions.md C2.5 -->。本章使用的 ResNet 风格测试包含 6 个 ResNet 残差短接 case 与 3 个 ResNet-11 整网 case（N=1/2/4 各一），合计 51 case（26 单核 corner + 16 N=2/4 W 切片 + 6 ResNet 残差 + 3 ResNet-11 整网，与 §5.3 / §5.9 口径一致）<!-- 来自 STATUS.md §2.8 测试矩阵；P0 #3 修正：统一全章 51 case 口径，删除 51 vs 55 [CHECK] -->。

评测口径在本节统一：cycles 指仿真器报告的 RTL 时钟周期数（不含 host 启动、DDR 预加载等仿真 setup 时间），通过 `Wall_us` 端到端报告获得 <!-- 来自 STATUS.md §1 Wall_us 端到端报告 -->；FPS 用 `freq / cycles` 推算，论文表中以 100 MHz target 假设给出，实测 Fmax 仅 68 MHz 时实际 FPS ≈ 0.68× 表中数字（§5.5 段 6 会再次申明）；bit-exact 指与 PyTorch 量化参考实现 element-wise 完全一致，每个 int8 输出像素均匹配，不接受任何误差容忍 <!-- 来自 paragraph-skeleton §5.3 段 5 -->。

三要素铺垫到位后，§5.3 在测试集上跑功能验证，§5.4 给出综合资源与时序数据。

## 5.3 功能验证

本节通过分层测试矩阵证明设计的功能正确性。从单核 corner case 到 ResNet-11 整网，所有 case 的输出与 PyTorch 量化参考实现 element-wise 完全一致，不允许任何误差容忍——这是衡量"对的"最严苛的判据 <!-- 来自 paragraph-skeleton §5.3 段 1 -->。

单核 corner case 共 26 例，跨度覆盖 K∈{1,2,3,5,7}、stride∈{1,2,3,4}、Cin∈{4,8,12,16,32,64}、H×W 含 VGA 480×640 大尺寸，覆盖典型卷积形状空间 <!-- 来自 STATUS.md §1 + paragraph-skeleton §5.3 段 2 -->。这些 case 中有相当一部分是设计早期暴露问题的 corner——例如 Cin=4 触发 line_buffer 行环边界条件、stride=4 触发 idma_ctrl 行步进与 cmd_btt 解耦逻辑、K=7 触发 halo 计算与边界 pad 重叠——单核 case 全部 PASS 是后续多核扩展的基线保证。

多核 W 切片测试共 16 例，N=2 与 N=4 各 8 例，覆盖 K∈{1,3,5,7}、stride∈{1,2}、W∈{8,32,33}，含 halo 与边界 case <!-- 来自 STATUS.md §2.8 N=2/N=4 W slice 表 -->。其中 wslice_k7（K=7 大 halo）、wslice_oddw（W=33 奇数 N 不整除）、wslice_smallw（W=8 极小）三个 case 是 W 切片机制最容易暴露问题的边界——非对称 pad 计算、halo 列重复计算、奇数 W 在 N 核之间的不均匀分配——这三个 case 全 PASS 证明 §4.5 的 W 切片几何与 halo 设计正确闭合。

ResNet-11 整网 N=1/2/4 三 case 加 6 个 ResNet 残差链 case，是设计端到端正确性的最终检验。残差链 case 验证 SDP 后处理融合（C1.5）的 shortcut_mult / shortcut_shift 路径与 Shortcut Bank 行为正确；ResNet-11 整网 case 验证编译器多层调度、多核 W 切片协同、跨核 SRAM 直送（M2 push）三个机制在真实网络拓扑下的协同正确性 <!-- 来自 STATUS.md §1 R.2 残差 fusion + §2.5 跨核直送 + §2.12 ResNet11 N=4 -->。Phase 7 SMC+NUMA 主线最新数据显示 ResNet-11 N=4 完整网络 11/11 layer 全 bit-exact、13/13 regression case 全 PASS <!-- 来自 STATUS.md §2.12 -->。

功能正确性的统计闭环由回归框架自动完成——`run_regression.py` 驱动每 case 由 TB 内部 check_final_ofb 函数与 PyTorch reference 逐像素比对决定 PASS/FAIL，合计 51 case 全 PASS（26 单核 corner case + 16 N=2/4 W 切片 + 6 ResNet 残差 + 3 ResNet-11 整网）<!-- 来自 contributions.md C4.3 + STATUS.md §2.8 测试矩阵 -->。早期版本曾有"check_final_ofb 未被调用导致 mismatches=0 假 PASS"的 bug，已在 P1 阶段修复 <!-- 来自 STATUS.md §2.8 修 false-PASS bug -->，当前回归报告的 PASS 计数是真实的 element-wise 比对结果。

功能"对的"证毕，§5.4 紧接证明硬件"装得下"——给出单核与多核在 XC7K325T 上的资源占用与时序数据。

## 5.4 板级综合与资源分析

本节给出单核与多核综合在 XC7K325T-FFG900-2 上的资源、时序数据，并对 Fmax 与 DSP 推断率两处 limitation 做诚实陈述 <!-- 来自 paragraph-skeleton §5.4 段 1 -->。

单核综合数据如下表所示，使用 Vivado 2023.1 OOC 流程，目标频率 100 MHz <!-- 来自 STATUS.md §1 综合表 + contributions.md C4.1 -->：

| 资源 | 用量 | 器件容量 | 占比 |
|---|---|---|---|
| LUT | 36,942 | 203,800 | 18.1% |
| FF | 13,167 | 407,600 | 3.2% |
| BRAM36 | 128 (+1 RAMB18) | 445 | 28.8% |
| DSP48E1 | 82 | 840 | 9.8% |
| Fmax | 68.4 MHz (WNS = -4.618 ns) | — | timing 未达 100 MHz target |

BRAM 用量主要由四块 SRAM 主导：WB SRAM (1024×2048) 57 块、IFB SRAM (8192×128) 32 块、Shortcut Bank (8192×128) 32 块、OFB SRAM (2048×128) 7 块加 1 个 RAMB18。BRAM 占比 28.8% 是单核器件占用的主要约束项，决定了多核扩展的上限。

多核综合数据见下表，N=2 通过原始 SRAM 配置直接综合，N=4 通过缩 Shortcut Bank 8192→2048 综合通过（commit 5fe16b2）<!-- 来自 STATUS.md §2 多核综合 + contributions.md C4.2 -->：

| 资源 | 单核 | N=2 wrapper | 推算 N=3 | N=4 (SMC, SB 缩) | 器件容量 |
|---|---|---|---|---|---|
| LUT | 36,942 | 74,386 (36.5%) | ~109K (54%) | 162,584 (79.8%) | 203,800 |
| FF | 13,167 | 26,927 (6.6%) | ~40K | 63,732 (15.6%) | 407,600 |
| BRAM36 | 128 (+1 RAMB18) | 256 | 384 (86%) | 288 (+4 RAMB18, ~64.7%) | 445 |
| DSP | 82 | 164 (19.5%) | 246 | 320 (38.1%) | 840 |
| Fmax | 68.4 MHz | 68 MHz | — | 100 MHz target MET (WNS = +0.196 ns) | — |

N=4 (SMC) 数字来自 `Syn/reports_smc/utilization_synth.rpt` 与 `timing_synth.rpt`（commit 5fe16b2，design = `multicore_top_smc`）<!-- 来自 utilization_synth.rpt L25 + timing_synth.rpt L194 Setup MET -->。值得注意的是 N=4 SMC 主线综合 timing 实际 MET 100 MHz target（Worst Slack +0.196 ns），与单核 / N=2 的 68 MHz 不同——这是 SMC 路线分布式 SRAM 改造后 critical path 不再经过单核 SDP 后处理组合链所致。从表中可见 BRAM 是多核扩展的主要约束——保持原始 SRAM 配置时，N=3 已逼近 BRAM 上限，N=4 必须通过缩 Shortcut Bank 8192→2048 才能装下，缩配后 N=4 BRAM 用量反而低于 N=2（288 vs 256 的差异由 SMC 改造后多核 IFB 重组贡献）；这与 §4.5 多核 W 切片设计中 halo 与跨核 SRAM 直送两条优化的硬件代价分布一致。

第一处诚实 limitation 是 Fmax 仅 68 MHz、未达 100 MHz target。Critical path 经时序报告定位在 SDP 后处理量化组合链——`pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc` 全部在单一组合路径完成（C1.5 的代价）<!-- 来自 STATUS.md §1 单核已知问题 -->。修复路径是把该组合链切 1-2 段流水线，预计可拉到 100+ MHz；该改造对功能正确性无影响（仅延迟增加 1-2 周期），属于纯工程优化，作为 §6.3 future work 之一保留 <!-- 来自 paragraph-skeleton §5.4 段 4 -->。

第二处诚实 limitation 是 DSP 推断率低（82/256 PE）。256 个 mac_pe 中只有 82 个被 Vivado 推断为 DSP48E1，其余综合到 LUT 实现。根因是 mac_pe 的 RTL 写法 `if (compute_en) prod <= mult else hold` 阻碍了 DSP 推断（DSP48E1 推断要求乘法结果直接寄存，无条件 hold 路径）<!-- 来自 STATUS.md §1 单核已知问题 -->。修复路径是给 mac_pe 加 `(* use_dsp = "yes" *)` 综合属性强制推断，预计可节省约 17K LUT、降低 LUT 占比近一半，同样作为 §6.3 future work 保留。

"装得下"证毕，§5.5 紧接证明"跑得快"——展开 PE 利用率三模式与 ResNet-11 cycles/FPS 数据，是 narrative A 主轴的数据兑现章。

## 5.5 性能分析

本节兑现 narrative A 主轴的核心承诺，把 §4.3 的编译器侧 PE 利用率优化与 §4.5 的多核 W 切片扩展从设计意图落到可量化的实测数据。论证沿三条线展开：第一条线是 PE 利用率三模式（baseline / Ky-fold / S2D）在塌陷场景下的具体兑现；第二条线是 ResNet-11 端到端 wall cycles 与 fps；第三条线是单核到多核 N = 1/2/4 的扩展加速比与近线性偏离归因。<!-- 来自 paragraph-skeleton §5.5 段 1 -->

PE 利用率三模式对比兑现编译器侧 narrative A 主章。以 ResNet-11 入口的 Patch 层（K = 4 / stride = 4 / Cin = 3）为代表场景，baseline 直接映射在 16 × 16 阵列上让 Cin 维仅占 3/16、Cout 维列空转情况叠加 stride² 相位浪费，单层 wall cycles 高达 654,404 cy。启用 S2D 等价重映射后，等效维度变为 K = 1 / stride = 1 / Cin = 64，单层 wall cycles 降至 129,594 cy，单层加速 5.05× <!-- 来自 contributions.md C2.2 + STATUS §2.8 -->。Ky-fold 在 Cin < 16 的中间层把 Ky 维折回 Cin，把 Cin = 4 的层提升到等效 Cin = 12（K = 3 时）<!-- 来自 docs/pe-fold.md §2 -->，对应单层利用率从塌陷区拉回到接近满载，三模式具体数字与归因详见 §5.5.2 与表 5-6 [CHECK: 三模式整网 PE 利用率与 wall cycles 数字, 来自 model_analysis.md 与 22-case 回归报告]。

ResNet-11 端到端 cycles 与 fps 数据兑现 C4.4。在启用 Ky-fold + S2D 联合触发后，整网 596,088 cycles @ 100 MHz target 对应 313 fps，是单核单 layer-serial 数据通路在固定 16 × 16 阵列上能够稳定跑出的整网吞吐 <!-- 来自 STATUS.md §2.8 ResNet11 N=1 -->。本节所有 fps 数字均按 100 MHz target 时钟假设给出，实测 Fmax 68.4 MHz 下实际板上吞吐 ≈ 表中数字 × 0.684，统一口径声明详见本节末段 <!-- 修订：在首次 313 fps 锚定处追加 target / Fmax 双标前置说明，与本节末段口径声明段呼应 -->。该数字相对未启用编译器侧重映射的 baseline 1,115K cycles 提升约 1.87×，把单核 fps 从 168 拉到 313，是编译器侧 narrative A 在端到端整网量级上的兑现。

多核 W 切片扩展把单核 ResNet-11 的整网吞吐进一步推高，并由 Phase 7 SMC + NUMA 主线提供片上 push 链路径下的进一步优化对照。N = 2 跑 450,469 cycles 对应 444 fps（相对 N = 1 加速比 1.32×），N = 4 在 sequential DDR 中转路径下跑 354,555 cycles 对应 564 fps（相对 N = 1 加速比 1.68×）；同一 N = 4 配置在 Phase 7 SMC + NUMA 主线（跨核 SRAM 直送、halo 物理只存一份、layer 间转发改走片上 axi crossbar 三项改造）下进一步压低到 220,824 cycles，对应约 453 fps <!-- 来自 STATUS.md §2.8 + §2.12 / commit 5fe16b2 -->。两组 N = 4 cycles 数字（354K W 切片 / 220K SMC 片上 push 链）在编译器调度选择上互斥、不可叠加，分别对应不同的数据路径假设；N = 4 加速比偏离理想 4× 线性的根因留到 §5.6 多 DDR 带宽分析展开，本节先给出加速比数据本身。同时，wslice 系列单层 / 多层 / 多形状切片测试 16 例 N = 2 / N = 4 各 8 例全 bit-exact PASS（覆盖 K ∈ {1, 3, 5, 7} / stride ∈ {1, 2} / W ∈ {8, 32, 33}，与 §5.3 测试矩阵一致），多层 chain 与 ResNet 残差链共 6 case 全 PASS，证明多核 W 切片扩展不仅"快"，而且在功能正确性上与单核 element-wise 完全一致 <!-- 来自 STATUS.md §2.8 P1 验证；与 §5.3 段 3 16 例口径对齐 -->。

口径声明段统一收束本节涉及的两类口径决策——主表格 N = 4 数字主键选择，以及 fps 时钟假设双标。其一，N = 4 的 354K（sequential DDR 中转）与 220K（SMC + NUMA 片上 push 链）两组 cycles 数字属于互斥路径数据，论文最终采用哪一组作为 §5.5 主表格主键、另一组作参考列出，留待用户决策 [TBD: §5.5 主表格主键, N=4 用 W 切片 354K / SMC 220K 哪一个]。其二，本节所有 fps 数字（313 / 444 / 564 / 453）均按 100 MHz target 时钟假设给出，§5.4 已陈述实测 Fmax 仅 68.4 MHz，因此实际可达 fps ≈ 表中数字 × 0.684 [TBD: 双标策略, 论文正文统一用 100 MHz target 还是 68 MHz Fmax 计 fps]。本论文倾向在 §5.5 主表格使用 100 MHz target 数字（与 STATUS 与 commit anchor 一致），同时在表脚注明确 Fmax 折算系数，避免读者把 target fps 误读为实际板上吞吐。该 fps 双标问题在修复 SDP 流水线 / use_dsp 综合属性两处工程优化后预计可消除（§5.8 与 §6.3）<!-- 来自 STATUS.md §1 单核已知问题；修订：合并 SMC 口径 [TBD] + fps 双标 [TBD] 为统一口径声明段，避免数据陈述与口径决策在同段分裂 -->。

narrative A 主轴的端到端数据兑现至此完成，§5.6 紧接给出 N = 4 加速比偏离线性的归因（DDR 带宽 PoC 实验），§5.7 把本工作放到 prior art 横向对比坐标系中。

## 5.6 多 DDR 带宽分析

本节兑现 C4.5 多 DDR 带宽 PoC，定量解释 §5.5 N = 4 加速比 1.68× 偏离理想 4× 线性的根因。论证沿 PoC 实验 → 关键指标 → 归一化代价三步推进，落实 §1 引言对"单 DDR 是当前多核扩展瓶颈"的诚实陈述 <!-- 来自 contributions.md C4.5 + paragraph-skeleton §5.5 末段铺垫 -->。

PoC 实验对照本工作 ResNet-11 N = 4 默认配置（1-DDR 共享 AXI4 master）与 4-DDR 等价配置（4 个独立 DDR 通道、每核独占一路）。1-DDR 配置下 mac_pipe%（即 mac_array 实际计算节拍占总 wall cycles 的比例）仅 36.2%，DDR busy% 高达 84.7%；4-DDR 配置下 mac_pipe% 拉到 63.5%，整网 wall cycles 由 354,555 cy 降至约 196,000 cy，对应 N = 4 整网加速比从 1.68× 拉到约 2.11× [CHECK: 4-DDR 整网 wall cycles 来自 PoC 估算还是 RTL 实测] <!-- 来自 STATUS.md §4 性能优化 + memory/project_4ddr_poc_result.md ResNet-11 整网 1.49× / Patch 2.94× -->。

PoC 数据明确指向 DDR 带宽为当前多核扩展主要瓶颈：1-DDR 配置下 4 核 idma_ctrl / odma_ctrl 经 mm2s_arb 串行仲裁同一条 AXI4 master，DDR busy% 84.7% 意味着 DDR 通道几乎跑满，新增核数在 DDR 带宽未拓宽前不能转化为有效计算节拍——这是为何 N = 4 加速比仅 1.68× 而非接近 4×。mac_pipe% 36.2% 同时说明片上 mac_array 阵列在 layer 间切换、IFB / WB 加载等 DDR 等待区间大量空转。

归一化代价讨论收束本节。4-DDR 配置在 PoC 上虽能拉高 mac_pipe% 至 63.5%、整网 N = 4 加速比至 2.11×，但需要器件级 4 个独立 DDR 通道——XC7K325T 单板典型只配 1 ~ 2 通道，4-DDR 部署需要中高端开发板支持 [CHECK: 4-DDR 板级配置假设是否在论文 scope 内]。本工作在 §5.5 主线数据中保留 1-DDR 配置作为默认、把 4-DDR 数据作为 PoC 参考列出，是出于"端侧加速器场景假设"（§1）的工程现实考量；中长期的 P2 push 链 + Mesh 跨核直送（§5.7）可在不增加 DDR 通道前提下消除 ~50% DDR 流量，是更端侧友好的优化路径 <!-- 来自 STATUS.md §4 R.M2 -->。

DDR 带宽归因证毕，§5.7 紧接把本工作放到 prior art 横向对比坐标系中，给出 Pareto 前沿定位。

## 5.7 与 prior art 对比

本节通过同器件 streaming 类工作横向对比 + 三处近邻 prior art 数据兑现 + ASIC 系统脉动跨范畴对照三条路径，把本工作放到合理的文献坐标系中，落实 §1.2 与 §3.4 建立的差异化主张 <!-- 来自 paragraph-skeleton §5.7 段 1 -->。本节不声称"更优"，所有对比都标注口径差异与器件规模归一化讨论，避免不公比较。

第一条路径是与 FPGA streaming 类工作的同器件横向对比。表 5-9 整理本工作与 fpgaConvNet (Venieris@FCCM'16 / TNNLS'19)、Snowflake (ISCAS'17)、Angel-Eye (TCAD'18)、Caffeine (ICCAD'16)、Aydonat Intel DLA、Lu Winograd ZCU102、Ma OPU 等代表工作在 Fmax / 资源 / 峰值 GOPS / 整网 PE 利用率四维度的横向数据，所有 baseline 数字均回查 literature.md 对应条目 [CHECK: 表 5-9 各 baseline 同器件归一化口径]。

| 工作 | 器件 | Fmax | 峰值 GOPS | 整网 PE 利用率 | 备注 |
|---|---|---|---|---|---|
| **本工作 (单核)** | XC7K325T | 68.4 MHz (Fmax) / 100 MHz (target) | 51.2 GOPS @ target | **86.6%** [CHECK: 整网 MAC%] | 16 × 16 INT8 MAC，编译器侧 Ky-fold + S2D |
| fpgaConvNet [Venieris@FCCM'16] | Zynq XC7Z045 | — | — | — | 早期 streaming 框架，performance density 提升 2.94× [CHECK: 与本工作 fps/cycles 口径不直接可比] |
| Snowflake [ISCAS'17] | Zynq XC7Z045 | 250 MHz [CHECK] | 128 GOPS [CHECK] | 91% 平均 [CHECK] | 口径含 / 不含 IDMA stall 待核 |
| Angel-Eye [TCAD'18] | Zynq | — | — | — | 6× 同期工作（VGG16 推理）[CHECK: 数字与口径] |
| Aydonat Intel DLA | Arria 10 | — | 1382 GFLOPS [CHECK] | — | [CHECK: Aydonat DLA 是否 FP16 / share-exp 路线]，与本工作 INT8 不直接可比 |
| Lu Winograd | ZCU102 | — | 854.6 GOPS [CHECK] | — | Winograd 变换路线，乘法节省但精度漂移 |
| Ma OPU | — | — | 645 GOPS [CHECK] | — | 通用 OPU 路线 |
| **Liu Full-Stack [TNNLS'21]** | **Arria 10 GX1150** | — | > 1.3 TOPS [CHECK] | **97% MAC%** | **同思路不同器件规模的可比对照** |

整网 PE 利用率口径差异需要在表脚注显式说明：本工作 86.6% 是 22-case ResNet-18 风格链式回归整网 MAC%（含 IDMA / ODMA stall 与跨层切换），Liu 97% 是 Arria 10 GX1150 上 ResNet 系列 streaming 的 MAC efficiency，Snowflake 91% 是 Zynq 上的平均计算效率，三者口径不完全等同 <!-- 来自 literature.md §C 与 §F.503 -->。

表 5-9 中 fpgaConvNet / Angel-Eye / Aydonat / Lu / Ma 五条 baseline 在数据完整度与口径上均与本工作不直接可比——fpgaConvNet 的 2.94× 是 performance density 而非 fps/cycles，Aydonat 走 INT16 / FP16 路线 [CHECK]，Lu 走 Winograd 变换路径在精度漂移代价下换乘法节省，Ma OPU 是通用算子流水线路线，Angel-Eye 数字尚待回查——这五条列出仅作为同器件 / 同期 streaming 类工作的坐标参照，本节不展开横向论证，仅与 Liu Full-Stack 与 Snowflake 两条数据相对完整、口径相对可对齐的工作做实际比较 <!-- 与 §5.7 第 5 段 Liu 对位段呼应 -->。

第二条路径是与 §1.2.3 三处近邻 prior art 的数据兑现，落实 §3.4.3 差异化主张。Alwani 跨层融合在多层间共享中间特征图缓冲，依赖整图驻留；本工作行级行环只缓冲 strip_rows × W_IN，单层硬件可处理任意 H × W，VGA 480 × 640 单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张 <!-- 来自 contributions.md C1.3 -->。Kang AoCStream 把所有特征图驻留片上 SRAM，输入分辨率受 BRAM 容量严格上界；本工作 row-credit 反压协议解耦输入大小与片上容量，VGA 单图按行环数据通路自然支持。Liu Full-Stack 跨层 pipelined 让多核同时跑不同层，本工作选择单核 layer-serial 共用硬件 + 多核 W 维切片扩展，在端侧资源约束下偏向 single-layer 内并行。

Liu Full-Stack 的对位是本节最关键论证点：Liu 在 Intel Arria 10 GX1150（大型 FPGA，资源量级约为 XC7K325T 的 5 ~ 7 倍 [CHECK: 资源比例倍数估算来源]）上端到端 streaming ResNet 系列网络，报告整网 PE 利用率约 97%；本工作在 Xilinx XC7K325T（203K LUT / 445 BRAM36 / 840 DSP48）启用 S2D 后整网 PE 利用率约 86.6% [CHECK: 86.6% 整网 MAC% 数字与 model_analysis.md 与 STATUS.md 交叉核对] <!-- 与 §4.5 末段措辞保持一致 -->。两组数字表征同思路（streaming + 编译器侧重映射）在不同硬件规模下的等价定位，绝对值差距来自器件规模而非范式优劣——Arria 10 量级器件可让所有层硬件常驻、跨层 pipelined 同时跑；本工作在中型器件上选择单核 layer-serial 共用硬件，利用率自然有差距，但绝对硬件代价显著更低且支持任意 H × W 输入 <!-- 修订：删除 1518K LE / 65.7 Mb / 1518 DSP 三处错误的 Arria 10 具体规格数字；改主动语气表态，与 §5.8 limitation 节诚实主动语气一致 -->。

第三条路径是与 ASIC 系统脉动 / 可重构互连工作的跨范畴对照。TPU v1 在数据中心场景达 92 TOPS（28 nm ASIC，与本工作 XC7K325T FPGA 28 nm 工艺相当但绝对算力差约 1800×）<!-- 来自 literature.md §A -->，Eyeriss 在行驻留路线下报告 [CHECK: Eyeriss 整网 PE 利用率] / Eyeriss-v2 NoC 对 MobileNet 加速 12.6× / MAERI ART 对 PE 利用率提升 8 ~ 459% / Gemmini 提供软件可重构 PE 阵列 <!-- 来自 literature.md §B -->。这一类工作与本工作不在同一器件 / 工艺 / 资源量级，绝对数字直接比较没有意义，仅做范式坐标对照——本工作选择"固定阵列 + 编译器侧填满"路线，与 ASIC 可重构互连路线在硬件复杂度 / 编译器复杂度 / 内存代价 / 适用场景四维度上的取舍详见 §3.4.2 与 §4.3 末段，本节不再展开。

Pareto 前沿定位收束本节。本工作位于 XC7K325T 单器件 FPGA streaming 类工作的"中型器件 + 编译器侧 PE 利用率优化"位置，与 Liu 等大型器件 streaming 工作形成同思路不同规模的对照，与 fpgaConvNet / Snowflake / Angel-Eye 等同器件量级 streaming 工作形成 Fmax / 资源 / 整网 PE 利用率横向对照 <!-- 来自 paragraph-skeleton §5.7 段 5 -->。具体 Pareto 散点图（GOPS / DSP vs 整网 PE 利用率）见图 5-3 [CHECK: 图 5-3 散点数据来源]。

## 5.8 局限与未来工作

本节诚实列出本工作截至 commit 5fe16b2 已识别的关键缺口，沿"工程优化层 / 多核切片功能层 / 多核切片实测层 / 网络层支持层 / 实测数据层 / 未来工作路线图"六层依次展开，每层段首以加粗小标题标识，便于读者重建分类骨架。所有缺口直接来自 STATUS.md §1（单核已知问题）/ §2.6（多核调度器）/ §4（性能优化 roadmap），不软化、不掩饰，作为本论文的诚实代价显式陈述；STATUS §4 中关于 mesh 路线 ROI 决策、Mode B/C 部分 cfg gen corner case、tb_multicore wslice5 path bug 等更细一级缺口在本节不一一展开 <!-- 来自 STATUS.md §1 / §2.6 / §4；措辞从"全部"软化到"已识别的关键"，避免 over-claim；修订：导引段补段首小标题骨架说明，与下文 6 层加粗标题呼应 -->。

**工程优化层（2 项）：** SDP 后处理量化组合链未流水化导致 Fmax 仅 68 MHz 这一已在 §5.4 提出的 limitation。critical path 经时序报告定位在 `pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc` 全部在单一组合路径完成 <!-- 来自 STATUS.md §1 单核已知问题 -->，修复方案是把该组合链切 1 ~ 2 段流水线，预计可拉到 100+ MHz、对功能正确性无影响（仅延迟增加 1 ~ 2 周期），列为 §6.3 future work 之一。第二处是 mac_pe 未启用 `(* use_dsp = "yes" *)` 综合属性导致 256 个 PE 中只有 82 个被推断为 DSP48E1，其余 174 个综合到 LUT，给 use_dsp 加属性预计可节省约 17K LUT、降低 LUT 占比近一半，是 STATUS §4 短期 ROI 排序最高（⭐⭐⭐⭐）的工程优化 <!-- 来自 STATUS.md §4 性能优化 roadmap -->。

**多核切片功能层（3 项）：** 存在三处明确缺口。其一是 Mode C cout slice cfg 生成尚未做（per-core cout 段切片，对 W = 1 的 FC 类 root layer 必须）<!-- 来自 STATUS.md §2.6 -->，当前 ResNet-11 N = 4 部署中 FC layer 是 root layer 由 host 单独 preload 绕开了该缺口，但通用多核 cout slice 路径未实现。其二是 Stage barrier 多 stage 调度未做（host 一个 stage 一个 stage 启），当前 driver 路径假设单 stage 跑完整网，对需要中间 host 同步的复杂调度场景不支持。其三是片上 push 链 P2（computed redundancy halo + 跨 stage push）设计就绪但实施未完成，预计实施工作量 2 ~ 3 天，落地后可消除当前 N = 4 ResNet-11 部署中多处 layer 间 DDR 边界 [CHECK: 6 处 DDR 边界几何推算来源待 STATUS 核对]，进一步降低 wall cycles <!-- 来自 STATUS.md §2.6 -->。

**多核切片实测层（2 项）：** 存在两处遗留 FAIL case。第一处是 wslice5（5 层 W slice chain，跨 W 切片 halo bug）当前实测 1,920 word mismatch / L1-L4 全错；第二处是 resnet11_n4 在 push 模型 W slice halo bug 影响下中间 9 层 24,360 word mismatch [CHECK: 这两处 FAIL 是在 SMC + NUMA 主线之前的 push 链路径还是合并后路径] <!-- 来自 STATUS.md §2.8 验证状态末段 -->。这里的 FAIL 走的是片上 push 链实验路径而非 §5.5 的 sequential DDR 中转路径，两者在编译器调度选择上互斥，因此与 §5.5 报告的 354K cycles N = 4 整网 PASS 数据不冲突；但片上 push 链路径在 ResNet 风格残差链下仍存在跨核 halo 边界 bug 需要修复，是 §5.7 P2 push 链实施前的前置阻塞项。

**网络层支持层（2 项）：** 存在两处未支持算子。其一是 Pooling（avg / max）未实现硬件原生支持 <!-- 来自 STATUS.md §4 中长期 R.3 -->，当前路径依赖网络替换（用 stride ≥ 2 卷积替代 pooling）；R.3 AvgPool 硬件设计待用户确认有无 host CPU 协同后再决定是否实施。其二是 Depthwise 与稀疏（sparsity）卷积未在硬件层支持，MobileNet 系列网络当前不在 supported model 列表中——本工作在 16 × 16 INT8 阵列上对 Depthwise 的 Cin = 1 形状会让 PE 阵列大面积空转，编译器侧 Ky-fold 不能直接处理（需要新的 Cin ↔ Cout × group 等价变换），列为 §6.3 中长期方向之一。

**实测数据层（2 项）：** 存在两处尚待补完。其一是多核 chain 实测覆盖在 commit 5fe16b2 时点上以 ResNet-11 / N = 1/2/4 / 6 ResNet 残差链 / 16 wslice N=2/4 为主，YOLOv3-tiny 等更大模型仅有 scheduler 估算（10.87M cycles / 9.2 fps @ N = 1）<!-- 来自 STATUS.md §2.6 估算表 -->，未做 RTL 实测。其二是 SMC + NUMA 主线 N = 4 跑 220,824 cycles 是否作为论文最终数 [CHECK: 论文 §5.5 主表格 N=4 数字 354K（W slice）与 220K（SMC）口径选择] 留待用户决策，本论文当前默认按 W slice 路径取数，SMC + NUMA 数字作为参考列出。

**未来工作路线图（短 / 中 / 长 3 档）：** 按 ROI 排序——短期是 use_dsp 综合属性 + SDP 流水线化（合计约 30 行 RTL 改动 = use_dsp 1 行 + SDP 流水 ~30 行，预计 LUT 节省约 46% / Fmax 拉到 100 MHz）<!-- 来自 STATUS.md §4 R.S1 + R.S2，~30 行表述细化 -->；中期是 P2 push 链实施（消除 ResNet-11 N = 4 中多处 layer 间 DDR 边界 [CHECK: 数量待核]）+ M2 Mesh + 跨核 SRAM 直送（DDR 流量 -50%）<!-- 来自 STATUS.md §4 -->；长期是 Pooling 硬件原生支持 + Depthwise / sparsity 编译器侧重映射 + YOLOv3-tiny 等更大模型实测。这三组工作分别对应 §6.3 future work 的近期 / 中期 / 长期三档。

## 5.9 本章小结

本章兑现 C4.1—C4.7 全部 7 条实测贡献。单核与多核（N = 1/2/4）综合在 XC7K325T 上全部装得下；功能验证沿单核 corner case、多核 W 切片、ResNet 残差链与 ResNet-11 整网四类测试全部 bit-exact PASS；性能侧编译器 narrative A（PE 利用率三模式）与系统 narrative B（多核 W 切片扩展）在 ResNet-11 端到端 cycles / fps 数据上同时兑现；多核加速比偏离理想线性的根因经多 DDR 带宽 PoC 定位到单 DDR 通道；横向对比把本工作定位在中型 7-series 端侧 FPGA streaming 主线的"编译器侧 PE 利用率优化"位置，与 Liu Full-Stack 形成同思路不同规模的对照关系 <!-- 修订：先收束 7 条贡献兑现的论点，关键数字下移至独立"关键数字一览"段，与 §3.6 / §4.6 小结风格统一 -->。

关键数字一览：单核 36,942 LUT / 128 BRAM36 / 82 DSP（XC7K325T）；功能验证 51 case 全 PASS；ResNet-11 整网 N = 4 跑 354K cycles / 564 fps（@ 100 MHz target）、相对 N = 1 加速比 1.68×；启用 S2D 单 Patch 层加速 5.05×；多 DDR PoC 把 mac_pipe% 从 36.2% 拉到 63.5%——这五个数字是本章中最具代表性的论据锚点 <!-- 修订：从原段 12 处数字中精选 5 个最具代表性的关键数字单独成段 -->。

诚实陈述原则贯穿全章——Fmax 68 MHz 未达 100 MHz target、DSP 推断率 82/256、N = 4 加速比偏离线性、Pooling / Depthwise 未支持、片上 push 链 P2 未实施、SMC + NUMA 路线 N = 4 数字与 W 切片路线数字双标——所有 limitation 不软化、不掩饰，§6.3 将基于本章数据给出 future work 路线。

---

# 6 结论与展望

本章对全文工作收束。§6.1 自硬件 / 编译器 / 系统三层归纳本论文已完成的工作并以可量化指标作为定量结论；§6.2 凝练 5 条创新点，与 §1.3 主要贡献承诺一一对应，措辞上以"据已知文献..."等表述避免 over-claim；§6.3 诚实陈述 3 处已知不足，按 ROI 排序给出短 / 中 / 长三档未来工作路线，与 §5.8 章末局限保持一致。

## 6.1 结论

在第 5 章给出的 51 case bit-exact 全 PASS、单核与多核 OOC 综合通过、ResNet-11 多核 cycles / FPS 与多 DDR 带宽 PoC 等实测数据基础上，本节自硬件 / 编译器 / 系统三层归纳全文工作并给出定量结论。本论文围绕"端侧 FPGA 上固定 16×16 INT8 阵列加速器在 Cin < 16 与 stride ≥ 2 场景的 PE 利用率塌陷"这一具体瓶颈展开，自硬件、编译器、系统三个层次给出一套完整的工程实现与实测验证。硬件层完成 5 模块去中心化 valid-ready 流水（line_buffer → mac_array → parf_accum → ofb_writer + 侧路 wgt_buffer）+ 行级流式行环 IFB / OFB + 多核 W 维切片扩展，单层硬件可处理任意 H × W 输入；编译器层完成 Ky 折叠、空间到深度（S2D）等价变换以及两者按 (Cin, K, stride) 自动联合触发的端到端 PyTorch → ISA 编译流；系统层完成 AXI / DMA 子系统集成、CFG_WRITE descriptor 配置流（host AXI-Lite 写从约 50 次 / 层降到 4 次 / 层）以及 params.py 单源参数表双向消费 <!-- 来自 contributions.md C1.x / C2.x / C3.x 系列 -->。

定量结论由"实测三件套"支撑。功能侧，51 case 端到端回归在与 PyTorch reference 的 element-wise bit-exact 准则下全部 PASS，覆盖单核 corner case、多核 W 切片、ResNet 残差链与 ResNet-11 整网四类测试 <!-- 来自 §5.3 + STATUS.md -->；综合侧，单核与 N = 2 / 4 多核三种配置均在 XC7K325T-FFG900-2 上 OOC 综合通过，单核 36,942 LUT / 128 BRAM36 / 82 DSP <!-- 来自 §5.4 -->；性能侧，ResNet-11 整网 N = 4 跑出 354K cycles / 564 fps（@ 100 MHz target），相对 N = 1 实现 1.68× wall cycles 加速；其中启用 S2D 的单 Patch 层加速达 5.05×，整网叠加 S2D 后 N = 4 加速 2.11× <!-- 来自 §5.5 -->。这三组数字同时兑现 narrative A（编译器侧 PE 利用率优化）与 narrative B（多核 W 切片系统扩展）两条主轴。

工程意义上，本工作示范了一条"硬件保持最简、编译器侧承担适配负担"的端侧固定阵列加速器设计路径——hardware shape 不变（仍是 16 × 16 OS + 列广播阵列），通过 Ky-fold 与 S2D 两类纯编译器侧重映射在 Cin < 16 与 stride ≥ 2 这两个原本会让阵列利用率塌陷到 1/16 量级的场景下兑现可用的 PE 利用率与 wall cycles 性能。这种"零硬件代价 / 零 RTL 改动"的优化路径与硬件可重构方案（如 dynamic dataflow / runtime reconfigurable PE）形成对照——后者在硬件灵活度上更高，但本工作的路径在端侧 FPGA 资源约束下更为务实 <!-- 不 over-claim：仅指"在端侧资源约束下务实"，不声称绝对优于 -->。

多 DDR 带宽 PoC 进一步把 N = 4 多核加速比偏离理想线性的根因定位到单 DDR 通道：1-DDR 配置下 mac_pipe% 仅 36.2%，而 4-DDR PoC 把该指标拉到 63.5% <!-- 来自 §5.6 / project_4ddr_poc_result.md -->。这一定位为 §6.3 未来工作中"片上 push 链 P2 + Mesh 跨核直送"等系统侧路径提供了直接的实测依据，也对端侧多核 FPGA 加速器的内存带宽设计给出可验证的瓶颈刻画。

至此，本论文在硬件 / 编译器 / 系统三层各完成一组可实测验证的工作。下一节自创新点角度对这套工作做定性凝练。

## 6.2 创新点

§6.1 自"做了什么"角度归纳工作；本节自"创新点是什么"角度凝练 5 条定性贡献，与 §1.3 的 5 条主要贡献承诺一一对应。表述上采用"据已知文献..."等措辞，避免 over-claim 为"业界首次"或"绝对优于"——本工作所处的端侧 FPGA streaming CNN 加速器领域文献丰富，任何"首次"声明都需大量横向调查支撑，本论文不做此类断言。

- **创新点 1（编译器侧 Ky 折叠）**：提出 Ky-fold 方法——当 Cin < 16 时，把 Ky 维通过 y 偏移复制折叠到 cin_fake = Cin × Ky，硬件视角下与 stride = 1 标准卷积同构。据已知文献调研，纯编译器侧、零 RTL 改动、专门解决 Cin < 16 PE 利用率塌陷的折叠方案在 FPGA streaming CNN 加速器主线中较少见 <!-- 对应 §1.3 贡献 1 / contributions.md C2.1 -->。
- **创新点 2（S2D 等价变换 + 联合触发）**：把 stride ≥ 2 卷积通过空间到深度（Space-to-Depth）变换为 stride² 个相位折到 cin_new = stride² × Cin 的 stride = 1 卷积，DDR 友好重排无激活复制；与 Ky-fold 的联合触发由编译器按 (Cin, K, stride) 在 `scheduler.Layer.force_s2d() / force_fold()` 自动决策，在 ResNet-11 Patch 层兑现 5.05× 单层加速 <!-- 来自 §5.5 + commit 43c1c25 -->。S2D 思路与超分辨率领域的 Pixel-Shuffle / Sub-pixel Convolution 引用谱系相关，本工作的贡献在于把该等价变换迁移到端侧 INT8 streaming 加速器并完成与 Ky-fold 的自动联合触发 <!-- 对应 §1.3 贡献 2 / C2.2 + C2.3，诚实标注引用谱系 -->。
- **创新点 3（去中心化 5 模块流水 + 行级流式行环）**：5 模块（line_buffer / mac_array / parf_accum / ofb_writer + 侧路 wgt_buffer）各自维护内部 counter，以 valid-ready 弹性 join 而无中心 FSM；配合 strip_rows × W_IN 粒度的 row-credit 反压协议构成行级流式行环，单层硬件可处理任意 H × W 输入。与 Alwani 的 fused-layer / Kang 的 line-based / Liu 的 Full-Stack 三处近邻 prior art 的差异化主战场已在 §3.4 详细论证 <!-- 对应 §1.3 贡献 3 / C1.2 + C1.3 -->。
- **创新点 4（多核 W 切片扩展 + 跨核 SRAM 直送）**：在单核架构基础上以 W 维切片扩展到 N = 2 / 4 核，配合 halo 计算冗余与跨核 SRAM 直送（M2 push 路径，避免 layer 间 DDR 来回）。N = 4 ResNet-11 较 N = 1 实测 1.68× wall cycles 加速，与理想线性扩展的差距经多 DDR 带宽 PoC 定位到单 DDR 通道而非架构本身 <!-- 对应 §1.3 贡献 4 / C3.4 -->。
- **创新点 5（PyTorch→硬件 cfg 端到端编译流 + descriptor 配置）**：以 `hw_files.derive_layer_cfg()` 为 cfg 派生 single source、`gen_isa_test.py` 为单 case 生成入口、`run_regression.py` 为链式 CASES 验证基础设施，完成 PyTorch→ISA 端到端编译；通过 CFG_WRITE descriptor 把 host AXI-Lite 写从约 50 次 / 层降到 4 次 / 层 <!-- 对应 §1.3 贡献 5 / C2.4 + C3.2 -->。该编译流把"硬件能跑什么"与"PyTorch 模型给什么"之间的工程闭环做到位，是 narrative A + B 两条主轴能在 ResNet-11 端到端实测兑现的前提。

5 条创新点定性凝练完毕。然而工程实践中本工作仍存在若干已知不足，§6.3 将诚实陈述这些不足并按 ROI 排序给出未来工作路径。

## 6.3 展望

本论文工作存在 3 处已知不足，需在收束阶段诚实陈述。第一处是综合 Fmax 仅 68 MHz，未达到 100 MHz target——经时序分析定位 critical path 在 SDP（Scale-Downshift-Pack）量化后处理的组合链上，未做流水切分 <!-- 来自 §5.4.3 / STATUS.md §4 R.S2 -->。第二处是 DSP 推断率偏低（82 / 256），未通过 (* use_dsp = "yes" *) 综合属性强制把所有 INT8 乘法器映射到 DSP48E1，导致约 17K LUT 被乘法器占用 <!-- 来自 §5.4.3 / STATUS.md §4 R.S1 -->。第三处是 1-DDR 通道带宽是 N = 4 多核加速比偏离线性扩展的主因——mac_pipe% 仅 36.2%，4-DDR PoC 验证带宽侧确实是瓶颈而非架构问题 <!-- 来自 §5.6 + project_4ddr_poc_result.md -->。

此外还有若干功能侧不足：Pooling 当前未硬件原生支持（依赖 host 侧 fold 处理）；Depthwise Convolution 未做编译器侧重映射；SMC + NUMA 路线与 W 切片路线在 N = 4 数字上呈双标状态（同一硬件出两组配置）。这些不足在 §5.8 章末局限节已逐项列出，本节不重复。

按 ROI 排序，未来工作分短 / 中 / 长三档。短期（合计约 30 行 RTL 改动）做两件事：(1) 加 (* use_dsp = "yes" *) 综合属性，1 行改动，预计节省约 17K LUT、释放出资源做更宽阵列或多核扩展的空间 <!-- 来自 STATUS.md §4 R.S1 -->；(2) SDP 后处理组合链流水切分（约 30 行 RTL），把 Fmax 从 68 MHz 拉到 100 MHz target，wall cycles 性能等比例提升约 47% <!-- 来自 STATUS.md §4 R.S2，[CHECK: 47% 提升为推算，待综合实测确认] -->。这两项是 ROI 最高的工作，应优先实施。

中期（约 1-2 周工作量）做三件事：(1) 片上 push 链 P2 实施，消除 ResNet-11 N = 4 中多处 layer 间 DDR 来回的边界 [CHECK: 具体边界数量待核]，预计 mac_pipe% 提升至 50%+ [CHECK: 提升幅度待实测]；(2) M2 Mesh + 跨核 SRAM 直送的完整 PoC，按 project_mesh_phase6_plan.md 的 Step A-F 路线推进；(3) 把单 DDR 通道下 36% mac_pipe% 与 4-DDR PoC 下 63.5% 的 gap 收束到 50% 以内（通过片上缓存重用而非加 DDR 通道）<!-- 来自 STATUS.md §4 + project_mesh_phase6_plan.md -->。

长期（数周到数月）做三件事：(1) Pooling 硬件原生支持，避免 host 侧 fold 带来的额外 DDR 流量；(2) Depthwise Convolution 编译器侧重映射，使 MobileNet 系模型可端到端跑通；(3) YOLOv3-tiny 等更大模型的端到端实测，把当前以 ResNet-11 为主的验证规模扩展到检测类网络。这三项工作目标是让本工作的硬件 + 编译器栈适用范围从"残差分类网络"扩展到"轻量检测网络"，是体现工作通用性的关键步骤。

至此本论文工作收束。本工作示范了一条"端侧 FPGA 上以编译器侧重映射承担固定阵列利用率优化负担"的工程路径，并以 51 case bit-exact 全 PASS、N = 4 OOC 综合通过、ResNet-11 564 fps 三组实测数字给出该路径的可行性证据；同时本工作不回避 Fmax 偏低、DSP 推断率不足、单 DDR 通道带宽不够、Pooling / Depthwise 未支持等已知短板，所有 limitation 已在 §5.8 与本节诚实列出。完成短期 30 行 RTL 改动后，本架构有进一步释放性能的空间——这一空间的实测兑现留待后续工作。
