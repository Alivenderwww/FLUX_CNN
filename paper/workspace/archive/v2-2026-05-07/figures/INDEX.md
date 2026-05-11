# 图表索引（Phase 6 v2）

> 本索引列出 paper.md 全文引用的所有图（Fig.）与表（Tab.）。每张图/表配双语标题（中文 + Figure/Table 全拼），编号采用 X.Y 格式（章号.序号），与正文一致。
> 
> 最终绘制工具：用户用 Visio / Illustrator / matplotlib 完成。本目录提供示意稿（文字描述 + ASCII / 必要时 Python 初版）。

## 图清单（Figure）

| 编号 | 中文标题 | English Title | 出现章节 | 文件 | 状态 |
|------|----------|--------------|---------|------|------|
| 图 2.1 | 标准二维卷积七层循环嵌套示意图 | Figure 2.1 Seven-level loop nest of 2-D convolution | §2.2 | fig2-1-conv-loop-nest.md | 示意稿完成 |
| 图 2.2 | ResNet 残差短接 identity / projection 形态对比 | Figure 2.2 Identity vs projection short-cut in ResNet residual block | §2.2 | fig2-2-resnet-shortcut.md | 示意稿完成 |
| 图 2.3 | valid-ready 握手协议时序示意 | Figure 2.3 Timing diagram of the valid-ready handshake protocol | §2.4 | fig2-3-valid-ready-timing.md | 示意稿完成 |
| 图 2.4 | 行缓存与片上 ring 结构示意图 | Figure 2.4 Line buffer with on-chip ring structure | §2.4 | fig2-4-line-buffer-ring.md | 示意稿完成 |
| 图 2.5 | 子像素重排（Pixel-Shuffle）等价关系示意图 | Figure 2.5 Pixel-shuffle equivalence between strided convolution and unit-stride convolution | §2.5 | fig2-5-pixel-shuffle.md | 示意稿完成 |
| 图 3.1 | 加速器两层总体架构图 | Figure 3.1 Overall two-tier architecture of the accelerator | §3.3 | fig3-1-overall-architecture.md | 示意稿完成 |
| 图 3.2 | 多核拓扑与地址映射 | Figure 3.2 Multi-core topology and address mapping | §3.3 | fig3-2-multicore-topology.md | 示意稿完成 |
| 图 3.3 | 单源参数 params.py 双向消费流程 | Figure 3.3 Single-source parameter flow of params.py consumed by RTL and Python toolchain | §3.5 | fig3-3-params-single-source.md | 示意稿完成 |
| 图 4.1 | 五模块去中心化 valid-ready 流水线 | Figure 4.1 Five-module decentralised valid-ready pipeline | §4.2 | fig4-1-five-module-pipeline.md | 示意稿完成 |
| 图 4.2 | 流式行环数据通路示意图 | Figure 4.2 Schematic diagram of streaming row-ring datapath | §4.2 | fig4-2-streaming-ring.md | 示意稿完成 |
| 图 4.3 | 16×16 OS + 列广播 MAC 阵列 | Figure 4.3 16×16 output-stationary MAC array with column-broadcast dataflow | §4.2 | fig4-3-mac-array.md | 示意稿完成 |
| 图 4.4 | 分列累加器 PARF 结构图 | Figure 4.4 Per-column partial-sum accumulator (PARF) | §4.2 | fig4-4-parf-accum.md | 示意稿完成 |
| 图 4.5 | SDP 后处理量化融合组合链 | Figure 4.5 Fused SDP post-processing combinational chain | §4.2 | fig4-5-sdp-fused-chain.md | 示意稿完成 |
| 图 4.6 | Ky 折叠等价变换示意图 | Figure 4.6 Schematic diagram of the Ky-fold equivalent transform | §4.3 | fig4-6-ky-fold.md | 示意稿完成 |
| 图 4.7 | 空间到深度（S2D）等价变换示意图 | Figure 4.7 Schematic diagram of the space-to-depth (S2D) transform | §4.3 | fig4-7-s2d-transform.md | 示意稿完成 |
| 图 4.8 | PyTorch→ISA 端到端编译流 | Figure 4.8 End-to-end PyTorch-to-ISA compilation flow | §4.4 | fig4-8-pytorch-to-isa.md | 示意稿完成 |
| 图 4.9 | CFG_WRITE descriptor 配置流时序 | Figure 4.9 Configuration flow timing of CFG_WRITE descriptors | §4.4 | fig4-9-cfg-write-flow.md | 示意稿完成 |
| 图 4.10 | 多核 W 维切片与 halo 几何 | Figure 4.10 W-dimension slicing and halo geometry across cores | §4.5 | fig4-10-wslice-halo.md | 示意稿完成 |
| 图 4.11 | 跨核 SRAM 直送（M2 push）路径 | Figure 4.11 Cross-core SRAM direct push (M2 push) path | §4.5 | fig4-11-cross-core-push.md | 示意稿完成 |
| 图 5.3 | Pareto 前沿散点图（GOPS/DSP vs 整网 PE 利用率） | Figure 5.3 Pareto frontier scatter plot of GOPS-per-DSP versus end-to-end PE utilisation | §5.7 | fig5-3-pareto-scatter.md | 示意稿完成 / 数据 [CHECK] |

## 表清单（Table）

| 编号 | 中文标题 | English Title | 出现章节 | 文件 | 状态 |
|------|----------|--------------|---------|------|------|
| 表 2.1 | WS / OS / RS 三类数据流分类对比 | Table 2.1 Classification of WS / OS / RS dataflows in CNN accelerators | §2.3 | tab2-1-dataflow-classification.md | 示意稿完成 |
| 表 3.1 | 目标器件 XC7K325T 资源容量与本工作占用预算 | Table 3.1 Resource capacity of XC7K325T and resource budget of this work | §3.2 | tab3-1-target-device.md | 示意稿完成 |
| 表 3.2 | 目标算法 PE 利用率塌陷场景定义 | Table 3.2 Definition of PE-utilisation collapse scenarios on the target algorithm set | §3.2 | tab3-2-collapse-scenarios.md | 示意稿完成 |
| 表 3.3 | 三处近邻 prior art 差异化对比 | Table 3.3 Differentiation summary against three nearest prior arts | §3.4 | tab3-3-prior-art-diff.md | 示意稿完成 |
| 表 4.2 | CFG_WRITE descriptor 与裸 AXI-Lite 配置写次数对比 | Table 4.2 Per-layer configuration write counts of CFG_WRITE descriptors versus raw AXI-Lite writes | §4.4 | tab4-2-cfg-write-cost.md | 示意稿完成 |
| 表 5.6 | PE 利用率三模式（baseline / Ky-fold / S2D）与 wall cycles 对比 | Table 5.6 PE utilisation and wall cycles in three compiler modes (baseline / Ky-fold / S2D) | §5.5 | tab5-6-three-mode-perf.md | 示意稿完成 / 数据 [CHECK] |
| 表 5.9 | 与已有 FPGA streaming 工作横向对比 | Table 5.9 Comparison against representative FPGA streaming CNN accelerators | §5.7 | tab5-9-prior-art-compare.md | 示意稿完成 / 数据 [CHECK] |

## 状态汇总
- 图表总数：图 20 张 + 表 7 张 = 27 张
- 双语标题：每张均有中文 + Figure/Table 全拼英文
- 数据图表（§5 内）`[CHECK]` 计数：图 5.3 / 表 5.6 / 表 5.9 共 3 张需待实测数据补完
- 架构 / 数据流 / 等价变换示意图：所有元素均依据已实现的 RTL（commit 5fe16b2）与 docs/modules、docs/pe-fold.md 文件描述
- 编号规则：X.Y（章号.序号），与正文 §2.2 段中的 [依赖: Fig.2.1] 等引用一致
- 文件命名：fig{X}-{Y}-{shortname}.md / tab{X}-{Y}-{shortname}.md
