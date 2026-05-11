# FLUX_CNN 论文图表索引

本目录收录 paper.md (v3) 中全部图表的设计示意稿。每个文件给出双语标题、引用位置、必含元素、ASCII 示意稿；最终发表图由作者使用 PowerPoint / Visio / Adobe Illustrator / matplotlib 完成。

## 总览

| 编号 | 中文标题 | 英文标题 | 类型 | 首次出现 | 状态 |
|------|---------|---------|------|---------|------|
| 图 2.1 | 卷积计算原理示意图 | Schematic of convolution operation | 概念图 | §2.2.1 | 示意稿完成 |
| 图 2.2 | 单层神经网络量化计算原理示意图 | Schematic of single-layer quantized inference dataflow | 数据流图 | §2.5 | 示意稿完成 |
| 图 3.1 | 加速器系统总体架构图 | Overall system architecture of the accelerator | 架构框图 | §3.3 | 示意稿完成 |
| 图 3.2 | 加速核内数据通路结构图 | Datapath structure inside the accelerator core | 数据通路图 | §3.3 | 示意稿完成 |
| 图 3.3 | DMA 子系统结构图 | DMA subsystem structure | 架构框图 | §3.3 | 示意稿完成 |
| 图 4.1 | 行缓存模块结构图 | Line-buffer module structure | 架构框图 | §4.2 | 示意稿完成 |
| 图 4.2 | MAC 阵列模块结构图 | MAC-array module structure | 架构框图 | §4.3 | 示意稿完成 |
| 图 4.3 | 部分和累加模块结构图 | Partial-sum accumulator module structure | 架构框图 | §4.4 | 示意稿完成 |
| 图 4.4 | SDP 后处理模块结构图 | SDP post-processing module structure | 数据流图 | §4.5 | 示意稿完成 |
| 图 4.5 | 权重缓存模块结构图 | Weight-buffer module structure | 架构框图 | §4.6 | 示意稿完成 |
| 图 4.6 | 配置寄存器与 6 层嵌套 FSM 结构图 | cfg_regs and 6-level nested FSM structure | 架构框图 | §4.7 | 示意稿完成 |
| 图 4.7 | DMA 子系统结构图（实现细节） | DMA subsystem detailed structure | 架构框图 | §4.10 | 示意稿完成 |
| 图 4.8 | 多核扩展层结构图 | Multi-core expansion layer structure | 架构框图 | §4.11 | 示意稿完成 |
| 表 1.1 | 代表性 CNN 硬件加速器对比表 | Comparison of representative CNN hardware accelerators | 对照表 | §1.2.1 | 已嵌入正文 |
| 表 5.1 | 功能仿真验证结果汇总 | Functional simulation verification result summary | 结果汇总表 | §5.3 | 已嵌入正文 |
| 表 5.2 | 不同核数配置下的 FPGA 资源占用 | FPGA resource utilization under different N | 资源对比表 | §5.4.1 | 已嵌入正文 / 含 [CHECK] |
| 表 5.3 | 不同核数配置下的功耗估计 | Power estimation under different N | 功耗对比表 | §5.4.3 | 已嵌入正文 / 含 [CHECK] |
| 表 5.4 | ResNet11 整网端到端时延与帧率 | ResNet11 end-to-end latency and frame rate | 性能对比表 | §5.5.1 | 已嵌入正文 / 含 [CHECK] |
| 表 5.5 | *N*=4 配置下 1-DDR vs 4-DDR PoC 流水占空比对比 | Pipeline duty cycle comparison: 1-DDR vs 4-DDR PoC at N=4 | 性能对比表 | §5.5.2 | 已嵌入正文 |
| 表 5.6 | 本工作与代表性 CNN 加速器对比 | Comparison with representative CNN accelerators | 对照表 | §5.6 | 已嵌入正文 / 含 [CHECK] |

## 状态汇总

- **图（Fig）总数**：13 张（§2 × 2，§3 × 3，§4 × 8）
- **表（Tab）总数**：7 张（§1 × 1，§5 × 6）
- **示意稿完成**：13 张图（13/13）
- **数据已固化的表**：表 1.1、表 5.1、表 5.5（4-DDR PoC 对比已实测）、表 5.4（部分实测，部分 [CHECK]）
- **含 [CHECK] 数据待补的表**：表 5.2（*N*=4 综合）、表 5.3（功耗 6 项全部 [CHECK]）、表 5.4（*N*=1 baseline cycles 1 项 [CHECK]）、表 5.6（多列对比数据 [CHECK]）
- **§5 内 [CHECK] 数据计数**：表 5.2 内 4 行 × 4 项 = 16 项（其中 *N*=1/*N*=2 已实测）；表 5.3 内 9 项；表 5.4 内 1 项；表 5.6 内 24 项；合计 §5 表内 [CHECK] = 26 项（不含已实测）
- **双语标题**：每张图均双语，参见各文件首部 H1 标题

## 立项硬约束遵循情况

- FLUX_CNN 是 ASIC 加速器：图 3.1 / 4.x 强调"专用化数据通路"，FPGA 仅作验证标注，未在架构图主标题中突出 FPGA 字样
- 16×16 INT8 阵列 / WRF/ARF/PARF=32 / IFB=8192 / WB=1024 / OFB=2048：在 §3、§4 各结构图中以参数标注形式出现，与 paper.md 一致

## 文件命名规则

- 图：`fig{章号}-{序号}-{短名}.md`，例：`fig4-1-line-buffer.md`
- 表：`tab{章号}-{序号}-{短名}.md`，例：`tab5-2-resource.md`（本批表已嵌入正文，不再单独出 tab*.md，仅在本 INDEX 标记）

## 后续 Polisher 阶段需要核对的事项

1. 表 5.2 / 5.3 / 5.4 / 5.6 中 [CHECK] 项 — 等综合 / 功耗 / 实测数据补齐
2. 图 4.7（DMA 子系统）与图 3.3（DMA 子系统）：两处图标题相同，建议在 Phase 7 把 §3.3 改为"DMA 子系统外部接口图"、§4.10 改为"DMA 子系统内部结构图"以区分粒度
3. 图 3.2 在 §3.3 段 "如图 3.2 所示核内数据通路结构" 引用，但段落只有 "如图 3.2 所示" 后无独立标题行 — Polisher 应补一行 "**图 3.2 加速核内数据通路结构图**"
