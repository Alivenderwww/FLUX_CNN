# 论文图表索引

本索引列出论文 paper.md 中引用的全部图与表，含双语标题、对应章节与示意稿文件路径。所有示意稿位于 `paper/workspace/figures/` 目录下。

## 图（Figure，13 张）

| 编号 | 双语标题 | 对应章节 | 文件 |
|---|---|---|---|
| 图 2.1 / Figure 2.1 | 卷积运算原理示意图 / Schematic of convolution operation | §2.2 | fig2-1-conv-principle.md |
| 图 2.2 / Figure 2.2 | INT8 量化推理原理 / INT8 quantized inference | §2.5 | fig2-2-quant-inference.md |
| 图 3.1 / Figure 3.1 | 加速器系统总体架构图 / System architecture overview | §3.3 | fig3-1-system-arch.md |
| 图 3.2 / Figure 3.2 | 核内数据通路结构 / Core datapath structure | §3.3 | fig3-2-core-datapath.md |
| 图 3.3 / Figure 3.3 | DMA 子系统结构 / DMA subsystem overview | §3.3 | fig3-3-dma-overview.md |
| 图 4.1 / Figure 4.1 | 行缓存模块结构图 / line_buffer module structure | §4.2 | fig4-1-line-buffer.md |
| 图 4.2 / Figure 4.2 | MAC 阵列模块结构图 / mac_array module structure | §4.3 | fig4-2-mac-array.md |
| 图 4.3 / Figure 4.3 | 部分和累加模块结构图 / parf_accum module structure | §4.4 | fig4-3-parf-accum.md |
| 图 4.4 / Figure 4.4 | SDP 后处理模块结构图 / ofb_writer SDP post-processing | §4.5 | fig4-4-sdp-postproc.md |
| 图 4.5 / Figure 4.5 | 权重缓存模块结构图 / wgt_buffer module structure | §4.6 | fig4-5-wgt-buffer.md |
| 图 4.6 / Figure 4.6 | 配置寄存器与 6 层嵌套 FSM 结构图 / cfg_regs + 6-level nested FSM | §4.7 | fig4-6-cfg-regs-fsm.md |
| 图 4.7 / Figure 4.7 | DMA 子系统结构图 / DMA subsystem detailed | §4.10 | fig4-7-dma-subsystem.md |
| 图 4.8 / Figure 4.8 | 多核扩展层结构图 / multicore_top module structure | §4.11 | fig4-8-multicore-top.md |

## 表（Table，7 张）

| 编号 | 双语标题 | 对应章节 | 文件 |
|---|---|---|---|
| 表 1.1 / Table 1.1 | 代表性 CNN 加速器对比 / Comparison of representative CNN accelerators | §1.2 | tab1-1-prior-art-comparison.md |
| 表 5.1 / Table 5.1 | 功能仿真验证结果汇总 / Summary of functional simulation results | §5.3 | tab5-1-functional-verification.md |
| 表 5.2 / Table 5.2 | 不同核数配置下的 FPGA 资源占用 / FPGA resource utilization under different core-count configurations | §5.4 | tab5-2-resource-utilization.md |
| 表 5.3 / Table 5.3 | 不同核数配置下的功耗估计 / Power estimation under different core-count configurations | §5.4 | tab5-3-power-estimation.md |
| 表 5.4 / Table 5.4 | ResNet11 整网端到端时延与帧率 / End-to-end latency and throughput on ResNet11 | §5.5 | tab5-4-end-to-end-latency.md |
| 表 5.5 / Table 5.5 | *N*=4 配置下 1-DDR vs 4-DDR PoC 流水占空比对比 / Pipeline duty cycle comparison: 1-DDR vs 4-DDR PoC under *N*=4 | §5.5 | tab5-5-ddr-occupancy.md |
| 表 5.6 / Table 5.6 | 本工作与代表性 CNN 加速器对比 / Comparison with representative CNN accelerators | §5.6 | tab5-6-prior-art-compare.md |

## 状态汇总

- 图：13 张，示意稿全部完成；最终绘制工具由用户在 PowerPoint / Visio / Adobe Illustrator / matplotlib 中完成。
- 表：7 张，markdown 表初稿全部完成，可直接嵌入 paper.md。
- [CHECK] 数据待补：表 5.3 功耗（依赖 Vivado power 报告）、表 5.4 *N*=1 未启用空间到深度精确 cycles、表 5.6 文献对比数据（依赖原论文回查）、表 1.1 / 表 5.6 共享部分文献条目。
- [TBD] 设计待定：无（所有图表设计要素已对齐 paper.md 当前版本）。

## 编号映射约定

- 图编号 = 章号.序号（如图 4.1 表示第 4 章第 1 张图）
- 表编号 = 章号.序号（如表 5.3 表示第 5 章第 3 张表）
- 示意稿文件名 = `fig{章}-{序}-{shortname}.md` 或 `tab{章}-{序}-{shortname}.md`
- shortname 用英文短描述以便文件名扁平化排序
