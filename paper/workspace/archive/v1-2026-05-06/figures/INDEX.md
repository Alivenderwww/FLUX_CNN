# Phase 6 Figure / Table 索引

> 阶段产出：`paper/workspace/figures/` 下逐张图表的示意稿
> 输入快照：`paper/workspace/paper.md` (§1-§8 全 PASS, ~470 行, commit b158cab 时点)
> 完成时间：2026-04-30

## 全部图表清单

| 编号 | 文件 | 类型 | 标题 | 首次引用位置 | 状态 |
|------|------|------|------|-------------|------|
| **Fig.1** | [fig1-top-level-architecture.md](fig1-top-level-architecture.md) | 架构框图 | FLUX_CNN Top-Level System Architecture | §1.3 line 44, 详述 §4.1 | 示意稿 + ASCII + TikZ 骨架 ✅；最终图 [TBD: 用户用 PowerPoint/Visio 完成] |
| **Fig.2** | [fig2-sram-vs-image-size.md](fig2-sram-vs-image-size.md) | 双对数曲线图 | On-Chip SRAM Footprint vs Image Size | §2.4 line 108 [TBD] 占位 | 示意稿 + matplotlib 完整代码 ✅；**[TBD: 是否进终稿] 用户决定** |
| **Tab.1** *(was Tab.2)* | [tab2-compiler-vs-hwreconfig.md](tab2-compiler-vs-hwreconfig.md) | 4-axis 设计对比 | Compiler-Only Fold vs Hardware-Reconfigurable | §5.4 | 表初稿可直接嵌入 paper.md ✅ |
| **Tab.2** *(was Tab.3)* | [tab3-pe-utilization-three-modes.md](tab3-pe-utilization-three-modes.md) | per-layer PE% 数据表 | PE Utilization Across 3 Compilation Modes | §7.2 | 结构 ✅；**全部具体百分比 [CHECK]** 等实测 |
| **Tab.3** *(was Tab.4)* | [tab4-bram-breakdown-multicore.md](tab4-bram-breakdown-multicore.md) | BRAM 资源分解表 | BRAM Breakdown — Single + Multi-Core Projection | §7.3 | 数字基本 ✅，仅 parf_col Misc [CHECK] |
| **Tab.4** *(was Tab.5)* | [tab5-end-to-end-latency-decomposition.md](tab5-end-to-end-latency-decomposition.md) | 11-layer cycles 分解 | Per-Layer End-to-End Latency Decomposition | §7.4 | 结构 ✅；**全部 per-layer cycles [CHECK]** 等重跑 |
| **Tab.5** *(was Tab.6)* | [tab6-multicore-scaling-sweep.md](tab6-multicore-scaling-sweep.md) | 多核 scaling sweep | Multi-Core Scaling — N=1/2/4 Sweep | §7.5 | 部分 ✅ (wslice1, DDR-mode)；**其他 cases + ResNet 实测 [CHECK]** |
| **Tab.6** *(was Tab.7)* | [tab7-prior-art-performance-comparison.md](tab7-prior-art-performance-comparison.md) | head-to-head 性能对照 | Performance Comparison with Prior Art | §7.6 | 结构 ✅；**baseline 数字大量 [CHECK]** 等 reviewer 阶段查原文 |
| **Tab.7** *(was Tab.8)* | [tab8-design-axis-comparison.md](tab8-design-axis-comparison.md) | 5-axis 设计立场矩阵 | Design-Axis Comparison Along 5 Qualitative Dimensions | §7.6 | 结构 + 立场 ✅；§3.6 / §7.6 5-axis 命名已统一（Phase 7 处理） |

> **Phase 7 polisher 已执行表号重编号**：paper.md 中的 Tab.X 引用已从 Tab.2-Tab.8 重编为 Tab.1-Tab.7（消除 Tab.1 跳号）。本目录下的 figure 文件名沿用旧命名，未做物理重命名（投稿出稿时由用户用 LaTeX `\label{}` / 终稿目录新建即可）。

## 状态汇总

### 数量
- **总图数**：2 (Fig.1, Fig.2)
- **总表数**：7 (Tab.2 到 Tab.8 — **注意：Tab.1 缺失，paper.md 跳号**)
- **示意稿完成**：9/9 ✅

### [CHECK] 标记数（按图表）
| 图表 | [CHECK] 数 | 主要待补内容 |
|------|------------|-------------|
| Fig.1 | 1 | BRAM 数字与 §7.3 同步 |
| Fig.2 | 1 | 1.6 MB ceiling 精确口径 |
| Tab.2 | 1 | Table 编号终稿确认 |
| **Tab.3** | **~30** | **全部 fold/s2d 后 per-layer PE% + network avg — narrative A 核心数据** |
| Tab.4 | 2 | parf_col Misc BRAM + 重综合刷新 |
| **Tab.5** | **~50** | **全部 per-layer cycles + Wall_us — §7.4 重跑数据** |
| **Tab.6** | **~40** | **8 个 wslice cases + ResNet 11-layer multicore 实测** |
| **Tab.7** | **~30** | **10 个 prior art baseline 的整网 MAC% / Fmax / 资源** |
| Tab.8 | 1 | Angel-Eye SDP fusion 形态 |

**总 [CHECK] 数**：约 156（其中 §7 内数据图表占 150+，与 paper.md §7 内 21 处 [CHECK] 标记是同一类问题——都等 project-analyst 跑回归 + literature-scout 查原文补全）。

### [TBD] 标记数（按图表）
| 图表 | [TBD] 数 | 主要决策 |
|------|----------|---------|
| Fig.1 | 3 | 拆图 / R.1 重构对照 / rdma_ctrl 实虚线 |
| **Fig.2** | **2** | **是否进终稿 / 配色 / x 轴单位** |
| Tab.2 | 1 | 是否加 im2col 列 |
| Tab.3 | 2 | 是否配条形图作 Fig.3 / Layer 编号风格 |
| Tab.4 | 1 | (a)(b) 子表是否合并 |
| Tab.5 | 2 | Layer 编号一致性 / Wall_us 双口径列 |
| Tab.6 | 2 | 单/双栏 / ResNet projection 区分 |
| Tab.7 | 2 | "Workload normalized GOPS" 列 / ASIC vs FPGA 拆分 |
| Tab.8 | 3 | §3.6 vs §7.6 轴命名统一 / 行数裁剪 / 视觉化方案 |

**总 [TBD] 数**：18

## 核心交叉引用一致性

### paper.md → figures/ 完整覆盖
| paper.md 提到的编号 | 是否有对应 figures/ 文件 |
|---------------------|--------------------------|
| Fig.1 (§1.3 / §4.1) | ✅ fig1-top-level-architecture.md |
| Fig.2 (§2.4 [TBD]) | ✅ fig2-sram-vs-image-size.md ([TBD] 是否进终稿) |
| Tab.2 (§5.4) | ✅ tab2-compiler-vs-hwreconfig.md |
| Tab.3 (§7.2) | ✅ tab3-pe-utilization-three-modes.md |
| Tab.4 (§7.3) | ✅ tab4-bram-breakdown-multicore.md |
| Tab.5 (§7.4) | ✅ tab5-end-to-end-latency-decomposition.md |
| Tab.6 (§7.5) | ✅ tab6-multicore-scaling-sweep.md |
| Tab.7 (§7.6) | ✅ tab7-prior-art-performance-comparison.md |
| Tab.8 (§7.6) | ✅ tab8-design-axis-comparison.md |

### figures/ → paper.md 反向覆盖
所有 9 个 figures/ 文件都对应 paper.md 中的显式引用 ✅

### **缺号问题：Tab.1 不存在**
- paper.md 跳号：`Tab.2` 是首次出现的表格编号
- **可能原因**：`Tab.1` 原计划放 §3.6 Positioning of FLUX_CNN 的"5 axis 立场表"，但被 paper.md line 438 (§7.6) 的 "fold §3.6's positioning table into Table 8" 决策合并掉了
- **建议**：polisher 阶段把所有 Tab.X 重新编号 `Tab.2 → Tab.1`, `Tab.3 → Tab.2`, ..., `Tab.8 → Tab.7`，避免 reviewer 困惑。本阶段 figures/ 文件名沿用 paper.md 当前编号

## 优先级建议（给 polisher / project-analyst）

按 contributions.md §8.3 修订路径排序：

1. **🔴 最高优先级**：补 Tab.3 全部 [CHECK] — narrative A 核心数据（跑 `run_regression.py --fold` / `--fold --s2d` 取 cycles 反算 PE%）
2. **🔴 最高优先级**：Fig.1 用户用 PowerPoint/Visio 出最终版（架构图反复被读者扫，必须做漂亮）
3. **🟡 中优先级**：补 Tab.5 per-layer cycles（重跑 11-layer chain 取 CASE_PROFILE）
4. **🟡 中优先级**：补 Tab.6 ResNet 11-layer multicore 实测（contributions.md §8.3 排序第 4，1-2 天）
5. **🟢 较低优先级**：Tab.7 reviewer 阶段查 baseline 原文（投稿前两周冲刺）
6. **🟢 较低优先级**：Tab.8 § 3.6/7.6 轴命名统一（polisher 阶段 30 分钟搞定）
7. **🔵 用户决策**：Fig.2 是否进终稿；如进，matplotlib 代码已就绪可直接出 PDF

## 编号建议（polisher 阶段必做）

paper.md 当前 Tab.2-Tab.8 跳过 Tab.1。建议：
```
Tab.2 → Tab.1 (§5.4 compiler vs hwreconfig)
Tab.3 → Tab.2 (§7.2 PE utilization 3 modes)
Tab.4 → Tab.3 (§7.3 BRAM breakdown)
Tab.5 → Tab.4 (§7.4 latency decomposition)
Tab.6 → Tab.5 (§7.5 multicore scaling)
Tab.7 → Tab.6 (§7.6 prior art performance)
Tab.8 → Tab.7 (§7.6 design-axis comparison)
```
重命名也同步本目录文件名 `tab2-...md → tab1-...md` 等。

## 备注

- Phase 6 严格遵守"示意稿"性质：文字描述足够清晰 + 可选 ASCII / matplotlib / TikZ 骨架
- 没有发明新数字 — 所有 [CHECK] 都明确标记数据来源
- 避免冗余图：可用文字说清的不画（如"R.1 重构 bias 重定位前后"对照图——文字一句话足矣，不画）
- 没有为未实现模块画图（如 ResNet 完整 multicore 实测尚未跑——表里标 [CHECK] 而非画"未来架构图"）
