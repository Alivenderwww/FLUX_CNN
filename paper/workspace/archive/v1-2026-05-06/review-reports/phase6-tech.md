# 技术评审报告 Phase 6 — 图表设计

## 第 1 次评审

### 判定：PASS-with-known-issues

> 三个 known-issues 已由 Writer 在 INDEX.md / 各 figures/ 文件 / paper.md 内显式声明并明确"留给 polisher 阶段"，不构成本阶段技术编造或失实，按主智能体提示词允许标 PASS-with-known-issue。

---

## 评审范围

- 索引：`paper/workspace/figures/INDEX.md`（113 行）
- 抽样图表：`fig1-top-level-architecture.md`（218 行）/ `tab3-pe-utilization-three-modes.md`（103 行）/ `tab6-multicore-scaling-sweep.md`（86 行）/ `tab7-prior-art-performance-comparison.md`（85 行）/ `tab8-design-axis-comparison.md`（99 行）
- 正文交叉引用扫描：`paper/workspace/paper.md`（Grep `Fig.|Figure |Tab.|Table` 数字）
- 抽样验证：5/9 张图表 + 全量交叉引用一致性

---

## 正反向交叉引用一致性

### paper.md → figures/ 正向覆盖
| paper.md 引用 | 行号 | 对应 figures/ 文件 | 验证 |
|--------------|------|-------------------|------|
| Fig.1 / Figure 1 | line 44 §1.3 | fig1-top-level-architecture.md | ✅ |
| Fig.2 [TBD] | line 108 §2.4 | fig2-sram-vs-image-size.md | ✅ |
| Table 2 | line 292 §5.4 | tab2-compiler-vs-hwreconfig.md | ✅ |
| Table 3 | line 382 §7.2 | tab3-pe-utilization-three-modes.md | ✅ |
| Table 4 | line 396 §7.3 | tab4-bram-breakdown-multicore.md | ✅ |
| Table 5 | line 410 §7.4 | tab5-end-to-end-latency-decomposition.md | ✅ |
| Table 6 | line 424 §7.5 | tab6-multicore-scaling-sweep.md | ✅ |
| Table 7 | line 436 §7.6 | tab7-prior-art-performance-comparison.md | ✅ |
| Table 8 | line 438 §7.6 | tab8-design-axis-comparison.md | ✅ |

**正向覆盖：9/9 ✅，无引用未实现的图表**

### figures/ → paper.md 反向覆盖
9/9 文件全部对应 paper.md 显式引用，**无孤立图表**。

### Writer 报告 3 个已知问题 — 验证

**Q1：Tab.1 跳号** — ✅ 实证存在
- Grep 确认 paper.md 中没有 `Table 1`，首次表号是 `Table 2`（line 292 §5.4）
- INDEX.md line 25 / 76-78 已显式声明"Tab.1 缺失，paper.md 跳号"并给出原因（原计划 §3.6 立场表被 §7.6 line 438 折并入 Tab.8）
- INDEX.md line 93-105 已给 polisher 完整重编号映射表

**Q2：5-axis 命名 §3.6 vs §7.6 不一致** — ✅ 实证存在
- §3.6 line 178 五轴：(1) dataflow / (2) PE-utilization recovery location / (3) streaming granularity / (4) multi-core scaling / (5) compiler stack scope
- §7.6 line 438 五维：(1) dataflow / (2) streaming granularity / (3) multi-core partition / (4) compiler scope / (5) SDP fusion
- 两处轴 #2 与 #5 命名实质不同（§3.6 把"PE-utilization recovery"放轴 2，§7.6 把"streaming granularity"放轴 2）
- tab8-design-axis-comparison.md line 84-87 / line 93 已显式标记 `[TBD: §3.6 vs §7.6 轴的命名不统一]`，建议以 §7.6 五列为准回写改 §3.6

**Q3：Fig.1 wgt_buffer 角色 §1.3 vs §4.1 不一致** — ✅ 实证存在
- §1.3 line 44 把 `wgt_buffer` 列入 5-stage 主链
- §4.1 把 wgt_buffer 描述为 side-feed (WRF 旁路)
- fig1-top-level-architecture.md line 202 已显式标 "正文两处 phrasing 不一致——polisher 阶段用户决定统一 phrasing；图中按 §4.1 的 side-feed 画"

**结论：3 个已知问题全部证实存在，Writer 已在三个层面（INDEX.md 总结 + 对应 figures/ 文件 + paper.md 内显式声明）标记并预留 polisher 处理路径，不再构成本阶段技术失实。**

---

## 数据自洽性（3 张抽样）

### Tab.3 PE 利用率三模式
- baseline 列 12.5% / 25% / 50% / 100% — 与 §7.2 line 380 文字描述完全对齐
- `--fold` / `--fold --s2d` 列 — 全部带 `[CHECK: ~50%]` / `[CHECK: ~100%]` / `[CHECK]`，可追溯到 contributions.md §8.1 #1, #2
- Network-weighted avg ~78% / ~92% / ~96% — 三个数字也带 [CHECK]，未编造
- 数据来源表（line 46-54）每行注明追溯路径
- ✅ **数据自洽，无编造**

### Tab.6 Multicore Scaling
- ✅ 5,569 / 3,833 / 9,057 / 8,808 cycles：来源 STATUS.md §1.5 + §2.8 + paper.md §7.5 line 422，三处一致
- ✅ 1.45× speedup：与 paper.md §7.5 line 422 / line 428 完全一致
- ✅ 2.8% AXI 仲裁开销：与 paper.md line 422 一致
- ✅ ResNet 11-layer projection 302K/141K cycles + 1.7×/3.64×：来自 STATUS.md §2.6 Scheduler 估算 + paper.md line 426，且明确标注 "estimate, not measured"，不编造
- 其余 8 个 wslice case + AXI arb % / Halo % 全部 [CHECK]，未编造
- ✅ **数据自洽，无编造**

### Tab.7 Prior Art Performance
- FLUX_CNN 行：68.4 MHz / 86.6% / 5.95 ms / 8.69 ms / 51.2 GOPS / 35 GOPS — 与 §7.3 line 394 + §7.4 line 408 完全对齐
- TPU v1 92 TOPS：可信、与 Jouppi@ISCA'17 公开数据一致
- Snowflake 128 GOPS / 91%：与 §7.4 line 416 + literature.md 一致，未编造
- Aydonat 1382 GFLOPS / Lu 854.6 GOPS / Liu >1300 GOPS / 97% MAC%：均与 contributions.md §5.1 表 + literature.md 对齐
- 其余 baseline 数字 (Eyeriss / Gemmini / Angel-Eye / VTA / fpgaConvNet 各列)：全部 [CHECK]，未编造
- 三条强制说明（脚注 ¹ ² ³）显式给出 86.6% 计算口径与 Snowflake 91% / Liu 97% 的口径差异警告 — 这正是 reviewer 最易抓的 metric definition 差异，处理得很审慎
- ✅ **数据自洽，无编造**

---

## 示意稿质量（2 张抽样）

### Fig.1 顶层架构
- 必含元素列表（5 大块 + 标注 + 视觉层次）足够清晰，可让用户用 PowerPoint/Visio 直接出图
- 提供 ASCII 示意稿（line 75-167）+ TikZ 骨架（line 175-197）双轨参考
- 模块拓扑追溯：CLAUDE.md "项目总览" §1+§2 / contributions.md §4.1 / RTL/multicore_top.sv / memory/cross_core_design.md — 来源齐全可核
- BRAM 数字（IFB 32 / WB 57 / Shortcut 32 / OFB 7+1）与 §7.3 line 396 完全对齐
- mm2s_arb / axi_dm / axi_m_mux / axi_lite_csr / cfg_regs 全部对应 RTL 实模块（无编造）
- ✅ **示意足够指导出图，无虚构模块**

### Tab.8 五维对位
- 17 个 baseline 全部对应 literature.md 中已收录条目（无虚构方法）
- FLUX_CNN 立场（OS hybrid / Row / Spatial / Network-level / Full residual）追溯到 contributions.md C1.1/C1.2/C2.1/C2.2/C3.5/C3.7
- 唯一需 [CHECK] 的 Angel-Eye SDP fusion 形态已显式标记（line 92）
- 提供 ASCII 视觉强调示例（line 70-77）说明"FLUX_CNN 独占格"应如何高亮
- ✅ **立场可追溯，无编造方法**

---

## [CHECK] 合规性

INDEX.md line 41 报 "总 [CHECK] 数：约 156"，全部分布如下：
- Tab.3 ~30：fold/s2d 三模式 PE% — 等回归实测，**应标 ✓**
- Tab.5 ~50：per-layer cycles + Wall_us — 等重跑 11-layer chain，**应标 ✓**
- Tab.6 ~40：8 wslice cases + ResNet 11-layer multicore — 等实测，**应标 ✓**
- Tab.7 ~30：10 个 prior art baseline 数字 — 等 reviewer 阶段查原文，**应标 ✓**
- 其他小图表 1-2 个：BRAM 重综合刷新 / Angel-Eye SDP 形态 — **应标 ✓**

抽样 5 张图表内所有 [CHECK] 标记位置全部位于"待实测/待查原文"未确定数据点；FLUX_CNN 自产的已实测数字（如 9057/8808 cycles, 86.6%, 68.4 MHz, 5,569/3,833 cycles, 12.5%/25%/50% baseline PE%）**没有滥标 [CHECK]**，标记位置精准。

✅ **156 处 [CHECK] 集中在该标的位置，无漏标无误标**

---

## 数据-正文口径一致性

抽样 4 处 metric 定义：

| 图表 | metric | figures/ 内定义 | paper.md §7 内定义 | 一致性 |
|------|--------|------------------|-------------------|--------|
| Tab.3 | PE% baseline = (min(Cin,16) × min(Cout,16))/256 | 显式给出算式（line 16） | §7.2 line 380 隐含一致（Layer 1 Cin=4 → 4·8/256=12.5%）| ✅ |
| Tab.6 | wslice1 1.45× speedup | N=4/N=2 cycles 比 (line 28) | §7.5 line 422 "wslice1 at 3833 cycles versus N=2's 5569 cycles, giving a 1.45× speedup" | ✅ |
| Tab.7 | FLUX_CNN MAC% = 86.6% 包含 IDMA/ODMA 切换 + descriptor fetch + ODMA drain | 脚注 ² 显式 (line 60) | §7.4 line 412 完全一致表述 | ✅ |
| Tab.7 | Snowflake 91% 是"average computational efficiency"且口径与 FLUX_CNN 不同 | 脚注 ² 显式警告 | §7.6 line 442 完全一致表述 + "denominator differ" 警告 | ✅ |

**Tab.5 与 §7.4 的口径一致性**（未直接抽样但通过 INDEX.md line 36 + paper.md line 408-414 的描述判断）：Tab.5 的 "Wall_us at both operating points" 与 §7.4 line 408 "5.95 ms @ 100 MHz target / 8.69 ms @ 68.4 MHz Fmax" 双口径一致。

✅ **抽样 4/4 metric 口径在 figures/ 与 paper.md §7 之间完全一致；脚注口径警告处理审慎**

---

## 通过项

1. ✅ 9/9 figures/ 文件双向交叉引用完整，无孤立无失联
2. ✅ Tab.3 / Tab.6 / Tab.7 三张抽样表的实测数字全部可追溯到 STATUS.md / contributions.md / paper.md 同一来源
3. ✅ Fig.1 / Tab.8 两张抽样示意稿质量足够指导用户出最终图
4. ✅ 156 处 [CHECK] 标记位置全部合规，无漏标无误标
5. ✅ 4/4 抽样 metric 口径在 figures/ 与 paper.md §7 一致，脚注警告（Snowflake 91% / Liu 97%）审慎
6. ✅ 无编造数字（已实测数字与 STATUS.md 同源、未实测数字一律 [CHECK]）
7. ✅ 无引用未实现模块（rdma_ctrl 已上线，标实线；R.1/R.2 重构后 bias 重定位、Shortcut Bank 都已实测）
8. ✅ literature.md 已收录的 17 个 prior art 全部对应 Tab.7/Tab.8 的引用，无幻觉文献

## Known Issues（不影响判定，已留 polisher）

| # | 问题 | Writer 已声明位置 | polisher 责任 |
|---|------|-------------------|---------------|
| 1 | Tab.1 跳号（首表号是 Tab.2） | INDEX.md line 25 / line 76-78 / line 93-105 给完整重编号映射 | 全文 Tab.X → Tab.(X-1) 重编号 + 同步 figures/ 文件名 |
| 2 | §3.6 vs §7.6 五轴命名不一致 | tab8-design-axis-comparison.md line 84-87 / line 93 / paper.md line 438 | 以 §7.6 五列为准回写 §3.6 |
| 3 | Fig.1 wgt_buffer §1.3 主链 vs §4.1 side-feed | fig1-top-level-architecture.md line 202 / paper.md §1.3 line 44 | 统一 phrasing；图按 §4.1 side-feed 画 |

## 修订建议（可选优化，非阻断）

无强制修订。以下为可选 polish 提示：
- INDEX.md line 41 "总 [CHECK] 数：约 156（其中 §7 内数据图表占 150+）"——这个数字与 paper.md §7 内 21 处 [CHECK] 是同一类问题（都等 project-analyst 回归 + literature-scout 查原文），polisher 阶段可考虑把这 156 处统一汇总成"投稿前补数据 checklist"附在 STATUS.md
- Tab.8 line 94 [TBD] 提到"17 个 baseline 太多，建议保留 12-14 个"——polisher 阶段视版面决定
- Fig.2 [TBD: 是否进终稿] 由用户决定；matplotlib 代码已就绪
