# Tab.8: Design-Axis Comparison Along 5 Qualitative Dimensions

## 在 paper.md 中的引用位置
- **首次引入**：§7.6 Comparison with Prior Art, line 438
  - 原文：`Table 8 is the design-axis comparison along five qualitative dimensions: dataflow (weight-stationary / output-stationary / row-stationary / hybrid), streaming granularity (batch / row / pixel), multi-core partition axis (channel / spatial / none), compiler scope (op-only / network-level), and SDP fusion (none / partial / full residual).`
- **被引用次数**：1 次显式

## 论证作用
**定位 FLUX_CNN 在 5 维设计空间里"独占的格"**。区别于 Tab.7 的数字比较，Tab.8 是定性立场比较，让读者直接看到 FLUX_CNN 的 niche：**row-ring streaming + W-slice multi-core + compile-time Ky-fold/S2D + full SDP fusion**。

也是 §3.6 五轴定位的"图表化复述"——paper.md line 438 已声明 "we will fold §3.6's positioning table into Table 8 to avoid duplicate prose."

## 列定义
| 列名 | 含义 |
|------|------|
| Method | 方法名 + 引用 |
| Dataflow | WS / OS / RS / hybrid |
| Streaming Granularity | batch / row / pixel |
| Multi-core Partition Axis | channel / spatial / none |
| Compiler Scope | op-only / network-level |
| SDP Fusion | none / partial / full residual |

## 表初稿

| Method | Dataflow | Streaming Granularity | Multi-core Partition | Compiler Scope | SDP Fusion |
|--------|----------|------------------------|----------------------|----------------|------------|
| **FLUX_CNN (this work)** | **OS w/ column-broadcast (hybrid)** | **Row-level** (ring buffer) | **Spatial (W-slice)** w/ halo | **Network-level** (Ky-fold + S2D + chain) | **Full** (bias + shortcut residual + quant) |
| TPU v1 [Jouppi@ISCA'17] | WS | Batch | None (single chip) | Op-only (XLA frontend) | Partial (bias + activation) |
| Eyeriss [Chen@ISSCC'16, JSSC'17] | RS | Pixel (row-stationary) | None | Op-only | Partial |
| Eyeriss-v2 [Chen@JETCAS'19] | RS w/ reconfig NoC | Pixel | None | Op-only | Partial |
| Gemmini [Genc@DAC'21] | WS / OS configurable | Batch (tile-and-reload) | Multi-instance | Op-only (RoCC ISA) | Partial |
| NVDLA [NVDLA] | OS w/ central FSM | Batch | Multi-instance (CDP/PDP/SDP independent) | Op-only (compiler frontend external) | **Full residual** |
| Snowflake [Gokhale@ISCAS'17] | OS (vector engine) | Row (line-buffer) | None | Op-only | Partial |
| Angel-Eye [Guo@TCAD'18] | OS w/ custom ISA | Pixel (line-buffer) | None | Op-level (Caffe→hardware) | [CHECK: line 438 paper.md "Angel-Eye SDP fusion 具体形态待查"] |
| Aydonat DLA [Aydonat@FPGA'17] | Winograd 1-D streaming | Row | None | Op-only | Partial |
| Lu Winograd [Lu@FCCM'17] | Winograd | Row | None | Op-only | Partial |
| Liu Full-Stack [Liu@TNNLS'21] | OS streaming | **Row** (layer-fused) | **Spatial** (different layout) | **Network-level** (full-stack) | **Full residual** |
| Alwani Fused-layer [Alwani@MICRO'16] | (per-layer module) | **Layer** (fusion across multiple) | None | Op-level | None |
| Kang AoCStream [Kang@Sensors'23] | (per-layer module) | **Row** (line-buffer per layer) | None (layer-pipelined) | Op-only | Partial |
| MAERI [Kwon@ASPLOS'18] | Reconfigurable (any) | Batch | None | Op-only (search-based mapping) | Partial |
| VTA [Moreau@ASPLOS'19] | OS / WS configurable | Batch | None | **Network-level** (TVM stack) | Partial |
| fpgaConvNet [Venieris@TNNLS'19] | per-layer streaming module | Row | None | **Network-level** (HLS template generator) | Partial |
| Simba [Shao@MICRO'19] | OS multi-chiplet | Batch | **Channel-slice** (36 chiplets) | Op-only | Partial |

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| FLUX_CNN 全部立场 | contributions.md C1.1 / C1.2 / C2.1 / C2.2 / C3.5 / C3.7 / paper.md §3.6 / §7.6 | ✅ |
| TPU v1 WS / single-chip | literature.md (TPU-ISCA17) | ✅ |
| Eyeriss RS | literature.md (Eyeriss-ISCA16) | ✅ |
| Gemmini WS/OS configurable / RoCC | literature.md (Gemmini-DAC21) | ✅ |
| NVDLA OS / SDP full residual | literature.md (NVDLA) | ✅ |
| Snowflake OS line-buffer | literature.md (Snowflake-ISCAS17) | ✅ |
| Angel-Eye OS custom ISA | literature.md (Angel-Eye-TCAD18) | ✅ control style; [CHECK: SDP fusion 具体形态 — paper.md line 438] |
| Aydonat / Lu Winograd | literature.md | ✅ |
| Liu Full-Stack OS streaming + residual + spatial partition | literature.md (Liu-FullStack-TNNLS21) | ✅ |
| Alwani layer-fusion | literature.md (Alwani-MICRO16) | ✅ |
| Kang AoCStream layer-pipelined | literature.md (Kang-Sensors23) | ✅ |
| MAERI ART reconfigurable | literature.md (MAERI-ASPLOS18) | ✅ |
| VTA TVM stack | literature.md (VTA-MICRO19) | ✅ |
| fpgaConvNet HLS network-level | literature.md (fpgaConvNet-TNNLS19) | ✅ |
| Simba channel-slice 36 chiplets | literature.md (Simba-MICRO19) | ✅ |

## 视觉强调建议

让 FLUX_CNN 行的"独占格"（**Row-level streaming + Spatial multi-core + Network-level compiler + Full SDP fusion** 四个特性同时具备）通过加粗、彩色高亮或背景色强调出来：

```
┌─────────────────┬──────────┬──────────┬─────────┬──────────────┬──────────────┐
│  FLUX_CNN       │  OS hyb  │ ★Row ★  │ ★Spatial│ ★Network-lv │ ★Full resid  │ ← 4 ★ unique combo
├─────────────────┼──────────┼──────────┼─────────┼──────────────┼──────────────┤
│  TPU v1         │   WS     │   Batch  │  None   │   Op-only    │   Partial    │
│  Eyeriss        │   RS     │   Pixel  │  None   │   Op-only    │   Partial    │
│  ...            │   ...    │   ...    │  ...    │   ...        │   ...        │
└─────────────────┴──────────┴──────────┴─────────┴──────────────┴──────────────┘
```

唯一同时具备 "Row" 与 "Spatial" 与 "Network-level" 三个 ★ 的对照只有 **Liu Full-Stack [TNNLS'21]**——但 Liu 不区分 streaming granularity 与 layer-fused 的差别，paper.md §3.4 line 162 已论证 "same direction, different scale point"。

## 与正文的一致性检查

- [x] §7.6 line 438 5 维 axis 完全一致：dataflow / streaming granularity / multi-core partition / compiler scope / SDP fusion
- [x] §3.6 line 178 五轴定位框架（dataflow / PE-utilization recovery / streaming granularity / multi-core scaling / compiler stack scope）— **注意** §3.6 5 轴与 §7.6 5 维 不完全一一对应
  - §3.6 第 2 轴 "PE-utilization recovery 位置" 在 §7.6 改为 "Compiler scope"
  - §3.6 第 5 轴 "compiler stack scope" 在 §7.6 改为 "SDP fusion"
  - paper.md line 438 已说明 "fold §3.6's positioning table into Table 8" — **建议 polisher 阶段统一**两处轴的命名
- [x] §3.4 line 158-162 Alwani / Kang / Liu 三个 prior art 在 streaming granularity 列分别 "Layer" / "Row (per-layer block)" / "Row (layer-fused)" — 表内立场对齐

## 不确定项

- [CHECK: contributions.md §8.1 #12 / paper.md line 438] Angel-Eye SDP fusion 具体形态——查 Guo@TCAD'18 看是否含 shortcut residual / 仅 bias+ReLU
- [TBD: §3.6 vs §7.6 轴的命名不统一] paper.md 当前两处 5-axis 定义有偏差。**polisher 阶段必须统一**——建议以 §7.6 的 5 列为准，回写改 §3.6
- [TBD: 是否合并某些行] 17 个 baseline 太多；建议保留 12-14 个最相关的（去掉 Eyeriss 与 Eyeriss-v2 重复 / Aydonat 与 Lu Winograd 选 1）
- [TBD: 视觉化方案] 是否用 ✓/—/✗ 等符号代替文字，让"独占格"更醒目；权衡可读性 vs 信息量

## 备注

这张表与 Tab.7 是姐妹关系：Tab.7 是数字 head-to-head，Tab.8 是设计立场矩阵。**两表互补不互替**。如果版面紧张，**Tab.8 比 Tab.7 更不可或缺**——Tab.7 reviewer 会接受详细数字分散到 §7.3-§7.5 文字描述里，Tab.8 的 "FLUX_CNN unique cell" 立场必须可视化。
