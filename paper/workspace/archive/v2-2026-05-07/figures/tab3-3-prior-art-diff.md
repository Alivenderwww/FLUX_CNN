# 表 3.3 三处近邻 prior art 差异化对比
# Table 3.3 Differentiation summary against three nearest prior arts

## 在论文中的角色
- 首次引入：§3.4 段 "行级行环与 Liu Full-Stack Streaming CNN Accelerator（TNNLS'21）的差异在于并行维度的取舍..." [依赖: Tab.3.3]
- 论证作用：把 §3.4.3 与 Alwani / Kang / Liu 三处近邻 prior art 的差异化主张以单表收束，是全文最关键差异化论证章的总结。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 工作 | Alwani / Kang / Liu / 本工作 | literature.md §C |
| 缓冲粒度 | 整图 / 整层 / strip 几行 | 论文表述 |
| 输入分辨率上限 | 是否受 BRAM 容量限制 | 论文表述 |
| 并行维度 | 跨层 / 单层内 W / 其他 | 论文表述 |
| 多层调度 | layer-pipelined / layer-serial / fused | 论文表述 |
| DDR 流量代价 | off-chip FM 减少策略 | literature.md |
| 器件 | 实验器件 | literature.md |

## 行定义
- Alwani Fused-layer (MICRO'16)
- Kang AoCStream (FCCM'23 / Sensors'23)
- Liu Full-Stack (TNNLS'21)
- **本工作 (FLUX_CNN)**

## 表初稿

| 工作 | 缓冲粒度 | 分辨率上限 | 并行维度 | 多层调度 | DDR 流量代价 | 器件 |
|------|--------|----------|---------|---------|------------|------|
| **Alwani Fused-layer** [MICRO'16] | 多层中间 FM 同时驻留 | 受 BRAM 严格限制 | 跨层 fused | layer-fused | VGGNet-E 前 5 层 77 MB → 3.6 MB（-95%）| ASIC |
| **Kang AoCStream** [FCCM'23] | 整层 FM 驻留 SRAM | line buffer ∝ 图宽 → 受 BRAM 严格限制 | 跨层 pipelined（每层独立 dataflow block） | layer-pipelined | 不依赖外存（全片上） | low-end FPGA |
| **Liu Full-Stack** [TNNLS'21] | 跨层 pipelined | 大型器件资源宽裕 | 跨层（多核同时跑不同层） | layer-pipelined | 多级并行 + DSP 充分利用 | Arria 10 GX1150 |
| **本工作 FLUX_CNN** | **strip_rows × W_IN 行级 ring** | **解耦（任意 H × W）** | **单层内 W 维切片** | **layer-serial 共用硬件** | **DDR 中转 + M2 push（跨核 SRAM 直送）** | **XC7K325T 中型** |

## 与正文一致性检查
- [x] Alwani 多层 fused 缓冲 / 77 MB→3.6 MB — 与 §1.2.3 / §3.4.3 一致
- [x] Kang 整层片上 SRAM / line buffer ∝ 图宽 — 与 §1.2.3 / §3.4.3 一致
- [x] Liu 跨层 pipelined / Arria 10 — 与 §1.2.3 / §3.4.3 一致
- [x] 本工作 strip 行级 ring / 单层内 W 切片 / layer-serial 共用硬件 — 与 §3.4.3 / §4.5 一致

## 不确定项
- §3.4.3 各对位段 [CHECK-3.4.3] 措辞最终版定后回查表中"DDR 流量代价"列是否需要更精细数字
