# 表 1.1 代表性 CNN 硬件加速器对比
# Table 1.1 Representative CNN Hardware Accelerators Comparison

## 在论文中的角色

- 首次引入位置：§1.2（研究现状）
- 被哪些段引用：§1.2 末段（line 60 前后）
- 论证作用：横向对比代表性 CNN 加速器的阵列规模、工作频率、峰值算力、功耗与能效，为本文的 ASIC 立项提供学术参照系，并展示当前主流工作点（端侧 50–500 GOPS、1–3 W；服务器端 TPU 量级）。

## 列定义

| 列名 | 含义 | 单位 | 数据来源 |
|------|------|------|----------|
| Accelerator | 加速器名称 | - | 文献 |
| Year | 发表年份 | - | 文献 |
| Platform | 实现平台（工艺节点或 FPGA 型号） | nm / 板卡 | 文献 |
| Freq (MHz) | 工作频率 | MHz | 文献 |
| Peak Perf (GOPS) | 峰值算力 | GOPS | 文献 |
| Power (mW) | 整体功耗 | mW | 文献 |
| Efficiency (TOPS/W) | 能效比 | TOPS/W | 文献（部分由 GOPS/Power 折算） |
| Ref | 文献编号 | - | 本论文参考文献 |

## 行定义

| 行 | 数据来源 | 置信度 |
|----|----------|--------|
| DianNao (2014, 65 nm) | [11] | 直接引文献 |
| TPU v1 (2017, 28 nm) | [12] | 直接引文献 |
| Eyeriss (2017, 65 nm) | [13] | 直接引文献 |
| NVDLA* (2018, 16 nm) | [18] | 直接引文献，NVDLA 配置参数较多，已标 * 表示典型小型配置 |
| Eyeriss v2 (2019, 65 nm) | [14] | 直接引文献 |
| VTA (2019, ZCU102) | [27] | 频率 100 MHz 锚定 literature.md L441；其余指标待回查；FPGA 平台无 Power 数据 |
| 后续行（如需扩展） | literature.md | 可补 SCNN / DeepFire2 / OPU 等 |

## 表初稿（Markdown 表）

| Accelerator | Year | Platform | Freq (MHz) | Peak Perf (GOPS) | Power (mW) | Efficiency (TOPS/W) | Ref |
|---|---|---|---|---|---|---|---|
| DianNao | 2014 | 65 nm | 980 | 452 [CHECK: 数据待回查原论文] | 485 [CHECK: 数据待回查原论文] | 0.92 [CHECK: 数据待回查原论文] | [11] |
| TPU v1 | 2017 | 28 nm | 700 | 92000 [CHECK: 数据待回查原论文] | 40000 [CHECK: 数据待回查原论文] | 2.30 [CHECK: 数据待回查原论文] | [12] |
| Eyeriss | 2017 | 65 nm | 200 | 168 [CHECK: 数据待回查原论文] | 278 [CHECK: 数据待回查原论文] | 0.12 [CHECK: 数据待回查原论文] | [13] |
| NVDLA* | 2018 | 16 nm | 1000 | 128 [CHECK: 数据待回查原论文] | ~300 [CHECK: 典型小型配置估值] | ~0.40 [CHECK: 典型小型配置估值] | [18] |
| Eyeriss v2 | 2019 | 65 nm | 200 | 153.6 [CHECK: 数据待回查原论文] | 606 [CHECK: 数据待回查原论文] | 0.96 [CHECK: 数据待回查原论文] | [14] |
| VTA | 2019 | ZCU102 | 100 | 170 [CHECK: 数据待回查原论文] | - | - | [27] |

注：
- NVDLA 行 `*` 表示典型小型配置；Power 与 Efficiency 列的 `~` 标记表示典型小型配置估值，置信度低于 ASIC 实测数据。
- VTA 频率 100 MHz 来源于 literature.md L441（与表 5.6 对齐）；在 ZCU102 FPGA 上未给出整体 Power 与 Efficiency，留 `-`。
- Eyeriss 的 Efficiency 0.12 TOPS/W 为 168 GOPS / 278 mW × 0.001 计算（与文献一致）。
- 本表与表 5.6 共享文献条目，所有 [CHECK: 数据待回查原论文] 标注与表 5.6 一致；待 Phase 7 数据回填后两表同步去 [CHECK]。

## 不确定项

- [CHECK] FLUX_CNN 自身行是否要并入此表？当前未列入，因本表定位为"研究现状"对比，FLUX_CNN 性能数据集中在第 5/6 章；如需放入，建议放在 §6 综合对比表中。
- [CHECK] DianNao / TPU v1 / Eyeriss / Eyeriss v2 / NVDLA / VTA 的 Peak Perf / Power / Efficiency 均待回查原论文确认（与表 5.6 同等条目同步标注）；目前仅 VTA 频率（100 MHz）来自 literature.md L441 已锚定来源。
- [CHECK] NVDLA Power `~300 mW` 与 Efficiency `~0.40 TOPS/W` 为典型小型配置估值，文献 [18] 实际给出多种配置参数；表注已显式说明置信度。
- [TBD] 是否要补充 SCNN [15] / OPU [20] / DNNBuilder [21] / DeepFire2 [22] 行以扩充对比维度？目前正文已涉及但表中未列。

## image 生成提示词

无（表格直接以 markdown 形式嵌入论文）。
