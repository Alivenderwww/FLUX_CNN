# 表 5.6 / Table 5.6 — 本工作与代表性 CNN 加速器对比 / Comparison with Representative CNN Accelerators

## 在论文中的角色

- 首次引入位置：§5.6 与已有工作对比
- 被哪些段引用：§5.6.p2（表本身）；§5.6.p3-p4 围绕本表展开峰值算力定位与功能维度对比；§5.7 本章小结、§6.1 结论间接引用
- 论证作用：将本工作的实测指标与 §1.2 综述中代表性 CNN 加速器（DianNao、TPU v1、Eyeriss、Eyeriss v2、NVDLA、VTA、Gemmini、OPU、Peng et al.）的实测数据进行横向对比，定位本工作在端侧加速器中的峰值算力与能效区间，并明确 FPGA Fmax 与 DSP 推断率两项后续优化方向。

## 列定义

| 列名 | 含义 | 单位 | 数据来源 |
|---|---|---|---|
| 加速器 | 工作名称与文献编号 | — | literature.md |
| 工艺 / 平台 | ASIC 工艺节点或 FPGA 型号 | nm / FPGA | literature.md |
| 频率 | 实测或目标时钟频率 | MHz | literature.md / 本工作综合 |
| 阵列规模 | MAC 阵列形状 | — | literature.md |
| 峰值算力 | 阵列规模 × 2 ops/MAC × Fmax | GOPS | literature.md / 本工作计算 |
| 功耗 | 典型工作负载下整芯片或加速器子模块功耗 | W | literature.md |
| 能效 | 峰值算力 / 功耗 | TOPS/W | literature.md |

## 行定义

| 行 | 数据来源 | 置信度 |
|---|---|---|
| DianNao [11] | literature.md | 频率与阵列规模已确认；GOPS / 功耗 / 能效待回查原论文 |
| TPU v1 [12] | literature.md | 频率与阵列规模已确认；GOPS 92,000 待回查；功耗、能效待回查 |
| Eyeriss [13] | literature.md | 频率与阵列规模已确认；其余待回查 |
| Eyeriss v2 [14] | literature.md | 频率与平台已确认；其余待回查 |
| NVDLA [18] | literature.md | 频率与平台已确认；其余待回查 |
| VTA [27] | literature.md L441（频率 100 MHz） | 频率已确认；阵列、GOPS 待回查；VTA 为 FPGA 框架，功耗与能效不直接对照 |
| Gemmini [19] | literature.md L151（能效 0.106 TOPS/W = 106.1 GOPS/W） | 能效已确认；其余待回查 |
| OPU [20] | literature.md | 频率与平台已确认；其余待回查 |
| Peng et al. [24] | literature.md | 频率与平台已确认；其余待回查 |
| 本工作（100 MHz 假设） | 本工作综合 + 计算 | 阵列、频率、峰值算力已确认；功耗与能效待 Vivado power 报告 |
| 本工作（68.4 MHz 实测） | 本工作综合 + 计算 | 阵列、频率、峰值算力已确认；功耗与能效待 Vivado power 报告 |

## 表初稿（Markdown，可直接嵌入 paper.md）

表 5.6 本工作与代表性 CNN 加速器对比

| 加速器 | 工艺 / 平台 | 频率（MHz） | 阵列规模 | 峰值算力（GOPS） | 功耗（W） | 能效（TOPS/W） |
|---|---|---|---|---|---|---|
| DianNao [11] | 65 nm | 980 | 16×16 NFU | 452 [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| TPU v1 [12] | 28 nm | 700 | 256×256 | 92,000 [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| Eyeriss [13] | 65 nm | 200 | 14×12 PE | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| Eyeriss v2 [14] | 65 nm | 200 | mesh | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| NVDLA [18] | 16 nm | 1000 | 可配置 | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| VTA [27] | ZCU102 FPGA | 100 | 可配置 | [CHECK: 数据待回查原论文] | — | — |
| Gemmini [19] | 22 nm | 1000 | 16×16 | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | 0.106 |
| OPU [20] | XC7Z045 FPGA | 200 | 可配置 | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| Peng et al. [24] | Arria 10 FPGA | 1000 | 全栈 | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] | [CHECK: 数据待回查原论文] |
| 本工作（100 MHz 假设） | XC7K325T FPGA | 100 | 16×16 | 51.2 | [CHECK] | [CHECK] |
| 本工作（68.4 MHz 实测） | XC7K325T FPGA | 68 | 16×16 | 35.0 | [CHECK] | [CHECK] |

取数原则：

- 实测峰值算力与能效比直接采用各工作原文公布数据；本工作的峰值算力按 16×16 阵列 × 2 ops/MAC × Fmax 计算，分别给出 100 MHz 假设值（51.2 GOPS）与 68.4 MHz 实测值（35.0 GOPS）。
- 不同工作所采用的工艺节点、时钟频率、目标网络与功耗包线差异显著，本表对照仅作为定位参考。

定位结论：

- 本工作峰值算力处于端侧加速器中下区间，原因有三：MAC 阵列规模 16×16 与 Eyeriss、Gemmini 同量级；FPGA 综合 Fmax 仅 68.4 MHz；DSP 推断率低导致部分 MAC 实现于 LUT。
- 后续优化方向：在 SDP 量化链插入 1 至 2 级流水寄存器（估计 Fmax → 100 MHz 以上），在 mac_pe 上加 `(* use_dsp = "yes" *)` 综合属性约束（估计降低约 17,000 LUT 用量）。

## 不确定项

- [CHECK] DianNao / TPU v1 / Eyeriss / Eyeriss v2 / NVDLA / VTA / Gemmini / OPU / Peng et al. 的峰值算力（GOPS）、功耗（W）、能效（TOPS/W）均待回查原论文确认；目前仅 VTA 频率（100 MHz）与 Gemmini 能效（0.106 TOPS/W）来自 literature.md 已锚定来源。
- [CHECK] 本工作功耗（W）与能效（TOPS/W）：依赖表 5.3 Vivado power 报告补齐后回填。

## image 生成提示词

无（表格直接以 markdown 形式嵌入论文）。
