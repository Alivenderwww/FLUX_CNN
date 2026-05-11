# 表 5.9 与已有 FPGA streaming 工作横向对比
# Table 5.9 Comparison against representative FPGA streaming CNN accelerators

## 在论文中的角色
- 首次引入：§5.7 段 "第一条路径是与 FPGA streaming 类工作的同器件横向对比。表 5-9 整理本工作与 fpgaConvNet... 等代表工作在 Fmax / 资源 / 峰值 GOPS / 整网 PE 利用率四维度的横向数据..."
- 论证作用：把本工作放到 FPGA streaming 主线的横向坐标系中，展示"中型器件 + 编译器侧 PE 利用率优化"位置。所有 baseline 数字均回查 literature.md，口径差异在表脚注显式说明。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 工作 | 文献名 + 引用 | literature.md §C |
| 器件 | 实验 FPGA | literature.md |
| Fmax | 实测最高频率 (MHz) | literature.md / 本工作 STATUS.md |
| 资源 (LUT / DSP / BRAM) | 综合占用 | literature.md |
| 峰值 GOPS | 峰值算力 | literature.md |
| 整网 PE 利用率 | 端到端实测 MAC% | literature.md |
| 备注 | 路线 / 数据完整度 / 口径 | 论文表述 |

## 行定义
- 本工作 (单核) — XC7K325T，本工作主线
- 本工作 (N=4 W slice / N=4 SMC) — XC7K325T 多核
- fpgaConvNet [Venieris@FCCM'16 / TNNLS'19] — Zynq XC7Z045
- Snowflake [Bottleson ISCAS'17] — Zynq XC7Z045
- Angel-Eye [Guo TCAD'18] — Zynq
- Caffeine [Zhang ICCAD'16]
- Aydonat Intel DLA — Arria 10
- Lu Winograd — ZCU102
- Ma OPU
- Liu Full-Stack [TNNLS'21] — Arria 10 GX1150（同思路，不同器件规模可比对照）

## 表初稿

| 工作 | 器件 | Fmax | 资源 (LUT / DSP / BRAM) | 峰值 GOPS | 整网 PE 利用率 | 备注 |
|------|------|------|------------------------|----------|--------------|------|
| **本工作 (单核)** | XC7K325T | 68.4 (Fmax) / 100 (target) | 36,942 / 82 / 128 | 51.2 @ target | **86.6%** [CHECK] | 16×16 INT8，编译器侧 Ky-fold + S2D |
| **本工作 (N=4 W 切片)** | XC7K325T | 68 / 100 (target) | 162,584 / 320 / 288 | [CHECK] | [CHECK] | sequential DDR 中转路径 |
| **本工作 (N=4 SMC + NUMA)** | XC7K325T | **100 MHz MET** | 162,584 / 320 / 288 | [CHECK] | [CHECK] | 跨核 SRAM 直送 + 分布式 SRAM |
| fpgaConvNet [Venieris@FCCM'16] | Zynq XC7Z045 | — [CHECK] | — | — | — | 早期 streaming 框架，performance density 提升 2.94× （与 fps/cycles 不直接可比）|
| Snowflake [Bottleson ISCAS'17] | Zynq XC7Z045 | 250 [CHECK] | — | 128 [CHECK] | 91% 平均 [CHECK] | OS 数据流；口径含/不含 IDMA stall 待核 |
| Angel-Eye [Guo TCAD'18] | Zynq | — [CHECK] | — | — [CHECK] | — | 6× 同期工作（VGG16）[CHECK] |
| Caffeine [Zhang ICCAD'16] | — | — | — | — | — | layer-pipelined |
| Aydonat Intel DLA | Arria 10 | — | — | 1382 GFLOPS [CHECK] | — | FP16 / share-exp 路线，与 INT8 不直接可比 |
| Lu Winograd | ZCU102 | — | — | 854.6 [CHECK] | — | Winograd 变换路径，乘法节省但精度漂移 |
| Ma OPU | — | — | — | 645 [CHECK] | — | 通用 OPU 路线 |
| **Liu Full-Stack [TNNLS'21]** | **Arria 10 GX1150** | — | — | > 1.3 TOPS [CHECK] | **97% MAC%** | **同思路不同器件规模可比对照** |

## 表脚注（重要！口径差异显式说明）

> **口径差异**：
> 1. 本工作 86.6% 是 22-case ResNet-18 风格链式回归整网 MAC%（含 IDMA / ODMA stall 与跨层切换）[CHECK 与 model_analysis.md / STATUS.md 交叉核对]。
> 2. Liu 97% 是 Arria 10 GX1150 上 ResNet 系列 streaming 的 MAC efficiency。
> 3. Snowflake 91% 是 Zynq 上的平均计算效率 [CHECK 含/不含 IDMA stall]。
> 4. fpgaConvNet 2.94× 是 performance density 而非 fps/cycles，与本工作不直接可比。
> 5. Aydonat DLA 走 INT16 / FP16 路径 [CHECK]，与本工作 INT8 路径不直接可比。
> 6. Lu Winograd 走 Winograd 变换路径，在精度漂移代价下换乘法节省，与 Direct INT8 路径不直接可比。
> 7. 本工作 N=4 W 切片 354K 与 N=4 SMC 220K 是互斥路径数字 [TBD 主表主键]，本表两行并列。
> 8. 本工作 fps 数字按 100 MHz target 时钟假设给出，实测 Fmax 68.4 MHz 时实际板上 fps ≈ 表中数字 × 0.684（详见 §5.5 末段口径声明）。

## 数据来源
- paper.md §5.7 段表 5-9
- literature.md §C（fpgaConvNet / Snowflake / Angel-Eye / Caffeine / Aydonat / Lu / Ma / Liu Full-Stack 各条目）
- 本工作数据：STATUS.md §1 / §2 / contributions.md C4.x

## 与正文一致性检查
- [x] 表中 baseline 列表（fpgaConvNet / Snowflake / Angel-Eye / Caffeine / Aydonat / Lu / Ma）与 §5.7 第一段一致
- [x] Liu Full-Stack 单独高亮（同思路不同器件规模可比对照） — 与 §5.7 第 5 段一致
- [x] 本工作 86.6% 与 Liu 97% 对位 — 与 §4.3 末段 / §5.7 第 5 段一致
- [x] 口径差异表脚注 — 与 §5.7 第 4 段表脚注承诺一致

## 不确定项
- 大量 baseline 数字 [CHECK]，需要从 literature.md 详查后回填——这是 §5.7 表的根本性 [CHECK]，paper.md §5.7 已用 [CHECK: 表 5-9 各 baseline 同器件归一化口径] 标出
- 本工作 N=4 (W 切片 / SMC) 两行的 GOPS / 整网 PE 利用率 [CHECK]，等用户从 22-case 回归报告复核
- 整网 PE 利用率 86.6% [CHECK]，需要 model_analysis.md 与 STATUS.md 交叉核对取数语义
