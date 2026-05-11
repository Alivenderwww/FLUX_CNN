# v4 §5 验证章 — 参数扫频实验需求清单

> 本文档为 paper.md §5 验证与性能分析章 "参数影响扫频实验" 设计稿。
> 用户跑完实验后，由 §5 写作 agent 把数据回写至 §5.5 后新增子节（建议编号 §5.5.5 参数敏感度分析）。
> 本文档不进 paper.md，仅作为实验执行 + 数据采集对照表使用。
> 日期：2026-05-09，对应 commit `e8ca7b0`（H 步长分离 baseline）+ Round M（DSP SIMD 实现）。

---

## 1 实验设计原则

### 1.1 实验目的

§5.5 现有四个子节（端到端时延 / PE 利用率 / 多核加速比 / 调度策略量化）给的是 **ResNet11 整网单点实测**，回答的是"在固定网络上，FLUX_CNN 能跑多快、瓶颈在哪"。
本节扫频实验补的是 **参数空间敏感度**，回答的是"当输入工作负载在 *C*ᵢₙ / *C*ₒᵤₜ / *K* / stride / *W*ᵢₙ / *N* 维度变化时，PE 利用率 / wall cycles / 加速比如何随之变化，编译器侧的 Ky 折叠与空间到深度在哪个参数区间的收益最大"。

### 1.2 与 §5.5 现有内容的边界

| §5.5 现有 | 本节扫频补充 |
|---|---|
| §5.5.1 ResNet11 整网单点 wall cycles + 帧率（5 个配置） | 单层 wall cycles 随单一参数变化的曲线 |
| §5.5.2 ResNet11 各层 PE 利用率分布（11 层离散值） | PE 利用率随 *C*ᵢₙ / stride / *K* 连续变化的趋势 |
| §5.5.3 *N*=1/2/4 多核加速比单点（含 / 不含跨核直送） | 不同 *W*ᵢₙ 下 *N*=1/2/4 加速比曲线 |
| §5.5.4 控制变量法替换调度组件（4 类） | 不同负载形状下 IDMA stall 占比的演化 |

### 1.3 输出形式（每个实验）

- 1 张定量表（典型数据点，列 5–8 行）
- 1 张折线图或柱状图（横轴扫描参数，纵轴 1–2 个因变量）
- 1–2 段定量分析（描述趋势 + 解释拐点 + 与论文论点对接）

---

## 2 实验列表（共 5 个核心扫频实验）

### 实验 A：*C*ᵢₙ 扫频与 Ky 折叠适用域

| 项 | 内容 |
|---|---|
| 编号 | 实验 A |
| 名称 | 不同 *C*ᵢₙ 下 Ky 折叠对 PE 利用率的影响 |
| 自变量 | *C*ᵢₙ ∈ {1, 2, 4, 8, 16, 32, 64}（7 点） |
| 固定参数 | *K*=3，stride=1，pad=1，*H*ᵢₙ=*W*ᵢₙ=32，*C*ₒᵤₜ=16 |
| 对照 | 每个 *C*ᵢₙ 跑两次：未启用 Ky 折叠 / 启用 Ky 折叠（`--ky-fold`） |
| 因变量 | PE 利用率（mac_util）、wall cycles、mac_array fire 数 |
| 期望图 | 折线图：横轴 *C*ᵢₙ（对数刻度），纵轴 PE 利用率（%），两条曲线（折叠前 / 折叠后） |
| 期望表 | 7 行 × 5 列（*C*ᵢₙ / fire 数 / wall_cy 折叠前 / wall_cy 折叠后 / 加速比） |
| 论文意义 | 量化 Ky 折叠在 *C*ᵢₙ < 16 区间的收益曲线，回答 "Ky 折叠是否对 *C*ᵢₙ=8 / 4 / 2 都有效"，对接 §5.5.2 中 Patch 层 5.05× 单点结论 |

**复现命令**（以 *C*ᵢₙ=4 为例，其它 *C*ᵢₙ 值平移）：
```bash
cd toolchain
# 折叠前
python gen_isa_test.py --k 3 --h_in 32 --w_in 32 --num_cin 4 --num_cout 16 --pad 1 --case-name fold_off_cin4
# 折叠后
python gen_isa_test.py --k 3 --h_in 32 --w_in 32 --num_cin 4 --num_cout 16 --pad 1 --ky-fold --case-name fold_on_cin4
# 仿真
cd ../sim/tb_core_dma && vsim -c -do run.tcl
# 数据来源：CASE_RESULT 行的 wall_cy + mac_fire + mac_util 字段
```

每点仿真预算约 1–2 分钟（*C*ᵢₙ=64 时 IFM 数据量较大，约 5 分钟）。

---

### 实验 B：stride 扫频与降采样层调度对照

| 项 | 内容 |
|---|---|
| 编号 | 实验 B |
| 名称 | 不同 stride 下降采样层 wall cycles 与 H 步长分离收益 |
| 自变量 | stride ∈ {1, 2, 3, 4}（4 点） |
| 固定参数 | *K*=1，pad=0，*H*ᵢₙ=*W*ᵢₙ=64，*C*ᵢₙ=*C*ₒᵤₜ=16 |
| 对照 | 每个 stride 跑两次：默认（原 H 步长 = stride 行间拉数据）/ 启用 H 步长分离（`STRIDE_H` 写 1，driver 自动） |
| 因变量 | wall cycles、IDMA 实际数据量（cmd 数）、PE 利用率 |
| 期望图 | 双柱状图：横轴 stride，纵轴 wall cycles，两组柱（H 步长分离前 / 后）+ 副轴 PE 利用率折线 |
| 期望表 | 4 行 × 5 列（stride / IDMA cmd 数前 / IDMA cmd 数后 / wall_cy 前 / wall_cy 后） |
| 论文意义 | 量化 §5.5.2 中"降采样层访存量与计算 fire 数失配"在 stride 维度的演化，对接 H 步长分离整网 -6.5% 的层级根因（实验 6/Round I 数据） |

**复现命令**：
```bash
cd toolchain
# stride=2 默认
python gen_isa_test.py --k 1 --h_in 64 --w_in 64 --num_cin 16 --num_cout 16 --stride 2 --pad 0 --case-name ds_s2_default
# stride=2 + H 步长分离
FLUX_STRIDE_H=1 python gen_isa_test.py --k 1 --h_in 64 --w_in 64 --num_cin 16 --num_cout 16 --stride 2 --pad 0 --case-name ds_s2_hsep
cd ../sim/tb_core_dma && vsim -c -do run.tcl
```

每点仿真预算约 30 秒。stride=3 仅作完备性，ResNet11 主线无 stride=3 层；可作"扫描覆盖"使用。

---

### 实验 C：*K* 扫频与卷积核 round chunking 开销

| 项 | 内容 |
|---|---|
| 编号 | 实验 C |
| 名称 | 不同卷积核大小 *K* 下 mac_array fire 数与 wall cycles 的关系 |
| 自变量 | *K* ∈ {1, 3, 5, 7, 8}（5 点） |
| 固定参数 | stride=1（*K*=8 例外取 stride=4 + 空间到深度做 Patch 等价对照），*H*ᵢₙ=*W*ᵢₙ=32，*C*ᵢₙ=16，*C*ₒᵤₜ=16 |
| 对照 | *K*=8 case 额外跑一次 `--s2d`，与 *K*=1 stride=1 *C*ᵢₙ=64 等价对照 |
| 因变量 | wall cycles、fire 数、PE 利用率、轮次（round）数 |
| 期望图 | 折线图：横轴 *K*²（即每空间位置 fire 数 1/9/25/49/64），纵轴 wall cycles |
| 期望表 | 5 行 × 4 列（*K* / fire 总数 / wall_cy / 平均每 fire 周期数） |
| 论文意义 | 验证 16×16 阵列对 *K*² ≤ 256 输入维度的覆盖能力，量化大核（*K*=7/8）下的 round chunking（轮次切片）开销，对接 §3.4 阵列宽度选型论证 |

**复现命令**：
```bash
cd toolchain
python gen_isa_test.py --k 1 --h_in 32 --w_in 32 --num_cin 16 --num_cout 16 --pad 0 --case-name k1
python gen_isa_test.py --k 3 --h_in 32 --w_in 32 --num_cin 16 --num_cout 16 --pad 1 --case-name k3
python gen_isa_test.py --k 5 --h_in 32 --w_in 32 --num_cin 16 --num_cout 16 --pad 2 --case-name k5
python gen_isa_test.py --k 7 --h_in 32 --w_in 32 --num_cin 16 --num_cout 16 --pad 3 --case-name k7
python gen_isa_test.py --k 8 --h_in 32 --w_in 32 --num_cin 16 --num_cout 16 --stride 4 --pad 0 --s2d --case-name k8_s2d
cd ../sim/tb_core_dma && vsim -c -do run.tcl
```

每点 30 秒至 1 分钟。

---

### 实验 D：核数 *N* 扫频与 *W*ᵢₙ 切片粒度

| 项 | 内容 |
|---|---|
| 编号 | 实验 D |
| 名称 | 不同 *W*ᵢₙ 下 *N*=1/2/4 多核扩展加速比 |
| 自变量 | *N* ∈ {1, 2, 4}（3 点）× *W*ᵢₙ ∈ {16, 32, 64, 128}（4 点）= 12 case |
| 固定参数 | *K*=3，stride=1，pad=1，*H*ᵢₙ=*W*ᵢₙ（方阵），*C*ᵢₙ=*C*ₒᵤₜ=16 |
| 对照 | 每个组合开启跨核 SRAM 直送（默认 ON） |
| 因变量 | wall cycles、相对 *N*=1 加速比、流水占空比（mac_pipe%） |
| 期望图 | 折线图：横轴 *W*ᵢₙ（对数刻度），纵轴加速比（×），三条曲线（*N*=1=1.00× 基线 / *N*=2 / *N*=4） |
| 期望表 | 12 行 × 5 列（*N* / *W*ᵢₙ / wall_cy / 加速比 / mac_pipe%） |
| 论文意义 | 量化 §5.5.3 单点结论"*N*=4 加速 1.68×–3.13×"在不同 *W*ᵢₙ 下的演化曲线，定位"哪个 *W*ᵢₙ 区间多核扩展接近线性，哪个区间被晕区开销与 DDR 带宽限制压回"，对接 §5.5.2 中 1-DDR 带宽是 *N*=4 加速比偏离线性的论点 |

**复现命令**（以 *W*ᵢₙ=64、*N*=4 为例，其它平移）：
```bash
cd toolchain
python run_multicore_chain.py --smc --case_name sweep_n4_w64 --n_cores 4 \
    --layers '[{"K":3,"stride":1,"H_in":64,"W_in":64,"C_in":16,"C_out":16,"pad":1}]'
cd ../sim/tb_smc && vsim -c -do run.tcl
# 数据从 CASE_PROFILE 行的 wall_cy + mac_pipe 字段读取
```

每点仿真预算约 2–3 分钟，全 12 case 约 30–40 分钟。

---

### 实验 E：ResNet11 分层瓶颈分项可视化

| 项 | 内容 |
|---|---|
| 编号 | 实验 E |
| 名称 | ResNet11 *N*=4 主线各层 wall cycles 与 idle 来源分项 |
| 自变量 | layer_id ∈ {L0, L1, ..., L10}（11 层） |
| 固定参数 | ResNet11 整网，*N*=4，含空间到深度 + 跨核 SRAM 直送 + H 步长分离 + 本地段优先（即 §5.5.1 主线 wall cycles 190,133 配置） |
| 因变量 | 每层 wall cycles、PE 利用率、四类 idle（act_id / wgt_id / psm_id / acc_id）占比 |
| 期望图 | 堆叠柱状图：横轴 11 个层名，每柱分 5 段（fire 有效周期 + 4 类 idle），副轴折线为 PE 利用率 |
| 期望表 | 11 行 × 7 列（层名 / 形状 *K*-stride-*C*ᵢₙ-*C*ₒᵤₜ / wall_cy / fire / act_id / wgt_id / PE 利用率） |
| 论文意义 | 可视化 §5.5.2 文字描述的"*K*=3 主路径 78–93%、降采样 16–40%、Patch 70.9%、FC 5%"分布；将控制变量法（§5.5.4）的 4 类 idle 拆解从整网层面下放到每层，回答"哪些层是 IDMA 等待主导，哪些是计算 fire 主导" |

**复现命令**（直接用现有脚本）：
```bash
cd toolchain
python run_multicore_chain.py --smc --demo resnet11 --case_name smc_resnet11 --n_cores 4
cd ../sim/tb_smc && vsim -c -do run.tcl
# 数据来源：tb_smc_chain.sv 的 dump_pe_profile task 输出，每层每核 act_st/act_id/wgt_st/wgt_id/psm_id 字段
# 现有数据已在 paper/data/exp8_pe_util_roadmap.md 表 1，可直接复用
```

无需新跑仿真，本实验是对已有数据的图表化呈现。

---

## 3 候选实验池（不纳入 §5.5.5 主体，留作附录或后续）

| 候选 | 名称 | 不纳入原因 |
|---|---|---|
| F | *C*ₒᵤₜ 扫频 | *C*ₒᵤₜ < 16 时阵列列方向硬件不复用、PE 列空转，定性结论已在 §3.4 给出，扫频曲线信息量较低 |
| G | DSP SIMD 与 LUT 实现 P&R 资源对照 | 已在 §5.4.1 表 5.1 单点给出，扩展为"不同 *N* 配置下 LUT/DSP/Fmax"的 3 行表更合适，建议合并入 §5.4 而非 §5.5 |
| H | bias_rf 切换开销 / SmartConnect 替换 / mm2s 仲裁 | 已在 §5.5.4 控制变量法给出单点结论；扫频版本边际收益低 |

---

## 4 图表标题对照表（双语，按 spec §11.6）

| 编号 | 中文标题 | 英文标题 |
|---|---|---|
| 图 5.A | 不同 *C*ᵢₙ 下 Ky 折叠对 PE 利用率的影响 | Figure 5.A Effect of Ky-folding on PE utilization under different *C*ᵢₙ |
| 表 5.A | 不同 *C*ᵢₙ 下单层 wall cycles 与 Ky 折叠加速比 | Table 5.A Single-layer wall cycles and Ky-folding speedup under different *C*ᵢₙ |
| 图 5.B | 不同 stride 下降采样层 wall cycles 与 H 步长分离对照 | Figure 5.B Wall cycles of downsampling layer versus stride with and without H-stride separation |
| 表 5.B | 不同 stride 下 IDMA 命令数与 wall cycles 对照 | Table 5.B IDMA command count and wall cycles under different stride |
| 图 5.C | 不同卷积核 *K* 下 wall cycles 随 *K*² 的变化 | Figure 5.C Wall cycles versus *K*² under different convolution kernel sizes |
| 表 5.C | 不同卷积核 *K* 下单层 fire 数与平均每 fire 周期数 | Table 5.C Single-layer fire count and average cycles per fire under different *K* |
| 图 5.D | 不同 *W*ᵢₙ 下 *N*=1/2/4 多核扩展加速比 | Figure 5.D Multi-core speedup of *N*=1/2/4 under different *W*ᵢₙ |
| 表 5.D | 不同 *W*ᵢₙ 与核数下整网 wall cycles 与流水占空比 | Table 5.D Wall cycles and pipeline duty cycle under different *W*ᵢₙ and core counts |
| 图 5.E | ResNet11 *N*=4 主线各层 wall cycles 分项与 PE 利用率 | Figure 5.E Per-layer wall cycles breakdown and PE utilization on ResNet11 *N*=4 main configuration |
| 表 5.E | ResNet11 *N*=4 主线各层形状、fire 数与 idle 来源分项 | Table 5.E Per-layer shape, fire count and idle source breakdown on ResNet11 *N*=4 main configuration |

> §5 写作 agent 回写时按论文最终编号统一改为 5.6 / 5.7 / 5.8 / 5.9 / 5.10 等。

---

## 5 数据采集字段约定

每个 case 的 sim 输出（CASE_RESULT / CASE_PROFILE 行）需采集以下字段：

| 字段 | 含义 | 来源 |
|---|---|---|
| wall_cy | 整 case 端到端周期数 | tb 顶层 cycle counter |
| mac_fire | mac_array 实际有效周期数 | dump_pe_profile：每核 fire 字段 |
| mac_util | PE 利用率 = 2 × MAC 量 / (256 × wall_cy)（单位 %） | tb 计算 |
| mac_pipe | 流水占空比 = stage-0 join 周期 / wall_cy（单位 %） | dump_pe_profile：mac_pipe 字段 |
| act_id | mac_array 等 IFM 数据的 idle cycle | dump_pe_profile |
| wgt_id | mac_array 等 weight 的 idle cycle | dump_pe_profile |
| psm_id | mac_array 等 psum 反压的 idle cycle | dump_pe_profile |
| acc_id | parf_accum 等 mac fire drain 的 idle cycle | dump_pe_profile |
| idma_cmd | IDMA 实际下发的命令数 | dispatcher counter |
| ddr_busy | 单路 DDR 占空比（多核 case 含此项） | tb_smc 顶层 |

---

## 6 仿真时间预算与执行顺序建议

| 实验 | case 数 | 单 case 时长 | 总时长 |
|---|---:|---|---|
| A *C*ᵢₙ 扫频 | 14（7×2） | 1–5 分钟 | 30–60 分钟 |
| B stride 扫频 | 8（4×2） | 30 秒 | 5 分钟 |
| C *K* 扫频 | 5 | 30 秒–1 分钟 | 5 分钟 |
| D *N* × *W*ᵢₙ 扫频 | 12 | 2–3 分钟 | 30–40 分钟 |
| E ResNet11 分层 | 1（已有数据） | 0 | 0 |
| **合计** | **40** | – | **约 1.5 小时** |

建议执行顺序：E（无需仿真，先出图）→ B → C → A → D（耗时最长放最后）。

---

## 7 §5.5.5 节回写大纲建议（供 §5 agent 参考）

```
#### 5.5.5 参数敏感度分析

性能分析的最后一节考察 FLUX_CNN 在工作负载参数维度上的扩展性。固定其它参数，分别扫描
*C*ᵢₙ、stride、*K*、*W*ᵢₙ 与核数 *N* 五个维度，量化每个维度对 PE 利用率与 wall cycles 的
影响曲线，定位编译器侧 Ky 折叠与空间到深度优化的最大收益区间。

(1) *C*ᵢₙ 扫频与 Ky 折叠适用域 — 实验 A 数据 + 图 5.A + 表 5.A + 1 段定量分析
(2) stride 扫频与降采样层 H 步长分离 — 实验 B + 图 5.B + 表 5.B + 1 段
(3) 卷积核 *K* 扫频与轮次开销 — 实验 C + 图 5.C + 表 5.C + 1 段
(4) 多核 *N* × *W*ᵢₙ 扫频 — 实验 D + 图 5.D + 表 5.D + 1 段
(5) ResNet11 分层瓶颈分项 — 实验 E + 图 5.E + 表 5.E + 1 段

收尾 1 段：综合参数敏感度结论，回扣 §5.5.4 控制变量法量化的瓶颈分类。
```

预计 §5.5.5 篇幅约 1500–2000 字 + 5 张图 + 5 张表，与现有 §5.5.1–§5.5.4 篇幅相当。

---

## 8 数据回写注意事项

- 所有数字必须从 sim 实测取，不要从估算/外推填入。Round I baseline (commit `e8ca7b0`) 与 Round M（DSP SIMD）在不同 case 集合上 wall cycles 一致（已 bit-exact 校验），扫频仿真任选其一即可。
- 与 §5.4 表 5.1 (Round 4 routed) 对照时注意 Fmax 折算系数 1.438，本节扫频仍在 100 MHz 假设下给 wall cycles 对比，保持与 §5.5 现有口径一致。
- 实验 B / D 涉及多核 case，须在 `tb_smc` 而非 `tb_core_dma` 跑；实验 A / C 单核可在 `tb_core_dma`。
- 实验 E 直接用 `paper/data/exp8_pe_util_roadmap.md` 表 1 数据，无需重跑。

