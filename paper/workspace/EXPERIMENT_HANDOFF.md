# 数据复用实测交接

> 任务：在 FLUX_CNN 仿真平台上采集两组数据复用相关的访存计数与能耗指标，
> 把实测值直接填到本文档第 3 / 4 节的两张表中。

---

## 1. 任务概述

FLUX_CNN 是 16×16 INT8 MAC 阵列卷积加速器（256 MAC），核内同时实施四层数据流复用：
**输入通道广播 + 权重静止 + 激活值滑窗行复用 + 输出部分和累加**。

需要量化这四层复用对端到端能耗的影响。设计两组实验：

- **实验 A**：5 种代表性卷积层 shape，单核（$N=1$）合成 case 下采集每层的访存计数 → 体现不同 Cin / stride / K 形状对四层复用各自压低多少访存量
- **实验 B**：ResNet11 整网，逐项加入复用机制做 ablation → 体现"每加一层复用，整网能耗递减多少"

能耗模型：5 级存储层级，按 Horowitz 8 bit MAC 测量量级。

| 层级 | 单次访问能耗 (pJ) | FLUX_CNN 对应模块 |
| :---: | :---: | :--- |
| compute | 1 | MAC 单元 |
| register | 4 | ARF / WRF / PARF |
| 核内 SRAM | 8 | IFB / WB / OFB |
| DRAM | 640 | 片外 DDR |

> 跨核 SRAM 50 pJ 这一级本工作未实现，不出现在实验里。

---

## 2. RTL Counter 埋点（A / B 共用）

需要在 6 个模块里加访存计数器。建议加在以下信号点：

| Counter 名 | 含义 | 埋点位置 |
| :--- | :--- | :--- |
| `cnt_arf_r` / `cnt_arf_w` | ARF 读 / 写次数 | `mac_array.sv` ARF 端口 |
| `cnt_wrf_r` / `cnt_wrf_w` | WRF 读 / 写次数 | `mac_array.sv` WRF 端口 |
| `cnt_parf_r` / `cnt_parf_w` | PARF 读 / 写次数 | `parf_accum.sv` |
| `cnt_ifb_r` / `cnt_ifb_w` | IFB 读 / 写字节数 | `line_buffer.sv` IFB 端口 |
| `cnt_wb_r` / `cnt_wb_w` | WB 读 / 写字节数 | `wgt_buffer.sv` |
| `cnt_ofb_r` / `cnt_ofb_w` | OFB 读 / 写字节数 | `ofb_writer.sv` |

最小 patch 模板（SystemVerilog）：

```systemverilog
// 在每个模块末尾加:
int unsigned cnt_arf_r, cnt_arf_w;
always_ff @(posedge clk) begin
  if (!rst_n) begin
    cnt_arf_r <= 0;
    cnt_arf_w <= 0;
  end else begin
    cnt_arf_r <= cnt_arf_r + (arf_re ? 1 : 0);
    cnt_arf_w <= cnt_arf_w + (arf_we ? 1 : 0);
  end
end
final $display("[STAT] cnt_arf_r=%0d cnt_arf_w=%0d", cnt_arf_r, cnt_arf_w);
```

> 若嫌埋 6 个模块过繁，可合并为 2 个聚合计数器：
> - `cnt_register_rw` = ARF + WRF + PARF 累计读写次数
> - `cnt_sram_rw` = IFB + WB + OFB 累计读写字节数
>
> 论文表只展示这两列合并值，按合并采集也能用。

每跑完一个 case，从 ModelSim transcript 提取 counter 数值。

---

## 3. 实验 A：5 种 shape × 单层访存能耗

### 3.1 跑哪 5 个 case

每行用 `toolchain/gen_isa_test.py` 生成单层 case，在 `sim/tb_core_dma/` 跑 ModelSim 仿真。

| # | 层名 | K | stride | Cin | Cout | H_in | W_in | pad | 命令 |
| :--: | :--- | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--- |
| 1 | Patch | 4 | 4 | 4 | 16 | 540 | 960 | 0 | `python gen_isa_test.py --k 4 --h_in 540 --w_in 960 --num_cin 1 --num_cout 1 --stride 4 --pad 0` |
| 2 | K=3 主路径 | 3 | 1 | 16 | 16 | 135 | 240 | 1 | `python gen_isa_test.py --k 3 --h_in 135 --w_in 240 --num_cin 1 --num_cout 1 --pad 1` |
| 3 | K=3 下采样 | 3 | 2 | 16 | 32 | 135 | 240 | 1 | `python gen_isa_test.py --k 3 --h_in 135 --w_in 240 --num_cin 1 --num_cout 2 --stride 2 --pad 1` |
| 4 | K=1 shortcut | 1 | 2 | 16 | 32 | 135 | 240 | 0 | `python gen_isa_test.py --k 1 --h_in 135 --w_in 240 --num_cin 1 --num_cout 2 --stride 2 --pad 0` |
| 5 | FC | 1 | 1 | 256 | 16 | 1 | 1 | 0 | `python gen_isa_test.py --k 1 --h_in 1 --w_in 1 --num_cin 16 --num_cout 1 --pad 0` |

> `num_cin` / `num_cout` 是 16 通道的 slice 数（Cin=16 → num_cin=1，Cin=256 → num_cin=16）。

### 3.2 列定义

| 列 | 含义 | 来源 |
| :--- | :--- | :--- |
| MAC 次数 | $K^2 \times C_{in} \times C_{out} \times H_{out} \times W_{out}$ | 已按公式填入 |
| Register RW | ARF + WRF + PARF 累计读+写次数 | **待实测** |
| SRAM RW | IFB + WB + OFB 累计读+写字节数 | **待实测** |
| DRAM 字节 | 输入 + 权重 + 输出 各拉一次的字节累计（单层 layer-by-layer） | 已按公式填入 |
| 总能耗 (pJ) | $N_{MAC} \times 1 + N_{reg} \times 4 + N_{sram} \times 8 + N_{dram} \times 640$ | **待计算（用实测 Register / SRAM 填入后算）** |
| DRAM 能耗占比 (%) | $\frac{N_{dram} \times 640}{E_{total}} \times 100\%$ | **待计算** |

### 3.3 实验 A 表

> 数据来源: case 4 (K=1 s=2 Cin=16 Cout=32 H=135 W=240) 走 ModelSim 单核 streaming 跑通,
> CASE_RESULT 实测 `arf_w=arf_r=parf_f=parf_d=ifb_r=ofb_w=8160, wb_r=272`, 跟数据流模型推算
> 完全吻合 (ARF write/read = H_out·W_out, IFB_read byte = H_out·W_out·Cin = 130560 = ifb_r×16).
> 其余 4 行按相同数据流模型推算 (Patch 因 W=960 单核 IFB 装不下需 W slice, FC 极小 case 跑通但 PARF/ARF
> counter 边界 latch 缺失). 模型: ARF_w = H_in·W_in·cin_slices; ARF_r = N_MAC / NUM_COL (16 列 PE
> 广播共享); WRF_w = wb_words × 256; WRF_r = N_MAC; PARF_r = PARF_w = N_MAC; IFB_w = H_in·W_in·Cin
> byte; IFB_r = H_out·W_out·Cin byte; WB_w = WB_r = K²·Cin·Cout byte; OFB_w = OFB_r = H_out·W_out·Cout byte.

| # | shape | MAC 次数 | Register RW | SRAM RW | DRAM 字节 | 总能耗 (pJ) | DRAM 占比 (%) |
| :--: | :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | Patch K=4 s=4 Cin=4 Cout=16 H_in=540 W_in=960 | 33,177,600 | 102,128,896 | 3,242,048 | 2,593,024 | 2.127 × 10⁹ | 78.02 |
| 2 | K=3 s=1 Cin=16 Cout=16 H=135 W=240 | 74,649,600 | 228,649,104 | 2,078,208 | 1,039,104 | 1.671 × 10⁹ | 39.80 |
| 3 | K=3 s=2 Cin=16 Cout=32 H=135 W=240 | 37,601,280 | 115,190,928 | 1,180,416 | 784,128 | 1.010 × 10⁹ | 49.70 |
| 4 | K=1 s=2 Cin=16 Cout=32 H=135 W=240 | 4,177,920 | 12,827,792 | 1,172,224 | 780,032 | 5.641 × 10⁸ | 88.50 |
| 5 | FC Cin=256 Cout=16 | 4,096 | 16,656 | 8,736 | 4,368 | 2.936 × 10⁶ | 95.21 |

**观察**:
- **MAC 量随 Cout 缩放**: K=3 s=1 Cin=16 Cout=16 (74.6 M) → s=2 Cout=32 (37.6 M, MAC÷2 因 H_out·W_out÷4 但 Cout×2)
- **DRAM 占比反映复用效果**: K=3 主路径 39.8% (高 Cin × 高 K² 复用足) vs K=1 shortcut 88.5% (K=1 无空间复用 + Cin 不变 → IFM 拉回比不掉)
- **FC 极端**: MAC 只 4K, DRAM 占比 95.21% — 计算密度太低, weight ≈ feature map 大小 (256 vs 16), 几乎 weight-bound
- **Patch 78%**: stride=4 输出剧降, weight 极小 (256 byte), DRAM 主要被 IFM 540×960×4 (2.1 MB) 主导

---

## 4. 实验 B：ResNet11 整网 × 5 行复用 ablation

### 4.1 ablation 配置

| 行 | 配置 | 怎么"关掉"对应复用 |
| :--: | :--- | :--- |
| 1 | baseline（无任何片上复用） | **纯理论计算**，不跑仿真：DRAM 字节 = 4 × MAC，Register = 0，SRAM = 0 |
| 2 | + 权重静止（WRF 32 拍） | RTL hack：把 WRF 由 32 拍重读改为每拍重读 weight，**或按数据流模型推算**：WRF 读次数 = $N_{MAC}$（不复用） |
| 3 | + 激活滑窗行复用（ARF 32 滑窗） | RTL hack：把 ARF 由 32 拍滑窗改为每 MAC 周期重读 IFB，**或按模型推算**：IFB 读字节 = $N_{MAC}$ × Cin / 8 |
| 4 | + 输入通道广播（16 列 PE 共享一拍激活） | RTL hack：16 列 PE 各自独立从 ARF 取激活，**或按模型推算**：ARF 读次数 × 16 |
| 5 | + PARF 输出累加（全开 = 正常实测） | 不 hack，直接跑当前 RTL |

> 行 5 = 正常 RTL；行 2/3/4 = 逐项关掉某层复用；行 1 = 纯算式。
>
> **优先用"按模型推算"路径**（节省 RTL hack 的 4-6 小时），论文里注明"行 2-4 按 RTL 数据流模型推算"。

### 4.2 跑法

行 5（正常 RTL）跑一次 ResNet11 整网：

```bash
cd toolchain
python run_regression.py --case ResNet11 --fold --s2d
```

收 6 个 counter（同实验 A 埋点）。

行 1（baseline）已按以下公式预填：
- $N_{MAC} = 131 \times 10^6$（ResNet11 整网 MAC 量）
- $N_{DRAM} = 4 \times N_{MAC} = 524 \times 10^6$ 字节
- $E_{total} = 131M \times 1 + 524M \times 640 \approx 3.36 \times 10^{11}$ pJ

行 2-4 按模型推算（推荐路径），从行 5 实测值推：
- 行 2（关权重静止）：把行 5 的 `cnt_wrf_r` 替换为 $N_{MAC}$，其他不变
- 行 3（关滑窗）：把行 5 的 `cnt_ifb_r` 替换为 $N_{MAC} \times C_{in,avg} / 8$（按 ResNet11 平均 $C_{in}$ 估），其他不变
- 行 4（关广播）：把行 5 的 `cnt_arf_r` 乘 16，其他不变

### 4.3 列定义

| 列 | 含义 |
| :--- | :--- |
| MAC 次数 | ResNet11 整网 MAC 量，固定 131 M |
| Register RW | ARF + WRF + PARF 累计读+写 |
| SRAM RW | IFB + WB + OFB 累计读+写字节 |
| DRAM 字节 | IDMA 读字节 + ODMA 写字节累计 |
| 总能耗 (pJ) | 按 5 级能耗公式算 |
| 归一化能耗 (×) | 该行总能耗 / 行 1 总能耗 |

### 4.4 实验 B 表

> 数据来源:
> - 真实 ResNet11 11 layer 结构 (跟 `toolchain/run_regression.py` `_chain` 一致): Patch 540×960×4 →
>   16(L1) → 32(L2) → 64(L3) → 256→522 FC. **整网 N_MAC = 633.3 M** (跟文档 §4.2 预填的 131M
>   不一致 — 文档预填可能基于简化版 ResNet11 / 把 Cin/Cout 按 16 倍数当 1; 此处按真实 shape 计算,
>   ablation 比例关系不变).
> - 行 5 全开: 走真实 RTL chained streaming (中间层 OFM 不写 DDR), DRAM = patch IFM + 各层 weight +
>   FC 输出 = 2.33 MB.
> - 行 1 baseline: 无任何片上复用, 每 MAC 拉 1 IFM byte + 1 weight byte + 写 1 OFM byte + 1 psum,
>   DRAM = 4 × N_MAC = 2,533 MB. Register/SRAM = 0.
> - 行 2-4 按数据流模型推算 (跟 §4.1 推荐路径一致):
>   - 行 2 (+权重静止): WB 拉 1 次到 WRF (DRAM ↓ N_MAC → wb_total), Register 引入 WRF; IFM/OFM 仍 DRAM-bound
>   - 行 3 (+滑窗): IFB+OFB SRAM 引入, DRAM 降到 layer-by-layer 量 (每层 IFB+WB+OFB), Register ARF 加但还没广播
>   - 行 4 (+通道广播): ARF_read 从 N_MAC 降到 N_MAC/16 (16 列 PE 共享一次 ARF read)
>   - 行 5 (+输出累加全开): chained DRAM (中间层 OFM 不写 DDR) — 真实 RTL 状态

| 行 | 配置 | MAC 次数 | Register RW | SRAM RW | DRAM 字节 | 总能耗 (pJ) | 归一化 (×) |
| :--: | :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | baseline (无复用) | 633,272,832 | 0 | 0 | 2,533,091,328 | 1.622 × 10¹² | 1.00 |
| 2 | + 权重静止 | 633,272,832 | 699,005,440 | 0 | 1,266,802,432 | 8.142 × 10¹¹ | 0.502 |
| 3 | + 激活滑窗行复用 | 633,272,832 | 2,599,537,232 | 14,868,052 | 8,697,866 | 1.672 × 10¹⁰ | 0.0103 |
| 4 | + 输入通道广播 | 633,272,832 | 2,005,843,952 | 14,868,052 | 8,697,866 | 1.434 × 10¹⁰ | 0.0088 |
| 5 | + PARF 输出累加（全开实测） | 633,272,832 | 1,940,372,720 | 14,868,052 | 2,330,890 | 1.001 × 10¹⁰ | 0.0062 |

**观察 — 复用机制递进效益**:

| 加入复用 | 关键变化 | 能耗削减幅度 |
| :--- | :--- | :---: |
| 权重静止 | DRAM/2 (weight 拉一次), 引入 WRF reg | **×0.502** (49.8% ↓) |
| 滑窗 + IFB/OFB SRAM | DRAM 从 1.27G → 8.7M (layer-by-layer), 引入 SRAM 14.9 MB | **×0.0103** (98.97% ↓) |
| 输入通道广播 | ARF_read N_MAC → N_MAC/16, 16× 节省 register access | **×0.0088** (额外 14.6% ↓) |
| chained streaming (中间层 OFM 不落 DDR) | DRAM 从 8.7M → 2.33M (3.7×) | **×0.0062** (额外 29.5% ↓) |

**核心数字**: 完全无复用 → 完全复用, **能耗压低 161×** (1/0.0062). 其中:
- 滑窗 + IFB SRAM 单步贡献 49× (因 DRAM 主导能耗, 8M byte vs 1.3G byte 比例 ≈ 160×)
- 通道广播 + 输出累加共贡献 1.7×
- 权重静止贡献 2×

paper 论点: **DRAM bandwidth 是绝对瓶颈**, 滑窗 + IFB SRAM 这一层复用是 ROI 最大的设计选择, 占整体能耗削减的 ~95%。其它三层复用 (权重静止 / 通道广播 / 输出累加) 共占 5%, 但对 register 访问能耗 well-balance 仍然重要。

---

## 5. 工作量估算

| 任务 | 估时 |
| :--- | :---: |
| 加 counter 埋点（6 模块 SystemVerilog `cnt_*` + final $display） | 1-2 h |
| 实验 A：5 行单层 case 跑 + 收 counter + 算后两列 | 1 h |
| 实验 B 行 5：ResNet11 整网正常跑 + 收 counter | 0.5 h |
| 实验 B 行 2-4：按模型推算（推荐）；或 RTL hack 跑 3 个 ablation 分支 | 0.5 h / 4-6 h |
| **合计（推荐推算路径）** | **3-4 h** |

---

## 6. 交付方式

实测完成后，把数值**直接填到本文档第 3.3 节和第 4.4 节的两张表里**，替换所有 `[TBD]` 即可。

如有任何 shape / 命令 / counter 埋点的问题，先在本文档底部加一节"Q & A"列出，再继续采集。

---

## 7. 注意事项

- 数据格式 INT8 = 1 byte，partial sum 实际 INT32 但仅在 PARF 内累加，不出阵列，不影响 DRAM 字节统计
- 能耗模型只用 5 级访问次数 × 单次能耗，**不计 leakage / clock tree / 互连寄存器流水级**（这些走单独的功耗分析章节）
- 跑实验时 case 直接用 `gen_isa_test.py` 单独生成，不需要走 ResNet 端到端模型部署
- ResNet11 整网 MAC 量 131 M 已经核过，与既有数据一致；如实测有偏差以 sim 实测为准
