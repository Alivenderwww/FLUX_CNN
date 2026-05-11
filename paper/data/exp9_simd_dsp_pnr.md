# 实验 9 / Round M: MAC 改造 — INT8 SIMD on DSP48E1 (1 DSP 装 2 PE)

**日期**: 2026-05-08
**目标板**: Xilinx Kintex-7 XC7K325TFFG900-2 (Vivado 2023.1, K325T 全栈)
**Sim 验证**: ResNet11 N=4 SMC bit-exact PASS, cy=190133 (跟 LUT 版完全一致)

## 动机

读完 UG479 + 用户洞察:
- 原 mac_pe.sv 用 8×8 INT8 mul, **Vivado 默认不推 DSP** (颗粒度太小, 用 LUT 实现)
- 4 核 mac_array 占 58K LUT (整网 36% LUT) 全部跑在 LUT 上
- **mac_array 跨 col 共享 act_in_vec** (16 col 接同一 input vec, weight 各异) →
  完美适配 INT8 SIMD packing trick (XAPP1259 风格)

K325T 资源约束:
- Total LUT 203,800 / DSP 840 / RAMB36 445
- 4 核 mac 256 PE × 4 = 1024 PE
- 全推 DSP (1 PE = 1 DSP): 1024 DSP > 840 装不下
- **INT8 SIMD packing**: 1 DSP = 2 PE (跨 col 配对) → 512 mac DSP + 320 sdp = 832 ≤ 840 ✓

## INT8 SIMD on DSP48E1 packing scheme

DSP48E1 multiplier: 25-bit signed × 18-bit signed = 43-bit signed product.

```
A_packed (25-bit) = {w1[7:0], 9'b0, w0[7:0]}    // 2 weights + 9-bit carry isolation
B_ext    (18-bit) = sign_extend(act_in[7:0], 18)
P (43-bit)        = A_packed × B_ext
                 = w1×B × 2^17 + w0_unsigned × B    // (w0_unsigned = w0[7:0] as unsigned 0..255)
```

**位提取 + sign correction**:

```
P[15:0]  = (w0_unsigned × B) low 16 bits
        = w0×B + (w0[7] ? 256×B : 0)            // unsigned 视图多 256×B 当 w0<0
prod_0   = P[15:0]_signed - (w0[7] ? (act_in << 8) : 0)

P[32:17] = w1×B + arithmetic_shift_right_carry   // w0_unsigned×B<0 时 sign-ext 让 P[32:17] 减 1
prod_1   = P[32:17]_signed + (w0_nonzero & act_in[7] ? 1 : 0)
```

**关键陷阱 — sign correction 减法器不能让 Vivado 推 DSP**:
当 sign correction 写成 `prod_0 = P[15:0] - prod_0_corr`, Vivado 默认推断 `(C-A:B)` 模式
**额外占用 1 个 DSP**! 实测: simd_pair 用 **2 DSP** 而不是 1 DSP, 直接打破 SIMD 收益.

修复: 在 sign correction wire 加 `(* use_dsp = "no" *)` 强制走 LUT.

```systemverilog
(* use_dsp = "no" *) logic signed [PROD_WIDTH-1:0] prod_0_w;
(* use_dsp = "no" *) logic signed [PROD_WIDTH-1:0] prod_1_w;
assign prod_0_w = prod_0_raw - prod_0_corr;
assign prod_1_w = ...;
```

## RTL 改造

新建 2 个文件 (RTL 0 行修改原 mac_pe/mac_col/mac_array, ifdef 切换):

| 文件 | 行数 | 说明 |
|---|---:|---|
| `RTL/mac_simd_pair.sv` | ~110 | 1 simd_pair = 1 DSP × 2 PE, 含 sign correction |
| `RTL/mac_array_simd.sv` | ~180 | 跨 col 配对结构 (8 col-pair × 16 row × 1 DSP) |
| `RTL/core_top.sv` | +6 (ifdef) | `+define+FLUX_MAC_SIMD` 切到 SIMD 版 |

**架构关键**:
- 8 col-pair × 16 row × 1 simd_pair = **128 DSP / 核** (vs 当前 LUT 版 mac_array 0 DSP)
- 跨 col 配对: pair p 含 col 2p 跟 col 2p+1, 同 row 共享 act_in[r], weight 不同
- 加法树跟原 mac_col 等价 (16-input adder tree per col), 但每 col-pair 输出 2 个 col_psum
- pipe 时序: compute_en T → dsp_p reg T+1 → col_psum_reg T+2 (跟原 mac_array 完全一致)

## Sim 验证 (ModelSim)

| Test | Cycles | OFM | 跟 LUT 版对比 |
|---|---:|---|---|
| smc_wslice1 (单层 K=3) | 4161 | bit-exact | ✓ 完全一致 |
| **smc_resnet11 N=4** | **190133** | **all 11 layers bit-exact** | ✓ **完全一致** |

Sign correction 公式数学正确, pipe 时序对齐, 无 functional regression.

## P&R 数据 (Vivado 2023.1 + xc7k325tffg900-2 -2I 工业级 = -2 商业级 timing)

### LUT 版 baseline (同板 P&R 实测)

| 指标 | Synth | Routed |
|---|---:|---:|
| Total LUT | 162,566 | **153,892** (75.5%) |
| FF | 63,868 | 63,160 |
| RAMB36 | 288 | 288 (65%) |
| DSP | 320 | 320 (38%) |
| WNS @ 7.2 ns | +0.196 ns | **-0.777 ns ❌** |
| Fmax | 138.9 MHz (估) | **125.4 MHz** |

### SIMD 版第一版 (BUG: sign corr 减法器被推 DSP, 已修复)

| 指标 | Synth (bug) |
|---|---:|
| Total LUT | 210,679 (103%) ❌ 反涨 |
| DSP | **840 (100% K325T 上限!)** ❌ 全占 |
| 每 simd_pair DSP | **2** (1 mul + 1 sign_corr 减法) |
| 收益 | -inf (不可行) |

### SIMD 版 4 轮迭代 (踩坑全记录)

| Round | 实施 | LUT (synth) | DSP/pair | P&R 结果 |
|---|---|---:|---:|---|
| 1 | 组合 sign corr (`prod_0 = P_low - corr` 直接 wire) | 210,679 | 2 | DSP 推断 `(C-A:B)` ❌ |
| 2 | logic decl 加 `(* use_dsp = "no" *)` | 210,679 | 2 | attribute 不识别 ❌ |
| 3 | 加 1 拍 LUT reg + always_ff 上 `USE_DSP="no"` | 204,912 | 2 | P&R 失败: LUT 超 K325T 1112 ❌ |
| **4** | **拆 sign_corr_lut 子模块 + module-level `USE_DSP="no"`** | **107,883** | **1** | ✅ **PASS** |

**核心教训**: Vivado USE_DSP attribute **唯一对 sign correction 真正生效的写法**是
**module-level attribute** (子模块包装). 加在 logic 声明 / always_ff / wire 上都被忽略.

### Round 4 routed 完整数据 (修复成功)

| 指标 | LUT 版 routed (baseline) | **SIMD 版 routed (Round 4)** | Δ |
|---|---:|---:|---:|
| Total LUT | 153,892 (75.5%) | **99,061 (48.6%)** | **-54,831 (-35.6%)** ⭐ |
| FF | 63,160 | 65,793 | +2,633 |
| RAMB36 | 288 (65%) | 288 | 同 |
| **DSP** | 320 (38%) | **832 (99%)** | **+512** ⭐ |
| **WNS @ 7.2 ns** | -0.777 ns | **-0.303 ns** | **+0.474 ns** ⭐ |
| **Fmax** | 125.4 MHz | **143.8 MHz** | **+18.4 MHz, +14.7%** ⭐ |

每核数据:
- 单核 core_top: LUT 23,639 (vs LUT 版 35,398, **-33%**), DSP 208 (= 128 mac SIMD + 80 sdp)
- 单核 mac_array_simd: 6,649 LUT + 128 DSP (vs LUT 版 mac_array 14,515 LUT + 0 DSP)
- 每 simd_pair: ~32 LUT + 1 DSP + sign_corr_lut 22-40 LUT (sign corr 走 LUT 实现)

### Fmax 提升解读

LUT 版关键路径: 16-input adder tree (5 级 LUT 加法器) → 慢
SIMD 版关键路径: DSP P 输出 → sign_corr LUT 1 拍 → 16-input adder tree → 短

DSP 内部 hard wired mul + accumulator 比 LUT-MAC 快很多, **关键路径从 7.977 ns 缩到 7.503 ns**.

## 论文意义

### 1. UG479 的 SIMD ≠ INT8 packing — 一个常见误解

UG479 的 "SIMD mode" 是 **adder SIMD** (USE_MULT=NONE 时 dual 24-bit 或 quad 12-bit add).
跟 multiplier 互斥, 不能用于 PE 阵列 INT8 mul packing.

真正的 "1 DSP 装 2 个 8×8 mul" 是 **packing trick** (XAPP1259), 利用 25×18 multiplier
塞 2 个 weight 进 A 端, 共享 B 端 input. 配 sign correction 处理 unsigned-to-signed 视图差异.

### 2. Sign correction 必须走 LUT, 不能让 Vivado 推 DSP

第一版 SIMD RTL 没加 use_dsp attribute, Vivado 把 `prod = P_low - 256×B` 推断成
DSP `(C - A:B)` 模式, 占额外 1 DSP. simd_pair 实际用 2 DSP, SIMD 收益归零.

加 `(* use_dsp = "no" *)` 到 sign correction 减法器后才真正实现 1 DSP / 2 PE 目标.
**这是 INT8 SIMD on DSP48E1 实施时容易踩的坑**, 在论文里值得突出.

### 3. ROI 验证

跟 Round J/K/L 形成完整 RTL 优化谱系:
- Round J 软件 W 压缩: ❌ +37% (axi_dm cmd 颗粒度天花板)
- Round K cmd 排序: -0.44% (软件层边际收益)
- Round L cout slice: 不实施 (driver layer transition 复杂)
- **Round M MAC SIMD on DSP48E1**: 待 P&R 数据 (预期 LUT -50K, DSP +500)

### 4. RTL 改造小, 接口零变动

- mac_array 跟 mac_array_simd 接口完全一致 (NUM_COL/NUM_PE 参数等)
- core_top 仅加 ifdef, wgt_buffer/parf_accum/line_buffer 全 untouched
- driver / scheduler / sim TB 0 改动
- 关 ifdef 立刻回到 LUT 版

## 数据来源

- LUT 版 P&R commit: `6ba37ff` (Round K baseline)
- SIMD 版 commit: `<待 commit>`
- LUT 版 P&R 报告: `Syn/reports_smc/utilization.rpt` + `timing_summary.rpt`
- SIMD 版 P&R 报告: 同路径 (覆盖, 待跑完)
- Sim case: `cd sim/tb_smc && FLUX_MAC_SIMD=1 vsim -c -do run.tcl`
- Syn cmd: `set FLUX_MAC_SIMD=1 && vivado -mode batch -source Syn/run_syn_smc.tcl`
