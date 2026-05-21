# paper/data/ 实验数据总览

按 Round 时序 + 分支 整理, 给论文写作时快速 grep。

## main 分支 — K325T 论文性能数据

| Exp | Round | 内容 | 关键数据 |
|---|---|---|---|
| exp1 | A | W slice 切片公平 | ResNet11 220K → 205K cy (-6.6%) |
| exp2 | – | IDEAL_SMC 控制变量 | sim model crossbar = -7% (SMC IP arbitration overhead 量化) |
| exp3 | B-D | cout slice / TB host parallel / desc preload / S2MM pause | 累计 217K → 204K cy (-6%) |
| exp4 | E | mm2s_arb WDMA 饥饿 | L0 wgt_st 减半 |
| exp5 | – | ds layer memory-bound 控制变量 | Mac fire 数 vs IDMA 拉数据量数学推导 |
| exp6 | I | H stride 分离 | ResNet11 -5.9% (-12K cy), L3/L6/L9 ds layer 减 40-43% |
| exp7 | J | W 压缩软件层 (失败) | +37% 灾难退化, axi_dm cmd 颗粒度天花板 |
| exp8 | K | cmd 排序 + cout slice ROI | Round K -0.44%, Round L 不实施 |
| exp9 | M | INT8 SIMD on DSP48E1 ⭐ | LUT 162→99K (-36%), DSP 320→832, Fmax 125→144 MHz |
| exp10 | – | VD100 demo plan | 板级化路线图 |
| exp11 | – | ResNet11 真图推理 | 3 张合成图 sim bit-exact PASS |
| [k7_n4_smc_synth_report_2026_05_19](k7_n4_smc_synth_report_2026_05_19.md) | – | K7-325T N=4 SMC 综合报告 (本次更新) | post-route Fmax **135.86 MHz** / 173K cy / **783 FPS** / Power 1.925 W |

**main 分支最终数据 (2026-05-19 更新, K325T xc7k325tffg900-2 routed)**:
- ResNet11 N=4 SIMD: **173,432 cy** / **783.5 FPS @ 135.86 MHz** (routed Fmax)
- LUT: 93,756 (46%) / FFs: 60,925 (15%) / DSP: 832 (99%) / RAMB36: 288 (65%)
- Power: 1.925 W (Dynamic 1.74 + Static 0.185), 143.6 GOP/W peak efficiency
- vs 历史 (190,133 cy / 700 FPS @ 133.3 MHz): **−8.8% cycles, +11.9% FPS**

历史数据 (exp9 SIMD baseline, 仅参考):
- ResNet11 N=4: 190,133 cy, paper INDEX 老版写的 "757 FPS @ 143.8 MHz" 是按 synth-only Fmax 算的乐观估算, routed 实际 700 FPS @ 133.3 MHz

## vd100-demo 分支 — VE2302 板级 demo

| Exp | 内容 | 关键数据 |
|---|---|---|
| exp12 | VD100 OOC 综合 PASS | LUT 128K (85%) / DSP 144 / **URAM 64** ⭐ / **Fmax 182 MHz** |
| exp13 | BD 集成 (wrapper + tcl) | RTL 就位, BD GUI finalize 待板级集成 |

**vd100-demo 分支最终数据** (VE2302 xcve2302-sfva784-1LP-e-s OOC):
- ResNet11 N=3 sim 估: ~230K cy / **~430 FPS @ 100 MHz**
- LUT: 128,375 (85%) / DSP: 144 (31%) / URAM: 64 (67%, Versal 自动映射)
- Fmax: 182.3 MHz (vs K325T 144 MHz, 7nm 工艺红利 +27%)

## 双板对比 (paper 主要表格)

| 维度 | K325T (main) | VE2302 (vd100-demo) |
|---|---:|---:|
| 工艺 | 28nm | 7nm |
| LUT | 99,061 | 128,375 |
| DSP | 832 (DSP48E1, INT8 SIMD) | 144 (DSP58, no SIMD) |
| 大 SRAM 映射 | RAMB36 × 288 | RAMB36 × 81 + URAM × 64 ⭐ |
| Fmax | **135.86 MHz (routed)** | 182.3 MHz (OOC) |
| ResNet11 cy | **173,432 (sim, 2026-05-19 更新)** | ~230,000 (sim 估, N=3) |
| 整网 latency @ Fmax | 1.32 ms | 1.26 ms |
| FPS @ Fmax | **783.5** (2026-05-19 routed) | **~792** (估) |
| Host 接口 | (无) | PS 千兆网 + Windows Python |
| 板级 demo | sim/综合数据 | 待板级 bring-up |

## paper 写作建议

### 主线 ("我们做了什么")
1. 自研 INT8 SIMD on DSP48E1 包装 (mac_simd_pair, exp9 Round M)
2. ds layer H stride 分离 (exp6 Round I)
3. 控制变量法量化 SMC IP / mm2s_arb / ds memory-bound 各部分 overhead

### 跨平台 ("可移植性")
1. 同 RTL 跨 K325T (28nm) → VE2302 (7nm), 性能 +27% 自然提升
2. URAM 自动映射 (Versal 独有), 无需 RTL attribute
3. Vitis BD 集成路径 (multicore_top_vd100_bd wrapper)

### 失败实验 ("反思")
1. Round J 软件 W 压缩 +37% 退化 → axi_dm IP 天花板
2. Round L cout slice ROI 不划算 → driver layer transition 复杂度

## Commits 索引

main: `798af63` (K325T SIMD Round 4 P&R PASS)
vd100-demo: `0603352` (BD wrapper + tcl 集成)

文件位置: `paper/data/exp{1,2,...}_<title>.md`, 每个实验独立 markdown.
