# 实验 10 计划: VD100 板 PL demo (功能验证用)

**日期**: 2026-05-08
**目标板**: ALINX VD100 (Versal AI Edge VE2302, 用户手中已有)
**最终论文板**: K325T (XC7K325TFFG900-2I, sim + 综合 + P&R 数据)
**职责分工**: VD100 = 功能验证 demo, K325T = 论文性能数据

## 核心约束

不用 AIE-ML — 当前 FLUX_CNN 架构本身就是模仿 AIE-ML 设计的, 用 AIE 等于"用现成 IP 跑模型"
跟自研架构无关. **VD100 PL 部分跑当前 RTL** 验证功能/数据流, AIE-ML 完全不碰.

## VE2302 PL 资源 vs 当前设计需求

| 维度 | VE2302 PL 容量 | 4 核 LUT 版 (K325T routed) | 占用 |
|---|---:|---:|---:|
| LUT | 150,272 | 153,892 | **102%** ⚠️ 略超 |
| DSP58 | 464 | 320 | 69% ✓ |
| BRAM (36Kb) | 5.4 Mb | 10.4 Mb | **193%** ❌ 装不下 |
| **+ UltraRAM** | **+43.6 Mb (Versal 独有)** | – | – |
| BRAM + URAM 合计 | 49 Mb | 10.4 Mb | 21% ✓ (大 SRAM 映射 URAM) |

LUT 略超 + BRAM 不够 → **降到双核版** demo:

| 维度 | 双核估算 | VE2302 PL | 占用 |
|---|---:|---:|---:|
| LUT | ~80K (含 axi_dm) | 150,272 | 53% ✓ |
| DSP | 160 (sdp) | 464 | 34% ✓ |
| BRAM (含 URAM) | ~3.2 Mb | 49 Mb | 7% ✓ |
| 性能 (sim) | 450,469 cy / 444 FPS @ 100MHz (历史 STATUS §216) | – | – |

## 改造点列表 (RTL 95% 复用, 只换 IP / board 整合)

| 模块 | K325T (7-series) | VD100 (Versal) | 改动 |
|---|---|---|---|
| RTL: mac_array (LUT 版) | 不变 | 不变 | 0 |
| RTL: core pipeline | 不变 | 不变 | 0 |
| RTL: ifb/ofb/wb sram_model | BRAM | URAM (改 attribute) | < 50 行 |
| axi_dm IP | DataMover | Vitis Versal axi_dma | 0.5 周 (重生成 IP) |
| axi_smc_4to4 | crossbar IP | NoC + Smart Connect 自动 infer | 1 周 (Versal NoC 配置) |
| DDR controller | 没用 (sim stub) → MIG IP | Versal NoC DDRMC (内置 hard IP) | 0.5 周 (NoC port 连接) |
| PCIe | 不用 (单板验证) / XDMA | CPM PCIe (内置) — 但 demo 不一定要 | 0 (用 PS 直连即可) |
| Board top | 自写 + xdc | ALINX VD100 board files | 0.5 周 |
| Host runtime | 没 PS, 要外接 | **A72 跑 baremetal C** (Versal 内置 PS) | 1 周 |
| Vitis Unified IDE | 不用 | 第一次用要熟悉 | 0.5-1 周 |

**总投入**: 6-8 周到 VD100 demo 跑通

## 不上 SIMD on DSP58 的原因

VE2302 用 DSP58 (27×24 multiplier), 不是 DSP48E1 (25×18). DSP58 支持原生 INT8 SIMD
(1 DSP 跑 4×8 INT8 mul). 但:

1. 当前 mac_simd_pair RTL 是按 DSP48E1 的 25×18 packing 写的, packing 公式不同
2. VD100 demo 只是验证功能, 不追求最终性能 (那是 K325T 论文数据的事)
3. 双核 LUT 版完全够用, 不必额外 1-2 周改 DSP58 packing

## 双线推进策略

### 线 1: 论文数据 (不上板, K325T 综合 + sim)
- ✅ ResNet11 N=4 sim cy=190258 (SIMD 版) / 190133 (LUT 版)
- ⏳ K325T SIMD 版 P&R 数据 (round 3 在跑)
- ⏳ K325T LUT 版 P&R 数据 (已完成: WNS=-0.777ns, Fmax=125.4 MHz, LUT 153K, DSP 320)
- 出 paper 性能表 / 资源表 / Fmax 数据

### 线 2: 功能验证 demo (VD100 双核版上板)
- 目标: ResNet11 真实硬件跑通, 取真实 latency 含 DDR4/PS overhead
- 投入 6-8 周
- 出 paper demo section (real device verification)
- RTL 复用率 95%, 只换 DDR/AXI IP + board top + PS host runtime

## 风险 + 备选

**风险**: Vitis Unified IDE 第一次用学习曲线大. NoC 配置 + AXI Smart Connect 需要熟悉。

**备选**: 如果 VD100 demo 时间太紧, 可以走"PL 双核 + 板上 BRAM 全用 (不接 DDR4)"路线, 把 weight + IFM 全装 PL BRAM, 跳过 DDR/PS host. 简化版 demo, 跑预存测试图就行.

## 数据来源

- VE2302 datasheet: DS950 (v1.15)
- ALINX VD100 用户手册 / board files
- 历史 sim 数据: STATUS §216 (N=2 = 450K cy)
- LUT 版 routed: Syn/reports_smc/utilization.rpt
