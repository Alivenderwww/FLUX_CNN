# Tab.6: Multi-Core Scaling Sweep — N=1 / N=2 DDR / N=4 W-Slice

## 在 paper.md 中的引用位置
- **首次引入**：§7.5 Multi-Core Scaling, line 424
  - 原文：`Table 6 reports the multi-core scaling sweep across three columns — single-core baseline, N=2 DDR-mode, and N=4 W-slice (20-case subset) — with cycles, observed speedup, AXI arbitration overhead percentage, and W-slice halo recomputation overhead.`
- **被引用次数**：1 次显式

## 论证作用
**narrative B 多核规模化的核心数据表**。证明 W-slice multi-core 在 BRAM-bound 边缘 FPGA 上是工作的（20/20 W-slice case bit-exact）。所有具体 cycles / speedup 数字标 [CHECK]，等 §7.5 实测对齐。

## 列定义
| 列名 | 含义 |
|------|------|
| Case | 测试 case 名（来自 sim/tb_multicore/） |
| Shape (K/s/Cin/Cout/W×H) | 卷积形状 |
| N=1 cycles | 单核 baseline |
| N=2 (DDR mode) cycles | M1.5 阶段双核 (无跨核 SRAM) |
| N=4 (W-slice) cycles | M2.5 阶段四核 W-slice |
| Speedup N=2 / N=1 | 观察 speedup |
| Speedup N=4 / N=2 | 观察 speedup |
| AXI arb overhead | AXI 仲裁开销 % |
| Halo recomp overhead | W-slice halo 重叠开销 % |

## 表初稿

| Case | Shape (K/s/Cin/Cout/W×H) | N=1 cycles | N=2 cycles | N=4 cycles | N=2/N=1 | N=4/N=2 | AXI arb % | Halo % |
|------|--------------------------|-----------|------------|------------|---------|---------|-----------|--------|
| **wslice1** (headline)| 3/1/16/16/32×32 | [CHECK]† | **5,569** | **3,833** | [CHECK] | **1.45×** | [CHECK: ~3%] | [CHECK] |
| wslice_oddw | 3/1/16/16/33×33 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK: 高 (W=33 not /4)] |
| wslice_smallw | 3/1/16/16/8×8 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK: 高 (startup dom)] |
| wslice_k7 | 7/1/16/16/32×32 | [CHECK] | [CHECK] | **34,097** | [CHECK] | [CHECK] | [CHECK] | [CHECK: 高 (large halo)] |
| wslice_stride2 | 3/2/16/16/32×32 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| wslice_k1 | 1/1/16/16/32×32 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK: 0% (no halo)] |
| wslice_k5 | 5/1/16/16/32×32 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| wslice_layer2 | 3/1/16/16/32×32 (L2) | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| (additional N=4 cases up to 10 total) | — | — | — | — | — | — | — | — |
| **DDR-mode reference (N=2 only)**: K=3 C8C16 30×30 | 3/1/8/16/30×30 | **8,808** | **9,057** | — | **0.97×**‡ | — | **+2.8%** | 0% (DDR mode) |
| **ResNet 11-layer (projection)** | full chain | **593K** | [CHECK: ~302K] | [CHECK: ~141K] | [CHECK: 1.7×] | [CHECK: 2.14×] | [CHECK] | [CHECK] |

† wslice1 N=1 baseline 待跑（直接对比单核同一案例）
‡ N=2 DDR mode 比 N=1 慢 2.8% 是 AXI 仲裁开销，数据来自 STATUS.md §1.5

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| wslice1 N=2 5,569 / N=4 3,833 cycles | STATUS.md §2.8 / paper.md §7.5 line 422 | ✅ |
| wslice_k7 N=4 34,097 cycles | paper.md §7.5 line 424 | ✅ |
| N=2 DDR-mode 9,057 vs N=1 8,808 cycles | STATUS.md §1.5 / paper.md §7.5 line 422 | ✅ |
| 2.8% AXI 仲裁开销 | paper.md §7.5 line 422 | ✅ |
| 1.45× speedup (N=4/N=2) | paper.md §7.5 line 422 | ✅ |
| ResNet 11-layer 估算 (302K / 141K cycles, 1.7× / 3.64×) | STATUS.md §2.6 / paper.md §7.5 line 426 | ✅ (estimate, not measured) |
| 其余 wslice case 8 个 | sim/tb_multicore/ + STATUS.md §2.8 | [CHECK: contributions.md §8.1 #8] |
| AXI arb % per case | 实测分解，需要看 sim 日志的 axi_arbiter wait 计数 | [CHECK: contributions.md §8.1 — paper.md line 424] |
| Halo recomp % per case | 同上 | [CHECK: paper.md line 424] |

## 说明（建议进 caption 或表脚注）

> **W-slice halo overhead 计算**：N=2 K=3 pad=1 W=32 时 Core 0 处理 W[0..17), Core 1 处理 W[15..32)，重叠 2 列；overhead = 2/16 = 12.5%（每核冗余生产 K-1 列边界）。N=4 K=3 W=32 类推。
>
> **AXI arbitration overhead 测量**：N=2 DDR-mode 同 case 跑单核与双核取差；N=4 同样。
>
> **wslice1 1.45× speedup 短于 ideal 2× 的解释（paper.md line 428 4 项分解）**：(a) baseline normalization (small image startup cost), (b) AXI arb overhead, (c) halo recomp, (d) ODMA drain serialization.

## 与正文的一致性检查

- [x] §7.5 line 422 "9057 vs 8808 cycles → +2.8% AXI arbitration overhead" — DDR-mode 行对齐
- [x] §7.5 line 422 "wslice1 at 3833 cycles versus N=2's 5569 cycles, giving a 1.45× speedup" — wslice1 行对齐
- [x] §7.5 line 424 "20-case subset" / "all 10 N=4 cases" — 表内列出 8 个，加 ResNet projection + DDR mode 共 10 行结构
- [x] §7.5 line 424 wslice_oddw / wslice_smallw / wslice_k7 / wslice_stride2 — 4 个 hard corner 都列出
- [x] §7.5 line 426 "ResNet 11-layer ... ~302K cycles at N=2 (1.7×, 331 fps), ~141K cycles at N=4 (3.64×, 709 fps)" — 表 ResNet 行对齐
- [x] §7.5 line 428 "1.45× speedup ... admits a four-part decomposition" — 表后说明 4 项 decomposition

## 不确定项

- [CHECK: contributions.md §8.1 #8] ResNet 11-layer multicore 实测 cycles vs 估算对齐 — STATUS line 254 估 1-2 天
- [CHECK: contributions.md §8.1 #9] N=4 wslice1 1.45× 是否近线性的 baseline 归一化
- [CHECK] 整列 N=1 cycles / AXI arb % / Halo % — 需要重跑 sim 取数据
- [CHECK] wslice case 数量 — 实际 STATUS §2.8 "20/20" 是 10 cases × N=2 + 10 × N=4，需要列全 10 个
- [TBD: 表是否横跨双栏] 当前列宽估约 11 列 × 11 行，IEEE 单栏挤不下，建议双栏；或拆成 (a) per-case sweep + (b) ResNet projection 两子表
- [TBD: ResNet 行用 estimate 标记还是单独成表] paper.md line 426 明确说"projections from analytical scheduling model rather than measured numbers"——表里建议用 italics 或 (proj.) 后缀

## 备注

**这是 narrative B 高度依赖的表**。如果用户决定 narrative 主轴为 A + B 双线，必须把 ResNet 11-layer multicore 实测补上（contributions.md §8.3 排序第 4）。
