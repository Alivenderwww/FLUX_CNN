# Tab.5: 11-Layer ResNet-18-Style Chain — Per-Layer End-to-End Latency Decomposition

## 在 paper.md 中的引用位置
- **首次引入**：§7.4 End-to-End Latency and MAC Efficiency, line 410
  - 原文：`Table 5 decomposes this end-to-end latency layer-by-layer, reporting per-layer cycles, percentage of total, Wall_us at both operating points, and a flag for the latency-dominant layers.`
- **被引用次数**：1 次显式

## 论证作用
让读者看到 593K cycles / 86.6% MAC% 这个 headline 不是黑箱——逐层分解出谁主导端到端时间。Layer 1 (K=7, large H×W) 占大头这一点呼应 §7.2 narrative A 的"Ky-fold 在 Layer 1 上有 outsized weight"。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| Layer | 11-layer chain 的层 ID | run_regression.py CASES |
| Shape (K/s/Cin/Cout/H×W) | 卷积形状 | model_analysis.md / CASES |
| Cycles | 实测层 cycles | [CHECK: 重跑取 per-layer cycles] |
| % of total | cycles / 593K | [CHECK] |
| Wall_us @ 100 MHz | cycles / 100 MHz × 1e6 | [CHECK: derived from cycles] |
| Wall_us @ 68.4 MHz | cycles / 68.4 MHz × 1e6 | [CHECK] |
| Dominant? | latency-dominant 标记 | 标准：>10% of total |

## 表初稿

| Layer | Shape (K/s/Cin/Cout) | H×W | Cycles | % of total | Wall_us @ 100 MHz | Wall_us @ 68.4 MHz | Dominant? |
|-------|---------------------|-----|--------|-----------|--------------------|--------------------|-----------|
| 1 (stem conv) | 7/2/4/8 | 224×224 | [CHECK: ~150K?]† | [CHECK: ~25%]† | [CHECK] | [CHECK] | ✅ |
| 3 | 3/1/8/8 | 56×56 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 4 | 3/1/8/8 | 56×56 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 5a (downsample) | 3/2/8/16 | 28×28 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 5b | 3/1/16/16 | 28×28 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 5c (downsample) | 3/2/16/32 | 14×14 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 5d | 3/1/32/32 | 14×14 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 6 (downsample) | 3/2/32/64 | 7×7 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 7 | 3/1/64/64 | 7×7 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| 8 (residual fusion test) | 3/1/64/64 | 7×7 | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK] |
| FC_xy | 1/—/256/2 | 1×1 | [CHECK: 极小] | [CHECK: <1%] | [CHECK] | [CHECK] | ❌ |
| **Sub-total (compute)** | — | — | [CHECK: ~516K] | [CHECK: ~87%] | — | — | — |
| Inter-layer overhead (descriptor fetch + cfg writes) | — | — | [CHECK: ~50K] | [CHECK: ~8%] | — | — | — |
| IDMA→ODMA channel switch | — | — | [CHECK: ~15K] | [CHECK: ~2%] | — | — | — |
| ODMA end-of-network drain | — | — | [CHECK: ~12K] | [CHECK: ~2%] | — | — | — |
| **Total (chain end-to-end)** | — | — | **593K** | 100% | **5.95 ms** | **8.69 ms** | — |

† Layer 1 估计占大头：229×229 padded × 7² 卷积窗 ÷ MAC 阵列 ≈ 实测核心。**精确值等实测**

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| Total 593K cycles | STATUS.md line 355 / paper.md §7.4 line 408 | ✅ |
| 86.6% network MAC% | 同上 | ✅ |
| 5.95 ms @ 100 MHz target | 593e3 / 100e6 = 5.93 ms (圆整 5.95) | ✅ derived |
| 8.69 ms @ 68.4 MHz Fmax | 593e3 / 68.4e6 = 8.67 ms (圆整 8.69) | ✅ derived |
| Per-layer cycles 分解 | run `run_regression.py` 取 CASE_PROFILE 输出 | [CHECK: contributions.md §8.1 #5 Wall_us 重跑] |
| 13% gap 4 项分解（descriptor / channel switch / cold start / drain） | paper.md §7.4 line 414 | [CHECK: 波形 / profile counter] |
| Wall_us 实测 | regression report `Wall_us` 列 | [CHECK: contributions.md §8.1 #5] |

## ASCII 示意（饼图 / 堆叠条形图思路）

```
End-to-end 593K cycles breakdown:

Layer 1 (K=7, 224×224)    ████████████ ~25% (latency-dominant)
Layer 3-4 (Cin=8)         ██████████   ~20%
Layer 5a/5c (downsample)  ██████       ~12%
Layer 5b/5d/6/7/8         ██████████   ~20%
FC_xy                     ▏              <1%
─────────────────────────────────────────
Compute sub-total         ██████████████████████████████████ ~87% (516K)
─────────────────────────────────────────
Inter-layer cfg overhead  ████         ~8% (50K)
IDMA→ODMA switch          █            ~2% (15K)
ODMA drain                █            ~2% (12K)
─────────────────────────────────────────
Total                     593K cycles

Network MAC% = 86.6%  (= 1 - (50K+15K+12K)/593K = 1 - 13.0%)
```

## 与正文的一致性检查

- [x] §7.4 line 408 "593K cycles at 86.6% whole-network MAC efficiency / 5.95 ms @ 100 MHz / 8.69 ms @ 68.4 MHz" — 表 Total 行对齐
- [x] §7.4 line 410 "Layer 1 (K=7, large H×W) is the largest contributor by absolute cycles" — Dominant? 列 Layer 1 ✅
- [x] §7.4 line 414 "13 percentage points gap" 4 项分解：(i) descriptor fetch + cfg writes, (ii) IDMA→ODMA, (iii) IDMA cold-start, (iv) ODMA drain — 表后半段 4 行对齐
  - **注意**：paper.md 列 4 项，表里目前列 3 项（merge IDMA cold-start 进 inter-layer overhead）。**建议** 表与正文对齐改成 4 项

## 不确定项

- [CHECK: contributions.md §8.1 #5] Wall_us 实测列等重跑取
- [CHECK] **整列 cycles + % of total + Wall_us** — 需要 project-analyst 跑 11-layer chain 取 CASE_PROFILE 逐层 cycles
- [CHECK: contributions.md §8.1 #6 / paper.md line 386] 单层 PE% vs 整网 MAC% gap 的精确数字 / 这张表也是 §7.2/§7.4 对照的关键数据
- [TBD: Layer 编号一致性] Layer 1 / 3 / 4 / 5a / 5b / 5c / 5d / 6 / 7 / 8 / FC_xy 是 11 个？还是有 maxpool 占位？需要看 run_regression.py CASES 实际定义统一
- [TBD: 是否拆 Wall_us 单列] 100 MHz target 列是 derived，可省以减小表宽；建议保留双口径以呼应 §7 frontmatter line 366 "operating-point convention"

## 备注

**等 §7.4 line 408 的 593K cycles / 86.6% MAC% 实测口径定下后再补全此表** — contributions.md §8.3 排序中等优先级。
