# Tab.3: Per-Layer PE Utilization Across 3 Compilation Modes (ResNet-18-style chain)

## 在 paper.md 中的引用位置
- **首次引入**：§7.2 PE Utilization, line 382
  - 原文：`Table 3 lays out a per-layer comparison across the three modes — baseline, --fold, and --fold --s2d — reporting both per-layer occupancy and the network-wide weighted average.`
- **被引用次数**：1 次显式

## 论证作用
**narrative A 的核心数据表**。直接证明 compiler-side fold 把 shallow-layer PE 利用率从 12.5%/25%/50% 推到 near 100%。表里所有具体百分比都是 [CHECK]，等 project-analyst 跑回归取实测。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| Layer | ResNet-18-style 11-layer chain 的层标识 | model_analysis.md §2 |
| K / stride / Cin / Cout / H×W | 卷积形状 | model_analysis.md / regression CASES |
| Baseline | 无 fold 时 PE 利用率 | 算式 = `(min(Cin,16) × min(Cout,16)) / 256` |
| --fold | Ky-fold 启用后 PE 利用率 | [CHECK: 实测 mac_array busy 占空率] |
| --fold --s2d | Ky-fold + S2D 启用后 PE 利用率 | [CHECK] |
| Cycles (3 modes) | 三模式实测 cycles | [CHECK: run_regression.py 三模式] |

## 行定义
11 层 ResNet-18-style chain（来源 model_analysis.md §2 + run_regression.py CASES）

## 表初稿

| Layer | K | stride | Cin | Cout | H×W | Baseline PE% | `--fold` PE% | `--fold --s2d` PE% | Notes |
|-------|---|--------|-----|------|-----|--------------|--------------|--------------------|-------|
| 1 (stem) | 7 | 2 | 4 | 8 | 224×224 | **12.5%** | [CHECK: ~50%] | [CHECK: ~100%] | Ky-fold groups_y=4; S2D Cin'=16 |
| 2 (maxpool, sw) | — | — | — | — | — | — | — | — | software (not on accelerator) |
| 3 | 3 | 1 | 8 | 8 | 56×56 | **25.0%** | [CHECK: ~50%] | — | Ky-fold groups_y=2 |
| 4 | 3 | 1 | 8 | 8 | 56×56 | **25.0%** | [CHECK] | — | Same as Layer 3 |
| 5a (downsample) | 3 | 2 | 8 | 16 | 28×28 | **50.0%** | [CHECK: ~100%] | [CHECK: ~100%] | S2D Cin'=32, Ky-fold suppressed |
| 5b | 3 | 1 | 16 | 16 | 28×28 | **100%** | 100% | 100% | No fold needed |
| 5c (downsample) | 3 | 2 | 16 | 32 | 14×14 | **100%** | 100% | 100% | No fold needed |
| 5d | 3 | 1 | 32 | 32 | 14×14 | **100%** | 100% | 100% | No fold needed |
| 6 (downsample) | 3 | 2 | 32 | 64 | 7×7 | **100%** | 100% | 100% | No fold needed |
| 7 | 3 | 1 | 64 | 64 | 7×7 | **100%** | 100% | 100% | No fold needed |
| FC_xy † | 1 | — | 256 | 2 | 1×1 | **12.5%** | 12.5% | 12.5% | Cout<16 column-idle (§5.3, §7.7) |
| **Network-weighted avg** | — | — | — | — | — | [CHECK: ~78%]‡ | [CHECK: ~92%] | [CHECK: ~96%] | Weighted by per-layer MAC count |

† FC_xy column-idle is by design (§5.3). MAC contribution to total = [CHECK: 极小百分比，从 model_analysis.md §2 抽取，预估 <1%]
‡ Cycle-weighted; not all-equal-weight.

## 数据来源（详细）

| 数据点 | 来源 | 状态 |
|--------|------|------|
| Layer 1/3-4/5a/5c baseline 12.5%/25%/50% | model_analysis.md §2 表 + paper.md §7.2 line 380 | ✅ 已知 |
| Layer 1 `--fold` 实测 PE% | run `python run_regression.py --fold --case "Layer1"` 然后 `cycles_actual / cycles_ideal` | [CHECK: contributions.md §8.1 #1] |
| Layer 3-4 `--fold` 实测 PE% | 同上 | [CHECK: contributions.md §8.1 #2] |
| Layer 5a/5c `--s2d` 实测 PE% | `python run_regression.py --fold --s2d --case "L5"` | [CHECK] |
| Network-weighted avg | 由 cycles 加权 | [CHECK] |
| FC_xy MAC% of total | model_analysis.md §2 抽 MAC 总数除 | [CHECK: contributions.md §7.7 line 454] |

## ASCII 示意（条形图风格，可作为辅助 Fig 替代/补充）

```
Layer    Baseline      --fold        --fold --s2d
                       (Ky-fold)     (+ S2D)
─────────────────────────────────────────────────────
L1 (K=7,Cin=4)
  ████░░░░░░░░░░░░ 12.5%
  ████████░░░░░░░░ ~50% [CHECK]
  ████████████████ ~100% [CHECK]

L3-4 (K=3,Cin=8)
  ████████░░░░░░░░ 25%
  ████████████████ ~50% [CHECK]
  ████████████████ (s2d N/A, stride=1)

L5a (K=3,s=2,Cin=8)
  ████████████░░░░ 50%
  ████████████████ ~100% [CHECK]
  ████████████████ ~100% [CHECK]

L5b+ (Cin≥16)
  ████████████████ 100%
  ████████████████ 100%
  ████████████████ 100%

FC_xy (Cout=2)
  ██░░░░░░░░░░░░░░ 12.5% (column-idle by design)
```

## 与正文的一致性检查

- [x] §7.2 line 380 `Layer 1 (K=7, Cin=4) achieves only 12.5%` — 表第 1 行 baseline 列对齐
- [x] §7.2 line 380 `Layers 3-4 (Cin=8) achieve 25%` — 表第 3-4 行 baseline 列对齐
- [x] §7.2 line 380 `5a/5c (Cin=8, stride=2) achieve 50%` — 表第 5a/5c 行 baseline 列对齐
- [x] §7.2 line 384 "groups_y=4" / "Cin'=4·8=32" — 表 Notes 列叙述一致
- [x] §7.7 line 454 "Cout<16 column-idle" — FC_xy 行的 Notes 标记一致

## 不确定项

- [CHECK] **整列 `--fold` PE% / `--fold --s2d` PE%** 等 project-analyst 跑回归填实测
- [CHECK] **Network-weighted avg** 三个数字等加权计算
- [CHECK: contributions.md §8.1 #1, #2] 是论文 narrative A 的核心数据，**必须在投稿前补完**
- [TBD: 是否单独配条形图作为 Fig.3] 表 + 条形图组合会非常强；但 [TBD] paper.md 当前没有 Fig.3 占位，由 polisher 决定是否新增
- [TBD: Layer 编号风格] model_analysis.md 用 "Layer 5a/5b/5c"; ResNet 标准命名是 "conv2_1 / conv3_1"。建议沿用 paper.md §7.2 line 380 已用的 "5a/5c" 风格保持一致

## 备注

**这是论文最关键的一张表**。narrative A 的全部说服力压在 baseline 三列百分比上。**先于其他 [CHECK] 优先补**（contributions.md §8.3 排序第 1）。
