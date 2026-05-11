# Tab.4: BRAM Breakdown — Single-Core Module-Level + Multi-Core Projection

## 在 paper.md 中的引用位置
- **首次引入**：§7.3 Resource Utilization and Fmax, line 396
  - 原文：`Table 4 takes this single-core breakdown and projects it across N=1, N=2, N=3, and N=4 cores against the XC7K325T's 445-BRAM ceiling, which is what makes the multi-core scaling story of §7.5 numerically pinned rather than hand-waved.`
- **被引用次数**：1 次显式

## 论证作用
**narrative B 的硬约束证据表**——证明 multi-core scaling 的 binding constraint 是 BRAM 而不是 LUT/interconnect。"3 cores is the hard ceiling on XC7K325T without shrinking any per-core SRAM" 这个论点的数据支撑就在这张表。

## 列定义
| 列名 | 含义 |
|------|------|
| Module / N value | 模块名（左半）或核数（右半）|
| BRAM36 count | BRAM36 块数 |
| % of 445 BRAM ceiling | 占 XC7K325T BRAM 上限百分比 |
| Notes | 备注 / 容量 |

## 表初稿（双子表合并）

### Part (a): Single-core BRAM breakdown

| Module | SRAM Geometry | BRAM36 | % of 445 | Notes |
|--------|---------------|--------|----------|-------|
| WB SRAM (`wgt_buffer`) | 1024 × 2048 b | **57** | 12.8% | Weight buffer, dominant |
| IFB SRAM (`line_buffer`, ring) | 8192 × 128 b | **32** | 7.2% | Input feature ring |
| Shortcut Bank (`sdp` residual) | 8192 × 128 b | **32** | 7.2% | R.2 refactor |
| OFB SRAM (`ofb_writer`, ring) | 2048 × 128 b | **7 + 1 RAMB18** | 1.6% | + 1 half-block |
| Misc (parf_col + bias_rf + others) | per-col SRAM × 16, etc. | [CHECK: 1] | 0.2% | Per-col PARF inferred to dist RAM |
| **Single-core total** | — | **128 + 1 RAMB18** | **28.8%** | STATUS.md §1 |

### Part (b): Multi-core projection vs 445-BRAM ceiling

| N | LUT (% of 203,800) | BRAM36 (% of 445) | Status |
|---|--------------------|-------------------|--------|
| **N=1 (single core)** | 36,942 (18.1%) | **128** (28.8%) | ✅ Synthesized + measured |
| **N=2** | 74,386 (36.5%) | **256** (57.5%) | ✅ Synthesized + measured |
| **N=3 (projection)** | ~109K (54%) | **384** (86%) | ⚠ At edge of device |
| **N=4 (projection, default cfg)** | ~146K (72%) | **512** ❌ (>445) | ❌ Exceeds ceiling |
| **N=4 (with shortcut bank shrink 8192→2048)** | ~146K (72%) | **416** (93.5%) | ✅ Fits with reduced shortcut |

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| WB SRAM 57 BRAM | STATUS.md §1 (BRAM 明细) + paper.md §7.3 line 396 | ✅ |
| IFB / Shortcut 32 BRAM each | 同上 | ✅ |
| OFB 7+1 RAMB18 | 同上 | ✅ |
| Misc parf_col 数字 | [CHECK: contributions.md §8.1 #7 — parf BRAM 数量贡献单独估算] | [CHECK] |
| N=1 36,942 LUT / 128 BRAM | STATUS.md §1 | ✅ |
| N=2 74,386 LUT / 256 BRAM | STATUS.md §2 / paper.md §7.3 line 398 | ✅ |
| N=3 / N=4 推算 | paper.md §7.3 line 398 | ✅ (推算非实测) |
| N=4 with shrink 8192→2048 → 416 | paper.md §7.3 line 398 (4×104=416) | ✅ |
| 445 BRAM ceiling | XC7K325T 数据手册 | ✅ |

## ASCII 示意（堆叠条形图风格，可作为辅助 Fig）

```
BRAM
500 │ ╳╳╳╳╳╳╳╳╳╳╳╳╳╳╳╳╳ ── 445 ceiling ─────────────────
    │
400 │                              ┌─OFB──┐
    │                              │ 7+1  │
    │                              ├──────┤
    │              ┌─OFB──┐        │ Sct  │
    │              │ 7+1  │        │ Bank │
300 │              ├──────┤        │  32  │
    │              │ Sct  │        ├──────┤
    │              │ Bank │        │ IFB  │
    │              │  32  │        │  32  │
    │  ┌─OFB──┐    ├──────┤        ├──────┤
200 │  │ 7+1  │    │ IFB  │        │ WB   │
    │  ├──────┤    │  32  │        │  57  │
    │  │ Sct  │    ├──────┤        ╞══════╡
    │  │  32  │    │ WB   │        │ ✕ 4  │ ← ❌ 512 > 445
100 │  ├──────┤    │  57  │        │ cores│   needs shrink
    │  │ IFB  │    ╞══════╡        │      │
    │  │  32  │    │ ✕ 2  │        │      │
    │  ├──────┤    │ cores│        │      │
    │  │ WB   │    │ =256 │        │      │
  0 │  │  57  │    │      │        │      │
    └──┴──────┴────┴──────┴────────┴──────┴──────────
       N=1       N=2          N=3 (~384)    N=4 (~512)
       128       256          ⚠at edge       ❌ exceeds
```

## 初版代码（matplotlib，stacked bar，简单可用）

```python
import matplotlib.pyplot as plt
import numpy as np

cores = ['N=1', 'N=2', 'N=3 (proj)', 'N=4 (proj)\ndefault', 'N=4 (proj)\nshortcut shrink']
wb_bram = [57, 114, 171, 228, 228]
ifb_bram = [32, 64, 96, 128, 128]
sct_bram = [32, 64, 96, 128, 32]   # shrunk to 8 each in last column
ofb_bram = [8, 16, 24, 32, 32]
totals = [n[0]+n[1]+n[2]+n[3] for n in zip(wb_bram, ifb_bram, sct_bram, ofb_bram)]

fig, ax = plt.subplots(figsize=(7, 4.5))
x = np.arange(len(cores))
bottom = np.zeros(len(cores))
for label, vals, color in [
    ('WB (57/core)', wb_bram, '#4C72B0'),
    ('IFB (32/core)', ifb_bram, '#55A868'),
    ('Shortcut (32 or 8/core)', sct_bram, '#C44E52'),
    ('OFB (8/core)', ofb_bram, '#8172B2'),
]:
    ax.bar(x, vals, bottom=bottom, label=label, color=color, edgecolor='black')
    bottom += np.array(vals)

ax.axhline(445, linestyle='--', color='gray', linewidth=2, label='XC7K325T ceiling (445)')
ax.set_xticks(x)
ax.set_xticklabels(cores, fontsize=9)
ax.set_ylabel('BRAM36 count')
ax.set_title('BRAM Breakdown vs Multi-Core Scaling on XC7K325T')
ax.legend(loc='upper left', fontsize=8)
ax.set_ylim(0, 600)

# Annotate totals
for xi, t in zip(x, totals):
    ax.text(xi, t + 8, str(t), ha='center', fontsize=9, fontweight='bold')

plt.tight_layout()
plt.savefig('tab4-or-fig-bram-breakdown.pdf', bbox_inches='tight')
```

## 与正文的一致性检查

- [x] §7.3 line 394 "128 BRAM36 plus 1 RAMB18" — Part (a) 总计一致
- [x] §7.3 line 396 "WB 57 / IFB 32 / Shortcut 32 / OFB 7+1" — 全部对齐
- [x] §7.3 line 398 "N=2 74,386 LUT (36.5%) / 256 BRAM (57.5%)" — Part (b) 一致
- [x] §7.3 line 398 "N=3 estimate of 384 BRAM (86%)" — 一致
- [x] §7.3 line 398 "N=4 at 512 BRAM exceeds 445" — 一致
- [x] §7.3 line 398 "shortcut bank reduction from 8192 to 2048 entries (cutting 24 BRAM per core, giving 4×104=416 < 445)" — 一致
- [x] §7.3 line 400 "three cores is the hard ceiling" — Part (b) N=3 ⚠ + N=4 ❌ 标记直接支持

## 不确定项

- [CHECK: contributions.md §8.1 #7] Misc / parf_col BRAM 数字未明确，论文当前 128 总数没分给 parf。**建议** Part (a) 加 `[CHECK]` 标记，由 project-analyst 看综合 cell utilization 报告补
- [CHECK: 资源数字在投稿前重综合刷新] paper.md line 394 已标注。如果 polisher 阶段加了 `(* use_dsp *)` 重综合，整张表数字要重出
- [TBD: 是否合并 Part (a) + Part (b) 为单表] 双子表当前清晰；如果 IEEE 排版压力，可合并为"模块×N=1..4"网格
