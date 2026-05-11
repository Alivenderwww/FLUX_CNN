# Tab.2: Compiler-Only Fold vs. Hardware-Reconfigurable Arrays — 4-Axis Comparison

## 在 paper.md 中的引用位置
- **首次引入**：§5.4 Comparison with Hardware-Reconfigurable Alternatives, line 292
  - 原文：`We compare the two routes along four axes — hardware complexity, compiler complexity, memory cost, and applicable scenarios — summarized in Table 2 [CHECK: Table 2 编号待终稿确认].`
- **被引用次数**：1 次显式

## 论证作用
narrative A 的核心 design-axis 表，明确 FLUX_CNN compiler-only 路线 vs 硬件可重构（MAERI / Eyeriss-v2）路线沿 4 个 axis 上的位置。**避免主张"主导优势"**——表格框架强调 trade-off。

## 列定义
| 列名 | 含义 |
|------|------|
| Axis | 4 个比较维度 |
| Compiler-only (FLUX_CNN) | 本工作 |
| Hardware-Reconfigurable (MAERI / Eyeriss-v2) | 对照组 |

## 行定义（即 4 个 axis）
| Axis | 数据来源 | 置信度 |
|------|----------|--------|
| Hardware complexity | paper.md §5.4 line 292 + literature.md MAERI/Eyeriss-v2 | ✅ |
| Compiler complexity | paper.md §5.4 + Timeloop/MAESTRO refs | ✅ |
| Memory cost | paper.md §5.4 + docs/pe-fold.md | ✅ |
| Applicable scenarios | paper.md §5.4 末段 | ✅ |

## 表初稿（Markdown，可直接嵌入 paper.md）

| Axis | Compiler-only (FLUX_CNN) | Hardware-Reconfigurable (MAERI [Kwon@ASPLOS'18] / Eyeriss-v2 [Chen@JETCAS'19]) |
|------|--------------------------|-------|
| **Hardware complexity** | Fixed 16×16 broadcast-and-systolic INT8 MAC grid; no reconfigurable interconnect | Reconfigurable distribution + reduction networks (e.g. ART tree, hierarchical mesh NoC) added at synthesis cost |
| **Compiler complexity** | Must decide and apply Ky-fold + S2D transforms per layer (~few hundred LoC Python) | Conventional dataflow mappings via Timeloop / MAESTRO-style search [Parashar@ISPASS'19][Kwon@MICRO'20] |
| **Memory cost** | Ky-fold inflates input by `groups_y` (typically 4–8); S2D is in-place permutation (no inflation) | No compiler-side inflation; memory cost of reconfigurable interconnect is silicon area not data footprint |
| **Applicable scenarios** | Best for ahead-of-time-compiled deployments where the compiler has full visibility into layer dimensions | Better for dynamically-shifting workloads where layer geometry changes at runtime |

## 与正文的一致性检查

- [x] §5.4 line 292 4 axis 命名完全一致
- [x] MAERI / Eyeriss-v2 / Timeloop / MAESTRO 引用方式与 paper.md 一致
- [x] "Ky-fold inflates input by `groups_y`" — 与 §5.1 line 260 一致
- [x] "S2D is in-place" — 与 §5.2 line 270 "DDR-bandwidth-neutral" 一致

## 不确定项

- [CHECK: Table 2 编号待终稿确认] paper.md line 292 已标注 — 编号在 polisher 阶段统一定下
- [TBD: 是否单独将 im2col 作为第 3 列] paper.md §5.4 line 292 提到 "im2col-style approaches inflate input by K²"——可考虑加 cuDNN 列作 worst-case extreme reference。**建议加**，让读者看到 "FLUX_CNN 在两个极端（fixed array baseline / full im2col）之间的 sweet spot"
- [TBD: 表是否横排] 4 行 × 2 列适合纵排；如果加 im2col 列变 4 行 × 3 列，仍能在 IEEE 单栏内放下

## 增强版（推荐，加 im2col 列）

| Axis | Fixed Array (baseline) | Compiler-only Fold (FLUX_CNN) | Full im2col [cuDNN] | Hardware-Reconfigurable (MAERI / Eyeriss-v2) |
|------|------------------------|-------------------------------|---------------------|----------------------------------------------|
| **Hardware complexity** | Minimal (fixed grid) | Minimal (same as baseline) | Minimal (GEMM core) | High (reconfigurable distribution + reduction NoC) |
| **Compiler complexity** | Trivial (loop emit) | Moderate (Ky-fold + S2D decision logic, ~400 LoC) | Heavy (im2col materialization) | Search-based mapping (Timeloop / MAESTRO) |
| **Memory cost** | None added | `groups_y`× IFB inflation (Ky-fold), in-place (S2D) | `K²`× input inflation | None (paid in silicon area) |
| **Applicable scenarios** | Layers with `Cin ≥ HW_PE` only | Pre-trained, ahead-of-time-compiled CNN inference | DRAM-rich, large-buffer GPU/TPU | Dynamically-varying workloads |
