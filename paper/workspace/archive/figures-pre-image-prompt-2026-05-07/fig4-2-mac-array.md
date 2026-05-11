# 图 4.2 MAC 阵列模块结构图
# Figure 4.2 MAC-array module structure

## 在论文中的角色
- 首次引入：§4.3 MAC 阵列模块
- 引用位置：paper.md 第 296、298 行
- 论证作用：直观展示加速核的算力核心 — 16 列 × 16 PE 二维结构、列方向广播激活、列内沿 cin 累加、加法树规约、PSUM 输出。读者应理解 256 个 INT8 MAC 的二维拓扑与三层数据复用映射。

## 图类型
阵列结构图。

## 设计要素

### 必含元素
1. **16 列 × 16 PE 阵列**：核心 2D 网格；每个格子是一个 INT8 MAC 单元。可用 4×4 缩略示意 + "..." 表 16×16。
2. **列方向广播激活**：顶部一根横向粗箭头，每列接收同一拍激活向量 16 元（沿 cin 维度）。
3. **每列权重**：每列顶部上方标 "WRF[col]"，画一个小寄存器堆指向该列。
4. **列内加法树**：每列底部一棵小加法树，将 16 个 PE 的乘积规约为 1 路 PSUM。
5. **16 路 PSUM 输出**：底部 16 根箭头朝下指向 parf_accum。
6. **三层并行度标注**：Cout 维 = 16 列、Cin 维 = 16 PE、Kx/Ky 顺序展开。

### 标注要求
- 每列对应一个输出通道：标 "col 0 → cout[0]"、"col 1 → cout[1]"...
- 列内 PE：标 "PE 0..15 along Cin"
- 单 PE = INT8 × INT8 → 16-bit → 加法树 → 32-bit PSUM
- 标注峰值算力 256 ops/cy × Fmax

### 视觉层次
- 主角：2D 16×16 阵列 + 顶部激活广播 + 底部 16 路 PSUM 输出
- 配角：每列侧的 WRF 块、加法树
- 背景：cfg 输入、本地计数器 (*k_x*, *k_y*, *cin_pe*)

## 数据来源
- paper.md §4.3
- docs/modules/mac_array.md
- contributions.md C1.1（数据流）

## ASCII 示意稿

```
   from line_buffer / ARF      from wgt_buffer (WRF×16)
        │ INT8 vec[16]                  │ INT8 weights
        │ (cin)                         │ (16 cols × 32 depth)
        ▼                               ▼

   ━━━━━━━━━━━━ activation broadcast across 16 cols ━━━━━━━━━━━━
        │       │       │       │       │       │       │
        │       │       │       │       │       │       │
   col0 │  col1 │  col2 │  col3 │ ...   │ col15 │
   ┌─▼─┐ ┌─▼─┐ ┌─▼─┐ ┌─▼─┐               ┌─▼─┐
   │PE0│ │PE0│ │PE0│ │PE0│   ...         │PE0│   ─┐
   ├───┤ ├───┤ ├───┤ ├───┤               ├───┤    │ along Cin
   │PE1│ │PE1│ │PE1│ │PE1│               │PE1│    │ (16 PEs)
   ├───┤ ├───┤ ├───┤ ├───┤               ├───┤    │
   │ : │ │ : │ │ : │ │ : │               │ : │    │
   ├───┤ ├───┤ ├───┤ ├───┤               ├───┤    │
   │PE15│ │PE15│ │PE15│ │PE15│           │PE15│  ─┘
   └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘               └─┬─┘
   ┌─▼─┐ ┌─▼─┐ ┌─▼─┐ ┌─▼─┐               ┌─▼─┐
   │adder│ │adder│ │adder│ │adder│ ...   │adder│   adder tree per col
   │tree │ │tree │ │tree │ │tree │       │tree │   (16-input)
   └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘               └─┬─┘
     │     │     │     │                   │
     │     │     │     │                   │
     ▼     ▼     ▼     ▼                   ▼
   PSUM[0] PSUM[1] PSUM[2] PSUM[3]      PSUM[15]   (32-bit each)
            │
            ▼  16-way PSUM to parf_accum (v/r)

   Annotations:
   - 16 cols × 16 PEs = 256 INT8 MACs
   - peak: 256 ops/cy × Fmax
   - dataflow: weight-stationary (WRF) + activation broadcast + col-cin parallel
   - kx/ky unrolled sequentially over cycles
```

## 与正文的一致性检查
- [x] §4.3 "16 列 × 16 PE 共 256 个 INT8 乘加单元" — 图中明示
- [x] §4.3 "每列对应一个输出通道；列内沿 Cin 方向并行" — 图中标注
- [x] §4.3 "每周期由 ARF 广播 1 拍激活值向量到 16 列" — 顶部水平广播线
- [x] §4.3 "列内 16 路乘积经一棵加法树规约为 1 路 PSUM" — 每列底部加法树

## 不确定项
- [TBD: PE 单元内部画乘法器 + 累加寄存器，还是只画一个抽象 MAC 框] — 倾向后者保持简洁，乘法器位宽在图注说明
