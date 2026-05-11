# 图 4.2 MAC 阵列模块结构图
# Figure 4.2 MAC-array module structure

## 在论文中的角色
- 首次引入：§4.3 MAC 阵列模块（paper.md L296 / L298）
- 引用位置：paper.md L296 / L298
- 论证作用：直观展示加速核的算力核心 — 16 列 × 16 PE 二维结构、列方向广播激活、列内沿 cin 累加、加法树规约、PSUM 输出。读者应理解 256 个 INT8 MAC 的二维拓扑，以及 WS + 激活值复用 + 输出通道广播复合数据流的硬件落地形态。

## 图类型
阵列结构图（2D 网格 + 顶部广播 + 底部规约）。

## 设计要素

### 必含元素
1. **16 列 × 16 PE 阵列**：核心 2D 网格；每个格子是一个 INT8 MAC 单元。绘图时用 4 列 + "..." + 1 列示意，纵向用 4 行 + "..." + 1 行示意。
2. **顶部激活值广播总线**：横贯 16 列的粗箭头，每列接收同一拍激活向量。
3. **每列权重侧路（WRF×16）**：每列上方/侧方画一个小方框 "WRF[col] = 32"，由 wgt_buffer 提供。
4. **列内加法树**：每列底部一棵 16-input 加法树，将列内 16 个 PE 的乘积规约为 1 路 32 位 PSUM。
5. **16 路 PSUM 输出**：底部 16 根粗箭头朝下指向 parf_accum（valid-ready）。
6. **本地计数器（3 级）**：(k_x, k_y, cin_pe)，画在阵列左下角。
7. **三层并行度标注**：Cout 维 = 16 列、Cin 维 = 16 PE/列、Kx/Ky 顺序展开。

### 标注要求
- 单 PE：INT8 × INT8 → 16 b 乘积 → 加法树 → 32 b PSUM
- 峰值算力：256 ops/cy × Fmax（图注）
- 数据流：WS + activation broadcast + col-cin parallel
- 列对应：col i → cout[i]
- 行对应：PE j → cin[j]

### 视觉层次
- 主角：2D 16×16 阵列 + 顶部激活广播线 + 底部 16 路 PSUM
- 配角：每列侧 WRF、列内加法树
- 背景：cfg 输入、本地计数器

## 数据来源
- paper.md §4.3
- docs/modules/mac_array.md
- contributions.md C1 系列（数据流）
- CLAUDE.md（NUM_COL=NUM_PE=16, DATA=8, PSUM=32）

## ASCII 示意稿

```
   from line_buffer / ARF              from wgt_buffer
     │ INT8 vec[16] (cin) v/r            │ INT8 weights (16 cols × 32)
     ▼                                   ▼
   ━━━━━━━━━━━━━━ activation broadcast across 16 columns ━━━━━━━━━━━━━━
     │       │       │       │   ...   │           ┌───────┐
     │       │       │       │         │      ◀────│ WRF×16│
     ▼       ▼       ▼       ▼         ▼           │ (32)  │
   col0    col1    col2    col3      col15         └───────┘
   ┌───┐  ┌───┐  ┌───┐  ┌───┐         ┌───┐
   │PE0│  │PE0│  │PE0│  │PE0│   ...   │PE0│  ─┐
   ├───┤  ├───┤  ├───┤  ├───┤         ├───┤   │
   │PE1│  │PE1│  │PE1│  │PE1│         │PE1│   │ along Cin
   ├───┤  ├───┤  ├───┤  ├───┤         ├───┤   │ (16 PEs / col)
   │ : │  │ : │  │ : │  │ : │         │ : │   │
   ├───┤  ├───┤  ├───┤  ├───┤         ├───┤   │
   │PE15│ │PE15│ │PE15│ │PE15│        │PE15│ ─┘
   └─┬─┘  └─┬─┘  └─┬─┘  └─┬─┘         └─┬─┘
   ┌─▼─┐  ┌─▼─┐  ┌─▼─┐  ┌─▼─┐         ┌─▼─┐
   │add│  │add│  │add│  │add│   ...   │add│   16-input
   │tree│ │tree│ │tree│ │tree│        │tree│  adder tree per col
   └─┬─┘  └─┬─┘  └─┬─┘  └─┬─┘         └─┬─┘
     ▼      ▼      ▼      ▼              ▼
   PSUM[0] PSUM[1] PSUM[2] PSUM[3] ... PSUM[15]   (32-bit each)
     │       │       │       │           │
     └───────┴───────┴───────┴── 16-way ─┘
                     │
                     ▼  to parf_accum (v/r)

   ┌──────────────────────┐
   │ local counters       │   (k_x, k_y, cin_pe)
   │ 3-level              │   bounds from cfg_regs
   └──────────────────────┘

   16 cols × 16 PEs = 256 INT8 MACs   |   peak: 256 ops/cy × Fmax
   dataflow: weight-stationary (WRF) + act broadcast + col-cin parallel
   kx / ky unrolled sequentially over cycles
```

## 与正文的一致性检查
- [x] §4.3 "16 列 × 16 PE 共 256 个 INT8 乘加单元" — 图中明示
- [x] §4.3 "每列对应一个输出通道；列内沿 Cin 方向并行" — 图中标注
- [x] §4.3 "每周期由 ARF 广播 1 拍激活值向量到 16 列" — 顶部水平广播总线
- [x] §4.3 "列内 16 路乘积经一棵加法树规约为 1 路 PSUM" — 每列底部加法树
- [x] §4.3 "三个维度并行度（Cout=16 列 / Cin=16 PE / Kx/Ky 顺序）" — 图注

## 不确定项
- [TBD: PE 单元内部画乘法器 + 累加寄存器，还是只画一个抽象 MAC 框] — 倾向后者保持简洁，乘法器位宽在图注说明

## image 生成提示词

### 中文版

科研论文配图，**MAC 阵列模块（mac_array）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图（16×16 阵列天然为正方形）。**

**版式（中央 2D 阵列 + 顶部广播 + 右侧 WRF + 底部规约）**：
- **顶部（激活值广播）**：水平贯穿一根粗黑色实线箭头，从左侧 "from line_buffer / ARF"（淡橙色 CMYK 0/30/30/0 小方框）伸出，覆盖 16 列宽度，标 "activation broadcast: INT8 vec[16] (cin)"。
- **右上侧（权重侧路）**：画一个小方框 "WRF×16 (32 each)"（淡橙色填充），用细实线箭头水平连入阵列顶部，标 "weight stationary"。
- **中央（主角，2D 阵列）**：画 16 列 × 16 行的网格（实际绘图用 5 列 + "..." 表示 16 列，5 行 + "..." 表示 16 行）。每个格子是一个小方框 "PE"（淡蓝色 CMYK 30/10/0/0 填充、黑色细边框），格子内不写文字。**列方向**用浅灰色虚线分隔每一列；**行方向**在最左列外侧标 "PE 0 / PE 1 / ... / PE 15 along Cin"（竖排小字）；**列方向**在最顶行外侧标 "col 0 / col 1 / ... / col 15 → cout"（横排小字）。
- **底部（加法树 + PSUM 输出）**：每列底部画一个小三角形 "adder tree"（淡黄色 CMYK 0/10/30/0 填充），向下伸出粗黑色箭头，标 "PSUM[i] (32 b)"。16 个 PSUM 箭头汇成下方一根总粗箭头，标 "16-way PSUM, valid-ready  →  to parf_accum"。
- **左下角**：画一个小方框 "local counters (k_x, k_y, cin_pe)"（淡灰色填充）。

**图注**：底部居中写两行小字 "16 cols × 16 PEs = 256 INT8 MACs   |   peak: 256 ops/cy × Fmax" 与 "Dataflow: weight-stationary + activation broadcast + col-cin parallel; kx/ky unrolled sequentially"，Times New Roman 8 pt 斜体。

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素、PE 内部乘法器示意。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **MAC-array module (mac_array) structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure (the 16×16 array is naturally square).**

**Layout (central 2D array + top broadcast + right WRF + bottom reduction)**:
- **Top (activation broadcast)**: A thick horizontal black solid arrow runs across the top spanning all 16 columns, originating from "from line_buffer / ARF" (light-orange CMYK 0/30/30/0 small box) on the left, labeled "activation broadcast: INT8 vec[16] (cin)".
- **Upper-right (weight side path)**: Place a small box "WRF×16 (32 each)" (light-orange fill), connected horizontally to the top of the array via a thin solid arrow, labeled "weight stationary".
- **Center (focal point, 2D array)**: Draw a 16-column × 16-row grid (use 5 columns + "..." to suggest 16, and 5 rows + "..." to suggest 16). Each cell is a small box "PE" (light-blue CMYK 30/10/0/0 fill, thin black border) with no text inside. Separate columns with light-gray dashed lines; outside the leftmost column annotate vertically "PE 0 / PE 1 / ... / PE 15 along Cin"; above the top row annotate horizontally "col 0 / col 1 / ... / col 15 → cout".
- **Bottom (adder trees + PSUM outputs)**: Below each column draw a small triangle "adder tree" (light-yellow CMYK 0/10/30/0 fill) pointing downward into a thick black arrow labeled "PSUM[i] (32 b)". The 16 PSUM arrows merge into one bold arrow below, labeled "16-way PSUM, valid-ready  →  to parf_accum".
- **Lower-left**: Place a small box "local counters (k_x, k_y, cin_pe)" (light-gray fill).

**Figure caption**: Center two small italic lines at the bottom: "16 cols × 16 PEs = 256 INT8 MACs   |   peak: 256 ops/cy × Fmax" and "Dataflow: weight-stationary + activation broadcast + col-cin parallel; kx/ky unrolled sequentially", Times New Roman 8 pt.

**Typography**: All English in Times New Roman 10 pt with bold module titles; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements, internal multiplier sketches inside PE cells. Maintain rigorous engineering-paper aesthetics throughout.
