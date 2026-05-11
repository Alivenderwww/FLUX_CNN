# 图 4.3 部分和累加模块结构图
# Figure 4.3 Partial-sum accumulator module structure

## 在论文中的角色
- 首次引入：§4.4 部分和累加模块（paper.md L310 / L312）
- 引用位置：paper.md L310 / L312
- 论证作用：展示 PARF 在物理上拆分为 16 个独立 SRAM（parf_col × NUM_COL=16）、16 列共享 wr_addr/we 的外壳结构，以及"读出累加值 → 相加 → 回写 → cin_slice 完成时发射"的三步流程。读者应理解列私有读端口 + 共享写控制为何匹配 MAC 阵列的输出通道广播形态。

## 图类型
模块结构图（含 16 列独立 SRAM + 共享外壳控制）。

## 设计要素

### 必含元素
1. **16 列独立 SRAM**：横向排列 16 个小 SRAM 块，每块标 "parf_col[i]"，容量 32 项 × 32 b。绘图用 4 列 + "..." + 1 列示意。
2. **共享 wr_addr / we 外壳**（上方）：一根横线贯穿所有 16 列 SRAM 的写控制端，标 "shared wr_addr / we (driven by outer shell)"。
3. **16 路读数据通路**：每列 SRAM 的读端口独立，标 "col-private read"。
4. **16 个加法器**：每列一个，把当前周期 PSUM 与读出累加值相加。
5. **写回路径**：加法器输出回写本列 SRAM。
6. **cin_slice=0 跳过读出 mux**：每列加法器前的 mux 选择 "0 (init)" 或 "PARF read"，由 cin_slice 索引控制。
7. **发射逻辑**：当 (tile_w, cout_col) 槽位完成所有 cin_slice 累加，向下游 ofb_writer 发射。
8. **本地计数器（2 级）**：(tile_w, cin_slice)，画在模块左下角。

### 标注要求
- PARF=32 容量，逻辑视图为 16 列 × 32 项
- 物理实现：16 个独立 SRAM 实例 = parf_col × NUM_COL
- 写地址共享，读数据独立
- 上游：16 路 PSUM (32 b) v/r；下游：16 路 accum-done PSUM (32 b) v/r
- cfg 输入：cin_slice_total, tile_w_max

### 视觉层次
- 主角：16 列独立 SRAM 阵列
- 配角：共享 wr_addr/we 外壳横线、16 个加法器、cin_slice=0 mux、发射逻辑
- 背景：cfg 输入、本地计数器

## 数据来源
- paper.md §4.4
- docs/modules/parf_accum.md
- CLAUDE.md "parf_accum 内部由 parf_col × NUM_COL 组成（每列独立 SRAM，外壳共享 wr_addr/we）"

## ASCII 示意稿

```
   from mac_array (16× PSUM 32 b, v/r)
        │   │   │   │       │
        ▼   ▼   ▼   ▼       ▼

   ┌──────────── shared wr_addr / we (outer shell) ────────────┐
   │                                                            │
   │  col 0    col 1    col 2    col 3    ...    col 15         │
   │  ┌───┐    ┌───┐    ┌───┐    ┌───┐           ┌───┐          │
   │  │ + │    │ + │    │ + │    │ + │   ...     │ + │  (16 add)│
   │  └─▲─┘    └─▲─┘    └─▲─┘    └─▲─┘           └─▲─┘          │
   │    │mux    │mux     │mux     │mux             │mux         │
   │    │(0/rd) │(0/rd)  │(0/rd)  │(0/rd)          │(0/rd) ◀── cin_s=0 sel
   │  ┌─▼─┐    ┌─▼─┐    ┌─▼─┐    ┌─▼─┐           ┌─▼─┐          │
   │  │SRM│    │SRM│    │SRM│    │SRM│   ...     │SRM│  PARF=32 │
   │  │p0 │    │p1 │    │p2 │    │p3 │           │p15│  per col │
   │  │32×│    │32×│    │32×│    │32×│           │32×│  (priv.  │
   │  │32b│    │32b│    │32b│    │32b│           │32b│   read)  │
   │  └─┬─┘    └─┬─┘    └─┬─┘    └─┬─┘           └─┬─┘          │
   │    │        │        │        │                │           │
   └────┼────────┼────────┼────────┼────────────────┼───────────┘
        ▼        ▼        ▼        ▼                ▼
       ┌────────────────────────────────────────────────┐
       │ emit logic — fire when all cin_slice complete  │
       │              for current (tile_w, cout_col)    │
       └─────────────────────┬──────────────────────────┘
                             │ 16× accum-done PSUM (32 b) v/r
                             ▼
                       to ofb_writer

   ┌──────────────────────────┐
   │ local counters           │  (tile_w, cin_slice) — 2 levels
   │ cfg: cin_slice_total,    │  bounds from cfg_regs
   │      tile_w_max          │
   └──────────────────────────┘
```

## 与正文的一致性检查
- [x] §4.4 "16 列共享同一组写地址（wr_addr）与写使能（we），由外壳模块统一驱动" — 图中外壳横线贯穿 16 列写控制
- [x] §4.4 "每列的写数据来自本列 mac_array 输出 PSUM，每列的读数据独立返回" — 列私有数据通路
- [x] §4.4 "cin_slice=0 时跳过读出，直接写入新值" — 图中 mux 选 0 路径
- [x] §4.4 "PARF=32 容量对应一个 cout_slice 内 32 个 tile_w 输出位置的部分和缓存" — 每列 32 项标注
- [x] §4.4 "(tile_w, cin_slice) 二级本地计数器自维护" — 图中本地计数器块

## 不确定项
- [TBD: 16 列在图上挤不下，建议绘图阶段用 4 列 + "..." + 1 列示意省略号]

## image 生成提示词

### 中文版

科研论文配图，**部分和累加模块（parf_accum）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。绘图时 16 列横向用 5 列 + "..." 省略，让宽高接近 1:1。**

**版式（自上而下：输入 → 共享写控制壳 → 16 列 SRAM + 加法器 → 发射 → 输出）**：
- **顶部（输入）**：水平排列 5 个粗箭头朝下，从左到右标 "PSUM[0] / PSUM[1] / PSUM[2] / PSUM[3] / ... / PSUM[15] (32 b)"，顶部居中标 "from mac_array, 16-way valid-ready"。
- **中部主体（主角）**：绘制一个大圆角矩形，外框上方画一根**水平贯穿的浅灰色横线**（淡蓝色 CMYK 30/10/0/0 细线），标 "shared wr_addr / we (outer shell)"，横线连接到内部 5 个 SRAM 块的写控制端。框内**水平排列 5 列子结构**（中间 4 列实绘 + 第 5 列前留 "..."），每列从上到下：
  - 一个小三角形/方框 "+"（加法器，淡黄色 CMYK 0/10/30/0 填充）
  - 加法器左下角一个小菱形 "mux"（标 "0 / PARF rd"）
  - 一个小方框 "SRAM parf[i] 32×32 b"（淡蓝色填充）
  - 列间用浅灰色细虚线分隔
  - 列右侧标 "col-private read" 小字
- **mux 控制注释**：在最右列 mux 旁画一根细黑色实线箭头，从右侧伸入，标 "cin_slice=0 sel"。
- **下部（发射逻辑）**：5 列 SRAM 下方画一个长条形 "emit logic — fire when all cin_slice complete for (tile_w, cout_col)"（淡橙色 CMYK 0/30/30/0 填充）。下方接一根粗黑色箭头朝下，标 "16× accum-done PSUM (32 b), valid-ready  →  to ofb_writer"。
- **左下角**：画一个小方框 "local counters (tile_w, cin_slice); cfg: cin_slice_total, tile_w_max"（淡灰色填充）。

**图注**：底部居中写一行小字 "PARF = 32 / col, physical = 16 independent SRAMs (parf_col × NUM_COL); shared write address, private read data"，Times New Roman 8 pt 斜体。

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **partial-sum accumulator module (parf_accum) structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. Use 5 columns + "..." to indicate the 16 lanes so the figure stays close to 1:1.**

**Layout (top to bottom: inputs → shared write control shell → 16-col SRAM + adders → emit → output)**:
- **Top (inputs)**: Place 5 thick downward arrows in a row, labeled left to right "PSUM[0] / PSUM[1] / PSUM[2] / PSUM[3] / ... / PSUM[15] (32 b)", with a centered top caption "from mac_array, 16-way valid-ready".
- **Center (focal point)**: Draw a large rounded rectangle. At the top of the frame draw a **horizontal light-blue (CMYK 30/10/0/0) thin line crossing the entire width**, labeled "shared wr_addr / we (outer shell)", connecting to the write-control terminal of all 5 SRAM blocks inside. Inside, **horizontally arrange 5 column substructures** (4 actual columns plus "..." before the 5th). Each column, top to bottom:
  - A small triangle or box "+" (adder, light-yellow CMYK 0/10/30/0 fill)
  - A small diamond "mux" below-left of the adder (labeled "0 / PARF rd")
  - A small box "SRAM parf[i] 32×32 b" (light-blue fill)
  - Light-gray dashed lines separating columns
  - "col-private read" caption to the right of each column
- **Mux control annotation**: Beside the rightmost mux draw a thin black solid arrow entering from the right, labeled "cin_slice=0 sel".
- **Bottom (emit logic)**: Below the 5 columns draw a long horizontal bar "emit logic — fire when all cin_slice complete for (tile_w, cout_col)" (light-orange CMYK 0/30/30/0 fill). Below it a thick black arrow downward, labeled "16× accum-done PSUM (32 b), valid-ready  →  to ofb_writer".
- **Lower-left**: Place a small box "local counters (tile_w, cin_slice); cfg: cin_slice_total, tile_w_max" (light-gray fill).

**Figure caption**: Center a small italic footnote at the bottom: "PARF = 32 / col, physical = 16 independent SRAMs (parf_col × NUM_COL); shared write address, private read data", Times New Roman 8 pt.

**Typography**: All English in Times New Roman 10 pt with bold module titles; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
