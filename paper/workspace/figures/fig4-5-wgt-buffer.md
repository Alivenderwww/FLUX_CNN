# 图 4.5 权重缓存模块结构图
# Figure 4.5 Weight buffer module structure

## 在论文中的角色
- 首次引入：§4.6 权重缓存模块（paper.md L338 / L342）
- 引用位置：paper.md L338 / L342 / §4.12 本章小结
- 论证作用：展示 wgt_buffer 内部 WB（1024 word SRAM）+ WRF（32 寄存器堆）的双层结构与"WB 加载（一层一次） → WRF 加载（一个 cout_slice 一次） → 侧路供权重给 16 列 MAC"两步流程，强调权重静止数据流下 cout_slice 内反复使用 WRF、不再访问 DDR/WB 的关键设计。读者应理解 WB 与 WRF 的容量差异、加载频次差异、以及对 16 列 PE 的并行供权方式。

## 图类型
模块结构图（左侧 WB SRAM 大方块 + 中间 WRF 寄存器堆 + 右侧 MAC 阵列简化方块），整体接近 1:1 正方形布局。

## 设计要素

### 必含元素
1. **WB 主体（左侧大方块）**：标 "Weight Buffer (WB) — 1024 word SRAM"，内部画一行行小格示意 SRAM 阵列。
2. **WB 写口（顶部输入）**：从上方接入一根粗箭头标 "from wdma_ctrl (MM2S, full layer)"，强调"一层一次"。
3. **WRF 主体（中间方块）**：标 "Weight Register File (WRF) — 32 entries × 16 cols"，画 16 列 × 32 行小格阵列示意。
4. **WB → WRF 内部数据通路**：WB 与 WRF 间一根粗黑箭头，标 "load 1 cout_slice (16×32 weights, one-shot)"。
5. **WRF → MAC 阵列广播口（右侧）**：从 WRF 右侧画 16 根细水平线（或合并为一根粗箭头标 "16 cols"），标 "weight broadcast (no back-pressure)"。
6. **MAC 阵列简化方块（右侧）**：画一个 16×16 PE 网格示意（或仅画 4×4 缩略 + "16×16" 文字），标 "MAC array (256 PE)"，灰色填充表示"非本图主角"。
7. **本地计数器（左下小方块）**：标 "(c_out, c_in, k_x, k_y) 4-level local counters"，从 cfg_regs 接入循环边界（细虚线）。
8. **cfg_regs 输入（底部小标签）**：标 "cfg: K, C_in, cout_slice 边界"，细虚线箭头进入计数器。

### 标注要求
- WB 容量：1024 word（来自 CLAUDE.md params.py，WB=1024）
- WRF 容量：32 entries × 16 columns（NUM_PE=NUM_COL=16, WRF=32）
- WB 写口频次：每层加载一次
- WRF 加载频次：每 cout_slice 加载一次（cout_slice 内反复使用，稳态下不访问 DDR/WB）
- 权重广播：无反压（与 mac_array 的权重静止数据流配合）

### 视觉层次
- 主角：WB（左大方块）→ WRF（中等方块）→ 16 列广播口（中间数据通路）
- 配角：MAC 阵列简化方块（灰色填充）、本地计数器
- 背景：cfg_regs 输入（细虚线）

## 数据来源
- paper.md §4.6
- docs/modules/wgt_buffer.md（如存在）
- CLAUDE.md params.py：WB=1024 / WRF=32 / NUM_COL=NUM_PE=16
- contributions.md（权重静止 + 双层缓存设计）

## ASCII 示意稿

```
                 ┌─ from wdma_ctrl (MM2S, full layer) ─┐
                 │  (write entire layer's weights)     │
                 ▼
   ┌───────────────────────────────┐         ┌───────────────────────┐
   │  Weight Buffer (WB)           │         │  MAC array (16×16 PE) │
   │  1024 word SRAM               │         │   (next module)       │
   │                               │         │                       │
   │  ┌─┬─┬─┬─┬─┬─┬─┬─┬─┐          │         │  ┌─┬─┬─┬─┐            │
   │  ├─┼─┼─┼─┼─┼─┼─┼─┼─┤          │         │  ├─┼─┼─┼─┤            │
   │  ├─┼─┼─┼─┼─┼─┼─┼─┼─┤          │         │  ├─┼─┼─┼─┤  16×16     │
   │  ├─┼─┼─┼─┼─┼─┼─┼─┼─┤          │         │  ├─┼─┼─┼─┤  PE grid   │
   │  └─┴─┴─┴─┴─┴─┴─┴─┴─┘          │         │  └─┴─┴─┴─┘ (gray)     │
   │  (entire layer's weights)     │         │                       │
   └────────────┬──────────────────┘         └───────────▲───────────┘
                │                                        │
                │ load 1 cout_slice                      │ weight
                │ (16×32 weights, one-shot)              │ broadcast
                │                                        │ (16 cols,
                ▼                                        │  no back-pressure)
   ┌───────────────────────────────┐                    │
   │  Weight Register File (WRF)   │                    │
   │  32 entries × 16 columns      │                    │
   │                               ├────────────────────┘
   │  ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┐                  
   │  ├─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┤                  
   │  ├─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┤   16 cols       
   │  └─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┴─┘                  
   │  (1 cout_slice, reused)       │                    
   └───────────────────────────────┘                    

   ┌───────────────────────┐
   │ local counters:       │ ◀── cfg: K, C_in, cout_slice (dashed)
   │ (c_out, c_in, k_x, k_y)│
   └───────────────────────┘
```

## 与正文的一致性检查
- [x] §4.6 "WB=1024 word + WRF=32 双层结构" — WB 主体方块标 "1024 word SRAM"，WRF 主体方块标 "32 entries × 16 cols"
- [x] §4.6 "WB 加载：一层启动时整层权重一次写入" — WB 顶部写口标 "from wdma_ctrl (MM2S, full layer)"
- [x] §4.6 "WRF 加载：每个 cout_slice 一次性读 16 列 × 32 项" — WB→WRF 内部箭头标 "load 1 cout_slice (16×32, one-shot)"
- [x] §4.6 "之后整段 cout_slice 内所有空间位置直接消费 WRF，不再访问 WB" — WRF 方块下方标 "(1 cout_slice, reused)"
- [x] §4.6 "侧路接 MAC 阵列权重广播口（无反压）" — WRF→MAC 通路标 "weight broadcast (16 cols, no back-pressure)"
- [x] §4.6 "(c_out, c_in, k_x, k_y) 4 级本地计数器，循环边界由 cfg_regs 提供" — 左下计数器框 + cfg 虚线
- [x] WB=1024 / WRF=32 / 16 列 与 CLAUDE.md params.py 一致

## 不确定项
- [TBD: WRF 实际是否为 32×16 二维寄存器堆（每列独立 32 项），还是 32×128 b（16 列共享同一行索引）— 当前按"每列独立 32 项"画，与 mac_array 的 16 列并行结构对齐]
- [CHECK: WB 的具体读位宽（应为 BUS_DATA_W=128 b）— 图中暂不标位宽，避免与 wdma_ctrl 实现细节冲突]

## image 生成提示词

### 中文版

科研论文配图，**权重缓存模块（wgt_buffer）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。布局为左下 WB + 上方 WRF + 右侧 MAC 阵列的 L 字形三块并列结构。**

**版式（L 字形三块结构）**：
- **主体（圆角矩形大框）**：标题 "Weight Buffer Module (wgt_buffer)"（淡黄色 CMYK 0/10/30/0 填充、深灰色边框）。
- **左下方块 — WB**：圆角矩形，标 "Weight Buffer (WB) — 1024 word SRAM"（淡蓝色 CMYK 30/10/0/0 填充），内部画一个 8 行 × 9 列的小格阵列示意 SRAM bank，下方小字 "(entire layer's weights)"。WB 顶部画一根粗黑色实线箭头朝下，上方标 "from wdma_ctrl (MM2S, full layer)"。
- **左上方块 — WRF**：圆角矩形（位于 WB 上方稍偏右），标 "Weight Register File (WRF) — 32 entries × 16 columns"（淡蓝色填充），内部画一个 4 行 × 16 列的小格阵列示意 16 列寄存器堆，下方小字 "(1 cout_slice, reused)"。
- **WB → WRF 通路**：从 WB 顶部到 WRF 底部一根粗黑色实线竖向箭头，箭头中段标 "load 1 cout_slice (16×32 weights, one-shot)"。
- **右侧方块 — MAC array (灰色, 配角)**：圆角矩形（位于 WRF 右侧），标 "MAC array (16×16 PE)" 与 "(next module)" 小字，淡灰色 CMYK 0/0/0/15 填充表示"非本图主角"，内部画 4×4 简化 PE 网格示意。
- **WRF → MAC 通路**：从 WRF 右边缘画 16 根并列细水平线汇成一根粗黑色实线箭头进入 MAC array，箭头标 "weight broadcast (16 cols, no back-pressure)"。
- **底部小框 — local counters**：位于 WB 下方，画一个小方框 "local counters: (c_out, c_in, k_x, k_y) 4-level"（淡橙色 CMYK 0/30/30/0 填充），从下方接一根细虚线（深灰色），标 "cfg_regs: K, C_in, cout_slice 边界"。
- **关键标注**：在 WB 与 WRF 之间用小字补一行 "WB: load 1× per layer    WRF: load 1× per cout_slice    weights reused inside cout_slice"。

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗，容量数字（1024 word / 32 entries / 16 cols）用粗体；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Weight buffer module (wgt_buffer) structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. The layout is an L-shape of three blocks: WB at lower-left, WRF at upper-left (above WB), and MAC array at right.**

**Layout (L-shape, three blocks)**:
- **Main body (large rounded rectangle)**: Titled "Weight Buffer Module (wgt_buffer)" (light-yellow CMYK 0/10/30/0 fill, dark-gray border).
- **Lower-left block — WB**: Rounded rectangle labeled "Weight Buffer (WB) — 1024 word SRAM" (light-blue CMYK 30/10/0/0 fill); inside draw an 8-row by 9-column small grid representing the SRAM bank, with small caption "(entire layer's weights)" below. From the top of WB draw a thick black solid arrow downward (entering from above), labeled "from wdma_ctrl (MM2S, full layer)".
- **Upper-left block — WRF**: Rounded rectangle (above WB, slightly to the right), labeled "Weight Register File (WRF) — 32 entries × 16 columns" (light-blue fill); inside draw a 4-row by 16-column small grid representing the 16-column register file, with small caption "(1 cout_slice, reused)" below.
- **WB → WRF path**: From the top of WB to the bottom of WRF draw a thick black solid vertical arrow, with mid-arrow label "load 1 cout_slice (16×32 weights, one-shot)".
- **Right block — MAC array (gray, supporting role)**: Rounded rectangle (to the right of WRF), labeled "MAC array (16×16 PE)" with small caption "(next module)", light-gray CMYK 0/0/0/15 fill to indicate it is not the main subject; inside draw a 4×4 simplified PE grid.
- **WRF → MAC path**: From the right edge of WRF draw 16 parallel thin horizontal lines merging into a single thick black solid arrow entering the MAC array, labeled "weight broadcast (16 cols, no back-pressure)".
- **Bottom small box — local counters**: Below WB draw a small box "local counters: (c_out, c_in, k_x, k_y) 4-level" (light-orange CMYK 0/30/30/0 fill); from below it a thin dashed dark-gray line labeled "cfg_regs: K, C_in, cout_slice bounds".
- **Key annotation**: Between WB and WRF add a small line of text: "WB: load 1× per layer    WRF: load 1× per cout_slice    weights reused inside cout_slice".

**Typography**: All English in Times New Roman 10 pt with bold module titles; capacity numbers (1024 word / 32 entries / 16 cols) in bold; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
