# 图 4.1 行缓存模块结构图
# Figure 4.1 Line-buffer module structure

## 在论文中的角色
- 首次引入：§4.2 行缓存模块（paper.md L282 / L284）
- 引用位置：paper.md L282 / L284
- 论证作用：展示"片外像素流 → 片上 K×K 卷积窗口"转换的具体硬件结构。读者看完图后应理解 IFB ring buffer + 行写入 / 窗口读取 / 行释放三步流程，以及 row credit 反压机制如何让任意 H×W 输入特征图在一次 start 内单遍跑完。

## 图类型
模块结构图（含数据通路 + 反压控制流）。

## 设计要素

### 必含元素
1. **IFB SRAM（IFB=8192 word）**：位于上半部，画 5–6 行的环形缓冲示意（ring 结构）。
2. **写指针 wptr / 读指针 rptr**：环形 ring 上两个指针箭头，wptr 表示 IDMA 写入位置，rptr 表示窗口生成器读出位置。
3. **窗口生成器**：从 ring 中按 (y_out, x_out) 逐拍读出 K×K 窗口，沿 cin 维聚成 16 元向量。
4. **ARF（Activation Register File）=32**：在窗口生成器右侧的小寄存器堆，缓存当前窗口供 PE 重复读取。
5. **本地计数器（4 级）**：(y_out, x_out, k_x, k_y) 四级，画在模块左下角。
6. **2 路 valid-ready 接口**：上游接 idma_ctrl 写口（顶部入），下游接 mac_array 窗口口（底部出）。
7. **row_credit 反压通路**：从 line_buffer 回到 idma_ctrl 的虚线箭头，标 "row_credit (release row)"。

### 标注要求
- ring 容量约束：strip_rows × W_in ≤ IFB=8192 word
- forward-pressure 发射条件：rows_available ≥ y_out × stride + K_y
- 上游：INT8 row valid-ready；下游：16-elem activation vec valid-ready
- 模块右上角注 "cfg from cfg_regs: H_in, W_in, K, stride, pad, strip_rows, cin_slice"

### 视觉层次
- 主角：IFB ring + 窗口生成器
- 配角：ARF、本地计数器
- 背景：row_credit 反压虚线、cfg 输入

## 数据来源
- paper.md §4.2
- docs/modules/line_buffer.md
- contributions.md C1 系列（任意 H×W 输入）

## ASCII 示意稿

```
   from idma_ctrl                                     cfg_regs
     │ INT8 row v/r                                       │ H_in, W_in,
     ▼                                                    │ K, stride,
   ┌─────────────────────────────────────────────┐        │ pad, strip_rows
   │ line_buffer                                 │ ◀──────┘
   │                                             │
   │   ┌────────── IFB ring (8192 word) ─────┐   │
   │   │   row 0  ─┐                         │   │
   │   │   row 1   │   ↻ wptr (write)        │   │
   │   │   row 2   │                         │   │
   │   │    ...    │   ↻ rptr (read)         │   │
   │   │  row R-1 ─┘                         │   │
   │   │   (strip_rows × W_in ≤ IFB)         │   │
   │   └─────────────┬───────────────────────┘   │
   │                 │ K×K window slice          │
   │                 ▼                           │
   │           ┌──────────────┐                  │
   │           │   ARF (32)   │                  │
   │           │  hold curr.  │                  │
   │           │  K×K window  │                  │
   │           └──────┬───────┘                  │
   │                  │ 16-elem act vec (cin) v/r│
   │   ┌──────────────┴────┐                     │
   │   │ local counters    │ (y_out, x_out,      │
   │   │ 4-level           │  k_x, k_y)          │
   │   └───────────────────┘                     │
   │                                             │
   │   row_credit ── back-pressure ───┐          │
   └──────────────────────────────────┼──────────┘
                                      │
                                      ▼
                              to idma_ctrl (release row)

                                      │
                                      │ act vector to mac_array
                                      ▼

   Forward-pressure rule:
   rows_available ≥ y_out × stride + K_y  →  emit window
```

## 与正文的一致性检查
- [x] §4.2 三步流程（行写入 / 窗口读取 / 行释放）与图一致
- [x] forward-pressure 发射条件以图注形式给出
- [x] row_credit 反压路径在图中明示
- [x] 与 §3.3 图 3.2 中 line_buffer 块对齐

## 不确定项
- [TBD: 是否在 ring 上明确标 K=3 / K=7 两种典型 strip_rows] — 倾向不标，保持图通用，仅在图注里说明 "strip_rows 由编译器派生"

## image 生成提示词

### 中文版

科研论文配图，**行缓存模块（line_buffer）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（自上而下三层 + 左侧反压回线）**：
- **顶层（输入接口）**：左上方画一个细长方框 "from idma_ctrl"（淡灰色 CMYK 0/0/0/10 填充），下接一根细黑色实线箭头进入主体框，箭头旁标 "INT8 row, valid-ready"。右上方画 "cfg_regs" 小方框（淡灰色填充），用细虚线连入主体框右侧，标 "H_in, W_in, K, stride, pad, strip_rows"。
- **中央（主角）**：绘制一个大圆角矩形 "line_buffer"（淡黄色 CMYK 0/10/30/0 填充、深灰色边框）。框内**上半部**画 IFB ring buffer（淡蓝色 CMYK 30/10/0/0 椭圆环，6 段格子表示 row 0..R-1，旁标 "IFB = 8192 word, strip_rows × W_in"），ring 上画两个小箭头：顺时针的 wptr（黑色）与稍小的 rptr（深蓝色）。框内**中部**画一个梯形或漏斗形状 "window generator"，下方接一个小方框 "ARF (32)"（淡橙色 CMYK 0/30/30/0 填充）。框内**左下角**画一个小方框 "local counters (y_out, x_out, k_x, k_y)"。
- **底部（输出接口）**：ARF 下方画一根粗黑色实线箭头向下，标 "16-elem act vec (cin), valid-ready  →  to mac_array"。
- **左侧（反压回线）**：从 line_buffer 框左边缘画一根**深红色虚线箭头**绕回顶层 "from idma_ctrl"，标 "row_credit (release row)"。

**图注**：底部居中写一行小字 "Forward-pressure rule: rows_available ≥ y_out × stride + K_y"，Times New Roman 8 pt 斜体。

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **line-buffer module structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (three tiers top-to-bottom + a left-side back-pressure return)**:
- **Top tier (input interface)**: At upper-left, place a thin rectangle "from idma_ctrl" (light-gray CMYK 0/0/0/10 fill). A thin black solid arrow goes down into the main body, labeled "INT8 row, valid-ready". At upper-right, place a small "cfg_regs" box (light-gray fill) connected by a thin dashed line into the right edge of the main body, labeled "H_in, W_in, K, stride, pad, strip_rows".
- **Center (focal point)**: Draw a large rounded rectangle "line_buffer" (light-yellow CMYK 0/10/30/0 fill, dark-gray border). Inside the **upper half** draw the IFB ring buffer (light-blue CMYK 30/10/0/0 elliptical ring with 6 segments labeled rows 0..R-1, annotated "IFB = 8192 word, strip_rows × W_in"); place two small arrows on the ring: a clockwise wptr (black) and a slightly smaller rptr (dark blue). In the **middle** draw a trapezoid "window generator" with a small box "ARF (32)" (light-orange CMYK 0/30/30/0 fill) below it. At the **lower-left corner** draw a small box "local counters (y_out, x_out, k_x, k_y)".
- **Bottom (output interface)**: Below ARF draw a thick black solid arrow downward, labeled "16-elem act vec (cin), valid-ready  →  to mac_array".
- **Left side (back-pressure return)**: From the left edge of line_buffer draw a **dark-red dashed arrow** wrapping back up to "from idma_ctrl", labeled "row_credit (release row)".

**Figure caption**: Center a small italic footnote at the bottom: "Forward-pressure rule: rows_available ≥ y_out × stride + K_y", Times New Roman 8 pt.

**Typography**: All English in Times New Roman 10 pt with bold module titles; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
