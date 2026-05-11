# 图 2.1 卷积计算原理示意图
# Figure 2.1 Schematic of convolution operation

## 在论文中的角色
- 首次引入位置：§2.2.1 卷积层（paper.md L122）
- 引用位置：paper.md L122
- 论证作用：直观解释 *K*×*K* 卷积核在输入特征图上滑动、逐元素乘加并产生单个输出像素的基本原理。读者应据此理解后续多通道展开、滑窗复用、行缓存设计的几何根源。

## 图类型
概念示意图（卷积运算几何图）。

## 设计要素

### 必含元素
1. **输入特征图**：左侧大方块网格，标注 "Input feature map (H_in × W_in)"，建议绘制 5×5 或 6×6 网格便于看清。
2. **卷积核**：网格上覆盖一个 3×3 的高亮框（淡蓝色填充），标注 "K×K kernel (K=3)"，框内绘制 3×3 权重 *w(0,0)…w(2,2)*。
3. **滑动方向**：在输入特征图上画两个箭头（一个水平向右、一个垂直向下）表示 *x* / *y* 方向的滑动。
4. **乘加运算**：中间区域写公式或绘制 9 路乘法 + 加法树，输入是输入像素 *a(i+u, j+v)* 与权重 *w(u, v)*。
5. **输出特征图**：右侧网格，标注 "Output feature map (H_out × W_out)"，高亮当前 (*i*, *j*) 输出像素，箭头从乘加结果指向该像素。
6. **偏置 *b***：加在加法树之后，结果送入输出像素。

### 标注要求
- 输入网格大小标注 *H_in* × *W_in*
- 卷积核标注 *K* = 3
- 输出网格标注 *H_out* × *W_out*
- 公式注 "*Y(i,j)* = ΣΣ *a(i+u, j+v)* · *w(u,v)* + *b*"
- 乘加路径 9 条用细线连接

### 视觉层次
- 主角：滑动中的 3×3 卷积核（淡蓝色填充）+ 当前输出像素（淡绿色填充）
- 配角：输入网格与输出网格的灰色细线
- 背景：滑动方向箭头（虚线）

## 数据来源
- paper.md §2.2.1（公式 1）
- 范文：`paper/参考/基于 FPGA 的脉冲神经网络加速器设计/2.x.md`（卷积原理页）

## ASCII 示意稿

```
   Input feature map (H_in × W_in)             Output feature map (H_out × W_out)
   ┌───┬───┬───┬───┬───┬───┐                   ┌───┬───┬───┬───┬───┐
   │ a │ a │ a │ a │ a │ a │                   │   │   │   │   │   │
   ├───┼───┼───┼───┼───┼───┤                   ├───┼───┼───┼───┼───┤
   │ a │░w░│░w░│░w░│ a │ a │ ──── slide x ──▶  │   │ Y │   │   │   │
   ├───┼░░░┼░░░┼░░░┼───┼───┤                   ├───┼───┼───┼───┼───┤
   │ a │░w░│░w░│░w░│ a │ a │                   │   │   │   │   │   │
   ├───┼░░░┼░░░┼░░░┼───┼───┤                   ├───┼───┼───┼───┼───┤
   │ a │░w░│░w░│░w░│ a │ a │                   │   │   │   │   │   │
   ├───┼───┼───┼───┼───┼───┤                   └───┴───┴───┴───┴───┘
   │ a │ a │ a │ a │ a │ a │                          ▲
   └───┴───┴───┴───┴───┴───┘                          │
                                              ┌───────┴────────┐
        K×K kernel (K=3)                      │  Σ Σ a·w + b   │
        with sliding window                   └────────────────┘
                                              dot-product + bias

   Y(i,j) = Σ_u=0..K-1 Σ_v=0..K-1 a(i+u, j+v) · w(u,v) + b
```

## 与正文的一致性检查
- [x] §2.2.1 "用一个 *K*×*K* 的卷积核在输入特征图上逐点滑动" — 图中绘制 3×3 高亮框 + 滑动箭头
- [x] §2.2.1 "卷积核覆盖区域的输入与对应权重逐元素相乘并累加" — 图中乘加路径
- [x] 公式 (1) "*Y(i,j)* = …" — 图中公式注
- [x] §2.2.1 "卷积核在滑动过程中权重保持不变" — 卷积核位置随滑动变化但权重 *w* 不变（图注说明）

## 不确定项
- [TBD: 输入特征图绘 5×5 还是 6×6，最终由作者按版面权衡] — 倾向 6×6（含 3×3 滑窗 + 余量便于显示滑动方向）

## image 生成提示词

### 中文版

科研论文配图，**卷积运算原理示意图**，IEEE 期刊配色风格，整体白底黑字，禁用任何彩色背景与卡通风格。横向布局。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式**：左侧绘制 6×6 输入特征图网格，每格用浅灰色细线分隔，整体外框用黑色实线，左上方标注 "Input feature map (H_in × W_in)"。在网格中央位置叠加一个 3×3 高亮卷积核框，淡蓝色（CMYK 30/10/0/0）半透明填充，深蓝色实线边框，每格内写权重符号 w(0,0)、w(0,1)…w(2,2)，左上方标注 "K×K kernel (K=3)"。卷积核框右侧画一个细黑实线水平箭头，下方画一个细黑实线垂直箭头，箭头旁分别写 "slide x"、"slide y"。

**右侧**：绘制 4×4 输出特征图网格，外框黑色实线，左上方标 "Output feature map (H_out × W_out)"，其中第 1 行第 2 列单元用淡绿色（CMYK 20/0/30/0）填充并写 "Y"。

**中间下方**：绘制一个圆角矩形框，框内写公式 "Y(i,j) = Σ Σ a(i+u, j+v) · w(u,v) + b"，用 Times New Roman 斜体；从输入网格的 3×3 高亮区到该框画 9 条灰色细斜线（表示乘加路径），从该框到右侧输出 Y 单元画一条黑色实线带箭头。

**字体**：所有英文用 Times New Roman 10 pt，中文（如有）用思源黑体 10 pt。**禁止出现**手写感字体、彩色渐变、装饰阴影、图标、3D 立体效果、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **schematic of convolution operation**, IEEE-journal style, overall white background with black text and lines, no decorative colors or cartoon elements. Horizontal layout. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout**: Draw a 6×6 input feature map grid on the left, with light gray cell borders and a black outer frame; label "Input feature map (H_in × W_in)" at upper-left. Overlay a 3×3 highlighted convolution kernel near the center of the grid, with light-blue (CMYK 30/10/0/0) semi-transparent fill and a dark-blue solid border; label kernel weights w(0,0), w(0,1)…w(2,2) inside the cells, and "K×K kernel (K=3)" at upper-left of the kernel. To the right of the kernel draw two thin solid black arrows: one horizontal "slide x" and one vertical "slide y".

**Right side**: Draw a 4×4 output feature map grid with black outer frame, labeled "Output feature map (H_out × W_out)"; highlight the cell at row 1 column 2 in light green (CMYK 20/0/30/0) and write "Y" inside.

**Center bottom**: Draw a rounded rectangle box containing the formula "Y(i,j) = Σ Σ a(i+u, j+v) · w(u,v) + b" rendered in Times New Roman italic. Connect the 3×3 highlighted region of the input grid to this box with 9 thin gray diagonal lines (representing multiply-accumulate paths); from this box draw a solid black arrow to the highlighted output cell Y.

**Typography**: All English text in Times New Roman 10 pt; any Chinese text in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, icons, 3D effects, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
