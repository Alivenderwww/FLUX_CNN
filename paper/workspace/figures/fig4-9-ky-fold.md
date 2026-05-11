# 图 4.9 Ky 折叠原理示意图
# Figure 4.9 Ky-fold transformation principle

## 在论文中的角色
- 首次引入：§4.8 编译器优化 Ky 折叠（paper.md L471 / L473）
- 引用位置：paper.md L471 / L473
- 论证作用：直观展示 Cin < 16 时编译器侧把卷积核 *K_y* 维"折"到伪 Cin 维的机制；读者应理解折叠前 PE 阵列在 Cin 方向的列空转，与折叠后 cin_fake 维度填满 16 PE 的对照关系；为 §4.8 的数学描述（公式(1)→(2)）提供可视化支撑。

## 图类型
变换原理示意图（折叠前 / 折叠后两栏对照 + 中央折叠箭头）。

## 设计要素

### 必含元素
1. **左栏（折叠前 / 硬件原视角）**：
   - 16 行 PE 列（垂直堆叠 16 个小方框，象征 NUM_PE=16 沿 Cin 方向并行）
   - 仅前 *C*ᵢₙ 行（以 *C*ᵢₙ=3 为例：前 3 行）填充蓝色，标 "*c_i*=0 / 1 / 2"
   - 后 13 行留白，标 "PE idle (13 / 16)"
   - 顶部标 "*K*=3, *C*ᵢₙ=3 卷积窗口"
   - 底部标 "PE 利用率 = 3 / 16 = 18.75%"
2. **中央（折叠箭头 + 公式简写）**：
   - 粗箭头从左指向右，标 "Ky-fold (compiler-side)"
   - 箭头下方一小段写 cin_fake = (*k_y* mod *groups_y*) × *C*ᵢₙ + *c_i*；*groups_y* = ⌊16 / *C*ᵢₙ⌋
   - 以 *C*ᵢₙ=3 为例：*groups_y* = 5，cin_fake ∈ [0..14]
3. **右栏（折叠后 / 等效卷积视角）**：
   - 同样 16 行 PE 列
   - 前 15 行填充三种颜色（5 个 *k_y* 行 × 3 cin），分别标 "*k_y*=0..4" 与对应 *c_i*
   - 第 16 行留白，标 "1 PE idle"
   - 顶部标 "*K_y_eff* = *kyper* = ⌈*K*/*groups_y*⌉ = 1"
   - 底部标 "PE 利用率 = 15 / 16 = 93.75%"

### 标注要求
- 折叠前后 PE 利用率数字对照清晰可见（18.75% → 93.75%）
- *groups_y* 与 *kyper* 两个推导参数的关系简要标注
- 编译器侧 X' 复制方向（*y* 偏移复制到 cin 维）以箭头补充

### 视觉层次
- 主角：左右两栏 PE 列对照
- 配角：中央折叠箭头与简化公式
- 背景：顶部卷积窗口符号与底部利用率数据

## 数据来源
- paper.md §4.8（公式 (1) / (2) 及推导）
- docs/pe-fold.md（数学推导原文）
- contributions.md C2.1（Ky 折叠贡献）
- model_analysis.md（ResNet 首层 *K*=3、*C*ᵢₙ=3 实例）

## ASCII 示意稿

```
   ┌──────────────────┐                           ┌──────────────────┐
   │ 折叠前 (Cin=3)   │                           │ 折叠后 (cin_fake)│
   │  K=3 卷积窗口    │                           │  K_y_eff = 1     │
   ├──────────────────┤                           ├──────────────────┤
   │ PE 0  ■ c_i=0    │                           │ PE 0  ▣ ky=0,c0  │
   │ PE 1  ■ c_i=1    │   ━━━━━━━━━━━━━━━━━━▶    │ PE 1  ▣ ky=0,c1  │
   │ PE 2  ■ c_i=2    │    Ky-fold              │ PE 2  ▣ ky=0,c2  │
   │ PE 3  □ idle     │   (compiler-side)        │ PE 3  ▣ ky=1,c0  │
   │ PE 4  □ idle     │                           │ PE 4  ▣ ky=1,c1  │
   │ PE 5  □ idle     │   cin_fake               │ PE 5  ▣ ky=1,c2  │
   │ PE 6  □ idle     │   = (k_y mod g_y)·C_in   │ PE 6  ▣ ky=2,c0  │
   │  :    :          │     + c_i                │  :    :          │
   │  :    :          │                           │ PE 14 ▣ ky=4,c2  │
   │ PE 14 □ idle     │   g_y = ⌊16 / C_in⌋ = 5  │ PE 15 □ idle     │
   │ PE 15 □ idle     │                           │                  │
   ├──────────────────┤                           ├──────────────────┤
   │ Util = 3/16      │                           │ Util = 15/16     │
   │       = 18.75%   │                           │       = 93.75%   │
   └──────────────────┘                           └──────────────────┘

      X (DDR layout)            ━━ DDR-friendly ━━           X' (cin-extended)
      (y, x, c_i)               y-shift replicate            (y, x, cin_fake)
                                in cin only

   编译器侧零 RTL 改动：
     · 权重 W' 按 cin_fake 索引重排，DDR 中按等效卷积布局摆放
     · 输入 X' 按 k_y 偏移在 cin 维上复制（y' = y + g·g_y + (k_y mod g_y)）
     · cfg_regs 用 cin_slice = cin_fake / 16 派生
     · 硬件继续按"普通卷积"运行，无需感知折叠存在
```

## 与正文的一致性检查
- [x] §4.8 "*C*ᵢₙ < 16 ∧ *K* > 1 时启用 Ky 折叠" — 左栏的 PE idle 部分体现折叠动机
- [x] §4.8 公式 (1) → (2)：cin_fake = (*k_y* mod *groups_y*) × *C*ᵢₙ + *c_i* — 中央简化公式与右栏标号一致
- [x] §4.8 "*groups_y* = ⌊NUM_PE / *C*ᵢₙ⌋ = 5（*C*ᵢₙ=3 时）" — 中央与右栏标注一致
- [x] §4.8 "*kyper* = ⌈*K* / *groups_y*⌉" — 右栏顶部 *K_y_eff* = 1（*K*=3 / *groups_y*=5 → 1）
- [x] §4.8 "在 *y* 方向偏移复制到 cin 维" — 底部箭头与 X' 标注

## 不确定项
- [TBD: 是否在右栏额外画出 *kyper* > 1 的场景（例如 *K*=7、*C*ᵢₙ=4，*groups_y*=4、*kyper*=2，需要两次发射）] — 倾向不画，主图保持单组折叠的核心对照；*kyper* > 1 在图注里以一行小字提示

## image 生成提示词

### 中文版

科研论文配图，**编译器优化 Ky 折叠原理示意图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（左右两栏对照 + 中央折叠箭头）**：
- **左栏（折叠前 / 硬件原视角）**：画一个圆角矩形，标题"折叠前 (*C*ᵢₙ=3)"（思源黑体 10 pt 加粗）。框内自上而下排列 16 个小方框（代表 PE 0..15）。前 3 个方框（PE 0/1/2）填淡蓝色 CMYK 30/10/0/0、内部分别写"*c_i*=0""*c_i*=1""*c_i*=2"；后 13 个方框留白、内部统一写"idle"（小字斜体灰色）。框顶部小字"*K*=3 卷积窗口"，框底部小字"PE 利用率 = 3 / 16 = 18.75%"。
- **中央（折叠箭头 + 公式）**：画一根粗黑色实线水平箭头，长度约占画面宽度的 1/4，从左栏右侧伸向右栏左侧；箭头中央上方写"Ky-fold (compiler-side)"（Times New Roman 10 pt 加粗）；箭头下方两行小字（Times New Roman 8 pt）"cin_fake = (*k_y* mod *g*_y)·*C*ᵢₙ + *c_i*" 与 "*g*_y = ⌊16 / *C*ᵢₙ⌋ = 5"。
- **右栏（折叠后 / 等效卷积视角）**：画一个圆角矩形，标题"折叠后 (cin_fake)"（思源黑体 10 pt 加粗）。框内自上而下排列 16 个小方框（代表 PE 0..15）。前 15 个方框按 5 组 × 3 cin 着色：每组 3 个相邻方框使用同一种淡色（5 组分别用淡蓝、淡黄、淡绿、淡紫、淡橙的 CMYK 浅色填充），方框内分别小字写"*k_y*=0..4"与"*c_i*=0/1/2"；最后 1 个方框（PE 15）留白写"idle"。框顶部小字"*K_y_eff* = *kyper* = ⌈*K*/*g*_y⌉ = 1"，框底部小字"PE 利用率 = 15 / 16 = 93.75%"。

**底部图注**：画面底部居中两行小字（Times New Roman 8 pt 斜体）"DDR-friendly: y-shift replicate in cin only" 与 "Hardware view: equivalent stride=1 conv with extended cin; no RTL change"。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Compiler-side Ky-fold transformation principle**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (left-vs-right two-column comparison + central folding arrow)**:
- **Left column (before folding / native hardware view)**: Draw a rounded rectangle, title "Before folding (*C*ᵢₙ=3)" (Source Han Sans 10 pt bold). Inside, list 16 small boxes vertically (representing PE 0..15). Fill the first 3 boxes (PE 0/1/2) with light blue CMYK 30/10/0/0 and label "*c_i*=0", "*c_i*=1", "*c_i*=2" respectively. Leave the remaining 13 boxes blank and label "idle" (small italic gray). Top of the frame: "*K*=3 conv window" (small text); bottom of the frame: "PE utilization = 3 / 16 = 18.75%" (small text).
- **Center (folding arrow + formula)**: Draw a thick horizontal black solid arrow occupying about 1/4 of the figure width, originating from the right edge of the left column and pointing to the left edge of the right column. Above the arrow center: "Ky-fold (compiler-side)" (Times New Roman 10 pt bold). Below the arrow: two small lines (Times New Roman 8 pt) "cin_fake = (*k_y* mod *g*_y)·*C*ᵢₙ + *c_i*" and "*g*_y = ⌊16 / *C*ᵢₙ⌋ = 5".
- **Right column (after folding / equivalent conv view)**: Draw a rounded rectangle, title "After folding (cin_fake)" (Source Han Sans 10 pt bold). Inside, list 16 small boxes vertically (representing PE 0..15). Color the first 15 boxes in 5 groups × 3 cin: each group of 3 adjacent boxes shares one light tone (5 groups respectively in light blue, light yellow, light green, light purple, light orange CMYK soft fills). Label each box with "*k_y*=0..4" and "*c_i*=0/1/2" in small text. Leave the last box (PE 15) blank and label "idle". Top of the frame: "*K_y_eff* = *kyper* = ⌈*K*/*g*_y⌉ = 1" (small text); bottom of the frame: "PE utilization = 15 / 16 = 93.75%" (small text).

**Bottom caption**: Two small italic lines centered at the bottom (Times New Roman 8 pt) "DDR-friendly: y-shift replicate in cin only" and "Hardware view: equivalent stride=1 conv with extended cin; no RTL change".

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
