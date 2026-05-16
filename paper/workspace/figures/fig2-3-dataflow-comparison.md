# 图 2.3 WS / OS / RS 三种数据流对比
# Figure 2.3 Comparison of WS / OS / RS dataflows

## 在论文中的角色
- 首次引入：§2.3 数据流分类与硬件复用策略（paper.md L213-L231）
- 引用位置：§2.3.1 / §2.3.2 / §2.3.3 三小节均可引用此图
- 论证作用：用三 panel 并列把 Weight-Stationary / Output-Stationary / Row-Stationary 三种典型数据流在同一 PE 阵列底图上的数据流动方向作可视化对比。读者看到此图后应理解：WS 让权重固定、激活与 psum 流动；OS 让输出固定、权重与激活流动；RS 让行级最小卷积单元静止、三向复用。为 §3.5 选择"WS + 激活滑窗 + 输出通道广播"提供视觉背景。

## 图类型
三 panel 并列对比图（同一 PE 阵列底图 + 数据流箭头颜色编码）。

## 设计要素

### 必含元素
1. **三 panel 横向并列**：左 WS / 中 OS / 右 RS，三 panel 完全等宽。
2. **每 panel 内的 PE 阵列底图**：4×4 PE 网格（缩简版，象征任意 N×N 阵列），每 PE 用小方框表示。
3. **数据流颜色编码（三 panel 一致）**：
   - **红色实线箭头**：激活值（Activation）流动方向
   - **绿色实线箭头**：权重（Weight）流动方向
   - **蓝色实线箭头**：部分和（Psum）流动方向
   - **加粗黑色边框**：表示"静止"在 PE 内的数据
4. **三 panel 具体差异**：
   - **左 panel — WS（权重静止）**：每 PE 加粗黑色边框 + 内部小字"W"（权重静止）；红色箭头从左侧自上而下射入 PE（激活流入）；蓝色箭头从底部射出（psum 流出）。底部标注"代表：nn-X, NVDLA, FLUX_CNN"。
   - **中 panel — OS（输出静止）**：每 PE 加粗黑色边框 + 内部小字"O"（输出静止）；红色箭头从左侧射入；绿色箭头从顶部射入；无 psum 流出（在 PE 内累加完成后才输出）。底部标注"代表：ShiDianNao"。
   - **右 panel — RS（行静止）**：每 PE 加粗边框 + 内部小字"row"；红色箭头沿对角线方向流动（输入复用）；绿色箭头沿水平方向流动（权重复用）；蓝色箭头沿垂直方向流动（psum 累加）。底部标注"代表：Eyeriss"。
5. **图例**：底部居中一行"红=激活 (Activation) / 绿=权重 (Weight) / 蓝=部分和 (Psum) / 加粗边框=静止数据"。

### 标注要求
- 每 panel 顶部一行 panel 标题"WS / Weight-Stationary"等
- 每 panel 内 PE 阵列上方小字"PE Array"
- 每 panel 底部一行"代表实现：..."
- 三 panel 共享同一图例

### 视觉层次
- 主角：三 panel 内的 PE 阵列底图
- 配角：三向数据流箭头（红绿蓝颜色编码）
- 背景：panel 边框（淡灰色实线）

## 数据来源
- paper.md §2.3.1 / §2.3.2 / §2.3.3（三种数据流定义与代表实现）
- 文献 [13] Eyeriss、[12] ShiDianNao、[14] NVDLA、[10] nn-X
- contributions.md C1.1（FLUX_CNN 采用 WS + OC-broadcast）

## ASCII 示意稿

```
   ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
   │  WS              │  │  OS              │  │  RS              │
   │ (Weight-Static)  │  │ (Output-Static)  │  │ (Row-Stationary) │
   ├──────────────────┤  ├──────────────────┤  ├──────────────────┤
   │  PE Array        │  │  PE Array        │  │  PE Array        │
   │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│  │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│  │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│
   │ →│W│→│W│→│W│→│W││  │ ↓│O│ │O│ │O│ │O││  │ ↘│r│ │r│ │r│ │r││
   │   └─┘ └─┘ └─┘ └─┘│  │ →└─┘ └─┘ └─┘ └─┘│  │   └─┘ └─┘ └─┘ └─┘│
   │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│  │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│  │   ┌─┐ ┌─┐ ┌─┐ ┌─┐│
   │ →│W│ │W│ │W│ │W││  │ ↓│O│ │O│ │O│ │O││  │ →│r│ │r│ │r│ │r││
   │   └─┘ └─┘ └─┘ └─┘│  │ →└─┘ └─┘ └─┘ └─┘│  │   └─┘ └─┘ └─┘ └─┘│
   │   ↓ ↓ ↓ ↓ (psum) │  │   (psum 内蓄)    │  │   ↓ ↓ ↓ ↓ (psum) │
   │                  │  │                  │  │                  │
   │ 代表: nn-X, NVDLA│  │ 代表: ShiDianNao │  │ 代表: Eyeriss    │
   │       FLUX_CNN   │  │                  │  │                  │
   └──────────────────┘  └──────────────────┘  └──────────────────┘

   图例: ──▶ 红色 = 激活流动    ──▶ 绿色 = 权重流动    ──▶ 蓝色 = 部分和流动
         ▓ 加粗边框 = 静止数据
```

## 与正文的一致性检查
- [x] §2.3.1 "权重静止数据流的核心思想是将卷积核权重预先加载到 PE 阵列内的本地寄存器" — 左 panel 加粗 W
- [x] §2.3.1 "代表实现包括 nn-X 等早期加速器，以及 NVDLA 等工业级 ASIC 加速器" — 左 panel 底部标注
- [x] §2.3.2 "输出静止数据流将累加目标 ... 固定在 PE 内的本地寄存器" — 中 panel 加粗 O
- [x] §2.3.2 "代表实现包括 ShiDianNao" — 中 panel 底部标注
- [x] §2.3.3 "行静止数据流由 Eyeriss 提出 ... 多个 PE 通过纵向、横向、对角线三种方向的数据传递" — 右 panel 三向箭头
- [x] §3.5 "本设计选用权重静止 ... 复合数据流" — 左 panel 底部加上 FLUX_CNN 标注

## 不确定项
- [TBD: 是否在 WS panel 中额外标注"输出通道广播 (OC-broadcast)"以突出 FLUX_CNN 的复合数据流] — 倾向不在 §2 教科书图中突出本工作，保持中立；在 §3.5 重新画或追加正文一段说明

## image 生成提示词

### 中文版

科研论文配图，**WS / OS / RS 三种数据流对比图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1.2:1（横向稍长），适合 A4 单栏插图。**

**版式（三 panel 横向并列，间距均匀）**：
- 画面分为 3 个等宽的圆角矩形 panel，左中右排列，panel 之间间隔约 4% 画面宽度。每 panel 边框深灰色 #404040 实线、内部白色底。
- **每 panel 内部结构（自上而下）**：
  - 顶部标题：第一行 panel 编号 + 名称（思源黑体 11 pt 加粗），第二行英文全名（Times New Roman 9 pt 斜体）。左 panel "(a) WS / Weight-Stationary"，中 panel "(b) OS / Output-Stationary"，右 panel "(c) RS / Row-Stationary"。
  - 中部 PE 阵列：4×4 个小方框（每方框约 4 mm 宽，淡灰 CMYK 0/0/0/10 填充、深灰边框）。在每 panel 中，方框内部小字标注静止数据类型：
    - 左 panel：每方框内写"W"（加粗），且方框边框宽度增至 2 pt（表示权重静止）
    - 中 panel：每方框内写"O"（加粗），方框边框宽度增至 2 pt（表示输出静止）
    - 右 panel：每方框内写"row"（小字），方框边框宽度增至 2 pt（表示行静止）
  - 数据流箭头（重要——颜色编码）：
    - **左 panel WS**：从 PE 阵列左侧水平射入红色 #c00000 实线箭头（激活流入），从阵列底部向下射出蓝色 #1f4e79 实线箭头（psum 流出），权重无外部箭头（静止）。
    - **中 panel OS**：从阵列顶部向下射入绿色 #2e7d32 实线箭头（权重流入），从左侧水平射入红色 #c00000 箭头（激活流入），阵列底部无箭头（psum 在 PE 内累加完成后才发射）。
    - **右 panel RS**：阵列水平方向（左到右）画绿色实线箭头（权重水平复用），对角线方向（左上到右下）画红色实线箭头（激活对角线复用），垂直方向（上到下）画蓝色实线箭头（psum 垂直累加）。三向箭头交织呈现"三向复用"的视觉印象。
  - 底部一行小字（Times New Roman 9 pt）"代表实现 / Representative：" + 名单：
    - 左 panel："nn-X, NVDLA, FLUX_CNN"
    - 中 panel："ShiDianNao"
    - 右 panel："Eyeriss"
- **底部公共图例**：在三 panel 下方居中加一行小字"红 = 激活 Activation，绿 = 权重 Weight，蓝 = 部分和 Psum，加粗边框 = 静止数据 Stationary"，Times New Roman 9 pt。

**字体**：所有英文 Times New Roman 10 pt，panel 标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Comparison of WS / OS / RS dataflows**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1.2:1 (slightly wider), suitable for a single-column A4 figure.**

**Layout (three equal-width panels side by side)**:
- Split the canvas into 3 rounded-rectangle panels with a ~4% gap between them. Each panel has a dark-gray #404040 solid border and white interior.
- **Panel content (top to bottom)**:
  - Top title: line 1 panel label + name (Source Han Sans 11 pt bold), line 2 English full name (Times New Roman 9 pt italic). Left "(a) WS / Weight-Stationary", middle "(b) OS / Output-Stationary", right "(c) RS / Row-Stationary".
  - Middle PE array: a 4×4 grid of small square cells (~4 mm wide, light-gray CMYK 0/0/0/10 fill, dark-gray border). Cell-interior small label:
    - Left panel: each cell labeled "W" (bold), with thickened 2 pt border (weight-stationary).
    - Middle panel: each cell labeled "O" (bold), 2 pt border (output-stationary).
    - Right panel: each cell labeled "row" (small), 2 pt border (row-stationary).
  - Dataflow arrows (color-coded):
    - **Left panel (WS)**: From the left side, draw red #c00000 solid arrows flowing horizontally into the array (activation in); from the bottom, blue #1f4e79 solid arrows flowing downward out (psum out); weights have no external arrows (stationary).
    - **Middle panel (OS)**: From the top, green #2e7d32 solid arrows flowing down into the array (weight in); from the left, red #c00000 arrows flowing in (activation in); psum has no arrow (accumulated inside PE, only emitted at the end).
    - **Right panel (RS)**: Green horizontal arrows (left to right) representing weight horizontal reuse, red diagonal arrows (top-left to bottom-right) representing activation diagonal reuse, blue vertical arrows (top to bottom) representing psum vertical accumulation. The three arrow directions interlace to visualize three-way reuse.
  - Bottom line (Times New Roman 9 pt): "Representative: " followed by:
    - Left: "nn-X, NVDLA, FLUX_CNN"
    - Middle: "ShiDianNao"
    - Right: "Eyeriss"
- **Common legend at the very bottom**: Centered line "Red = Activation, Green = Weight, Blue = Psum, Bold border = Stationary data" in Times New Roman 9 pt.

**Typography**: All English in Times New Roman 10 pt with bold panel titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
