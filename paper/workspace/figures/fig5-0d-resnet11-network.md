# 图 5.0d ResNet11 网络结构
# Figure 5.0d ResNet11 network architecture

## 在论文中的角色
- 首次引入：§5.3.4 ResNet11 完整网络验证（paper.md L576-L593）
- 引用位置：§5.3.4 整段、§5.4 资源占用对比的网络锚点、§5.5 性能分析 ResNet11 整网行
- 论证作用：把 FLUX_CNN 用作主要 benchmark 的 ResNet11 网络结构横向流式拓扑可视化，让读者理解 11 层的层间形状演化（Patch + 3 个残差块 + FC）与残差短路连接的物理位置。该图是 §5.3.4 / §5.4 / §5.5 多处性能数据的网络锚点。

## 图类型
横向流式网络拓扑图（11 层方块横向串联 + 残差短路虚线 + 4 色层类型编码）。

## 设计要素

### 必含元素
1. **横向流式拓扑（自左而右 11 层方块）**：
   - **Input**（标"540×960×4"）→
   - **Patch**（K=4, s=4, 4→16）→
   - **Main 1**（K=3, s=2, 16→16）→
   - **Main 2**（K=3, s=1, 16→16）→ + **Downsample 1**（K=1, s=2, 16→16，虚线短路）→
   - **Main 3**（K=3, s=2, 16→32）→
   - **Main 4**（K=3, s=1, 32→32）→ + **Downsample 2**（K=1, s=2, 16→32，虚线短路）→
   - **Main 5**（K=3, s=2, 32→64）→
   - **Main 6**（K=3, s=1, 64→64）→ + **Downsample 3**（K=1, s=2, 32→64，虚线短路）→
   - **FC**（256→522）
2. **每层方块标注**：4 行小字
   - 第 1 行：层名（如"Main 1"）
   - 第 2 行：(K, s)（如"K=3, s=2"）
   - 第 3 行：(Cin → Cout)（如"16→16"）
   - 第 4 行：(H × W)（如"68×120"）
3. **残差短路连接（3 条虚线）**：
   - **残差 1**：从 Main 1 输入端 → Downsample 1 → Main 2 输出端（虚线弧形）
   - **残差 2**：从 Main 3 输入端 → Downsample 2 → Main 4 输出端
   - **残差 3**：从 Main 5 输入端 → Downsample 3 → Main 6 输出端
4. **4 色层类型编码**：
   - **Patch（K=4, s=4）**：深金色 #b8860b 填充
   - **Main（K=3 主路径）**：深蓝色 #1f4e79 填充
   - **Downsample（K=1, s=2 短路）**：深绿色 #2e7d32 填充
   - **FC（全连接）**：深灰色 #404040 填充
5. **底部图例**：4 色 + 4 类层名（"■ Patch / ■ Main / ■ Downsample / ■ FC"）
6. **顶部标题与输入输出**：左侧标"Input 540×960×4 RGBA / 灰度4通道"，右侧标"FC output 522 classes"

### 标注要求
- 每层 H×W 需准确（按 paper.md §5.3.4 / contributions.md 数据）：Input 540×960、Patch 后 135×240、Main1 后 68×120、Main3 后 34×60、Main5 后 17×30
- 残差短路用虚线 + 圆弧标"+"或"⊕"汇合点
- FC 输出 522 类（按 paper.md 数据；若实测是 256→522 则保留）

### 视觉层次
- 主角：11 层方块横向串联
- 配角：3 条残差短路虚线弧
- 背景：底部 4 色图例

## 数据来源
- paper.md §5.3.4 L576-L578（ResNet11: 1 Patch + 3 残差块 + 1 FC = 11 层）
- contributions.md / STATUS.md（ResNet11 层参数）
- model_analysis.md（目标模型 PE 利用率分析）
- 标准 ResNet 范式（He et al.）

## ASCII 示意稿

```
                                                Residual 1                                  Residual 2                                  Residual 3
                                              ┌─────────────────────────────┐             ┌─────────────────────────────┐             ┌─────────────────────────────┐
                                              │                             │             │                             │             │                             │
                                              ▼                             │             ▼                             │             ▼                             │
   ┌──────┐  ┌────────┐  ┌────────┐  ┌────────┐│┌──────────┐  ┌────────┐  ┌────────┐│┌──────────┐  ┌────────┐  ┌────────┐│┌──────────┐  ┌────────┐
   │Input │  │ Patch  │  │ Main 1 │  │ Main 2 │││Downsamp 1│  │ Main 3 │  │ Main 4 │││Downsamp 2│  │ Main 5 │  │ Main 6 │││Downsamp 3│  │  FC    │
   │540×  │─▶│ K=4    │─▶│ K=3    │─▶│ K=3    │┼┤ K=1, s=2 │─▶│ K=3    │─▶│ K=3    │┼┤ K=1, s=2 │─▶│ K=3    │─▶│ K=3    │┼┤ K=1, s=2 │─▶│ 256→522│
   │960×  │  │ s=4    │  │ s=2    │  │ s=1    │ │ 16→16     │  │ s=2    │  │ s=1    │ │ 16→32     │  │ s=2    │  │ s=1    │ │ 32→64     │  │        │
   │4 ch  │  │ 4→16   │  │ 16→16  │  │ 16→16  │ │           │  │ 16→32  │  │ 32→32  │ │           │  │ 32→64  │  │ 64→64  │ │           │  │        │
   │      │  │135×240 │  │ 68×120 │  │ 68×120 │ │           │  │ 34×60  │  │ 34×60  │ │           │  │ 17×30  │  │ 17×30  │ │           │  │        │
   └──────┘  └────────┘  └────────┘  └────────┘  └──────────┘  └────────┘  └────────┘  └──────────┘  └────────┘  └────────┘  └──────────┘  └────────┘
    (金色)    (金)        (蓝)       (蓝)         (绿短路)      (蓝)       (蓝)         (绿短路)      (蓝)       (蓝)         (绿短路)      (灰)

   图例: ■ Patch (K=4, s=4)  ■ Main (K=3 主路径)  ■ Downsample (K=1, s=2 短路)  ■ FC (全连接)
   注: 输入 540×960×4 → FC 输出 522 类；空间分辨率自左而右递减；通道数递增
```

## 与正文的一致性检查
- [x] §5.3.4 "ResNet11 包含 1 个 Patch 卷积（K=4、stride=4）" — Patch 层方块
- [x] §5.3.4 "3 个残差块（每块含 2 层卷积加 1 条残差路径）" — Main 1-2 + Downsample 1、Main 3-4 + Downsample 2、Main 5-6 + Downsample 3
- [x] §5.3.4 "1 个全连接层共 11 层" — FC 层方块
- [x] §5.3.4 "三种配置下整网输出与 PyTorch 端的浮点参考 11/11 层逐字节匹配" — 图标题 ResNet11
- [x] §4.9 "ResNet11 Patch 层（K=4、stride=4、Cin=4）" — Patch 层标注 K=4 s=4 4→16

## 不确定项
- [TBD: Main 1 的 H×W 精确值] — 按 paper.md 540×960 输入 → Patch s=4 后 135×240 → Main1 s=2 后 68×120 推算（向上取整）；若 model_analysis.md 给出精确值以后者为准
- [TBD: FC 是 256→522 还是其他维度] — 用户在任务描述中写"256→522"，按此标注；若与 paper.md 不符以 paper.md 为准

## image 生成提示词

### 中文版

科研论文配图，**ResNet11 网络结构横向拓扑图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1.5:1（横向较长）以容纳 11 层方块，必要时分两行排布以满足 A4 单栏 1:1.2 限制。**

**版式（横向流式拓扑，11 层方块串联，3 条残差短路弧）**：
- **顶部标题区**：画面左上角小字"ResNet11 网络结构 / ResNet11 network architecture"（思源黑体 10 pt 加粗）。
- **主流方块（自左而右 11 个等高小方框）**：每方块约画面 7%×30% 大小，按层类型 4 色填充：
  - **Patch 层（位置 2）**：深金色 #b8860b 实心填充 + 深灰 #404040 边框
  - **Main 层（位置 3、4、6、7、9、10）**：深蓝 #1f4e79 实心填充
  - **Downsample 层（位置 5、8、11，但 11 实际是 FC，所以 Downsample 是 5、8、11 中的 5、8 与一个内嵌项；按 ASCII 图布局共 3 个）**：深绿 #2e7d32 实心填充
  - **FC 层（最右端）**：深灰 #404040 实心填充
  - **Input（最左端）**：淡灰 CMYK 0/0/0/10 填充（不计入 11 层）
- 主流方块自左而右严格按序：Input → Patch → Main 1 → Main 2 → Downsample 1 → Main 3 → Main 4 → Downsample 2 → Main 5 → Main 6 → Downsample 3 → FC。方块之间用细黑色实线箭头连接。
- **每方块内 4 行小字**（白色字 Times New Roman 8 pt 加粗，便于在深色填充上可读）：
  - 行 1 层名（如"Main 1"）
  - 行 2 (K, s)（如"K=3, s=2"）
  - 行 3 (Cin → Cout)（如"16→16"）
  - 行 4 (H × W)（如"68×120"）
- **残差短路弧（3 条）**：每条短路从对应残差块入口（Main i 输入端）向上画一条深红色 #c00000 虚线弧（线宽 1.5 pt），经过 Downsample 方块上方（虚线穿过该方块）后回到 Main(i+1) 输出端的"⊕"汇合点（一个小圆圈标"+"）。3 条短路弧分别覆盖 Main 1/2 + Downsample 1、Main 3/4 + Downsample 2、Main 5/6 + Downsample 3。弧线中段标小字"Residual 1/2/3"（Times New Roman 8 pt 斜体）。
- **底部图例**：画面下方居中四色方块图例 + 文字，单行："■ Patch (K=4,s=4) ■ Main (K=3 主路径) ■ Downsample (K=1,s=2 短路) ■ FC (全连接)"（Times New Roman 9 pt）。
- **底部注释**：再下方一行小字（Times New Roman 8 pt 斜体）"输入 540×960×4 → FC 输出 522 类；空间分辨率自左而右递减，通道数自左而右递增 / Input 540×960×4 → FC out 522 classes"。

**字体**：所有英文 Times New Roman 10 pt，方块内字 8 pt 加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。深色填充方块内的字必须用白色以保持对比；浅色填充方块（Input）内的字用黑色。整体保持工科论文严谨风格。**若 11 层横向排不下，可按 6 + 5 分两行布局，第一行 Input→Main 2，第二行 Downsample 1→FC，残差短路虚线跨行**。

### English version

Scientific paper figure, **ResNet11 network architecture (horizontal flow topology)**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1.5:1 (wider) to fit 11 layer blocks; if needed split into 2 rows to satisfy single-column A4 1:1.2 limit.**

**Layout (horizontal flow, 11 layer blocks in series, 3 residual shortcuts)**:
- **Top title**: Upper-left small bold label "ResNet11 network architecture" (Source Han Sans 10 pt bold).
- **Main-flow blocks (11 equal-height rectangles)**: Each block ~7%×30% of canvas, filled by layer type:
  - **Patch (position 2)**: deep-gold #b8860b solid fill + dark-gray #404040 border
  - **Main (positions 3, 4, 6, 7, 9, 10)**: deep-blue #1f4e79 solid fill
  - **Downsample (positions 5, 8, 11 — but position 12 is FC; 3 downsamples total)**: deep-green #2e7d32 solid fill
  - **FC (rightmost)**: dark-gray #404040 solid fill
  - **Input (leftmost)**: light-gray CMYK 0/0/0/10 fill (not counted in the 11 layers)
- Block order left-to-right: Input → Patch → Main 1 → Main 2 → Downsample 1 → Main 3 → Main 4 → Downsample 2 → Main 5 → Main 6 → Downsample 3 → FC. Connect blocks with thin solid black arrows.
- **Inside each block, 4 lines of small text** (white Times New Roman 8 pt bold for legibility on dark fills):
  - Line 1: layer name (e.g., "Main 1")
  - Line 2: (K, s) (e.g., "K=3, s=2")
  - Line 3: (Cin → Cout) (e.g., "16→16")
  - Line 4: (H × W) (e.g., "68×120")
- **Residual shortcut arcs (3)**: Each shortcut starts at the entry of a residual block (Main i input), arcs upward in a deep-red #c00000 dashed curve (1.5 pt), passes above the Downsample block, and lands at a "+" merge point at Main (i+1) output. The 3 arcs cover Main 1/2 + Downsample 1, Main 3/4 + Downsample 2, Main 5/6 + Downsample 3. Label each arc mid-segment "Residual 1/2/3" (Times New Roman 8 pt italic).
- **Bottom legend**: Centered single line with color squares and text: "■ Patch (K=4, s=4) ■ Main (K=3 path) ■ Downsample (K=1, s=2 shortcut) ■ FC" (Times New Roman 9 pt).
- **Bottom caption**: One italic line (Times New Roman 8 pt) "Input 540×960×4 → FC output 522 classes; spatial resolution decreases, channel count increases left-to-right".

**Typography**: All English in Times New Roman 10 pt; block-interior text in white 8 pt bold (for contrast on dark fills); Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Use white text on dark fills and black text on light fills for legibility. **If 11 blocks don't fit horizontally, lay them out in two rows (6 + 5) with shortcuts spanning the row break**.
