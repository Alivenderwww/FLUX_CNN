# 图 3.6 三层数据复用叠加
# Figure 3.6 Three-tier data reuse overlay

## 在论文中的角色
- 首次引入：§3.5 数据流选择（paper.md L328-L340）
- 引用位置：§3.5 整段，特别是 L332（权重静止）/ L334（滑窗复用）/ L336（输出通道广播）
- 论证作用：把 §3.5 三段陈述的"权重静止 / 激活值滑窗复用 / 输出通道广播"三种数据复用机制在同一张 16×16 PE 阵列底图上以三色箭头叠加可视化，让读者在一张图内同时看到三层复用如何共同作用于 FLUX_CNN 的 16×16 阵列。该图是 §3.5 复合数据流的核心可视化。

## 图类型
单底图三向数据流叠加示意图（16×16 PE 网格 + 三色箭头叠加 + 图例分类）。

## 设计要素

### 必含元素
1. **底图 — 16×16 PE 阵列**：16 列 × 16 行小方框，构成完整的 FLUX_CNN MAC 阵列底图。每个小方框宽高约 4 mm，淡灰 CMYK 0/0/0/10 填充、深灰 #404040 细实线边框。
2. **第一层复用 — 权重静止（红色叠加）**：在每个 PE 方框内放一个小红色 #c00000 实心圆点，旁边小字"W"，表示每 PE 持有 1 个权重保持不变。底部图例"红 ● = 权重静止 (每 PE 内 W 保持)"。
3. **第二层复用 — 激活值滑窗复用（蓝色叠加）**：在阵列上方画一个 3×3（K=3）的蓝色方框窗口（淡蓝 CMYK 30/10/0/0 填充、深蓝 #1f4e79 边框、加粗），表示"当前 K×K 卷积窗口"；窗口右上角画一条向右的水平箭头（深蓝实线）+ "→ 滑窗 (sliding)"标注，表示窗口在输入特征图上水平滑动；窗口下方画一条向下的细虚线连接到阵列，象征激活值通过 ARF 馈入。底部图例"蓝 ▢ = 激活滑窗 (K×K 窗口在 ARF 内复用)"。
4. **第三层复用 — 输出通道广播（绿色叠加）**：在阵列左侧（每行起始位置）画一个深绿色 #2e7d32 大圆形分发节点（标"act"），从分发节点出发画 16 条水平绿色实线箭头（一行一条），分别射向 16 列的同一行 PE，表示"同一拍激活值向量同时广播到 16 列"。底部图例"绿 ─▶ = 输出通道广播 (1 拍 act → 16 列同时消费)"。
5. **顶部标题与方向标注**：阵列顶部标"16 列 (Cout 方向, 16 路输出通道)"，左侧纵向标"16 PE (Cin 方向, 16 路并行)"。

### 标注要求
- 三色箭头叠加但不互相覆盖（红点在 PE 内部、蓝窗在阵列上方、绿箭头在阵列左侧水平射入）
- 每复用机制对应一行图例（颜色 + 符号 + 中英文说明）
- 顶部 / 左侧方向标注用箭头 + 文字（"Cout →" / "↑ Cin"）

### 视觉层次
- 主角：16×16 PE 阵列底图
- 配角：三色叠加箭头（红 / 蓝 / 绿）
- 背景：图例（底部居中三行）

## 数据来源
- paper.md §3.5 L332（权重静止 + WB/WRF 双层）
- paper.md §3.5 L334（激活值滑窗复用 + ARF）
- paper.md §3.5 L336（输出通道广播 + 16 列并行）
- paper.md §3.5 L338（16×16 PE 阵列与 cout/cin 自然对齐）
- contributions.md C1.1（OC-broadcast 数据流）

## ASCII 示意稿

```
        ←─────────────  Cout 方向 (16 列输出通道) ─────────────→
              ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ...  (3×3 滑窗)
              │蓝│ │蓝│ │蓝│ │ │ │ │ │ │ │ │ │ │ │ │       ▢→ 滑窗
              └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ ...

              ▼ act 广播
   ┌act┐ ──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐ ... (16 列同时消费)
   │ ● │    │ ● │   │ ● │   │ ● │   │ ● │   ← row 0
   └───┘    └───┘   └───┘   └───┘   └───┘
   ┌act┐ ──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐ ...
   │ ● │    │ ● │   │ ● │   │ ● │   │ ● │   ← row 1
   └───┘    └───┘   └───┘   └───┘   └───┘
   :  :       :       :       :       :        :
   ┌act┐ ──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐──▶┌─W─┐ ...
   │ ● │    │ ● │   │ ● │   │ ● │   │ ● │   ← row 15
   └───┘    └───┘   └───┘   └───┘   └───┘

   ↑
   │ Cin 方向 (16 PE 并行)
   ↓

   图例:
   红 ● = 权重静止 (每 PE 内 W 保持，WB→WRF 加载后不变)
   蓝 ▢ = 激活滑窗 (K×K 窗口在 ARF=32 内复用，每 yout/xout 滑动)
   绿 ─▶ = 输出通道广播 (1 拍 act 向量 → 16 列同时消费)
```

## 与正文的一致性检查
- [x] §3.5 "权重静止 ... 让每个 PE 持有的权重保持不变，重复用于多次乘加运算" — 红点在每 PE 内
- [x] §3.5 "WB 与 WRF=32 的双层结构 ... 一层卷积启动时一次性读入 WRF" — 图例红点说明
- [x] §3.5 "激活值滑窗复用利用了相邻空间位置卷积窗口的列重叠性 ... 通过行缓存模块的环形缓冲实现" — 蓝色 3×3 窗口 + 滑动箭头
- [x] §3.5 "ARF=32 缓存当前窗口供 PE 重复读取" — 图例蓝窗说明
- [x] §3.5 "输出通道广播 ... 16 列 PE 配置为 16 个并行输出通道，每周期由行缓存模块广播 1 拍激活值向量到 16 列" — 绿色水平箭头 + 16 行广播
- [x] §3.5 "16×16 PE 阵列在片上呈现每列一个输出通道、每列内沿 Cin 方向 16 PE 并行的二维结构" — 顶部 Cout 方向标注 + 左侧 Cin 方向标注

## 不确定项
- [TBD: 是否把 K×K 滑窗画在阵列上方独立位置，还是覆盖在阵列内某行] — 倾向画在上方独立位置（不遮挡阵列底图），用虚线连接到阵列
- [TBD: 是否标注 ARF / WRF / PARF 的物理位置] — 倾向不在主图中标，避免复杂度；§4 各模块图分别详述

## image 生成提示词

### 中文版

科研论文配图，**16×16 PE 阵列三层数据复用叠加示意图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），适合 A4 单栏插图。**

**版式（单底图 + 三层箭头叠加）**：
- **画面正中绘制 16×16 PE 阵列底图**：16 列 × 16 行小方框，整体阵列约占画面 60%×60%。每方框宽高约 4 mm，淡灰 CMYK 0/0/0/10 填充、深灰 #404040 细实线边框（线宽 0.5 pt）。每方框内放一个小红色 #c00000 实心圆点（直径约 1.5 mm），象征"权重静止"；圆点旁加小字"W"（Times New Roman 7 pt 加粗红色）。
- **顶部方向标注**：阵列顶部水平居中一行"16 列 Cout 方向（输出通道并行）"，箭头"→"指向右侧。
- **左侧方向标注**：阵列左侧纵向一行"16 PE Cin 方向（输入通道并行）"，箭头"↑"指向上方。
- **第二层叠加 — 激活值滑窗复用（蓝色）**：在阵列上方约 5 mm 处放一个 3×3 蓝色方框（淡蓝 CMYK 30/10/0/0 填充、深蓝 #1f4e79 加粗边框 1.5 pt），代表"K=3 卷积窗口"。窗口右侧外画一条向右的深蓝实线箭头（线宽 1.5 pt），标小字"滑窗 sliding (yout/xout)"。窗口下方画一条向下的深蓝细虚线（线宽 0.5 pt）连接到阵列顶部，象征激活值通过 ARF=32 馈入阵列。
- **第三层叠加 — 输出通道广播（绿色）**：在阵列左侧每行起点（共 16 行）各画一个深绿色 #2e7d32 实心圆（直径约 3 mm，标小字"act"），从该圆出发向右画一条水平深绿色实线箭头（线宽 1 pt），贯穿对应行的 16 列 PE，象征"1 拍激活值向量同时广播到 16 列"。16 行各画一条，共 16 条绿色水平箭头。
- **底部图例（三行）**：在阵列下方居中三行小字（Times New Roman 9 pt），每行前加颜色符号：
  - 第 1 行："红 ● = 权重静止 (每 PE 内 W 保持，WB→WRF 加载后不变) / Red ● = Weight-stationary"
  - 第 2 行："蓝 ▢ = 激活滑窗 (K×K 窗口在 ARF=32 内复用) / Blue ▢ = Sliding-window reuse"
  - 第 3 行："绿 ─▶ = 输出通道广播 (1 拍 act → 16 列同时消费) / Green ─▶ = Output-channel broadcast"

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。三色（红 / 蓝 / 绿）使用纯色非渐变。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Three-tier data reuse overlay on a 16×16 PE array**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square), suitable for a single-column A4 figure.**

**Layout (single base array + three overlay layers)**:
- **Center of canvas — 16×16 PE array**: 16 columns × 16 rows of small square cells, occupying about 60%×60% of the canvas. Each cell ~4 mm, light-gray CMYK 0/0/0/10 fill, dark-gray #404040 thin border (0.5 pt). Inside each cell, place a small red #c00000 filled circle (~1.5 mm dia) representing weight stationarity, with a small bold "W" label (Times New Roman 7 pt red).
- **Top direction label**: Above the array, centered "16 columns — Cout direction (output channel parallel)", with arrow "→".
- **Left direction label**: To the left of the array, vertical "16 PEs — Cin direction (input channel parallel)", with arrow "↑".
- **Overlay 2 — Activation sliding-window reuse (blue)**: About 5 mm above the array place a 3×3 blue square frame (light-blue CMYK 30/10/0/0 fill, deep-blue #1f4e79 bold border 1.5 pt) representing the K=3 conv window. To its right draw a horizontal deep-blue solid arrow (1.5 pt) with caption "sliding (yout/xout)". Below the window draw a thin deep-blue dashed vertical line connecting to the top of the array, indicating activation feed via ARF=32.
- **Overlay 3 — Output-channel broadcast (green)**: At the left starting point of each of the 16 rows, draw a deep-green #2e7d32 filled circle (~3 mm dia, label "act"). From each circle draw a horizontal deep-green solid arrow (1 pt) extending rightward across all 16 columns in that row, representing one activation vector broadcast to 16 columns simultaneously. Draw 16 such arrows total.
- **Bottom legend (three lines, Times New Roman 9 pt)**:
  - Line 1: "Red ● = Weight-stationary (W held per PE; WB→WRF loaded once)"
  - Line 2: "Blue ▢ = Sliding-window reuse (K×K reused in ARF=32)"
  - Line 3: "Green ─▶ = Output-channel broadcast (1 cycle act → 16 columns)"

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Use solid pure red / blue / green (no gradients). Maintain rigorous engineering-paper aesthetics throughout.
