# 图 4.11 空间到深度（S2D）重排示意
# Figure 4.11 Space-to-depth transformation

## 在论文中的角色
- 首次引入：§4.9 编译器优化：空间到深度（S2D）（paper.md L474-L484）
- 引用位置：§4.9 整段
- 论证作用：把 §4.9 公式 (3)→(4) 的"stride² 个相位折到 cin 维"等价变换可视化为输入像素分相位重排示意图。读者应理解：stride=2 时输入 4×4 像素方块被切成 4 个相位（top-left / top-right / bot-left / bot-right），每相位独立抽取，4 相位拼接成 4×Cin 张量；硬件视角看到的是 stride=1、cin 加宽 4 倍的等效卷积。**该图为组合图：左半页面留白嵌算法 4.2 伪代码 PNG，右半画 stride=2 重排示意**。

## 图类型
组合图（左半留白嵌算法伪代码 PNG / 右半画 4×4 像素分相位重排示意 + 张量拼接）。

## 设计要素

### 必含元素
1. **左半页面（50% 宽度，留白）**：保留供 Word 编排时拼合算法 4.2 伪代码 PNG。Claude Design 出图时绘制为浅灰矩形占位框，中央"[伪代码图算法 4.2 留白区]"。
2. **右半页面（50% 宽度）— 三阶段重排示意（自上而下）**：
   - **阶段 1（原始 4×4 像素方块）**：右半顶部画一个 4×4 像素方块（16 个小方格），每格按相位编号用 4 种颜色染色：
     - 顶部左相位（r_y=0, r_x=0）：淡蓝 CMYK 30/10/0/0
     - 顶部右相位（r_y=0, r_x=1）：淡黄 CMYK 0/10/30/0
     - 底部左相位（r_y=1, r_x=0）：淡绿 CMYK 30/0/30/0
     - 底部右相位（r_y=1, r_x=1）：淡紫 CMYK 10/20/0/0
   - 4×4 像素方块中，每个 2×2 子块内 4 格按相位染色（不重复），整图共 4 个 2×2 子块，每子块代表 stride=2 卷积在该位置的 4 个相位。标题"原始输入特征图 (X) — stride=2"。
   - **阶段 2（分相位抽取，4 个子张量并排）**：阶段 1 下方画 4 个并列的子张量（每子张量为 2×2 缩小图），分别由同一相位的所有像素组成。每子张量用对应相位颜色填充，标题"phase (r_y, r_x) = (0,0) / (0,1) / (1,0) / (1,1)"。
   - **阶段 3（沿 cin 维拼接）**：阶段 2 下方画一个 2×2×（4×Cin）张量（用一个 2×2 方块表示空间维 + 一个 4×Cin 长度的深度方向标注），强调"4 相位沿 cin 维拼接为 cin_new = 4 × Cin"。标题"等效输入 (X') — stride=1, cin_new = 4·Cin"。
3. **变换箭头**：阶段之间各画一条粗黑色实线箭头，标"S2D rearrange (no data copy)"与"channel concat"。
4. **DDR 友好注**：右半底部加一行小字"输入数据在 DDR 中只是按相位重排索引，不需要复制（与 Ky-fold 的 y 偏移复制不同）"。

### 标注要求
- 每相位用一致颜色编码贯穿三阶段
- 阶段 1 标注"K=4, stride=4"或"K=3, stride=2"两种典型场景（首推 stride=2 + K=3，与论文 ResNet11 Patch 例对齐）
- 阶段 3 张量旁标"K_new = ⌈K/stride⌉, stride_new=1, cin_new = stride²·Cin"
- 左半伪代码区下方标"算法 4.2 / Algorithm 4.2"

### 视觉层次
- 主角：右半阶段 1（4×4 输入像素方块，4 种颜色分相位）
- 配角：阶段 2 子张量并排 + 阶段 3 cin 拼接
- 背景：左半页面伪代码占位框

## 数据来源
- paper.md §4.9 L476-L482（S2D 数学推导与公式 (3)/(4)）
- contributions.md C2.2（S2D 编译器侧无复制重排）
- docs/pe-fold.md（S2D 数学推导原文）
- STATUS §2.8（ResNet11 Patch 层 K=4/stride=4 → K=1/stride=1/cin=64，整网周期 654K→129K）

## ASCII 示意稿

```
   ┌─────────────────────┬──────────────────────────────────────────┐
   │                     │  阶段 1: 原始输入 X (stride=2)            │
   │                     │  ┌──┬──┬──┬──┐                           │
   │                     │  │蓝│黄│蓝│黄│  ← row 0                  │
   │                     │  ├──┼──┼──┼──┤                           │
   │                     │  │绿│紫│绿│紫│  ← row 1                  │
   │                     │  ├──┼──┼──┼──┤                           │
   │                     │  │蓝│黄│蓝│黄│  ← row 2                  │
   │  [伪代码图           │  ├──┼──┼──┼──┤                           │
   │   算法 4.2           │  │绿│紫│绿│紫│  ← row 3                  │
   │   留白区             │  └──┴──┴──┴──┘                           │
   │   Pseudo-code        │       │ S2D rearrange (no copy)         │
   │   placeholder        │       ▼                                  │
   │   for Algorithm 4.2] │  阶段 2: 4 相位抽取 (并排子张量)         │
   │                     │  ┌──┐ ┌──┐ ┌──┐ ┌──┐                    │
   │                     │  │蓝│ │黄│ │绿│ │紫│  各 2×2             │
   │                     │  │蓝│ │黄│ │绿│ │紫│                     │
   │                     │  └──┘ └──┘ └──┘ └──┘                     │
   │                     │ (0,0) (0,1) (1,0) (1,1)                  │
   │                     │       │ channel concat                   │
   │                     │       ▼                                   │
   │                     │  阶段 3: 等效输入 X' (stride=1)           │
   │                     │  ┌──────┬──────┐ cin_new = 4 × Cin        │
   │                     │  │ 2×2  │ 2×2  │ K_new = ⌈K/stride⌉      │
   │                     │  ├──────┼──────┤ stride_new = 1           │
   │                     │  │ 2×2  │ 2×2  │ depth: ▓▓▓▓ 4·Cin       │
   │                     │  └──────┴──────┘                          │
   └─────────────────────┴──────────────────────────────────────────┘
   注: 输入在 DDR 中仅按相位重排索引，不需要复制（与 Ky-fold 不同）
```

## 与正文的一致性检查
- [x] §4.9 "将 (k_x mod stride, k_y mod stride) 的 stride² 个相位折到 cin 维" — 阶段 2 四相位抽取
- [x] §4.9 "等价转换为 stride=1 的卷积" — 阶段 3 stride_new=1
- [x] §4.9 "S2D 的核心性质是输入数据在 DDR 中只是按相位重排索引，不需要复制" — 阶段 1→阶段 2 箭头标"no data copy" + 底部注释
- [x] §4.9 公式 (4) "K_new = ⌈K/s⌉, cin_new = s²·Cin" — 阶段 3 标注
- [x] §4.9 "ResNet11 Patch 层（K=4、stride=4）实测显示 ... 单层加速 5.05×" — 可在图注里加一行小字（可选）

## 不确定项
- [TBD: 主图示例用 stride=2 K=3 还是 stride=4 K=4（ResNet11 Patch）] — 倾向 stride=2 K=3（图更紧凑、4 相位足够清晰）；ResNet11 Patch 用数字在底注里提到
- [TBD: 阶段 3 是否画立体张量（含 cin 深度方向）还是只画 2D + 深度数值标注] — 倾向 2D + 深度标注（避免 3D 立体）

## image 生成提示词

### 中文版

科研论文配图，**空间到深度（S2D）重排示意图（组合图）**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），适合 A4 单栏插图。整张图分左右两栏：左半页面 50% 留白给伪代码 PNG 嵌入；右半页面 50% 绘制三阶段重排示意**。

**左半页面（50% 宽度，伪代码占位）**：
- 整块用浅灰 CMYK 0/0/0/10 填充的圆角矩形占位（边框为深灰色 #404040 虚线）。
- 矩形中央两行小字"[伪代码图算法 4.2 留白区]"与"[Pseudo-code placeholder for Algorithm 4.2]"，思源黑体 / Times New Roman 10 pt 斜体。
- 下方一行小字"算法 4.2 S2D 编译器重排 / Algorithm 4.2 S2D compiler rearrangement"（思源黑体 9 pt 加粗）。

**右半页面（50% 宽度，自上而下三阶段）**：
- **阶段 1（占右半高度 33%）— 原始输入 X (stride=2)**：标题"阶段 1：原始输入 X (stride=2)"（思源黑体 10 pt 加粗）。绘制一个 4×4 像素方块（16 个小方格，每格约 5 mm 见方，黑色 0.5 pt 边框）。每 2×2 子块内 4 格按相位染色（不重复）：
  - 位置 (0,0)(0,2)(2,0)(2,2) — 淡蓝 CMYK 30/10/0/0
  - 位置 (0,1)(0,3)(2,1)(2,3) — 淡黄 CMYK 0/10/30/0
  - 位置 (1,0)(1,2)(3,0)(3,2) — 淡绿 CMYK 30/0/30/0
  - 位置 (1,1)(1,3)(3,1)(3,3) — 淡紫 CMYK 10/20/0/0
- 阶段 1 与阶段 2 之间画一条向下的深灰色 #404040 粗箭头（线宽 1.5 pt），右侧标"S2D rearrange (no data copy)"（Times New Roman 9 pt 斜体）。
- **阶段 2（占右半高度 27%）— 4 相位抽取**：标题"阶段 2：分相位抽取（4 子张量并排）"。水平排列 4 个 2×2 子张量，每子张量为 2×2 小方格（保持阶段 1 同色）：
  - 子张量 (0,0)：全淡蓝
  - 子张量 (0,1)：全淡黄
  - 子张量 (1,0)：全淡绿
  - 子张量 (1,1)：全淡紫
- 每子张量下方标小字"phase (r_y, r_x) = (0,0)/(0,1)/(1,0)/(1,1)"（Times New Roman 8 pt）。
- 阶段 2 与阶段 3 之间画一条向下的深灰色粗箭头，右侧标"channel concat (沿 cin 维拼接)"（Times New Roman 9 pt 斜体）。
- **阶段 3（占右半高度 30%）— 等效输入 X' (stride=1)**：标题"阶段 3：等效输入 X' (stride=1)"。绘制一个 2×2 大方块，每格内部以 4 个并列窄条（淡蓝 / 淡黄 / 淡绿 / 淡紫）表示 cin 维上的 4 相位拼接。方块右侧标三行参数"K_new = ⌈K/stride⌉""stride_new = 1""cin_new = stride² · Cin = 4·Cin"（Times New Roman 9 pt）。
- **底部注释**：右半底部居中一行小字（Times New Roman 8 pt 斜体）"DDR-friendly: input data only re-indexed by phase, NOT duplicated (unlike Ky-fold)"，再加一行中文"DDR 友好：输入数据仅按相位重排索引，不需要复制（与 Ky-fold 不同）"。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。4 种相位色用纯色非渐变。整体保持工科论文严谨风格。**左半页面 50% 区域保持纯净占位，不绘制任何额外元素**。

### English version

Scientific paper figure, **Space-to-depth (S2D) transformation schematic (composite figure)**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square) or 1:1.2 (slightly taller), suitable for a single-column A4 figure. Canvas split into a 50% left column (pseudo-code placeholder) and a 50% right column (three-stage rearrangement schematic).**

**Left column (50% width, pseudo-code placeholder)**:
- Fill the column with a light-gray CMYK 0/0/0/10 rounded rectangle (dashed dark-gray #404040 border).
- Center two italic lines: "[Pseudo-code placeholder for Algorithm 4.2]" (Times New Roman 10 pt italic) and a Chinese counterpart.
- Beneath: small bold caption "Algorithm 4.2 S2D compiler rearrangement" (Source Han Sans 9 pt bold).

**Right column (50% width, three vertical stages)**:
- **Stage 1 (33% height) — Original input X (stride=2)**: Title "Stage 1: Original input X (stride=2)" (Source Han Sans 10 pt bold). Draw a 4×4 pixel grid (16 small squares, each ~5 mm, 0.5 pt black border). Color each cell by phase (no repeats within a 2×2 sub-block):
  - Positions (0,0)(0,2)(2,0)(2,2) — light-blue CMYK 30/10/0/0
  - Positions (0,1)(0,3)(2,1)(2,3) — light-yellow CMYK 0/10/30/0
  - Positions (1,0)(1,2)(3,0)(3,2) — light-green CMYK 30/0/30/0
  - Positions (1,1)(1,3)(3,1)(3,3) — light-purple CMYK 10/20/0/0
- Between Stage 1 and Stage 2 draw a downward dark-gray #404040 thick arrow (1.5 pt) with caption "S2D rearrange (no data copy)" (Times New Roman 9 pt italic).
- **Stage 2 (27% height) — 4-phase extraction**: Title "Stage 2: Per-phase extraction (4 sub-tensors side by side)". Horizontally arrange 4 2×2 sub-tensors keeping Stage 1 colors:
  - Sub-tensor (0,0): all light-blue
  - Sub-tensor (0,1): all light-yellow
  - Sub-tensor (1,0): all light-green
  - Sub-tensor (1,1): all light-purple
- Below each sub-tensor caption "phase (r_y, r_x) = (0,0)/(0,1)/(1,0)/(1,1)" (Times New Roman 8 pt).
- Between Stage 2 and Stage 3 draw a downward thick arrow with caption "channel concat (along cin)" (Times New Roman 9 pt italic).
- **Stage 3 (30% height) — Equivalent input X' (stride=1)**: Title "Stage 3: Equivalent input X' (stride=1)". Draw a 2×2 large square; inside each cell place 4 parallel narrow stripes (light-blue / light-yellow / light-green / light-purple) representing the 4 phases concatenated along cin. To the right list three parameters: "K_new = ⌈K/stride⌉", "stride_new = 1", "cin_new = stride² · Cin = 4·Cin" (Times New Roman 9 pt).
- **Bottom caption**: Centered italic line at the bottom (Times New Roman 8 pt) "DDR-friendly: input data only re-indexed by phase, NOT duplicated (unlike Ky-fold)", with a Chinese counterpart above.

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Use solid pure colors (no gradients) for the four phases. Maintain rigorous engineering-paper aesthetics throughout. **Keep the left 50% region as a clean placeholder with only the caption text — no additional elements**.
