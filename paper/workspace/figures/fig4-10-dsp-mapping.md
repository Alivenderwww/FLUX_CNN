# 图 4.10 DSP 块跨列复用映射方案原理图
# Figure 4.10 Cross-column DSP-block mapping scheme

## 在论文中的角色
- 首次引入位置：§4.12 DSP 块跨列复用映射方案（paper.md L519）
- 引用位置：paper.md L521 / L523-524
- 论证作用：可视化"将相邻两列 INT8 乘法合并到单个 DSP48E1 块"的位打包方案。读者据此理解 25 bit×18 bit DSP 乘法器如何承载两路独立 INT8 乘积，9 bit 进位隔离位如何避免串扰，以及为何整阵列从 256 LUT 乘法器折合为 128 DSP 块。

## 图类型
位级数据通路示意图（含位段拼接与解包逻辑）。

## 设计要素

### 必含元素
1. **左侧输入端**：
   - 权重 $w_0$（INT8）方块，标 "w0[7:0]"
   - 权重 $w_1$（INT8）方块，标 "w1[7:0]"
   - 激活值 $\mathit{act}$（INT8）方块，标 "act[7:0]"
2. **A 端 25 bit 打包**：长条形矩形分三段
   - 低 8 bit：填 $w_0$（淡蓝色）
   - 中 9 bit：填 "9'b0 进位隔离位"（淡灰色，标注"isolation"）
   - 高 8 bit：填 $w_1$（淡蓝色）
   - 整体标 "A = {w1, 9'b0, w0}" 25 bit
3. **B 端 18 bit 符号扩展**：方块标 "B = sign\_ext(act)" 18 bit，下方注 "B[17:0]"
4. **DSP48E1 核心**：中央大方块（淡黄色）标 "DSP48E1 25×18 multiplier"，输出 43 bit P
5. **P 端 43 bit 输出**：长条形矩形分两个数据段
   - 低段：$P[15:0]$ + 符号补偿 → $w_0 \times \mathit{act}$（深蓝色）
   - 高段：$P[33:17]$ 等 + 符号补偿 → $w_1 \times \mathit{act}$（深绿色）
6. **符号校正逻辑块**：两个小方块"sign correct"接在 P 段之后，输出最终 INT16 psum
7. **阵列层级缩略图**（右侧）：16×16 PE 阵列示意，每行 8 个 DSP 块（每 DSP 服务相邻两列），整阵列共 8×16 = 128 个 DSP

### 标注要求
- 关键比特位置：`A[7:0]`、`A[16:8]`、`A[24:17]`、`P[15:0]`、`P[33:17]`
- 关键常量："9'b0 isolation"、"sign-ext 18 bit"
- 整阵列规模：单核 128 DSP / $N=4$ 配置 512 DSP
- 数据等价标注："位级行为与 LUT 实现完全一致"

### 视觉层次
- 主角：中央 DSP48E1 块 + A 端 25 bit 打包条
- 配角：左侧三个 INT8 输入 + 右侧两路 INT16 输出
- 背景：右侧阵列缩略图（淡灰色作上下文）

## 数据来源
- paper.md §4.12 DSP 块跨列复用映射方案（L519-528）
- AMD UG479 _7Series_DSP48E1.pdf（DSP48E1 25×18 multiplier 规范）
- contributions.md C5（DSP 跨列复用）

## ASCII 示意稿

```
   Inputs (INT8)              25-bit A packing                     18-bit B
   ┌────────┐
   │ w0[7:0]│──┐    ┌────────────┬────────────┬────────────┐    ┌──────────────┐
   └────────┘  └───▶│ w1[24:17]  │ 9'b0[16:8] │ w0[7:0]    │    │ sign_ext(act)│
   ┌────────┐  ┌───▶│  INT8      │ isolation  │  INT8      │    │   INT18      │
   │ w1[7:0]│──┘    └────────────┴────────────┴────────────┘    └──────┬───────┘
   └────────┘                              │                           │
   ┌────────┐                              │                           │
   │act[7:0]│──────────────────────────────┼───────────────────────────┘
   └────────┘                              ▼                           ▼
                                  ┌─────────────────────────────────────┐
                                  │      DSP48E1  25×18 multiplier      │
                                  │              P = A × B              │
                                  └────────────────┬────────────────────┘
                                                   │ 43-bit P[42:0]
                            ┌──────────────────────┴──────────────────────┐
                            ▼                                             ▼
                  ┌──────────────────┐                          ┌──────────────────┐
                  │ extract P[15:0]  │                          │ extract P[33:17] │
                  │ + sign correct   │                          │ + sign correct   │
                  └────────┬─────────┘                          └────────┬─────────┘
                           │ INT16                                       │ INT16
                           ▼                                             ▼
                    w0 × act  (col 0)                              w1 × act  (col 1)


   Array view: 16×16 PE = 8 DSP-pairs × 16 rows
       row 0  : DSP DSP DSP DSP DSP DSP DSP DSP   (8 blocks, each serves 2 columns)
       row 1  : DSP DSP DSP DSP DSP DSP DSP DSP
        ...
       row 15 : DSP DSP DSP DSP DSP DSP DSP DSP
   Single-core total: 8 × 16 = 128 DSP48E1
   N=4 config:        4 × 128 = 512 DSP48E1
```

## 与正文的一致性检查
- [x] §4.12 "把相邻两列的 INT8 乘法器合并映射到一个 DSP48E1" — 图中两 $w$ 一 $\mathit{act}$ 共享 DSP
- [x] §4.12 "$A = \{w_1, 9'\mathrm{b}0, w_0\}$ 25 bit" — A 端三段打包条
- [x] §4.12 "9 bit 进位隔离位保证两路乘积不跨段串扰" — 中段标注 isolation
- [x] §4.12 "整 $16\times16$ 阵列折合为 8×16 = 128 个 DSP48E1 块" — 右侧阵列缩略图
- [x] §4.12 "位级行为与 LUT 实现完全一致" — 标注等价说明

## 不确定项
- [TBD: 是否在图中显式画出"符号校正"组合逻辑的内部细节] — 倾向不画，保持图整体简洁；详细公式留正文 §4.12 段
- [TBD: 阵列缩略图是否需要标注哪两列共享 DSP] — 倾向标第 0/1 列、第 2/3 列... 一对，避免歧义

## image 生成提示词

### 中文版

科研论文配图，**DSP48E1 块跨列复用映射方案原理图**，IEEE 期刊配色风格，整体白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1.4:1（横向略宽），适合 A4 单栏插图。**

**版式（自左向右三大区，从上到下分主视图与阵列缩略图两层）**：

- **左上区（输入端）**：纵向排列 3 个 INT8 输入方块，从上到下分别标 "w0[7:0]"、"w1[7:0]"、"act[7:0]"（淡灰色 CMYK 0/0/0/10 填充，黑色实线边框，Times New Roman 9 pt）。

- **中上区（A 端 25 bit 打包条）**：一条横向长矩形分三段，从右到左：低 8 bit 段（淡蓝色 CMYK 30/10/0/0）标 "w0[7:0]"；中 9 bit 段（淡灰色 CMYK 0/0/0/15）标 "9'b0 isolation"；高 8 bit 段（淡蓝色 CMYK 30/10/0/0）标 "w1[24:17]"。整条上方标 "A = {w1, 9'b0, w0}  (25 bit)"。w0 / w1 输入用细黑线箭头连入对应段。

- **中下区（B 端 18 bit）**：A 端条下方画一个稍短的方块，淡黄色 CMYK 0/10/30/0 填充，标 "B = sign\_ext(act)  (18 bit)"。act 输入用细黑线箭头连入。

- **中央区（DSP48E1 块）**：A 端 / B 端条右侧画一个大圆角矩形（淡黄色 CMYK 0/10/30/0 填充、深灰色边框），内写两行 "DSP48E1" / "25 × 18 multiplier"，下方注 "P = A × B  (43 bit)"。两条粗黑色箭头从 A 端、B 端汇入。

- **右上区（P 端 43 bit 解包）**：DSP 块右侧画一条横向长矩形分两段：低段 "P[15:0] → w0×act" 用深蓝色 CMYK 70/30/0/0 填充；高段 "P[33:17] → w1×act" 用深绿色 CMYK 60/0/40/20 填充。两段下方各接一个 "sign correct" 小方块（淡灰色），输出箭头分别标 "INT16 (col 0)" 和 "INT16 (col 1)"。

- **底部阵列缩略图区**：横向占整图下半部，画 16×16 网格，每行用 8 个小淡蓝色方块（每个标 "DSP"）表示相邻两列共享一个 DSP；网格左侧标 "row 0"、"row 1"、"row 15"。下方居中两行注释 "Single-core total: 8 × 16 = 128 DSP48E1" 和 "N=4 config: 512 DSP48E1"，字号 8 pt。

**字体**：所有英文 Times New Roman 9-10 pt，关键标题加粗；中文（如有）思源黑体 9-10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **cross-column DSP-block mapping scheme**, IEEE-journal style, overall white background with black text and lines, no decorative colors or cartoon elements. **Overall figure aspect ratio approximately 1.4:1 (slightly wider than tall), to fit a single-column A4 figure.**

**Layout (three regions left-to-right, two tiers top-to-bottom)**:

- **Top-left (input)**: Vertically stack three INT8 input boxes labeled "w0[7:0]", "w1[7:0]", "act[7:0]" (light-gray CMYK 0/0/0/10 fill, solid black borders, Times New Roman 9 pt).

- **Top-center (25-bit A packing)**: Horizontal long rectangle split into three segments, right-to-left: low 8-bit "w0[7:0]" (light-blue CMYK 30/10/0/0); middle 9-bit "9'b0 isolation" (light-gray CMYK 0/0/0/15); high 8-bit "w1[24:17]" (light-blue). Above the bar label "A = {w1, 9'b0, w0}  (25 bit)". Connect w0 / w1 inputs to corresponding segments with thin black arrows.

- **Center-bottom (18-bit B)**: Below the A bar, draw a shorter box (light-yellow CMYK 0/10/30/0 fill), labeled "B = sign_ext(act)  (18 bit)". Connect act input with an arrow.

- **Center (DSP48E1 block)**: Right of A / B bars, draw a large rounded rectangle (light-yellow fill, dark-gray border) with two lines "DSP48E1" / "25 × 18 multiplier", below "P = A × B  (43 bit)". Two thick black arrows from A and B converge into it.

- **Top-right (43-bit P unpack)**: Right of DSP block, horizontal long rectangle split into two segments: low "P[15:0] → w0×act" (dark-blue CMYK 70/30/0/0 fill); high "P[33:17] → w1×act" (dark-green CMYK 60/0/40/20 fill). Each segment connects downward to a small "sign correct" box (light-gray), with output arrows labeled "INT16 (col 0)" and "INT16 (col 1)".

- **Bottom (array view)**: Horizontal occupying lower half. Draw 16×16 grid where each row uses 8 small light-blue boxes labeled "DSP" (each serving 2 columns). Label rows "row 0", "row 1", "row 15" on the left. Below the grid, center two annotations: "Single-core total: 8 × 16 = 128 DSP48E1" and "N=4 config: 512 DSP48E1", 8 pt.

**Typography**: All English in Times New Roman 9-10 pt with bold titles; any Chinese in Source Han Sans 9-10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
