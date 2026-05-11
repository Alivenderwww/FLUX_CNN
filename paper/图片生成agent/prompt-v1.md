# 图像生成提示词训练 — v1

## 原始图片

`PixPin_2026-05-10_11-56-07.png` — Xilinx UG479 DSP48E1 Slice 内部数据通路框图

## 我对原图的特征观察

### 整体风格
- **来源**：AMD / Xilinx 用户指南（UG479 风格）框图，技术白皮书插图标准
- **画布**：横向，宽高比约 2.3:1，单栏宽度
- **底色**：纯白
- **主色**：深紫色 / 深靛蓝（约 CMYK 80/70/0/50 或 Hex #3B2D7E 风格）的线条与英文文字
- **强调色**：红色（仅用于位宽数字 `/48`、`/30`、`/18`、`/25`、`/4`、`/5`、`/7`、`/3`）
- **次色**：黑色（仅用于底部脚注一行）
- **字体**：Arial / Helvetica 无衬线，全英文，无中文
- **风格**：严谨、技术性、无装饰、无渐变、无阴影、无 3D 效果、无图标、无背景填充

### 版式（从外到内、从左到右）

**外框**：深紫色虚线大矩形包围整张图，代表 DSP48E1 Slice 边界。框外标注的信号是单元的对外端口；框内是内部数据通路。

**左侧（输入寄存器组）**：
- 上方矩形 `Dual B Register`，输入信号 `B`（30 bit，红色 `/30` 标注）从左侧进入
- 下方更大的矩形 `Dual A, D, and Pre-adder`（3 行文字），输入信号 `A`（30 bit）、`D`（25 bit）从左进入
- 输出 `ACOUT*`、`BCOUT*` 从框左上角伸出

**中央（数据通路）**：
- 椭圆形/圆角矩形 `MULT 25 X 18` — 25×18 乘法器
- 乘法器输出 → 小方块 `M`（乘法器结果寄存器）
- 三个垂直多路选择器（梯形/三角形）`X`、`Y`、`Z` 从上至下排列；每个左侧有 `0` 或 `0/1` 输入
- 两个小矩形 `17-Bit Shift`：一个连接 M 到 Y、一个连接 C 输入到 Z
- 顶部中央 `ALUMODE`（4 bit，红色 `/4`）输入，经一个小寄存器进入中央 ALU
- 中央 ALU：圆形/D 形符号包含 `+` 与 `-`，接收 X / Y / Z 三路选择器输出

**右侧（输出寄存器与 pattern detector）**：
- 一列小方块 `P`（P 寄存器）垂直堆叠 4-5 个
- 右下方一个小菱形 `=` 符号（pattern detector）
- 标注 `CREG/C Bypass/Mask`
- 框右侧伸出输出：`CARRYCASCOUT*`、`MULTSIGNOUT*`、`PCOUT*`（48 bit）、`CARRYOUT`（4 bit）、`P`（48 bit）、`PATTERNDETECT`、`PATTERNBDETECT`

**底部（控制输入）**：
- `INMODE`（5 bit）从左下进入，经一个小寄存器
- `CARRYIN` 信号线
- `OPMODE`（7 bit）信号线
- `CARRYINSEL` 信号线
- 右下：`MULTSIGNIN*`、`CARRYCASCIN*`、`PCIN*`（48 bit）从底部进入
- 左下：`BCIN*`、`ACIN*` 从底部进入

**位宽标注规范**：
- 用红色斜线标注 `/N` 的形式叠加在信号线上
- 出现的位宽：`/48`（C、P、PCIN、PCOUT）、`/30`（A、B）、`/25`（D、MULT 输入）、`/18`（B 寄存器输出、MULT 输入）、`/7`（OPMODE）、`/5`（INMODE）、`/4`（ALUMODE、CARRYOUT）、`/3`、`/1`

**底部脚注**：
- 黑色小字 Arial 7 pt 斜体一行：`*These signals are dedicated routing paths internal to the DSP48E1 column. They are not accessible via fabric routing resources.`

## v1 提示词（中文版）

```
科研技术框图，AMD / Xilinx 用户指南（UG479）白皮书风格，**Xilinx DSP48E1 Slice 内部数据通路框图**。纯白背景，深靛紫色线条与英文文字（约 Hex #3B2D7E），红色位宽数字标注（约 Hex #C8102E），黑色脚注。Arial 无衬线字体，全英文。整体宽高比约 2.3:1，横向插图。

**外框**：深靛紫色虚线大矩形包围整张图（代表 DSP48E1 边界），框外标注的信号是对外端口，框内是内部数据通路。

**左侧（输入寄存器组）**：
- 上方矩形框 "Dual B Register"，左侧输入信号 "B" 带红色斜线标注 "/30"
- 下方更大矩形框 "Dual A, D, and Pre-adder"（3 行文字），左侧输入 "A"（红色 /30）、"D"（红色 /25）、"C"（红色 /48）
- 框左上角伸出对外信号 "BCOUT*"（红 /18）、"ACOUT*"（红 /30）

**中央（计算路径）**：
- 椭圆形 "MULT 25 X 18" 乘法器
- 乘法器右侧紧邻小方块 "M"
- 三个梯形多路选择器，从上到下分别标 "X"、"Y"、"Z"，每个左侧标 "0" 或 "0/1" 表示选项
- 两个细长矩形 "17-Bit Shift"：一个在 M 到 Y 之间、一个在 C 到 Z 之间
- 顶部中央输入 "ALUMODE"（红 /4），经小方块进入中央 ALU
- 中央 ALU 用圆形/D 形符号画，内含 "+" 与 "−" 两个符号

**右侧（输出寄存器与 pattern detector）**：
- 4-5 个垂直堆叠的小方块 "P"（P 寄存器）
- 右下方一个小菱形 "=" 符号（pattern detector）
- 文字标注 "CREG/C Bypass/Mask"
- 框右侧伸出对外输出：从上至下 "CARRYCASCOUT*"、"MULTSIGNOUT*"、"PCOUT*"（红 /48）、"CARRYOUT"（红 /4）、"P"（红 /48）、"PATTERNDETECT"、"PATTERNBDETECT"

**底部（控制输入）**：
- 左下：信号 "INMODE"（红 /5）经小方块、"CARRYIN"、"OPMODE"（红 /7）、"CARRYINSEL"
- 右下：信号 "MULTSIGNIN*"、"CARRYCASCIN*"、"PCIN*"（红 /48）
- 左下角再下方：信号 "BCIN*"、"ACIN*"

**位宽标注**：所有位宽用红色斜线表达式 "/N" 标注，叠加在信号线上方或旁边。出现的位宽包括 /48、/30、/25、/18、/7、/5、/4、/3、/1。

**底部脚注**：图框下方居中一行黑色 Arial 7 pt 斜体小字 "*These signals are dedicated routing paths internal to the DSP48E1 column. They are not accessible via fabric routing resources."

**字体**：所有文字 Arial 9-10 pt，模块标签加粗，信号名常规，脚注 7 pt 斜体。

**严禁**：彩色渐变、阴影、3D 立体、图标、卡通元素、绿色/黄色/橙色装饰色、模块内部填充色（仅有边框、白色内部）、水印、中文字符、手写体。
```

## v1 提示词（英文版）

```
Technical block diagram, AMD / Xilinx user guide (UG479) whitepaper style, **Xilinx DSP48E1 Slice internal datapath block diagram**. Pure white background, dark indigo-purple line art and English text (approx Hex #3B2D7E), red bit-width annotations (approx Hex #C8102E), black footnote. Arial sans-serif font throughout, all English. Overall aspect ratio approximately 2.3:1, landscape orientation.

OUTER FRAME: A large rectangular dashed dark-indigo boundary encloses the entire DSP48E1 slice. Signals labeled outside the frame are external slice ports; signals inside are internal datapath.

LEFT REGION (input registers):
- Top: rectangular box "Dual B Register" with input signal "B" entering from the left, labeled "/30" in red diagonal slash notation
- Below it: a larger rectangular box "Dual A, D, / and Pre-adder" (3 lines of text), with inputs "A" (red /30), "D" (red /25), "C" (red /48) entering from the left
- Top-left corner of the frame outputs: "BCOUT*" (red /18) and "ACOUT*" (red /30)

CENTER REGION (datapath):
- An oval / rounded-rectangle "MULT 25 X 18" multiplier
- Immediately to the right: a small square "M" (multiplier register)
- Three vertical trapezoidal multiplexers labeled top-to-bottom: "X", "Y", "Z"; each has "0" or "0/1" inputs on the left side
- Two narrow rectangles "17-Bit Shift": one connecting M to Y, one connecting C to Z
- Top center: input "ALUMODE" (red /4) entering through a small square into the central ALU
- Central ALU symbol: a circular / D-shaped figure containing "+" and "−" signs, receiving X / Y / Z multiplexer outputs

RIGHT REGION (output registers and pattern detector):
- A vertical column of 4-5 small "P" boxes (P-registers) on the right
- Bottom-right: a small diamond "=" symbol (pattern detector)
- Label "CREG/C Bypass/Mask"
- Right-frame outputs from top to bottom: "CARRYCASCOUT*", "MULTSIGNOUT*", "PCOUT*" (red /48), "CARRYOUT" (red /4), "P" (red /48), "PATTERNDETECT", "PATTERNBDETECT"

BOTTOM REGION (control inputs):
- Bottom-left: signals "INMODE" (red /5) through a small square, "CARRYIN", "OPMODE" (red /7), "CARRYINSEL"
- Bottom-right: signals "MULTSIGNIN*", "CARRYCASCIN*", "PCIN*" (red /48)
- Below the bottom-left region: signals "BCIN*", "ACIN*"

BIT-WIDTH ANNOTATION STYLE: All bit-widths use red diagonal slash notation "/N" overlaid on signal lines. Bit-widths appearing in the diagram: /48, /30, /25, /18, /7, /5, /4, /3, /1.

BOTTOM FOOTNOTE: Below the frame, centered, one line of small black Arial 7 pt italic text: "*These signals are dedicated routing paths internal to the DSP48E1 column. They are not accessible via fabric routing resources."

TYPOGRAPHY: All text Arial 9-10 pt, module labels bold, signal names regular weight, footnote 7 pt italic.

STRICTLY FORBIDDEN: color gradients, drop shadows, 3D effects, icons, cartoon elements, green / yellow / orange decorative colors, fill colors inside boxes (boxes have outline only, white interior), watermarks, Chinese characters, handwriting fonts.
```

## v1→生成图片对照清单（用户生成图后我会按此清单评估）

| 类别 | 检查点 | 重要度 |
|---|---|---|
| 整体 | 宽高比约 2.3:1 横向 | 高 |
| 整体 | 纯白底 | 高 |
| 整体 | 深靛紫色线条与文字（非黑色，非蓝色） | 高 |
| 整体 | 红色位宽标注 `/N` | 高 |
| 字体 | 全英文 Arial / Helvetica 无衬线 | 高 |
| 外框 | 大矩形虚线包围 | 高 |
| 左侧 | "Dual B Register" 矩形 + B 输入 | 高 |
| 左侧 | "Dual A, D, and Pre-adder" 矩形（3 行文字） + A/D/C 输入 | 高 |
| 左上角 | ACOUT* / BCOUT* 输出 | 中 |
| 中央 | 椭圆 "MULT 25 X 18" + M 方块 | 高 |
| 中央 | 三个 X / Y / Z 多路选择器（梯形） | 高 |
| 中央 | 两个 "17-Bit Shift" 矩形 | 中 |
| 中央 | 顶部 ALUMODE 输入 | 中 |
| 中央 | 圆形/D 形 ALU 含 +/− 符号 | 高 |
| 右侧 | 4-5 个垂直堆叠 P 方块 | 高 |
| 右下 | 菱形 = 符号 + CREG/C Bypass/Mask 标注 | 中 |
| 右侧 | 输出端口 6-7 个（CARRYCASCOUT*/PCOUT*/P/PATTERNDETECT 等） | 高 |
| 底部 | INMODE/CARRYIN/OPMODE/CARRYINSEL 控制输入 | 中 |
| 底部 | BCIN*/ACIN*/PCIN* 等带 * 信号 | 中 |
| 底部 | 黑色小字脚注一行 | 中 |
| 风格 | 无渐变、无阴影、无 3D、无图标 | 高 |
| 风格 | 模块内白色填充无背景色 | 高 |

## 下一步

请你把上面的"v1 提示词（英文版）"或"v1 提示词（中文版）"喂给图像生成模型（DALL-E / Stable Diffusion / Midjourney 等），把结果图回传给我；我会按对照清单逐项评估生成图与原图的差异，列出 v2 提示词的改进点。
