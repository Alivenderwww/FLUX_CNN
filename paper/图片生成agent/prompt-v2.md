# 图像生成提示词训练 — v2

基于 `compare-v1.md` 中 6 项 P0/P1 偏差的改进版。

## 核心改进点（vs v1）

| 改进 | v1 问题 | v2 对策 |
|---|---|---|
| 宽高比 | 生成 ~1.2:1 接近正方形 | 用最强约束放在 prompt 最前面 |
| C 信号路径 | C 直接连到 17-Bit Shift，缺 "C" 方块 | 详述 C 路径含 "C" 方块 |
| 顶部 A:B 信号 | 完全缺失 | 显式列入顶部信号清单 |
| 左上角 ACOUT/BCOUT 出框 | 水平伸出（错） | 强调 45° 转角向上 |
| 位宽数字 | "/00" "/45" "/JS" 乱码 | 列出每个数字的精确值 + 字号要求 |
| 底部控制信号位置 | 在框外（错） | 强调"框内底部" |

## v2 提示词（中文版）

```
**严格宽高比约束：横向 2.3:1（宽是高的 2.3 倍），不是正方形，绝对不是 1:1 或 4:3。**

科研技术框图，AMD / Xilinx 用户指南（UG479）白皮书风格，**Xilinx DSP48E1 Slice 内部数据通路框图**。纯白背景；线条与英文文字使用深靛紫色 Hex #3B2D7E（不是黑色、不是蓝色、不是海军蓝）；位宽数字标注使用鲜红色 Hex #C8102E（必须清晰可辨，不能与背景或紫色混淆）；底部脚注使用黑色。Arial 无衬线字体，全英文。

整张图整体布局**横向细长**，宽是高的 2.3 倍。

**外框（最外层）**：深靛紫色虚线大矩形包围全图（DSP48E1 Slice 边界）。框外标签是对外端口，框内是内部数据通路。

**顶部边界（从左到右依次）**：
- 框左上角斜向上弯出 "BCOUT*"（带 45° 转角向上，红色 "/18"）与 "ACOUT*"（红色 "/30"）—— **必须是 45° 弯角向上，不是水平向左**
- 顶部偏左：水平信号 "A:B" 从框中央上方伸出（红色 "/48"）—— **不能省略此信号**
- 顶部中央偏右：信号 "ALUMODE"（红色 "/4"）从框上方进入，经一个小方块到 ALU
- 顶部右侧：从左到右分别是 "MULTSIGNOUT*"、"CARRYCASCOUT*"、"PCOUT*"（红色 "/48"）三个出口

**左侧（输入寄存器组）**：
- 上方：矩形框 "Dual B Register"，左侧输入信号 "B"（红色 "/30"），从输入到框中标 "/18"
- 下方：更大矩形框，**3 行文字**："Dual A, D," / "and Pre-adder"，左侧输入 "A"（红色 "/30"）、"D"（红色 "/25"），左下输入 "C"（红色 "/48"）

**中央（计算路径，从左到右）**：
- 椭圆形 "MULT 25 X 18" 乘法器
- 紧邻右侧：小方块 "M"（M 寄存器）
- 三个梯形多路选择器，垂直排列从上到下："X"、"Y"、"Z"；每个左侧标 "0"（X）或 "0/1"（Y）或 "0"（Z）
- 两个细长矩形 "17-Bit Shift"：一个连接 M → Y、一个连接 C → Z
- **C 信号的完整路径**：C（红 /48）从框左侧底部进入 → 经过一个独立的小方块 "C" → 经 "17-Bit Shift" 矩形 → 进入 Z 多路选择器 —— **"C" 方块必须存在，不能省略**
- 顶部 ALUMODE 输入经一个小方块进入下方中央 ALU
- 中央 ALU：D 形符号（左半圆右半平），内部上方画 "+" 下方画 "−"，接收 X / Y / Z 三路输出

**右侧（输出寄存器与 pattern detector，从上到下垂直排列）**：
- 5 个小方块 "P"（P 寄存器）垂直堆叠，每个 P 对齐一条输出：
  - 第 1 个 P → "CARRYCASCOUT*" 与 "MULTSIGNOUT*"（顶部出口）
  - 第 2 个 P → "CARRYOUT"（红 /4）
  - 第 3 个 P → "P"（红 /48，主输出）
  - 第 4 个 P → "PATTERNDETECT"
  - 第 5 个 P → "PATTERNBDETECT"
- 第 4 个 P 旁边一个小菱形 "=" 符号（pattern detector）
- 文字标注 "CREG/C Bypass/Mask"（在第 4 个 P 与第 5 个 P 之间）

**底部（控制输入，全部在框内底部，不在框外）**：
- 左下：从左到右水平进入框内的 4 行信号
  - "INMODE"（红 "/5"）经一个小方块
  - "CARRYIN"（无位宽标注）
  - "OPMODE"（红 "/7"）
  - "CARRYINSEL"（无位宽标注）
- 右下框内：信号 "MULTSIGNIN*"、"CARRYCASCIN*"
- 框外左下角：信号 "BCIN*"、"ACIN*"
- 框外右下角：信号 "PCIN*"（红 "/48"）

**位宽标注规则**：
- 全部位宽用红色斜线 "/N" 标注，**字号足够大可清晰辨认**，不能产生 "/00" "/JS" 等乱码
- 出现的位宽数字精确清单：**/48**（C、A:B、PCOUT、PCIN、P）、**/30**（A、B、ACOUT）、**/25**（D、MULT 输入）、**/18**（B 寄存器输出、BCOUT、MULT 输入）、**/7**（OPMODE）、**/5**（INMODE）、**/4**（ALUMODE、CARRYOUT）、**/3**、**/1**

**底部脚注**：图框下方居中一行黑色 Arial 7 pt 斜体小字 "*These signals are dedicated routing paths internal to the DSP48E1 column. They are not accessible via fabric routing resources."

**字体**：所有文字 Arial 9-10 pt（模块标签加粗，信号名常规，脚注 7 pt 斜体）。

**严禁**：彩色渐变、阴影、3D 立体、图标、卡通元素、绿色 / 黄色 / 橙色装饰色、模块内部填充色（仅有边框、内部纯白）、水印、中文字符、手写体。

**严禁宽高比偏离**：生成图必须是 2.3:1 横向细长，**绝不能是正方形或近似正方形**。
```

## v2 提示词（英文版）

```
**STRICT aspect ratio constraint: landscape 2.3:1 (width is 2.3× the height), NOT square, absolutely NOT 1:1 or 4:3.**

Technical block diagram, AMD / Xilinx user guide (UG479) whitepaper style, **Xilinx DSP48E1 Slice internal datapath block diagram**. Pure white background; line art and English text in dark indigo-purple Hex #3B2D7E (NOT black, NOT pure blue, NOT navy); bit-width annotations in bright red Hex #C8102E (must be clearly legible, never confused with background or purple); bottom footnote in black. Arial sans-serif font, all English.

Overall layout is **landscape, elongated horizontally**, width is 2.3× the height.

**OUTER FRAME**: A large rectangular dashed dark indigo-purple boundary encloses the entire figure (DSP48E1 slice boundary). Labels outside the frame are external ports; inside is the internal datapath.

**TOP EDGE (from left to right)**:
- Top-left corner: "BCOUT*" (red "/18") and "ACOUT*" (red "/30") exit through **45-degree corner turns upward**, NOT horizontally to the left
- Top-left center: a horizontal signal "A:B" extends upward from the frame top (red "/48") — **this signal must NOT be omitted**
- Top center-right: signal "ALUMODE" (red "/4") enters from above through a small square box into the central ALU
- Top-right: from left to right, three exits: "MULTSIGNOUT*", "CARRYCASCOUT*", "PCOUT*" (red "/48")

**LEFT REGION (input registers)**:
- Top: rectangular box "Dual B Register", input signal "B" (red "/30") entering from the left, output to next stage labeled "/18"
- Below: a larger rectangular box with **3 lines of text**: "Dual A, D," / "and Pre-adder", inputs "A" (red "/30"), "D" (red "/25") entering from the left, and "C" (red "/48") entering from the bottom-left

**CENTER REGION (datapath, left to right)**:
- An oval "MULT 25 X 18" multiplier
- Immediately to its right: a small square "M" (multiplier register)
- Three vertical trapezoidal multiplexers stacked top to bottom: "X", "Y", "Z"; each has selectors on the left labeled "0" (X), "0/1" (Y), "0" (Z)
- Two narrow rectangles "17-Bit Shift": one connecting M to Y, one connecting C to Z
- **C signal complete path**: C (red /48) enters from the bottom-left of the frame → passes through an **independent small square box labeled "C"** → through a "17-Bit Shift" rectangle → enters the Z multiplexer — the **"C" box must exist and not be omitted**
- Top ALUMODE input enters through a small square box into the central ALU below
- Central ALU: D-shaped symbol (left half-circle, right half-flat), with "+" drawn in the upper interior and "−" in the lower interior, receiving X / Y / Z multiplexer outputs

**RIGHT REGION (output registers and pattern detector, vertical stack top to bottom)**:
- 5 small "P" boxes (P-registers) stacked vertically; each P aligns with one specific output:
  - 1st P → "CARRYCASCOUT*" and "MULTSIGNOUT*" (top exits)
  - 2nd P → "CARRYOUT" (red "/4")
  - 3rd P → "P" (red "/48", main output)
  - 4th P → "PATTERNDETECT"
  - 5th P → "PATTERNBDETECT"
- A small diamond "=" symbol (pattern detector) next to the 4th P
- Text label "CREG/C Bypass/Mask" between the 4th P and 5th P

**BOTTOM REGION (control inputs, ALL INSIDE the frame at the bottom, NOT outside the frame)**:
- Bottom-left interior: 4 horizontal signal lines entering the frame from left to right:
  - "INMODE" (red "/5") through a small square box
  - "CARRYIN" (no bit-width label)
  - "OPMODE" (red "/7")
  - "CARRYINSEL" (no bit-width label)
- Bottom-right interior: signals "MULTSIGNIN*", "CARRYCASCIN*"
- Outside the frame, bottom-left corner: "BCIN*", "ACIN*"
- Outside the frame, bottom-right corner: "PCIN*" (red "/48")

**BIT-WIDTH ANNOTATION RULES**:
- All bit-widths use red diagonal slash "/N" notation, **font large enough to be unambiguously readable**, must NOT produce garbled text like "/00", "/JS"
- Exact bit-width values appearing in the diagram: **/48** (C, A:B, PCOUT, PCIN, P), **/30** (A, B, ACOUT), **/25** (D, MULT input), **/18** (B-register output, BCOUT, MULT input), **/7** (OPMODE), **/5** (INMODE), **/4** (ALUMODE, CARRYOUT), **/3**, **/1**

**BOTTOM FOOTNOTE**: Below the frame, centered, one line of small black Arial 7 pt italic text: "*These signals are dedicated routing paths internal to the DSP48E1 column. They are not accessible via fabric routing resources."

**TYPOGRAPHY**: All text Arial 9-10 pt (module labels bold, signal names regular weight, footnote 7 pt italic).

**STRICTLY FORBIDDEN**: color gradients, drop shadows, 3D effects, icons, cartoon elements, green / yellow / orange decorative colors, fill colors inside boxes (boxes have outline only, interior pure white), watermarks, Chinese characters, handwriting fonts.

**STRICTLY FORBIDDEN ASPECT RATIO**: the generated image must be 2.3:1 landscape elongated, **must NOT be square or near-square**.
```

## v2 对照清单（在 v1 基础上加重点项）

| # | 检查点 | 重要度 | v1 评分 | v2 期望 |
|---|---|---|---|---|
| 1 | **宽高比 2.3:1 横向细长** | **极高** | ❌ | ✓ |
| 2 | **C 信号路径含独立 "C" 方块** | **极高** | ❌ | ✓ |
| 3 | **顶部 A:B 信号存在** | **极高** | ❌ | ✓ |
| 4 | **左上角 ACOUT*/BCOUT* 45° 弯角向上** | 高 | ❌ | ✓ |
| 5 | **位宽数字精确（/48, /30, /25, /18, /7, /5, /4 不乱码）** | 高 | ⚠ | ✓ |
| 6 | **底部 INMODE/CARRYIN/OPMODE/CARRYINSEL 在框内底部** | 高 | ❌ | ✓ |
| 7 | 5 个 P 寄存器对齐 5 路输出 | 中 | ⚠ | ✓ |
| 8 | "Dual A, D, and Pre-adder" 3 行文字 | 中 | ⚠ | ✓ |
| 9 | 配色深靛紫 #3B2D7E + 红 #C8102E | 中 | ✓（已对） | 保持 |
| 10 | 其余 v1 已 PASS 项（外框、MULT 椭圆、X/Y/Z、ALU 等） | 高 | ✓ | 保持 |

## 下一步

请把上面 v2 英文版提示词喂给 GPT Image 2（或同模型），把生成结果命名为 `gen-v2-gptimage2.png` 放到本目录，告诉我即可。我会按 v2 对照清单做第二轮 diff，决定是否需要 v3 或封版。
