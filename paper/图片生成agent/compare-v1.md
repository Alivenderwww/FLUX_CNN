# v1 生成图对比与 v2 改进点

## v1 模型
- 模型：GPT Image 2
- 文件：`gen-v1-gptiamge2.png`
- 提示词：`prompt-v1.md` 英文版

## 总体保真度判定

**总体接近度：~70%**。v1 已经把握住主色调、主要模块拓扑、字体风格，但有 6 项关键偏差影响精确度。

## 逐项对照评估（按 prompt-v1.md 末尾 22 项清单）

| # | 检查点 | 重要度 | 原图 | v1 生成图 | 评分 |
|---|---|---|---|---|---|
| 1 | 宽高比约 2.3:1 横向 | 高 | ~2.3:1 横向细长 | **~1.2:1 接近正方形** | ❌ 严重偏差 |
| 2 | 纯白底 | 高 | 白 | 白 | ✓ |
| 3 | 深靛紫色线条与文字 | 高 | 深靛紫 #3B2D7E | 深靛紫（基本对，色相略偏蓝） | ✓ |
| 4 | 红色位宽 /N 标注 | 高 | 红色 /N 清晰 | **红色但部分字符识别错（"/00" 应是 /18 或 /30；"/45" 应是 /48；"/JS" 乱码）** | ⚠ 部分对 |
| 5 | 全英文 Arial / Helvetica | 高 | Arial | Arial 类无衬线 | ✓ |
| 6 | 大矩形虚线外框 | 高 | 深紫虚线 | 有虚线但深度不够 | ✓ |
| 7 | "Dual B Register" 矩形 + B 输入 | 高 | 矩形 + B + /30 | 有矩形 + B + /30 但矩形偏窄 | ⚠ |
| 8 | "Dual A, D, and Pre-adder" 矩形（3 行）+ A/D/C 输入 | 高 | 3 行文字 + A/D 输入 | "Dual A, D, / and Pre-adder" 2 行 + A/D 输入 | ⚠ 文字断行不同 |
| 9 | 左上角 ACOUT*/BCOUT* 输出弯出 | 中 | 框左上角向上弯出（45° 转角 + /18 + /30 标注） | **从框左侧水平伸出（无弯角）位置错** | ❌ |
| 10 | 椭圆 "MULT 25 X 18" + M 方块 | 高 | 椭圆 + M | 椭圆 + M | ✓ |
| 11 | 三个 X / Y / Z 多路选择器（梯形） | 高 | 三个梯形（左侧短、右侧长的等腰梯形） | 三个梯形（形状基本对） | ✓ |
| 12 | 两个 "17-Bit Shift" 矩形 | 中 | 一个连 M→Y、一个连 C→Z | 有两个 17-Bit Shift（位置基本对） | ✓ |
| 13 | 顶部 ALUMODE 输入 | 中 | 顶部中央 ALUMODE + /4 + 小方块 | 顶部 ALUMODE + /4 | ✓ |
| 14 | 圆形/D 形 ALU 含 +/− 符号 | 高 | D 形（左圆右平）+ "+" 上、"−" 下 | 圆/D 形 + "+" 与 "−" | ✓ |
| 15 | 4-5 个垂直堆叠 P 方块 | 高 | 5 个 P 方块（4 个垂直堆 + 1 个 PATTERNDETECT 行） | 5 个 P 但分布不对（部分 P 偏到 X/Y/Z 选择器之后单独成列） | ⚠ |
| 16 | 菱形 = 符号 + CREG/C Bypass/Mask 标注 | 中 | 菱形 = + CREG/C Bypass/Mask | 有 = + "CREG/C / Bypass/Mask" 文字 | ✓ |
| 17 | 右侧 6-7 个输出端口 | 高 | CARRYCASCOUT*/MULTSIGNOUT*/PCOUT*/CARRYOUT/P/PATTERNDETECT/PATTERNBDETECT | 7 个输出名都有 | ✓ |
| 18 | 底部 INMODE/CARRYIN/OPMODE/CARRYINSEL | 中 | 框内底部 4 行水平信号 | 4 行水平信号但**位置在框外下方**（应在框内底部） | ⚠ |
| 19 | BCIN*/ACIN*/PCIN* 等带 * 信号 | 中 | 框外左下角 BCIN*/ACIN*；框外右下 PCIN* | 框外左下 BCIN*/ACIN*、框外右下 PCIN* | ✓ |
| 20 | 黑色小字脚注一行 | 中 | 完整脚注 | "*These signals are dedicated routing paths internal to the DSP48E1 column." 脚注存在 | ✓ |
| 21 | 无渐变/阴影/3D/图标 | 高 | 无 | 无 | ✓ |
| 22 | 模块内白色无填充 | 高 | 白色 | 白色 | ✓ |

## 关键偏差详述

### 偏差 A：宽高比严重失配（**最严重**）
- 原图：约 850×370 像素，宽高比 ~2.3:1（**横向细长**）
- 生成图：约 720×600 像素，宽高比 ~1.2:1（**接近正方形**）
- 后果：版面被压缩成方形后，左右两块内容堆得很紧，标签互相重叠，视觉密度过高

### 偏差 B：C 信号路径缺关键 "C" 方块
- 原图：信号 C（48 bit）从框左侧 D 输入下方水平进入，**经过一个独立的 "C" 方块**，再连到 17-Bit Shift，最后进入 Z 多路选择器
- 生成图：C 直接从左侧拉到 17-Bit Shift，**缺少 "C" 方块**

### 偏差 C：左上角 ACOUT*/BCOUT* 出框方向错
- 原图：ACOUT* 与 BCOUT* 从框左上角斜向上方伸出（**带 45° 转角**），位置在框顶部偏左，标注红色 /30 与 /18
- 生成图：ACOUT* 与 BCOUT* 从框**左侧水平伸出**（横向），位置不对

### 偏差 D：位宽数字部分识别错
- 原图：清晰的 /18, /25, /30, /48, /4, /5, /7, /3, /1 红色数字
- 生成图：出现 "/00"、"/45"、"/JS"、"/JS" 等乱码或错认数字（图像生成模型对小字号红字的渲染不稳定）

### 偏差 E：顶部 "A:B" 信号缺失
- 原图：框顶部水平有 "A:B" 信号线（48 bit，位于 ACOUT* 与 ALUMODE 之间）
- 生成图：**完全缺失 A:B 信号**

### 偏差 F：底部控制信号位置错位
- 原图：INMODE/CARRYIN/OPMODE/CARRYINSEL 在**框内底部**（4 行水平进入框内右侧）
- 生成图：4 行控制信号在**框外下方**（位置错）

## v2 改进策略

按改进优先级：

### P0（必改）
1. **宽高比强制约束**：在 v2 提示词最前面强调 "**STRICT aspect ratio 2.3:1, landscape, NOT square. Width approximately 2.3× the height.**"
2. **C 信号路径详述**：v2 加 "C input enters from the left side at the bottom area, passes through an independent small square box labeled 'C', then connects to a '17-Bit Shift' box, finally enters the Z multiplexer."
3. **顶部信号补全**：v2 加 "Top of frame contains horizontal signals from left to right: 'A:B' (red /48), 'ALUMODE' (red /4), 'MULTSIGNOUT*', 'CARRYCASCOUT*', 'PCOUT*' (red /48)."
4. **左上角 ACOUT/BCOUT 弯出**：v2 加 "Top-left corner: 'BCOUT*' (red /18) and 'ACOUT*' (red /30) exit through 45-degree corner turn upward and leftward, NOT horizontally."

### P1（重要改）
5. **位宽数字精确化**：v2 在每个标注处明确"red bold /18 not /00"等具体数字（用 explicit list）
6. **底部控制信号位置**：v2 加 "Control signals INMODE (red /5), CARRYIN, OPMODE (red /7), CARRYINSEL enter the frame from the BOTTOM-LEFT INSIDE the frame, positioned at the bottom interior of the slice, NOT below the frame."
7. **P 寄存器位置**：v2 加 "Right side: 5 small 'P' boxes vertically stacked, EACH P box aligned with one specific output: top P → CARRYCASCOUT/MULTSIGNOUT path, second P → CARRYOUT, third P → P (48-bit), fourth P → PATTERNDETECT/PATTERNBDETECT."

### P2（细化）
8. **"Dual A, D, and Pre-adder" 文字断行**：v2 强调 "3 lines of text: line 1 'Dual A, D,' / line 2 'and Pre-adder' (or similar 3-line layout)"
9. **配色精确**：v2 强调 "lines and text in dark indigo-purple Hex #3B2D7E (NOT black, NOT pure blue, NOT navy)"
10. **红色位宽精确**：v2 强调 "bit-width annotations in bright red Hex #C8102E or #E60012, large enough to be unambiguously readable, never confuse 1/0/8 with similar digits"

## 下一步

把 v2 提示词写入 `prompt-v2.md`，针对 P0 + P1 改进点强化约束。
