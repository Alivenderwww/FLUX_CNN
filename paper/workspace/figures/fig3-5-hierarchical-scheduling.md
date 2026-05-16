# 图 3.5 分层调度示意
# Figure 3.5 Hierarchical scheduling

## 在论文中的角色
- 首次引入：§3.4 调度方案（paper.md L316-L326）
- 引用位置：§3.4 第二段（核内）/ 第三段（核外）/ 第四段（多核）
- 论证作用：把 §3.4 文字描述的"核内 6 级嵌套 FSM"与"核外 Task Descriptor 链表"用单张组合图同框呈现，建立读者对"硬件展开静态嵌套，软件编排宏粒度任务"双层调度结构的直观印象。**该图为组合图：左半页面留白给算法 3.1 伪代码 PNG 拼合，右半画 FSM 嵌套树 + Task Descriptor 链表**。

## 图类型
组合图（左半留白嵌算法伪代码 PNG / 右半画 FSM 嵌套树 + 描述符链表）。

## 设计要素

### 必含元素
1. **左半页面（50% 宽度，留白）**：保留供 Word 编排时拼合算法 3.1 伪代码 PNG。Claude Design 出图时此区域绘制为浅灰矩形占位框 + 中央居中标注"[伪代码图算法 3.1 留白区]"，使 Claude Design 输出后用户可在 Word 中替换。
2. **右半页面（50% 宽度）—上中下三层结构**：
   - **顶层（核外宏指令流）**：水平画 Task Descriptor 链表，由 5-6 个矩形节点组成（IDMA_DESC / WDMA_DESC / CFG_WRITE / ODMA_DESC / IDMA_DESC ... 顺次排列），每节点之间用箭头连接组成链表；链表头部标"DDR 中描述符链表头基址"。
   - **中层（DFE 描述符获取引擎 + sequencer）**：一个圆角矩形标"DFE pull → sequencer dispatch"，从顶层链表拉一条向下箭头表示"按顺序拉取并分发到对应控制器"。
   - **底层（核内 6 级嵌套自循环 FSM）**：嵌套树结构，从外到内：cs（cout_slice） → yout → tile → cins（cin_slice）→ round → pos，每级用一个圆角矩形包围下一级（俄罗斯套娃式嵌套），每级旁标对应模块（cs→ofb_writer、yout/tile→line_buffer、cins→parf_accum、round/pos→mac_array + wgt_buffer）。最内层 pos 旁标"PE 阵列位置"。
3. **跨核握手（顶层右侧补充）**：在 Task Descriptor 链表上方加一条小箭头指向另一个核（Core 1 / 2 / 3 ...），标"跨核握手 (multi-core sync)"，表示核间通过描述符 + cfg_regs 握手寄存器同步。

### 标注要求
- 6 级 FSM 每级旁注明循环边界（如 cs ≤ cout_slice_total）
- 顶层描述符链表上方标"宏粒度 / Macro-level"
- 底层 6 级 FSM 上方标"微粒度 / Micro-level"
- 左半页面伪代码区下方标"算法 3.1 / Algorithm 3.1"

### 视觉层次
- 主角：右半底层 6 级嵌套 FSM 树
- 配角：顶层描述符链表与中层 DFE
- 背景：左半页面算法占位框（浅灰）

## 数据来源
- paper.md §3.4 L316-L326（分层调度三层结构）
- paper.md §4.7 L443-L458（cfg_regs + sequencer 详述）
- contributions.md C1.2 / C1.6（去中心化 valid-ready + 7 层循环）

## ASCII 示意稿

```
   ┌─────────────────────────┬─────────────────────────────────────┐
   │                         │  Macro-level: Task Descriptor List  │
   │                         │  ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐      │
   │                         │  │ID│→│WD│→│CF│→│OD│→│ID│→│..│      │
   │                         │  └──┘ └──┘ └──┘ └──┘ └──┘ └──┘      │
   │  [伪代码图               │   ▲                                 │
   │   算法 3.1               │   │ DDR 链表头基址                  │
   │   留白区                 │                                    │
   │   Pseudo-code            │  ┌──────────────────────────────┐  │
   │   placeholder            │  │ DFE 描述符获取 + sequencer   │  │
   │   for Algorithm 3.1]     │  │ 按顺序拉取 → 分发控制器       │  │
   │                         │  └────────────┬─────────────────┘  │
   │                         │               │                    │
   │                         │  Micro-level: 6-level nested FSM    │
   │                         │  ┌───────────────────────────────┐  │
   │                         │  │ cs (cout_slice) — ofb_writer │  │
   │                         │  │ ┌───────────────────────────┐ │  │
   │                         │  │ │ yout — line_buffer        │ │  │
   │                         │  │ │ ┌─────────────────────┐   │ │  │
   │                         │  │ │ │ tile — line_buffer  │   │ │  │
   │                         │  │ │ │ ┌─────────────────┐ │   │ │  │
   │                         │  │ │ │ │ cins — parf_acc │ │   │ │  │
   │                         │  │ │ │ │ ┌──────────────┐│ │   │ │  │
   │                         │  │ │ │ │ │ round — mac  ││ │   │ │  │
   │                         │  │ │ │ │ │ ┌──────────┐ ││ │   │ │  │
   │                         │  │ │ │ │ │ │ pos — PE │ ││ │   │ │  │
   │                         │  │ │ │ │ │ └──────────┘ ││ │   │ │  │
   │                         │  │ │ │ │ └──────────────┘│ │   │ │  │
   │                         │  │ │ │ └─────────────────┘ │   │ │  │
   │                         │  │ │ └─────────────────────┘   │ │  │
   │                         │  │ └───────────────────────────┘ │  │
   │                         │  └───────────────────────────────┘  │
   └─────────────────────────┴─────────────────────────────────────┘
       (左半 50% 留白)              (右半 50% 嵌套树 + 链表)

   跨核握手：Task Descriptor 链表头部 ──▶ Core 1/2/3 (核间同步)
```

## 与正文的一致性检查
- [x] §3.4 "第一层调度位于核内，基于配置寄存器与 6 层嵌套自循环 FSM" — 右半底层嵌套树 cs→yout→tile→cins→round→pos
- [x] §3.4 "cs 指 cout_slice，遍历输出通道切片；yout 是输出行；tile 是 W 维 tile；cins 是 cin_slice 累加；round 是同一 tile 内的累加轮次；pos 是 PE 阵列位置" — 嵌套树 6 级标注
- [x] §3.4 "每一层循环由对应模块的本地计数器自维护" — 每级旁注模块名
- [x] §3.4 "第二层调度位于核外，基于宏观指令流与任务描述符" — 顶层 Task Descriptor 链表
- [x] §3.4 "多层卷积的串接通过描述符链表组织：每层对应若干描述符" — 链表节点 IDMA_DESC/WDMA_DESC/CFG_WRITE/ODMA_DESC
- [x] §3.4 "第三层调度跨多核展开 ... 核间的同步通过 cfg_regs 中的握手寄存器" — 顶层右侧跨核握手箭头

## 不确定项
- [TBD: 是否在嵌套树中标注每级循环边界来源（cfg_regs 哪个寄存器）] — 倾向不标，避免图过密；正文 §4.7 表 4.x 已列
- [TBD: 算法 3.1 是否已在 paper.md 中存在] — 当前 paper.md 未见独立"算法 3.1"伪代码块；该图按"预留位"设计，伪代码 PNG 由用户后续生成

## image 生成提示词

### 中文版

科研论文配图，**分层调度示意图（组合图）**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），适合 A4 单栏插图。整张图分左右两栏：左半页面 50% 留白给伪代码 PNG 嵌入；右半页面 50% 绘制 FSM 嵌套树 + Task Descriptor 链表。**

**左半页面（50% 宽度，伪代码占位）**：
- 整块用浅灰 CMYK 0/0/0/10 填充的圆角矩形占位（边框为深灰色 #404040 虚线，提示该区域待用户后期替换）。
- 矩形中央居中两行小字"[伪代码图算法 3.1 留白区]"与"[Pseudo-code placeholder for Algorithm 3.1]"，思源黑体 / Times New Roman 10 pt 斜体。
- 矩形下方一行小字"算法 3.1 描述符链表生成 / Algorithm 3.1 Descriptor-list generation"（思源黑体 9 pt 加粗）。

**右半页面（50% 宽度，垂直分上中下三层）**：
- **顶层（占右半高度 25%）— Macro-level Task Descriptor 链表**：标题"Macro-level: Task Descriptor List"（Times New Roman 10 pt 加粗）。水平摆放 5-6 个等大小的圆角矩形节点（淡蓝色 CMYK 30/10/0/0 填充），节点内分别写"IDMA""WDMA""CFG""ODMA""IDMA""..."（Times New Roman 9 pt 加粗）。节点之间用细黑色实线箭头连接构成链表。链表头部上方标小字"DDR 链表头基址"（Times New Roman 8 pt 斜体）。右侧加一条向外的深金色 #b8860b 粗实线箭头指向画面右侧边缘外的小标签"跨核握手 → Core 1/2/3"。
- **中层（占右半高度 15%）— DFE + sequencer**：标题"DFE pull → sequencer dispatch"放在一个圆角矩形（淡黄色 CMYK 0/10/30/0 填充、深灰边框）。顶层链表底部到该矩形画一条向下细箭头，标小字"按顺序拉取并分发到对应控制器"。
- **底层（占右半高度 60%）— Micro-level 6 级嵌套 FSM 树**：标题"Micro-level: 6-level nested self-loop FSM"。绘制俄罗斯套娃式嵌套圆角矩形，从外到内 6 层：
  1. 最外层（最大）：边框深蓝 #1f4e79 实线，淡蓝填充，标题左上"cs (cout_slice)"，右上小字"← ofb_writer"。
  2. 第 2 层：边框深绿 #2e7d32 实线，淡绿填充，标题"yout"，右上"← line_buffer"。
  3. 第 3 层：边框深金 #b8860b 实线，淡黄填充，标题"tile"，右上"← line_buffer"。
  4. 第 4 层：边框深灰 #404040 实线，淡灰填充，标题"cins (cin_slice)"，右上"← parf_accum"。
  5. 第 5 层：边框深蓝实线，淡蓝填充，标题"round"，右上"← mac_array"。
  6. 最内层（最小）：边框深绿实线，淡绿填充，标题"pos (PE 阵列位置)"，右上"← mac_array + wgt_buffer"。每层标题字号 Times New Roman 9 pt 加粗。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。整体保持工科论文严谨风格。**左半页面 50% 区域保持纯净占位，不绘制任何 PE 阵列、芯片或装饰元素，仅一个虚线圆角矩形 + 占位文字**。

### English version

Scientific paper figure, **Hierarchical scheduling schematic (composite figure)**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square) or 1:1.2 (slightly taller), suitable for a single-column A4 figure. The canvas is split into a 50% left column (placeholder for pseudo-code PNG) and a 50% right column (FSM nested tree + Task Descriptor list).**

**Left column (50% width, pseudo-code placeholder)**:
- Fill the entire column with a light-gray CMYK 0/0/0/10 rounded rectangle (dashed dark-gray #404040 border to signal "user-replaceable region").
- Center two italic lines inside: "[Pseudo-code placeholder for Algorithm 3.1]" (Times New Roman 10 pt italic) and a Chinese counterpart.
- Beneath the rectangle, a small bold caption "Algorithm 3.1: Descriptor-list generation" (Source Han Sans 9 pt bold).

**Right column (50% width, three vertical tiers)**:
- **Top tier (25% height) — Macro-level Task Descriptor list**: Title "Macro-level: Task Descriptor List" (Times New Roman 10 pt bold). Horizontally place 5-6 equal-sized rounded rectangles (light-blue CMYK 30/10/0/0 fill) labeled "IDMA", "WDMA", "CFG", "ODMA", "IDMA", "..." (Times New Roman 9 pt bold). Connect them left-to-right with thin solid black arrows. Above the first node, italic small caption "DDR list-head base addr". On the right edge, draw a thick deep-gold #b8860b arrow pointing outward to a small label "Cross-core sync → Core 1/2/3".
- **Middle tier (15% height) — DFE + sequencer**: A rounded rectangle (light-yellow CMYK 0/10/30/0 fill, dark-gray border) titled "DFE pull → sequencer dispatch". Connect from the top list with a thin downward arrow labeled "sequential fetch & dispatch to controllers".
- **Bottom tier (60% height) — Micro-level 6-level nested FSM tree**: Title "Micro-level: 6-level nested self-loop FSM". Draw Russian-doll nested rounded rectangles, 6 layers from outer to inner:
  1. Outermost: deep-blue #1f4e79 border, light-blue fill, title "cs (cout_slice)" with "← ofb_writer" at the upper-right corner.
  2. Layer 2: deep-green #2e7d32 border, light-green fill, title "yout" with "← line_buffer".
  3. Layer 3: deep-gold #b8860b border, light-yellow fill, title "tile" with "← line_buffer".
  4. Layer 4: dark-gray #404040 border, light-gray fill, title "cins (cin_slice)" with "← parf_accum".
  5. Layer 5: deep-blue border, light-blue fill, title "round" with "← mac_array".
  6. Innermost (smallest): deep-green border, light-green fill, title "pos (PE position)" with "← mac_array + wgt_buffer".
- Each title in Times New Roman 9 pt bold.

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. **The left 50% column must remain a clean dashed placeholder with only the caption text — no PE array, chip, or decorative element**.
