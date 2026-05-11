# 图 4.6 配置寄存器与 6 层嵌套 FSM 结构图
# Figure 4.6 Configuration registers and 6-level nested FSM structure

## 在论文中的角色
- 首次引入：§4.7 配置寄存器与 6 层嵌套 FSM（paper.md L352 / L356）
- 引用位置：paper.md L352 / L356 / §4.12 本章小结
- 论证作用：展示 cfg_regs **双写口**设计（控制核 csr_w 仅写 4 个启动寄存器 + sequencer seq_w 写逐层 cfg）与 sequencer 实现的 **6 层嵌套循环 FSM**（cs → yout → tile → cins → round → pos）的协同关系。强调"控制核 AXI-Lite 写次数 50 余次/层 → 4 次/层"的关键收益，以及 6 层循环由各模块本地计数器自维护、无中心 FSM 协调的分布式控制思想。

## 图类型
左右二分结构图：左侧 cfg_regs **双写口**（控制核口 + sequencer 口）+ 50 余 cfg 寄存器分类示意；右侧 6 层嵌套循环用**俄罗斯套娃式嵌套方框**表示（外层 cs 最大、内层 pos 最小）。整体接近 1:1.2 高略大于宽。

## 设计要素

### 必含元素

#### 左侧 — cfg_regs 双写口
1. **csr_w 写口（左上）**：从顶部 "Control Core (CPU)" 接入一根细线进入 cfg_regs 顶部 csr_w 端口，标 "AXI-Lite (4 regs / system, 1× start_dfe)"。
2. **seq_w 写口（左下）**：从右侧 sequencer 接入一根线进入 cfg_regs 底部 seq_w 端口，标 "from sequencer (CFG_WRITE descriptors, ~50/layer)"。
3. **cfg_regs 内部 4 类寄存器分组**：
   - 循环边界类：H_in、W_in、C_in、C_out、K、stride、pad（淡蓝色）
   - 缓冲基址类：IFB / WB / OFB / bias / shortcut 基址（淡橙色）
   - 量化参数类：shift_amt、sat_min/max、shortcut_mult/shift（淡绿色）
   - 调度参数类：cin_slice、cout_slice、tile_w、strip_h（淡紫色）
4. **cfg 输出（右侧）**：右侧画一根粗箭头朝右，标 "cfg → 各模块本地计数器（loop bounds）"。

#### 右侧 — 6 层嵌套循环（俄罗斯套娃式）
5. **最外层 cs**（cout_slice）：最大方框，标 "L1: cs (cout_slice)" + 模块归属 "→ ofb_writer / SDP"
6. **第 2 层 yout**：标 "L2: yout (output row)" + "→ line_buffer"
7. **第 3 层 tile**：标 "L3: tile (W-dim tile)" + "→ line_buffer"
8. **第 4 层 cins**：标 "L4: cins (cin_slice accum)" + "→ parf_accum"
9. **第 5 层 round**：标 "L5: round (per-tile accum round)" + "→ mac_array / wgt_buffer"
10. **最内层 pos**：最小方框，标 "L6: pos (PE position)" + "→ mac_array / wgt_buffer"
11. **侧栏注**：右下小字 "no central FSM — each module's local counter steps within its layer; valid-ready handshake stitches loops"

#### 底部 — 描述符流分发
12. **DFE → sequencer 入口**：左下方画 DFE 方块（淡灰色），输出 desc_fifo（FIFO 容量 128），sequencer 从 desc_fifo 取 CFG_WRITE 描述符写入 cfg_regs 的 seq_w 端口。
13. **done_sticky 输出**：sequencer 右下角画一根细线标 "done_sticky → IRQ / status"，朝下回到控制核。

### 标注要求
- cfg 寄存器总数：约 50+
- csr_w 端口：4 个启动寄存器（描述符链表基址、链表长度、控制位、start_dfe 触发位）
- desc_fifo 容量：128
- 6 层循环顺序：cs → yout → tile → cins → round → pos（外 → 内）
- 各层归属模块（标注于嵌套方框右侧）

### 视觉层次
- 主角：cfg_regs（左大方块）+ 6 层嵌套方框（右）
- 配角：DFE / desc_fifo（左下灰色配角）、控制核（左上灰色配角）
- 背景：cfg 输出箭头、done_sticky 回线

## 数据来源
- paper.md §4.7
- docs/modules/cfg_regs.md / sequencer.md / dfe.md
- contributions.md（cfg 双写口 + 6 层嵌套 FSM 设计贡献）
- CLAUDE.md（DFE 自动拉描述符、控制核 4 寄存器）

## ASCII 示意稿

```
   ┌──────────────────┐            ┌──────────────────────────────────────┐
   │ Control Core     │            │  6-level nested loops (no central FSM)│
   │ (CPU, gray)      │            │  ┌────────────────────────────────┐  │
   └────────┬─────────┘            │  │ L1: cs (cout_slice)            │  │
            │ AXI-Lite              │  │   → ofb_writer / SDP           │  │
            │ (4 regs/system,       │  │  ┌──────────────────────────┐  │  │
            │  1× start_dfe)        │  │  │ L2: yout (output row)    │  │  │
            ▼                      │  │  │   → line_buffer          │  │  │
   ┌──────────────────────┐         │  │  │ ┌──────────────────────┐ │  │  │
   │ cfg_regs             │ ──cfg─▶│  │  │ │ L3: tile (W-dim tile)│ │  │  │
   │  ┌────────────────┐  │         │  │  │ │   → line_buffer      │ │  │  │
   │  │ loop bounds    │  │         │  │  │ │ ┌──────────────────┐ │ │  │  │
   │  │ (H,W,Cin,Cout, │  │         │  │  │ │ │ L4: cins         │ │ │  │  │
   │  │  K,stride,pad) │  │         │  │  │ │ │  → parf_accum    │ │ │  │  │
   │  ├────────────────┤  │         │  │  │ │ │ ┌──────────────┐ │ │ │  │  │
   │  │ buffer addrs   │  │         │  │  │ │ │ │ L5: round    │ │ │ │  │  │
   │  │ (IFB,WB,OFB,…) │  │         │  │  │ │ │ │  → mac/wrf   │ │ │ │  │  │
   │  ├────────────────┤  │         │  │  │ │ │ │ ┌──────────┐ │ │ │ │  │  │
   │  │ quant params   │  │         │  │  │ │ │ │ │ L6: pos  │ │ │ │ │  │  │
   │  │ (shift, sat..) │  │         │  │  │ │ │ │ │  → mac/  │ │ │ │ │  │  │
   │  ├────────────────┤  │         │  │  │ │ │ │ │   wrf    │ │ │ │ │  │  │
   │  │ schedule params│  │         │  │  │ │ │ │ └──────────┘ │ │ │ │  │  │
   │  │ (cin/cout_slice│  │         │  │  │ │ │ └──────────────┘ │ │ │  │  │
   │  │  ,tile,strip)  │  │         │  │  │ │ └──────────────────┘ │ │  │  │
   │  └────────────────┘  │         │  │  │ └──────────────────────┘ │  │  │
   │   csr_w     seq_w    │         │  │  └──────────────────────────┘  │  │
   └────▲──────────▲──────┘         │  └────────────────────────────────┘  │
        │          │ CFG_WRITE      │                                      │
        │          │ desc (~50/lyr) │  no central FSM — local counters     │
        │          │                │  + valid-ready handshake             │
        │   ┌──────┴────────┐       └──────────────────────────────────────┘
        │   │ sequencer     │ ──────── done_sticky → IRQ / status ────┐
        │   └──────▲────────┘                                          │
        │          │ desc                                              │
        │   ┌──────┴────────┐                                          │
        │   │ desc_fifo     │ ◀── DFE (auto-fetch desc list)           │
        │   │ (128 entries) │                                          │
        │   └───────────────┘                                          │
        └──────────────────────────────────────────────────────────────┘
```

## 与正文的一致性检查
- [x] §4.7 "cfg_regs 内部约 50 余个寄存器，可分四类" — 左侧方块 4 类寄存器分组
- [x] §4.7 "csr_w 仅写 4 个启动寄存器" — csr_w 端口标 "4 regs/system"
- [x] §4.7 "seq_w 由 sequencer 消费 CFG_WRITE 描述符流写入" — seq_w 端口标 "CFG_WRITE descriptors, ~50/layer"
- [x] §4.7 "6 层循环 (cs → yout → tile → cins → round → pos)" — 右侧 6 层嵌套方框 L1..L6
- [x] §4.7 各层归属模块 "ofb_writer / line_buffer / parf_accum / mac_array / wgt_buffer" — 嵌套方框右侧标注一一对应
- [x] §4.7 "无中心 FSM，模块本地计数器 + valid-ready 握手" — 右下小字标注
- [x] §4.7 "DFE 按描述符类型分发，CFG_WRITE 由 sequencer 取出" — DFE → desc_fifo → sequencer 路径
- [x] §4.7 "desc_fifo 容量从 32 扩到 128" — desc_fifo 方框标 "128 entries"
- [x] §4.7 "层完成后 done_sticky 拉起，控制核读 STATUS 或接 IRQ" — done_sticky 输出回控制核

## 不确定项
- [TBD: cfg_regs 实际寄存器数 "50 余" 在 docs/modules/cfg_regs.md 有精确清单 — 图中按 §4.7 文字描述，标 "~50+"]
- [TBD: 6 层嵌套方框的"俄罗斯套娃式"嵌套深度较深，实际绘图时若空间紧张可改为缩进列表式（同样表达嵌套层级），由用户在 PowerPoint/Visio 中决定]

## image 生成提示词

### 中文版

科研论文配图，**配置寄存器与 6 层嵌套 FSM 结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。布局为左右二分：左侧 cfg_regs 双写口结构（占约 45% 宽度），右侧 6 层嵌套循环方框（占约 55% 宽度），底部 sequencer + DFE + desc_fifo 横排支撑。**

**版式**：
- **顶部小框**：左上方 "Control Core (CPU)" 淡灰色 CMYK 0/0/0/15 填充小方框，从下方接一根细黑色实线箭头朝下，标 "AXI-Lite (4 regs/system, 1× start_dfe)"。
- **左侧主体 — cfg_regs**：圆角矩形大框（淡黄色 CMYK 0/10/30/0 填充、深灰色边框），标题 "cfg_regs"。内部从上到下分 4 个子方框：
  1. "loop bounds (H, W, C_in, C_out, K, stride, pad)" — 淡蓝色 CMYK 30/10/0/0 填充
  2. "buffer addrs (IFB, WB, OFB, bias, shortcut)" — 淡橙色 CMYK 0/30/30/0 填充
  3. "quant params (shift_amt, sat_min/max, shortcut_mult/shift)" — 淡绿色 CMYK 30/0/20/0 填充
  4. "schedule params (cin_slice, cout_slice, tile_w, strip_h)" — 淡紫色 CMYK 20/20/0/0 填充
  - 顶部端口标 "csr_w" 接 Control Core 的箭头；底部端口标 "seq_w" 接来自 sequencer 的箭头（标 "CFG_WRITE desc (~50/layer)"）。
  - 右侧画一根粗黑色实线箭头标 "cfg → 各模块本地计数器 (loop bounds)" 指向右侧 6 层循环方框。
- **右侧主体 — 6 层嵌套循环**：俄罗斯套娃式嵌套方框，从外到内：
  - L1 最外（淡蓝色边框，无填充）："L1: cs (cout_slice)  → ofb_writer / SDP"
  - L2（嵌套于 L1 内）："L2: yout (output row)  → line_buffer"
  - L3："L3: tile (W-dim tile)  → line_buffer"
  - L4："L4: cins (cin_slice accum)  → parf_accum"
  - L5："L5: round (per-tile accum round)  → mac_array / wgt_buffer"
  - L6 最内（最小方框，淡蓝色填充）："L6: pos (PE position)  → mac_array / wgt_buffer"
  - 右下角小字斜体注："no central FSM — local counters + valid-ready handshake"
- **底部 — 描述符流支撑层**：横向 3 块从左到右：
  - "DFE (auto-fetch desc list)" 淡灰色填充小方框
  - "desc_fifo (128 entries)" 淡灰色填充小方框
  - "sequencer" 淡黄色填充小方框
  - 用粗黑色实线箭头串联：DFE → desc_fifo → sequencer
  - sequencer 上方接一根线进入 cfg_regs 的 seq_w（即"CFG_WRITE desc"路径）
  - sequencer 右侧画一根细线朝右上回到 Control Core，标 "done_sticky → IRQ / status"

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗，循环层级 L1..L6 编号加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **Configuration registers (cfg_regs) and 6-level nested FSM structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. Layout is left-right split: cfg_regs dual-write-port structure on left (~45% width), 6-level nested loop boxes on right (~55% width), with sequencer + DFE + desc_fifo as a horizontal supporting row at bottom.**

**Layout**:
- **Top small box**: Upper-left "Control Core (CPU)" small box (light-gray CMYK 0/0/0/15 fill); from below it draw a thin black solid arrow downward, labeled "AXI-Lite (4 regs/system, 1× start_dfe)".
- **Left main body — cfg_regs**: Large rounded rectangle (light-yellow CMYK 0/10/30/0 fill, dark-gray border), titled "cfg_regs". Inside, four sub-boxes stacked top to bottom:
  1. "loop bounds (H, W, C_in, C_out, K, stride, pad)" — light-blue CMYK 30/10/0/0 fill
  2. "buffer addrs (IFB, WB, OFB, bias, shortcut)" — light-orange CMYK 0/30/30/0 fill
  3. "quant params (shift_amt, sat_min/max, shortcut_mult/shift)" — light-green CMYK 30/0/20/0 fill
  4. "schedule params (cin_slice, cout_slice, tile_w, strip_h)" — light-purple CMYK 20/20/0/0 fill
  - Top port labeled "csr_w" accepting arrow from Control Core; bottom port labeled "seq_w" accepting arrow from sequencer (labeled "CFG_WRITE desc (~50/layer)").
  - From right edge draw a thick black solid arrow labeled "cfg → per-module local counters (loop bounds)" pointing to the 6-level loop on the right.
- **Right main body — 6-level nested loops**: Russian-doll style nested rectangles, from outermost to innermost:
  - L1 outermost (light-blue border, no fill): "L1: cs (cout_slice)  → ofb_writer / SDP"
  - L2 (nested inside L1): "L2: yout (output row)  → line_buffer"
  - L3: "L3: tile (W-dim tile)  → line_buffer"
  - L4: "L4: cins (cin_slice accum)  → parf_accum"
  - L5: "L5: round (per-tile accum round)  → mac_array / wgt_buffer"
  - L6 innermost (smallest box, light-blue fill): "L6: pos (PE position)  → mac_array / wgt_buffer"
  - Lower-right corner small italic note: "no central FSM — local counters + valid-ready handshake"
- **Bottom — descriptor flow supporting row**: Three boxes horizontally left to right:
  - "DFE (auto-fetch desc list)" light-gray fill small box
  - "desc_fifo (128 entries)" light-gray fill small box
  - "sequencer" light-yellow fill small box
  - Connected by thick black solid arrows: DFE → desc_fifo → sequencer
  - From sequencer top, a line goes up into cfg_regs' seq_w port (the "CFG_WRITE desc" path)
  - From sequencer right side, a thin line goes upper-right back to Control Core, labeled "done_sticky → IRQ / status"

**Typography**: All English in Times New Roman 10 pt with bold module titles; loop level numbers L1..L6 in bold; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
