# 图 4.4 SDP 后处理模块结构图
# Figure 4.4 SDP post-processing module structure

## 在论文中的角色
- 首次引入：§4.5 SDP 后处理模块（paper.md L324 / L326）
- 引用位置：paper.md L324 / L326
- 论证作用：展示 ofb_writer 内部 5 步处理链 "PSUM → bias add → shift → saturate → residual fusion → saturate → OFB write"，强调 INT8 量化推理后处理一站式完成（含 ResNet 残差通路），无需主机介入。读者应理解 bias_rf / Shortcut Bank 两个侧路寄存器堆与 OFB ring 的角色。

## 图类型
数据流图（5 阶段流水，**用 2 行折回**避免横向过长）。

## 设计要素

### 必含元素
1. **输入 PSUM 32-bit**（左上）：从 parf_accum 来的 16 路 accum-done PSUM。
2. **bias_rf 寄存器堆**：第 1 阶段上方侧路，标 "bias_rf (32 b / cout)"，由 RDMA 预加载。
3. **Stage 1 — bias add**：第 1 阶段加法器（PSUM + bias）。
4. **Stage 2 — shift**：算术右移，移位量来自 cfg_regs（shift_amt）。
5. **Stage 3 — saturate to INT8**：饱和裁剪到 [−128, 127]。
6. **第 1 行折到第 2 行的转角**：粗箭头折回，提示后续两步在第 2 行。
7. **Shortcut Bank 残差缓冲**：第 4 阶段上方侧路，标 "Shortcut Bank (INT8 / spatial)"，由上一层 OFM 直接写入。
8. **Stage 4 — residual scale + add**：按 shortcut_mult / shortcut_shift 缩放后相加（含 bypass mux，由 cfg 选通）。
9. **Stage 5 — saturate again**：第二次饱和到 INT8。
10. **OFB ring**：右侧环形缓冲（OFB=2048 word），按 (y_out, x_out, c_out) 写入。
11. **row_credit 反向控制流**：OFB → ODMA → ofb_writer 的虚线。
12. **本地计数器（4 级）**：(y_out, x_out, cout_col, cout_slice)。

### 标注要求
- 各阶段位宽：bias add 后 32 b → shift 后 INT8（饱和前先到 INT8 范围）→ residual 后再饱和 INT8
- 标注 "single-stage combinational (current)；future: insert pipeline reg → see §5"
- OFB ring 容量：OFB=2048 word
- bias_rf 由 RDMA 预拉入；Shortcut Bank 由上一层 OFM push（无主机介入）
- 残差 bypass：由 cfg 中残差使能位选通

### 视觉层次
- 主角：5 阶段流水链（折成 2 行）
- 配角：bias_rf、Shortcut Bank、OFB ring
- 背景：cfg 输入（shift_amt、sat_min/sat_max、shortcut_mult/shift、residual_en）、row_credit 虚线

## 数据来源
- paper.md §4.5
- docs/modules/ofb_writer.md
- contributions.md（SDP 后处理一站式融合）
- STATUS.md / memory（SDP 量化链当前为单 stage 组合实现，关键路径影响 Fmax）

## ASCII 示意稿

```
   from parf_accum (16× PSUM 32 b, v/r)
        │
        ▼
   ┌──────────────────────────────────────────────────────────────┐
   │ ofb_writer (SDP — Scalar Data Path)                          │
   │                                                              │
   │   bias_rf (32 b / cout)                                      │
   │        │                                                     │
   │        ▼                                                     │
   │   ┌────────┐    ┌─────────┐    ┌──────────┐                  │
   │   │ +bias  │ ─▶│  shift  │ ─▶│ saturate │ ─┐                │
   │   │ (32 b) │    │  (>>)   │    │ → INT8   │  │ stage 1→2→3   │
   │   └────────┘    └─────────┘    └──────────┘  │               │
   │     [stage 1]    [stage 2]      [stage 3]    │               │
   │                                              │ wrap row 1→2  │
   │   ┌──────────────────────────────────────────┘               │
   │   │                                                          │
   │   │      Shortcut Bank (INT8 / spatial)                      │
   │   │            │ residual                                    │
   │   ▼            ▼                                             │
   │   ┌────────────┐    ┌──────────┐    ┌──────────────┐         │
   │   │ +residual  │ ─▶ │ saturate │ ─▶ │  OFB ring    │         │
   │   │ (× scale,  │    │ → INT8   │    │ (2048 word)  │         │
   │   │  +bypass)  │    │          │    │              │         │
   │   └─────▲──────┘    └──────────┘    └──────┬───────┘         │
   │   [stage 4 + bypass]  [stage 5]            │                 │
   │     ▲                                       │                 │
   │     └── cfg.residual_en (mux)              │                 │
   │                                             │                 │
   │   row_credit ◀───── from ODMA (back-pressure)                │
   │                                             │                 │
   │   currently single-stage combinational                       │
   │   (future: pipeline reg insertion → see §5)                  │
   └─────────────────────────────────────────────┼────────────────┘
                                                 │ INT8 row
                                                 ▼
                                              to ODMA

   local counters: (y_out, x_out, cout_col, cout_slice) — 4 levels
   cfg: shift_amt, sat_min/max, shortcut_mult/shift, residual_en
```

## 与正文的一致性检查
- [x] §4.5 五步流程 "偏置加 → 移位 → 饱和 → 残差 → 饱和 → OFB" 与图中 5 stage 顺序一致
- [x] §4.5 "目前在单 stage 内组合实现，未做 stage 切分" — 图注 "currently single-stage combinational"
- [x] §4.5 "Shortcut Bank 与 bias_rf 让残差加与偏置加都在片上完成，主机不需要为每层介入" — 两个侧路寄存器堆在图中明示
- [x] §4.5 "OFB 写入端按行粒度组织为环形缓冲；ODMA 拉起 row_credit" — 图中 OFB ring + row_credit 反压
- [x] OFB=2048 word 标注与 CLAUDE.md 一致

## 不确定项
- [TBD: 残差融合在某些层不启用 — 已在图中以 bypass mux 表示，由 cfg.residual_en 选通]
- [CHECK: Shortcut Bank 容量在 §4.5 未给出具体数字，待 §5.4 缩减后实测确认 — 图中暂不标注容量]

## image 生成提示词

### 中文版

科研论文配图，**SDP 后处理模块（ofb_writer）结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。5 个阶段必须用 2 行折回布局，避免横向过长。**

**版式（双行折返流水）**：
- **顶部输入**：左上方画 "from parf_accum (16× PSUM 32 b, v/r)" 小标签，下接一根细黑色实线箭头进入 SDP 主框。
- **主体（圆角矩形大框）**：标题 "ofb_writer (SDP — Scalar Data Path)"（淡黄色 CMYK 0/10/30/0 填充、深灰色边框）。
- **第 1 行（左 → 右，stage 1–3）**：水平摆放 3 个方框：
  1. "+bias (32 b)"（stage 1，淡蓝色 CMYK 30/10/0/0 填充）— 上方画一个小方框 "bias_rf (32 b / cout)"（淡橙色 CMYK 0/30/30/0），用细实线箭头向下连入，标 "bias"
  2. "shift (>>)"（stage 2，淡蓝色填充）
  3. "saturate → INT8"（stage 3，淡蓝色填充）
  - 3 个方框间用粗黑色实线箭头水平连接
- **行间转角**：第 3 个方框右侧画一根粗箭头向下再向左折回到第 2 行起点（U 字形折回路径），标 "wrap row 1 → row 2"
- **第 2 行（左 → 右，stage 4–5 + OFB）**：水平摆放 3 个方框：
  4. "+residual (× scale, + bypass)"（stage 4，淡蓝色填充）— 上方画一个小方框 "Shortcut Bank (INT8 / spatial)"（淡橙色填充），用细实线箭头向下连入，标 "residual"。stage 4 方框左下角画一个小菱形 "mux"，从下方接一根细虚线标 "cfg.residual_en"
  5. "saturate → INT8"（stage 5，淡蓝色填充）
  6. "OFB ring (2048 word)"（淡绿色 CMYK 30/0/20/0 椭圆环或圆角矩形，环上画一个顺时针箭头表示 ring 推进）
- **底部输出**：OFB ring 下方画一根粗黑色实线箭头朝下，标 "INT8 row  →  to ODMA"
- **反压回线**：从 OFB ring 左侧画一根**深红色虚线箭头**绕回 stage 5 下方，标 "row_credit (from ODMA)"
- **左下角图注**：小字 "currently single-stage combinational (future: pipeline reg → see §5)"，Times New Roman 8 pt 斜体
- **右下角**：画一个小方框 "local counters (y_out, x_out, cout_col, cout_slice); cfg: shift_amt, sat_min/max, shortcut_mult/shift, residual_en"（淡灰色填充）

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗，stage 编号用 [stage N] 小字标注于方框下方；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **SDP post-processing module (ofb_writer) structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. The 5 stages MUST be arranged in a two-row wrap-back layout to avoid an overly wide figure.**

**Layout (two-row wrapped pipeline)**:
- **Top input**: Place a small label "from parf_accum (16× PSUM 32 b, v/r)" at upper-left, with a thin black solid arrow downward into the SDP main frame.
- **Main body (large rounded rectangle)**: Titled "ofb_writer (SDP — Scalar Data Path)" (light-yellow CMYK 0/10/30/0 fill, dark-gray border).
- **Row 1 (left → right, stages 1–3)**: Place 3 boxes horizontally:
  1. "+bias (32 b)" (stage 1, light-blue CMYK 30/10/0/0 fill) — above it draw a small box "bias_rf (32 b / cout)" (light-orange CMYK 0/30/30/0), connected downward by a thin solid arrow labeled "bias"
  2. "shift (>>)" (stage 2, light-blue fill)
  3. "saturate → INT8" (stage 3, light-blue fill)
  - Connect the 3 boxes with thick black solid arrows horizontally
- **Row wrap**: From the right of box 3 draw a thick arrow going down then bending left back to the start of row 2 (U-shape wrap path), labeled "wrap row 1 → row 2"
- **Row 2 (left → right, stages 4–5 + OFB)**: Place 3 boxes horizontally:
  4. "+residual (× scale, + bypass)" (stage 4, light-blue fill) — above it a small box "Shortcut Bank (INT8 / spatial)" (light-orange fill), connected downward by a thin solid arrow labeled "residual". At lower-left of box 4 draw a small diamond "mux" with a thin dashed line from below labeled "cfg.residual_en"
  5. "saturate → INT8" (stage 5, light-blue fill)
  6. "OFB ring (2048 word)" (light-green CMYK 30/0/20/0 elliptical ring or rounded rectangle, with a clockwise arrow on the ring indicating advance)
- **Bottom output**: Below OFB ring draw a thick black solid arrow downward, labeled "INT8 row  →  to ODMA"
- **Back-pressure return**: From the left of OFB ring draw a **dark-red dashed arrow** wrapping back below stage 5, labeled "row_credit (from ODMA)"
- **Lower-left caption**: Small italic text "currently single-stage combinational (future: pipeline reg → see §5)", Times New Roman 8 pt
- **Lower-right**: A small box "local counters (y_out, x_out, cout_col, cout_slice); cfg: shift_amt, sat_min/max, shortcut_mult/shift, residual_en" (light-gray fill)

**Typography**: All English in Times New Roman 10 pt with bold module titles; stage numbers in small "[stage N]" labels under each box; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
