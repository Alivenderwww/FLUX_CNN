# 图 3.2 加速核内数据通路结构图
# Figure 3.2 Datapath structure inside the accelerator core

## 在论文中的角色
- 首次引入：§3.3 总体架构（paper.md L234）
- 引用位置：paper.md L234
- 论证作用：在系统架构（图 3.1）下推一层，给出加速核内 5 个流水模块 + 共享 cfg_regs 的串接关系。读者读完此图后应理解 "line_buffer → mac_array → parf_accum → ofb_writer + 侧路 wgt_buffer" 的核内拓扑。

> **Polisher 提醒**：paper.md L234 后未必有独立 "**图 3.2 ...**" 加粗标题行，建议 Phase 7 补一行。

## 图类型
数据通路图（核内 5 模块 + cfg_regs）。

## 设计要素

### 必含元素
1. **行缓存模块（line_buffer）**：左上首块；内部画 IFB ring + ARF；上接 IDMA 写口，下出 *K*×*K* 窗口。
2. **MAC 阵列模块（mac_array）**：核心计算块；标注 "16 col × 16 PE = 256 INT8 MAC"；输入激活 + 权重；输出 16 路 PSUM。
3. **权重缓存模块（wgt_buffer）**：右上侧路；上接 WDMA；画 WB(1024) → WRF(32)；侧路供 mac_array。
4. **部分和累加模块（parf_accum）**：左下；内含 16 个独立 SRAM (parf_col × 16)，每列 PARF=32。
5. **SDP 后处理模块（ofb_writer）**：右下；内部链 "bias→shift→sat→residual→sat→OFB"；右接 OFB ring；上接 bias_rf + Shortcut Bank；输出 ODMA。
6. **共享 cfg_regs**：底部一条横贯的浅色长条，向 5 个模块各拉一条细虚线（"cfg broadcast"）。
7. **valid-ready 握手**：每两个模块之间的连线上画 "v/r" 小标。

### 标注要求
- 数据宽度：激活向量 INT8×16、权重向量 INT8×16、PSUM 32-bit×16 路、OFM INT8
- 容量：IFB=8192 word、WB=1024 word、WRF=32、ARF=32、PARF=32、OFB=2048 word
- "row credit" 反向控制流（line_buffer → IDMA、ofb_writer → ODMA）

### 视觉层次
- 主角：5 个模块串联的主数据流（粗箭头）
- 配角：cfg_regs 广播线、bias_rf 与 Shortcut Bank、ARF
- 背景：IDMA / WDMA / ODMA 接口（仅画接口框，不展开）

## 数据来源
- contributions.md C1.x
- §4.2 - §4.6 各模块 paper.md 文字
- CLAUDE.md "Core pipeline (5 模块 + 共享 cfg_regs)"

## ASCII 示意稿

```
   IDMA              WDMA              bias / shortcut
    │ INT8 row        │ INT8 weights    │
    ▼                 ▼                 │
   ┌─────────────┐   ┌──────────────┐   │
   │ line_buffer │   │ wgt_buffer   │   │
   │ IFB(8192)+  │   │ WB(1024)→    │   │
   │ ARF(32)     │   │ WRF(32)      │   │
   └──────┬──────┘   └──────┬───────┘   │
          │ K×K window       │ INT8×16   │
          │ v/r              │ v/r       │
          └────────┬─────────┘           │
                   ▼                     │
        ┌──────────────────────────┐     │
        │  mac_array               │     │
        │  16 col × 16 PE          │     │
        │  = 256 INT8 MAC          │     │
        └────────────┬─────────────┘     │
                     │ 16× PSUM (32 b)   │
                     │ v/r               │
                     ▼                   │
        ┌──────────────────────────┐     │
        │  parf_accum              │     │
        │  16 × parf_col SRAM      │     │
        │  PARF=32 each col        │     │
        └────────────┬─────────────┘     │
                     │ 16× accum-done    │
                     │ v/r               │
                     ▼                   │
        ┌────────────────────────────────┴───┐
        │  ofb_writer (SDP)                  │
        │  bias → shift → sat → residual     │
        │  → sat → OFB(2048)                 │
        └────────────────┬───────────────────┘
                         │ INT8 row, row credit ↑
                         ▼
                       ODMA

   ┌──────────────────────────────────────────────────────┐
   │  cfg_regs (50+ regs)  ── broadcast (dashed) ──       │
   └──────────────────────────────────────────────────────┘
```

## 与正文的一致性检查
- [x] §3.3 描述的 5 模块 + 共享 cfg_regs 拓扑与图一致
- [x] 模块间 valid-ready 握手在图中以 "v/r" 标注
- [x] WRF / ARF / PARF 三级寄存器堆全部出现（与 §6.1 创新点一致）

## 不确定项
- [TBD: ARF 是否独立画一个块还是嵌入 line_buffer 内] — 倾向嵌入 line_buffer 输出端

## image 生成提示词

### 中文版

科研论文配图，**加速核内数据通路结构图**，IEEE 期刊配色风格，白底黑字，禁用彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（2 行 + 底部 cfg_regs 横条，紧凑接近正方形）**：
- **第一行（左右并列两个接口模块）**：左上 line_buffer 圆角矩形（淡蓝 CMYK 30/10/0/0 填充、深蓝边框），内分 3 行写 "line_buffer" / "IFB(8192)+ARF(32)"；右上 wgt_buffer 圆角矩形（同色），内写 "wgt_buffer" / "WB(1024)→WRF(32)"。两模块顶部各连一条向下细黑箭头到外部 IDMA / WDMA 标签。
- **第二行（计算主轴）**：第一行下方居中画 mac_array 大圆角矩形（淡黄 CMYK 0/10/30/0 填充，最深色作为主角），内分 3 行写 "mac_array" / "16 col × 16 PE" / "= 256 INT8 MAC"。line_buffer 与 wgt_buffer 都以粗黑实线箭头汇入 mac_array 顶部，箭头旁分别标 "K×K window v/r" / "INT8×16 v/r"。
- **第三行**：mac_array 下方画 parf_accum 圆角矩形（淡蓝填充），内写 "parf_accum / 16 × parf_col SRAM / PARF=32 each col"，箭头标 "16× PSUM 32b, v/r"。
- **第四行**：parf_accum 右侧画 ofb_writer 长条圆角矩形（淡绿 CMYK 20/0/30/0 填充），内分 2 行写 "ofb_writer (SDP)" / "bias→shift→sat→residual→sat→OFB(2048)"，左侧由 parf_accum 用箭头汇入，右侧画细箭头连到 ODMA 外部标签，箭头旁标 "INT8 row, row credit↑"。ofb_writer 上方再画一个小方块 "bias / shortcut" 标签连入。
- **底部 cfg_regs 横条**：贯穿全图底部，浅灰 CMYK 0/0/0/10 填充长条，内写 "cfg_regs (50+ regs) — broadcast"，从该长条向上拉 5 条细灰色虚线分别接到 5 个模块（line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer）。

**字体**：英文 Times New Roman 10 pt，模块名加粗；位宽与 v/r 标签 Times New Roman 8 pt 等宽。**禁止**手写体、彩色渐变、3D 阴影、装饰图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **datapath structure inside the accelerator core**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (two upper rows + cfg_regs strip at bottom, compact near-square)**:
- **Row 1 (two interface modules side-by-side)**: Upper-left line_buffer rounded rectangle (light-blue CMYK 30/10/0/0 fill, dark-blue border), three lines: "line_buffer" / "IFB(8192)+ARF(32)". Upper-right wgt_buffer rounded rectangle (same color): "wgt_buffer" / "WB(1024)→WRF(32)". Each module top connects upward via thin black arrows to external "IDMA" / "WDMA" labels.
- **Row 2 (compute focal point)**: Below row 1, centered, draw a large mac_array rounded rectangle (light-yellow CMYK 0/10/30/0 fill, deepest tone as focal point), three lines: "mac_array" / "16 col × 16 PE" / "= 256 INT8 MAC". Both line_buffer and wgt_buffer connect to mac_array top via thick black arrows, labeled "K×K window v/r" and "INT8×16 v/r" respectively.
- **Row 3**: Below mac_array draw parf_accum rounded rectangle (light-blue fill): "parf_accum / 16 × parf_col SRAM / PARF=32 each col"; arrow labeled "16× PSUM 32 b, v/r".
- **Row 4**: To the right of parf_accum draw a long ofb_writer rounded rectangle (light-green CMYK 20/0/30/0 fill), two lines: "ofb_writer (SDP)" / "bias→shift→sat→residual→sat→OFB(2048)". Connect parf_accum to its left input; from its right draw a thin arrow to external "ODMA" label with caption "INT8 row, row credit↑". Above ofb_writer add a small "bias / shortcut" label box connecting in.
- **Bottom cfg_regs strip**: Spanning the full figure bottom, light-gray (CMYK 0/0/0/10) long bar containing "cfg_regs (50+ regs) — broadcast". Draw 5 thin gray dashed lines from this strip up to the 5 modules.

**Typography**: English in Times New Roman 10 pt, module names bold; bit-widths and v/r labels in Times New Roman 8 pt monospaced. **Strictly forbidden**: handwritten fonts, color gradients, 3D shadows, decorative icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
