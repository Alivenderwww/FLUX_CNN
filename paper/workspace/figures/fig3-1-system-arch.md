# 图 3.1 加速器系统总体架构图
# Figure 3.1 Overall system architecture of the accelerator

## 在论文中的角色
- 首次引入：§3.3 总体架构（paper.md L226 / L228 / L230）
- 引用位置：paper.md L226 / L228 / L230
- 论证作用：奠定整篇硬件章节的"控制核 + 加速核 + 多核扩展 + DDR + AXI 互连"的全局视图。读者看到此图后应理解 FLUX_CNN 对外接口（1× AXI4 主 + 1× AXI-Lite 从）与对内层级。

## 图类型
系统级架构框图。

## 设计要素

### 必含元素
1. **控制核（Host CPU）**：上方方块，标注 "Host CPU (ARM / RISC-V)"。
2. **片外 DDR**：下方方块，标注 "Off-chip DDR3"。
3. **AXI 互连**：中央偏上方块，标注 "AXI Interconnect"。
4. **多核扩展层（multicore_top）**：中央大框，内含 *N* 个加速核（*N* = 1/2/4 标注）。
5. **加速核（Core[i]）**：multicore_top 内重复 *N* 次的小框，每核标 "Core 0 / Core 1 / ..."。
6. **2 个外部接口**：
   - AXI4 完整主接口（粗箭头：multicore_top → AXI 互连 → DDR）
   - AXI-Lite 从接口（细箭头：控制核 → AXI 互连 → multicore_top.csr）
7. **跨核 SRAM 直送通道**（虚线 / dash）：核间 IFB 互通，标注 "cross-core SRAM push (N≥2)"。

### 标注要求
- AXI4 主线上标注 "BUS_DATA_W = 128 bit"
- AXI-Lite 从线上标注 "CSR_DATA_W = 32 bit"
- 每核标注 "16×16 INT8 MAC"
- 多核地址映射：Core[i] IFB `0x8000_0000 + i × 0x1000_0000`（图注）

### 视觉层次
- 主角：multicore_top 大框（含 *N* 个加速核）
- 配角：DDR、控制核、AXI 互连
- 背景：跨核 SRAM 直送虚线（仅 *N* ≥ 2 时存在）

## 数据来源
- contributions.md C1 系列（架构）
- CLAUDE.md "顶层结构两层" 段
- STATUS.md §1 / §2

## ASCII 示意稿

```
   ┌──────────────────────────┐                ┌──────────────────────────┐
   │   Host CPU (ARM/RISC-V)  │ ── done IRQ ── │ (optional debug / IO)    │
   └────────────┬─────────────┘                └──────────────────────────┘
                │ AXI-Lite (32 b, CSR)
                ▼
   ┌─────────────────────────────────────────────────────────────────────┐
   │                       AXI Interconnect                              │
   └────────────┬───────────────────────────────────────────┬────────────┘
                │ AXI-Lite (32 b)                AXI4 (128 b) │
                ▼                                            ▼
   ┌─────────────────────────────────────────────────────────────────────┐
   │                multicore_top  (N = 1 / 2 / 4)                       │
   │   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐         │
   │   │  Core 0  │   │  Core 1  │   │  Core 2  │   │  Core 3  │         │
   │   │ 16×16    │   │ 16×16    │   │ 16×16    │   │ 16×16    │         │
   │   │ INT8 MAC │   │ INT8 MAC │   │ INT8 MAC │   │ INT8 MAC │         │
   │   └────┬─────┘   └────┬─────┘   └────┬─────┘   └────┬─────┘         │
   │        └─────────────┴── x-core SRAM push ──┴────────┘  (dash)      │
   └────────────────────────────────┬────────────────────────────────────┘
                                    │ AXI4 Master (128 b)
                                    ▼
                            ┌──────────────────┐
                            │   Off-chip DDR3  │
                            └──────────────────┘

   Address map: Core[i] IFB = 0x8000_0000 + i × 0x1000_0000
```

## 与正文的一致性检查
- [x] §3.3 "加速器对外仅暴露两个接口：一个 AXI4 完整主接口与一个 AXI-Lite 从接口" — 图中两根接口线
- [x] §3.3 "多核扩展层包含 *N* 个加速核（*N*=1/2/4 已综合通过）" — 图中标注 *N*=1/2/4
- [x] 跨核 SRAM 直送在 §3.3 段末 + §4.11 详述 — 图中以虚线提示

## 不确定项
- [TBD: 是否在主图中显示控制核以外的外设（如 USB / Camera Sensor）] — 倾向不显示，保持架构图纯净

## image 生成提示词

### 中文版

科研论文配图，**加速器系统总体架构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（自上而下三层）**：
- **顶层**：左上方画 Host CPU 方框（淡灰色 CMYK 0/0/0/10 填充、黑色实线边框），内写 "Host CPU (ARM / RISC-V)"，约 12 字宽。
- **中上层**：水平贯穿一条 AXI Interconnect 长条形方框（淡蓝色 CMYK 30/10/0/0 填充），内写 "AXI Interconnect"，约 10 字宽。Host CPU 下方画一条向下黑色细箭头连到 AXI Interconnect，箭头旁标 "AXI-Lite (32 b, CSR)"。
- **中央层（主角）**：画 multicore_top 大圆角矩形（淡黄色 CMYK 0/10/30/0 填充、深灰色边框），框上方写标题 "multicore_top (N = 1 / 2 / 4)"。框内**水平 2×2 矩阵**排列 4 个加速核小方框（淡蓝色填充、Core 0 / Core 1 / Core 2 / Core 3），每核内分两行写 "16×16" / "INT8 MAC"，约 8 字宽。4 个核之间用细虚线连接表示 "cross-core SRAM push (dash)"，虚线下方写小字标注。
- **底层**：multicore_top 下方画一条粗黑色实线箭头连到 Off-chip DDR3 方框（淡灰色填充），箭头旁标 "AXI4 Master (128 b)"。AXI Interconnect 与 multicore_top 之间用粗箭头连接，标 "AXI4 (128 b)"。

**图注**：底部居中加一行小字 "Address map: Core[i] IFB = 0x8000_0000 + i × 0x1000_0000"，Times New Roman 8 pt。

**字体**：所有英文 Times New Roman 10 pt，标题加粗；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **overall system architecture of the accelerator**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (three tiers, top to bottom)**:
- **Top tier**: Place a Host CPU box (light-gray CMYK 0/0/0/10 fill, solid black border) at the upper-left, labeled "Host CPU (ARM / RISC-V)" in roughly 12 characters width.
- **Upper-middle tier**: Draw a horizontal long rectangle "AXI Interconnect" (light-blue CMYK 30/10/0/0 fill), about 10 characters wide. Connect Host CPU to AXI Interconnect with a thin black arrow downward, labeled "AXI-Lite (32 b, CSR)".
- **Center tier (focal point)**: Draw a large rounded rectangle "multicore_top" (light-yellow CMYK 0/10/30/0 fill, dark-gray border) with title "multicore_top (N = 1 / 2 / 4)" on top. Inside, arrange 4 accelerator-core sub-boxes in a **horizontal 2×2 matrix** (light-blue fill: Core 0 / Core 1 / Core 2 / Core 3), each containing two lines "16×16" and "INT8 MAC" in roughly 8 characters width. Connect the 4 cores with thin dashed lines representing "cross-core SRAM push (dash)", with a small caption below.
- **Bottom tier**: Below multicore_top draw a thick solid black arrow to the Off-chip DDR3 box (light-gray fill), labeled "AXI4 Master (128 b)". Connect AXI Interconnect to multicore_top with a thick arrow labeled "AXI4 (128 b)".

**Figure caption**: Center a small footnote at the bottom: "Address map: Core[i] IFB = 0x8000_0000 + i × 0x1000_0000", Times New Roman 8 pt.

**Typography**: All English in Times New Roman 10 pt with bold titles; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
