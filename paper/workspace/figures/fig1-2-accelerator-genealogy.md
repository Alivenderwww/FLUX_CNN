# 图 1.2 CNN 加速器演进谱系树
# Figure 1.2 CNN accelerator evolution genealogy

## 在论文中的角色
- 首次引入：§1.2 国内外相关研究工作进展（paper.md L100-L145）
- 引用位置：§1.2.1 端侧 CNN 硬件加速器架构研究进展
- 论证作用：用一张时间轴谱系树把 2014→2024 这十年代表性 CNN 加速器按 ASIC / FPGA / Hybrid 三大分支组织起来，让读者在阅读 §1.2 大段文献综述之前先获得"地图视角"，并直观看到 FLUX_CNN 在 2024 端侧节点的位置与最近邻工作（NVDLA、Gemmini、Peng et al.）。

## 图类型
横向时间轴谱系树（三大分支 + 节点 + 本工作高亮）。

## 设计要素

### 必含元素
1. **横向时间轴**：底部画一条横向时间线，标 2014 / 2016 / 2018 / 2020 / 2022 / 2024 六个刻度。
2. **三大分支（自上而下三条水平泳道）**：
   - **ASIC 分支（顶部）**：DianNao (2014) → Eyeriss (2017) → TPU v1 (2017) → Eyeriss v2 (2019) → NVDLA (2018) → Simba (2019)
   - **FPGA 分支（中部）**：VTA (2019) → OPU (2020) → Gemmini (2021) → DNNBuilder (2018) → DeepFire2 (2023) → Peng et al. (2022)
   - **Hybrid / 软硬协同分支（底部）**：MAERI (2018) → SIGMA (2020) → Fused-Layer (2016) → NN-Baton (2021)
3. **本工作节点（高亮）**：在 2024 末端、FPGA 分支位置高亮 "FLUX_CNN (本工作)"，使用深金色 #b8860b 填充 + 加粗黑色边框 + 一颗五角星标记。
4. **代际承袭箭头**：节点之间用细灰色箭头表示思想承袭（如 Eyeriss → Eyeriss v2、TPU → Simba），跨分支引用用虚线箭头（如 VTA → NVDLA）。
5. **图例**：右下角小框写"● ASIC / ▲ FPGA / ◆ Hybrid / ★ 本工作"。

### 标注要求
- 每节点紧贴名称下方一行年份（如 "DianNao 2014"）
- 关键节点（DianNao / Eyeriss / TPU v1 / NVDLA / Gemmini）字号略大
- FLUX_CNN 节点旁加一行小字："端侧流式 + 分层调度"
- 时间轴下方标"年份 / Year"

### 视觉层次
- 主角：FLUX_CNN 高亮节点（深金色 + 五角星）
- 配角：三大分支泳道与代表节点
- 背景：时间轴刻度线（淡灰色虚线，每 2 年一格）

## 数据来源
- paper.md §1.2.1 文献综述段落 L102-L108（DianNao、Eyeriss、TPU、NVDLA、Simba、VTA、OPU、Gemmini、DNNBuilder、DeepFire2、Peng et al.）
- paper.md §1.2.2 软硬协同段落 L131-L141（VTA、MAERI、SIGMA、Fused-Layer、NN-Baton）
- 文献 [11]-[36] 全部 prior art 条目
- contributions.md §一句话定位（FLUX_CNN 在 streaming 谱系中的位置）

## ASCII 示意稿

```
   ┌─ ASIC ──────────────────────────────────────────────────────────┐
   │ DianNao      Eyeriss   TPU v1   NVDLA      Eyeriss v2  Simba    │
   │  ●─────────────●─────●─────────●─────────●─────────●            │
   │ 2014           2017  2017      2018       2019      2019         │
   │                                                                  │
   ├─ FPGA ──────────────────────────────────────────────────────────┤
   │       DNNBuilder            VTA   OPU   Gemmini    Peng  DeepFire│
   │        ▲────────────────────▲────▲─────▲──────────▲─────▲       │
   │       2018                  2019 2020  2021       2022  2023     │
   │                                                              ★   │
   │                                                       FLUX_CNN   │
   │                                                       2024（本）│
   ├─ Hybrid / 协同 ────────────────────────────────────────────────┤
   │ Fused-Layer        MAERI    SIGMA           NN-Baton             │
   │  ◆──────────────────◆────────◆─────────────◆                    │
   │ 2016               2018     2020           2021                  │
   └────────────────────────────────────────────────────────────────┘
   ─────┬───────┬───────┬───────┬───────┬───────┬───────▶ 年份 Year
       2014    2016    2018    2020    2022    2024

   图例：● ASIC   ▲ FPGA   ◆ Hybrid / 协同   ★ 本工作
```

## 与正文的一致性检查
- [x] §1.2.1 "寒武纪团队提出的 DianNao 系列首开先河" — ASIC 分支起点 2014
- [x] §1.2.1 "Eyeriss 在数据流层面做出了奠基性贡献" — ASIC 分支 2017
- [x] §1.2.1 "NVIDIA 的 NVDLA 作为开源参考设计" — ASIC 分支 2018
- [x] §1.2.1 "Gemmini ... 完整的 SoC 集成方案" — FPGA 分支 2021
- [x] §1.2.2 "MAERI 通过可重构 mesh 互联" — Hybrid 分支 2018
- [x] §1.2.2 "Simba 采用 36 芯片多核 mesh 架构" — ASIC 分支 2019
- [x] 表 1.1 中的所有节点（DianNao、TPU v1、Eyeriss、NVDLA、Eyeriss v2、VTA、Simba、OPU、Gemmini、Peng et al.）在谱系树中都有对应位置

## 不确定项
- [TBD: 是否在每节点旁加引用编号（如 [11]）] — 倾向加，便于读者交叉查询文献表
- [TBD: 是否在 Hybrid 分支再细分 dataflow-reconfigurable 与 layer-fusion 两子分支] — 倾向不细分，保持图的清晰度，正文已细分叙述

## image 生成提示词

### 中文版

科研论文配图，**CNN 加速器演进谱系树**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。** 由于内容横向跨度大，可适度采用 1.2:1 横向以容纳时间轴。

**版式（横向时间轴 + 三大泳道分支）**：
- **底部时间轴**：画面下方画一条水平实线（深灰 #404040），标 6 个刻度 2014 / 2016 / 2018 / 2020 / 2022 / 2024，刻度旁小字 Times New Roman 9 pt。每个刻度向上引一条淡灰色 CMYK 0/0/0/10 细虚线作为参考栅格。
- **三大水平泳道**（自上而下，每条占画面 1/3 高度）：
  - **顶部 ASIC 分支**：水平淡蓝色 CMYK 30/10/0/0 背景条，左侧标题"ASIC"（思源黑体 10 pt 加粗）。从左至右依次摆放圆形节点：DianNao (2014)、Eyeriss (2017)、TPU v1 (2017)、NVDLA (2018)、Eyeriss v2 (2019)、Simba (2019)。每节点为深蓝 #1f4e79 实心圆 + 名称下方年份。节点之间用细灰色实线箭头连接（表示思想承袭）。
  - **中部 FPGA 分支**：水平淡黄色 CMYK 0/10/30/0 背景条，左侧标题"FPGA"。节点：DNNBuilder (2018)、VTA (2019)、OPU (2020)、Gemmini (2021)、Peng et al. (2022)、DeepFire2 (2023)。节点形状用深绿 #2e7d32 实心三角。在 2024 末端高亮放置 **FLUX_CNN（本工作）** 节点：深金色 #b8860b 五角星 + 实心圆叠加，圆周加粗黑色边框，旁边小字"端侧流式 + 分层调度"。
  - **底部 Hybrid 分支**：水平淡灰色 CMYK 0/0/0/10 背景条，左侧标题"Hybrid / 软硬协同"。节点：Fused-Layer (2016)、MAERI (2018)、SIGMA (2020)、NN-Baton (2021)。节点形状用学术深灰 #404040 实心菱形。
- **跨分支虚线**：在 VTA(FPGA) → NVDLA(ASIC) 等思想跨界节点之间，用细灰色虚线箭头表示跨分支引用关系（仅 2-3 条最关键的）。
- **图例**：画面右下角放一个小矩形框（淡灰色填充），内含 4 行小字："● ASIC""▲ FPGA""◆ Hybrid""★ 本工作"，Times New Roman 8 pt。

**字体**：所有英文 Times New Roman 10 pt，节点名加粗；中文（如有）思源黑体 10 pt。**严肃学术风格，IEEE 期刊配色，禁止卡通**：禁止手写体、彩色渐变、阴影、3D 立体、图标、emoji、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **CNN accelerator evolution genealogy tree**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **Overall aspect ratio approximately 1:1 (square) or 1:1.2 (slightly taller). A 1.2:1 horizontal layout is acceptable to accommodate the time axis.**

**Layout (horizontal timeline + three swim-lane branches)**:
- **Bottom timeline**: A horizontal solid dark-gray #404040 line at the bottom with 6 tick marks at 2014 / 2016 / 2018 / 2020 / 2022 / 2024 (Times New Roman 9 pt). Light-gray CMYK 0/0/0/10 dashed vertical grid lines rise from each tick.
- **Three horizontal swim-lanes** (top to bottom, each ~1/3 of canvas height):
  - **Top ASIC lane**: Light-blue CMYK 30/10/0/0 background strip; left-side title "ASIC" (Source Han Sans 10 pt bold). Place circular nodes left-to-right: DianNao (2014), Eyeriss (2017), TPU v1 (2017), NVDLA (2018), Eyeriss v2 (2019), Simba (2019). Each node is a deep-blue #1f4e79 filled circle with name and year. Connect consecutive nodes with thin gray solid arrows representing idea lineage.
  - **Middle FPGA lane**: Light-yellow CMYK 0/10/30/0 background strip; left-side title "FPGA". Nodes: DNNBuilder (2018), VTA (2019), OPU (2020), Gemmini (2021), Peng et al. (2022), DeepFire2 (2023). Use deep-green #2e7d32 filled triangles. At the 2024 end, highlight the **FLUX_CNN (this work)** node: deep-gold #b8860b filled star overlaid on a filled circle with a bold black border; small caption "edge streaming + hierarchical scheduling".
  - **Bottom Hybrid lane**: Light-gray CMYK 0/0/0/10 background strip; left-side title "Hybrid / SW-HW co-design". Nodes: Fused-Layer (2016), MAERI (2018), SIGMA (2020), NN-Baton (2021). Use dark-gray #404040 filled diamonds.
- **Cross-branch dashed arrows**: 2-3 thin gray dashed arrows for the most important cross-branch idea flows (e.g., VTA → NVDLA influence).
- **Legend**: A small box at the lower-right with light-gray fill, containing 4 lines: "● ASIC", "▲ FPGA", "◆ Hybrid", "★ This work" in Times New Roman 8 pt.

**Typography**: All English in Times New Roman 10 pt with bold node names; Chinese (if any) in Source Han Sans 10 pt. **Serious academic style, IEEE palette, no cartoon**: strictly forbid handwritten fonts, color gradients, drop shadows, 3D effects, icons, emoji, or cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
