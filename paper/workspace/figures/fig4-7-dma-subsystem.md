# 图 4.7 DMA 子系统结构图
# Figure 4.7 DMA subsystem structure

## 在论文中的角色
- 首次引入：§4.10 DMA 子系统与 AXI 接口集成（paper.md L406 / L410）
- 引用位置：paper.md L406 / L410 / §4.12 本章小结
- 论证作用：展示 DMA 子系统由 1 个 Xilinx axi_dm IP（提供 MM2S/S2MM 两通道）+ 4 类自研控制器（idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl）+ mm2s_arb（轮转仲裁 MM2S）+ axi_m_mux（聚合 MM2S/S2MM/DFE 主口为 1 个 AXI4 主接口）+ axi_lite_csr（暴露 4 个启动期寄存器）+ DFE（描述符获取引擎，自动按类型分发到 4 个控制器与 sequencer）组成。强调"对外仅 1 个 AXI4 主 + 1 个 AXI-Lite 从"的简洁集成接口与"控制核仅写 4 寄存器即可启动整个多层卷积链"的关键收益。

## 图类型
3 层分层架构图（上层 — 控制接口与描述符通路；中层 — 4 类自研控制器；下层 — IP / 仲裁 / 聚合 / 对外口），整体接近 1:1 正方形。布局参考 fig3-3 风格但更详细。

## 设计要素

### 必含元素

#### 上层 — 控制接口与描述符通路
1. **axi_lite_csr（左上）**：方块标 "axi_lite_csr (4 start regs + status)"，对外接 AXI-Lite Slave 端口（细线朝上标 "AXI-Lite ← Control Core"）。
2. **DFE（右上）**：方块标 "DFE (Descriptor Fetch Engine, auto-fetch chain)"，由 axi_lite_csr 的 start_dfe 触发（细虚线箭头从 csr → DFE，标 "start_dfe"）。
3. **DFE → desc 分发**：DFE 下方画 5 路细箭头分别标 "TYPE_IDMA / TYPE_WDMA / TYPE_RDMA / TYPE_ODMA / TYPE_CFG"，分别接 4 个 dma_ctrl 与 sequencer。

#### 中层 — 4 类自研 DMA 控制器
4. **idma_ctrl（中左 1）**：方块标 "idma_ctrl (DDR → IFB)"，淡蓝色填充。
5. **wdma_ctrl（中左 2）**：方块标 "wdma_ctrl (DDR → WB)"，淡蓝色填充。
6. **rdma_ctrl（中左 3）**：方块标 "rdma_ctrl (DDR → bias_rf / Shortcut)"，淡蓝色填充。
7. **odma_ctrl（中右）**：方块标 "odma_ctrl (OFB → DDR / cross-core IFB)"，淡橙色填充（区分 S2MM 通道）。
8. **sequencer（中最右）**：方块标 "sequencer (CFG_WRITE → cfg_regs)"，淡灰色填充（配角，标"已在图 4.6 详述"）。

#### 下层 — axi_dm IP / 仲裁 / 聚合 / 对外口
9. **mm2s_arb（下层左）**：方块标 "mm2s_arb (round-robin)"，3 个输入分别来自 idma_ctrl / wdma_ctrl / rdma_ctrl 命令口，1 个输出接 axi_dm IP 的 MM2S 命令端。
10. **axi_dm IP（下层中）**：双格方块标 "axi_dm IP (Xilinx, Vivado 2023.1)"，内部分两格："MM2S channel" 与 "S2MM channel"。
11. **axi_m_mux（下层右）**：方块标 "axi_m_mux (3-to-1 master aggregator)"，3 个输入分别接 axi_dm.MM2S 主口 / axi_dm.S2MM 主口 / DFE 主口，1 个输出接对外 AXI4 Master 端口。
12. **对外 AXI4 主口（最下方）**：粗箭头朝下标 "AXI4 Master → DDR (via SoC interconnect)"。

#### 数据流标注
13. **MM2S 数据流**：axi_dm.MM2S → idma_ctrl → IFB ；axi_dm.MM2S → wdma_ctrl → WB ；axi_dm.MM2S → rdma_ctrl → bias_rf/Shortcut（细数据流箭头从下方回到中层）。
14. **S2MM 数据流**：odma_ctrl → axi_dm.S2MM → DDR/cross-core IFB（细数据流箭头从中层进入下方 axi_dm.S2MM）。
15. **关键标注（右侧侧栏）**：
    - "MM2S 通道：1 路，3 个 ctrl 共享，mm2s_arb 轮转仲裁"
    - "S2MM 通道：1 路，odma_ctrl 独占，无需仲裁"
    - "对外接口：仅 1 AXI4 Master + 1 AXI-Lite Slave"

### 标注要求
- axi_dm IP 来源：Xilinx，Vivado 2023.1 IP 集成生成
- mm2s_arb 仲裁策略：轮转（round-robin），每段 burst 完成后切换
- desc_fifo 容量：128（与图 4.6 一致）
- 描述符类型 5 类：TYPE_IDMA / WDMA / RDMA / ODMA / CFG
- axi_m_mux 聚合：3 路（MM2S / S2MM / DFE）→ 1 路对外 AXI4 主

### 视觉层次
- 主角：4 类 DMA 控制器（中层）+ axi_dm IP / mm2s_arb / axi_m_mux（下层）
- 配角：DFE / axi_lite_csr（上层）、sequencer（淡灰色）
- 背景：对外 AXI4 / AXI-Lite 端口（细黑线指出方块外）

## 数据来源
- paper.md §4.10
- docs/modules/idma_ctrl.md / wdma_ctrl.md / odma_ctrl.md / mm2s_arb.md / axi_m_mux.md / dfe.md / axi_lite_csr.md（部分）
- contributions.md（DMA 子系统简洁集成接口贡献）
- CLAUDE.md（DMA 子系统模块清单）

## ASCII 示意稿

```
   ┌────────────────────────────────────────────────────────────────────────┐
   │  AXI-Lite ← Control Core                                              │
   │     │                                                                  │
   │     ▼                                                                  │
   │  ┌──────────────────┐    start_dfe     ┌─────────────────────────────┐│
   │  │ axi_lite_csr     │ ─ ─ ─ ─ ─ ─ ─ ─▶│ DFE (Descriptor Fetch       ││
   │  │ (4 start regs +  │                  │      Engine, auto-fetch)    ││
   │  │  status)         │                  └─────────────┬───────────────┘│
   │  └──────────────────┘                                │ desc           │
   │                              ┌────────┬────────┬─────┴─────┬────────┐│
   │                              │TYPE_IDMA│TYPE_WDMA│TYPE_RDMA│TYPE_ODMA││
   │                              │         │         │         │TYPE_CFG ││
   │                              ▼         ▼         ▼         ▼         ││
   │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ ││
   │  │idma_ctrl │  │wdma_ctrl │  │rdma_ctrl │  │odma_ctrl │  │sequencer│ ││
   │  │DDR → IFB │  │DDR → WB  │  │DDR →bias/│  │OFB→DDR or│  │CFG_WRITE│ ││
   │  │          │  │          │  │ Shortcut │  │ x-core   │  │→cfg_regs│ ││
   │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘  │(see 4.6)│ ││
   │       │ cmd         │ cmd         │ cmd         │ cmd     └─────────┘ ││
   │       └──────┬──────┴─────────────┘             │                    ││
   │              ▼                                  │                    ││
   │       ┌──────────────┐                          │                    ││
   │       │  mm2s_arb    │ (round-robin)            │                    ││
   │       │  3-to-1      │                          │                    ││
   │       └──────┬───────┘                          │                    ││
   │              ▼                                  ▼                    ││
   │       ┌─────────────────────────────────────────────┐                ││
   │       │  axi_dm IP (Xilinx, Vivado 2023.1)          │                ││
   │       │  ┌──────────────┐   ┌──────────────────┐    │                ││
   │       │  │ MM2S channel │   │  S2MM channel    │    │                ││
   │       │  └──────┬───────┘   └────────▲─────────┘    │                ││
   │       └─────────┼────────────────────┼──────────────┘                ││
   │  data: ─────────┼────────────────────┼────────────────────────────── ││
   │                 ▼                    │                               ││
   │       (back to idma/wdma/rdma)       │                               ││
   │                                                                      ││
   │              ┌──────────────────────┐  DFE master                    ││
   │              │  axi_m_mux           │ ◀─── (3-rd input)              ││
   │              │  (3-to-1 aggregator) │ ◀─── axi_dm.MM2S master        ││
   │              │                      │ ◀─── axi_dm.S2MM master        ││
   │              └──────────┬───────────┘                                ││
   │                         ▼                                            ││
   │           AXI4 Master → DDR (via SoC interconnect)                   ││
   └────────────────────────────────────────────────────────────────────────┘

   key notes:
   - MM2S: 1 channel shared by 3 ctrls (idma/wdma/rdma), mm2s_arb round-robin
   - S2MM: 1 channel exclusive to odma_ctrl, no arbitration
   - external interface: 1 AXI4 Master + 1 AXI-Lite Slave only
```

## 与正文的一致性检查
- [x] §4.10 "1 个 Xilinx axi_dm IP（Vivado 2023.1），MM2S + S2MM 两通道" — 下层中央方块标 "axi_dm IP (Xilinx, Vivado 2023.1)" 内分 2 格
- [x] §4.10 "4 类自研控制器：idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl" — 中层 4 个方块齐全
- [x] §4.10 "idma_ctrl 与 wdma_ctrl 通过 mm2s_arb 串行共享 MM2S；rdma_ctrl 也参与 MM2S 竞争" — mm2s_arb 标 "3-to-1"，3 个输入对应
- [x] §4.10 "S2MM 由 odma_ctrl 独占，无需仲裁" — odma_ctrl 直接接 axi_dm.S2MM，旁注说明
- [x] §4.10 "axi_m_mux 把 axi_dm 的 MM2S/S2MM 主口与 DFE 主口三路聚合" — axi_m_mux 标 "3-to-1 aggregator"，3 个输入注明
- [x] §4.10 "axi_lite_csr 暴露 4 个启动期寄存器 + start_dfe" — axi_lite_csr 方块标 "4 start regs + status"，start_dfe 虚线
- [x] §4.10 "DFE 自动从 DDR 拉描述符，按类型分发 5 类（IDMA/WDMA/RDMA/ODMA/CFG）" — DFE 下方 5 路分发
- [x] §4.10 "对外仅 1 AXI4 主 + 1 AXI-Lite 从" — 最下方对外口 + 最上方 AXI-Lite 端口

## 不确定项
- [TBD: ifb_axi_slave 模块（多核 cross-core SRAM push 用）是否在本图出现 — 当前只画"对外 AXI4 主口去 DDR"主路径，cross-core push 路径在 fig4-8 多核扩展层图中体现，本图 odma_ctrl 标 "OFB → DDR or x-core" 已暗示]
- [CHECK: rdma_ctrl 在单核 vs 多核场景下的具体接入位置 — 按 §4.10 "rdma_ctrl 在多核扩展层中也参与 MM2S 通道竞争" 解读，单核场景下也存在但优先级较低，按"参与 mm2s_arb"画]

## image 生成提示词

### 中文版

科研论文配图，**DMA 子系统结构图**，IEEE 期刊配色风格，白底黑字，禁用任何彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。强制 3 层布局（上层控制 + 中层 4 类控制器 + 下层 IP/仲裁/聚合），避免横向元素过多导致整体过宽。**

**版式（3 层分层架构）**：
- **主框（圆角矩形大框）**：标题 "DMA Subsystem"（淡黄色 CMYK 0/10/30/0 填充、深灰色边框）。
- **上层 — 控制接口（占顶部约 25% 高度）**：左右二分
  - 左侧 "axi_lite_csr (4 start regs + status)"，淡灰色 CMYK 0/0/0/15 填充小方框；顶部一根细线朝上标 "AXI-Lite ← Control Core"
  - 右侧 "DFE (Descriptor Fetch Engine, auto-fetch chain)"，淡灰色填充方框
  - 中间一根细虚线箭头从 axi_lite_csr 指向 DFE，标 "start_dfe"
  - DFE 下方分出 5 路细黑色实线箭头朝下，分别标 "TYPE_IDMA / TYPE_WDMA / TYPE_RDMA / TYPE_ODMA / TYPE_CFG"
- **中层 — 4 类 DMA 控制器 + sequencer（占中部约 35% 高度）**：横向 5 个方框
  1. "idma_ctrl (DDR → IFB)" 淡蓝色 CMYK 30/10/0/0
  2. "wdma_ctrl (DDR → WB)" 淡蓝色
  3. "rdma_ctrl (DDR → bias_rf / Shortcut)" 淡蓝色
  4. "odma_ctrl (OFB → DDR or x-core IFB)" 淡橙色 CMYK 0/30/30/0（区分 S2MM 通道）
  5. "sequencer (CFG_WRITE → cfg_regs)" 淡灰色，下方小字 "(see Fig. 4.6)"
  - DFE 5 路分发箭头分别接入这 5 个方框顶部
- **下层 — axi_dm IP / mm2s_arb / axi_m_mux（占底部约 40% 高度）**：从左到右
  - 左侧 "mm2s_arb (round-robin, 3-to-1)" 淡绿色 CMYK 30/0/20/0 方框；顶部 3 根细线分别从 idma_ctrl / wdma_ctrl / rdma_ctrl 命令口接入
  - 中央 "axi_dm IP (Xilinx, Vivado 2023.1)" 大方框（深灰色边框、白色填充），内部水平分两格："MM2S channel" 与 "S2MM channel"；mm2s_arb 输出箭头进入 MM2S 端，odma_ctrl 输出箭头进入 S2MM 端
  - 右侧 "axi_m_mux (3-to-1 master aggregator)" 淡绿色方框；3 个输入分别接 axi_dm.MM2S master / axi_dm.S2MM master / DFE master（DFE 经一根长走线从右上角连下来）
  - 底部一根粗黑色实线箭头从 axi_m_mux 朝下，标 "AXI4 Master → DDR (via SoC interconnect)"
- **右侧侧栏 key notes**（小字）：
  - "MM2S: 1 channel, shared by 3 ctrls (idma/wdma/rdma), mm2s_arb round-robin"
  - "S2MM: 1 channel, exclusive to odma_ctrl, no arbitration"
  - "external: 1 AXI4 Master + 1 AXI-Lite Slave only"

**字体**：所有英文 Times New Roman 10 pt，模块标题加粗，TYPE_xxx 描述符类型标签用斜体；中文（如有）思源黑体 10 pt。**禁止**手写体、彩色渐变、阴影、3D 立体、图标、卡通元素。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **DMA subsystem structure**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure. Enforce 3-layer layout (top: control interface; middle: 4 DMA controllers; bottom: IP/arbiter/aggregator) to avoid an overly wide figure.**

**Layout (3-layer hierarchical architecture)**:
- **Main frame (large rounded rectangle)**: Titled "DMA Subsystem" (light-yellow CMYK 0/10/30/0 fill, dark-gray border).
- **Top layer — control interface (~25% height)**: Left-right split
  - Left "axi_lite_csr (4 start regs + status)" small box with light-gray CMYK 0/0/0/15 fill; from top a thin line going up labeled "AXI-Lite ← Control Core"
  - Right "DFE (Descriptor Fetch Engine, auto-fetch chain)" light-gray fill box
  - Between them a thin dashed arrow from axi_lite_csr to DFE labeled "start_dfe"
  - From DFE bottom, 5 thin black solid arrows fan out downward, labeled "TYPE_IDMA / TYPE_WDMA / TYPE_RDMA / TYPE_ODMA / TYPE_CFG"
- **Middle layer — 4 DMA controllers + sequencer (~35% height)**: 5 boxes horizontally
  1. "idma_ctrl (DDR → IFB)" light-blue CMYK 30/10/0/0
  2. "wdma_ctrl (DDR → WB)" light-blue
  3. "rdma_ctrl (DDR → bias_rf / Shortcut)" light-blue
  4. "odma_ctrl (OFB → DDR or x-core IFB)" light-orange CMYK 0/30/30/0 (distinguishing S2MM channel)
  5. "sequencer (CFG_WRITE → cfg_regs)" light-gray, with small caption "(see Fig. 4.6)" below
  - DFE 5-way fan-out arrows connect into the tops of these 5 boxes
- **Bottom layer — axi_dm IP / mm2s_arb / axi_m_mux (~40% height)**: Left to right
  - Left "mm2s_arb (round-robin, 3-to-1)" light-green CMYK 30/0/20/0 box; 3 thin lines on top connecting from idma_ctrl / wdma_ctrl / rdma_ctrl command ports
  - Center "axi_dm IP (Xilinx, Vivado 2023.1)" large box (dark-gray border, white fill), internally split horizontally into two cells: "MM2S channel" and "S2MM channel"; mm2s_arb output arrow enters MM2S, odma_ctrl output arrow enters S2MM
  - Right "axi_m_mux (3-to-1 master aggregator)" light-green box; 3 inputs from axi_dm.MM2S master / axi_dm.S2MM master / DFE master (DFE routed via a long wire from upper-right corner)
  - Bottom thick black solid arrow from axi_m_mux going down, labeled "AXI4 Master → DDR (via SoC interconnect)"
- **Right-side key notes (small text)**:
  - "MM2S: 1 channel, shared by 3 ctrls (idma/wdma/rdma), mm2s_arb round-robin"
  - "S2MM: 1 channel, exclusive to odma_ctrl, no arbitration"
  - "external: 1 AXI4 Master + 1 AXI-Lite Slave only"

**Typography**: All English in Times New Roman 10 pt with bold module titles; TYPE_xxx descriptor labels in italics; Chinese (if any) in Source Han Sans 10 pt. **Strictly forbidden**: handwritten fonts, color gradients, drop shadows, 3D effects, icons, cartoon elements. Maintain rigorous engineering-paper aesthetics throughout.
