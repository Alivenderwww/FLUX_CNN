# Fig.1: FLUX_CNN Top-Level System Architecture

## 在 paper.md 中的引用位置
- **首次引入**：§1.3 Our Approach, line 44 (`[TBD: Fig.1 顶层框图占位 — 5 模块核流水 + DMA 子系统 + 多核 wrapper，§4.1 详述]`)
- **详述位置**：§4.1 Top-Level Architecture, lines 192–198
- **被引用次数**：仅 1 次显式 (`Figure 1` in §1.3)；§4.1 隐式承接

## 论证作用
让读者在第一页看到完整 FLUX_CNN system layout：5-stage 去中心化 valid-ready core pipeline + Xilinx axi_dm IP backed DMA subsystem + 可选 N-core wrapper，外部仅 1 个 AXI4 Master + 1 个 AXI-Lite Slave。这张图同时为 narrative A (compiler-side PE utilization) 和 narrative B (row-streaming + multi-core scaling) 提供 visual anchor。

## 图类型
**架构框图（block diagram）** — 多 hierarchy，含 core pipeline / DMA / AXI 互联 / multicore wrapper

## 设计要素

### 必含元素

**1. Core pipeline (5 模块串联，valid-ready 握手箭头)**
- `line_buffer` → `mac_array` (16×16 INT8) → `parf_accum` (内含 16×`parf_col`) → `ofb_writer`
- `wgt_buffer` 侧路供 WRF → `mac_array` (虚线表示 side-feed 而非 forward path)
- `sdp` 在 `parf_accum` 后 / `ofb_writer` 前（NVDLA 风格命名）
- `sequencer` 浮在 core pipeline 上方，仅做 cross-block kick-off（不是 central scheduler，要在图注或视觉权重上体现"轻"）
- `bias_rf` + `Shortcut Bank (8192×128 SRAM)` 接到 sdp（R.1+R.2 重构后位置）

**2. DMA subsystem**
- 三个轻量 controllers: `idma_ctrl` / `wdma_ctrl` / `odma_ctrl` (+ `rdma_ctrl` 可选画虚线，表示 R.2 残差读)
- Xilinx `axi_dm` IP（独立框，标注 "Vendor IP, AXI DataMover PG022"）
- `mm2s_arb` 在 idma_ctrl + wdma_ctrl 与 axi_dm.MM2S 之间
- `axi_dm.S2MM` 直接给 odma_ctrl
- `dfe`（descriptor fetch engine）独立模块

**3. AXI 互联层**
- `axi_m_mux` 聚合 axi_dm.MM2S + axi_dm.S2MM + dfe 到 1 个外部 master 口
- `axi_lite_csr` 暴露 1 个外部 slave 口
- `cfg_regs` 接到 axi_lite_csr 与 sequencer 之间（双端写口 csr_w / seq_w）

**4. 外部边界**
- 1 个 AXI4 Master（出 axi_m_mux）→ Off-chip DDR
- 1 个 AXI-Lite Slave（入 axi_lite_csr）← Host
- 这两条线要明显（粗箭头）以体现 "narrow SoC integration boundary"

**5. Multi-core wrapper（可选层，画在最外层虚线框）**
- `multicore_top.sv` 框住 N 个 core_top 实例
- Cores 之间用 `axi_2to3 / axi_4to5` crossbar 连接（标注"M2 跨核 SRAM 直送"）
- 每个 core 含 `ifb_axi_slave` 作为跨核 SI 写口
- 可在图右下角小尺寸示意 N=4 配置（4 个核 + 4to5 crossbar）

### 标注要求
- `mac_array`: "16×16 INT8 PE, 256 MAC"
- `parf_accum`: "16 × parf_col single-port BRAM"
- IFB SRAM: "8192×128 (32 BRAM)"
- WB SRAM: "1024×2048 (57 BRAM)"
- Shortcut Bank: "8192×128 (32 BRAM, R.2)"
- `mm2s_arb`: "serial arbitration"
- 外部接口： "AXI4 (BUS_DATA_W=128)" / "AXI-Lite (32-bit)"
- 多核 wrapper: "N=2/4 closes timing on XC7K325T (§7.3)"

### 视觉层次
- **主角**：5-stage core pipeline，居中，颜色最深 / 最大
- **配角**：DMA subsystem，居下方，中等灰度
- **背景**：multicore wrapper 用最外层虚线框，浅灰色填充，不抢主体视觉
- **箭头语义区分**：
  - 实线粗箭头 = forward data path (valid-ready)
  - 虚线 = side-feed (wgt_buffer→mac_array, residual→sdp)
  - 点划线 = 控制 / credit (rows_consumed credit from odma_ctrl→ofb_writer)

## 数据来源
- 模块拓扑：`CLAUDE.md` "项目总览" §1（core pipeline 5 模块）+ §2（DMA 子系统）
- BRAM 数字：`paper/workspace/contributions.md` §4.1 综合资源表 + STATUS.md §1
- 多核拓扑：`RTL/multicore_top.sv` + `RTL/AXI4/axi_arbiter.sv` + `memory/cross_core_design.md`
- 命名约定（`line_buffer → mac_array → parf_accum → sdp → ofb_writer`）：paper.md §4.1 line 196 (NVDLA-inspired)

## ASCII 示意稿

```
                      ┌────────────────────────────────────────┐
                      │  Host CPU                              │
                      │  (Linux / bare-metal)                  │
                      └──┬──────────────────────────────────┬──┘
                         │ AXI-Lite (32-bit)                │ irq (level)
                         ▼                                  │
   ┌─────────────────────────────────────────────────────────────────┐
   │ multicore_top  (N=2/4, dashed box, lighter shade)               │
   │                                                                 │
   │  ┌──────────── axi_lite_csr (slave port) ─────────────┐         │
   │  │             │                                       │         │
   │  │             ▼                                       ▼         │
   │  │      ┌──────────┐  csr_w   ┌──────────┐    ┌─────────────┐    │
   │  │      │ cfg_regs │◄────────│  dfe     │    │ desc_fifo   │    │
   │  │      │ (dual-W) │  seq_w  │ (descrip │    │  (32→128)   │    │
   │  │      └────┬─────┘  ◄──────│  fetch)  │    └─────────────┘    │
   │  │           │               └────┬─────┘                       │
   │  │           ▼ cfg                │ AXI-Lite write to cfg       │
   │  │      ┌──────────┐               │                             │
   │  │      │sequencer │ (lightweight kick-off only, NOT central)   │
   │  │      └────┬─────┘                                             │
   │  │           │ start                                             │
   │  │  ╔════════▼═══════════════════════════════════════════════╗   │
   │  │  ║   CORE PIPELINE  (5-stage, decentralized valid-ready)  ║   │
   │  │  ║                                                        ║   │
   │  │  ║   IFB ring                                             ║   │
   │  │  ║   (8192×128                                            ║   │
   │  │  ║    32 BRAM)                                            ║   │
   │  │  ║      │ valid/ready                                     ║   │
   │  │  ║      ▼                                                 ║   │
   │  │  ║  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐ ║   │
   │  │  ║  │ line_buffer ├───►│  mac_array  ├───►│ parf_accum  │ ║   │
   │  │  ║  │ (row ring,  │    │  16×16 PE   │    │ 16×parf_col │ ║   │
   │  │  ║  │  modulo     │    │  256 INT8   │    │ (single-port│ ║   │
   │  │  ║  │  strip_rows)│    │  MAC        │    │  BRAM each) │ ║   │
   │  │  ║  └─────────────┘    └──────▲──────┘    └──────┬──────┘ ║   │
   │  │  ║                            │                  │        ║   │
   │  │  ║                       ┌────┴─────┐            ▼        ║   │
   │  │  ║                       │wgt_buffer│      ┌──────────┐   ║   │
   │  │  ║                       │ (WB SRAM │      │   sdp    │   ║   │
   │  │  ║                       │ 1024×2048│      │ pipe→mult│◄──── bias_rf
   │  │  ║                       │  57 BRAM)│      │ →shift   │   ║   │
   │  │  ║                       └──────────┘      │ →+zp→ReLU│◄──── Shortcut
   │  │  ║                                         │ →clip    │   ║   │     Bank
   │  │  ║                                         │ →trunc   │   ║   │  (8192×128
   │  │  ║                                         └─────┬────┘   ║   │   32 BRAM)
   │  │  ║                                               │        ║   │
   │  │  ║                                          ┌────▼─────┐  ║   │
   │  │  ║                                          │ofb_writer│  ║   │
   │  │  ║                                          │ (OFB ring│  ║   │
   │  │  ║                                          │  7 BRAM  │  ║   │
   │  │  ║                                          │  +1 R18) │  ║   │
   │  │  ║                                          └────┬─────┘  ║   │
   │  │  ║                                               │        ║   │
   │  │  ║   ◄──────row credits (rows_consumed)──────────┤        ║   │
   │  │  ╚═══════════════════════════════════════════════╪════════╝   │
   │  │                                                  │            │
   │  │     ┌────────────────────────────────────────────┘            │
   │  │     │                                                          │
   │  │     ▼                                                          │
   │  │  ┌────────┐    ┌────────┐    ┌────────┐                       │
   │  │  │idma_ctl│    │wdma_ctl│    │odma_ctl│   (lightweight        │
   │  │  └───┬────┘    └───┬────┘    └───┬────┘    cmd gen + done)    │
   │  │      │             │              │                            │
   │  │      └──┬──────────┘              │                            │
   │  │         ▼                         │                            │
   │  │   ┌──────────┐                    │                            │
   │  │   │ mm2s_arb │ (serial)           │                            │
   │  │   └────┬─────┘                    │                            │
   │  │        ▼                          ▼                            │
   │  │  ╔══════════════════════════════════════════════════════════╗  │
   │  │  ║ axi_dm  (Xilinx AXI DataMover IP, PG022)                 ║  │
   │  │  ║   MM2S channel        S2MM channel                       ║  │
   │  │  ╚════════╤════════════════════╤════════════════════════════╝  │
   │  │           │                    │                                │
   │  │           ▼                    ▼                                │
   │  │  ┌─────────────────────────────────────────────────────────┐    │
   │  │  │             axi_m_mux  (aggregator)                     │    │
   │  │  │   MM2S + S2MM + DFE  →  1 external AXI4 Master          │    │
   │  │  └────────────────────────┬────────────────────────────────┘    │
   │  │                           │                                     │
   │  └───────────────────────────┘                                     │
   │   [N-core wrapper: in N=4 config, N copies of CORE PIPELINE        │
   │    above, each with own ifb_axi_slave, connected via axi_4to5      │
   │    crossbar for cross-core SRAM direct-feed]                       │
   └─────────────────────────────┬─────────────────────────────────────┘
                                 │ AXI4 (BUS_DATA_W=128)
                                 ▼
                      ┌──────────────────┐
                      │  Off-chip DDR    │
                      └──────────────────┘
```

## 初版代码（可选 — TikZ 推荐，因为复杂层级）

[TBD: 复杂多层架构图。建议用户用 PowerPoint / Visio / Inkscape 完成最终版；
TikZ 也可，但层级嵌套较多，pgfplots/positioning 写起来不简单。
matplotlib 不推荐 — 这种 hierarchical block diagram 用 networkx + matplotlib 会很丑。]

如果用户选择 TikZ：

```latex
% 推荐使用 tikz \usetikzlibrary{positioning,fit,backgrounds,arrows.meta}
% 框架结构：
% \begin{tikzpicture}[
%   block/.style={draw,minimum width=2.2cm,minimum height=0.9cm,fill=blue!10},
%   ipblock/.style={block,fill=orange!15},  % vendor IP
%   group/.style={draw,dashed,inner sep=8pt},
%   arr/.style={-Latex,thick}
% ]
%   \node[block] (lb) {line\_buffer};
%   \node[block,right=of lb] (mac) {mac\_array\\16$\times$16};
%   \node[block,right=of mac] (parf) {parf\_accum\\16$\times$parf\_col};
%   \node[block,right=of parf] (sdp) {sdp};
%   \node[block,right=of sdp] (ofb) {ofb\_writer};
%   \draw[arr] (lb)--(mac); \draw[arr] (mac)--(parf); ...
%   \begin{scope}[on background layer]
%     \node[group,fit=(lb)(mac)(parf)(sdp)(ofb),label=above:{Core Pipeline}] {};
%   \end{scope}
%   % DMA层、AXI 互联层 类似 fit + label 包起来
% \end{tikzpicture}
```

## 与正文的一致性检查

- [x] §1.3 line 44 提到的 "5-stage decentralized valid-ready pipeline (`line_buffer → wgt_buffer → mac_array → parf_accum → ofb_writer`)" — 图中顺序对齐
  - **注意**：§1.3 把 wgt_buffer 列入主链；§4.1 line 194 把 wgt_buffer "side-on into mac_array from the WRF rather than sitting in the main forward path"。**正文两处 phrasing 不一致**——polisher 阶段用户决定统一 phrasing；图中按 §4.1 的"side-feed"画
- [x] §4.1 line 192 "one AXI4 master + one AXI-Lite slave" — 外部边界图明示
- [x] §4.1 line 194 提到 idma_ctrl / wdma_ctrl / odma_ctrl + axi_dm + mm2s_arb + axi_m_mux + axi_lite_csr — 全部入图
- [x] §4.1 line 196 "naming of the core stages — line_buffer → mac_array → parf_accum → sdp" — 图采用此命名
- [x] §6.4 multicore wrapper — 图右下角小图示意 N=4 拓扑

## 不确定项 / 待用户决定

- [TBD: Fig.1 是否将 multicore wrapper 单独画一张子图（例如 Fig.1a 单核 + Fig.1b 多核拓扑），还是统一塞进一张大图。建议拆分以避免 IEEE 单栏排版下的可读性问题]
- [TBD: 是否画出 R.1 重构前 (bias 在 mac_array 内) 与重构后 (bias 在 sdp) 的对照——建议不画，避免读者困惑，只画当前最新架构]
- [TBD: rdma_ctrl（残差读 DMA）在图中虚线 vs 实线——R.2 已上线，建议实线]
- [CHECK: BRAM 数字 (32/57/32/7+1) 必须与 §7.3 line 396 表保持一致；polisher 阶段如果数字刷新需同步更新此图]

## 备注

复杂度高的图，建议用户用 PowerPoint / Visio 出 .pptx 源文件 + 导出 PDF，方便后续 reviewer 修改。
