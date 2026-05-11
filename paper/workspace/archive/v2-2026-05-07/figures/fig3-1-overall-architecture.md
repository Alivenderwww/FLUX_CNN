# 图 3.1 加速器两层总体架构图
# Figure 3.1 Overall two-tier architecture of the accelerator

## 在论文中的角色
- 首次引入：§3.3 段 "系统采用两层架构——核流水（5 模块 + 共享 cfg_regs）+ DMA 子系统..." [依赖: Fig.3.1]
- 论证作用：全文最重要的架构总览图。让读者一眼看清两层架构（核流水 vs DMA 子系统）+ 外部接口最简（1 AXI4 主口 + 1 AXI-Lite 从口）+ 各模块的位置关系。本图是后续所有详细图（Fig.4.1-4.5、Fig.4.10）的总览。

## 图类型
分层框图：上层为核流水（5 模块 + cfg_regs），下层为 DMA 子系统（控制器 + IP + 仲裁）；外部对外仅暴露左侧 1 AXI4 主口 + 右侧 1 AXI-Lite 从口。

## 设计要素

### 必含元素
- **顶部边界（外部接口）**：上方一条粗线，左端标 "AXI4 Master to DDR" 接到 multicore_top 外面的 DDR3/DDR4 控制器；右端标 "AXI-Lite Slave from Host" 接到上位 SoC
- **核流水层（上半区）**：
  - 5 个矩形模块，从左到右：`line_buffer` → `mac_array (16×16 OS+列广播)` → `parf_accum (16 列 PARF)` → `ofb_writer (含 SDP)`
  - 侧路：`wgt_buffer` 在 mac_array 上方，向 mac_array 提供权重
  - 中央放一个 `cfg_regs (共享)` 浮于五模块之上，向所有 5 模块画虚线连接（表示配置寄存器广播）
  - 模块之间用粗箭头连，标 valid-ready 握手（小标 "v/r"）
- **DMA 子系统层（下半区）**：
  - 三套自研控制器：`idma_ctrl` / `wdma_ctrl` / `odma_ctrl`
  - `mm2s_arb` 仲裁器在 idma_ctrl 与 wdma_ctrl 之间，向上聚合到 axi_dm
  - `axi_dm IP（Vivado）` 块（标 "Vivado IP"）含 MM2S / S2MM 两通道
  - `axi_m_mux` 主口聚合器，把 axi_dm.MM2S/S2MM + DFE 的 descriptor 拉取请求聚合到一条 AXI4 主口
  - `axi_lite_csr` + `DFE (Descriptor Front End)` 配置接口
  - cfg_regs 双口：一口接 axi_lite_csr（host 直写），一口接 DFE（descriptor 批量更新）
- **垂直连接**：
  - line_buffer ← idma_ctrl（IFB 加载）
  - wgt_buffer ← wdma_ctrl（权重加载）
  - ofb_writer → odma_ctrl（输出回写）

### 标注要求
- 模块标 "5 模块去中心化 valid-ready 流水" 横向跨 5 模块上方
- DMA 层标 "DMA 子系统：1 AXI4 主口对外"
- 侧标 "外部仅暴露 1 AXI4 + 1 AXI-Lite，集成代价最小"
- mm2s_arb 旁加注 "串行仲裁，避免 IFB / WB 竞争"
- 用颜色区分：核流水（蓝）、DMA 控制器（绿）、Vivado IP（橙）、cfg_regs（紫）、外部接口（黑）

### 视觉层次
- 主角：5 模块核流水（最上一行，最大）
- 配角：DMA 子系统（下半区）
- 背景：cfg_regs / 外部接口标签

## ASCII 示意稿

```
                 外部 ◀──── AXI4 Master ───▶            AXI-Lite Slave ◀── Host SoC
                                │                              │
   ════════════════════════════ │ ════════════════════════════ │ ═════════════
                                │                              │
                          ┌─────┴────────┐              ┌──────┴──────┐
                          │   axi_m_mux  │              │ axi_lite_csr│
                          └─┬───┬────┬───┘              └──────┬──────┘
                            │   │    │                         │
                          ┌─┴───┴─┐  │                         │
                          │ axi_dm │  │ (DFE descriptor 拉取)   │
                          │ (Vivado│  └──────┐                  │
                          │  IP)   │         │                  │
                          │ MM2S   │         ▼                  │
                          │  S2MM  │  ┌──────────┐              │
                          └─┬─┬────┘  │   DFE    │              │
                       MM2S│ │S2MM    │ (desc FE)│              │
                            │ │       └────┬─────┘              │
                       ┌────┴─┴───┐        │                    │
                       │ mm2s_arb │        │                    │
                       └──┬───┬───┘        │                    │
              wdma_ctrl ──┘   └── idma_ctrl │                   │
                  │              │          │                   │
                  ▼              ▼          ▼                   ▼
            ┌────────┐    ┌────────────┐   ┌──────────────────────┐
            │wgt_buf │    │ line_buffer│   │     cfg_regs         │
            │(WB)    │    │  (IFB ring)│   │  (双口: csr + DFE)   │
            └────┬───┘    └──────┬─────┘   └──────────────────────┘
                 │               │              ↓ 配置广播至 5 模块
                 │               ▼                              ↓
                 │         ┌──────────────┐                     ↓
                 └────────▶│  mac_array   │── v/r ──┐           ↓
                           │  16×16 INT8  │         │           ↓
                           │  OS+列广播   │         ▼           ↓
                           └──────────────┘   ┌──────────┐      ↓
                                              │parf_accum│──v/r─┘
                                              │ 16×PARF  │
                                              └────┬─────┘
                                                   ▼
                                              ┌──────────┐
                                              │ ofb_writer│
                                              │ +SDP 后处理│
                                              └────┬─────┘
                                                   ▼
                                              odma_ctrl ──▶ axi_dm.S2MM
                                              (S2MM 独占通道)

   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   上层：核流水 5 模块 + cfg_regs 共享配置（去中心化 valid-ready）
   下层：DMA 子系统（3 控制器 + Vivado axi_dm IP + mm2s_arb + axi_m_mux）
```

## 数据来源
- paper.md §3.3 / §4.4.3
- contributions.md C1.1 / C1.2 / C3.1 / C3.2 / C3.3
- CLAUDE.md 项目总览 "顶层结构两层"
- docs/modules/core_top.md / dfe.md（细节回查）

## 与正文一致性检查
- [x] 5 模块顺序（line_buffer → mac_array → parf_accum → ofb_writer + 侧路 wgt_buffer）与 §3.3 / §4.2 一致
- [x] DMA 子系统组件（idma/wdma/odma_ctrl、mm2s_arb、axi_dm、axi_m_mux、axi_lite_csr、DFE）与 §3.3 / §4.4.3 一致
- [x] 外部接口（1 AXI4 主口 + 1 AXI-Lite 从口）与 §3.3 / §4.5 一致
- [x] cfg_regs 双口（csr + DFE）与 §4.4 / contributions.md C3.3 一致

## 不确定项
无。本图基于已实现的 RTL（commit 5fe16b2）。
