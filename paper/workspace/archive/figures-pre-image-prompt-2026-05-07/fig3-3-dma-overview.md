# 图 3.3 DMA 子系统结构图（外部接口视角）
# Figure 3.3 DMA subsystem structure (external interface view)

## 在论文中的角色
- 首次引入：§3.3 总体架构（段尾）
- 引用位置：paper.md 第 238 行 "如图 3.3 所示 DMA 子系统结构"
- 论证作用：在 §3.3 末尾给出 DMA 子系统的"对外只表现一对 AXI"的概览。与 §4.10 图 4.7 (DMA 实现细节) 形成"概览 → 详细"两级。

> **Polisher 提醒**：图 3.3 与图 4.7 标题相近，建议在 §3.3 改用 "图 3.3 DMA 子系统对外接口图" 或在 §4.10 改用 "图 4.7 DMA 子系统内部结构图"。

## 图类型
架构框图（接口视角）。

## 设计要素

### 必含元素
1. **axi_dm IP**（中央）：标注 "Xilinx AXI DataMover IP"，含 MM2S 与 S2MM 两条通道。
2. **idma_ctrl / wdma_ctrl**：左上，共享 MM2S 通道，中间有 mm2s_arb（仲裁器）。
3. **odma_ctrl**：左下，独占 S2MM 通道。
4. **rdma_ctrl**：左中，从 DDR 拉 bias / shortcut。
5. **dfe（描述符获取引擎）**：左侧最上，独立挂 axi_m_mux。
6. **axi_m_mux**：右侧聚合器，把 axi_dm 的 MM2S/S2MM 主口 + dfe 主口三路聚合到唯一的 AXI4 主接口。
7. **axi_lite_csr**：右下角，独立 AXI-Lite 从口（4 个启动寄存器）。

### 标注要求
- mm2s_arb 旁标 "round-robin"
- axi_m_mux 输出标 "→ AXI4 Master (single port to DDR)"
- axi_lite_csr 输入标 "← AXI-Lite Slave (CSR)"
- 描述符 FIFO 容量 128

### 视觉层次
- 主角：axi_dm IP + axi_m_mux（系统集成关键）
- 配角：4 类自研 DMA 控制器
- 背景：dfe 描述符通路

## 数据来源
- §4.10 文字
- CLAUDE.md "DMA 子系统" 段
- RTL 文件头注释（idma_ctrl/wdma_ctrl/odma_ctrl/mm2s_arb）

## ASCII 示意稿

```
           ┌────────────────────────────────────────────────────────┐
           │ Accelerator Core (5 pipelined modules + cfg_regs)      │
           └─┬──────────────┬──────────────┬─────────────┬──────────┘
             │ IFB write    │ WB write     │ OFB read    │ bias/shortcut write
             ▼              ▼              ▲             ▼
          ┌──────┐       ┌──────┐       ┌──────┐      ┌──────┐
          │idma  │       │wdma  │       │odma  │      │rdma  │
          │ctrl  │       │ctrl  │       │ctrl  │      │ctrl  │
          └──┬───┘       └──┬───┘       └──┬───┘      └──┬───┘
             │ MM2S cmd     │ MM2S cmd     │ S2MM cmd    │ MM2S cmd
             └──┬───────────┘              │             │
                ▼                          │             │
            ┌────────┐                     │             │
            │mm2s_arb│ (round-robin)       │             │
            └───┬────┘                     │             │
                ▼                          ▼             ▼
          ┌─────────────────────────────────────────────────────┐
          │              axi_dm IP (Xilinx)                     │
          │      MM2S channel        S2MM channel               │
          └────────┬───────────────────────┬────────────────────┘
                   │ MM2S master           │ S2MM master
                   │                       │
   ┌────┐          │                       │
   │dfe │ ─────────┼───────────────────────┼─────┐
   └────┘ desc ptr │                       │     │
                   ▼                       ▼     ▼
                  ┌──────────────────────────────────┐
                  │       axi_m_mux (3-to-1)         │
                  └────────────────┬─────────────────┘
                                   │ AXI4 Master (128b) → external DDR
                                   ▼
                            ┌─────────────┐
                            │ AXI4 Master │
                            └─────────────┘

     ┌────────────────────────────────────────────────────┐
     │ axi_lite_csr  ←───── AXI-Lite Slave (32b) (CSR)   │
     │  4 start regs + status regs                       │
     └────────────────────────────────────────────────────┘
```

## 与正文的一致性检查
- [x] §3.3 段末 "DMA 子系统由 1 个 Xilinx axi_dm IP 加多个自研控制器组成"
- [x] mm2s_arb 串行仲裁、odma 独占 S2MM、axi_m_mux 三路聚合
- [x] axi_lite_csr 独立 from axi_m_mux（一对接口分别处理批量与配置）

## 不确定项
- [TBD: 是否在本图区分 idma/wdma/odma 进出 buffer 的箭头方向（IFB 写入 = 进，OFB 读 = 出）] — 已在 ASCII 中以箭头方向区分
