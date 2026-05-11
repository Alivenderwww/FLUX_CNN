# 图 3.3 DMA 子系统结构图（外部接口视角）
# Figure 3.3 DMA subsystem structure (external interface view)

## 在论文中的角色
- 首次引入：§3.3 总体架构（paper.md L238）
- 引用位置：paper.md L238
- 论证作用：在 §3.3 末尾给出 DMA 子系统的"对外只表现一对 AXI"的概览，与 §4.10 详细 DMA 实现图（图 4.7）形成"概览 → 详细"两级。

> **Polisher 提醒**：图 3.3 与图 4.7 标题相近，建议 §3.3 用 "图 3.3 DMA 子系统对外接口图" 或 §4.10 用 "图 4.7 DMA 子系统内部结构图"。

## 图类型
架构框图（接口视角）。

## 设计要素

### 必含元素
1. **axi_dm IP**（中央）：标注 "Xilinx AXI DataMover IP"，含 MM2S 与 S2MM 两条通道。
2. **idma_ctrl / wdma_ctrl**：左上层，共享 MM2S 通道，中间有 mm2s_arb（仲裁器）。
3. **odma_ctrl**：右上层，独占 S2MM 通道。
4. **rdma_ctrl**：左中，从 DDR 拉 bias / shortcut（共享 MM2S）。
5. **dfe（描述符获取引擎）**：左侧顶部，独立挂 axi_m_mux。
6. **axi_m_mux**：右下聚合器，把 axi_dm 的 MM2S/S2MM 主口 + dfe 主口三路聚合到唯一的 AXI4 主接口。
7. **axi_lite_csr**：右上独立方块，AXI-Lite 从口（4 个启动寄存器）。

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
    Accelerator Core (5 pipeline modules + cfg_regs)
   ┌────┬────┬────┬────┐                ┌──────────────────────┐
   │IFB │ WB │OFB │bias│                │ axi_lite_csr         │
   │wr  │ wr │rd  │/ sc│                │ 4 start regs         │◀── AXI-Lite (32 b)
   └─┬──┴─┬──┴─┬──┴─┬──┘                └──────────────────────┘
     │    │    │    │
     ▼    ▼    ▲    ▼
   ┌────┐┌────┐┌────┐┌────┐    ┌──────┐
   │idma││wdma││odma││rdma│    │ dfe  │  desc FIFO=128
   │ctrl││ctrl││ctrl││ctrl│    └──┬───┘
   └─┬──┘└─┬──┘└─┬──┘└─┬──┘       │
     │MM2S │MM2S │S2MM │MM2S      │
     └──┬──┴──┬──┘ │   │          │
        ▼     ▼    │   │          │
      ┌──────────┐ │   │          │
      │ mm2s_arb │ │   │          │
      │(round-rb)│ │   │          │
      └─────┬────┘ │   │          │
            ▼      ▼   ▼          │
   ┌─────────────────────────────┐│
   │   axi_dm IP (Xilinx)        ││
   │ MM2S ch.        S2MM ch.    ││
   └────────┬────────────┬───────┘│
            │ MM2S       │ S2MM   │
            ▼            ▼        ▼
        ┌──────────────────────────────┐
        │   axi_m_mux  (3-to-1)        │
        └──────────────┬───────────────┘
                       │ AXI4 Master (128 b) → DDR
                       ▼
                ┌─────────────┐
                │ AXI4 Master │
                └─────────────┘
```

## 与正文的一致性检查
- [x] §3.3 段末 "DMA 子系统由 1 个 Xilinx axi_dm IP 加多个自研控制器组成"
- [x] mm2s_arb 串行仲裁、odma 独占 S2MM、axi_m_mux 三路聚合
- [x] axi_lite_csr 独立于 axi_m_mux（一对接口分别处理批量与配置）

## 不确定项
- [TBD: 是否在本图区分 idma/wdma/odma 进出 buffer 的箭头方向（IFB 写入 = 进，OFB 读 = 出）] — 已在 ASCII 中以箭头方向区分

## image 生成提示词

### 中文版

科研论文配图，**DMA 子系统结构图（外部接口视角）**，IEEE 期刊配色风格，白底黑字，禁用彩色背景与卡通风格。**整体画面宽高比约 1:1（正方形）或 1:1.2（高略大于宽），避免整体过于竖向（高:宽 > 1.5:1）或过于横向（宽:高 > 1.5:1），以适合 A4 单栏插图。**

**版式（自上而下两层 + 右侧 csr 独立层，整体接近正方形）**：
- **顶部接口带**：上方画一条扁长圆角矩形 "Accelerator Core (5 pipeline modules + cfg_regs)"，淡灰 CMYK 0/0/0/10 填充，内部分 4 列小标签 "IFB wr / WB wr / OFB rd / bias-sc"。右侧另画独立 axi_lite_csr 小方块（淡黄 CMYK 0/10/30/0 填充），写 "axi_lite_csr / 4 start regs"，右侧细箭头进入并标 "AXI-Lite (32 b) Slave"。
- **第一控制层（4 类 DMA 控制器，水平排列）**：顶部接口带下方水平并列 4 个方块（淡蓝 CMYK 30/10/0/0 填充）"idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl"，每方块约 8 字宽。idma / wdma / rdma 的输出箭头汇向下方 mm2s_arb 方块（淡绿 CMYK 20/0/30/0 填充），内写 "mm2s_arb (round-robin)"。odma 输出箭头独立向下指向 axi_dm 的 S2MM 通道。
- **第二数据层（axi_dm IP，主角）**：mm2s_arb 与 odma_ctrl 下方画一条横跨的 axi_dm IP 大圆角矩形（最深色 CMYK 40/15/0/10 填充作为主角），内分 2 行写 "axi_dm IP (Xilinx)" / "MM2S channel ｜ S2MM channel"。
- **dfe 旁路**：第一控制层左侧另画 dfe 小方块（淡蓝填充），内写 "dfe / desc FIFO=128"，下方箭头独立连入 axi_m_mux。
- **第三聚合层**：axi_dm 下方画 axi_m_mux 圆角矩形（淡橙 CMYK 0/20/40/0 填充），内写 "axi_m_mux (3-to-1)"，axi_dm 的 MM2S / S2MM + dfe 三路输入箭头汇入。axi_m_mux 下方画 AXI4 Master 小方块连接，箭头右标 "AXI4 Master (128 b) → DDR"。

**字体**：英文 Times New Roman 10 pt，模块名加粗；位宽与通道标签 Times New Roman 8 pt 等宽。**禁止**手写体、彩色渐变、3D 阴影、卡通元素、装饰图标。整体保持工科论文严谨风格。

### English version

Scientific paper figure, **DMA subsystem structure (external interface view)**, IEEE-journal style, white background with black text, no decorative colors or cartoon elements. **The overall figure aspect ratio should be approximately 1:1 (square) or 1:1.2 (slightly taller than wide). Avoid overly tall layouts (height:width > 1.5:1) or overly wide layouts (width:height > 1.5:1), to fit a single-column A4 figure.**

**Layout (two stacked tiers + standalone CSR on the right, near-square)**:
- **Top interface band**: Draw a wide flat rounded rectangle "Accelerator Core (5 pipeline modules + cfg_regs)" (light-gray CMYK 0/0/0/10 fill) with 4 sub-labels inside: "IFB wr / WB wr / OFB rd / bias-sc". On the right, draw a separate axi_lite_csr small box (light-yellow CMYK 0/10/30/0 fill) labeled "axi_lite_csr / 4 start regs", with a thin arrow on its right labeled "AXI-Lite (32 b) Slave".
- **First control tier (4 DMA controllers, horizontally arranged)**: Below the top band, place 4 boxes side-by-side (light-blue CMYK 30/10/0/0 fill): "idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl", each ~8 characters wide. Outputs of idma / wdma / rdma converge downward to a mm2s_arb box (light-green CMYK 20/0/30/0 fill) labeled "mm2s_arb (round-robin)". The odma_ctrl output arrow goes directly down to the axi_dm S2MM channel.
- **Second data tier (axi_dm IP, focal point)**: Below mm2s_arb and odma_ctrl, draw a wide axi_dm IP rounded rectangle (deepest tone CMYK 40/15/0/10 fill as focal point) in two lines: "axi_dm IP (Xilinx)" / "MM2S channel ｜ S2MM channel".
- **dfe sidepath**: To the left of the first control tier, draw a small dfe box (light-blue fill) labeled "dfe / desc FIFO=128", with its own arrow downward into axi_m_mux.
- **Third aggregation tier**: Below axi_dm draw an axi_m_mux rounded rectangle (light-orange CMYK 0/20/40/0 fill) labeled "axi_m_mux (3-to-1)"; the three inputs (MM2S, S2MM from axi_dm + dfe) merge into it. Below axi_m_mux draw an AXI4 Master box; arrow to the right labeled "AXI4 Master (128 b) → DDR".

**Typography**: English in Times New Roman 10 pt, module names bold; bit-widths and channel labels in Times New Roman 8 pt monospaced. **Strictly forbidden**: handwritten fonts, color gradients, 3D shadows, cartoon elements, decorative icons. Maintain rigorous engineering-paper aesthetics throughout.
