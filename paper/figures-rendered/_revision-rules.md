# v5.1 修订规则（基于用户全面反馈 grill 共识）

> 适用所有重做 / 小修的 TikZ 图。在写任何 figX-Y.tex 之前先读本文件。
> 旧的 25 张 v5 版本已被用户全面反馈，本规则用于第二轮重做。

---

## 全局规则（**所有图必须遵守**）

1. **节点内文字极简**：一两个单词搞定（如 `Conv` / `Weights` / `Accumulator`），**不要堆长串**（如 "16×16 INT8 MAC + 三级 RF" 这种是反例）。位宽/类型/参数等说明放节点旁的箭头标注或图例上。

2. **不加图标题**：TikZ 内不画 "图 X.Y" / "Algorithm X" 标题，由论文 Word caption 承担。

3. **不加补充说明**：
   - 禁用括号 `(...)` 包围辅助信息（如 "AXI4 (128 b)" 改成 "AXI4 128 b"，或者放进图例）
   - 禁用 `Note: ...` / "注: ..." 行
   - 禁用 figure-md 中的"次要说明"（如 "from prev layer" 这种语气松散的注释）

4. **信息密度均匀**：避免大片空白区域；如果某象限明显空，要么挪动元素填补，要么缩小整图尺寸。

5. **宽高比 2:3 ~ 2:1**：不要过竖（高/宽 > 1.5）也不要过横（宽/高 > 2.5）。多数图目标 1:1 ~ 1.5:1。

6. **配色与字体**：继续用 `_tikz-style.tex` 已固化的配色和字体。重点使用：
   - `flBlue` 淡蓝：节点主填充
   - `flYellow` 淡黄：侧路输入（如权重、偏置）
   - `flGray` 淡灰：存储 / Quantizer
   - `flGreen` 浅绿：激活 / Quantizer 节点
   - `coAccent` 强调红：突出关注点（如 Accumulator、远端访问、反压、Residual 短路）
   - 文字色统一黑色，标注辅助色用 `coAnno` (#404040)

7. **连线箭头**：箭头线宽**不能太细**（最低 0.8pt）、箭头**不能太小**（用 `>={Stealth[length=3mm,width=2mm]}` 起步）。标注字体不能小于 `\footnotesize`。

8. **复用现成样板**：
   - 整体布局参考 `fig2-4-numa-topology.tex`（用户拍板基线）
   - 数据流复用 `fig4-9-ky-fold.tex` 的"算法 + 概念图"组合
   - 6 层 FSM 见下方专项

---

## fig2-2 量化推理通路 —— 仿白皮书图 1 版式

**风格参考**：A White Paper on Neural Network Quantization（Qualcomm 2021）Figure 1(a) "Diagram for quantized on-device inference with fixed-point operations"。

**版式**：
- 纵向单列流水（**自下而上**）：Input → Conv → Accumulator → Activation → Requantization → Output
- 侧路两路：Weights → Conv（左侧分支）、Biases → Accumulator（右侧分支）
- 节点颜色：
  - Conv / Activation：深灰主体（深蓝灰，`coAnno` 或 `flBlue` 加深）
  - **Accumulator：红色 `coAccent` 实心填充 + 白字**（突出溢出关注点）
  - Requantization / Quantizer：青绿（`flGreen` 加深）
  - Weights / Biases：深蓝实心 `coBaseline` + 白字
- 箭头旁标位宽：`int8` / `int32`（不带任何括号，斜体小字）

**节点文字一律一个词**：Conv / Weights / Biases / Accumulator / Activation / Requantization / Output / Input。

---

## 6 层嵌套 FSM 表现 —— **counter × 模块矩阵**（替代俄罗斯套娃）

适用图：fig3-5 分层调度示意 / fig4-6 cfg_regs + 6 层 FSM

**版式**：
- 主体：一张表格矩阵
  - **行**（自外到内 6 行）：cs / yout / tile / cins / round / pos
  - **列**（5 个流水模块）：line_buffer / mac_array / parf_accum / ofb_writer / wgt_buffer
  - **单元格**：标"该 counter 是否驱动该模块本地推进事件"
    - 主推动 = 实心 ● + 事件名（如 `evt_iss_cs_wrap`）
    - 间接相关 = 空心 ○
    - 不相关 = 空白
- 矩阵右侧或上方加一个小型阶梯伪代码框（4-6 行）作为补充
- fig3-5 在矩阵上方加 Task Descriptor 链表（macro-level）
- fig4-6 在矩阵左侧加 cfg_regs 字段分组（loop bounds / buffer addrs / quant params / schedule params）

**核心论证**：体现"无中心 FSM、循环嵌套关系隐含在模块互连拓扑中"——矩阵让这种隐含关系显式可视化。

---

## fig3-1 / 3-2 / 3-3 三张分层 RTL 复刻 —— 按模块层级分

| 图 | 视角 | 必含 RTL 模块 |
|---|---|---|
| fig3-1 系统架构 | multicore_top 顶层 | Host CPU + axi_lite_csr + axi_lite_1to4 + DFE + 4× ConvCore（不展开内部，单方块）+ ifb_axi_slave × N + Xilinx SmartConnect + Off-chip DDR |
| fig3-2 核内数据通路 | 单 ConvCore 内部 | cfg_regs + sequencer + line_buffer → mac_array → parf_accum → ofb_writer（5 级主流水）+ wgt_buffer 侧路 + (idma/wdma/odma/rdma)_ctrl 5 个 DMA 控制器接口 |
| fig3-3 DMA 子系统 | DMA 部分 | idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl + mm2s_arb（3-to-1 仲裁）+ axi_dm IP（MM2S + S2MM 双通道）+ axi_m_mux + DFE |

**axi_dm 作为黑盒** —— 不展开内部，只画输入输出端口。

---

## 已删除 4 张

- fig1-1 端侧场景：删（用户：太单调、信息量不足）
- fig2-1 卷积原理：删（用户自画）
- fig4-2 MAC 阵列：删（用户自画）
- fig4-3 部分和累加：删（用户自画）

---

## 修订汇总表

### 大改 14 张

| 图 | 用户反馈 | 修订要点 |
|---|---|---|
| fig1-2 加速器谱系 | 形状+颜色+纵轴三重区分混乱；箭头细灰；本工作星星太大 | **只保留颜色区分**；纵轴改散点（同泳道交错）；箭头加粗；本工作星标尺寸减半 |
| fig1-3 创新点思维导图 | 排版稀疏 | 整体收紧版面，5 辐射节点拉近中心；每节点文字简化为关键词 |
| fig2-2 量化推理通路 | 参考白皮书图 1 | 见上方专项 |
| fig3-1 系统架构 | 太简单，按 RTL 完整复刻 | 见上方专项 |
| fig3-2 核内数据通路 | 排版稀疏；箭头/字体太小 | 见上方专项；箭头线宽 ≥ 0.8pt，标注 ≥ `\footnotesize` |
| fig3-3 DMA 子系统 | 排版稀疏；按 RTL 复刻 | 同上 |
| fig3-5 分层调度 | 排版混乱；6 层 FSM 表现一般 | 用 counter × 模块矩阵替代套娃 |
| fig3-6 三层复用叠加 | 信息密度低、意义不明 | 整图重设计；强化"三层复用如何在 PE 阵列上同时作用"的语义；可考虑分 3 panel 横向对比 |
| fig4-1 line_buffer | 信息密度低、框太大 | 缩小总框；增加内部细节（ARF / FILL/CONSUME 两侧并行 / kx-ky 计数器） |
| fig4-4 SDP | 元素遮挡；右上空白 | 重排；填补空白处放 row_credit 反压细节或位宽时序 |
| fig4-5 wgt_buffer | 右上/右下空白 | 紧凑化；填补空白处放 WB-WRF round-chunk 分块或 round 计数推进 |
| fig4-6 cfg_regs + FSM | 6 层 FSM 一般；元素堆砌 | 用 counter × 模块矩阵 + cfg_regs 字段分组 |
| fig4-7 DMA 子系统 | 同 fig3-3 排版稀疏 | 与 fig3-3 是同一主题在 §4.10 重复——**考虑合并或聚焦 SMC 互联**；待确认 |
| fig4-8 多核扩展 | 线连飞 | 重新规划走线，跨核 push 用绕行而非穿越 |
| fig4-10 DSP 跨列 | 排版烂 | 重做整体布局；位段打包条要清晰 |

### 小修 7 张

| 图 | 修订点 |
|---|---|
| fig2-3 数据流对比 | 元素重叠；行静止 act 斜箭头位置不对 |
| fig2-4 NUMA（用户拍板） | "100 ns" 标注：字色改成 `coImproved` 绿色；白底改透明底 |
| fig2-5 能效金字塔（matplotlib） | 横轴改**线性坐标**（更能体现差距） |
| fig3-1 雷达图（matplotlib） | 去掉刻度数字 20/40/60/80/100；去掉 5 指标名后括号补充 |
| fig4-9 Ky-fold | 右侧 PE 折叠改**横排**（当前竖排不好） |
| fig4-11 S2D | 右侧 S2D 示意有元素堆砌遮挡，重排避开 |
| fig5-0d ResNet11 | 加号位置错（看起来像 Main1 输入直接 + Main2 输出再接 DS1）——按 ResNet 残差结构修正 |
