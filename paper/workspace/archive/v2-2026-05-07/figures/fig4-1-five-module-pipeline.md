# 图 4.1 五模块去中心化 valid-ready 流水线
# Figure 4.1 Five-module decentralised valid-ready pipeline

## 在论文中的角色
- 首次引入：§4.2 段 "第一层骨架是 5 模块去中心化 valid-ready 流水..." [依赖: Fig.4.1]
- 论证作用：详细展开图 3.1 上半区的核流水部分。重点是"每模块自维护 counter（H/W/Ky/Kx/Cin/Cout）"+"无中心 FSM"+"valid-ready 弹性 join"，对应贡献 C1.2。后续图 4.2-4.5 是本图各模块的细化。

## 图类型
横向流水图：5 个模块矩形 + valid-ready 双线 + 各模块内部 counter list 标注 + 共享 cfg_regs 顶部一条总线。

## 设计要素

### 必含元素
- **横向 5 模块**（从左到右）：
  1. `line_buffer`（IFB ring + 滑窗读取，自维护 H/W/Ky/Kx counter）
  2. `mac_array`（16×16 INT8 OS+列广播，自维护 Cin/Cout counter）
  3. `parf_accum`（16 列 PARF SRAM 累加，自维护 Cin/Cout/Kx counter）
  4. `ofb_writer`（含 SDP 后处理，自维护 H_out/W_out/Cout counter）
  5. `wgt_buffer` （在 mac_array 上方，侧路供 WRF；自维护 Cout counter）
- **模块间**：每对相邻模块之间画 `valid →` 和 `← ready` 双线（红色 valid，蓝色 ready），顶部小字 "v/r 弹性 join"
- **顶部**：一条横贯 5 模块的虚线总线，标 `cfg_regs (共享，所有模块按字段读)`
- **每模块矩形内部**：用小字列出该模块的 counter（如 line_buffer 内部小字 `H_cnt / W_cnt / Ky_cnt / Kx_cnt`）
- **底部一句话**："5 模块各自维护 counter，无中心 FSM。上下游通过 valid-ready 弹性 join，气泡可任意传递。"

### 标注要求
- 在 mac_array 与 parf_accum 之间额外标注一行 "PSUM 32-bit"（说明这一段总线宽度与其他不同）
- 在 ofb_writer 输出端标注 "→ odma_ctrl"（去 DMA 子系统）
- wgt_buffer 与 mac_array 之间标 "WRF (16 行权重)"
- line_buffer 输入端标 "← idma_ctrl (IFB 加载)"
- 用对比色突出 cfg_regs 总线（紫色虚线）

### 视觉层次
- 主角：5 个模块矩形（最大、最显眼）
- 配角：valid-ready 连线
- 背景：cfg_regs 总线 + counter 标注

## ASCII 示意稿

```
   ┌─ 共享 cfg_regs（每模块按字段读取，无 broadcast 状态机） ──────────────┐
   │                                                                       │
   ▼               ▼               ▼               ▼               ▼

   ┌─ wgt_buffer (侧路) ────────────────────────────────────┐
   │  自维护 Cout counter / WRF 16 行权重                    │
   └────────────┬───────────────────────────────────────────┘
                │ 权重广播给 mac_array 16 行
                ▼
                                                              
 idma  ┌──────────┐  v ──→  ┌──────────┐  v ──→  ┌──────────┐  v ──→  ┌──────────┐ → odma
   ──→ │line_     │  ←── r  │mac_array │  ←── r  │parf_     │  ←── r  │ofb_      │ ──→
       │buffer    │         │ 16×16    │         │accum     │         │writer    │
       │ IFB ring │         │ INT8     │         │ 16 PARF  │         │ +SDP 后  │
       │ H/W/Ky/  │         │ OS+列广播│         │ Cin/Cout │         │  处理     │
       │ Kx 计数  │         │ Cin/Cout │         │ /Kx 计数 │         │ H_out/   │
       │          │         │ 计数     │         │          │         │ W_out/   │
       │          │         │          │         │          │         │ Cout 计数│
       └──────────┘         └──────────┘         └──────────┘         └──────────┘
        (line_      INT8 act    (16 PE       PSUM 32      累加完成      INT8 out
         buffer       8-bit       行 ×        bit                       后写 OFB ring
         滑窗读)       16 列)                                            ↓ odma_ctrl

   v = valid (生产方拉高)，r = ready (消费方拉高)
   仅当 v ∧ r 同时为高，一拍数据被消费——天然支持弹性 join 与气泡传递
   ★ 5 模块各自维护 counter，无中心 FSM；任一模块可单独被仿真激励驱动验证 ★
```

## 数据来源
- paper.md §4.2 段 1
- contributions.md C1.2
- docs/modules/core_top.md / line_buffer.md / mac_array.md / parf_accum.md / ofb_writer.md / wgt_buffer.md
- RTL/core_top.sv（commit 5fe16b2）

## 与正文一致性检查
- [x] 5 模块名（line_buffer / mac_array / parf_accum / ofb_writer / wgt_buffer）与 §4.2 / CLAUDE.md 完全一致
- [x] "wgt_buffer 侧路供 WRF"——与 §4.2 / CLAUDE.md 表述一致
- [x] cfg_regs 共享所有模块——与 §4.2 / contributions.md C1.2 表述一致
- [x] 每模块自维护 counter，无中心 FSM——与 §4.2 / contributions.md C1.2 元层论断一致

## 不确定项
无。本图基于已实现的 RTL（commit 5fe16b2）。
