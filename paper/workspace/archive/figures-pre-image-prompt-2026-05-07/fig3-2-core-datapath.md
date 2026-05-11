# 图 3.2 加速核内数据通路结构图
# Figure 3.2 Datapath structure inside the accelerator core

## 在论文中的角色
- 首次引入：§3.3 总体架构
- 引用位置：paper.md 第 234 行 "如图 3.2 所示核内数据通路结构"
- 论证作用：在系统架构（图 3.1）下推一层，给出加速核内 5 个流水模块 + 共享 cfg_regs 的串接关系。读者读完此图后应理解"line_buffer → mac_array → parf_accum → ofb_writer + 侧路 wgt_buffer"的核内拓扑。

> **Polisher 提醒**：paper.md 第 234 行后没有独立的 "**图 3.2 ...**" 加粗标题行，应在 Phase 7 补一行。

## 图类型
数据通路图（核内 5 模块 + cfg_regs）。

## 设计要素

### 必含元素
1. **行缓存模块（line_buffer）**：左侧首块；内部画 IFB ring（多行环形缓冲）；上接 IDMA 写口，下出 *K*×*K* 窗口。
2. **MAC 阵列模块（mac_array）**：核心计算块；标注 "16 col × 16 PE = 256 INT8 MAC"；输入激活窗口 + 权重；输出 16 路 PSUM。
3. **部分和累加模块（parf_accum）**：紧接 mac_array 下游；内部画 16 个独立 SRAM (parf_col × 16)，每列 PARF=32。
4. **SDP 后处理模块（ofb_writer）**：parf_accum 下游；内部链 "bias→shift→saturate→residual→saturate→OFB"；右侧画 OFB ring；上接 bias_rf + Shortcut Bank。
5. **权重缓存模块（wgt_buffer）**：侧路；上接 WDMA 写口，画 WB SRAM（1024 word）+ WRF（32）；侧路供 mac_array。
6. **共享 cfg_regs**：底部一条横贯的浅色长条，向 5 个模块各拉一条细虚线（"cfg broadcast"）。
7. **valid-ready 握手**：每两个模块之间的连线上画 "v/r" 小标。

### 标注要求
- 数据宽度：激活向量 INT8×16、权重向量 INT8×16、PSUM 32-bit×16 路、OFM INT8
- 容量：IFB=8192 word、WB=1024 word、WRF=32、ARF=32、PARF=32、OFB=2048 word
- 在 line_buffer 上画 ARF=32（位于 mac_array 左侧的小 RF 也可）
- 标注 "row credit" 反向控制流（line_buffer → IDMA、ofb_writer → ODMA）

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
   IDMA                                      WDMA                bias / shortcut
    │                                          │                       │
    │ INT8 row                                 │ INT8 weights          │
    ▼                                          ▼                       │
┌─────────────────┐         ┌────────────────────────┐                 │
│ line_buffer     │         │ wgt_buffer             │                 │
│  IFB ring       │         │  WB(1024) → WRF(32)    │                 │
│  (8192 word)    │         └─────────────┬──────────┘                 │
│   ┌─ ARF(32) ─┐ │                       │                            │
└────┬──────────┘ │                       │ INT8×16 weights            │
     │  K×K window│                       ▼                            │
     │  v/r       │  ┌──────────────────────────────────┐               │
     └────────────┴─▶│  mac_array                       │               │
                     │   16 col × 16 PE = 256 INT8 MAC  │               │
                     │   col-broadcast act + col-cin par│               │
                     └──────────────────────┬───────────┘               │
                                            │ 16× PSUM (32b)            │
                                            │ v/r                       │
                                            ▼                           │
                                ┌────────────────────────┐              │
                                │ parf_accum             │              │
                                │  parf_col × 16 SRAMs   │              │
                                │  PARF=32 each col      │              │
                                └──────────┬─────────────┘              │
                                           │ 16× accum-done             │
                                           │ v/r                        │
                                           ▼                            │
                                ┌──────────────────────────────────────┐│
                                │ ofb_writer (SDP)                     ││
                                │  bias→shift→sat→residual→sat→OFB     │◀┘
                                │  ┌──── OFB ring (2048 word) ─────┐   │
                                └────────────────────┬─────────────────┘
                                                     │ INT8 row
                                                     │ row credit ↑
                                                     ▼
                                                   ODMA

         ┌──────────────────────────────────────────────────────────┐
         │ cfg_regs (50+ regs, dual write port)                     │
         │  ── broadcast cfg to all 5 modules (dashed) ──           │
         └──────────────────────────────────────────────────────────┘
```

## 与正文的一致性检查
- [x] §3.3 描述的 5 模块 + 共享 cfg_regs 拓扑与图一致
- [x] 模块间 valid-ready 握手在图中以 "v/r" 标注
- [x] WRF / ARF / PARF 三级寄存器堆全部出现（与 §6.1 创新点一致）

## 不确定项
- [TBD: ARF 是否独立画一个块还是嵌入 line_buffer 内] — 倾向嵌入 line_buffer 输出端，因 ARF 在 RTL 中位于 line_buffer 与 mac_array 边界
