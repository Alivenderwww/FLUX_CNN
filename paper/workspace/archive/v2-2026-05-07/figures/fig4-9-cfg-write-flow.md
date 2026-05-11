# 图 4.9 CFG_WRITE descriptor 配置流时序
# Figure 4.9 Configuration flow timing of CFG_WRITE descriptors

## 在论文中的角色
- 首次引入：§4.4 段 "CFG_WRITE descriptor 配置流是 host 配置开销优化的关键工程贡献..." [依赖: Fig.4.9, Tab.4.2]
- 论证作用：展示"裸 AXI-Lite 50 次/层 vs descriptor 4 次/层"的对比 + DFE 双口仲裁机制。是 §4.4 / contributions.md C3.2 / C3.3 的图示根据。配套表 4.2（数字对比）。

## 图类型
对照时序图：上半"裸 AXI-Lite 方案"（host 写 ~50 次），下半"CFG_WRITE descriptor 方案"（host 写 4 次 + DFE 自动批量更新）；右侧 cfg_regs 双口仲裁结构。

## 设计要素

### 必含元素
- **上半（裸 AXI-Lite 朴素方案）**：
  - 时间轴：host 端连续 50 次 `axi_lite.write(addr, data)`，每次写一个 cfg_regs 字段
  - 标注 "host CPU 时序压力 + AXI-Lite 带宽占用"
  - 时间轴下小字 "~50 次写 = ~50 × t_axi_lite_write"
- **下半（CFG_WRITE descriptor 优化方案）**：
  - 时间轴：
    - host 写 1：`axi_lite.write(DESC_PTR_REG, ddr_addr)` — 设 descriptor list 在 DDR 的起始地址
    - host 写 2：`axi_lite.write(DESC_LEN_REG, n_layers)` — 设 descriptor 数量
    - host 写 3：`axi_lite.write(START_REG, 1)` — 触发 DFE 拉取 descriptor + 自动更新 cfg_regs
    - 之后：`DFE` 自动从 DDR 读 descriptor list，批量更新 cfg_regs（每条 descriptor 一次性更新一组字段，无 host 介入）
    - host 写 4：`axi_lite.read(STATUS_REG)` 轮询完成（done sticky）
  - 标注 "host 写次数：4 / 层（与裸方案 ~50 / 层 对比 12.5×）"
- **右侧 cfg_regs 双口结构**：
  - cfg_regs 块画两个端口：左端口接 axi_lite_csr（host 直写，慢路径），右端口接 DFE（descriptor 批量更新，快路径）
  - 中央仲裁逻辑：DFE 写优先级高，axi_lite_csr 写仅在 DFE 空闲时生效
  - 标注 "双口 cfg_regs：仲裁更新（C3.3）"

### 标注要求
- 上下两半用红 / 绿对比色（红=朴素，绿=优化）
- 时间轴单位：用拍数 cycles 标注，假定 t_axi_lite_write ≈ 4 cycles
- 在右侧 cfg_regs 双口下方加 "Done sticky 寄存器：host 轮询每层完成"
- 顶部加大字标 "CFG_WRITE descriptor：host AXI-Lite 写次数 ~50/层 → 4/层"

### 视觉层次
- 主角：上下时间轴对比
- 配角：右侧 cfg_regs 双口结构
- 背景：拍数标注

## ASCII 示意稿

```
   ★ CFG_WRITE descriptor：host AXI-Lite 写次数 ~50/层 → 4/层（约 12.5×）★

   ─────  上半：裸 AXI-Lite 朴素方案（红，~50 次/层）  ─────────────

   host: w[0] w[1] w[2] w[3] w[4] w[5] w[6] ... w[48] w[49]
          │    │    │    │    │    │    │         │     │
          ▼    ▼    ▼    ▼    ▼    ▼    ▼         ▼     ▼
   AXI-Lite ─────────────────────────────────────────────→ cfg_regs
              连续 50 次单字段写
   时间：~50 × t_axi_lite_write ≈ 200 cycles / 层
   ▶ 问题：host CPU 时序压力大；多层切换时延敏感

   ─────  下半：CFG_WRITE descriptor 方案（绿，4 次/层）  ───────────

   host: w[DESC_PTR]  w[DESC_LEN]  w[START]                  r[STATUS]
            │             │           │                          ▲
            ▼             ▼           ▼                          │
         设 desc 起址  设 desc 数  触发 DFE          host 轮询 done sticky
                                       │
                                       ▼
   DFE 自动从 DDR 拉 descriptor list ─────→ cfg_regs 批量更新
   (每条 desc 一次性更新一组字段，无 host 介入)
   时间：~4 × t_axi_lite + n × t_desc_fetch
        其中 t_desc_fetch 主要受 DDR 突发限制

   ─────  右侧：cfg_regs 双口仲裁结构（C3.3）  ──────────────────

                  ┌──────────────────────────────────┐
   axi_lite_csr ──▶│ Port A (慢路径，host 直写)       │
   (host 写)       │                                  │
                  │  ┌────────────────────────────┐  │
                  │  │     仲裁逻辑                │  │
                  │  │  · DFE 写优先级高           │  │
                  │  │  · axi_lite 仅在 DFE 空闲   │  │
                  │  │    时生效                   │  │
                  │  └────────────────────────────┘  │
                  │                                  │
   DFE ─────────▶│ Port B (快路径，descriptor 批量) │
   (descriptor    │                                  │
    自动更新)     │  cfg_regs 字段值                 │
                  └──────────────────────────────────┘
                  Done sticky 寄存器：host 轮询每层完成

   ★ 写次数从 ~50/层 → 4/层（DESC_PTR + DESC_LEN + START + STATUS 轮询）★
```

## 数据来源
- paper.md §4.4 段 "CFG_WRITE descriptor 配置流..."
- contributions.md C3.2 / C3.3
- docs/modules/dfe.md
- RTL/cfg_regs.sv（commit 5fe16b2，双口仲裁）

## 与正文一致性检查
- [x] 朴素 ~50 次/层、descriptor 4 次/层 — 与 §4.4 / contributions.md C3.2 完全一致
- [x] 双口 cfg_regs（一口 axi_lite_csr，一口 DFE，仲裁更新） — 与 §4.4 / contributions.md C3.3 一致
- [x] Done sticky 寄存器供 host 轮询 — 与 §4.4 一致
- [x] DFE 拉取 descriptor list from DDR — 与 §4.4 一致

## 不确定项
- t_axi_lite_write ≈ 4 cycles 为典型 AXI-Lite 单字段写代价，具体数字 [CHECK: 与 RTL 实测一致性可由 STATUS / sim profile 复核]，本图仅作时序量级示意，不作精确性能基准。
