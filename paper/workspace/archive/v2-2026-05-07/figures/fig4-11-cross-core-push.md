# 图 4.11 跨核 SRAM 直送（M2 push）路径
# Figure 4.11 Cross-core SRAM direct push (M2 push) path

## 在论文中的角色
- 首次引入：§4.5 段 "跨核 SRAM 直送（M2 push）是多核加速比的关键优化..." [依赖: Fig.4.11]
- 论证作用：对照"朴素 DDR 中转方案"vs"跨核 SRAM 直送方案"，展示 producer 核 ODMA 直接把输出写到 consumer 核 IFB region 的机制。是 §4.5 / contributions.md C3.4 中"多核加速比的关键优化"图示根据。

## 图类型
对照图：左半"朴素 DDR 中转"（producer→DDR→consumer，2 次 DDR 访问），右半"M2 push"（producer→consumer SRAM，1 次跨核 AXI 写）。

## 设计要素

### 必含元素
- **左半 朴素 DDR 中转方案**：
  - producer 核 (core[i])：layer N 输出 → ODMA → AXI4 主口 → DDR
  - DDR 中存放 layer N FM
  - consumer 核 (core[i+1])：IDMA → AXI4 主口 → DDR ← layer N FM → IFB
  - 标注 "2 次 DDR 访问 / 层 → DDR 带宽瓶颈"
  - 用红色高亮 DDR 带宽（粗箭头）
- **右半 M2 push 方案**：
  - producer 核 (core[i])：layer N 输出 → ODMA → AXI4 主口 → 写到 consumer 核 IFB region 地址（0x8000_0000 + (i+1) × 0x1000_0000）
  - 跳过 DDR
  - consumer 核 (core[i+1])：IFB 直接收到 layer N FM，无需自己拉取
  - 标注 "0 次 DDR 访问 / 层间转发 → DDR 带宽节省 ~50%"
  - 用绿色高亮跨核直送路径
- **底部对比表**：

| 方案 | DDR 访问次数 / 层间转发 | DDR 带宽占用 | wall cycles 影响 |
|------|----------------------|-------------|----------------|
| 朴素 DDR 中转 | 2（写 + 读） | 高 | N=4 整网 596K → 354K（不启用 push）|
| M2 push（跨核 SRAM 直送） | 0 | 低 | N=4 整网 354K → 220K （Phase 7 SMC + push）|

### 标注要求
- 用 axi_4to5 fanout 节点表示 4 核共享 AXI 主口
- producer 写到 consumer IFB region 的地址用统一地址映射规则（0x8000_0000 + i × 0x1000_0000）标注
- 在右半图加强调 "跨核 SRAM 直送 = 同一 AXI 主口 + 不同 IFB region 地址"
- 底部加一句："M2 push 是 N=4 加速比从 1.68× 进一步压低到 SMC 主线 220K cycles 的关键优化（commit 5fe16b2）"

### 视觉层次
- 主角：左右两条数据流路径
- 配角：底部对比表
- 背景：地址映射标注

## ASCII 示意稿

```
   ──── 左半：朴素 DDR 中转方案 ─── ─── 右半：M2 push 方案 (本工作) ───

   layer N (producer = core[0])      layer N (producer = core[0])
        │                                  │
        ▼ ODMA                             ▼ ODMA
   ┌─────────────┐                     ┌─────────────┐
   │ axi_4to5    │                     │ axi_4to5    │
   │ (主口聚合)  │                     │ (主口聚合)  │
   └────┬────────┘                     └────┬────────┘
        │ AXI4 写                           │ AXI4 写
        ▼                                   │ 目标地址 = consumer IFB
   ┌─────────────┐                          │ region:
   │  DDR        │                          │ 0x8000_0000 + 1×0x1000_0000
   │  layer N FM │                          │ = 0x9000_0000  (core[1] IFB)
   └────┬────────┘                          ▼
        │ AXI4 读                       ┌────────────────────┐
        ▼ IDMA                          │ core[1] IFB region │
   ┌─────────────┐                      │ (无需自己 IDMA)    │
   │ axi_4to5    │                      │  ◀── 直接收到      │
   │ (主口聚合)  │                      │      layer N FM    │
   └────┬────────┘                      └────────────────────┘
        ▼                                       │
   layer N+1 (consumer = core[1])               ▼
                                          layer N+1 (consumer = core[1])

   2 次 DDR 访问 / 层间转发              0 次 DDR 访问 / 层间转发
   (1 次写 + 1 次读)                     (跨核 SRAM 直送)
   DDR 带宽占用：高 (×)                   DDR 带宽节省：~50% (✓)

   ─────  对比表  ───────────────────────────────────────────────

   ┌────────────────┬──────────────┬──────────┬──────────────────┐
   │ 方案           │ DDR/转发     │ 带宽     │ N=4 整网 cycles  │
   ├────────────────┼──────────────┼──────────┼──────────────────┤
   │ 朴素 DDR 中转  │ 2 次/层      │ 高       │ 354K (sequential)│
   │ M2 push        │ 0 次/层      │ 低       │ 220K (SMC 主线)  │
   └────────────────┴──────────────┴──────────┴──────────────────┘

   ★ M2 push 是把 N=4 加速比 1.68× 进一步推到 SMC 220K cycles 的关键 ★
   commit 5fe16b2 / Phase 7 SMC + NUMA 主线
```

## 数据来源
- paper.md §4.5 段 "跨核 SRAM 直送（M2 push）..."
- contributions.md C3.4 / C4.6
- STATUS.md §2.8 (354K W 切片) / §2.12 (220K SMC + NUMA)
- CLAUDE.md "多核地址映射" 段

## 与正文一致性检查
- [x] producer ODMA 直接写到 consumer IFB region 地址 — 与 §4.5 / contributions.md C3.4 一致
- [x] 共享 AXI 主口（axi_4to5）+ 不同 IFB region 地址 — 与 §3.3 / §4.5 一致
- [x] 354K (W 切片) vs 220K (SMC + push) — 与 §5.5 / STATUS.md §2.8 / §2.12 一致
- [x] 地址映射 0x8000_0000 + i × 0x1000_0000 — 与 CLAUDE.md / §3.3 一致

## 不确定项
- 220K 与 354K 哪一个作为论文最终主表数字 — paper.md §5.5 标 [TBD]，与本图无冲突，本图同时展示两种路径数字。
