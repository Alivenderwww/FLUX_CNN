# 图 4.4 分列累加器 PARF 结构图
# Figure 4.4 Per-column partial-sum accumulator (PARF)

## 在论文中的角色
- 首次引入：§4.2 段 "第四层骨架是分列累加器 PARF..." [依赖: Fig.4.4]
- 论证作用：可视化 PARF 的 "16 个 parf_col 内部独立 SRAM + 外壳共享 wr_addr/we" 折中设计——既具备每列独立累加语义，又能在 16 列同时写入时共用同一组写入控制逻辑。对应贡献 C1.4。

## 图类型
分块结构图：16 个 parf_col 单元（横向排列）+ 顶部共享 wr_addr/we 控制信号 + 每个 parf_col 内部独立 SRAM 与 rd_addr。

## 设计要素

### 必含元素
- **顶部控制总线**：`wr_addr (5-bit)` + `we (1-bit)`，跨 16 个 parf_col 同步广播
- **16 个 parf_col 单元**（横向排列）：
  - 每个 parf_col 内部：1 块独立 SRAM（深度 PARF=32, 宽度 32-bit），1 个独立 rd_addr（5-bit），1 个累加器 + 写入逻辑
  - 每列从 mac_array.PSUM_j 读入新一笔 PSUM，与当前 SRAM[wr_addr] 累加后写回
  - 标注 "parf_col[j]，j = 0..15"
- **底部输出**：16 列累加完成后，按 rd_addr 顺序读出 → 送 ofb_writer + SDP
- **左侧标注**：标 "外壳共享 wr_addr / we：节省全局连线"
- **右侧标注**：标 "内部独立 SRAM + 独立 rd_addr：每列对应不同输出像素"

### 标注要求
- 用绿色块表示 SRAM，蓝色块表示累加器；wr_addr/we 总线用粗黑虚线（共享广播）
- 顶部一句话："PARF：内部独立 + 外壳共享的折中设计 — 16 列同步写入仅需一组控制逻辑，但每列累加语义独立"
- 底部一句话："Cin·Ky·Kx 次乘加完成后，rd_addr 按列读出 32-bit PSUM → SDP 量化 → 8-bit OFB"

### 视觉层次
- 主角：16 个 parf_col 单元（最大）
- 配角：顶部共享控制总线
- 背景：底部输出连线

## ASCII 示意稿（简化为 4 列示意，正图为 16 列）

```
   PARF：内部独立 + 外壳共享的折中设计

   ┌─ 共享 wr_addr (5-bit) + we (1-bit)  ──── 跨 16 列同步广播 ────────┐
   │                                                                    │
   ▼              ▼              ▼              ▼              ▼

   ┌─parf_col 0─┐  ┌─parf_col 1─┐  ┌─parf_col 2─┐ ...  ┌─parf_col 15─┐
   │            │  │            │  │            │      │             │
   │ ┌────────┐ │  │ ┌────────┐ │  │ ┌────────┐ │      │ ┌─────────┐ │
   │ │ SRAM 0 │ │  │ │ SRAM 1 │ │  │ │ SRAM 2 │ │      │ │ SRAM 15 │ │
   │ │ depth  │ │  │ │ depth  │ │  │ │ depth  │ │      │ │ depth   │ │
   │ │ 32     │ │  │ │ 32     │ │  │ │ 32     │ │      │ │ 32      │ │
   │ │ ×32 b  │ │  │ │ ×32 b  │ │  │ │ ×32 b  │ │      │ │ ×32 b   │ │
   │ └────┬───┘ │  │ └────┬───┘ │  │ └────┬───┘ │      │ └────┬────┘ │
   │      │ +   │  │      │ +   │  │      │ +   │      │      │ +    │
   │      ▼     │  │      ▼     │  │      ▼     │      │      ▼      │
   │  acc + ←─PSUM_0   acc + ←─PSUM_1  acc + ←─PSUM_2 ... acc + ←PSUM_15
   │  rd_addr 0 │  │  rd_addr 1 │  │  rd_addr 2 │      │  rd_addr 15 │
   │ (独立)     │  │ (独立)     │  │ (独立)     │      │  (独立)     │
   └─────┬──────┘  └─────┬──────┘  └─────┬──────┘      └─────┬───────┘
         │               │                │                   │
         ▼               ▼                ▼                   ▼
       PSUM_0          PSUM_1          PSUM_2          ...  PSUM_15
       32-bit          32-bit          32-bit               32-bit
                            ──→ ofb_writer + SDP 量化 → 8-bit OFB

   ★ 16 列同步写入仅需一组 wr_addr/we 控制（外壳共享，节省全局连线）★
   ★ 每列 SRAM 与 rd_addr 独立（每列对应不同输出像素，语义灵活）★
```

## 数据来源
- paper.md §4.2 段 4
- contributions.md C1.4
- docs/modules/parf_accum.md
- params.py: PARF = 32, PSUM = 32, NUM_COL = 16

## 与正文一致性检查
- [x] 16 个 parf_col + 每列 PARF=32 深度 SRAM — 与 §4.2 / params.py 一致
- [x] 外壳共享 wr_addr/we、内部独立 rd_addr — 与 §4.2 / contributions.md C1.4 一致
- [x] PSUM 32-bit → SDP → 8-bit OFB — 与 §2.2 / §4.2.6 一致

## 不确定项
无。
