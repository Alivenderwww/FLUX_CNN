# 图 4.2 流式行环数据通路示意图
# Figure 4.2 Schematic diagram of streaming row-ring datapath

## 在论文中的角色
- 首次引入：§4.2 段 "第二层骨架是行级流式行环数据通路..." [依赖: Fig.4.2]
- 论证作用：本图是 narrative B（系统集成）支撑的关键示意——展示 IFB ring（strip_rows × W_IN）+ row-credit 反压协议 + OFB ring 的工作机理。是 §3.4.3 与 Alwani / Kang 差异化论证的图示根据：缓冲粒度从"层数级 / 整图驻留"降到"行级"。

## 图类型
环形 / 滚筒图：左侧 IFB ring（strip_rows 行 × W_IN 列），中间 mac_array 与 SDP 流水，右侧 OFB ring（strip_rows_out 行 × W_OUT 列），上下加 idma / odma。两侧用 row-credit 反压箭头双向连接。

## 设计要素

### 必含元素
- **左侧 IFB ring**：
  - 一个垂直滚筒结构，画 `strip_rows = 8` 行 × W_IN 列示意（W_IN 用 ... 简化表示）
  - 写指针 wp（绿色，由 idma_ctrl 推进，标 "← idma 加载"）
  - 读指针 rp（蓝色，由 mac_array 滑窗推进）
  - 容量标注：`IFB = 8192 word`
  - row-credit：从 mac_array 反压到 idma_ctrl，标 "credit++ (一行消费完)" / "credit-- (一行写入)"
- **中间 mac_array + parf + SDP 流水**：标 "图 4.1 五模块流水"
- **右侧 OFB ring**：
  - 类似 IFB 的滚筒结构，画 `strip_rows_out` 行 × W_OUT 列
  - 写指针 wp（由 ofb_writer 推进）
  - 读指针 rp（由 odma_ctrl 推进 → "→ odma 写回"）
  - 容量标注：`OFB = 2048 word`
  - 编译器决策：`ofb_strip_rows_max = (OFB - 1) // row_words`
- **顶部一行小字**：`strip_rows × W_IN 行级 ring：BRAM 占用与输入特征图行数 H 解耦`

### 标注要求
- 行级 ring 与 §2.4 通用 line buffer ring（图 2.4）的差异：strip_rows 由编译器静态决策（`ofb_strip_rows_max`），不是固定 Ky；同时引入 row-credit 反压
- 在 IFB ring 顶部加一句："VGA 480×640 单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张"
- 用对比色：IFB ring 蓝色，OFB ring 绿色，row-credit 反压箭头红色

### 视觉层次
- 主角：左右两个 ring
- 配角：中间 mac_array 流水（用方块简化）
- 背景：row-credit 反压箭头 + 顶部一句话

## ASCII 示意稿

```
   strip_rows × W_IN 行级 ring：BRAM 占用与 H 解耦
   VGA 480×640 单图 4.9 MB 仅用约 10 KB ring 即可单 start 跑完整张

  ┌────────────────────┐                         ┌────────────────────┐
  │  IFB ring (8192 w) │  → mac_array →          │  OFB ring (2048 w) │
  │  strip_rows × W_IN │   parf_accum →          │  strip_rows_out ×  │
  │                    │   SDP 后处理            │  W_OUT             │
  │  ┌──────────────┐  │                         │  ┌──────────────┐  │
  │  │ row 0 ▒▒▒▒▒▒ │←─wp (idma)               │  │ row 0 ░░░░░░ │←─wp (ofb)
  │  │ row 1 ▒▒▒▒▒▒ │                          │  │ row 1 ░░░░░░ │
  │  │ row 2 ▒▒▒▒▒▒ │←─rp (mac 滑窗)           │  │ row 2 ░░░░░░ │←─rp (odma)
  │  │ row 3 ▒▒▒▒▒▒ │                          │  │ row 3 ░░░░░░ │
  │  │ row 4 ▒▒▒▒▒▒ │                          │  │ ...          │
  │  │ row 5 ▒▒▒▒▒▒ │                          │  │              │
  │  │ row 6 ▒▒▒▒▒▒ │                          │  │              │
  │  │ row 7 ▒▒▒▒▒▒ │                          │  │              │
  │  └──────────────┘                          │  └──────────────┘
  └────────────────────┘                         └────────────────────┘
        ↑↓ row-credit                                    ↑↓ row-credit
        反压协议                                          反压协议
        ┌─────────────────────────────────────────────────────────┐
        │ idma_ctrl ──加载一行──→ IFB.credit--                    │
        │ mac_array ──消费一行──→ IFB.credit++                    │
        │ ofb_writer ──写一行──→ OFB.credit--                     │
        │ odma_ctrl ──读出一行──→ OFB.credit++                    │
        └─────────────────────────────────────────────────────────┘
        
  编译器静态决策：strip_rows = (IFB - 1) // row_words 等
                  ofb_strip_rows_max = (OFB - 1) // row_words

  与 §2.4 通用 line buffer 的差异：
  · strip_rows 由编译器决定，非固定 Ky
  · 引入 row-credit 反压，生产 / 消费方解耦
  · 单层硬件可处理任意 H × W 输入
```

## 数据来源
- paper.md §4.2 段 2 / §3.4.3
- contributions.md C1.3
- docs/modules/line_buffer.md / ofb_writer.md
- RTL/ofb_writer.sv（commit 5fe16b2）

## 与正文一致性检查
- [x] IFB = 8192 word / OFB = 2048 word — 与 CLAUDE.md / params.py 一致
- [x] `ofb_strip_rows_max = (OFB - 1) // row_words` — 与 §3.4.3 / §4.2 一致
- [x] VGA 480×640 单图 ~10 KB ring — 与 §3.4.3 / contributions.md C1.3 一致
- [x] row-credit 反压协议 — 与 §3.4.3 / §4.2 一致

## 不确定项
无。
