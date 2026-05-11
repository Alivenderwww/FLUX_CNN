# 图 4.3 部分和累加模块结构图
# Figure 4.3 Partial-sum accumulator module structure

## 在论文中的角色
- 首次引入：§4.4 部分和累加模块
- 引用位置：paper.md 第 310、312 行
- 论证作用：展示 PARF 在物理上拆分为 16 个独立 SRAM (parf_col × NUM_COL=16)、每列共享 wr_addr/we 的外壳结构，以及"读出累加值→相加→回写→cin_slice 完成时发射"的三步流程。

## 图类型
模块结构图（含 16 列独立 SRAM）。

## 设计要素

### 必含元素
1. **16 列独立 SRAM**：横向排列 16 个小 SRAM 块，每块标 "parf_col[i]"，容量 32 项。
2. **共享写地址 / 写使能控制器**（外壳）：上方一根横线拉到所有 16 列 SRAM，标 "shared wr_addr / we"。
3. **16 路读数据通路**：每列 SRAM 各自的读端口独立，标 "col-private read".
4. **16 个加法器**：每列一个，把当前周期 PSUM 与读出累加值相加。
5. **写回路径**：加法器输出回写 SRAM。
6. **cin_slice=0 跳过读出 mux**：每列加法器前的 mux 选择 "0 (init)" 或 "PARF read"。
7. **发射逻辑**：当 (*tile_w*, *cout_col*) 槽位完成所有 cin_slice 累加，向下游发射。
8. **本地计数器**：(*tile_w*, *cin_slice*) 二级。

### 标注要求
- PARF=32 容量
- 16 列独立 SRAM 实例 = parf_col × NUM_COL
- 写地址共享，读数据独立
- 上游：16 路 PSUM (32b)；下游：16 路 accum-done (32b)

### 视觉层次
- 主角：16 列独立 SRAM 阵列
- 配角：共享 wr_addr/we 外壳、16 个加法器、cin_slice=0 mux
- 背景：cfg 输入、本地计数器

## 数据来源
- paper.md §4.4
- docs/modules/parf_accum.md
- CLAUDE.md "parf_accum 内部由 parf_col × NUM_COL 组成"

## ASCII 示意稿

```
                        ┌─── shared wr_addr / we (driven by outer shell) ───┐
                        │                                                    │
   from mac_array  ────┐│                                                    │
   16× PSUM (32b) v/r  ││                                                    │
                       ▼▼                                                    │
   col 0    col 1    col 2    col 3   ...   col 14   col 15                  │
   ┌───┐    ┌───┐    ┌───┐    ┌───┐         ┌───┐    ┌───┐                   │
   │ + │    │ + │    │ + │    │ + │   ...   │ + │    │ + │   (16 adders)     │
   └─▲─┘    └─▲─┘    └─▲─┘    └─▲─┘         └─▲─┘    └─▲─┘                   │
     │mux     │mux     │mux     │mux           │mux     │mux                 │
     │(0/rd)  │(0/rd)  │(0/rd)  │(0/rd)        │(0/rd)  │(0/rd) ◀── cin_s=0 │
     │        │        │        │              │        │                    │
   ┌─▼─┐    ┌─▼─┐    ┌─▼─┐    ┌─▼─┐         ┌─▼─┐    ┌─▼─┐                   │
   │SRAM    │SRAM    │SRAM    │SRAM         │SRAM    │SRAM   PARF=32 each    │
   │parf0   │parf1   │parf2   │parf3   ...  │parf14  │parf15 (private read,  │
   │32×32b  │32×32b  │32×32b  │32×32b       │32×32b  │32×32b  shared write)  │
   └────┬───┘└────┬──┘└────┬──┘└────┬──┘    └────┬───┘└────┬──┘              │
        │        │         │        │              │        │                │
        │        │         │        │              │        │                │
        │        │  (read after cin_slice complete) │        │                │
        ▼        ▼         ▼        ▼              ▼        ▼                │
       ┌─────────────────────────────────────────────────────┐               │
       │  emit logic — when all cin_slice done for this tile │ ──────────────┘
       └────────────────────────────┬────────────────────────┘
                                    │
                                    ▼  16× accum-done PSUM (32b) v/r
                              to ofb_writer

   local counters: (tile_w, cin_slice) — 2 levels
   cfg from cfg_regs: cin_slice_total, tile_w_max
```

## 与正文的一致性检查
- [x] §4.4 "16 列共享同一组写地址（wr_addr）与写使能（we），由外壳模块统一驱动；每列的写数据来自本列 mac_array PSUM" — 图中外壳横线 + 列私有数据
- [x] §4.4 "cin_slice=0 时跳过读出，直接写入新值" — 图中 mux 选择 0 路径
- [x] §4.4 "PARF=32 容量对应一个 cout_slice 内 32 个 tile_w 输出位置" — 容量在图注

## 不确定项
- [TBD: 16 列在图上挤不下，建议绘图阶段用 4 列示意 + "..." 省略号]
