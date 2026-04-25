# parf_col

PARF（Partial-sum Register File）的单列 SRAM 存储。`parf_accum` 内部例化 `NUM_COL=16` 个 `parf_col`，每列独立读写地址和写使能，使 Kx-fold 可以让不同列组按不同偏移写 psum。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `PSUM_WIDTH` | 每条 psum 位宽 | 32 |
| `PARF_DEPTH` | 列内 psum 项数 | 32 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk` | in | 时钟（无复位） |
| `we / wr_addr / wdata` | in | 写端口 |
| `rd_addr / rdata` | in/out | 读端口（组合读，drain 用） |
| `old_at_wr` | out | 当前 `wr_addr` 位置的旧值（组合读，给 mac_array 做 acc_seed 融合用） |

## 内部结构

`mem[]` 是 PARF_DEPTH × PSUM_WIDTH 的存储阵列。同步写、双口组合读：
- `we=1` 当拍 `mem[wr_addr] ← wdata`（NBA）
- `rdata = mem[rd_addr]` 组合读
- `old_at_wr = mem[wr_addr]` 组合读

两个组合读端口（`rd_addr` 和 `wr_addr`）独立，可以同拍读两个不同地址。

## 时序

写：`we=1` 当拍写入，下一拍 mem 反映新值。

读：`rdata` 跟随 `rd_addr` 组合变化，无寄存器延迟。`old_at_wr` 跟随 `wr_addr` 组合变化。

写读同拍同地址时，由于写是 NBA，当拍 `rdata`/`old_at_wr` 仍读出旧值，下一拍才读出新值。这个行为是设计意图：mac_array 的 acc_seed 融合就靠这一拍延迟拿到旧 psum 累加上新乘积。

## 数据通路（无复位）

`mem[]` 没有复位。上电 X 不影响输出正确性，因为外层 `parf_accum` 通过 `acc_out_valid` 信号在 PARF 没被填过的启动期遮蔽 `rdata`。

## 在 parf_accum 中的位置

`parf_accum` 例化 16 个 `parf_col` 实例（generate for 循环），并对每列分别生成：
- 每列的 `we_col[c]`：基于 fill_fire 和该列的 `wr_addr_col[c]` 是否在合法范围内 `[0, cur_valid_w_fill)`
- 每列的 `wr_addr_col[c]`：`wr_addr - col_group[c] × cfg_fold_col_shift`，其中 col_group 由 cfg_fold_cout_orig 决定
- 共用的 `rd_addr`：drain 阶段的读地址
- 共用的 `wdata`：mac_array 输出的 psum 切片

每列的 `old_at_wr[c]` 拼成 16-列宽总线送回 `parf_accum`，再分发给 `mac_array` 做 acc_seed。
