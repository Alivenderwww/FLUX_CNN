# parf_col

PARF（Partial-sum Register File）的单列 SRAM 存储。`parf_accum` 内部例化 `NUM_COL=16` 个 `parf_col`，每列各自一块 SRAM。当前所有列共享 wr_addr / we / rd_addr，只是 wdata 切片不同——拆成 per-col 子模块是为后续给每列做独立寻址留扩展空间。

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

`parf_accum` 例化 16 个 `parf_col` 实例（generate for 循环）：
- `we`：所有列共享 `fill_fire`
- `wr_addr / rd_addr`：所有列共享，由外壳 fill / drain 计数器生成
- `wdata`：每列从 `psum_in_vec` 切自己那 32 bit 段
- `rdata`：每列输出 32 bit，拼成 `acc_out_vec` 总线
- `old_at_wr`：每列输出 32 bit，拼成 `old_psum_at_wr` 送回 `mac_array` 做 acc_seed
