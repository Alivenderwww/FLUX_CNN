# std_rf

标准寄存器文件。同步写、组合读。用于 PE 内部存权重（WRF）和 ARF 项寄存器场景。

## 参数

| 参数 | 含义 |
| --- | --- |
| `DATA_WIDTH` | 字宽 |
| `DEPTH` | 深度 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk` | in | 时钟 |
| `rst_n` | in | 异步复位低有效（只复位仿真计数器，不复位 ram） |
| `we / waddr / wdata` | in | 写端口（同步写） |
| `raddr / rdata` | in/out | 读端口（组合读，0 拍延迟） |

## 时序

写：`we=1` 当拍 `ram[waddr] ← wdata`（NBA），下一拍 ram 数组反映新值。

读：`rdata = ram[raddr]` 是组合赋值，`raddr` 当拍变化 `rdata` 同拍跟随变化，无寄存器延迟。

写读同拍同地址：`we` 当拍读出仍是旧值（NBA 赋值在当拍读完后才生效），下一拍才能读到新写入值。

## 数据通路

`ram[]` 无复位、无初值。仿真时上电 X，但调用方契约保证"先写后读"（如 `wgt_buffer` 的 LOAD 阶段灌满后 COMPUTE 才读），上电 X 不产生可观测污染。

## 仿真计数器（synthesis translate_off）

- `total_write_ops`：累计写次数
- `first_write_logged`：首次写后置 1，可启用 `$display` 输出首写日志（默认注释）

## 在项目中的使用

每个 PE（`mac_pe`）内部例化一个 `std_rf` 作 WRF：
- `DATA_WIDTH=8`，`DEPTH=WRF_DEPTH=32`
- 存这 PE 自己的 K² 个权重（K≤7 时 K²≤49 > 32 时分轮加载，K=1 仅 1 个权重）
