# mac_pe

单个 PE（处理单元），包含一个 8-bit × 8-bit signed 乘法器和一个 32 项的局部权重寄存器堆 WRF。是 16×16 PE 阵列的最小单元。

## 参数

| 参数 | 含义 |
| --- | --- |
| `DATA_WIDTH` | 激活和权重位宽，固定 8 |
| `WRF_DEPTH` | 局部 WRF 深度，固定 32 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk / rst_n` | in | 时钟与复位 |
| `wrf_we / wrf_waddr / wrf_wdata` | in | WRF 写端口（来自 wgt_buffer 的逐 PE 灌权重） |
| `wrf_raddr` | in | WRF 读地址（来自 mac_array 的全局 (kx, ky) 计数器） |
| `act_in` | in | 输入激活（来自 mac_array 同列的同行像素，按行广播；signed 8-bit） |
| `compute_en` | in | 流水推进闸门 |
| `prod_out` | out | 乘积寄存器输出（signed 16-bit） |

## 内部结构

例化一个 `std_rf` 作为 WRF：写口接外部 `wrf_we/waddr/wdata`，读口 `wrf_raddr` 组合输出 `active_weight_us`。`active_weight = signed'(active_weight_us)` 把存储里 unsigned 字节当 signed 读出。

乘法寄存一拍：`prod_out` 在 `compute_en=1` 当拍后沿更新成 `act_in × active_weight`，`compute_en=0` 当拍保持上一拍的 `prod_out`。

## 时序

权重写入：`wrf_we=1` 当拍写 WRF，下一拍即可被读出。`wrf_raddr` 组合译码，没有读寄存延迟。

乘法流水：T 拍当 `compute_en=1` 时，T 拍 `act_in` 和 `wrf_raddr` 决定的 weight 进入乘法器，T+1 拍后沿 `prod_out` 寄存好这两值的乘积。`compute_en=0` 当拍冻结 `prod_out`，让上游已算好的乘积顺着下游流水继续移动。

`act_in` 与 `wrf_raddr` 在同一拍由 `mac_array` 提供，无需 PE 内部对齐。

## 数据通路（无复位，按数据路径处理）

`u_wrf.ram` 数组没有复位（仿真时上电 X，靠"先写后读"契约保证）。`prod_out` 寄存器没有复位，上电 X 不影响下游正确性，因为 `mac_col` 加法树后续的 valid 寄存器会遮蔽 pipeline 启动期未稳定值。

## 在阵列中的位置

`mac_array` 有 16 列 × 16 行 = 256 个 `mac_pe`：
- 列索引 `c` 决定 PE 的 cout 维（接收同 cout 的权重）
- 行索引 `p` 决定 PE 的 cin 维（接收同 cin 的输入广播）
- `act_in` 由 `mac_array` 把 16 个 cin 通道的输入按行展开广播到 16 列
- `wrf_raddr` 是 `(ky, kx)` 全局计数器，所有 256 个 PE 共享相同 raddr（每 PE 用各自 WRF 里 (ky, kx) 处的权重）
