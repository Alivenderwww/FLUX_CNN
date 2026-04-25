# mac_col

PE 列的封装：`NUM_PE=16` 个 `mac_pe` + 一棵 16-input 全组合加法树 + 1 拍输出寄存器。代表 PE 阵列的一列（一个 cout）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_PE` | 列内 PE 数（cin 维） | 16 |
| `DATA_WIDTH` | 激活/权重位宽 | 8 |
| `PROD_WIDTH` | 单个乘积位宽 | 16 |
| `PSUM_WIDTH` | psum 位宽 | 32 |
| `WRF_DEPTH` | 每 PE WRF 深度 | 32 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk / rst_n` | in | 时钟与复位 |
| `wrf_we[NUM_PE]` | in | 每 PE 独立写使能（16 bit，每位对应一个 cin 行的 PE） |
| `wrf_waddr` | in | 共用写地址（所有 PE 用同一个 waddr） |
| `wrf_wdata[NUM_PE×8]` | in | 16 个 PE 的写数据拼接（每 PE 各取 8 bit） |
| `wrf_raddr` | in | 共用读地址（所有 PE 用同一 raddr） |
| `act_in_vec[NUM_PE×8]` | in | 16 个 cin 通道的输入像素拼接（一拍一个像素位置的 16 cin） |
| `compute_en` | in | 流水推进闸门 |
| `psum_out` | out | 16 个 PE 乘积之和（signed 32 bit） |

## 内部结构

### 乘法器阵列
`generate for` 例化 16 个 `mac_pe`：
- 每 PE 接收 `wrf_we[i]`（独立）+ 共用的 `wrf_waddr/wdata` 切片 + 共用的 `wrf_raddr`
- 每 PE 的 `act_in` 取 `act_in_vec` 的对应 8-bit 片段（即第 i 个 cin 通道的像素）
- 每 PE 的 `prod_out` 进 `prod_array[i]`

### 空间加法树
`always_comb` 把 16 个 `prod_array[p]` 累加到 `adder_tree_out`（signed 32 bit）。综合时由工具决定是平衡二叉树还是其他形式，行为级是顺序累加。

### 加法树寄存
`adder_tree_reg` 在 `compute_en=1` 当拍后沿寄存 `adder_tree_out`，输出到 `psum_out`。`compute_en=0` 时保持。

## 时序

`compute_en` 上升后到 `psum_out` 出现对应输出有 **2 拍**固定延迟：

- T 拍：`compute_en=1`，`act_in_vec` 和 `wrf_raddr` 提供输入
- T+1 拍：`mac_pe.prod_out` 寄存好乘积，加法树组合算出 `adder_tree_out`
- T+2 拍：`adder_tree_reg` 寄存好和，`psum_out` 出现 T 拍输入对应的列累加结果

`compute_en=0` 时整列冻结，已在 pipe 里的乘积保持，直到 `compute_en` 重新拉高继续推进。

## 数据通路（无复位）

`adder_tree_reg` 没有复位寄存器。`mac_pe.prod_out` 也没有复位。上电 X 不影响下游正确性，因为 `mac_array` 的外层 valid 流水跟踪寄存器在数据未稳定的启动期不会让 `psum_out_valid` 拉高。

## 在阵列中的位置

`mac_array` 例化 `NUM_COL=16` 个 `mac_col`：
- 每列代表一个 cout 通道
- 16 列共享 `wrf_raddr / act_in_vec / compute_en`（cin 维输入广播给所有 cout 列）
- 每列有独立的 `wrf_we[NUM_PE]`，由 `wgt_buffer` 路由（每拍只给一个 (col, pe) 对位置写权重）
- 每列 `wrf_wdata` 切自 `wgt_buffer` 输出的 2048-bit 总线
