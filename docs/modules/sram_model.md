# sram_model

行为级 SRAM 模型，作为 IFB / WB / OFB 三块 SRAM 的 placeholder 用于功能仿真。综合时由真实 SRAM 宏替换。

## 参数

| 参数 | 含义 |
| --- | --- |
| `DEPTH` | 字深度 |
| `DATA_WIDTH` | 字宽（bit） |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk` | in | 时钟 |
| `we / waddr / wdata` | in | 写端口（同步写） |
| `re / raddr / rdata` | in/out | 读端口（同步读，1 拍延迟） |

## 时序

写：`we=1` 当拍 `mem[waddr] ← wdata`（NBA 赋值，下一拍 mem 数组才看到新值）。

读：`re=1` 当拍发起读请求，下一拍 `rdata` 寄存器才输出对应 `mem[raddr]`。`re=0` 当拍下一拍 `rdata` 被清成 0，不保持上次读出值。

读端口和写端口独立，同拍可以一边写一个地址一边读另一个地址。同拍读写同一地址时由于 mem 的 NBA 写后沿才生效，读出仍然是写之前的旧值。

## 数据通路

`mem[]` 数组没有复位，初值为 0（仿真器默认，TB 通过 `$readmemh` 预填）。读出寄存器 `rdata` 在 `re=0` 时清 0，避免读出上次残值。

## 仿真计数器（synthesis translate_off）

- `sram_read_cnt`：累计 `re=1` 拍数
- `sram_write_cnt`：累计 `we=1` 拍数

供 TB 经层次引用读取，做 SRAM 访问数统计。

## 在 core_top 中的实例化

| 实例 | DEPTH | DATA_WIDTH | 用途 |
| --- | ---: | ---: | --- |
| `u_ifb` | 8192 | 128 | 输入特征图（16 channel × 8 bit packed） |
| `u_wb` | 8192 | 2048 | 权重（16 cout × 16 cin × 8 bit packed） |
| `u_ofb` | 8192 | 128 | 输出特征图（16 cout × 8 bit packed） |
