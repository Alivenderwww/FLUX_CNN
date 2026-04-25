# desc_fifo

DFF 实现的同步 FIFO，连接 DFE（writer，来自 DDR 的 descriptor 流）和 Sequencer（reader，按 descriptor 设 cfg + 启动 DMA / core）。同时钟域，深度 32，宽度 256（一个 descriptor=2 个 128-bit AXI beat 拼成）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `DEPTH` | FIFO 项数 | 32 |
| `WIDTH` | 每项位宽 | 256 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `clk / rst_n` | in | 时钟与复位 |
| `wr_en / wr_data` | in | DFE 写端口 |
| `full` | out | 写满（DFE 看到 full 反压不再 push） |
| `rd_en / rd_data` | in/out | Sequencer 读端口 |
| `empty` | out | 空（Sequencer 看到 empty 等待） |
| `count` | out | 当前占用项数（debug/观测用） |

## 内部结构

`mem[DEPTH]` 是 256-bit 宽存储阵列，无复位。指针 `wr_ptr / rd_ptr` 都是 5-bit（`$clog2(32)`），按 ring 推进，到达 `DEPTH-1` 后下一拍归 0。`r_count` 是 6-bit 占用计数器（0..32）。

`do_wr = wr_en && !full`，`do_rd = rd_en && !empty`。同拍 push 同时 pop 不支持：full 时 push 被拒，empty 时 pop 被拒。

## 时序

写：`wr_en=1` 且 `full=0` 时，当拍 `mem[wr_ptr] ← wr_data`，下一拍 `wr_ptr` 推进，`r_count` +1。

读：`rd_data = mem[rd_ptr]` 是组合输出，FIFO 头部数据立即在 `rd_data` 上可见。`rd_en=1` 且 `empty=0` 时下一拍 `rd_ptr` 推进，`r_count` -1。读取者看到的"出队"是下一拍 `rd_data` 切到下一项。

`full` 和 `empty` 由 `r_count` 组合译码：count==DEPTH 时 full，count==0 时 empty。

## 数据通路与控制路径

`mem[]` 没有复位（数据路径，按设计原则不复位）。`wr_ptr / rd_ptr / r_count` 复位，因为它们决定 `full / empty / count` 这些下游可观测信号。

## 在 core_top 中的位置

实例 `u_desc_fifo`，深度 32 宽 256：
- DFE（`u_dfe`）每收到 256-bit descriptor（= 2 拍 128-bit AXI R 通道 burst 拼接）就 push 一次
- Sequencer（`u_sequencer`）按 layer 状态机的 SUBOP 阶段 pop 一个 descriptor，解码后写入 cfg_regs / 启动相应 DMA
