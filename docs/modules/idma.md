# idma（Input DMA）

DDR → IFB SRAM 的 read DMA。AXI4 read master，把整图按行 streaming 拉进 IFB ring buffer：每发一行 AR、收 R 通道写 IFB；ring 满（已灌但未被 line_buffer 消费的行 ≥ strip_rows）时停 AR 等消费。整图装得下 SRAM 时退化为一次性写满。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W / DATA_W / M_ID` | AXI 参数 | 32 / 128 / 2 |
| `SRAM_ADDR_W` | IFB 地址宽 | 13 |
| `LEN_W` | byte_len 宽（保留字段） | 24 |

## 接口

### Control

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start / done / busy` | in/out/out | |
| `src_base` | in | DDR 起始地址 |
| `byte_len` | in | 保留字段，streaming 下不用 |

### Streaming cfg

| 信号 | 含义 |
| --- | --- |
| `cfg_h_in_total` | 整图行数（停止条件） |
| `cfg_ifb_strip_rows` | IFB ring 容纳的行数 |
| `cfg_ifb_ky_step` | 每行 beats（NHWC：W_IN × cin_slices） |
| `cfg_ifb_ring_words` | wr_ptr 的 ring wrap 模数 |
| `rows_consumed` | line_buffer 已用完释放的行数 |
| `rows_available` | IDMA 已写完的行数（输出，给 line_buffer 反压） |

### AXI4 M

只用读通道（AR / R），写通道 AW/W/B 全部 tie 0。

### IFB SRAM 写端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `ifb_we / ifb_waddr / ifb_wdata` | out | IFB SRAM 写 |

## 状态机

```
S_IDLE → S_AR → S_R → S_AR → ... → S_DONE
              ↑       │
              └─ S_WAIT (ring_full)
```

- S_IDLE：等 start
- S_AR：发 ARVALID（除非 ring_full）。ar_fire 后 → S_R。ring_full 时 → S_WAIT
- S_R：接收一整行的 R burst，每 r_fire 写 IFB SRAM。M_RLAST 拍判完一行后：所有行写完 → S_DONE，否则回 S_AR
- S_WAIT：等 ring 不满（rows_consumed 推进）后回 S_AR
- S_DONE：r_done sticky，等下一 case start

## AXI 通道

```
M_ARADDR  = cur_addr
M_ARLEN   = beats_to_issue - 1                  // 最多 256 beat
M_ARBURST = INCR
M_ARVALID = (state == S_AR) && !ring_full

M_RREADY  = (state == S_R)
```

`beats_to_issue = min(row_beats_remaining, 256)`：当前行剩余 beats 不足 256 时一次发完整行；超过则分批。

## Ring 反压

```
ring_full = ((rows_written - rows_consumed) >= cfg_ifb_strip_rows)
```

ring_full 时 ARVALID 拉低，状态去 S_WAIT 等 line_buffer 推进 rows_consumed。

## 行计数

```
row_beats_remaining: start 时初始化为 cfg_ifb_ky_step；r_fire 减 1；行末（streaming_row_end）回到 cfg_ifb_ky_step
streaming_row_end  = r_fire && (row_beats_remaining == 1)
streaming_all_done = streaming_row_end && (rows_written == cfg_h_in_total - 1)
rows_written       += 1   on streaming_row_end
```

## IFB 地址

```
ifb_we    = r_fire
ifb_waddr = wr_ptr
ifb_wdata = M_RDATA

wr_ptr: start 时 0; r_fire 时 +1; 到 cfg_ifb_ring_words - 1 时下次回 0 (ring wrap)
```

## 数据通路与控制路径

- 数据路径无复位：cur_addr、row_beats_remaining、rows_written、wr_ptr
- 控制路径同步复位：state、r_done

`done = r_done && !start`，多 case 重入 start 当拍清 0。

## 在 core_top 中的位置

实例 `u_idma`：
- AXI4 M[0]（mux 第 0 路）
- start 来自 sequencer.start_idma_pulse；src_base 来自 cfg + sequencer 的 strip_ifb_ddr_offset 叠加
- IFB SRAM 写端口与 axi_lite_csr 的预填路径在 core_top 用 `or` 合并（idma_we | tb_preload_we）
- rows_consumed/rows_available 与 `u_line_buffer` 双向交互
