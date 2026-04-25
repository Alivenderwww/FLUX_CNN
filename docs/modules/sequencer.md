# sequencer

层级控制器：从 desc_fifo 取一条 descriptor 解析，按 strip 粒度发 start pulse 给 core 流水（line_buffer / ofb_writer）、wgt_buffer（仅 is_first strip）、IDMA / ODMA（每 strip）、WDMA（仅 is_first strip）。等待 strip 完成、pop 下一条，直到 END 描述符。

## 接口

### CTRL

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start_layer_pulse` | in | host 写 CTRL[5]=1 触发的 1 拍脉冲 |
| `layer_busy / layer_done` | out | layer 层级状态汇报 |

### FIFO 读端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `fifo_rd_data[256]` | in | 当前 FIFO 头部 descriptor |
| `fifo_empty` | in | FIFO 空标志 |
| `fifo_rd_en` | out | pop 当前头部（在 S_FETCH 状态当拍发） |

### Layer-level cfg 输入

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `cfg_h_out_total` | in | layer 完整高度（IDLE 期作为 strip_n_yout 的安全默认） |

### Strip-level cfg 输出

每条 conv descriptor 解码出的 per-strip 参数，给 line_buffer / ofb_writer 用：

| 信号 | 含义 |
| --- | --- |
| `strip_n_yout` | 这条 strip 处理几行 yout |
| `strip_pad_top / pad_bot / pad_left / pad_right` | strip 边界 pad |
| `strip_y_start` | strip 在整图里的 y 起点 |
| `strip_ifb_ddr_offset / byte_len` | IDMA 的 src 偏移和长度 |
| `strip_ofb_ddr_offset / byte_len` | ODMA 的 dst 偏移和长度 |

### Start pulse

| 信号 | 含义 |
| --- | --- |
| `start_core_pulse` | line_buffer / ofb_writer 启动（每 strip） |
| `start_wgt_pulse` | wgt_buffer 启动（仅 is_first，整 layer 一次） |
| `start_idma_pulse` | IDMA 启动（每 strip） |
| `start_odma_pulse` | ODMA 启动（每 strip） |
| `start_wdma_pulse` | WDMA 启动（仅 is_first，整 layer 一次） |

### Done aggregation

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `core_strip_done` | in | line_buffer.done && ofb_writer.done 联合 |
| `idma_strip_done / odma_strip_done` | in | DMA per-strip 完成 |
| `wdma_done` | in | WDMA 完成 |

## Descriptor 字段布局（256 bit = 8 word，little-endian）

```
word 0 [3:0]    type           0=NOP 1=CONV 2=BARRIER F=END
word 0 [7:4]    flags          {rsvd, streaming_en, is_last, is_first}
word 0 [11:8]   pad_top
word 0 [15:12]  pad_bot
word 0 [19:16]  pad_left
word 0 [23:20]  pad_right
word 1 [15:0]   strip_y_start
word 1 [31:16]  n_yout_strip
word 2 [19:0]   ifb_ddr_offset (bytes 相对 cfg_idma_src_base)
word 3 [23:0]   ifb_byte_len
word 4 [19:0]   ofb_ddr_offset (bytes 相对 cfg_odma_dst_base)
word 5 [23:0]   ofb_byte_len
word 6,7        rsvd
```

`flags[6]` (FLAG_STREAMING_EN) 是历史字段，硬件恒走 streaming 路径，不再解析。

## 状态机

```
S_IDLE → S_FETCH → S_PRELOAD → S_DISPATCH → S_WAIT → S_FETCH → ...
   ↑         ↓ NOP                                       ↓
   └─── S_END ←──────── S_BARRIER ←──────────────────────┘
```

各状态：

- **S_IDLE**：等 `start_layer_pulse`
- **S_FETCH**：从 FIFO pop 一条 descriptor。若 FIFO empty 则 stall。pop 成功且 type==CONV 时 latch 字段。按 type 跳转：
  - NOP：留在 S_FETCH
  - CONV is_first：→ S_PRELOAD
  - CONV !is_first：→ S_DISPATCH
  - BARRIER：→ S_BARRIER
  - END：→ S_END
- **S_PRELOAD**：进入第一拍发 `start_wdma_pulse`（仅 is_first）。等 `wdma_done`，然后 → S_DISPATCH。如果 `r_is_first=0` 直接跳过（实际不会进 PRELOAD，FETCH 已经分流）
- **S_DISPATCH**：当拍同时发 `start_core_pulse / start_idma_pulse / start_odma_pulse`，is_first 时还发 `start_wgt_pulse`。下一拍进 S_WAIT
- **S_WAIT**：等 `core_strip_done && idma_strip_done && odma_strip_done`，全 done 后回 S_FETCH
- **S_BARRIER**：等所有 in-flight DMA + core 完成（含 wdma_done），然后回 S_FETCH
- **S_END**：等所有 DMA 完成，回 S_IDLE，set `r_layer_done=1`

## 字段 latch

`latch_conv = fetch_ok && (hd_type == TYPE_CONV)`，当拍把 hd_* 寄存到 `r_*`：strip_n_yout、pad、y_start、ifb/ofb_ddr_offset、ifb/ofb_byte_len、is_first。

## Start pulse 生成

```
start_core_pulse = (state == S_DISPATCH)
start_wgt_pulse  = (state == S_DISPATCH) && r_is_first
start_idma_pulse = (state == S_DISPATCH)
start_odma_pulse = (state == S_DISPATCH)
start_wdma_pulse = preload_entry && r_is_first
```

`preload_entry` 是 S_PRELOAD 进入的第一拍：`(state == S_PRELOAD) && !r_prev_state_preload`。

## Strip cfg 输出 mux

```
strip_n_yout = layer_busy ? r_strip_n_yout : cfg_h_out_total
strip_pad_*  = layer_busy ? r_pad_*        : 0
strip_y_start= layer_busy ? r_strip_y_start: 0
```

IDLE 时给 line_buffer / ofb_writer 一个安全默认（cfg_h_out_total 整图，不会触发实际 issue 因为 start 没拉）。

## layer_done sticky

```
r_layer_done <= 1   when (state == S_END && state_next == S_IDLE)
r_layer_done <= 0   when start_layer_pulse
```

`layer_done = r_layer_done`，host 看到 layer_done=1 后再写 start_layer_pulse 启动下一 layer 时自动清 0。

## 数据通路与控制路径

- 数据路径无复位：descriptor latch 字段（`r_strip_n_yout / r_pad_* / r_strip_y_start / r_ifb_*  / r_ofb_* / r_is_first`）
- 控制路径同步复位：`state`、`r_layer_done`、`r_prev_state_preload`

## 在 core_top 中的位置

实例 `u_sequencer`：
- 上游：`u_desc_fifo` 的读端口
- start_layer_pulse 来自 `u_cfg.start_layer_pulse`
- start_core_pulse → `u_line_buffer.start` + `u_ofb_writer.start`
- start_wgt_pulse → `u_wgt_buffer.start`
- start_*dma_pulse → 各 DMA 的 start
- core_strip_done = `u_line_buffer.done && u_ofb_writer.done`
- IDMA / ODMA / WDMA 的 done 来自各自模块
- strip-level cfg 输出叠加到 cfg_regs 的 layer-level base 形成 IDMA / ODMA 的最终 src/dst（在 core_top 里做加法）
