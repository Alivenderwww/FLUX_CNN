# ofb_writer

接收 `psum_reshape` 输出的 16 路 psum，过 SDP 量化成 16 路 int8，写入 OFB SRAM。同时维护 row-level credit 给 ODMA（streaming 模式下）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL` | cout 通道数 | 16 |
| `DATA_WIDTH` | 输出位宽 | 8 |
| `PSUM_WIDTH` | psum 位宽 | 32 |
| `SRAM_DEPTH / ADDR_W` | OFB SRAM 深度 / 地址宽 | 8192 / 20 |

## 接口

### cfg
- `cfg_h_out`：输出高
- `cfg_tile_w / cfg_last_valid_w / cfg_num_tiles`：tile 划分
- `cfg_cout_slices`：cout 切片数
- `cfg_ofb_base / cfg_ofb_ring_words / cfg_ofb_strip_rows`：OFB 物理参数
- `cfg_sdp_*`：SDP 全套参数（`shift / mult / zp_out / clip_min / clip_max / round_en / relu_en`）

### 启停

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start / done` | in / out | 与 sequencer 的握手 |

### 上游（来自 parf_accum 经 psum_reshape）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `acc_out_valid / acc_out_ready / acc_out_vec[NUM_COL×32]` | in/out/in | 16 路 psum |

### OFB SRAM 写端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `ofb_we / ofb_waddr / ofb_wdata[NUM_COL×8]` | out | 写 OFB SRAM |

### Streaming row credit（给 ODMA）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `rows_drained` | in | ODMA 已搬走的 OFM 行数 |
| `row_done_pulse` | out | 每完成一个 yout 全部 cs 段写完时发一拍 |
| `rows_written` | out | 累计已写完的 OFM 行数 |

## 状态机

```
S_IDLE → S_RUN → S_DONE → (start 重入)
```

`done = (state == S_DONE) && !start`，多 case 重入时 start 同拍 done 立即掉 0。

## 四层计数器

按从内到外：`x_cnt → tile_cnt → cs_cnt → yout_cnt`。注意 cs 在 yout 内：`for yout: for cs: for tile: for x`。

```
cur_valid_w = (tile == last) ? cfg_last_valid_w : cfg_tile_w
x_is_last   = (x_cnt    == cur_valid_w     - 1)
tile_is_last= (tile_cnt == cfg_num_tiles   - 1)
cs_is_last  = (cs_cnt   == cfg_cout_slices - 1)
yout_is_last= (yout_cnt == cfg_h_out       - 1)
all_done    = x_is_last && tile_is_last && cs_is_last && yout_is_last
```

推进事件（comb）：

```
evt_fire_x_wrap    = acc_fire && x_is_last
evt_fire_tile_wrap = evt_fire_x_wrap    && tile_is_last
evt_fire_cs_wrap   = evt_fire_tile_wrap && cs_is_last
evt_fire_yout_wrap = evt_fire_cs_wrap   && yout_is_last
```

每层 always_ff 用 if-elseif 链，优先级：start > yout_wrap > cs_wrap > tile_wrap > x_wrap > +1 > 自保持。

## 握手与 ring 反压

```
ring_full     = ((rows_written - rows_drained) >= cfg_ofb_strip_rows)
acc_out_ready = (state == S_RUN) && !ring_full
acc_fire      = acc_out_valid && acc_out_ready
```

ring_full：写满 strip_rows 行而 ODMA 还没搬走时反压。整图装得下 OFB 时 `cfg_ofb_strip_rows = H_OUT`，ring 永远不满。

## SDP 直通

`u_sdp` 例化在内部，`acc_out_vec` 当拍组合输出 `sdp_out`。SDP 是纯组合，零延迟。

## OFB 写

```
ofb_we    = acc_fire
ofb_waddr = ofb_ptr
ofb_wdata = sdp_out
```

`ofb_ptr` 在 `evt_start` 拍初始化为 `cfg_ofb_base`，每 `acc_fire` +1，到达 `cfg_ofb_base + cfg_ofb_ring_words - 1` 时下一 fire 回 `cfg_ofb_base`（ring wrap）。物理上每 fire 写 1 个 word（16 cout × 8 bit）。

OFB 物理布局是连续的（无内部 padding）：每 yout 写 W_OUT × cout_slices 个 word（NHWC，一行内所有 cs 段相邻）。整 strip 写 `H_OUT × W_OUT × cout_slices` 个 word。

## Row credit

`row_done_pulse = evt_fire_cs_wrap`：当一个 yout 的所有 cs 段都写完时发一拍。`rows_written` 计数器在每个 `evt_fire_cs_wrap` +1，复位（控制路径，被 ODMA 看到）。

## 数据通路与控制路径

- 数据路径无复位：x_cnt、tile_cnt、cs_cnt、yout_cnt、ofb_ptr
- 控制路径同步复位：state、rows_written

无复位的 4 层计数器在 S_IDLE 期间 acc_fire=0 + ofb_we=0，上电 X 不产生可观测副作用；`evt_start` 拍统一初始化。

## 仿真握手计数器（synthesis translate_off）

acc 接口的 {fire, stall, idle} 三计数器：
- `hs_acc_fire`：V=1 & R=1
- `hs_acc_stall`：V=1 & R=0（parf_accum 想送但 ofb_writer 不收，多半因 ring_full）
- `hs_acc_idle`：V=0 & R=1（ofb_writer 可以收但 parf_accum drain 还没到，正常 fill 期）

## 在 core_top 中的位置

实例 `u_ofb_writer`：
- 上游：`u_psum_reshape.out_*`（间接来自 `u_parf_accum.acc_out_*`）
- 下游：`u_ofb` SRAM 的写端口
- start/done 由 `u_sequencer` 同步驱动
- `rows_written / rows_drained / row_done_pulse` 与 `u_odma` 双向交互（streaming row credit）
