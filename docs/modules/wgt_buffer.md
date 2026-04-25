# wgt_buffer

权重和 bias 的供给。从 WB SRAM 读权重和 bias，按 PE 把权重灌进 mac_array 的 256 个 WRF；同时输出当前 cs 对应的 bias_vec 给 mac_array 做 acc_seed。

权重布局：WB 前缀 `cout_slices` 个 word 是 bias（每 word 低 NUM_COL × 32 bit 装 16 个 int32），其后按 `(cs, cins, ky, kx)` 顺序铺权重，每 (cs, cins, ky, kx) 一个 word（2048 bit = 16 cout × 16 cin × 8 bit）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL / NUM_PE` | PE 阵列尺寸 | 16 / 16 |
| `DATA_WIDTH` | 权重位宽 | 8 |
| `PSUM_WIDTH` | bias 位宽 | 32 |
| `WRF_DEPTH` | 每 PE 的 WRF 深度 | 32 |
| `MAX_COUT_SLICES` | BRF 深度（支持 cout=512 即 32 slice） | 32 |
| `SRAM_DEPTH / ADDR_W` | WB SRAM 深度 / 地址宽 | 8192 / 20 |

## 接口

### cfg
和 line_buffer 一致的 tile / strip / yout 参数，再加：
- `cfg_kk = K × K`（无 fold）或 `kxper × kyper`（fold 后）
- `cfg_total_wrf = kk × cin_slices`，cold load 阶段总写 WRF 次数（packed 模式）
- `cfg_rounds_per_cins`：当 kk × cin_slices > WRF_DEPTH 时把 round 切片
- `cfg_round_len_last`：最后一 round 的长度（其余 round 都是 32）
- `cfg_wb_base / cfg_wb_cout_step`：WB 起点和跨 cs 步长
- `cfg_fold_cout_groups / cfg_fold_col_shift`：Kx-fold 的 x_cnt 扩展参数

### WB 读端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `wb_re / wb_raddr` | out | 读 WB |
| `wb_rdata[2048]` | in | 一 word 16 cout × 16 cin × 8 bit |

### WRF 写端口（给 mac_array）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `wrf_we[256]` | out | 256 个 PE 各一位（同拍全 1 是 broadcast 写一个 (kx, ky) slot） |
| `wrf_waddr` | out | 共用写地址 |
| `wrf_wdata[2048]` | out | 等于 `wb_rdata`（直通） |

### COMPUTE 端口（给 mac_array）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `wgt_valid / wgt_ready` | out/in | 权重 valid-ready |
| `wrf_raddr` | out | 当前 (kx, ky) slot index |

### bias 端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `bias_vec[NUM_COL×32]` | out | 当前 cs 对应的 16 个 int32 bias，组合读 BRF |

## 状态机

```
S_IDLE → S_BIAS_LOAD → S_LOAD → S_COMPUTE → S_DONE
   ↑                                            │
   └────────────────  start ────────────────────┘
```

- **S_BIAS_LOAD**：扫 WB[0..cout_slices-1]，把每 word 低 16×32 bit 装成 16 个 int32 写入 BRF（`bias_rf`）。1 拍读延迟，bias_load_cnt 走完 cout_slices 个进入下一态。
- **S_LOAD**（cold load）：从 `cfg_wb_base` 开始读 round 0 的权重灌进 256 个 PE 的 WRF[0..c_cur_round_len-1]。一拍一个 word（256 PE 同拍写各自的 wdata 切片），读完 cur_load_len 个 word 进入 COMPUTE。
- **S_COMPUTE**：compute 和 load 并行（cins-ahead 流水）。compute 侧推 `pos_cnt → x_cnt → cins/tile/yout/cs`，load 侧推 `l_pos / l_round / l_cins / ...`。compute 一直跑直到最后 (yout, cs) 的最后一拍。
- **S_DONE**：等下一 case 的 start。

## 双指针：compute ptr 和 load ptr

compute 侧：`x_cnt`（iss_pos 同步）、`pos_cnt`（WRF 读地址 = (kx, ky) slot）、`round_cnt`、`cins/tile/yout/cs_cnt`。

load 侧：`l_pos`、`l_slots_done`、`l_round`、`l_cins`、`l_tile`、`l_yout`、`l_cs`。这套指针目标是 compute 的"下一 round"：cold load 完了 round 0 后立即开始装 round 1，compute round 0 跑完时 round 1 已就绪。

`load_caught_up`：1 表示 load 领先 compute 2 个 round（已经装完下一个、再下一个还没开始），等 compute 消费一个 round 之后回到 0。`l_exhausted`：所有 load round 都装完。

## 跨 round 的 hazard

WRF 是 1R/1W 单端口（`std_rf`）。compute 在 round=N 读 WRF[pos]，load 准备 round=N+1 的权重也要写 WRF。三种情况避开 hazard：

1. `l_pos >= c_cur_round_len`：load 写到 compute 本 round 不读的高位 slot，安全
2. `pos_cnt > l_pos`：compute 已经走过 `l_pos`，那 slot 可以被覆盖
3. `pos_cnt == l_pos && x_is_last && wgt_fire`：compute 本拍读完 slot `l_pos` 的最后一 fire，下一拍 compute 推到 `l_pos+1`，loader 在 t+2 写入时不冲突

`chunked_hazard_ok = (1) || (2) || (3)`。`chunked_load_active = S_COMPUTE && !l_exhausted && !load_caught_up && chunked_hazard_ok`。

## 起点选择 `l_start_pos_next`

跨 round 时 loader 的起点：如果下一 target round 比刚装完的更长，从 `l_cur_round_len` 起步（先写 compute 当前 round 不读的高 slot）；否则从 0 顺序写。这样保证 hazard 规则 (1) 始终能命中。

```
l_start_pos_next = (l_cur_round_len_next > l_cur_round_len) ? l_cur_round_len : 0
```

## K=1 的 round_wrap bubble

当 `c_cur_round_len == 1`（K=1 layer，WRF 只用 1 slot）：cins 切换时 compute 这拍读 WRF[0]、loader 这拍写 WRF[0]，但 WRF 是 NBA 写、组合读，下一拍 compute 仍然要读 WRF[0]，可能撞写后未完。

修复：`evt_c_round_wrap && c_cur_round_len==1` 时下一拍设 `wgt_bubble=1`，那拍 `wgt_valid=0` 让 mac_array 停一拍，给 WRF 写入完成留时间。K≥2 时 round_len≥2，写读地址错开，无需 bubble。

## 端口输出

```
wb_re    = sload_wb_re_gate | chunked_load_active | bias_re_gate
wb_raddr = bias_re_gate        ? bias_load_cnt
         : chunked_load_active ? l_wb_base + l_pos
         :                       cur_wb_rd_base + wb_rd_cnt

wrf_we    = {256{wb_re_d1 | l_wb_re_d1}}            // 同拍全 1，broadcast 写一个 slot
wrf_waddr = l_wb_re_d1 ? l_wrf_waddr_d1 : wrf_wr_cnt
wrf_wdata = wb_rdata                                // 直通

wgt_valid = (state == S_COMPUTE) && !wgt_bubble
wrf_raddr = pos_cnt
```

`wb_re` 当拍 → `wb_rdata` 在下一拍出现。`wb_re_d1 / l_wb_re_d1 / l_wrf_waddr_d1` 是 1 拍延迟副本，跟 `wb_rdata` 对齐做 WRF 写。

## Compute 侧推进事件

```
evt_c_fire       = S_COMPUTE && wgt_fire
evt_c_x_wrap     = evt_c_fire     && x_is_last         // x_cnt 走完一行 (pos 推进)
evt_c_pos_wrap   = evt_c_x_wrap   && pos_is_last       // pos 走完一 round (round 推进)
evt_c_round_wrap = evt_c_pos_wrap && round_is_last     // round 走完 (cins 推进)
evt_c_cins_wrap  = evt_c_round_wrap && cins_is_last    // cins 走完 (tile 推进)
evt_c_tile_wrap  = evt_c_cins_wrap  && tile_is_last    // tile 走完 (cs 推进, 在 yout 内)
evt_c_cs_wrap    = evt_c_tile_wrap  && cs_is_last      // cs 走完 (yout 推进)
evt_c_yout_wrap  = evt_c_cs_wrap    && yout_is_last
```

注意 cs 在 yout 内，外层是 yout：`for yout: for cs: for tile: for cins: for round: for pos: for x`。

## Load 侧推进事件

和 compute 完全对偶，嵌套顺序一致：

```
evt_ld            = chunked_load_active
evt_ld_round_end  = evt_ld && l_round_done
evt_ld_round_wrap = evt_ld_round_end && l_round_is_last
evt_ld_cins_wrap  = evt_ld_round_wrap && l_cins_is_last
evt_ld_tile_wrap  = evt_ld_cins_wrap && l_tile_is_last
evt_ld_cs_wrap    = evt_ld_tile_wrap && l_cs_is_last
evt_ld_yout_wrap  = evt_ld_cs_wrap   && l_yout_is_last
```

## Cold load 完成时 load ptr 的初始值

cold load 装完 round 0 后，load 侧立即推进到下一 round 的起点。如果 cfg 是单 round 单 cins 单 tile 单 cs 单 yout（最简单 case），load 侧已无活可干，`l_exhausted=1`。否则按嵌套优先级 round > cins > tile > cs > yout 选第一层非 1 的维度推进 1：

| 第一层多 | l_round | l_cins | l_tile | l_cs | l_yout |
| --- | ---: | ---: | ---: | ---: | ---: |
| rounds > 1 | 1 | 0 | 0 | 0 | 0 |
| 单 round, cins > 1 | 0 | 1 | 0 | 0 | 0 |
| 单 round, 单 cins, tiles > 1 | 0 | 0 | 1 | 0 | 0 |
| 单 ..., cs > 1 | 0 | 0 | 0 | 1 | 0 |
| 单 ..., 单 cs, h_out > 1 | 0 | 0 | 0 | 0 | 1 |

## Load 侧 WB base 切换

```
l_wb_base_cs:  当前 cs 的 base (yout 间回到 cfg_wb_base, cs 间 += wb_cout_step)
l_wb_base:     load 侧实际读地址的 base
  evt_ld_cs_wrap   → cfg_wb_base                       (新 yout, cs=0)
  evt_ld_tile_wrap → l_wb_base_cs + wb_cout_step       (新 cs)
  evt_ld_cins_wrap → l_wb_base_cs                      (新 tile, cins=0)
  evt_ld_round_end → l_wb_base + l_cur_round_len       (round 内推进)
```

## Bias 通路（F-1b）

```
S_BIAS_LOAD: 扫 WB[bias_load_cnt], 1 拍延后写 BRF[bias_wr_idx_d1]
COMPUTE:     bias_vec[c] = bias_rf[cs_cnt][c]   组合读
```

`bias_rf` 数据路径但有复位（清 0），保证 bias_en=0 场景下残留 X 不污染 mac_array 的 acc_seed 通路。

## Kx-fold 影响

`cur_valid_w` 在 Kx-fold 时扩到 `cur_valid_w_orig + (groups_x-1) × col_shift`，`x_cnt` 跑这个扩展长度（和 line_buffer 的 iss_pos 范围一致，保证两侧握手 fire 数对齐）。

## 在 core_top 中的位置

实例 `u_wgt_buffer`：
- 上游：`u_wb` SRAM
- 下游：`u_mac_array.{wrf_*, wgt_*, bias_vec}`
- start/done：和 `u_line_buffer / u_parf_accum / u_ofb_writer` 由 `u_sequencer` 同步拉起

## 仿真观察

WRF 写入计数没在 wgt_buffer 单独维护，但 TB 通过 `pe_wrf_write_arr[col][pe]` 经层次引用各 PE 内的 std_rf.total_write_ops 累加。
