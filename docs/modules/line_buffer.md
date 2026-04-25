# line_buffer

输入特征图（IFB）到 mac_array 的供给。从 IFB SRAM 按 `(yout, cs, tile, cins, ky, kx, iss_pos)` 顺序读 16 通道宽度的像素，经过一个 `act_buf` ring（或 reuse 模式下的线性 FILL 区）转交给 mac_array。同时支持 padding（越界位置喂 0 不读 IFB）和 streaming 行级反压（IDMA 没写到的行不读）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_PE` | 每像素的 cin 通道数 | 16 |
| `DATA_WIDTH` | 每通道位宽 | 8 |
| `ARF_DEPTH` | act_buf 深度 | 32 |
| `SRAM_DEPTH / ADDR_W` | IFB SRAM 深度 / 地址宽 | 8192 / 20 |

## 接口

### cfg
- `cfg_h_out / cfg_w_in / cfg_h_in`：输出高、输入宽、输入高（虚拟，pad 前的有效行数）
- `cfg_k = Kx`（fold 后等于 kxper）、`cfg_ky`（fold 后等于 kyper）
- `cfg_stride`（1 或 2）
- `cfg_pad_top / cfg_pad_left`
- `cfg_tile_w / cfg_num_tiles / cfg_last_valid_w`
- `cfg_cin_slices / cfg_cout_slices`
- `cfg_ifb_base / cfg_ifb_ring_words / cfg_ifb_row_step / cfg_tile_in_step / cfg_iss_step / cfg_ifb_ky_step / cfg_tile_pix_step`：IFB 的物理地址参数
- `cfg_arf_reuse_en`：是否启用滑窗复用模式（仅 stride=1 && K>1 时为 1）
- `cfg_fold_cout_groups / cfg_fold_col_shift`：Kx-fold 的 iss_pos 扩展参数

### IFB 读端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `ifb_re / ifb_raddr` | out | 读 IFB SRAM |
| `ifb_rdata[NUM_PE×8]` | in | 一 word 16 cin × 8 bit |

### 给 mac_array 的 act 端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `act_valid / act_ready` | out / in | valid-ready |
| `act_vec[NUM_PE×8]` | out | 16 cin 通道在某像素位置的拼接 |

### Streaming 行同步（与 IDMA）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `rows_consumed` | out | LB 已用完释放的行数（IDMA 看这个判断 ring 是否可覆盖） |
| `rows_available` | in | IDMA 已写完的行数（LB 看这个等够 K 行才发 issue） |

## 状态机

```
S_IDLE → S_RUN → S_DONE → (start 重入)
```

- S_IDLE：等 start
- S_RUN：所有 issue 推进发生在这里
- S_DONE：`run_drained=1` 后进入，等下一 strip / case 的 start

`run_drained` 的判定：
- reuse_en=0：`issues_all_done && fifo_count==0 && !issue_advance_d1`（fifo 空 + 最后一拍 arrival 已落地）
- reuse_en=1：`issues_all_done && !issue_advance_d1`（CONSUME 全部完成，最后一拍 FILL 已落地）

`done = (state == S_DONE) && !start`，多 case 重入时 start 同拍 done 立即掉 0。

## 外层 6 级计数器（issue 侧主控）

按从内到外：
```
iss_pos → kx_cnt → ky_cnt → cins_cnt → tile_cnt → cs_cnt → yout_cnt
```

注意 cs 在 yout 内（`for yout: for cs: for tile: ...`）。每个计数器都没有显式复位，靠 `evt_start` 拍的赋值初始化。

各 wrap 事件（comb）：
```
evt_iss_pos_wrap  = evt_advance_any && iss_pos_is_last
evt_iss_kx_wrap   = evt_iss_pos_wrap && kx_is_last
evt_iss_ky_wrap   = evt_iss_kx_wrap && ky_is_last
evt_iss_cins_wrap = evt_iss_ky_wrap && cins_is_last
evt_iss_tile_wrap = evt_iss_cins_wrap && tile_is_last
evt_iss_cs_wrap   = evt_iss_tile_wrap && cs_is_last
evt_iss_yout_wrap = evt_iss_cs_wrap && yout_is_last
```

`evt_advance_any`：reuse_en=0 时由 `issue_ok_std` 驱动；reuse_en=1 时由 `act_fire` 驱动。

## 派生量

```
cur_valid_w_orig = (tile == last) ? last_valid_w : tile_w
cur_valid_w      = cur_valid_w_orig + (groups_x - 1) × col_shift     // Kx-fold 扩展
cur_fill_len     = cur_valid_w + cfg_k - 1                            // reuse_en=1 FILL 长度
                                                                       // 编译器保证 cur_fill_len ≤ ARF_DEPTH
```

## Padding 判定

```
y_row_base 累加器：strip 起点 = -pad_top；evt_iss_cs_wrap (新 yout) +cfg_stride
x_tile_base 累加器：strip 起点 = -pad_left；新 yout / 新 cs 回到 -pad_left；
                    evt_iss_cins_wrap (新 tile) += cfg_tile_pix_step
iss_pos_s = iss_pos × stride        (stride=1: shift 0; stride=2: shift 1)

current_src_y = y_row_base + ky_cnt
current_src_x = x_tile_base + iss_pos_s + kx_cnt

is_pad = (src_y < 0) | (src_y >= cfg_h_in) | (src_x < 0) | (src_x >= cfg_w_in)
```

`cfg_ifb_base` 由 Sequencer 预扣 pad offset（即 `cfg_ifb_base = 物理 base - pad_top × W_IN - pad_left`），让 ptr_*_base 累加用虚拟坐标，物理 SRAM 地址在 (ky=pad_top, kx=pad_left) 那拍自然指向 IFB[0]。is_pad 判定独立于物理地址，只看虚拟坐标越界。

## 两种 issue 模式

### reuse_en=0（无滑窗复用）

```
issue_ok_std = S_RUN && !issues_all_done
            && (fifo_count + issue_advance_d1 < ARF_DEPTH)
            && streaming_rows_ready
ifb_raddr_raw = ptr_kx_base + iss_offset
iss_offset 累加器: evt_iss_pos_wrap 归 0; issue_ok_std 时 += cfg_iss_step
```

每 issue_ok 当拍发一次 IFB 读（除非 is_pad，pad 拍只推进 issue_advance_d1 不发 ifb_re）。`act_buf` 当 ring buffer 用：`wr_idx` 写、`rd_idx` 读、`fifo_count` 跟踪占用。`act_valid = (fifo_count > 0)`。

### reuse_en=1（stride=1 && K>1 时）

```
fill_issue = S_RUN && !issues_all_done
          && !fill_pos_is_terminal && streaming_rows_ready
ifb_raddr_raw = ptr_ky_base + fill_offset
fill_pos / fill_offset / wr_idx_fill: fill_reset (evt_start || evt_iss_kx_wrap) 归 0
                                      每 fill_issue +1, 上限 cur_fill_len
```

每个 (cins, tile, ky) 进入时 `fill_reset` 归零。FILL 阶段把整行 cur_fill_len 个像素线性填到 ARF[0..cur_fill_len-1]。CONSUME 阶段（kx × iss_pos）组合读 `ARF[kx + iss_pos]`，不发 ifb_re。FILL 和 CONSUME 完全并行（FILL 是生产者推 fill_pos / wr_idx_fill，CONSUME 是消费者推 kx_cnt / iss_pos）。

CONSUME 端：
```
rd_idx_cons = kx_cnt + iss_pos
act_valid   = (rd_idx_cons < wr_idx_fill)    // 读位置已经被 arrival 填完
act_vec     = act_buf[rd_idx_cons]
```

ky 切换时：`evt_iss_kx_wrap` 拍 CONSUME 完成最后一读，下一拍 `fill_reset` 拉高让 FILL 归 0 重新开始下一 ky 行。

## Ring wrap

`act_buf` 之外，IFB SRAM 地址在大图 streaming 模式下也是 ring 的（一次只装 strip_rows 行）。`ptr_*_base` 加 step 时调用 `wrap_addr()` 检查是否 ≥ `cfg_ifb_ring_words`，超出就减一次。整图装得下 SRAM 时 `cfg_ifb_ring_words = H_IN × W_IN × cin_slices` 覆盖整图，wrap 永不触发。

## 5 层 ptr_*_base 推进

按外层进位级联，每层带宽 ADDR_W=20 bit：
- `ptr_yout_base`：strip 起点；`evt_iss_cs_wrap && !yout_is_last` 时 += `cfg_ifb_row_step`
- `ptr_tile_base`：cs 切换回到 `ptr_yout_base`；新 tile（cins_wrap）时 += `cfg_tile_in_step`
- `ptr_cins_base`：tile 切换回到 ptr_yout_base / ptr_tile_base；新 cin slice（ky_wrap）时 +1（NHWC 跨 cin slice 步长 = 1 word）
- `ptr_ky_base`：跨 cins / cs / yout 时回到对应外层 base；新 ky（kx_wrap）时 += `cfg_ifb_ky_step`
- `ptr_kx_base`：所有外层级联 + 新 kx（pos_wrap）时 += `cfg_cin_slices`

每个 always_ff 用 if-elseif 链确保只匹配一个事件，优先级是 yout_wrap > cs_wrap > tile_wrap > cins_wrap > ky_wrap > kx_wrap > pos_wrap > 自保持。

## issue_advance_d1 / is_pad_d1 / arrival

`ifb_re` 和 `ifb_rdata` 之间有 1 拍 SRAM 读延迟。`issue_advance_d1` 是 `issue_any` 的 1 拍延迟副本，用作 `arrival`（`act_buf` 写入信号）。`is_pad_d1` 锁存当拍 `is_pad_now`，下拍 arrival 时选择写入 `ifb_rdata` 还是写 0。

## Streaming 行级反压

```
rows_needed = clamp(y_row_base + cfg_ky, 0, cfg_h_in)
streaming_rows_ready = (rows_available >= rows_needed)
```

LB 当前 yout 需要的最大输入行号 < `rows_available`（IDMA 已写满的行数）才能发 issue。`rows_needed` 上限钳到 `cfg_h_in`，让 pad_bot 区不实际等 IDMA 提供（pad_bot 行是虚拟的，永远不会被 IDMA 写）。

`rows_consumed_raw`：每完成一个 yout（`evt_iss_cs_wrap && !yout_is_last`）+= cfg_stride。
`rows_consumed = max(rows_consumed_raw - pad_top, 0)`：pad_top 行 LB 实际重用 row 0（IFB 没存这些 pad 行），所以释放计数要扣掉 pad_top 才不会让 IDMA 过早覆盖尚在使用的真实行。

## 数据通路与控制路径

- 数据路径无复位：5 层 ptr_*_base、iss_offset、fill_offset、fill_pos、wr_idx、rd_idx、act_buf、is_pad_d1、x_tile_base、y_row_base
- 控制路径同步复位：state、issues_all_done、fifo_count、issue_advance_d1、wr_idx_fill、rows_consumed_raw

无复位寄存器都靠 `evt_start` 拍的初始赋值进入合法状态。

## 仿真计数器（synthesis translate_off）

- `arf_write_cnt`：act_buf 写次数（=每拍 arrival）
- `arf_read_cnt`：act_buf 读次数（=每拍 act_fire）
- `ifb_read_cnt`：IFB SRAM 读次数
- `pad_skip_cnt`：被 pad 吞掉未发 IFB 读的 issue 拍数

`arf_read_cnt / arf_write_cnt` 比值 >1 说明 reuse mode 启用，sliding window 复用 ARF 的次数。

## 在 core_top 中的位置

实例 `u_line_buffer`：
- 上游：`u_ifb` SRAM
- 下游：`u_mac_array.act_*`
- start/done 由 `u_sequencer` 同步驱动
- `rows_consumed / rows_available` 与 `u_idma` 双向交互（streaming row credit）
