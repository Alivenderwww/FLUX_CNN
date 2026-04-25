# 硬件循环嵌套

切片机制在硬件里的具体形态是 5 个核心模块各自的计数器嵌套。三个模块（line_buffer、wgt_buffer、ofb_writer）各跑一套 6-7 层循环，靠 valid-ready 握手对齐节拍；parf_accum 维护 fill / drain 两套计数器跟着上游走。所有模块都看同一份 cfg 寄存器（`CIN_SLICES / COUT_SLICES / NUM_TILES / H_OUT` 等）。

## 共同的循环结构

完整嵌套（从最外到最内）：

```
yout (整图行)
  cs (cout 切片)
    tile (W 方向 tile)
      cins (cin 切片)             ← 在 PARF 里时间累加
        ky (kernel y)
          kx (kernel x)
            x_in / iss_pos        ← 每拍一像素位置
```

注意要点：

1. **cs 在 yout 内**，不是最外层。这样 OFB 写出顺序天然是 NHWC（一行 yout 内连续写 cout_slices 个 16-cout 段），ODMA 直接按 NHWC gather 搬到 DDR。
2. **cins 在 tile 内**。一个 tile 处理时，先跑完所有 cin 切片对该 tile 的贡献，每片把 `K² × cur_valid_w` 个 MAC 累加进 PARF；cins=last 时 tile 的 PARF 就完整了，触发 drain。
3. **ky / kx / iss_pos 是最内层 3 重**，每拍走 1 步。Kx-fold 启用时 iss_pos 还会扩展 `(groups_x-1) × col_shift` 拍。

每层循环的 wrap 事件都从最内层 fire（`act_fire` / `wgt_fire` / `acc_fire`）一级一级往上传：内层 wrap 触发外层 +1。

## line_buffer 的循环

完整 7 层（含 iss_pos）：

```
for cs in [0, cout_slices):           # cs_cnt
  for yout in [0, n_yout_strip):      # yout_cnt
    ↑↓ 实际嵌套是: yout 外, cs 在 yout 内
  for yout:
    for cs:
      for tile in [0, num_tiles):     # tile_cnt
        for cins in [0, cin_slices):  # cins_cnt
          for ky in [0, KY):          # ky_cnt
            for kx in [0, K):         # kx_cnt
              for iss_pos in [0, cur_valid_w_ext):
                issue (IFB 读 + ARF 写) 或 sliding-window CONSUME
```

事件链（从内到外）：

```
evt_iss_pos_wrap   = advance && (iss_pos == cur_valid_w_ext-1)
evt_iss_kx_wrap    = evt_iss_pos_wrap   && (kx_cnt == K-1)
evt_iss_ky_wrap    = evt_iss_kx_wrap    && (ky_cnt == KY-1)
evt_iss_cins_wrap  = evt_iss_ky_wrap    && (cins_cnt == cin_slices-1)
evt_iss_tile_wrap  = evt_iss_cins_wrap  && (tile_cnt == num_tiles-1)
evt_iss_cs_wrap    = evt_iss_tile_wrap  && (cs_cnt == cout_slices-1)
evt_iss_yout_wrap  = evt_iss_cs_wrap    && (yout_cnt == n_yout-1)
```

**5 层 IFB 地址 base 跟着外层进位累加**：

| base | 跨过的事件 | 累加量 |
| --- | --- | --- |
| `ptr_yout_base` | yout 推进 | `+IFB_ROW_STEP` |
| `ptr_tile_base` | tile 推进 | `+TILE_IN_STEP` |
| `ptr_cins_base` | cins 推进 | `+1`（NHWC 内部 cin 跨片步长 = 1 word） |
| `ptr_ky_base` | ky 推进 | `+IFB_KY_STEP` |
| `ptr_kx_base` | kx 推进 | `+cin_slices` |

cs 切换（`evt_iss_cs_wrap` 但 yout 不换）时所有 IFB base 回到当前 yout 的起点——同一行的不同 cout 切片读的都是**同一份 IFB 数据**，只是权重不同。`cs_cnt` 不参与 IFB 地址。

## wgt_buffer 的循环

compute 侧和 line_buffer 完全对偶（同样 7 层），事件 `evt_c_*`：

```
for yout:
  for cs:
    for tile:
      for cins:
        for round:           # round_cnt: K² > 32 时切 round, K² ≤ 32 时只有 1 round
          for pos:            # pos_cnt: 即 (kx, ky) 联合 index, 在 round 内 0..round_len-1
            for x_cnt:        # x_cnt: 等价 line_buffer 的 iss_pos
              wgt_fire (送 wrf_raddr 给 mac_array)
```

WB 地址按外层进位推进：

| 事件 | 累加 |
| --- | --- |
| 跨 round 内（同 cins） | `l_wb_base += round_len`（写下一 round 的权重到 WRF 高位） |
| 跨 cins | 回到 `l_wb_base_cs`（同 cs 的起点） |
| 跨 tile（同 cs） | 回到 `l_wb_base_cs`（每 tile 重新读同一组权重） |
| 跨 cs | `l_wb_base_cs += WB_COUT_STEP`（跨 cs 步长 = `K² × cin_slices`） |
| 跨 yout | 回到 `cfg_wb_base`（cs=0 重新开始） |

注意 wgt_buffer 跨 tile 时 base **不增长**——每 tile 都重新加载同一 cs 的权重。这是因为权重不依赖 x 位置，所有 tile 共享一份；硬件只跨一次 (yout, cs) 边界，tile 内只在 cins / round / pos / x_cnt 维度变化。

WRF 里同一 cins 的所有 (ky, kx) 权重一次性灌完（cold load 阶段或 chunked load 的 round），compute 阶段 `wrf_raddr = pos_cnt` 直接选地址。

## parf_accum 的循环

只有 fill / drain 两套计数器：

**fill 侧**（每 mac fire 推 1 拍）：

```
for fill_tile in [0, num_tiles):     # fill_tile_cnt
  for cins in [0, cin_slices):       # cins_cnt
    for kk in [0, KK):                # kk_cnt = K × KY = (kx, ky) 联合 index
      for wr_addr in [0, cur_valid_w_fill_ext):
        fill PARF (按每列 wr_addr 偏移写)
  drain PARF                          # cins, kk 都走完一轮就该 drain
```

事件：

```
ev_fill_wr_wrap = fill_fire && wr_is_last_col
ev_fill_kk_wrap = ev_fill_wr_wrap && kk_is_last
fill_tile_done  = ev_fill_kk_wrap && cins_is_last       ← 整 tile fill 完
```

`fill_tile_done` 拉起 `drain_active`，drain 侧 `rd_addr` 从 0 走到 `cur_valid_w_drain-1`。drain 和下一 tile 的 fill 可以重叠（first_round 同步规则保护 PARF 同址读写不冲突）。

注意 parf_accum **不知道 yout 和 cs**——它只跑 (tile, cins, kk) 三层。yout / cs 推进对 parf_accum 透明：跨 yout 和跨 cs 时 fill 累加器自然回到 tile=0、cins=0、kk=0、wr_addr=0，因为 line_buffer 和 wgt_buffer 也都同步在 yout / cs 边界上 wrap，三方步调一致。

## ofb_writer 的循环

写出比 line_buffer 简单（只有 4 层）：

```
for yout in [0, n_yout):              # yout_cnt
  for cs in [0, cout_slices):          # cs_cnt
    for tile in [0, num_tiles):        # tile_cnt
      for x in [0, cur_valid_w):        # x_cnt
        acc_fire → SDP → ofb_we
```

每 acc_fire 写 1 个 word（16 cout × 8 bit）到 OFB SRAM，`ofb_ptr` 每拍 +1，到 `cfg_ofb_ring_words` 处 wrap。整 yout 写完 cs_cnt 推进；同一 (yout, cs) 段的 OFB 写出是连续 W_OUT 个 word。

`row_done_pulse = evt_fire_cs_wrap`：一个 yout 全部 cs 段都写完时发一拍，给 ODMA 做行级 credit。

## cs 切换的同步

cs 推进事件由 line_buffer 触发（`evt_iss_tile_wrap && cs_is_last == 0`），同拍 wgt_buffer 也由 `evt_c_tile_wrap` 触发推进 `cs_cnt`。两边步调完全一致——都跑完 num_tiles 个 tile 就一起切到下一 cs。

cs 切换时：
- line_buffer 的 IFB base 回到当前 yout 起点（重读同一 yout 数据）
- wgt_buffer 的 WB base 跳到下一 cs 段（`+WB_COUT_STEP`）
- parf_accum 的 fill_tile_cnt 自然回 0（tile_wrap 在 cs_wrap 之前先发生）
- ofb_writer 的 cs_cnt +1，下一段 16 cout 输出连续写在 OFB ring 里同一行内

## cins 切换的同步

cins 推进发生在每个 tile 内部（`evt_iss_ky_wrap && cins_is_last == 0`）。三方同步：
- line_buffer 的 `ptr_cins_base` +1（NHWC 跨 cin 切片步长 = 1 word）
- wgt_buffer 的 cins_cnt +1，进入下一组权重的 round
- parf_accum 的 `cins_cnt` +1，新一轮 (ky, kx) 累加叠加到 PARF 同址（这是 cin 切片的核心：同一 PARF cell 在 cins 维度做时间累加）

cins=last_slice && kk=last 时 fill_tile_done 拉起，PARF 该 tile 的累加完成。

## 切片对 MAC 周期数的影响

总 MAC fire 拍数（理想 / 不算 fold tail / 不算 pipeline overhead）：

```
H_out × cout_slices × num_tiles × cin_slices × K × KY × cur_valid_w
```

可以拆解成：

```
(H_out × W_out)         ← 输出像素数
× K × KY                 ← 每 cell 的 kernel 拍数
× cin_slices             ← cin 时间维累加
× cout_slices            ← cs 时间维串行
```

PE 阵列每拍并行 16×16 = 256 MAC，所以单层总周期数：

```
total_mac = H_out × W_out × K² × Cin × Cout         (理想 MAC 数)
total_cycle = total_mac / (PE_PER_CYCLE × util)
            = total_mac / 256 / util
```

util 由 fold / Kx tail / pipeline 等因素决定（详见 `pe-fold.md`）。切片本身不浪费——cin 切片是把 Cin 维拆成多片串行累加，等效 PE 利用率不变；cout 切片是 cs 维度时间展开，每片仍能填满 16 列。util 损失主要来自 PE 内部的"假 fire"（fold pad / 切片末片不足 16）。
