# parf_accum

PARF（Partial-sum Register File）累加器外壳。内部把 PARF 拆成 `NUM_COL=16` 个 `parf_col` 子模块（每列各自一块 SRAM），外壳负责 FILL / DRAIN 计数、握手、地址生成。所有列共享 wr_addr / we / rd_addr，只是 wdata 切片不同。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL` | 列数 | 16 |
| `PSUM_WIDTH` | psum 位宽 | 32 |
| `PARF_DEPTH` | 每列 SRAM 深度 | 32 |

## 接口

### cfg

| 信号 | 含义 |
| --- | --- |
| `cfg_tile_w / cfg_last_valid_w / cfg_num_tiles` | tile 划分参数 |
| `cfg_cin_slices` | cins 维划分数 |
| `cfg_kk` | kx × ky 总数（fold 后是 kxper × kyper） |

### 上游（来自 mac_array）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `psum_in_valid / psum_in_vec[NUM_COL×32] / psum_in_ready` | in/in/out | 16 路 psum 输入 |

### 下游（给 ofb_writer）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `acc_out_valid / acc_out_vec[NUM_COL×32] / acc_out_ready` | out/out/in | drain 阶段输出 |

### 反馈给 mac_array

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `is_first_round_fill_out` | out | 1 = 当前 fill 是 (cins=0, kk=0)，需要用 bias 作种子 |
| `old_psum_at_wr[NUM_COL×32]` | out | 每列 `parf_col[wr_addr]` 的旧值，用于累加种子 |

## 内部计数器

FILL 侧：
- `wr_addr` (5 bit)：列内写地址，0..cur_valid_w_fill-1
- `kk_cnt` (10 bit)：(kx, ky) 联合计数，0..cfg_kk-1
- `cins_cnt` (6 bit)：cin slice 计数，0..cfg_cin_slices-1
- `fill_tile_cnt` (8 bit)：tile 计数，0..cfg_num_tiles-1

DRAIN 侧：
- `rd_addr`：列内读地址
- `drain_tile_cnt`
- `drain_active`：1=正在 drain（输出有效）

## 派生量

```
cur_valid_w_fill   = (fill_tile_cnt == num_tiles-1) ? last_valid_w : tile_w
cur_valid_w_drain  = (drain_tile_cnt == num_tiles-1) ? last_valid_w : tile_w
is_first_round_fill = (cins_cnt == 0) && (kk_cnt == 0)
overlap            = drain_active && is_first_round_fill
```

## 握手协议

```
fill_fire   = psum_in_valid && psum_in_ready
drain_fire  = drain_active && acc_out_ready
acc_out_valid = drain_active

fill_would_tile_done = wr_is_last_col && kk_is_last && cins_is_last
drain_stall_fill     = fill_would_tile_done && drain_active
                       && !(drain_fire && rd_is_last)
psum_in_ready = (!overlap || acc_out_ready) && !drain_stall_fill
```

两条同步规则：

1. **overlap first_round**：当 drain 激活同时下一 tile 的 first_round（cins=0, kk=0）正在 fill，两者会同拍触碰同一 PARF 单元（drain 读旧值、fill 写新值）。`parf_col` 是组合读 + 同步写，同拍读旧值再写新值是安全的，但要保证 drain 不会"晚于" fill 推进——所以在 overlap 期 fill 必须等 acc_out_ready=1 时才能 fire（绑定到 drain 节奏）。

2. **fill 收尾不能领先 drain 收尾**：`cur_valid_w_fill` 极小时（特别是 last_valid_w=1）fill 的最后一拍可能在前一 tile drain 还没结束时到达。如果让 fill 这拍 fire 会导致下一拍 drain_tile_cnt 切换之前 fill 已经覆盖了下一 tile 的位置。`drain_stall_fill` 在 fill 即将 tile_done 但 drain 还没完成前一 tile 时阻塞 fill。

## 推进事件

```
ev_fill_wr_wrap = fill_fire && wr_is_last_col       // wr_addr 走完一轮 (kk 推进)
ev_fill_kk_wrap = ev_fill_wr_wrap && kk_is_last     // kk 走完一轮 (cins 推进)
fill_tile_done  = ev_fill_kk_wrap && cins_is_last   // 整 tile fill 完
drain_tile_done = drain_fire && rd_is_last          // 整 tile drain 完
```

## 寄存器更新

FILL 侧（控制路径，复位）：
- `wr_addr` 在 `ev_fill_wr_wrap` 归 0，否则 `fill_fire` +1
- `kk_cnt` 在 `ev_fill_kk_wrap` 归 0，否则 `ev_fill_wr_wrap` +1
- `cins_cnt` 在 `fill_tile_done` 归 0，否则 `ev_fill_kk_wrap` +1
- `fill_tile_cnt` 在 `fill_tile_done` 推进（last 归 0）

DRAIN 侧：
- `drain_active` 在 `fill_tile_done` 拉起，`drain_tile_done` 拉低
- `rd_addr` 在 `fill_tile_done` 或 `drain_tile_done` 归 0，`drain_fire` +1
- `drain_tile_cnt` 在 `fill_tile_done` 拷贝 `fill_tile_cnt`，`drain_tile_done` 推进

## per-col 实例化

```
generate for (gc = 0; gc < NUM_COL; gc++) begin
    parf_col u_col (
        .we      (fill_fire),
        .wr_addr (wr_addr),
        .wdata   (psum_in_vec[gc*32 +: 32]),
        .rd_addr (rd_addr),
        .rdata   (acc_out_vec[gc*32 +: 32]),
        .old_at_wr(old_psum_at_wr[gc*32 +: 32])
    );
end
```

## old_psum_at_wr 通路

每列 `parf_col` 用共享的 `wr_addr` 组合读出 `old_at_wr[c]`，拼成 `old_psum_at_wr` 总线送回 `mac_array`。`mac_array` 在 `is_first_round_fill=0` 时把这个旧 psum 加到当前列乘积之和上（作为种子），在 `=1` 时改用 `wgt_buffer.brf_out_vec`（bias）作种子。

## 仿真计数器

- `fill_fire_cnt`：累计 fill_fire 拍数
- `drain_fire_cnt`：累计 drain_fire 拍数

## 在 core_top 中的位置

实例 `u_parf_accum`：
- 上游：`u_mac_array.psum_*`
- 下游：`u_ofb_writer.acc_out_*`
- 反馈：`old_psum_at_wr` → `u_mac_array.old_psum_vec`，`is_first_round_fill_out` → `u_mac_array.is_first_round_fill`
