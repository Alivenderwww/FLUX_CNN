# mac_array

16 × 16 = 256 个 PE 的阵列封装。把 `line_buffer` 来的激活、`wgt_buffer` 来的权重读地址、和 `parf_accum` 回传的 acc_seed 三路输入合并，输出 16 路 psum 给 `parf_accum`。承担 valid-ready 握手、bias / old_psum 累加种子叠加。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL` | 列数（cout 维） | 16 |
| `NUM_PE` | 每列 PE 数（cin 维） | 16 |
| `DATA_WIDTH` | 激活/权重位宽 | 8 |
| `PSUM_WIDTH` | psum 位宽 | 32 |
| `WRF_DEPTH` | 每 PE WRF 深度 | 32 |

## 接口

### 权重写端口（来自 wgt_buffer）

| 信号 | 位宽 | 含义 |
| --- | ---: | --- |
| `wrf_we` | 256 (NUM_COL × NUM_PE) | 每 PE 一位独立写使能；同拍只有一位为 1（wgt_buffer 逐 PE 灌） |
| `wrf_waddr` | 5 | 共用写地址（所有 PE 用同一 waddr） |
| `wrf_wdata` | 2048 (NUM_COL × NUM_PE × 8) | 16 列 × 16 PE × 8 bit 拼接，wgt_buffer 一次给 256 个 PE 的写候选数据 |

### 激活通路（来自 line_buffer）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `act_in_vec[NUM_PE×8]` | in | 16 个 cin 通道在某像素位置的拼接 |
| `act_valid` | in | act 有效 |
| `act_ready` | out | 阵列消费 act |

### 权重读端口（来自 wgt_buffer）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `wrf_raddr` | in | 共用读地址（所有 PE 用同一 raddr） |
| `wgt_valid` | in | weight 有效 |
| `wgt_ready` | out | 阵列消费 weight |

### psum 输出（给 parf_accum）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `psum_out_vec[NUM_COL×32]` | out | 16 列 psum 拼接 |
| `psum_out_valid` | out | psum 有效 |
| `psum_in_ready` | in | parf_accum 反压 |

### 累加种子（来自 wgt_buffer / parf_accum）

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `is_first_round_fill` | in | 1=首轮（用 bias 作种子），0=后续轮（用 old_psum 作种子） |
| `bias_vec[NUM_COL×32]` | in | 来自 wgt_buffer 的 BRF |
| `old_psum_vec[NUM_COL×32]` | in | 来自 parf_accum 当前 wr_addr 位置的旧 psum |

## 握手协议

握手是 elastic join + global stall 模式：

```
can_advance = (~pipe_s2_valid) | psum_in_ready
act_ready   = can_advance & wgt_valid
wgt_ready   = can_advance & act_valid
compute_en  = can_advance
```

含义：当下游可以收（`pipe_s2_valid=0` 或 `psum_in_ready=1`）时，整个 pipe 推进一步；act 和 wgt 必须同拍都 valid 才被消费。`compute_en` 传给所有 16 个 mac_col 作为流水推进闸门。

## Pipeline valid 追踪

阵列内有 2 级流水（`mac_pe.prod_out` 和 `mac_col.adder_tree_reg`），需要追踪每级 valid：

```
stage0_has_data = act_valid & wgt_valid          // join 后 stage 0 装入数据
pipe_s1_valid   ← (can_advance ? stage0_has_data : pipe_s1_valid)
pipe_s2_valid   ← (can_advance ? pipe_s1_valid   : pipe_s2_valid)
psum_out_valid  = pipe_s2_valid
```

`pipe_s1_valid / pipe_s2_valid` 是控制路径，同步复位到 0。

## 时序

T 拍 `act_valid=1, wgt_valid=1, psum_in_ready=1`：
- T：`can_advance=1`，act 和 wgt 同拍 fire（join 成功），mac_pe 内部把 `act × weight` 算好准备寄存
- T+1：`mac_pe.prod_out` 寄存好乘积；`mac_col.adder_tree_out` 组合算出 16 个乘积之和；`pipe_s1_valid=1`
- T+2：`mac_col.adder_tree_reg` 寄存好和，`psum_out` 出现；`pipe_s2_valid=1`，`psum_out_valid=1`

下游若 `psum_in_ready=0` 时停滞，整个 pipe 冻结：所有 valid 寄存器保持，`compute_en=0` 让 mac_pe / mac_col 的数据寄存器也保持。等下游恢复 `ready=1` 后整个 pipe 同时推进一拍。

上游 act 或 wgt 缺一拍时，对应 ready 不会拉高，但下游若仍然 ready，pipe 内已存在的数据可以继续流出（`pipe_s2 → psum_out → parf` 在 `act_valid=0` 时仍然 fire）。

## 列内结构

generate for 例化 `NUM_COL=16` 个 `mac_col`：
- 每列接收切片后的 `wrf_we` (16 bit) 和 `wrf_wdata` (128 bit)
- 共享 `wrf_raddr / act_in_vec / compute_en`
- 输出 `col_psum_out`（来自 mac_col 内部 16 PE 加和）

每列再做一次"种子加"：
```
acc_seed = is_first_round_fill ? bias_vec[c] : old_psum_vec[c]
psum_out_vec[c] = col_psum_out + acc_seed
```

逻辑上是 17-input 加法，物理落地由综合工具决定（一般是 16-tree + 后接 1 加）。

## 数据通路与控制路径

- 数据路径无复位：`mac_pe.prod_out`、`mac_col.adder_tree_reg`、PE 内 WRF
- 控制路径同步复位：`pipe_s1_valid`、`pipe_s2_valid`

## 仿真握手计数器（synthesis translate_off）

每个 V/R 接口（act、wgt、psum）三计数器：
- `hs_*_fire`：V=1 & R=1，握手成功
- `hs_*_stall`：V=1 & R=0，上游已 valid 但下游不收（下游慢）
- `hs_*_idle`：V=0 & R=1，下游可收但上游没有数据（上游慢）

另有 `mac_fire_cnt`：stage 0 join 成功的周期数（= 真正送入 pipe 的 MAC 拍数）。

注意 `act_ready = can_advance & wgt_valid`，所以 `hs_act_idle` 只统计"`act_valid=0` 但 `wgt_valid=1`"的拍；同拍 `act_valid=0 && wgt_valid=0` 的"双 idle"两个 idle 计数器都不算（属于 setup 启动期）。

## 在 core_top 中的位置

实例 `u_mac_array`：
- 上游：`u_line_buffer.act_*` + `u_wgt_buffer.{wgt_*, wrf_*}`
- 下游：`u_parf_accum.psum_in_*`
- bias_vec：`u_wgt_buffer.brf_out_vec`
- old_psum_vec：`u_parf_accum.old_psum_vec`
- is_first_round_fill：`u_wgt_buffer` 的 round 计数器与 cins 计数器联合译码出
