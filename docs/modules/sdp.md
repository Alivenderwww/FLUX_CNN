# sdp

Single Data Processor。把 16 路 int32 psum 量化成 16 路 int8 输出。Per-tensor symmetric int8 量化 + 可选 ReLU + clip。整个模块纯组合，零延迟。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL` | 通道数（cout） | 16 |
| `PSUM_WIDTH` | 输入 psum 位宽 | 32 |

## cfg 输入

| 信号 | 位宽 | 含义 |
| --- | ---: | --- |
| `shift_amt` | 6 | 算术右移位数 |
| `mult` | signed 32 | 乘法 scale |
| `zp_out` | signed 9 | 输出 zero-point |
| `clip_min / clip_max` | signed 9 | 截断上下界 |
| `round_en` | 1 | 是否在右移前加 `2^(shift-1)` 做四舍五入 |
| `relu_en` | 1 | 是否在 clip 前对负值置 0 |

## 数据接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `psum_in[NUM_COL*32]` | in | 16 路 int32 psum 拼接 |
| `valid_in` | in | 上游 valid |
| `ofm_data[NUM_COL*8]` | out | 16 路 int8 输出拼接 |
| `valid_out` | out | 等于 `valid_in`（直通） |

## 计算流程（每通道独立，纯组合）

按 c ∈ [0, NUM_COL) 对每路独立计算：

1. `psum_ch[c] = signed(psum_in[c*32 +: 32])`，把 32-bit 切片解为 signed int32
2. `prod_ch[c] = psum_ch[c] × mult`，结果 signed 64-bit（int32 × int32 → int64）
3. `round_ch[c] = prod_ch[c] + round_bias`，其中 `round_bias = round_en && shift!=0 ? (1 << (shift-1)) : 0`
4. `shifted_ch[c] = round_ch[c] >>> shift_amt`，算术右移（保留符号）
5. `q_zp_ch[c] = shifted_ch[c] + zp_out_ext`，加零点（zp_out 经 signed 扩展到 EXT_W=42 bit）
6. `act_ch[c] = (relu_en && q_zp_ch[c] < 0) ? 0 : q_zp_ch[c]`，ReLU 前向阻断
7. `clip_ch[c] = clip(act_ch[c], clip_min_ext, clip_max_ext)`，signed 比较截断
8. `ofm_data[c*8 +: 8] = clip_ch[c][7:0]`，截 8 bit 输出

中间用 EXT_W=42 bit 是为了让 q + zp_out 的 signed 加法不溢出。`zp_out / clip_min / clip_max` 通过 SV 的 signed 自动符号扩展赋给 EXT_W signed 变量，保证 clip 比较走 signed 通路。

## cfg 兼容典型场景

| 场景 | mult | shift | zp_out | clip | relu_en | round_en |
| --- | ---: | ---: | ---: | --- | ---: | ---: |
| 老 uint8 ReLU | 1 | N | 0 | [0, 255] | 1 | 0 |
| Per-tensor int8 ReLU | M (~`s_x·s_w/s_y · 2^shift`) | 24~31 | 0 | [0, 127] | 1 | 1 |
| Per-tensor int8 无 ReLU | M | 24~31 | 0 | [-128, 127] | 0 | 1 |

## 时序

`valid_out = valid_in`，`ofm_data` 跟随 `psum_in / cfg` 同拍组合输出。整级 0 周期延迟，下游 `ofb_writer` 当拍消费。

## 在 core_top 中的位置

`u_sdp` 例化在 `u_ofb_writer` 内部。上游接 `psum_reshape` 的 `out_vec`，下游 `ofm_data` 进 `ofb_writer` 的 OFB 写数据通路。所有 cfg 信号从 `cfg_regs` 直连。
