# psum_reshape

`parf_accum` 输出和 `sdp` 之间的纯组合归约级。Kx-fold 启用时把 16 列里每 `cfg_cout_orig` 列一组的 `cfg_cout_groups` 组按 cout 维度对齐累加；不 fold 时直通。零延迟，不占周期。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL` | PE 列数 | 16 |
| `PSUM_WIDTH` | psum 位宽 | 32 |

## 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `cfg_cout_orig` | in | 原始 Cout 数（1..16），决定每组多少列 |
| `cfg_cout_groups` | in | 列组数（1..16），cout_fake = cout_orig × cout_groups |
| `in_valid / in_vec` | in | 来自 `parf_accum` 的 16 路 psum（每列 32-bit 拼成 512-bit） |
| `out_valid / out_vec` | out | 给 `sdp` 的 16 路 psum |

## 映射关系

```
out[co] = Σ_{g=0..cout_groups-1} in[g × cout_orig + co]    co ∈ [0, cout_orig)
out[co] = 0                                                  co ∈ [cout_orig, NUM_COL)
```

无 fold（cout_groups=1）时退化为 `out[co] = in[co]` 直通。

## 时序

`out_valid = in_valid` 直接连。`out_vec` 是组合逻辑，跟随 `in_vec` 同拍变化。整个 reshape 不占周期，下游 `sdp` 当拍接收。

加法树深度由 `cout_groups` 决定，最大 `log2(max_groups) = log2(16) = 4` 级。

## 内部结构

`always_comb` 双重循环：外层 `co` 遍历 0..NUM_COL-1（输出 cout 索引），内层 `g` 遍历 0..NUM_COL-1（潜在的列组索引）。当 `g < cfg_cout_groups` 且 `g × cfg_cout_orig + co` 不超过 NUM_COL 时累加 `in_vec[src_col]`。

`co ≥ cfg_cout_orig` 的输出位置直接置 0（这部分对 `sdp` / `ofb_writer` 是无效字段，不会被消费）。

## 在 core_top 中的位置

实例 `u_psum_reshape` 接在 `u_parf_accum.acc_out_vec` 和 `u_ofb_writer.acc_in_vec` 之间。`u_sdp` 例化在 `u_ofb_writer` 内部。`cfg_cout_orig` 和 `cfg_cout_groups` 由 `cfg_regs` 提供。
