# PE 利用率优化：Ky-fold / Kx-fold / Space-to-Depth

16 × 16 PE 阵列只有当 `Cin ≥ 16 && Cout ≥ 16` 时才能填满。Cin 小或 Cout 小时空间利用率分别只有 `Cin/16` 或 `Cout/16`。三种维度折叠技术分别处理这两种情况：

- **Ky-fold**：Cin 小时把卷积核 Ky 维折到 cin 维，编译器侧输入 y 方向偏移复制
- **Kx-fold**：Cout 小时把卷积核 Kx 维折到 cout 维，硬件 systolic psum shift
- **Space-to-Depth (S2D)**：stride ≥ 2 时把 (kx%stride, ky%stride) 相位折到 cin 维，编译器侧数据重排

## 1. Ky-fold（编译器侧）

### 数学

原卷积：

```
out[y_out, x_out, co] = Σ_{ky, kx, c} W[ky, kx, co, c] · I_padded[y_out·s + ky, x_out·s + kx, c]
```

按 `ky = g·kyper + ky_local` 分组，定义虚拟 cin 通道 `g·Cin + c`：

```
I'[y_virt, x, g·Cin + c] = I_padded[y_virt + g·kyper, x, c]
W'[ky_local, kx, co, g·Cin+c] = W[g·kyper + ky_local, kx, co, c]
```

虚拟卷积用 `K_new=kyper` 的 ky 维 + `cin_fake = groups_y · Cin` 的 cin 维即等价原计算。

### 参数选择

```
groups_y = HW_PE / Cin                    # PE 行数 / Cin
kyper    = ceil(K / groups_y)
pad_ky   = groups_y · kyper - K           # 末组零 pad 数
cin_fake = groups_y · Cin
```

### 实现位置

- `toolchain/hw_files.py::compute_fold_params(K, Cin, HW_PE)`：返回 `(groups, kyper, cin_fake, pad_ky)`
- `hw_files.py::fold_input()`：生成虚拟 ifm（每个 y_virt 位置存 groups_y × Cin 个通道，来自 groups_y 个 y-shifted 物理行）
- `hw_files.py::fold_weights()`：把 W 重排到 W'，pad_ky 位置补零
- HW 完全无感，按普通 conv 跑

### 代价

- IFB 占用 × `groups_y`（输入 y-方向偏移复制）
- pad_ky > 0 时一部分 PE 行在末组上空算（贡献为 0）

## 2. Kx-fold（硬件支持）

Kx 不能像 Ky 那样靠输入复制解决——MAC 阵列每拍所有列共享同一像素广播，列之间无法用不同输入。

### Systolic PSUM Shift

每列存一个 (cout, kx) 对的权重；不同列组算同一 cout 的不同 kx 贡献，写到 PARF 同一位置时按列组偏移：

```
col c (group g, co) 持有 W[ky, kx = g·kxper + kx_v, co, cin_row]
广播 I[y+ky, x_in, cin] (kx_v 外层, x_in 内层):
  col c 的目标 x_out = (x_in − g·kxper − kx_v) / stride
                     = iss_pos − g · col_shift
  col_shift = kxper / stride
```

### 约束

`kxper % stride == 0`（让所有 group 的 parity 对齐，每拍 16 列齐发）。

### iss_pos 扩展

LB 的 `iss_pos` 从 `cur_valid_w` 延长到 `cur_valid_w + (groups_x-1) × col_shift`，给最后一列组留 systolic 尾部。`line_buffer / wgt_buffer / parf_accum` 三方扩展量必须一致。

### PARF 每列 wr_addr 偏移

`parf_accum` 把 PARF 拆成 16 个独立 `parf_col`：

```
wr_addr_col[c] = wr_addr − col_group[c] × cfg_fold_col_shift
we_col[c]      = fill_fire && (0 ≤ wr_addr_col[c] < cur_valid_w_fill)
```

越界 mask 让首尾几拍只有部分列实际写入。

### psum_reshape 归约

drain 时用 `psum_reshape` 把同一 cout 下 `groups_x` 个列贡献求和，给 SDP `cout_orig` 路：

```
out[co] = Σ_{g=0..groups_x-1} in[g · cout_orig + co]    co ∈ [0, cout_orig)
out[co] = 0                                              co ∈ [cout_orig, NUM_COL)
```

纯组合，0 周期。

### K=1 round_wrap bubble

K=1 时 WRF 只用 1 slot。cins 切换瞬间 compute 读 WRF[0] 与 loader 写 WRF[0] 撞同址。修复：`c_cur_round_len == 1` 时 round_wrap 下一拍插 1 拍 bubble（`wgt_valid=0`）等 WRF 写完成。K≥2 时 round_len≥2 自然错开。

### 参数选择

`compute_fold_params_kx(K, Cout, stride, HW_COL)`：

```
groups_max = HW_COL // Cout
for g in range(groups_max, 1, -1):
    kxper = ceil(K/g) 向上取整到 stride 倍数
    if g·kxper ≥ K AND kxper % stride == 0:
        cand = (g, kxper, g·Cout, g·kxper-K, kxper/stride)
取 pad_kx 最小 (tie-break: groups 更大)
```

## 3. Space-to-Depth（编译器侧）

stride ≥ 2 的卷积，把 (kx%stride, ky%stride) 4 个相位折到 cin 维。等价为 stride=1、`K_new = ceil(K/stride)`、`Cin_new = stride² · Cin` 的卷积。

### 数学

`ky = stride·ky' + a, kx = stride·kx' + b, p = a·stride + b`：

```
I'[Y, X, p·Cin + c] = I_padded[Y·stride + a, X·stride + b, c]
W'[ky', kx', co, p·Cin+c] = W[ky'·stride + a, kx'·stride + b, co, c]
                            (越界补 0, K 不被 stride 整除时)
```

### 参数

```
K_new   = ceil(K / stride)
Cin_new = stride² · Cin
pad_waste = (K_new² · stride² - K²) / (K_new² · stride²)
```

K 被 stride 整除时 `pad_waste=0`（如 K=8 stride=2，4 个相位都是 4×4）；不整除时不同相位 sub-kernel 形状不齐，统一 pad 到 K_new × K_new 会浪费一部分 MAC。

### 实现位置

- `compute_s2d_params(K, Cin, stride)`：返回 `(K_new, Cin_new, applicable, pad_waste)`
- `s2d_input()`：pre-pad 到 stride 整除尺寸 + 重排 4 相位通道
- `s2d_weights()`：W 重排 + 内核 pad 到统一 K_new

### 代价 / 收益

- DDR / IFB 内存：等量重排（不复制）。相比 Ky-fold 的 `groups_y` 倍 inflation，多核场景 DDR 带宽节省明显
- HW 完全无感（按 stride=1 conv 跑）
- 启用 stride=1 ARF reuse_en=1 滑动窗口复用，IFB 读次数大幅下降
- 反压：reuse_en=1 模式 ky 边界 K 拍 FILL 启动惰性，对 cycles 有 ~3% 量级影响

## 4. ARF 容量约束

Kx-fold 下 LB 的 FILL 长度：

```
cur_fill_len = cur_valid_w_ext + K - 1 = (cur_valid_w + (groups_x-1)·col_shift) + K - 1
```

必须 ≤ `ARF_DEPTH = 32`。编译器选 `tile_w` 时要预留 `fold_tail = (groups_x-1)·col_shift`：

```
max_tile_w = ARF_DEPTH - K + 1 - fold_tail
```

## 5. 受益层判定

`run_regression.py` 自动决策：

| 优化 | 触发条件 |
| --- | --- |
| Ky-fold | `K > 1 AND Cin < 16` |
| Kx-fold | `K > 1 AND Cout < 16 AND Cout ∈ {1,2,4,8}` |
| S2D | `stride ≥ 2 AND K ≥ stride` |

S2D 启用后 Cin 变为 stride²·Cin，重新判定 Ky-fold 触发条件（多数情况下 S2D 后 Cin' ≥ 16 不再需要 Ky-fold）。

K=1 / FC 层不受益（无可折维度）。Cin/Cout ≥ 16 的主层已近满 util，不需要 fold。

## 6. 相关 RTL / 工具链文件

| 文件 | 内容 |
| --- | --- |
| `RTL/parf_col.sv` | Kx-fold 单列 PSUM 存储 |
| `RTL/parf_accum.sv` | per-col wr_addr 偏移 + we mask |
| `RTL/psum_reshape.sv` | Kx-fold drain 时 cout 归约 |
| `RTL/cfg_regs.sv` | FOLD_COUT_ORIG / FOLD_COUT_GROUPS / FOLD_COL_SHIFT |
| `RTL/line_buffer.sv` | iss_pos 扩展到 `cur_valid_w + fold_tail` |
| `RTL/wgt_buffer.sv` | x_cnt 同步扩展 + K=1 bubble |
| `toolchain/hw_files.py` | `compute_fold_params*` / `fold_input/weights` / `compute_s2d_params` / `s2d_input/weights` |
| `toolchain/gen_isa_test.py` | `--ky-fold` / `--kx-fold` / `--s2d` |
