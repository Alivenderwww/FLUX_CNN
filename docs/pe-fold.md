# PE 利用率优化：Ky / Kx fold

K 阶段引入的两种**维度折叠**技术，通过把卷积核的 Ky / Kx 维折叠到 PE 阵列的空闲维度（cin / cout），把 Cin < 16 或 Cout < 16 的"小通道"层从固定 12.5% ~ 50% 的空间利用率拉到 95%+。

---

## 1. 背景：16×16 PE 阵列的浪费

MAC 阵列物理结构：16 列 × 16 PE，每列一个 cout，每行一个 cin。

- `min(Cin, 16) × min(Cout, 16)` 才是**有效活跃 PE 数**
- stem 层 `Cin=4 Cout=8`：活跃 PE = 32 / 256 = **12.5%**，其余 224 个 PE 空转
- L1 `Cin=Cout=8`：活跃 PE = 64 / 256 = **25%**

原始 ResNet-18 风格 22 层回归的加权平均 MAC util 只有 **22.1%**，大头卡在 stem（75% 总时间 × 12.5% util）。

---

## 2. Ky-fold：纯软件方案

### 2.1 核心思路

Cin 小意味着 PE 行维空闲。把卷积核的 Ky 维"折"成额外的 Cin 通道：

- 原卷积：`Σ_{ky, kx, c} W[ky, kx, c, co] × I[y·s+ky, x·s+kx, c]`
- 按 Ky 分组：每 `ky_per_group` 个 Ky 为一组，`groups_y = PE_H / Cin` 组
- 虚拟 Cin: `cin_fake = groups_y × Cin`
- 虚拟输入：`I'[y, x, g·Cin + c] = I[y + g·ky_per_group·s, x, c]`（y 方向偏移 g·ky_per_group 行的原图复制）
- 虚拟权重：`W'[ky_local, kx, co, g·Cin+c] = W[g·ky_per_group + ky_local, kx, co, c]`

### 2.2 数学等价性

```
Σ_{ky_local, kx, g, c} W'[...] × I'[...]
= Σ_{ky_local, kx, g, c} W[g·ky_per_group + ky_local, kx, co, c]
                        × I[y·s + g·ky_per_group·s + ky_local·s, x·s + kx, c]
= Σ_{ky, kx, c} W[ky, kx, co, c] × I[y·s + ky·s, x·s + kx, c]   ※ 令 ky = g·ky_per_group + ky_local

wait, 这里 stride 用法有点问题。严格推导：
I'[y, x, g·Cin + c] = I_padded[y + g·ky_per_group, x, c]
虚拟卷积 (stride=s) 的 I 索引 = I'[y_out·s + ky_local, ...]
                             = I_padded[y_out·s + ky_local + g·ky_per_group, x·s+kx, c]
                             = I_padded[y_out·s + (g·ky_per_group + ky_local), ...]
                             = I_padded[y_out·s + ky_orig, x·s + kx, c]  ✓
```

### 2.3 实现位置

**纯软件**，RTL 零改动：
- `toolchain/hw_files.py::compute_fold_params(K, Cin, PE_H)` 计算 (groups, ky_per_group, cin_fake, pad_ky)
- `fold_input()` 生成虚拟 ifm（y-偏移复制）
- `fold_weights()` 重排权重矩阵

### 2.4 效果

| 层 | 原 util (spatial) | Ky-fold 后 util |
|---|---:|---:|
| Stem C4C8 | 12.5% | 50% |
| L1 C8C8 | 25% | 48.7% |
| L2.B1.Conv1 C8C16 | 50% | 97.2% |

Ky-fold 单独使 22 层总时间从 8.45M → 3.50M cycles (**2.42× 加速**)。

---

## 3. Kx-fold：需要硬件支持

Kx-fold 要把 Kx 折到 **cout 维度**。但 cout 在 MAC 阵列里表现为列，**列之间共享同一个输入广播**——没法像 Ky-fold 那样通过输入预处理解决。

### 3.1 Systolic PSUM Shift 方案

核心思想：**所有列共享同一 x_in 广播，但每列写入 PSUM 的目标地址按列组偏移**。

```
col c (group g, co) 持有权重 W[ky, kx = g·kxper + kx_v, co, cin_row]

cycle (kx_v 外层, x_in 内层) 广播 I[y+ky, x_in, cin]:
  col c 的目标 x_out = (x_in - col_kx) / stride
                     = (x_in - g·kxper - kx_v) / stride
```

### 3.2 stride 约束：`kxper % stride == 0`

让所有 group 的 parity 对齐（否则每拍只有部分列能发射）。

若 `kxper % stride == 0`，则 `g·kxper/stride` 是整数，每列的 wr_addr 偏移是**静态整数**：

```
iss_pos = x_in / stride (line_buffer 的 iss_pos 迭代)
col c 的 wr_addr = iss_pos − g × (kxper / stride) = iss_pos − g × col_shift
```

所有列同拍同 parity，整个 16 列齐发，无需 parity gate。

### 3.3 iss_pos 迭代范围延长

原 line_buffer `iss_pos` 从 0 扫到 `cur_valid_w-1`。Kx-fold 下要延长 `(groups-1) × col_shift` 拍，给最后一个列组留 systolic 尾部：

```
cur_valid_w_ext = cur_valid_w + (groups-1) × col_shift
```

参与 iter 扩展的模块：`line_buffer`、`wgt_buffer`、`parf_accum`。三者必须同步（否则 handshake fire 数对不上死锁，这是调试中找到的 hang bug）。

### 3.4 PARF 重构为 per-col 存储

原 `parf_accum` 所有列共用一个 wr_addr。为 Kx-fold，拆成：

- `parf_col.sv`：单列 PSUM 存储（PARF_DEPTH × 32-bit）+ 独立 we/wr_addr/rdata
- `parf_accum.sv`：外壳只管计数、握手、fill/drain 状态；per-col 地址生成

```systemverilog
// parf_accum 每列 wr_addr
wr_addr_col[c] = wr_addr_base − col_group[c] × cfg_fold_col_shift

// 越界 mask (wr_addr_col 超出 [0, cur_valid_w_fill) 时禁写)
we_col[c] = fill_fire && in_range(wr_addr_col[c])
```

### 3.5 psum_reshape 组合归约级

Kx-fold 后 cout_fake = cout_orig × groups，但原始 Cout 数量不变。drain 时需把同一 cout 下的 groups 个 psum 求和（int32 精度）才能给 SDP：

```systemverilog
// psum_reshape.sv (纯组合, 不占 cycle)
for co in [0, cout_orig):
    out[co] = Σ_{g=0..groups-1} in[g × cout_orig + co]
for co in [cout_orig, 16):
    out[co] = 0
```

adder tree 深 `log2(max_groups) = 4` 级，远低于 10 ns budget。

### 3.6 K=1 的时序守护（Bubble）

K=1 时 `c_cur_round_len = 1`（WRF 只存 1 个 weight）。cins 切换瞬间：
- compute 前一拍读 WRF[0] = cins=k 权重
- loader 这一拍写 WRF[0] = cins=k+1 权重（2 拍流水）
- 下一拍 compute 读 WRF[0] 时 ram 还未更新 → 读到旧值，数据错

修复：`c_cur_round_len == 1` 时在 `round_wrap` 下一拍插 1 拍 bubble（`wgt_valid=0`），等 WRF 写完成。K≥2 的 round_len≥2 slot 自然错开，无此 hazard。

---

## 4. ARF 约束与 TILE_W

Kx-fold 下 `cur_fill_len = cur_valid_w_ext + K − 1` 可能超 ARF_DEPTH=32。软件选 TILE_W 时要预留 fold_tail：

```python
max_tile_w = ARF_DEPTH − K + 1 − fold_tail
# fold_tail = (groups_x − 1) × col_shift
```

### Kx-fold 参数选择算法（`compute_fold_params_kx`）

```python
for g in range(groups_max, 1, -1):            # groups_max = PE_W // Cout
    kxper_min = ceil(K / g)
    kxper = ceil(kxper_min / stride) * stride  # 向上取整到 stride 倍数
    if g × kxper >= K and kxper % stride == 0:
        pad_kx = g × kxper − K
        col_shift = kxper // stride
        # 取 pad_kx 最小 (tie-break 取 groups 更大)
```

---

## 5. 回归性能（Ky + Kx fold 叠加）

| 层 | 原始 | Ky+Kx fold | 加速 | MAC util |
|---|---:|---:|---:|---:|
| Stem K=7 C4C8 960×540 s=2 | 6,353,678 | 1,107,469 | **5.74×** | **99.9%** |
| L1 K=3 C8C8 ×4 | 1,197,584 | 578,080 | 2.07× | 96.3% |
| 其他 18 层（Cin/Cout≥16 或 K=1） | 903,635 | 880,067 | 1.00× | 不变 |
| **合计** | **8,454,897** | **2,565,616** | **3.29×** | - |

@ 100 MHz: 84.5 ms → **25.7 ms / 39 fps**
@ 400 MHz: 6.4 ms / **156 fps**

加权平均 MAC util：**22.1% → ~77%**（≈197 个 PE 平均活跃）

---

## 6. 受益层筛选规则

Fold 生效条件（`run_regression.py` 里的自动决策）：

| 优化 | 生效条件 |
|---|---|
| Ky-fold | `K > 1` AND `Cin < 16` |
| Kx-fold | `K > 1` AND `Cout < 16` AND `Cout` 是 16 的整约数（∈ {1,2,4,8}） |

K=1 的 Downsample / FC 层不受益（无 ky/kx 可折）。Cin / Cout ≥ 16 的主层已近满 util，无需 fold。

---

## 7. 相关文件

| 文件 | 职责 |
|---|---|
| `RTL/parf_col.sv` | 单列 PSUM 存储 |
| `RTL/parf_accum.sv` | parf 外壳 + per-col wr_addr 生成 |
| `RTL/psum_reshape.sv` | drain 时 cout 归约级 |
| `RTL/cfg_regs.sv` | FOLD_COUT_ORIG / FOLD_COUT_GROUPS / FOLD_COL_SHIFT 寄存器 |
| `RTL/line_buffer.sv` | iss_pos 扩展到 `cur_valid_w + fold_tail` |
| `RTL/wgt_buffer.sv` | x_cnt 同步扩展 + K=1 bubble |
| `toolchain/hw_files.py` | fold 工具：`compute_fold_params` / `compute_fold_params_kx` / `fold_input` / `fold_weights` |
| `toolchain/gen_isa_test.py` | `--ky-fold` / `--kx-fold` 命令行入口 |
