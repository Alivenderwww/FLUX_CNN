# PE 利用率优化：Ky-fold / Space-to-Depth

16 × 16 PE 阵列只有当 `Cin ≥ 16 && Cout ≥ 16` 时才能填满。Cin 小时空间利用率掉到 `Cin/16`；Cout 小时掉到 `Cout/16`。当前架构只处理 Cin 小的情况——通过两种编译器侧的折叠把 PE 行填满。Cout 小时硬件不做任何复用，对应 PE 列空转。

- **Ky-fold**：Cin 小时把卷积核 Ky 维折到 cin 维，编译器侧输入 y 方向偏移复制
- **Space-to-Depth (S2D)**：stride ≥ 2 时把 (kx%stride, ky%stride) 相位折到 cin 维，编译器侧数据重排不复制

两种 fold 都是纯编译器层面的输入 / 权重重排，硬件按普通 stride=1 conv 跑。

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

## 2. Space-to-Depth（编译器侧）

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
- reuse_en=1 模式 ky 边界 K 拍 FILL 启动惰性，对 cycles 有 ~3% 量级影响

## 3. Cout 小的情况

不做硬件复用，对应 PE 列空转，输出实际利用率 = `Cout / 16`。这是当前架构的取舍——避免引入 Kx-fold 必需的列归约级（psum_reshape）和 parf_accum 内的每列地址偏移逻辑，硬件保持简洁。

如果未来想恢复 Cout 小的复用，参考 [`docs/roadmap.md`](roadmap.md) 里的"Kx 维度复用"项。

## 4. ARF 容量约束

ARF reuse_en=1 启用条件 `stride==1 && K>1`：FILL 长度 = `cur_valid_w + K - 1`，必须 ≤ `ARF_DEPTH = 32`，所以 `tile_w ≤ 33 - K`。S2D 后 stride=1 时这条约束起作用。

## 5. 受益层判定

`run_regression.py` 自动决策：

| 优化 | 触发条件 |
| --- | --- |
| Ky-fold | `K > 1 AND Cin < 16` |
| S2D | `stride ≥ 2 AND K ≥ stride` |

S2D 启用后 Cin 变为 stride²·Cin，重新判定 Ky-fold 触发条件（多数情况下 S2D 后 Cin' ≥ 16 不再需要 Ky-fold）。

K=1 / FC 层不受益（无可折维度）。Cin ≥ 16 的层已近满 PE 行利用，不需要 fold。

## 6. 相关 RTL / 工具链文件

| 文件 | 内容 |
| --- | --- |
| `RTL/parf_col.sv` | 单列 PSUM 存储（每列独立 SRAM） |
| `RTL/parf_accum.sv` | PARF 外壳（共享 wr_addr / we / rd_addr） |
| `toolchain/hw_files.py` | `compute_fold_params` / `fold_input/weights` / `compute_s2d_params` / `s2d_input/weights` |
| `toolchain/gen_isa_test.py` | `--ky-fold` / `--s2d` |
