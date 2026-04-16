# 配置寄存器

核心控制器 `core_ctrl` 内部维护一组配置寄存器（`logic` 类型，重置到 0）。仿真中由 testbench 通过**分层引用**（`u_core_top.u_ctrl.cfg_*`）直接写入 —— 不暴露外部总线。运行时 FSM 只读这些寄存器，自身不改写。

所有**派生量**（`H_OUT`、各种 `_STEP`、`ROUNDS_PER_CINS`、`LAST_VALID_W` 等）由 `gen_isa_test.py` 预先算好写入，硬件零乘除。

---

## 完整寄存器列表

### 循环边界

| 寄存器 | 位宽 | 含义 | 软件计算 |
|--------|------|------|----------|
| `cfg_h_out` | 16 | 输出特征图高度 | `(H_IN - K) / stride + 1` |
| `cfg_w_out` | 16 | 输出特征图宽度 | `(W_IN - K) / stride + 1` |
| `cfg_w_in` | 16 | 输入宽度（ky 行步进单位） | 直接取 `W_IN` |
| `cfg_k` | 4 | 卷积核边长（1/3/5/7/9） | 直接取 `K` |
| `cfg_stride` | 3 | 卷积步长（1~7） | 直接取 `stride` |
| `cfg_cin_slices` | 6 | 输入通道切片数 | `⌈Cin / NUM_PE⌉` |
| `cfg_cout_slices` | 6 | 输出通道切片数 | `⌈Cout / NUM_COL⌉` |
| `cfg_tile_w` | 6 | 空间 tile 宽度（通常 32） | 固定 `TILE_W=32` |
| `cfg_num_tiles` | 8 | 每行的 tile 数 | `⌈W_OUT / TILE_W⌉` |
| `cfg_last_valid_w` | 6 | 末尾 tile 的有效宽度 | `W_OUT - (num_tiles-1) × TILE_W` |

### 权重调度

| 寄存器 | 位宽 | 含义 | 软件计算 |
|--------|------|------|----------|
| `cfg_total_wrf` | 10 | Packed 模式下 WRF 装载总数 | `K² × cin_slices` |
| `cfg_wrf_packed` | 1 | 1=packed, 0=chunked | `total_wrf ≤ 32` |
| `cfg_kk` | 10 | K²（chunked 每 cins 的轮总权重数） | `K × K` |
| `cfg_rounds_per_cins` | 3 | 每 cins 内的轮次数 | `⌈K² / WRF_DEPTH⌉` |
| `cfg_round_len_last` | 6 | 最后一轮的权重数 | `K² - (rounds-1) × WRF_DEPTH` |

三种权重调度模式：

- **Packed**：`K² × cin_slices ≤ 32`。一次 `LOAD_WGT` 灌满整个 cs 的所有 cins 权重。
- **Chunked 无轮次**：`K² ≤ 32 < K² × cin_slices`。每个 cins 开头一次 `LOAD_WGT` 灌 K² 权重。
- **Chunked 多轮次**：`K² > 32`（K≥7）。每个 (cins, round) 一次 `LOAD_WGT`，每轮 ≤32 权重。

### 地址基址（20-bit）

| 寄存器 | 位宽 | 含义 | 软件计算 |
|--------|------|------|----------|
| `cfg_ifb_base` | 20 | IFB 起始地址 | 通常 0 |
| `cfg_wb_base` | 20 | WB 起始地址 | 通常 0 |
| `cfg_ofb_base` | 20 | OFB 起始地址 | 通常 0 |

### 地址步进（20-bit，软件预乘好）

| 寄存器 | 位宽 | 含义 | 软件计算 | 硬件何时用 |
|--------|------|------|----------|----------|
| `cfg_ifb_cin_step` | 20 | cin_slice 之间的 IFB 跨度 | `H_IN × W_IN` | `cins++` 时 `r_ifb_cins += ...` |
| `cfg_ifb_row_step` | 20 | yout 行之间的 IFB 跨度 | `stride × W_IN` | `yout++` 时 `r_ifb_yout += ...` |
| `cfg_wb_cin_step` | 20 | cin_slice 之间的 WB 跨度 | `K × K` | chunked `cins++` 时 `r_wb_cins += ...` |
| `cfg_wb_cout_step` | 20 | cs 之间的 WB 跨度 | `K² × cin_slices` | `cs++` 时 `r_wb_cs += ...` |
| `cfg_ofb_cout_step` | 20 | cs 之间的 OFB 跨度 | `H_OUT × W_OUT` | `cs++` 时 `r_ofb_cs += ...` |
| `cfg_tile_in_step` | 20 | tile 之间的输入 x 方向跨度 | `TILE_W × stride` | `tile++` 时 `x_tile_in_r += ...` |

### SDP

| 寄存器 | 位宽 | 含义 |
|--------|------|------|
| `cfg_sdp_shift` | 5 | 反量化右移量（0~31） |
| `cfg_sdp_relu_en` | 1 | ReLU 使能 |

---

## config.txt 示例

`gen_isa_test.py` 输出的 `config.txt`（K=3, Cin=8, Cout=8, 68×120, stride=1）：

```
H_OUT = 66
W_OUT = 118
W_IN = 120
K = 3
STRIDE = 1
CIN_SLICES = 1
COUT_SLICES = 1
TILE_W = 32
TOTAL_WRF = 9
WRF_PACKED = 1
KK = 9
ROUNDS_PER_CINS = 1
ROUND_LEN_LAST = 9
IFB_BASE = 0
WB_BASE = 0
OFB_BASE = 0
IFB_CIN_STEP = 8160
IFB_ROW_STEP = 120
WB_CIN_STEP = 9
WB_COUT_STEP = 9
OFB_COUT_STEP = 7788
NUM_TILES = 4
LAST_VALID_W = 22
TILE_IN_STEP = 32
SDP_SHIFT = 0
SDP_RELU_EN = 1
```

---

## 运行时指针

FSM 另外维护一组**运行时指针**（20-bit），从基址 `+ = STEP` 演进：

| 指针 | 初值 | 更新时机 | 增量 |
|------|------|----------|------|
| `r_ifb_yout` | `cfg_ifb_base` | `yout++` | `cfg_ifb_row_step` |
| `r_ifb_cins` | `r_ifb_yout` | `cins++` | `cfg_ifb_cin_step` |
| `r_ifb_ky` | `r_ifb_cins` | `ky++`（cins 内） | `cfg_w_in` |
| `r_ofb_cs` | `cfg_ofb_base` | `cs++` | `cfg_ofb_cout_step` |
| `r_ofb_yout` | `r_ofb_cs` | `yout++` | `cfg_w_out` |
| `r_wb_cs` | `cfg_wb_base` | `cs++` | `cfg_wb_cout_step` |
| `r_wb_cins` | `r_wb_cs` | chunked `cins++` | `cfg_wb_cin_step` |
| `x_tile_out_r` | 0 | `tile++` | `cfg_tile_w` |
| `x_tile_in_r` | 0 | `tile++` | `cfg_tile_in_step` |
| `r_wrf_base` (5-bit) | 0 | packed `cins++` | `cfg_kk` |

运行时地址由指针 + 几个小偏移组成，例如：

```systemverilog
ifb_raddr = r_ifb_ky + x_tile_in_r + sub_ifb_offset + sub_cnt * eff_stride
ofb_waddr = r_ofb_yout + x_tile_out_r + sub_cnt
wb_raddr  = r_wb_cins + {round_cnt, 5'd0} + sub_cnt   // chunked
          = r_wb_cs + sub_cnt                          // packed
wrf_raddr = r_wrf_base + pos_in_round                  // 5-bit
```

`round_cnt * 32` 用 `{round_cnt, 5'd0}` 实现（零门电路）；`sub_cnt * eff_stride` 是全硬件上唯一的小乘法器（16×3-bit，面积可忽略）。
