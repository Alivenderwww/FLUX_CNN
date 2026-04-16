# 配置寄存器

共享模块 `cfg_regs` 维护一组配置寄存器（`logic` 类型，重置到 0）。仿真中由 testbench 通过**分层引用**（`u_core_top.u_cfg.r_*`）直接写入 —— 不暴露外部总线。运行时**所有 5 个顶层模块**通过只读端口从 `cfg_regs` 取配置，自身不改写。

所有**派生量**（`H_OUT`、各种 `_STEP`、`ROUNDS_PER_CINS`、`LAST_VALID_W` 等）由 `gen_isa_test.py` 预先算好写入，硬件零乘除。

---

## 完整寄存器列表

### 循环边界

| 寄存器 | 位宽 | 含义 | 软件计算 | 消费者 |
|--------|------|------|----------|--------|
| `h_out` | 16 | 输出特征图高度 | `(H_IN - K) / stride + 1` | line_buffer, wgt_buffer, ofb_writer |
| `w_out` | 16 | 输出特征图宽度 | `(W_IN - K) / stride + 1` | (未被任何模块直接引用，历史遗留) |
| `w_in` | 16 | 输入宽度（ky 行步进单位） | 直接取 `W_IN` | line_buffer |
| `k` | 4 | 卷积核边长（1/3/5/7/9） | 直接取 `K` | line_buffer, wgt_buffer |
| `stride` | 3 | 卷积步长（1~7） | 直接取 `stride` | line_buffer |
| `cin_slices` | 6 | 输入通道切片数 | `⌈Cin / NUM_PE⌉` | line_buffer, wgt_buffer, parf_accum |
| `cout_slices` | 6 | 输出通道切片数 | `⌈Cout / NUM_COL⌉` | line_buffer, wgt_buffer, ofb_writer |
| `tile_w` | 6 | 空间 tile 宽度（通常 32） | 固定 `TILE_W=32` | line_buffer, wgt_buffer, parf_accum, ofb_writer |
| `num_tiles` | 8 | 每行的 tile 数 | `⌈W_OUT / TILE_W⌉` | line_buffer, wgt_buffer, parf_accum, ofb_writer |
| `last_valid_w` | 6 | 末尾 tile 的有效宽度 | `W_OUT - (num_tiles-1) × TILE_W` | line_buffer, wgt_buffer, parf_accum, ofb_writer |

### 权重调度

| 寄存器 | 位宽 | 含义 | 软件计算 |
|--------|------|------|----------|
| `total_wrf` | 10 | Packed 模式下 WRF 装载总数 | `K² × cin_slices` |
| `wrf_packed` | 1 | 1=packed, 0=chunked | `total_wrf ≤ 32` |
| `kk` | 10 | K²（每 cins 的 kernel 位置数） | `K × K` |
| `rounds_per_cins` | 3 | 每 cins 内的轮次数（chunked） | `⌈K² / WRF_DEPTH⌉` |
| `round_len_last` | 6 | 最后一轮的权重数（chunked） | `K² - (rounds-1) × WRF_DEPTH` |

三种权重调度模式：

- **Packed**（v1 支持）：`K² × cin_slices ≤ 32`。一次 `LOAD_WGT` 灌满整个 cs 的所有 cins 权重
- **Chunked 无轮次**（v1 不支持）：`K² ≤ 32 < K² × cin_slices`
- **Chunked 多轮次**（v1 不支持）：`K² > 32`（K ≥ 7）

### 地址基址（20-bit）

| 寄存器 | 位宽 | 含义 | 软件计算 |
|--------|------|------|----------|
| `ifb_base` | 20 | IFB 起始地址 | 通常 0 |
| `wb_base` | 20 | WB 起始地址 | 通常 0 |
| `ofb_base` | 20 | OFB 起始地址 | 通常 0 |

### 地址步进（20-bit，软件预乘好）

| 寄存器 | 位宽 | 含义 | 软件计算 | 硬件何时用 |
|--------|------|------|----------|----------|
| `ifb_cin_step` | 20 | cin_slice 之间的 IFB 跨度 | `H_IN × W_IN` | line_buffer cins++ 时 running base 加 |
| `ifb_row_step` | 20 | yout 行之间的 IFB 跨度 | `stride × W_IN` | line_buffer yout++ 时 running base 加 |
| `wb_cin_step` | 20 | cin_slice 之间的 WB 跨度 | `K × K` | (v1 未用) |
| `wb_cout_step` | 20 | cs 之间的 WB 跨度 | `K² × cin_slices` | wgt_buffer cs++ 时 running base 加 |
| `ofb_cout_step` | 20 | cs 之间的 OFB 跨度（= h_out×w_out） | `H_OUT × W_OUT` | (v1 OFB 地址是连续递增，不直接用) |
| `tile_in_step` | 20 | tile 之间的输入 x 方向跨度 | `TILE_W × stride` | line_buffer tile++ 时 running base 加 |

### SDP

| 寄存器 | 位宽 | 含义 |
|--------|------|------|
| `sdp_shift` | 5 | 反量化右移量（0~31） |
| `sdp_relu_en` | 1 | ReLU 使能 |

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

## 各模块如何使用 cfg（地址生成示例）

### line_buffer 的 IFB 地址

维护 5 层 running base，每层 counter 推进时该层 base 加对应 step：

```systemverilog
// 每层 base 的更新规则 (see line_buffer.sv advance_counters_and_ptrs)
yout++ : ptr_yout_base += cfg_ifb_row_step
tile++ : ptr_tile_base += cfg_tile_in_step
cins++ : ptr_cins_base += cfg_ifb_cin_step
ky++   : ptr_ky_base   += cfg_w_in
kx++   : ptr_kx_base   += 1

// 最终地址（每拍）
ifb_raddr = ptr_kx_base + iss_pos * cfg_stride
           // iss_pos * stride 是唯一的小乘法器（stride ≤ 2 综合为 shift）
```

### wgt_buffer 的 WRF 地址

只维护 3 层 running base（wrf_base_cins / ky / kx），因为 WRF 地址本身只到 5-bit：

```systemverilog
// WRF 写（LOAD 阶段）
wb_raddr  = cur_wb_base_cs + wb_rd_cnt
wrf_waddr = wrf_wr_cnt[4:0]

// WRF 读（COMPUTE 阶段）
wrf_raddr = wrf_base_kx[4:0]
          = (cins_cnt * kk + ky_cnt * k + kx_cnt) 的 running 累加
```

### parf_accum 的 tile 切换

`parf_accum` 只用 5 个字段（`tile_w / last_valid_w / num_tiles / cin_slices / kk`）判定当前 tile 宽度和累加轮次：

```systemverilog
cur_valid_w_fill = (fill_tile_cnt == num_tiles-1) ? last_valid_w : tile_w
cur_valid_w_drain = ...
```

### ofb_writer 的 OFB 写地址

因 OFB 物理连续（`ofb_cout_step = h_out × w_out`，一行 `w_out = sum(tile widths)` 完美覆盖），地址只需从 `ofb_base` 起每 fire +1。其他 cfg 只用来判 counter 边界。

---

## 软件预计算契约

`gen_isa_test.py` 在 `compute_cfg()` 生成所有 step 值。硬件**绝不**自己做乘除。改 cfg 语义时，Python 和 RTL 两边必须同步更新。典型的修改流程：

1. 在 `cfg_regs.sv` 加新字段 `r_xxx` + 对应只读端口
2. 在消费者模块加 input port 和使用逻辑
3. 在 `core_top.sv` 连线 `u_cfg.xxx → u_mod.cfg_xxx`
4. 在 `gen_isa_test.py::compute_cfg()` 计算并写 `config.txt`
5. 在 `tb_core_isa.sv::load_config()` 加一条 case 把 `config.txt` 的 key 映射到 `u_core_top.u_cfg.r_xxx`
