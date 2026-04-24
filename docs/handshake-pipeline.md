# 握手流水 — 去中心化推进与气泡分析

## 1. 总体哲学

原架构用一个**中心 FSM**（`core_ctrl`）推进 6 层嵌套循环，所有模块在它的精确时序下被驱动。这种结构难以 pipeline、难以做跨 tile overlap、对 DMA 不友好（需要精确的时序假设）。

新架构让每个模块**自带 counter 独立推进自己的循环**，通过 valid-ready 握手对齐节拍。模块之间完全解耦 ——  某个模块慢了（比如 DMA 送数据不及时），握手天然背压，不用任何协调逻辑。

5 个模块各自管什么：

| 模块 | 管的 counter | 输出 | 输入 ready | 推进条件 |
|---|---|---|---|---|
| `line_buffer` | `cs/yout/tile/cins/ky/kx + iss_pos` | `act_valid + act_vec` | `act_ready` | issue_ok (buffer 有空间) |
| `wgt_buffer` | `cs/yout/tile/cins + kx/ky/x_cnt` | `wgt_valid + wrf_raddr + WRF write` | `wgt_ready` | wgt_fire |
| `mac_array` | pipe valid 位 | `psum_out_valid + psum_out_vec` | `psum_in_ready` | can_advance |
| `parf_accum` | `wr_addr/kk/cins/fill_tile` + `rd_addr/drain_tile` | `acc_out_valid + acc_out_vec` | `acc_out_ready` | fill_fire / drain_fire |
| `ofb_writer` | `cs/yout/tile/x` | OFB write | - | fire |

---

## 2. 握手契约（必读）

### 2.1 act (line_buffer → mac_array)

- `line_buffer` 内部 ring buffer 有数据时拉 `act_valid=1`
- `mac_array.act_ready = can_advance & wgt_valid`（elastic join，只在 pipe 能前进且对方也 valid 时消费）

### 2.2 wgt (wgt_buffer → mac_array)

- `wgt_buffer` 在 COMPUTE 阶段始终 `wgt_valid=1`（LOAD 阶段为 0）
- `mac_array.wgt_ready = can_advance & act_valid`

### 2.3 psum (mac_array → parf_accum)

- `mac_array.psum_out_valid = pipe_s2_valid`（2 级 pipe 尾部的数据有效位）
- `parf_accum.psum_in_ready = !overlap || acc_out_ready`
  - 非 overlap 时 fill 自由接收
  - overlap（drain 中 & fill 处于 first_round）时要和 drain 同步 fire

### 2.4 acc (parf_accum → ofb_writer)

- `parf_accum.acc_out_valid = drain_active`
- `ofb_writer.acc_out_ready = (state == S_RUN)`（没 DONE 就一直 ready）

### 2.5 关键不变量

- `mac_pe.prod_out` 和 `mac_col.adder_tree_reg` 在 `compute_en=can_advance=0` 时**保持**，不清零。确保 stall 下 in-flight 数据不丢失
- `parf_accum` 的 overlap 同步条件保证 fill 绝不领先 drain（否则后续 drain 读到被覆盖的值）
- `line_buffer` 的 `wr_idx/rd_idx` 是模 32 的 5-bit，永不随 round 边界重置

---

## 3. line_buffer v3 的 cross-round pipeline

### 3.1 问题背景

IFB 读有 **1 拍硬件延迟**，ring buffer 的 `wr_ptr++` 也有 **1 拍寄存器更新延迟**。v2 实现每 round 都重置 wr/rd 指针，所以 round 开头：

```
Cycle 0: issue 0,  no arrival, no fire (rd==wr==0)          ← 气泡拍 1
Cycle 1: issue 1,  arrival 0 (wr→1), no fire (rd==wr==0 still combinational) ← 气泡拍 2
Cycle 2: issue 2,  arrival 1 (wr→2), fire 0                  ← 连续 fire 开始
```

2 拍 startup bubble × 2376 rounds = 4752 拍浪费（K=3 C8C8 实测 ACT idle=4750）。

### 3.2 v3 解决方案

**核心洞察**：consume 侧不需要 counter —— 它只按顺序从 act_buf 读，不需要知道自己在哪个 round。只有 issue 侧需要 counter（生成 IFB 地址）。

做法：
- `wr_idx / rd_idx` 改 5-bit 模运算，不重置
- `iss_pos` 追踪当前 issue round 内位置；到 `cur_valid_w-1` 时**同拍** advance 外层 counter + ptr_kx_base
- 下一拍 issue 新 round 的 pos 0（用 NEW ptr_kx_base）
- 老 round 的 last fire 和新 round 的 issue 0 共享同一拍（不同 act_buf 位置，无冲突）

### 3.3 新时序（cur_valid_w=32）

```
Round N 末 / Round N+1 初 (T=32 对应 round N 的第 33 拍)：

T=31: issue N-31 (iss_pos=31, 当前 round 最后一个), arrival N-30, fire N-29
      Edge: iss_pos<=0, counters advance to round N+1, ptr_kx_base updates

T=32: issue N+1-0 (!, 用新 ptr_kx_base), arrival N-31 (到 act_buf[31]), fire N-30
      Edge: wr_idx<=0 (31+1 mod 32), rd_idx<=31

T=33: issue N+1-1, arrival N+1-0 (到 act_buf[0]; 覆盖已被消费的 old pos 0), fire N-31 (round_done OLD)
      Edge: wr_idx<=1, rd_idx<=0 (31+1 mod 32)

T=34: issue N+1-2, arrival N+1-1 (到 act_buf[1]), fire N+1-0 at act_buf[0]  ← 无 bubble!
      Edge: wr_idx<=2, rd_idx<=1
```

每个 round 32 拍（= `cur_valid_w`）= 32 fires，0 bubble。整层只有**开头 1 次 2 拍 startup**（首 round 的 IFB 读延迟无法避免）。

### 3.4 done 判定

不再用 `round_done` 作为循环状态驱动。改为：
```
done = issues_all_done && fifo_count == 0 && !ifb_re_d1
```
所有 issues 已发、ring buffer 已空、无 in-flight IFB 数据时即视为完成。

---

## 4. parf_accum 的 cross-tile FILL/DRAIN overlap

### 4.1 问题背景

PARF 的累加语义是 tile 内循环 `cin_slices × K²` 次，然后整个 tile drain 到 ofb_writer。v1 FILL 和 DRAIN 串行，每 tile 有 `cur_valid_w` 拍 drain 时 mac 被反压（`psum_in_ready=0`）。

```
K=3 C8C8: 264 tiles × 32 drain = 8448 拍（测 7240 左右，受 last tile valid_w=22 影响）
```

### 4.2 v2 解决方案

FILL 和 DRAIN 拆成独立 FSM：
- FILL state: `wr_addr / kk_cnt / cins_cnt / fill_tile_cnt`
- DRAIN state: `rd_addr / drain_tile_cnt / drain_active`

`fill_tile_done` 事件触发 `drain_active <= 1` 和 `drain_tile_cnt <= fill_tile_cnt`。之后：
- 下一个 tile 的 FILL first_round 和前一个 tile 的 DRAIN 同步进行
- 同拍 fill 写 parf_data[X]（覆盖），drain 读 parf_data[X]（组合读旧值）
- DFF 数组的读写时序保证 drain 拿 tile N 值，下拍 register = tile N+1 first_round 值

### 4.3 冲突避免（关键约束）

fill 不能领先 drain —— 若 fill 先写了 X，drain 后读 X 会读到 tile N+1 值。

强制同步：
```
overlap = drain_active && is_first_round_fill
psum_in_ready = overlap ? acc_out_ready : 1'b1
```
overlap 时 fill 只在 drain 也 fire 时 fire，确保 `wr_addr == rd_addr` 或 fill 落后 drain。

### 4.4 时序总览

```
Tile N fill: 32 × kk × cin_slices = 288 拍 (K=3 C8C8)
Tile N drain: 32 拍，完全隐藏在 Tile N+1 fill first_round 的前 32 拍里

Tile 0 fill:  |--- 288 拍 ---|
Tile 1 fill:               |--- 288 拍 ---|
Tile 0 drain:              |--32--|        ← 与 tile 1 fill first_round 并行
Tile 1 drain:                               |--32--|
```

---

## 5. mac_array 的 elastic join

### 5.1 问题背景

mac_array 需要 act + wgt **同时** valid 才能 compute。如果只看 `act_valid` 拉 `act_ready`，当 wgt_valid=0 时 act 会被"白消耗"——act 送进去但没真 MAC。下游的 line_buffer 推进 counter，数据丢失。

### 5.2 正确的 join 契约

```
can_advance = !pipe_s2_valid | psum_in_ready
              ↑ pipe 有空间或下游能接
act_ready   = can_advance & wgt_valid     ← 只在双方 valid 且 pipe 能前进时 fire
wgt_ready   = can_advance & act_valid
compute_en  = can_advance                 ← 传给 mac_pe / mac_col 作 pipe enable
```

`act_ready` 依赖 `wgt_valid` 创建了组合环？不，`wgt_valid` 来自 wgt_buffer 内部状态，不反向依赖 act 通路。OK。

### 5.3 Pipe valid 追踪

mac_array 有 2 级 pipe（mac_pe prod_out reg + mac_col adder_tree reg）：
```
pipe_s1_valid <= can_advance ? (act_valid & wgt_valid) : pipe_s1_valid
pipe_s2_valid <= can_advance ? pipe_s1_valid           : pipe_s2_valid
psum_out_valid = pipe_s2_valid
```

当 pipe 全空（两级 valid=0），第一个 fire 进 stage 0，1 拍后进 stage 1，2 拍后 psum_out_valid=1。这就是首层 startup 的 2 拍。

---

## 6. 气泡分析（K=3 C8C8 66×118 s=1 实测）

```
Total cycles:    70,128
True MAC fires:  70,092
Bubble:              36  (0.05%)

握手统计：
  ACT  (lb → mac):  fire=70092  stall=9    idle=0    ← 完美
  WGT  (wb → mac):  fire=70092  stall=0    idle=9    ← layer 收尾
  PSUM (mac → prf): fire=70092  stall=0    idle=42   ← pipe drain
  ACC  (prf → ofb): fire=7788   stall=0    idle=62339 ← drain 有效时间只 12%
```

剩余气泡分解：
- **36 拍** = layer 开头 2 拍 pipe startup + layer 结尾 ~34 拍 pipe drain + parf final drain
- ACT idle=0 说明 round boundary bubble 已**完全消除**
- PSUM stall=0 说明 parf drain 已**完全 overlap**

### 理论下限

**MAC fire 总拍数** = `cout_slices × h_out × sum_of_tile_widths × cin_slices × K²`

K=3 C8C8 66×118 = 1 × 66 × 118 × 1 × 9 = 70,092

v3 实测 70,128 = 理论 + 36 拍。紧贴极限。

---

## 7. 握手相关的已完成优化

- ✅ **Chunked 权重调度**：K≥7 或 Cin>32 时 wgt_buffer 按 round 分块 LOAD；J-3 起统一为单一 cins-ahead 流水路径（无需 packed 分支）
- ✅ **K=1 bubble**：`c_cur_round_len=1` 时 `round_wrap` 后插 1 拍 `wgt_valid=0`，解决 WRF 2 拍写流水 hazard（详见 `docs/pe-fold.md`）
- ✅ **Padding**：line_buffer 坐标越界判定，越界时 arf 喂 0（零代价硬件 padding）
- ✅ **Kx-fold systolic PSUM shift**：PARF 拆 per-col 存储 + 每列独立 wr_addr 偏移 + psum_reshape 归约级（详见 `docs/pe-fold.md`）

### 仍未实现

- **stride=1 滑窗复用**：当前 IFB 读 = `TILE_W × K`；原 cfg-driven 可复用 K-1 个像素。主要是 IFB 带宽节约
- **加法树流水**：16-PE 求和当前全组合，100 MHz 够用；拉到 ~300-500 MHz 需要 1-2 级 pipe reg
- **residual / pooling**：见 `docs/roadmap.md`
