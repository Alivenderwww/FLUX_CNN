# 硬件架构

## 1. 顶层数据通路

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                  core_top                                            │
│                                                                                      │
│   cfg_regs  (共享配置，对所有模块只读；TB 通过 u_cfg.r_* 层次引用写入)                 │
│   ═════════════════════════════════════════════════════════════════════════════════  │
│                                                                                      │
│   IFB ─► line_buffer ─────(act v/r)──► mac_array ──(psum v/r)──► parf_accum ──┐     │
│          (ring buffer)                  (elastic join,                        │     │
│           act_buf × 32                   2-stage pipe)                      (acc v/r)│
│           fifo_count                          ▲                                │     │
│           iss_pos + 6 层 counter              │                                │     │
│                                               │                                ▼     │
│                    WB ─► wgt_buffer ───(wgt v/r)─┘                        ofb_writer │
│                          (WRF write port ─────┘  (wrf_raddr/we/wdata)     + SDP 组合 │
│                           4 层 counter)                                        │     │
│                                                                                ▼     │
│                                                                               OFB    │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

TB 通过层次引用（`u_core_top.u_cfg.r_*`）直接写配置寄存器，拉 `start`，等 `done` —— 无指令总线、无外部 AXI 接口。

---

## 2. 模块清单

| 模块 | 文件 | 说明 |
|------|------|------|
| `core_top` | `RTL/core_top.sv` | 顶层连线 + 三块 SRAM 实例；无单 FSM |
| `cfg_regs` | `RTL/cfg_regs.sv` | 共享配置寄存器；TB 通过层次引用写入，对模块只读广播 |
| `line_buffer` | `RTL/line_buffer.sv` | IFB 读 + ring buffer + act 流；自跑 6 层 counter + iss_pos |
| `wgt_buffer` | `RTL/wgt_buffer.sv` | WB 读 + WRF LOAD + wgt 流；自跑 4 层 counter（仅 packed） |
| `mac_array` | `RTL/mac_array.sv` | 16×16 MAC；elastic join 握手；2 级 pipe valid 追踪 |
| `mac_col` | `RTL/mac_col.sv` | 单列：16 PE + 加法树 + 1 级 pipe reg（无 PARF） |
| `mac_pe` | `RTL/mac_pe.sv` | 单 PE：WRF + 1 拍流水乘法器；stall 下寄存器保持 |
| `parf_accum` | `RTL/parf_accum.sv` | 特殊 FIFO；FILL / DRAIN 独立 FSM + cross-tile overlap |
| `ofb_writer` | `RTL/ofb_writer.sv` | 4 层 counter + 内嵌 SDP + OFB 写端口 |
| `sdp` | `RTL/sdp.sv` | 纯组合：>> shift → ReLU → clip to uint8 |
| `std_rf` | `RTL/std_rf.sv` | 通用同步写/组合读 RF（WRF 用） |
| `sram_model` | `RTL/sram_model.sv` | 行为级 SRAM（1 拍读延迟），综合映射到 RAMB36 |

退役文件（磁盘保留但不 include）：`core_ctrl.sv`（原单 FSM）。已删除：`core_isa_pkg.sv`（Macro-ISA 包）。

---

## 3. 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `NUM_COL` / `NUM_PE` | 16 / 16 | 输出通道并行度 / 输入通道并行度 |
| `WRF_DEPTH` | 32 | 权重 RF 深度（packed 下 `K²·cin_slices ≤ 32`） |
| `ARF_DEPTH` | 32 | ring buffer 深度（等于 `TILE_W`；5-bit 模运算自然 wrap） |
| `PARF_DEPTH` | 32 | 部分和 RF 深度（= TILE_W 输出像素数） |
| `SRAM_DEPTH` | 8192（TB 可 override 到 ≤ 65536） | IFB / WB / OFB 深度 |
| `ADDR_W` | 20 | 内部地址/指针位宽（1M 字可达） |
| `DATA_WIDTH` | 8 | 激活值 / 权重位宽（int8） |
| `PSUM_WIDTH` | 32 | 部分和位宽（int32） |
| `TILE_W` | 32（固定） | 空间 tile 宽度 |

`ARF_DEPTH=32` 是 `line_buffer` ring buffer 正确工作的关键 —— 5-bit `wr_idx/rd_idx` 自然 mod 32 wrap。改动需要重新设计 ring buffer 寻址。

---

## 4. 三层数据复用

### 4.1 权重静止复用（Weight Stationary）

v1 只支持 **packed 模式**（`K²·cin_slices ≤ 32`）：每个 `cs` 开头 `wgt_buffer` 从 WB 灌 `cfg_total_wrf` 个权重到 mac_array 的 WRF，整个 cs 内所有 `(yout, tile, cins, ky, kx)` 迭代都从这组 WRF 读。`wrf_raddr = cins·K² + ky·K + kx`，用 3 个 running base（`wrf_base_cins / ky / kx`）零乘法推进。

Chunked 模式（K≥7 或 Cin>32）v1 暂不支持，见 `docs/roadmap.md`。

### 4.2 输出通道广播（Output Channel Broadcast）

每拍一次 IFB 读产生一个 128-bit `act_vec`（16 通道 int8），**同时广播给所有 16 列**。每列用自己 PE 内的 WRF 权重独立计算各自通道的部分和。单次握手 → 16 个输出通道的 MAC 结果。

### 4.3 Ring Buffer & Cross-round Pipeline

`line_buffer` 的 `act_buf` 是 32-entry ring buffer。`wr_idx / rd_idx` 是 5-bit 模运算，**不随 round 边界重置**。`iss_pos` 到 `cur_valid_w-1` 时同拍 advance 外层 counter + `ptr_kx_base`，下一拍立即 issue 新 round pos 0。老 round 的 last fire 和新 round 的 issue 0 在同一拍共享 act_buf —— 不同位置，无冲突。

这消除了 per-round 的 2 拍 startup bubble，整层只在开头有 1 次 startup（IFB 1 拍读延迟 + wr_idx 1 拍更新的物理下限）。

原 cfg-driven FSM 架构有 stride=1 滑窗复用（`kx=0` 载入 TILE_W 个像素，`kx=1..K-1` 每个只载 1 个新像素）。v1 没有这个优化，所有 `(ky,kx)` 都完整读 `cur_valid_w` 像素。恢复滑窗复用在 v2 roadmap。

---

## 5. PARF / parf_accum 的 cross-tile overlap

原架构 PARF 在 `mac_col` 内部，FILL（累加）和 DRAIN（读出）在同一条时间线上严格串行。

新架构把 PARF 外置到 `parf_accum` 模块，内部拆成两个独立 FSM：

- **FILL**：维护 `wr_addr / kk_cnt / cins_cnt / fill_tile_cnt`，每拍根据 `psum_in` 做累加或覆盖写入
- **DRAIN**：维护 `rd_addr / drain_tile_cnt / drain_active`，当 FILL 完成一个 tile 时 (`fill_tile_done`) 立即拉起 `drain_active`

关键：DRAIN 的 32 拍**隐藏在**下一个 tile 的 FILL first_round 的 32 拍里。同拍 drain 读 parf_data[X]（组合读 → 拿到 tile N 的旧值），fill 写 parf_data[X]（同步写 → 下一拍 register = tile N+1 first_round 值）。没有地址冲突。

为防止 fill 领先 drain 导致后续 drain 读到 tile N+1 覆盖值，overlap 期间强制同步：`psum_in_ready = acc_out_ready`（fill 只在 drain 也 fire 时 fire）。

---

## 6. mac_array 的 Elastic Join

mac_array 同时需要 `act` 和 `wgt` 两路 valid 才能 compute。单向 ready 会把 in-flight 数据丢失。采用 elastic join：

```systemverilog
can_advance = ~pipe_s2_valid | psum_in_ready;   // pipe 可前进一步
act_ready   = can_advance & wgt_valid;          // 只有双方都 valid 才消耗 act
wgt_ready   = can_advance & act_valid;
compute_en  = can_advance;                       // 传给 mac_pe / mac_col
```

`mac_pe.prod_out` 和 `mac_col.adder_tree_reg` 在 `compute_en=0`（stall 或 pipe 空泡）时**保持**（不清零），避免丢数据。2 级 pipe valid 位 (`pipe_s1_valid / pipe_s2_valid`) 独立追踪数据有无。

这是 v1 到 v3 架构功能正确性的**关键契约** —— 写代码不要修改握手方向或 ready 定义。
