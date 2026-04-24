# 硬件架构

## 1. 顶层数据通路

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                  core_top                                                │
│                                                                                          │
│   cfg_regs  (共享配置, 通过 axi_lite_csr 外部写; 对核流水模块只读广播)                    │
│   ═══════════════════════════════════════════════════════════════════════════════════════│
│                                                                                          │
│   IFB ─► line_buffer ───(act v/r)──► mac_array ──(psum v/r)──► parf_accum ──► psum_reshape ──► ofb_writer │
│          (ring buffer)                (elastic join,          (parf_col ×16, (组合归约级,          (SDP + OFB write) │
│           act_buf × 32                 2-stage pipe)           per-col       Kx-fold cout                          │
│           fold-aware iss_pos)          ▲                       wr_addr)      归约)                                 │
│                                        │                                                                           │
│                   WB ─► wgt_buffer ────┘                                                                          │
│                        (cins-ahead                                                                                 │
│                         chunked loader                                                                             │
│                         + K=1 bubble)                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

host 通过 AXI-Lite（`axi_lite_csr`）写 `cfg_regs` + 下发 DMA descriptor，拉 `CTRL[5]`（start_layer）启动；`CTRL[4]`（start_dfe）触发 descriptor fetch。

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
| `parf_col` | `RTL/parf_col.sv` | 单列 PSUM 存储（PARF_DEPTH × 32-bit）+ 独立 we/wr_addr/rdata |
| `parf_accum` | `RTL/parf_accum.sv` | parf 外壳：FILL/DRAIN 独立 FSM + cross-tile overlap + per-col wr_addr 生成（Kx-fold 偏移） |
| `psum_reshape` | `RTL/psum_reshape.sv` | drain 组合归约级：Kx-fold 时 cout_groups × cout_orig 汇总成 cout_orig 路 int32 psum |
| `ofb_writer` | `RTL/ofb_writer.sv` | 4 层 counter + 内嵌 SDP + OFB 写端口 |
| `sdp` | `RTL/sdp.sv` | 纯组合：mult → shift → round → ReLU → clip |
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

J-3 起**统一走 chunked cins-ahead 流水路径**（原 packed 分支已废弃）。`wgt_buffer` 以 FSM `S_IDLE → S_BIAS_LOAD → S_LOAD (cold round 0) → S_COMPUTE (load/compute overlap) → S_DONE`：

- WRF 容量 `WRF_DEPTH=32`。每轮最多装 32 个权重（=`c_cur_round_len`）。
- K² ≤ 32（K ≤ 5）时 `rounds_per_cins=1`，round_len=K²；K=7 时 `rounds_per_cins=2`（32+17）
- compute 在 S_COMPUTE 期间连续 fire，loader 通过 hazard 检查（`pos_cnt > l_pos` 等）边算边灌下一 round / cins，实现完整 overlap
- 跨 tile / cs / yout 的权重地址链通过 `l_wb_base / l_wb_base_cs` 维护，无需再退回 S_LOAD

**K=1 守护**：`c_cur_round_len == 1` 时 WRF 每 cins 切换仅 1 个 slot，2 拍流水写不及时。在 `round_wrap` 下一拍插 1 拍 `wgt_valid=0` bubble 等 WRF 更新完成，仅 K=1 场景触发。

### 4.2 输出通道广播（Output Channel Broadcast）

每拍一次 IFB 读产生一个 128-bit `act_vec`（16 通道 int8），**同时广播给所有 16 列**。每列用自己 PE 内的 WRF 权重独立计算各自通道的部分和。单次握手 → 16 个输出通道的 MAC 结果。

### 4.3 Ring Buffer & Cross-round Pipeline

`line_buffer` 的 `act_buf` 是 32-entry ring buffer。`wr_idx / rd_idx` 是 5-bit 模运算，**不随 round 边界重置**。`iss_pos` 到 `cur_valid_w-1` 时同拍 advance 外层 counter + `ptr_kx_base`，下一拍立即 issue 新 round pos 0。老 round 的 last fire 和新 round 的 issue 0 在同一拍共享 act_buf —— 不同位置，无冲突。

这消除了 per-round 的 2 拍 startup bubble，整层只在开头有 1 次 startup（IFB 1 拍读延迟 + wr_idx 1 拍更新的物理下限）。

原 cfg-driven FSM 架构有 stride=1 滑窗复用（`kx=0` 载入 TILE_W 个像素，`kx=1..K-1` 每个只载 1 个新像素）。v1 没有这个优化，所有 `(ky,kx)` 都完整读 `cur_valid_w` 像素。恢复滑窗复用在 v2 roadmap。

---

## 5. PARF 架构（per-col 存储 + cross-tile overlap）

**K 阶段重构**：`parf_accum` 拆成外壳 + 16 个 `parf_col` 子模块：

- `parf_col`：单列 PSUM 存储（PARF_DEPTH × 32-bit）+ 独立 write port（we/wr_addr/wdata）+ 组合读 port
- `parf_accum`：外壳只管计数器（`wr_addr_base / kk_cnt / cins_cnt / fill_tile_cnt` + `rd_addr / drain_tile_cnt / drain_active`）、握手、per-col 地址生成

每列的 wr_addr 可独立：

```systemverilog
wr_addr_col[c] = wr_addr_base − col_group[c] × cfg_fold_col_shift
we_col[c]      = fill_fire && in_range(wr_addr_col[c])
```

无 Kx-fold 时 `cfg_fold_col_shift=0`，所有列同 wr_addr，行为等价原单体 parf。有 Kx-fold 时列组按 `col_shift` 偏移，实现 systolic psum shift。

### Cross-tile FILL/DRAIN overlap（保留）

- **FILL**：`wr_addr / kk_cnt / cins_cnt / fill_tile_cnt` 推进，每拍 per-col 累加写入
- **DRAIN**：`rd_addr / drain_tile_cnt / drain_active`；`fill_tile_done` 事件拉起 `drain_active`
- DRAIN 的 `cur_valid_w_drain` 拍**隐藏在**下一 tile 的 FILL first_round 里。同拍 drain 读（组合 → 旧值）、fill 写（同步 → 下拍 = 新值）
- overlap 同步约束：`psum_in_ready = (!overlap || acc_out_ready)` 保证 fill 不领先 drain

### Kx-fold 下的 cur_valid_w 扩展

```
cur_valid_w_fill_ext = cur_valid_w_fill + (cfg_fold_cout_groups − 1) × cfg_fold_col_shift
```

line_buffer、wgt_buffer 同步扩展 iss_pos / x_cnt 迭代范围（fire 数要对齐，否则 handshake 死锁 —— 这是 K-2 调试中找到的 hang bug）。

---

## 6. psum_reshape 归约级

Kx-fold 下虚拟 cout_fake = cout_orig × groups。drain 时需把 groups 份 int32 psum 合并：

```systemverilog
// psum_reshape.sv (纯组合, 0 延迟)
for (co = 0; co < cout_orig; co++)
    out[co] = Σ_{g=0..cout_groups−1} in[g × cout_orig + co]
for (co = cout_orig; co < NUM_COL; co++)
    out[co] = 0
```

adder tree 深 `log2(16)=4` 级。无 fold 时 `cout_orig=NUM_COL cout_groups=1`，逻辑直通。

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
