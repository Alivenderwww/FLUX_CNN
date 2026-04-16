# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FLUX_CNN is a CNN accelerator in SystemVerilog. A 16×16 int8 MAC array (256 MACs) is orchestrated by **5 个去中心化模块 + 共享配置寄存器**，模块间靠 valid-ready 握手串起流水。每个模块自带 counter 独立推进循环，不再有任何中心 FSM。

历史包袱：早期有过 Macro-ISA (OP_LDnMAC 等 64-bit 指令)，后来简化为 `core_ctrl` 单 FSM 根据配置寄存器自驱动 6 层循环。两者都已退役 —— 现在是 v3 握手架构，MAC 利用率紧贴 99.95% 理论极限。`docs/isa-legacy.md` 记录历史。

## 当前架构（v3 cross-round pipeline）

```
cfg_regs (共享配置；TB 通过 u_core_top.u_cfg.r_* 层次引用写入)
    │
    ▼  (所有模块只读)

IFB ─► line_buffer ─(act v/r)─► mac_array ─(psum v/r)─► parf_accum ─(acc v/r)─► ofb_writer ─► OFB
                                    ▲                                               │
       WB ─► wgt_buffer ─(wgt v/r)──┘                                              SDP(组合)
                          (+ WRF write port)
```

**5 个模块各自跑自己的循环**：
- `line_buffer`：6 层 (cs/yout/tile/cins/ky/kx) + 内部 iss_pos，ring buffer (32-entry) 存 IFB 读出数据 → act 流
- `wgt_buffer`：4 层 (cs/cins/round/pos)，从 WB 灌 WRF + COMPUTE 阶段发 wrf_raddr（v1 只支持 packed, `K²·cin_slices ≤ 32`）
- `mac_array`：纯计算 + elastic join 握手；`can_advance = !pipe_s2_valid | psum_in_ready`
- `parf_accum`：**特殊 FIFO**，FILL/DRAIN 两个独立 FSM，drain 隐藏在下一 tile fill 的 first_round
- `ofb_writer`：4 层 (cs/yout/tile/x) + 内部 SDP（shift/ReLU/clip），写 OFB

核心性能数据（K=3 C8C8 66×118 s=1）：**70,128 cycles, MAC 利用率 99.95%**，理论下限 70,092（MAC fire 拍数）。

`core_ctrl.sv` 文件还在磁盘但已从 file list / synth tcl 剔除，不再参与编译。

## Common Commands

All simulation commands **must be run from `sim/tb_core_isa/`** (TCL and `.f` files use relative paths):

```bash
cd sim/tb_core_isa

# 1. 生成测试数据 + 配置 + 金标准
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --stride 1 --num_cin 8 --num_cout 8 --shift 0
# flags: --k --h_in --w_in --stride --num_cin --num_cout --tile_w (保持 32) --shift --seed

# 2. 跑仿真
vsim -c -do tb_core_isa.tcl       # 批处理
# vsim -do tb_core_isa.tcl        # GUI

# 3. 跑回归
python run_regression.py --label v3-handshake --out regression_v3.txt
```

Synthesis 见 `Syn/run_syn.tcl`（非主要关注点）。

## Architecture 细节（写代码必读）

### 握手契约

**每条 (*) = {data, valid, ready}**：
- `line_buffer.act_valid` / `mac_array.act_ready`
- `wgt_buffer.wgt_valid` / `mac_array.wgt_ready`
- `mac_array.psum_out_valid` / `parf_accum.psum_in_ready`
- `parf_accum.acc_out_valid` / `ofb_writer.acc_out_ready`

**mac_array 的 elastic join** — 同时需要 act+wgt valid，下游 psum_in_ready 穿透到两侧：
```
can_advance = ~pipe_s2_valid | psum_in_ready
act_ready   = can_advance & wgt_valid
wgt_ready   = can_advance & act_valid
```

`mac_pe.prod_out` 和 `mac_col.adder_tree_reg` 在 `compute_en=can_advance` 低时**保持**（不清零），保证 stall 下 in-flight 数据不丢失。

### line_buffer v3 timing (关键)

- `iss_pos` 追踪当前 round 内位置，到 `cur_valid_w-1` 时 **同拍** advance 外层 counter + `ptr_kx_base`，下一拍直接 issue 新 round 的 pos 0
- `wr_idx` / `rd_idx` 是 5-bit 模运算（自然 mod 32），**永不随 round 边界重置**
- `fifo_count` 做 back-pressure：`issue_ok = (fifo_count + ifb_re_d1 < ARF_DEPTH)`
- `done = issues_all_done && fifo_count==0 && !ifb_re_d1`（drain 空 pipe）
- **不要碰 ring buffer 模运算**：改非模运算会立刻破坏 cross-round pipeline

### parf_accum 的 FILL/DRAIN overlap

- 两个独立 FSM：`wr_addr/kk_cnt/cins_cnt/fill_tile_cnt`（FILL） vs `rd_addr/drain_tile_cnt`（DRAIN）
- `fill_tile_done` 触发 `drain_active <= 1`，下一 tile 的 fill first_round 和 drain 同步进行
- **overlap 时同步规则**：`psum_in_ready = acc_out_ready`（fill 不能领先 drain，否则后续 drain 读到 tile N+1 覆盖值）
- 地址冲突避免依赖 DFF 数组的**组合读 + 同步写**（同拍 drain 读旧值、fill 写新值，互不影响）

### 运行指针（地址生成，零乘法）

所有地址推进都是 `base + step` 加法。`line_buffer` 维护 5 层 base (`ptr_yout_base / ptr_tile_base / ptr_cins_base / ptr_ky_base / ptr_kx_base`)，每层 counter 推进时该层 base 加对应 step，更内层 base 归位到新值。**唯一的小乘**是 `iss_pos * stride`，stride ∈ {1,2} 综合时退化为 shift/wire。

### v1 限制（v2 待办）

- **不支持 chunked**：`K²·cin_slices > 32` 的配置（K=7、C64+）跑不通，因为 wgt_buffer 假设 packed `total_wrf ≤ 32`
- **没有 stride=1 滑窗复用**：每个 `(ky,kx)` 都重跑一次 IFB 读（原 cfg-driven 有此优化）；对于 stride=2 无差别

## Project Conventions

- Language: **SystemVerilog (IEEE 1800)** — packed structs, enums, 无 packages (`core_isa_pkg` 已删)
- Parameters: `NUM_COL=NUM_PE=16`, `WRF_DEPTH=ARF_DEPTH=PARF_DEPTH=32`, `DATA_WIDTH=8`, `PSUM_WIDTH=32`, `SRAM_DEPTH=8192`
- 内部地址位宽：`ADDR_W=20`（1M words 可达）
- `gen_isa_test.py` 是 derived value 运算的 source of truth（step 大小、round_len、OFM 维度）。改 cfg 语义时两边同步更新
- 回归 baseline 存为 checked-in `regression_v3.txt`
- Commit 用 Chinese prefix (`Feat:`, `Perf:`, `Docs:`, 等)
- 加新 RTL 模块要同步更新 `sim/tb_core_isa/sim_file_list.f` 和 `Syn/run_syn.tcl`

## 文档导航

- `README.md` — 顶层叙述 + 性能表
- `docs/architecture.md` — 模块层次、数据通路、握手拓扑
- `docs/handshake-pipeline.md` — 握手协议、各模块推进、气泡分析
- `docs/config-registers.md` — 配置寄存器位宽 + 语义 + 预计算规则
- `docs/simulation.md` — 单测、回归、TB cfg poke 机制
- `docs/roadmap.md` — chunked、stride=1 滑窗、padding、残差、pooling 等未来工作
- `docs/isa-legacy.md` — 历史（Macro-ISA 和 cfg-driven FSM 两层）
- `model_analysis.md` — 目标模型 PE 利用率分析
- `docs/synthesis.md` — 综合 flow（非主要关注点，只作参考）
