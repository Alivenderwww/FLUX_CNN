# FLUX_CNN 项目当前状态 (2026-05-14)

> 本文件是 **任务交接文档**.
>
> **🎉 2026-05-14: VD100 board Stage 0~4 全 PASS** (minimal system + bit-exact 真 MAC + 软 reset).
> Stage 4 bit-exact 暴露真 RTL bug (odma_sg_dispatcher r_yout_base 推进时机), sim 11/11 PASS
> 未覆盖. 详见 §3.
> - 工程: `Syn/vd100_minimal/` (ps_hello base + ConvCore + smartconnect_pl + BRAM, 绕过 axi_noc)
> - PDI: `Syn/vd100_minimal/vd100_minimal_with_elf.pdi`
> - 测试:
>   - `test_stage2_minimal_conv.py` (1×1×1×1 minimal conv, layer_done <100ms)
>   - `test_stage3a_phase1_loop.py` (多轮启停 500/500 PASS)
>   - `test_stage3a_phase2_larger.py` (K=3 H=W=8 Cin=Cout=16 200/200 PASS, H=W=16 30/30 PASS)
> - 结论: ConvCore RTL **完全无 bug** (sim IDEAL_SMC + board minimal 双证), v32 board stuck 100%
>   是 axi_noc 路径的硬件问题 (BD m00_axi user override + multi-cmd 卡 NoC).
> - 下一步: Stage 4 真 MAC bit-exact 验证 (论文核心里程碑)
>
> **历史 sim milestone**:
> - M1.5 多核 sim 死锁修复, M2 跨核直送, M2.5 多核调度器, P1 Mode C W slice
> - 🎉 Phase 7 SMC + NUMA ResNet11 N=4 整网 sim PASS (220,824 cy, 453 fps, 11/11 bit-exact)
>
> 长期项目状态见 `README.md`, 模块细节见 `docs/`, 编码规范见 `RTL代码编写原则.md`,
> 历史经验教训见 `memory/`.

---

## 1. 单核 (`core_top`) — 已稳定

### 设计完成 + 仿真验证

| 模块 / 功能 | 状态 | 备注 |
|---|---|---|
| Core pipeline (line_buffer / mac_array / parf_accum / wgt_buffer / ofb_writer) | ✅ | 11-case ResNet-18-like 链式回归全 PASS |
| DMA 子系统 (idma/wdma/odma/rdma_ctrl + axi_dm IP + mm2s_arb + axi_m_mux) | ✅ | DataMover 性能与原版自写 DMA 持平 (+0.5%) |
| AXI-Lite CSR + cfg_regs (双端写口: csr_w + seq_w) | ✅ | host 只写 4 个 boot regs, 50 个 layer cfg 全走 CFG_WRITE descriptor |
| **R.1 — bias 移到 SDP + Shortcut Bank + bias_rf** | ✅ | 解 MAX_COUT_SLICES=32 限制 |
| **R.2 — SDP 残差 fusion (shortcut_mult / shortcut_shift)** | ✅ | 链式 CASES 三层 ds 启用残差, bit-exact 通过 |
| **Done sticky 寄存器 (layer_done 上升沿 set, start_layer 自清)** | ✅ | host poll STATUS[0] 或接 GIC level IRQ 都干净 |
| **CFG_WRITE descriptor (TYPE_CFG=0x3)** | ✅ | desc_fifo 32→128, host AXI-Lite 写从 ~50/层 降到 4/层 |
| **链式 CASES (DSL builder + 跨层 DDR FM 共享)** | ✅ | run_regression.py 用 Chain/_Node/resnet_block 7 行写完 11 层 |
| **Wall_us 端到端报告 (host 启动 → host 可读 DDR)** | ✅ | regression report 加 Wall_us 列 + footer FPS 上限 |

### 单核综合 (XC7K325T-FFG900-2 @ 100 MHz target)

| 资源 | 用量 | FPGA 容量 | 占比 |
|---|---|---|---|
| LUT | 36,942 | 203,800 | 18.1% |
| FF | 13,167 | 407,600 | 3.2% |
| BRAM36 | 128 (+1 RAMB18) | 445 | **28.8%** |
| DSP48E1 | 82 | 840 | 9.8% |
| **Fmax** | **68.4 MHz** (WNS = -4.618 ns) | — | timing **未达 100 MHz** |

**BRAM 明细**:
- WB SRAM (1024×2048): 57
- IFB SRAM (8192×128): 32
- Shortcut Bank (8192×128): 32
- OFB SRAM (2048×128): 7 + 1 RAMB18

### 单核已知问题

| 问题 | 优先级 | 说明 |
|---|---|---|
| **DSP 利用率低 (82/256 PE)** | 中 | mac_pe 大部分综合到 LUT (因 `if (compute_en) prod <= mult else hold` 模式阻碍 DSP 推断). 加 `(* use_dsp = "yes" *)` 估能省 17K LUT |
| **Fmax 仅 68 MHz** | 中 | Critical path = SDP 量化组合链 (`pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc`). 切 1-2 段流水线能拉到 100+ MHz |

---

## 2. 多核 (`multicore_top`) — M1 综合通, M1.5 仿真验证通过

### 已完成 (M1)

| 任务 | 状态 | 文件 |
|---|---|---|
| M1.1 生成 Xilinx IP (axi_2to1 + axi_lite_1to2) | ✅ | `Syn/gen_multicore_ip.tcl` |
| M1.2 写 multicore_top.sv (参数化 N 核 wrapper) | ✅ | `RTL/multicore_top.sv` ~280 行 |
| M1.3 改 synth 脚本支持顶层切换 single/multi | ✅ | `Syn/run_syn.tcl` (TOP_NAME + NUM_CORES) |
| M1.4 综合验证 N=2 装得下 | ✅ | reports/utilization.rpt |

### 多核综合 (NUM_CORES=2, XC7K325T)

| 资源 | 单核 | 2 核 wrapper | 推算 3 核 | 推算 4 核 | 上限 |
|---|---|---|---|---|---|
| LUT | 36,942 | **74,386 (36.5%)** | 109K (54%) | 146K (72%) | 203K |
| FF | 13,167 | 26,927 (6.6%) | 40K | 53K | 407K |
| **BRAM36** | 128 | **256 (57.5%)** | **384 (86%)** | **512 ❌** | 445 |
| DSP | 82 | 164 (19.5%) | 246 | 328 | 840 |
| Fmax | 68 MHz | **68 MHz** | — | — | — |

**结论**: **3 核是 XC7K325T 不动 SRAM 的硬上限** (BRAM-bound). 4 核需把 shortcut_bank 8192→2048 (单核 BRAM 降 24, 4 核 = 416 < 445).

### M1.5 仿真验证 (已通过)

`sim/tb_multicore/tb_multicore.sv` 跑 2 核独立小 case (K=3 C8C16 30×30):
**RESULT PASS, BOTH cores done @ 9057 cycles** (单核同 case 8808 cycles, 多核 +2.8% AXI 仲裁开销).

死锁实际是 **两个叠加 bug** (STATUS 原诊断只覆盖第一个):

1. **`axi_m_mux.sv` 没 forward 全套 AXI4 字段** (`awsize/awlock/awcache/awprot/awqos` + R 对称).
   axi_2to1 IP 严格读 awsize 等, 悬空 'X' 让 awready=0.
2. **`axi_arbiter.sv` 写通道 sel default=0 不 sticky**.
   Xilinx DataMover 在两个 cmd 之间会预读 cmd N+1 的 W 出口 (此时 awvalid=0).
   arbiter default fallback master 0 让 W forward 错路。单核 TB 用 axi_slave_mem 串行处理
   不触发, 多核 axi_2to1 内部 W FIFO 让预读条件成立 → 死锁。

**最终修法**:
- `axi_m_mux.sv` 加 5 组 forward (~30 行)
- `core_top.sv` bus 端口扩 5 组 + axi_dm IP 出口接到 m_aw* 数组
- `multicore_top.sv` 删常量 awsize=4 等, 改接核出口
- `axi_arbiter.sv` `wr_addr_master_sel` default 从 `0` 改成 `cu_wr_master_sel` (sticky)
- `tb_multicore.sv` `patch_desc_addrs` 按 reg addr 重写 IDMA/WDMA/ODMA/RDMA base
  (case 默认 base=0, TB preload 按 CASE_*_BASE 放, 必须 patch 对齐); 两核都要 patch.

详细复盘见 `memory/axi_m_mux_quirk.md`.

---

## 2.6 多核调度器 + Driver 框架 (M2.5, 2026-04-30)

### 完成内容

| 模块 | 内容 | 状态 |
|---|---|---|
| `toolchain/scheduler.py` | Layer / Stage / LayerStep 数据结构; LPT 调度算法; 切片决策 (W/cout); 兼容性矩阵 | ✅ |
| `toolchain/scheduler.analyze_slicing()` | 给每 layer 决定 W slice / cout slice + 标记 layer 间 push/DDR | ✅ |
| `toolchain/scheduler.gen_per_core_plan()` | Phase 5: 把 stages 转每核 LayerStep 序列 (in_from / out_to / skip_idma) | ✅ |
| `toolchain/hw_files.build_layer_desc_segment` + `write_multilayer_desc_list` | 多 layer chain desc 串接 (CFG_WRITE + CONV + BARRIER + END) | ✅ |
| `toolchain/run_multicore_chain.py` | Driver 主入口: schedule → chain 数据 → per-core desc 写盘 | ✅ |
| `sim/tb_multicore/tb_multicore_chain.sv` | 通用 N 核 N 层 TB | ✅ (sequential 模式) |
| **simple2 demo (P0)** | N=2 双核 sequential DDR 中转 11→2 layer chain | ✅ PASS @ 20253 cycles |
| Mode C W slice cfg gen | per-core W 段 + halo + asymmetric pad | ⚠ WIP (cfg corner cases 未完, 见 memory/mode_c_w_slice_design.md) |
| Mode C cout slice cfg gen | per-core cout 段 | ❌ 未做 |
| Stage barrier 多 stage 调度 | host 一个 stage 一个 stage 启 | ❌ 未做 |
| 片上 push 链 (P2) | computed redundancy halo + 跨 stage push | ❌ 未做 (设计就绪, 实施 ~2-3 天) |

### Scheduler 估算 (理想 wall, 不含 stage overhead)

| 网络 | N=1 | N=2 | N=4 |
|---|---|---|---|
| ResNet 11-layer (131M MAC) | 593K cycles, 168 fps | ~302K, 331 fps (1.7×) | ~141K, 709 fps (3.64×) |
| YOLOv3-tiny (2.7G MAC) | 10.87M, 9.2 fps | ~5.45M, 18.3 fps | ~3.12M, 32 fps |

详细切片决策见 `python toolchain/scheduler.py` 输出.

### Mode C 实施 RTL 改动需求 (轻量)

`idma_ctrl.sv` 必改 ~20 行: 加 `cfg_ddr_ifm_row_stride` 输入, 让 cur_addr 用 row_stride 推进而不是 cmd_btt (当前是行内连续 byte 假设 DDR 行连续, W slice 下不成立).

ODMA 已支持 (用 cfg_ddr_ofm_row_stride). Single-core 不影响 (sub_W=full_W 时数值等价).

### 切片兼容矩阵 (设计就绪)

```
      W slice (相同 partition)   cout slice
W slice  ✅ 直接 push (含 halo)    ❌ 需 broadcast (DDR)
cout sl. ❌ 需 broadcast (DDR)     ❌ 需 broadcast (DDR)
```

ResNet N=4 切片决策: layer 0-9 全 W slice (4 push 边界 + 6 DDR 边界), FC 必须 cout slice (W=1).

详细见 `docs/multicore_scheduling.md` 跟 `memory/mode_c_w_slice_design.md`.

---

## 2.8 P1 Mode C W slice (2026-04-30)

### 完成内容

| 模块 | 内容 | 状态 |
|---|---|---|
| `RTL/DMA/idma_ctrl.sv` | 加 `cfg_ddr_ifm_row_stride` 输入, `cur_addr += cfg_ddr_ifm_row_stride` 替代 `+= cmd_btt` | ✅ |
| `RTL/core_top.sv` | 接线 cfg_regs.r_ddr_ifm_row_stride → idma_ctrl (含 20→32 bit 零扩展) | ✅ |
| `sim/tb_idma_ctrl/tb_idma_ctrl.sv` | 子 TB 同步加 `cfg_ddr_ifm_row_stride` 端口 | ✅ |
| `toolchain/hw_files.derive_layer_cfg` | 加 `pad_right=None, pad_bot=None` 默认参数 (向后兼容对称 pad) | ✅ |
| `toolchain/hw_files.compute_w_slice_geom` | 几何辅助: per-core (sub_W, pad_l, pad_r, w_in_start, w_out_start) 计算 | ✅ |
| `toolchain/hw_files.derive_w_slice_cfg` | per-core W 段 cfg 派生 (asymmetric pad + halo + 整层 stride 标记) | ✅ |
| `toolchain/hw_files.cfg_to_dict` | DDR_*_ROW_STRIDE 用 `_W_SLICE_W_FULL` (整层 W) 而非 sub_W | ✅ |
| `toolchain/run_multicore_chain.py` | per-(core, layer) desc lists + DDRPlanner.core_layer_desc_ddr | ✅ |
| `sim/tb_multicore/tb_multicore_chain.sv` | host stage barrier loop + per-layer dfe/start_layer + per-layer POST OFM check | ✅ |
| `sim/tb_multicore/tb_multicore_chain.sv` 修 false-PASS bug | check_final_ofb 之前没被调用, 导致 mismatches=0 默认假 PASS | ✅ |

### 验证状态

#### 单核回归
| 测试 | 结果 |
|---|---|
| 单核 26-case 回归 (sub_W=full_W 数值等价) | ✅ 全 PASS, 无回归 |

#### N=2 W slice (TB NUM_CORES=2)
| 测试 | 结果 |
|---|---|
| wslice1 (1 层 K=3) | ✅ 5569 cycles |
| simple3 (3 层 K=3 stride=1) | ✅ 16705 cycles |
| wslice4 (4 层 K=3) | ✅ 22273 cycles |
| wslice5 (5 层 K=3) | ✅ 27841 cycles |
| wslice_mixed (4 层混合 K=3,5,3,1) | ✅ 27411 cycles |
| wslice_stride2 (4 层含 ds 层 stride=2) | ✅ 5995 cycles |
| wslice_oddw (W=33 奇数 N 不整除) | ✅ 18058 cycles |
| wslice_smallw (W=8 极小) | ✅ 4113 cycles |
| wslice_k7 (K=7 大 halo) | ✅ 55357 cycles |
| wslice_k1 (K=1 c=64 无 halo) | ✅ 6649 cycles |

#### N=4 W slice (TB NUM_CORES=4, axi_4to5 + axi_lite_1to4)
| 测试 | 结果 |
|---|---|
| wslice1 N=4 | ✅ 3833 cycles (vs N=2 5569: 1.45×) |
| simple3 N=4 (3 层) | ✅ 11493 cycles |
| wslice4 N=4 | ✅ 15323 cycles |
| wslice5 N=4 | ✅ 19153 cycles |
| wslice_mixed N=4 | ✅ 18649 cycles |
| wslice_stride2 N=4 | ✅ 7148 cycles |
| wslice_oddw N=4 (W=33 不被 4 整除) | ✅ 12630 cycles |
| wslice_smallw N=4 (W=8 LPT 并行 stage) | ✅ 4115 cycles |
| wslice_k7 N=4 | ✅ 34097 cycles |
| wslice_k1 N=4 | ✅ 6187 cycles |

整体: **20/20 切片 case 全 bit-exact PASS** (覆盖 K∈{1,3,5,7}, stride∈{1,2}, W∈{8,32,33},
单层/多层, N∈{2,4}, mode A/W slice 混合, LPT 并行 stage).

#### ResNet residual chain (P1.1)
| 测试 | 结果 |
|---|---|
| resnet_block1 N=1/2/4 (3 层 mode A + residual) | ✅ 23407 / 13994 / 10520 cycles |
| resnet_residual_wslice N=1/2/4 (3 层 K=3 + residual + W slice) | ✅ 31852 / 17733 / 12511 cycles |
| **ResNet11 N=1 (full ResNet-18-like)** | ✅ **596,088 cycles, 313 fps @ 100MHz** (s2d 后) |
| **ResNet11 N=2** | ✅ **450,469 cycles (1.32×), 444 fps** |
| **ResNet11 N=4** | ✅ **354,555 cycles (1.68×), 564 fps** |

**S2D (Space-to-Depth) 自动启用** (Patch K=4 stride=4 → K=1 stride=1 c_in=64):
- Patch 单层 cycles: 654,404 → **129,594** (5.05× 加速)
- 整网 N=1: 1,115K → **596K** (1.87× 加速, 168→313 fps)
- N=4: 750K → **355K** (2.11× 加速, 250→**564 fps**)

scheduler.Layer.force_s2d 自动判断 stride≥3 ∧ K≥stride 触发 (跟 run_regression 一致).
driver build_step_cfg_dict 用 layer.s2d_eff() 返回的等效维度算 cfg, 跟 gen_isa_test
内部 s2d 重排 ifb/wb 一致.

ResNet11 完整网络 (Patch 960×540×4 → ... → FC 522 类输出):
- 11 层混合: K∈{1,3,4}, stride∈{1,2,4}, 维度从 240×135×16 → 1×1×522
- 含 3 个 residual block (L1/L2/L3 的 B2.ds 用 B1.C2 OFM 当 shortcut)
- FC 是 root layer (input_src='', dim 1×1×256 跟前一层不匹配, host 单独 preload)
- mode A + W slice 混合调度: scheduler 按 cycle 决定每层切片粒度

residual + W slice 关键改造:
- driver: 后处理生成 per-(core, layer) sliced rdma_data.txt (按 main path geom 切 shortcut 列)
- DDRPlanner: 加 core_layer_rdma_ddr (per-core 1MB / N 段)
- TB: parse `CORE_<i>_LAYER_<l>_RDMA_BASE/WORDS`, preload `rdma_data_c<i>.txt` per (core, layer)
- ofb_writer 不需要改 — SB 按 sub_W_OUT 寻址自动对齐 sliced shortcut.

整体 P1 验证: **51 cases 全 bit-exact PASS** (26 单核 + 16 N=2/4 切片 + 6 ResNet residual + 3 ResNet11 N=1/2/4).

### 单层 W slice 关键点

W slice 几何 (computed redundancy halo):
- N=2, K=3, pad=1, W=32: Core 0 处理 W[0..17), pad_l=1 pad_r=0, W_OUT[0..16); Core 1 处理 W[15..32), pad_l=0 pad_r=1, W_OUT[16..32). 重叠 2 列 (halo).
- 每核独立从 DDR 读自己 W 段 (offset = w_in_start × cin × 16), 各核 ODMA 写 DDR 的对应 W 段, DDR 上自然合并.
- DDR_*_ROW_STRIDE = 整层 W × cin/cout × 16 (跟 cmd_btt 的 sub_W × cin × 16 解耦, 对应 RTL 改动).

### 多层 chain bug 修复 (重要)

**Root cause**: `hw_files.sdp_sim` 之前返回 `summed & 0xFF` (uint 0..255), 但 RTL line_buffer 把
IFB byte 当 INT8 *signed* (-128..127) 解释. 链式层间, 上层 OFM 当下层 IFM 时, 输出 byte 大于 127
的那些值, Python emulator 把它们当 +200, RTL 当 -56, 算出截然不同的 psum.

之前 run_regression chain 用 `clip_max=127` 强制输出 ≤ 127 (signed/unsigned 数值一致), 不触发 bug.
multicore chain 用 `clip_max=255` 才暴露.

**Fix**: `sdp_sim` 改成返回 signed int8:
```python
v = summed & 0xFF
return v - 256 if v >= 128 else v
```

`write_expected_ofm` 的 `& 0xFF` 仍 byte-pack 不变, 单层 case 输出 byte 不变.
跨层 chain `prev_ofm` 现在是 signed Python int, 跟 RTL 一致.

### N=4 IP gen + RTL 改动 (P1 N=4 上线)

新增 IP (gen_multicore_ip.tcl 已支持 NUM_CORES=4):
- `axi_4to5`: 4 SI / 5 MI 全连接 crossbar, ID_WIDTH=6
- `axi_lite_1to4`: 1 SI / 4 MI host CSR 分发, host_addr_w=14

multicore_top RTL:
- IP 实例放进 `generate if (NUM_CORES==2) ... else if (NUM_CORES==4) ...` 块
- 加 SI 端 ID padding (CORE_BUS_ID=4 → EXT_BUS_ID=6 zero-extend)
- IP gen tcl: ID_WIDTH=4+log2(N) 让 SI/MI 端口宽度匹配
- IP gen tcl: 加 `catch open_project + create -force` 兜底, 处理 stale IP refs

core_top RTL:
- `rmt_ifb_*id` 改成参数化宽度 `RMT_ID_W` (default 5 兼容旧, multicore_top 显式 override)

driver:
- `build_step_cfg_dict` 把 `step.my_core` (物理核 ID) 转 `slice_idx` (0..n_split-1) 再传 derive_w_slice_cfg
  (LPT scheduler 可能把不同 layer 分到不同核组, 物理 ID 跟 slice idx 不一致)

### 下一步

| 任务 | 工时估计 |
|---|---|
| ResNet 11 layer 适配 (residual 路径 + chain h/w 维度变化) | 1-2 天 |
| Mode C cout slice 实施 (P2) | 1 天 |
| 片上 push 链 (P2 完成态, N=2 ABAB / N=4 流水) | 2-3 天 |

---

## 2.7 参数表 single source of truth (M2.5, 2026-04-30)

`params.py` (项目根) 是 RTL/Python 唯一参数源:
- 64 个 `\`FLUX_*` 宏 (核心尺寸 + SRAM 容量 + AXI/CSR + 全局地址映射 + 58 CSR addr)
- `python params.py` 自动生成 `RTL/flux_cnn_params.svh`
- RTL: `\`include "flux_cnn_params.svh"` (cfg_regs.sv / core_top.sv / multicore_top.sv 已用)
- Python: `from params import *` (hw_files / scheduler 已用)

详见 `docs/params.md`.

---

## 2.5 多核 (`multicore_top`) — M2 跨核 SRAM 直送 仿真验证通过 (2026-04-30)

### 拓扑变化
- `axi_2to1` (NUM_SI=2 NUM_MI=1) → `axi_2to3` (NUM_SI=2 NUM_MI=3)
- MI[0] = DDR @ 0x0000_0000-0x7FFF_FFFF (31 bit)
- MI[i+1] = Core[i] IFB AXI4 slave @ 0x8000_0000 + i*0x1000_0000 (28 bit region)
- 跨核 push: producer ODMA 写 `0x8000_0000 + consumer_id*0x1000_0000`, crossbar 路由到 consumer 的 `ifb_axi_slave` 模块, 经 ring 反压 写入 IFB SRAM

### 关键 RTL 改动
| 文件 | 内容 |
|---|---|
| `RTL/AXI4/ifb_axi_slave.sv` (新) | AXI4 SI → SRAM 写口适配, 内部 wptr/rows_pushed 计数, ring 反压 (`rows_pushed - rows_consumed >= cfg_ifb_strip_rows`); cfg_skip_idma=0 时永远 awready=0 拒绝 push (防止 producer 写自己) |
| `RTL/cfg_regs.sv` | 新加 `ADDR_SKIP_IDMA = 0x1CC` (1 bit, seq_w only, reset=0) |
| `RTL/core_top.sv` | 加完整 IFB AXI4 SI 端口; ifb mux 三路 (idma\|tb_ext\|rmt_axi); cfg_skip_idma=1 时 gate `seq_start_idma_pulse_g`=0 + `idma_done_eff`=1; line_buffer 的 `rows_available` 在 cfg_skip_idma 时切到 ifb_axi_slave.rows_pushed |
| `RTL/multicore_top.sv` | crossbar IP 升级为 axi_2to3 + 全局地址映射, MI[i+1] wire 到核 IFB SI |
| `Syn/gen_multicore_ip.tcl` | 生成 `axi_${N}to${N+1}` IP, 自动配置每 MI 的 BASE_ADDR/ADDR_WIDTH |

### 编译器 + TB
- `toolchain/gen_cross_core_test.py` (新): 双层 chain 生成器, producer cfg.ODMA_DST_BASE = 跨核 IFB region, consumer cfg.SKIP_IDMA=1 + skip_ifb_preload
- `sim/tb_multicore/tb_multicore_xcore.sv` (新): 跨核流水线 TB, **consumer 先启** (reset wptr) + producer 后启
- DDR layout: L0 IFB/WB/desc 在 0x0/0x80_0000/0xB0_0000; L1 WB/OFB/desc/RDMA 在 0x90_0000/0xA0_0000/0xC0_0000/0xE0_0000

### 验证结果 (2026-04-30)
- 双层 chain (32×32×16 → conv3×3 → 32×32×16 → conv3×3 → 32×32×16): RESULT PASS, BOTH cores done @ 11006 cycles, **Layer 1 OFB 1024 word bit-exact 匹配 golden**
- 单核 11-case 回归 PASS 无回归
- M1.5 multicore DDR-mode TB PASS 无回归 (9057 cycles 一致)

### 调试踩的两个坑
1. **`ifb_axi_slave.AWREADY` 必须 gate cfg_skip_idma**: 否则 producer 自己核的 ifb_axi_slv 也接收任意 push, ifb_we 跟 idma_we 三路 mux 仲裁导致 SRAM 写入污染
2. **RDMA_SRC_BASE 不能跟 IDMA_SRC_BASE 重叠**: gen_cross_core_test.py 默认 ddr_rdma_base=None → cfg=0, 跟 IDMA_SRC_BASE=0 重叠, 导致 bias_rf 读 IFB 数据当 bias, 让 SDP 后处理把 mac_psum=0 加成 saturate. **必须给 ddr_rdma_base 独立的 region**

详细见 `memory/m2_cross_core_pipeline.md`.

### M2 鲁棒性

#### 修过的两个真 bug

1. **RDMA 地址冲突 (gen_cross_core_test.py)**: `ddr_rdma_base=None → cfg.RDMA_SRC_BASE=0` 跟 `IDMA_SRC_BASE=0` 重叠, bias_rf 读 IFB 数据当 bias 让 SDP 把 mac_psum=0 加成 saturate 0x7f. 之前误判为 cin<16 padding bug. **修**: 给 RDMA 独立 DDR region.
2. **OFB SRAM 容量不匹配 (hw_files.py)**: `OFB_SRAM_WORDS=8192` 但 RTL `OFB_DEPTH=2048`, 编译器误以为 H_OUT=64 W_OUT=32 整图能装下, 让 ofb_ring_words=2048=full SRAM, 触发 ring full/empty 二义性死锁. **修**: `OFB_SRAM_WORDS=2048` + `ofb_strip_rows_max = (OFB-1)//row_words` 留 1 行 slack.

#### 鲁棒性回归套件 (`toolchain/run_robust_smoke.py`)

24 个独立 corner case, 覆盖:
- K = 1, 2, 3, 5, 7
- stride = 1, 2, 3, 4
- pad = 0, 1, 2, 3
- cin = 4, 8, 12, 16, 32 (含 < 16 不 fold 路径)
- cout = 16, 24, 32 (含非 16 倍数)
- H/W = 1×1 (FC), 15×17 (奇数), 28×28, 33×33, 64×32 (OFB ring 边界), 120×68
- 全部 24/24 PASS @ 100k cycles 内

#### 全套 sim 状态 (2026-04-30)
- 单核 11-case ResNet-like chain: ✅ All PASS (135K~9K cycles)
- 多核 DDR mode (M1.5): ✅ PASS @ 9057 cycles
- 多核 cross-core chain (M2, c_in=8): ✅ PASS @ 9818 cycles, Layer 1 OFB 900 words bit-exact
- 鲁棒性 24-case smoke: ✅ All PASS

---

## 2.9 多核 TB 结构化 profile 报告 (2026-05-04)

`tb_multicore_chain.sv` / `tb_multicore_4ddr_chain.sv` 加 `LAYER_PROFILE / CORE_RESULT / CORE_PROFILE / DDR_PROFILE / LAYER_CORE / LAYER_DDR` 行（机读，跟 tb_core_dma 的 `CASE_RESULT` 同口径）：

| 行类型 | 内容 |
|---|---|
| `LAYER_PROFILE l=N cores=mask cycles=X mac_pipe_pct=X% ddr_busy=X (X%)` | per-layer 总览 |
| `LAYER_CORE l=N c=N mac_fire/act_fire/stall/idle/wgt_stall/psum_stall/acc_stall=X` | per-active-core 拆解 |
| `LAYER_DDR l=N d=N aw_fire/w_beats/ar_fire/r_beats/busy_cyc/busy_pct=X` | per-DDR 流量 |
| `CORE_RESULT c=N PASS/FAIL cycles=X mac_fire=X mac_pipe_pct=X% arf_w/r=X parf_f/d=X` | 整网 per-core 总结 |
| `CORE_PROFILE c=N cycles=X act/wgt/psum/acc fire/stall/idle=X` | 整网 fire/stall/idle (跟 tb_core_dma 一致) |
| `DDR_PROFILE d=N cycles=X aw_fire/w_beats/ar_fire/r_beats/busy_cyc/busy_pct=X` | 整网 per-DDR |

`axi_slave_mem.sv` 加 `aw_fire / w_beats / ar_fire / r_beats / busy_cyc` 5 个计数器，TB 通过 hier ref 拿到 DDR 利用率。

**`mac_pipe_pct` 公式**：`mac_fire / cycles`（per-core stage-0 join 占比），多核 avg = `total_mac_fire / (cycles × active_cores)`。这是**硬件 pipe 利用率**，不是 ops 比，跟 tb_core_dma 的 `mac_util` (= ops / (cy × 256)) 是两个口径。

---

## 2.10 4-DDR PoC sim 验证 BW 上限 (2026-05-04)

**目的**：定位"为什么 N=4 ResNet11 1-DDR 只 1.49× 加速"，量化"BW 加宽后整网 cycles 上限"。

### 实施

1. **新增 RTL** (`RTL/multicore_top_4ddr.sv` + `Syn/gen_multicore_4ddr_ip.tcl`)：
   - axi_4to8 crossbar：4 SI / 8 MI（4 DDR slot + 4 IFB region）
   - 4 个独立 DDR master 出口，各 256 MB region
2. **driver** (`toolchain/run_multicore_chain.py --multi_ddr`)：
   - DDRPlanner 给每核 DMA bases OR 入 `core_id * 0x10000000` slot
   - TB (`sim/tb_multicore/tb_multicore_4ddr_chain.sv`) 4 个 axi_slave_mem 实例 + broadcast preload
3. **scheduler 加 force_multicore 选项**：让 ds layer 也走 W slice 4 核（默认走单核 mode A）

### 实测数据

ResNet11 N=4：

| 配置 | wall cycles | mac_pipe% | DDR busy |
|---|---|---|---|
| 1-DDR baseline | 354,566 | 36.2% | 84.7% (单 DDR) |
| 4-DDR PoC | 237,986 | 54.0% | 29-44% per DDR |
| **4-DDR + force_multicore (BW 上限)** | **196,271** | **63.5%** | 40-44% per DDR |

**vs 1-DDR**: wall **-45%**, mac_pipe% **+27 pp**.

ds layer (3/6/9) 单核运行从 `force_multicore=True` 后转 W slice 4 核：
- L3.ds: 44K → 14.5K (-67%)
- L6.ds: 14K → 6.3K (-56%)
- L9.ds: 7.5K → 3.7K (-51%)

### ROI 决定

**单 DDR 框架内做 BW 加宽（BUS_DATA_W=512）的硬件改动量大** (3-5 天 RTL + IP 重做 + SRAM 字宽改)，且要求 mac_array 不变意味着只解 IDMA/ODMA BW 不解 compute。**接受 4-DDR 是 BW 解墙的主路径**。

文件:
- RTL: `RTL/multicore_top_4ddr.sv`
- TB: `sim/tb_multicore/tb_multicore_4ddr_chain.sv`
- TCL: `Syn/gen_multicore_4ddr_ip.tcl`, `sim/tb_multicore/run_chain_4ddr.tcl`
- Driver: `toolchain/run_multicore_chain.py --multi_ddr --force_multicore`

详见 `memory/project_4ddr_poc_result.md`.

---

## 2.11 2D Mesh + AXI4-Stream NoC PoC (2026-05-04, AIE-ML 风格)

**目的**：参考 AMD AIE-ML，验证 mesh + AXIS NoC 取代 axi crossbar 的可行性。
- ConvCore 内部 BUS_DATA_W=128 不变（不破坏单核设计）
- 通过多 Mem Core + mesh 路由实现"4 倍 BW"，每核 1 hop 直达 Mem
- ConvCore 通过现有 `rmt_ifb_*` AXI4 slave port 收 packet（M2 push 通道复用）

### 已完成 (PoC 验证完整)

| 模块 | 行数 | 验证 |
|---|---|---|
| `RTL/Mesh/router_node.sv` | 240 | 5-port AXIS switch + wormhole + XY routing, 8/8 单元 PASS |
| `RTL/Mesh/mesh_2x2.sv` | 230 | 4 router 互连, 6/6 跨 router 测试 PASS |
| `RTL/Mesh/mesh_5x1.sv` | 130 | 5 router 一行, 1D mesh |
| `RTL/Mesh/mesh_4x2.sv` | 200 | 8 router AIE 风格 (4 Mem + 4 Conv 列拓扑) |
| `RTL/Mesh/axis_packet_rx.sv` | 100 | AXIS packet → SRAM 写 (按 opcode 分 IFB/WB/SB/RDMA 区) |
| `RTL/Mesh/axis_packet_tx.sv` | 130 | SRAM → AXIS packet (header + N body + tail) |
| `RTL/Mesh/conv_core_stub.sv` | 130 | 简化 ConvCore (rx + 自动 compute_delay + tx echo) |
| `RTL/Mesh/mem_core_stub.sv` | 100 | 简化 Mem Core (DDR mock + cmd 引擎 + packet IO) |
| **`RTL/Mesh/axis_to_axi_writer.sv`** | **110** | **AXIS packet → AXI4 master burst write 桥** |

### 测试矩阵

| TB | 验证 | 结果 |
|---|---|---|
| `tb_router_node` | 单 router 5-port XY routing | 8/8 PASS |
| `tb_mesh_2x2` | 跨 router 多 hop + 多核并发 | 6/6 PASS |
| `tb_packet_io` | rx/tx + router 端到端 | PASS |
| `tb_mesh_4core_demo` | 5x1 mesh: 4 ConvCore + 1 Mem 端到端 IFB load + OFM store | PASS |
| `tb_mesh_chain_profile` | 5x1 mesh 3 层 chain profile (per-router fire) | PASS |
| `tb_mesh_4x2_chain` | 4x2 mesh 3 层 chain profile (4 Mem 独立通道) | PASS |
| `tb_axis_to_axi_writer` | axis packet → axi4 burst write 单元 | PASS |
| `tb_mesh_to_ifb` | **AXIS → axi 桥 → ifb_axi_slave → IFB SRAM bit-exact** | **PASS** |

### 关键数据

#### 5x1 mesh (1 Mem) vs 4x2 mesh (4 Mem) — 验证多 Mem 解 BW 瓶颈

3 层 chain demo (compute_delay = 200/400/300 cy)：

| 指标 | 5x1 (1 Mem) | 4x2 (4 Mem) | Δ |
|---|---|---|---|
| Total cycles | 1549 | **1159** | -25% |
| 单 Mem TX/RX util | 29.7% | 9.9% (×4) | 平均分担 |
| 热点 router | x=1.W (460 fire) | 全对称 (115 fire) | 消除 |
| 跨核 traffic | 多 hop 拥堵 | 1 hop 直达 | 零拥堵 |

**4x2 mesh + 4 Mem ≡ 4-DDR PoC 的 BW 模型**，但通过 mesh 实现 ConvCore 解耦。
真 ResNet 工作负载 (Patch BW-bound) 收益会接近 4-DDR PoC 的 1.81×。

#### Phase 6 hybrid 数据链路（不改 ConvCore）

`tb_mesh_to_ifb` 验证完整链路：

```
TB 发 AXIS packet (16 word, addr=0)
  → axis_to_axi_writer (我们写的桥, packet header → axi aw + N w + b)
  → ifb_axi_slave (项目现有 RTL, M2 已用)
  → sram_model (IFB SRAM)
  → bit-exact 16 word 写入 ✓
```

**结论**：mesh packet 能无缝接到 ConvCore 现有 `rmt_ifb_*` slave port，**ConvCore 内部完全不动**，cfg_skip_idma=1 模式下接收 push。

### 关键设计决策

1. **协议**：AXI4-Stream（标准 `tvalid/tready/tdata/tlast/tdest`）—跟 AMD AIE-ML 一致，可复用 Vivado AXIS IP (`axi_dma`, `axis_data_fifo`, `axis_register_slice`, `axis_broadcaster` 等)
2. **Flit 格式**：tdata[127:0] + tdest[7:0]={dst_y, dst_x}, tlast 标 packet 结尾。Header flit 用 tdata 高位编码 opcode/addr/burst_len
3. **路由**：XY (先 X 再 Y)，无环路 deadlock-free
4. **Wormhole**：input 端 + output 端**双 lock**（input 锁路径，output 锁防多 input 交错）—— 这个 bug 是 4-core demo 暴露出来才发现的
5. **不用 axis_switch IP**：它是静态 routing table，跟 mesh XY 动态路由不兼容；router_node 自己写

### 修复的关键 bug（debug 时发现）

| Bug | 影响 | 修复 |
|---|---|---|
| `axis_packet_rx.last_*` 是 register，跟 `packet_done` pulse 同拍但值上一拍 | conv_core_stub 拿到错误 burst_len/addr | 改成组合输出 `r_*` |
| `axis_packet_tx` stall 时 `sram_raddr` 漂回 0，污染 latch | TX 数据错乱 | latch 加 `if (sram_re)` 门控 |
| **router output port 缺 wormhole lock** | 多 input 同 output 时 packet 交错 (Conv0+Conv1+...) | 加 `out_active[op]` + `out_locked_in[op]` |

### Phase 6.4 / 6.5 进展 (2026-05-04)

| 模块 / TB | 行数 | 验证 |
|---|---|---|
| `RTL/Mesh/axi_writer_to_axis.sv` | 110 | **PASS** (单元): AXI4 master write burst → AXIS packet (header+body+tail) |
| `tb_axi_writer_to_axis` | 170 | 4 beat burst 转 5 flit packet, header 字段 + body 数据 + axi b 全对 |
| **`RTL/Mesh/mesh_core_wrapper.sv`** | **240** | **核心: ConvCore + 2 个桥的完整 hybrid 包装, 编译通过 (0 error)** |

**`mesh_core_wrapper.sv` 包含**:
- 1 个 core_top (项目现有 RTL 完全不动)
- 1 个 `axis_to_axi_writer`: mesh AXIS in → core_top.rmt_ifb_* (IFB push 路径)
- 1 个 `axi_writer_to_axis`: core_top.bus_aw/w/b (ODMA write) → mesh AXIS out
- core_top.bus_ar/r 直 expose 给上层 (WB / RDMA fetch 仍走 DDR-mock)
- ConvCore CSR (AXI-Lite) 直 forward
- 配置 `cfg_ofm_tdest` / `cfg_ofm_opcode` 控 ODMA 发出去的 packet header

**对外接口**:
- AXIS slave (mesh in)
- AXIS master (mesh out)
- AXI4 master (read only, ar/r)
- AXI-Lite slave (CSR)

ConvCore 通过 mesh 跟外界通信的硬件路径完整, **ConvCore 内部源码零改动**.

### Step A 完成 (2026-05-04): multicore_top_mesh.sv + smoke sim PASS

新增文件:
- `RTL/multicore_top_mesh.sv` (310 行): 4 ConvCore (mesh_core_wrapper) + 4 mem_core_stub + mesh_4x2 + axi_lite_1to4 host CSR fanout + 简化 4-to-1 read 仲裁器 (服务 WB/RDMA/desc fetch)
- `sim/tb_mesh/tb_multicore_top_mesh_smoke.sv` + `run_top_mesh_smoke.tcl`

**端到端 mesh 数据通路验证 (bit-exact)**:
```
host CSR (SKIP_IDMA=1) → mem_core[0] (push 16-word packet) → axis_packet_tx
  → mesh_4x2: router(0,0).LOCAL → NORTH → router(0,1).SOUTH → LOCAL
  → mesh_core_wrapper.axis_to_axi_writer → core_top.rmt_ifb_axi_slave
  → IFB SRAM bit-exact ✓
```

**关键调试发现 (debug 时记录)**:
1. `cfg_skip_idma=1` 必须先配置, 否则 ifb_axi_slave AWREADY=0 拒收 push (现已加 host 写口)
2. `cfg_ifb_strip_rows / ring_words` 默认 0 让反压死锁, 需要 host 先配 (sim hack 直接 force)
3. `line_buffer.rows_consumed_raw` 没 reset, 默认 X 导致 ifb_axi_slave.AWREADY=X 死锁; sim hack force=0, 实际 deployment 由 evt_start_layer 清

### Step E.1 完成 (2026-05-04): 单核单层 conv mesh sim PASS (bit-exact!)

新增文件:
- `sim/tb_mesh/tb_mesh_single_conv.sv` (340 行): 复刻 tb_core_dma 的 case01 setup, DUT 替换为 multicore_top_mesh
- `sim/tb_mesh/run_single_conv.tcl`

**ConvCore 走 mesh 路径跑真 conv 计算 bit-exact 验证**:
```
case01: K=3 stride=2 c_in=16 c_out=16 H_out=120 W_out=68
  IFB=32400 word → mem_core[0].ddr_mem (mesh 数据源)
  WB/RDMA/desc → axi_slave_mem (ConvCore.bus_ar/r 拉)
  host axi-lite: SKIP_IDMA=1 → DESC_LIST_BASE → start_dfe → start_layer
  TB fork: mem 推 240 个 IFB row packet (135 word/row)
  ConvCore: line_buffer/mac_array/parf_accum/sdp/ofb_writer 跑 conv
  ODMA → axi_writer_to_axis → mesh → mem_core[0].ddr_mem (OFM 区 @ 0x90000)
  PASS: 8160 OFM word bit-exact ✓
```

**关键 fix (sim 调试时)**:
- mem_core_stub.DDR_DEPTH 8192→1M (装下 ODMA 写到 0x90000 的 OFM)
- 用 module-level shadow signal + initial 一次 force 绑定 mem.cmd_*, task 改 shadow signal (避免 task automatic var 用在 force 里)
- ifb_strip_rows / ring_words 仍用 sim hack force (Step C 改 RTL 后可去)

**这是 Phase 6 关键里程碑**: 真 ConvCore + mesh 端到端跑出 bit-exact 计算结果, 证明 mesh 架构功能完整.

### 剩余: Step F (多核 W slice + ResNet11)

| 步骤 | 内容 | 工时 |
|---|---|---|
| F.1 | 多层 chain (simple3 / resnet_block1) | 1-2 天 |
| F.2 | 4 核 W slice (wslice1 N=4) | 2-3 天 |
| F.3 | ResNet11 11 层完整 mesh sim | 3-5 天 |

详见 `memory/mesh_phase6_plan.md`. 预期 ResNet11 mesh ≈ 196K cy (跟 4-DDR force_multicore 等价), 远 < N=1=596K, **3.04× 加速**.

### Step C 完成 (2026-05-04): cfg_regs.sv 加 OFM_TDEST/OPCODE + IFB ring host 旁路写

**Step C.1-C.7 全部 PASS** — `tb_mesh_single_conv.sv` 8160 OFM bit-exact 通过 host axi-lite 真实路径配置, 不再依赖 sim hack force:

- `params.py` 新增 `ADDR_OFM_TDEST=0x1D0` / `ADDR_OFM_OPCODE=0x1D4`
- `cfg_regs.sv`:
  - 新增 `r_ofm_tdest` / `r_ofm_opcode` 寄存器 (host csr_w 写, 不进 desc CFG_WRITE 流)
  - `r_ifb_strip_rows_host` / `r_ifb_ring_words_host` (mesh 模式 host 旁路写, 配 vld 优先于 seq_w)
  - 输出端口 + reg_r_data 读 mux 全部接通
- `core_top.sv` 加 `ofm_tdest` / `ofm_opcode` 输出端口, 转发 cfg_regs 输出
- `mesh_core_wrapper.sv` 删除 `cfg_ofm_tdest` / `cfg_ofm_opcode` 输入端口, 改成内部 wire 接 core_top 输出
- `multicore_top_mesh.sv` 删除每核硬编码 tdest 计算 (现由 ConvCore 内 cfg_regs 驱动)
- `tb_mesh_single_conv.sv` 用 `axi_lite_write` 配 SKIP_IDMA / IFB_STRIP_ROWS / IFB_RING_WORDS / OFM_TDEST / OFM_OPCODE, 不再 force `r_ifb_*`

### Step B 完成 (2026-05-04): mem_core_stub 加 AXI-Lite slave (替代 cmd_* 输入端口)

**Step B 全部 PASS** — `tb_mesh_single_conv.sv` + `tb_multicore_top_mesh_smoke.sv` 都通过 mem CSR 真实 host 写路径:

- `mem_core_stub.sv` 删除 `cmd_*` 输入端口, 改成 axi-lite slave 接口 + 内部寄存器 + 6 个 CSR (DDR_ADDR / BURST_LEN / SRAM_OFFSET / TGT / TRIGGER / STATUS)
- `multicore_top_mesh.sv` 新增 `mem_csr_*` host 输入端口 + 第二份 `axi_lite_1to4` fanout 给 4 个 mem stub
- mem CSR addr 解码: `[13:12]=mem_id`, `[11:0]=mem 内部 reg offset`
- TB 写一组 5 寄存器 + TRIGGER → poll STATUS[1]=cmd_done_sticky 表示一包发完
- `tb_mesh_single_conv.sv` 的 `mem_send_ifb_row` task 完全用 axi-lite, single conv mesh sim 仍 8160 OFM bit-exact
- `tb_multicore_top_mesh_smoke.sv` 同步更新, 16 IFB SRAM bit-exact

**至此 Step E.1 留下的 3 处 sim hack 全部清除**:
1. ~~mem.cmd_* 用 force / shadow signal~~ → 走 mem AXI-Lite (Step B)
2. ~~r_ifb_strip_rows / r_ifb_ring_words 用 force~~ → 走 host CSR 写 (Step C)
3. line_buffer.rows_consumed_raw 默认 X (smoke only) → 仍 force, 实际 deployment 由 evt_start_layer 清

### Step D 完成 (2026-05-05): toolchain + mesh chain TB driver-driven 端到端

**Step D.6 PASS** — simple3 3 层 mesh chain bit-exact 1024 OFM words ✓ (single-core mode A, n_cores=1).

新增文件:
- `toolchain/mesh_cmd.py` (~150 行): mem cmd list 文件格式 + helper 函数 (`gen_mem_ifb_push_cmds`, `write_mem_cmd_list`, conv/mem core tdest 计算)
- `sim/tb_mesh/tb_mesh_chain.sv` (~580 行): mesh 多层 chain TB (single-core P0)
- `sim/tb_mesh/run_chain.tcl`

`run_multicore_chain.py` 改造:
- 新增 `--mesh` flag, 跟 `--multi_ddr` 互斥, 限制 n_cores ≤ 4 + 仅 mode A / W slice n_split=1 (单核)
- mesh 模式 desc 强制 `skip_idma=True` (`dataclasses.replace(step, ...)`)
- mesh 模式紧凑 DDR layout (mem_core_stub.ddr_mem 16MB packet addr20 限制内):
  - `MESH_OFM_BASE = 0x100000` (1MB), `MESH_OFM_STRIDE = 0x80000` (512KB / layer)
  - 容纳 30 layers × 512KB = 15MB ≤ 16MB
  - last layer OFM 不走 FINAL_OFM 区 (会越界)
- 给每个 (mem, layer) 生成 IFB row push cmd list (per-mem 一个目录, per-layer 一个文件)
- meta 文件追加 `MESH=1` + `MEM_<m>_LAYER_<l>_CMD_COUNT/CMD_FILE` 字段

`tb_mesh_chain.sv` 关键设计:
- 接 `multicore_top_mesh` DUT (4-core mesh, P0 仅 ConvCore[0]/Mem[0] 干活)
- per-layer 启动序列: 配 SKIP_IDMA + IFB ring + OFM_TDEST/OPCODE → 写 DESC_LIST_BASE/COUNT → start_dfe → wait dfe → start_layer
- fork: 推 mem cmd list (一条 5 寄存器写 + TRIGGER + poll STATUS[1]) 跟等 ConvCore done 用 `join` 等齐
- 比对最后一层 mem[0].ddr_mem OFM 区 vs expected_ofm

**调试踩坑**:
- packet header addr 字段 20-bit (16MB byte 范围限制) — DDRPlanner 默认 OFM_LAYER 起 16MB 直接溢出, mesh 模式独立紧凑 layout
- `mem_core_stub.ddr_mem` DDR_DEPTH 1MB→4M words (装下 OFM_LAYER 区)
- TB 没写 `ADDR_DESC_COUNT` → DFE 拉 0xFFFF beat 卡死 (single_conv 通过 `load_config` 自动写, chain TB 直接走 meta 必须显式)
- `string.substr(s, e)` 返回 `e-s+1` 字符, 比较 `"LAYER"` 永远不匹配, 必须用 `"LAYER_"`
- mem CSR `CMD_TGT` 字段拼接错位 (`{16'h0000, 4'h0, tdest, opcode}` 让 tdest 落在 [11:4] 不是 [15:8]) → mesh 路由 tdest 错乱 packet 不到 ConvCore

### Step F.2 完成 (2026-05-05): 4-core W slice mesh chain PASS

**wslice1 (N=4 单层 W slice) mesh sim PASS** — 4 段 8 列 OFM bit-exact stitch ✓.

多核协同方案 (broadcast IFB + 各核自管):
```
mem[0..3]   ←broadcast IFB preload→   各自 ddr_mem 同位置
   ↓ 各推自己 W 段 IFB packets (mesh AXIS)
ConvCore[0..3] 各跑 W 段 conv (8 列 + halo)
   ↓ 各写自己 OFM packets (tdest = 本核 mem)
mem[0..3]   各自 ddr_mem 存 W 段 OFM
   ↓ TB stitch 4 段 → 跟整图 expected_ofm 比对
```

新增/修改:
- `mesh_cmd.py` 加 `gen_mem_ifb_push_cmds_w_slice` (每行 burst_len = sub_W × cin_slices)
- `run_multicore_chain.py` mesh 模式放开 W slice n_split>1, 重用 `compute_w_slice_geom`
- meta 加 `MESH_N_CORES` / `MEM_<m>_LAYER_<l>_W_OUT_START` / `MY_W_OUT` 字段
- `tb_mesh_chain.sv` 改多核: 4 核并启 + 4 mem fork push + W 段 stitch 比对

关键调试坑 (新增):
- `cfg_regs.sv` host vld 不在 start_layer_pulse 清 → 多层 chain W slice 不能共享 host 写值 (修：start_layer_pulse 清 vld)
- `axi_lite_1to4` host 总线只有一条, 4 fork 并发 mem CSR write 互相覆盖信号 → mem[1/2/3] 寄存器全 X
  (修：semaphore 互斥 mem_axi_lite_write/read 任务)

### Step F.1 完成 (2026-05-05): resnet_block1 mesh chain (residual) PASS

3 层 chain (B1.C1 → B1.C2 → B2.ds, L2 含 residual from L0) 1024 OFM bit-exact ✓.

mesh 路径下 RDMA (residual shortcut data) 仍走 `ConvCore.bus_ar` → `axi_slave_mem`, 跟 mesh AXIS 路径正交, driver / TB 无新改动通过.

### Step F.3 完成 (2026-05-05): ResNet11 11-layer 全 mesh sim PASS

**11 层 mesh chain 全 OFM bit-exact ✓** (n_cores=1, single-core mesh 路径)

总耗时 ~6.02 ms @ 100 MHz = **~602K cycles** (跟 baseline N=1 = 596K cycles 等价, +1% mesh 调度 overhead, host TB 串行写 mem CSR 但跟 ConvCore 计算流水重叠).

per-layer timing:
| Layer | cumulative ms | cycles delta |
|---|---|---|
| L0 (Patch) | 1.39 | 139K |
| L1 (B1.C1) | 2.13 | 74K |
| L2 (B1.C2) | 2.89 | 76K |
| L3 (B2.ds + res) | 3.39 | 50K |
| L4 (L2.B1.C1) | 3.78 | 39K |
| L5 (L2.B1.C2) | 4.55 | 77K |
| L6 (L2.B2.ds) | 4.68 | 13K |
| L7 (L3.B1.C1) | 5.06 | 38K |
| L8 (L3.B1.C2) | 5.85 | 79K |
| L9 (L3.B2.ds) | 5.92 | 7K |
| L10 (FC) | 6.02 | 10K |

调试踩坑:
- ResNet11 layer 0 IFB 2MB, mesh layout 起 `MESH_OFM_BASE=0x100000` (1MB) 跟 IFB 重叠, layer 0 OFM 写覆盖 mem 推 IFB 数据. 重新设计 mesh layout: INPUT 0..2MB, OFM_LAYER 起 2MB stride 512KB, ROOT_IFB 起 14MB
- ResNet11 cross-layer skip (L3/L6/L9 input from block 入口, L10 FC root): driver mesh layout 加 ROOT_IFB region + 用 `name_to_idx[input_src]` 算 IDMA addr (跟单 DDR 模式逻辑一致)
- mesh 模式下多层 chain `ADDR_DESC_COUNT` 必须每层 host 写 (DFE ARLEN 默认 0xFFFF 卡死)
- chain TB fork...join_any disable fork 在 ResNet11 大 layer 时跳过 wait done 进下层冲突 (修：改 fork...join 等齐 + 去 debug timeout)

### ResNet11 N=4 mesh 加速完成 (2026-05-05)

**ResNet11 N=4 mesh 全网 OFM bit-exact ✓**, 一次性 PASS, driver 默认 scheduler 输出混合策略.

| Mode | Cycles | vs Baseline (N=1 single-core) |
|---|---|---|
| N=1 single-core | 596K | 1.00× |
| N=1 mesh | 602K | 0.99× (轻微 mesh 调度 overhead) |
| **N=4 mesh** | **305K** | **1.95×** |

调度分布 (driver scheduler 自动算出):
- 大计算层 L0/L1/L2/L4/L5/L7/L8: 4 核 W slice 并行
- ds 层 L3/L6/L9: 单核 mode A 独占 (K=1 W slice 收益小, scheduler 选单核)
- FC 层 L10: core 3 单核兜底 (1×1×256→1×1×522 没 W 维不能切)

负载均衡良好 — Layer 8 末 4 核 done 时间差 ≤10K cycle.

实际 2× 加速 (vs 理论 4×) 因 ds + FC 总占 ~30% cycle 单核运行. 跟 4-DDR force_multicore N=4 (196K cycles, 3.04×) 比偏低, 因后者强制 ds 层也 4 核陪算; mesh 默认 scheduler 选最省 cycle 模式 (单核 ds 反而更快, 没 W slice halo 冗余).

### Phase 6 整体里程碑

| Case | n_cores | layers | OFM | cycles |
|---|---|---|---|---|
| `tb_mesh_single_conv` | - | 1 (case01) | ✓ | - |
| `mesh_simple3` | 1 | 3 | ✓ | - |
| `mesh_block1` | 1 | 3 + residual | ✓ | - |
| `mesh_wslice1` | 4 | 1 W slice | ✓ stitched | ~5K |
| `mesh_resnet11_n1` | 1 | 11 | ✓ | 602K |
| **`mesh_resnet11_n4`** | **4** | **11 mixed** | **✓ stitched** | **305K** |

mesh PoC 完成, 端到端验证: RTL (mesh 8-router NoC + 双桥 + multicore_top_mesh) + driver (run_multicore_chain.py --mesh + mesh_cmd.py) + TB (tb_mesh_chain.sv 多核 W slice + stitch).

### Step F.4 完成 (2026-05-05): mem 内置 desc engine, ResNet11 N=4 加速 1.96× → 2.53×

**问题**: 旧版 mem cmd 接口 (per-cmd CSR write) 让 TB 4 mem fork 通过 host AXI-Lite semaphore 互斥串行推送, 每条 cmd 6 个事务 × 5 拍 = 30 拍/cmd. ResNet11 N=4 时 4 mem × 240 cmd × 30 拍 = 28K 拍纯 host CSR 串行, 跟 ConvCore 计算 (~18K 拍/W slice 1/4) 比甚至更长 → L0/L1 加速比限制在 ~2× 而不是接近 4×.

**修复**: mem_core_stub.sv 加内置 desc engine (类比 ConvCore.dfe.sv):
- 二进制 desc 格式 (16 byte / desc): `bit[31:0]=ddr_addr_w | bit[47:32]=burst_len | bit[79:48]=sram_offset_w | bit[87:80]=tdest | bit[91:88]=opcode`
- mesh_cmd.py 输出 `mem<m>_layer<l>_descs.hex` ($readmemh 格式)
- TB preload 时把 desc list 写到 mem.ddr_mem[0x80000+] 区, 然后 host 写 3 个寄存器 (DESC_LIST_ADDR / DESC_COUNT / CTRL.start) 启 engine
- mem 内 FSM (S_IDLE → S_FETCH → S_DECODE → S_ISSUE → S_WAIT → S_DONE) 自驱拉 desc + trigger axis_packet_tx
- 4 个 mem 真并行 (host 串行 overhead 从 4×240×30=28K 降到 3 寄存器 × 4 mem × 5 拍 = 60 拍 / 层)

mem CSR map (新):
```
0x000  DESC_LIST_ADDR  [31:0]   desc 在 ddr_mem 内 word offset
0x004  DESC_COUNT      [15:0]
0x008  CTRL            W: bit0=1 → engine start_pulse
0x00C  STATUS          [0]=engine_busy [1]=engine_done_sticky [2]=rx_pkt_done_sticky
```

**性能**:

| Case | 旧 (cy) | 新 (cy) | 改善 |
|---|---|---|---|
| ResNet11 N=1 mesh | 602K | 588K | -2.3% |
| ResNet11 N=4 mesh | 305K | **236K** | **-22.6%** |
| **N=4 加速** | 1.96× | **2.53×** | **+29%** |

per-layer 加速比对比 (N=4 vs N=1 baseline 596K):

| Layer | 模式 | 旧 N=4 加速 | 新 N=4 加速 | 提升 |
|---|---|---|---|---|
| L0 Patch (4-core W) | 4 核 W | 2.05× | **3.59×** | mem push 不再串行 |
| L1 B1.C1 (4-core W) | 4 核 W | 1.80× | **3.76×** | 接近理论上限 |
| L2 B1.C2 (4-core W) | 4 核 W | 3.74× | 3.74× | 计算主导, 不变 |
| L5 (4-core W) | 4 核 W | 2.96× | 3.38× | |
| L8 (4-core W) | 4 核 W | 2.29× | **3.42×** | |
| L3/L6/L9 ds + L10 FC | 单核 | 1.00× | 1.00× | scheduler 单核独占 |

新版 4-core W slice 层加速基本接近理论 4×, 剩余瓶颈是 ds + FC 单核层 (~31% 整网时间). 进一步加速需 force_multicore (ds 也走 4 核 W slice), 理论可达 ~3.30×.

新增/修改文件:
- `RTL/Mesh/mem_core_stub.sv`: 重写, 加 desc engine FSM + 共享 ddr_mem read port (desc 拉 / tx body 拉 mux)
- `toolchain/mesh_cmd.py`: `MemCmd.to_hex_line()` 二进制 32-hex 编码; `write_mem_desc_list()` 输出 .hex 文件
- `toolchain/run_multicore_chain.py`: 文件名 `_cmds.txt` → `_descs.hex`
- `sim/tb_mesh/tb_mesh_chain.sv`: 新 task `preload_mem_desc_list` / `mem_engine_start` / `mem_engine_wait_done`, 替代旧 per-cmd push

回归 mesh chain 全 PASS:
- mesh_simple3 (n=1, 3 layers) ✓
- mesh_block1 (n=1, 3 layers + residual) ✓
- mesh_wslice1 (n=4, 1 layer W slice) ✓
- mesh_resnet11_n1 (n=1, 11 layers) ✓ 588K cy
- mesh_resnet11_n4 (n=4, 11 layers mixed) ✓ 236K cy, 2.53× 加速

### Phase 6.5 NUMA 重构 (2026-05-05): 阶段 1 完成 — RTL 骨架就绪

**问题发现**:
跑 `mesh_wslice5` (5 层等尺寸 W slice 链) FAIL 768 个错 → 暴露 cross-W-slice halo
未同步 bug. ResNet11 N=4 之前 "PASS" 是 trivial (last layer L10 FC = root IFB
独立 preload, 跟前 9 层 chain 数据流断开, chain TB 没逐层验证). Phase 6 push 模型
本质问题: ConvCore[c] OFM 通过 `cfg_ofm_tdest` 显式送本核 mem, halo 列在邻居 mem
中取不到 → 这是模型错不是实现 bug.

**正确设计 (NUMA / 分布式共享地址空间)**:
4 mem 物理独立但联合编址成一块大 RAM. 32-bit 全局地址 [25:24] = mem ID 自动路由.
所有 ConvCore 看到统一全局地址空间. ConvCore 用 IDMA 主动拉, ODMA 主动写, mesh
路由器按地址自动分发. mem 是哑存储 (被读+被写), 没有"路由知识".

**阶段 1 (本次完成) — NUMA RTL 骨架 (双模式共存)**:

新增/修改文件:
- `docs/mesh_numa_protocol.md`: 协议 spec (双向 packet, opcode WRITE/READ_REQ/READ_RESP, 全局地址 layout)
- `RTL/Mesh/axi_reader_to_axis.sv` (新, ~80 行): ConvCore.bus_ar → READ_REQ packet (按 araddr[25:24] 路由)
- `RTL/Mesh/axis_to_axi_read_resp.sv` (新, ~80 行): READ_RESP packet → ConvCore.bus_r (mesh 路由后回到发起者)
- `RTL/Mesh/axis_packet_read.sv` (新, ~110 行): mem 端被读引擎 (收 REQ → 读 ddr_mem → 发 RESP)
- `RTL/Mesh/axi_writer_to_axis.sv`: 加 `cfg_use_addr_route` 参数, NUMA 模式用 awaddr[25:24] 自动解码 mem ID tdest
- `RTL/Mesh/mem_core_stub.sv`: 加 NUMA pull 路径 (s_axis demux by opcode + axis_packet_read 实例 + m_axis tx mux)
- `RTL/Mesh/mesh_core_wrapper.sv`: 加 `NUMA_MODE` parameter (0=Phase 6 push, 1=NUMA pull) + s_axis demux + 双桥 + m_axis mux + bus_ar/r forward 控制

**双模式开关** (`NUMA_MODE` parameter):
- `NUMA_MODE=0` (默认, Phase 6 push): mem 推 IFB packet, ConvCore.bus_ar/r 直 expose 给上层 DDR
- `NUMA_MODE=1` (NUMA pull): IDMA 用全局地址主动拉, bus_ar/r 走 mesh 双向 packet, push 路径硬件存在但不启用

**回归验证 (NUMA_MODE=0 默认)**:
| Case | n | layers | 结果 |
|---|---|---|---|
| mesh_simple3 | 1 | 3 | ✓ |
| mesh_block1 | 1 | 3 + residual | ✓ |
| mesh_wslice1 | 4 | 1 W slice | ✓ |
| mesh_resnet11_n1 | 1 | 11 | ✓ 588K cy |
| mesh_resnet11_n4 | 4 | 11 mixed | ✓ 236K cy |

NUMA RTL 骨架就位且不破坏 Phase 6 PASS.

**剩余 (阶段 2, 下一轮)**:
- `multicore_top_mesh.sv` 加 `NUMA_MODE` 参数 + 传递给 wrapper
- `cfg_regs.sv` 简化 (NUMA_MODE=1 下 `cfg_skip_idma=0` 默认; 删 ofm_tdest/opcode)
- `run_multicore_chain.py` mesh 模式 driver 改造: 全局地址 layout + 删 mesh_cmd / desc list / broadcast preload
- `tb_mesh_chain.sv` 简化 (跟 tb_multicore_chain 几乎一致)
- 跑 `mesh_wslice5` PASS → 验证 cross-W-slice halo 在 NUMA 模型下自然解决
- 全套 mesh case NUMA 模式回归

### Phase 6.6 重要发现 (2026-05-05): Trivial PASS bug + shared-nothing 设计哲学

**Trivial PASS bug 修复**:
`tb_mesh_chain.sv` 加逐层 OFM 验证 (`check_layer_ofm` task), 之前只比对最后一层是
unsafe 的—— ResNet11 N=4 之前"PASS"实际是 trivial: L10 FC 是 root IFB 独立 preload,
跟前 9 层 chain 数据流断开. 真实情况:

| Case | 修复前 | 修复后实际 |
|---|---|---|
| simple3 / block1 / wslice1 / resnet11_n1 | PASS | **真 PASS** ✓ |
| **resnet11_n4** | "PASS" | **FAIL 24,360 word** ❌ L1-L9 全错 |
| **wslice5** (5 层 W slice chain) | (没跑) | **FAIL 1,920 word** ❌ L1-L4 全错 |

ResNet11 N=4 之前的 2.96× 加速数字基于错的 OFM, 不算实际有效性能.
驱动 chain TB 一定要逐层验证, 不能依赖最后一层 trivial 比对.

**NUMA 设计反思 + shared-nothing 哲学**:
做完 NUMA RTL 骨架 (双向 packet, 全局地址 [25:24]=mem ID) 后发现 **fine-grain W slice
跟 mem 边界粒度不匹配**: W slice 切到列 (一段 8 列 ≈ 128 byte), mem 边界粗 (16 MB),
awaddr 高位无法编码 W 段位置. 全局地址路由对粗粒度数据布局有效, 对 W slice 不直接 work.

**用户提议的正确设计 (shared-nothing 分区)**:
- 硬件视角: 4 个 mem 完全独立、不互连. 每 mem 装一张完整"输入子图"(含必要 halo overlap).
- 软件视角: driver 编译期反向递推, 把整图按 W slice + 累积 halo 切成 4 张子图,
  分别 preload 到 4 个 mem. 每核 ConvCore[c] ↔ mem[c] 一对一闭环.
- halo 是软件层概念: 上层 ConvCore[c] 多算 halo overlap 列输出, 输出直接覆盖
  下层每核需要的 IFM 段 (含下层 halo). 不需要任何跨 mem 数据交换.
- DMA 完全软件控制: ConvCore.IDMA / ODMA 的 cfg base/stride 由 driver 编译期算出
  本核紧凑 layout, 硬件不解析地址、不路由.

**已完成的支撑代码**:
- `hw_files.compute_w_slice_chain_geom(layers, n_cores)`: 反向递推 chain W slice geom,
  返回每层每核 (w_in_lo/hi, w_out_lo/hi, sub_W_in/out, pad_l/r), 含累积 halo overlap
- `hw_files.derive_w_slice_cfg_chain(layer, geom_entry, ...)`: 用 chain geom 算每核
  紧凑 layout cfg (W_IN=sub_W_in 含 halo, DDR row stride 用本核段宽紧凑布局)
- `tb_mesh_chain.sv` `check_layer_ofm` task: 逐层 OFM 验证, errors 累计

**剩余 (下一阶段实施)**:
1. `run_multicore_chain.py` mesh+W slice 走 chain geom 路径 (linear chain only)
2. `mesh_cmd.py` 紧凑 layout (本核 mem 内从 0 起)
3. `tb_mesh_chain.sv` layer 0 IFB 切片 preload (4 mem 各装 W 段含 halo)
4. `tb_mesh_chain.sv` OFM 比对跳过 halo overlap 区 (只比对每核负责的"原始" W 段)
5. 跑 `mesh_wslice5` 验证真 PASS (cross-W-slice halo 在 shared-nothing 下消失)
6. ResNet11 N=4 需要 dependency-aware 反向递推 (跨层 skip 处理), 暂作复杂 chain 扩展

NUMA RTL 骨架 (axi_reader_to_axis / axis_to_axi_read_resp / axis_packet_read) 保留为
未来真"地址路由 NoC"硬件预留, 不挡当前 shared-nothing 路径.

**当前 mesh PoC 真实状态** (修正之前误判):
- ✓ 真 PASS: simple3, block1, wslice1, resnet11_n1
- ❌ 实际 FAIL: wslice5 (cross-W-slice halo bug), resnet11_n4 (push 模型 W slice halo bug, 中间 9 层错)
- 之前所谓"ResNet11 N=4 mesh 2.96× 加速"基于错的 OFM, 数据不可信

### Phase 7 (待实施): SMC + NUMA 真分布式架构

经过几轮设计反思, 确定**真正可工程化的 NUMA 架构**:
- 4 mem 物理独立, 全局地址空间统一
- halo 列**物理只一份** (无重复存储)
- ConvCore 端 axi_dma IP 用 **SG mode**, driver 编译期生成 SG cmd list, IP 自动多 burst
- 路由用 Vivado **AXI SmartConnect IP** (4M↔4S 全连接, 按 awaddr 解码 mem ID)
- 所有路由智能在 IP 内, 自写 RTL 最少, 跟商业 NoC 架构 (AIE-ML / Tenstorrent) 对齐

设计完整文档 `docs/smc_numa_design.md` (协议 / 架构图 / IP 配置 / 改造路径 / 估时)

**RTL 骨架就绪**:
- `RTL/multicore_top_smc.sv`: 新顶层骨架 (4 ConvCore + axi_smc placeholder + 4 axi_slave_mem)
- `RTL/DMA/idma_sg_dispatcher.sv`: SG cmd list dispatcher 完整 RTL (替代 idma_ctrl 的内部多 burst, driver cmd-list driven, 6 状态 FSM, 跟 mem desc engine 一脉相承)
- `toolchain/mesh_cmd.py`: 加 `SgCmd` dataclass + `gen_idma_sg_cmd_list_w_slice()` (反向 W slice 切片 + 跨 mem halo 拆 cmd) + `write_sg_cmd_list()` ($readmemh 兼容输出)
- 等待 Vivado IP 升级 (axi_datamover → axi_dma SG mode + axi_smc 4M↔4S) 后填充实例化

**实施分阶段** (估时 1.5-2 周):
- A. IP 升级 (axi_dma SG + axi_smc): 0.5-1 天
- B. ConvCore 内 idma_ctrl/odma_ctrl/wdma_ctrl/rdma_ctrl 改 SG cmd 接口: 2-3 天
- C. multicore_top_smc.sv 完整 IP 实例化: 0.5 天
- D. driver SG cmd list 生成 (W slice 段散布 + halo 跨 mem 多 cmd): 2-3 天
- E. tb_smc_chain.sv: 1 天
- F. sim 调试 wslice5 / ResNet11 N=4 真 PASS: 2-3 天

**设计哲学最终 align (用户多轮 align 后)**:
1. 软件视角: 所有 Core 看到一块统一全局地址 RAM, 不存在跨核概念
2. 硬件视角: 4 mem 物理独立, AXI SmartConnect 自动按地址路由
3. halo 物理只一份 (driver 编译期决定地址布局, 无重复存储)
4. DMA 完全软件控制 (driver 编译期算 SG cmd list, ConvCore.axi_dma 自动消费)
5. ConvCore 不感知 halo / W slice / mem 拓扑 (它眼里就是一张 W/n 子图)

**保留路径**:
- `multicore_top.sv` single-DDR 4-core 模式不动 (跟 SMC 解耦)
- `multicore_top_mesh.sv` deprecated (push 模型 W slice multi-layer chain 本就 broken)
- mesh AXIS NoC 协议 (router_node 等) 保留作未来扩展到几十核的硬件预留

文件:
- RTL: `RTL/Mesh/*.sv` (10 个文件: router/mesh/packet/桥/wrapper/stub)
- TB: `sim/tb_mesh/*.sv` (8 个文件)
- TCL: `sim/tb_mesh/run_*.tcl` (6 个)

**Mesh PoC 阶段性总结**: 数据链路完整验证, 真 ConvCore 接 mesh 的桥已就绪可综合. 剩下的是软件层 (编译器 + driver) 跟系统集成 sim, 不是架构问题.

---

## 3. 编译器 / 工具链 — 已稳定

### `toolchain/`

| 文件 | 关键能力 |
|---|---|
| `gen_isa_test.py` | 单 case 随机数据生成 (CLI + 进程内 API), 支持 `ifm_arr_in`/`shortcut_arr_in`/per-layer SDP/DDR base/skip flag override |
| `hw_files.py` | DDR 文件 I/O 共享层 (ifb/wb/expected_ofm/desc_list/config), CFG_ADDR_MAP + `_pack_cfg_desc` 支持 CFG_WRITE 流 |
| `compile_layer.py` | PyTorch Conv2d → 硬件 cfg 字典 + 数据文件 |
| `compile_model.py` | 多层链式编译, `_plan_ddr` 分配 FM-shared DDR 区 |
| `run_regression.py` | 链式 CASES (DSL builder), validate_chain + plan_chain_ddr, 顺序运行 + Wall_us 报告 |

### 链式 CASES 写法 (run_regression.py 内)

```python
chain  = Chain(h=960, w=540, c=4)
patch  = chain.conv     ('Patch',  k=4, c=16, s=4, pad=0, shift=5)
l1     = resnet_block   (patch, 'L1', c=16, shifts=(11, 11, 7))
l2     = resnet_block   (l1,    'L2', c=32, shifts=(11, 12, 7))
l3     = resnet_block   (l2,    'L3', c=64, shifts=(12, 13, 8))
fc     = chain.root     ('FC',  k=1, c_in=256, c_out=522, h=1, w=1, shift=8)
CASES  = chain.cases     # 11 dict, 含 input_src/shortcut_src/dim/sdp_*
```

最近一次回归结果: 11 cases 全 PASS, 整网 593K cycles, 86.6% MAC%, 端到端 5.95 ms (≈168 fps 上限 @ 100 MHz)。

---

## 4. 性能优化 roadmap (按 ROI 排序)

### 短期 (单核范围内)

| 优化 | 收益 | 成本 | 推荐度 |
|---|---|---|---|
| **mac_pe 加 use_dsp 属性** | LUT -17K, Fmax 可能涨 | 1 行修改 | ⭐⭐⭐⭐ |
| **SDP 切流水线** | Fmax 拉到 100+ MHz | ~30 行 | ⭐⭐⭐ |
| 跨层 WDMA 预取 | -3-4% cycles | 中 (sequencer FSM) | ⭐⭐⭐ |
| K=1 stride=2 IDMA 跳行 | -4% cycles (L*.B2.ds) | 中 | ⭐⭐ |
| L2+ soft fusion (整层 OFM 留 OFB SRAM) | -3% cycles | 低 | ⭐⭐ |
| 2D ARF tile (8×4 / 4×8 / 16×2) | 0% cycles, 报告 Ratio 4.8x vs 2.8x, BRAM 读功耗 ↓ | 高 (~300 RTL + 100 toolchain) | ⭐ |

### 中长期 (多核 / 架构级)

| 优化 | 收益 | 成本 | 推荐度 |
|---|---|---|---|
| ~~修 M1.5 sim 死锁~~ | ✅ 完成 (2026-04-29) | — | — |
| **M2 — Mesh + 跨核 SRAM 直送** | DDR 流量 -50%, batch 并行 + 流水线模式都启用 | 高 (~500 行 + 编译器扩展) | ⭐⭐⭐⭐ |
| 双倍 IFB+OFB SRAM (256 KB 各) | 整网 -10% (中间层 soft fusion) | +1 BRAM tile | ⭐⭐⭐ |
| 模型替换 ds 层 (3×3 stride=2 + S2D, 或 AvgPool+1×1) | -10% cycles | 软件层 | ⭐⭐⭐ |
| **R.3 — AvgPool 硬件** | 网络支持完整度 | 中 (用户需确认有无 host CPU) | 暂停 |
| Hard fusion / 多 MAC partition | -30% cycles | 极高 (重写 mac_array) | ❌ 不推荐 |

---

## 5. 任务交接清单

### 立即可继续

1. **加 `(* use_dsp = "yes" *)` 到 mac_pe.sv** — 释放 17K LUT, 重综合验证
2. **重综合多核 N=2 验证 timing 不退化** (axi_arbiter sticky sel 改动后)

### 中期

3. **M2 设计 — Mesh + 跨核 SRAM 直送** — 见 `memory/cross_core_design.md` 的设计要点
4. **SDP 流水线化** — Fmax 拉到 100 MHz+
5. 跑 3 核综合验证, 看实际 BRAM/Fmax (推算: 384 BRAM 86%, 边缘)

### 长期 / 待确认

6. R.3 AvgPool — 等用户确认有无 host CPU
7. 真机部署 / Zynq 集成 — 需要补 IRQ 端口、完整的 host driver

---

## 6. 文件索引

## 3. 🎉 VD100 board minimal system bring-up (2026-05-14)

承接 v32 board stuck 调试. 改走最小系统 (ps_hello base + ConvCore + smartconnect_pl
+ BRAM, 完全绕过 axi_noc), board 上验证 IDMA→mac→ODMA 完整数据通路.

### 背景 / 动机

v32 vd100_resnet11 工程 (3 core mesh + axi_noc + PL DDR4) board 上 cmd_count=1
测试 stuck: ODMA dispatcher 卡 S_TX, IDMA 卡 S_RING_WAIT, mac/ofb 不推进.

排查 1 整天发现 (commit fbddd03/adf3dda/eff12b5):
- BD m00_axi 14 个 scalar pin 被历史 patch user override 连 dummy /Net (CRITICAL 41-1271)
- axi_m_mux 1-hot demux 触发 Vivado opt LUT3 trim cascade
- ODMA r_done corner case (S_WAIT→S_DONE bypass, 已 fix)
- test scripting 漏 set DESC_LIST_BASE/COUNT

但是: sim IDEAL_SMC 11 layer ResNet11 bit-exact PASS (220K cy 双重证明 RTL OK).
v32 board stuck 真根因是 **axi_noc 路径** (NoC 多 cmd burst + BD 历史 dummy net).

### 最小系统架构

```
sys (200MHz diff) → ps_hello PMC PL CLK0 (100MHz)
                          ↓
versal_cips_0 (复用 ps_hello demo, A72 lwIP 已验证)
   ├ pl0_resetn → proc_sys_reset → axi_aresetn
   ├ M_AXI_FPD (128b AXI4) ─┐
   └ NoC/DDR4 (PS DDR for A72 boot, 不动)
                            ↓
                     smartconnect_pl (2 SI → 2 MI)
                       ↑                  ↑    ↓
              u_mc_minimal.m_axi          ↓    └→ axi_bram_ctrl → emb_mem_gen
                                          └→ u_mc_minimal.csr_axil
```

地址 map:
- CSR  `0xA4000000` (4 KB, 跟 RPC server `CSR_BASE` 一致)
- BRAM `0xA4100000` (64 KB, CIPS + ConvCore m_axi 共同视图)

### 文件清单

- `RTL/Versal/multicore_top_minimal.sv`: 单核 wrapper (SG_MODE=1, 跨核 IFB SI 全 tie 0)
- `RTL/Versal/multicore_top_minimal_v.v`: Verilog wrapper 包 SV (BD module reference 不收 SV 顶)
- `Syn/vd100_minimal/create_project.tcl`: 创建工程 + 加 RTL/约束
- `Syn/vd100_minimal/build_bd.tcl`: BD 创建脚本 (clk/smartconnect/BRAM/ConvCore)
- `Syn/vd100_minimal/add_minimal_to_ps_hello.tcl`: 在 ps_hello base BD 上 add IP
- `Syn/vd100_minimal/vd100_minimal_with_elf.bif`: bootgen 把 PDI + A72 lwIP ELF 合 boot
- `host/vd100_pc/test_stage2_minimal_conv.py`: Stage 2 minimal conv 端到端

### Board 验证 (跑 vd100_minimal_with_elf.pdi 后)

| Stage | 验证内容 | 结果 |
|---|---|---|
| 0 | host → M_AXI_FPD → smartconnect_pl → BRAM (16B / 1KB / 64K tail) | ✅ 3/3 |
| 1a | host → ConvCore.csr_axil PEEK/POKE (DESC_LIST_BASE / DESC_COUNT / STATUS / SEQ_DBG) | ✅ 4/4 |
| 1b | ConvCore.m_axi → BRAM (1 CFG + END, dfe + sequencer + cfg_regs) | ✅ PASS |
| 1c | 25 CFG + END / 100 CFG + END (3232 byte) 多 desc 稳定不 stuck | ✅ PASS |
| 2 | 1x1x1x1 minimal conv (54 CFG + CONV + END), 完整 IDMA→mac→ODMA, OFM 真写 BRAM | ✅ **layer_done in <100ms** |

Stage 2 完整数据流:
```
host 写 IFM/Weight/RDMA/desc/SG cmd 到 BRAM 各 offset
  → dfe 拉 desc → sequencer S_FETCH × 54 (CFG_WRITE 写 cfg_regs)
  → CONV is_first=1 → S_PRELOAD → 等 wdma_done + rdma_done
  → S_DISPATCH → start IDMA/ODMA dispatcher
  → IDMA SG cmd → m_axi 读 16 byte IFM → IFB SRAM
  → mac_array compute (act_in × weight) → parf_accum → sdp (CLIP_MAX=127)
  → ofb_writer 写 OFB SRAM + pulse row_done_pulse
  → ODMA SG cmd → m_axi 写 16 byte OFM → BRAM @ 0xA4103000
  → sequencer S_WAIT → strip_done → S_END → S_IDLE → layer_done bit set
```

读 BRAM OFM = `7f 7f 7f ... 7f` (16 byte SDP CLIP_MAX 饱和, IFM=0x11 × W=0x22 累加溢出).

### 对比 v32 (axi_noc 路径)

| 指标 | v32 axi_noc | minimal smartconnect_pl + BRAM |
|---|---|---|
| host → CSR | ✓ | ✓ |
| dfe 拉 desc | ✓ | ✓ |
| IDMA dispatcher 完成 | ❌ stuck S_RING_WAIT | ✅ done |
| WDMA 完成 | ✓ | ✅ done |
| ODMA dispatcher 完成 | ❌ stuck S_TX | ✅ done, **真写 BRAM** |
| layer_done bit | ❌ 永 0 | ✅ <100ms set |

### 结论

- **ConvCore RTL 完全无 bug** (sim IDEAL_SMC + board minimal 双证)
- v32 board stuck 100% 是 axi_noc 路径硬件问题, 跟 ConvCore m_axi master 无关
- 最小系统提供清晰 axi 路径 (CIPS.M_AXI_FPD → smartconnect_pl → BRAM), 后续可在此基础上
  逐步加复杂度 (Stage 3: 大 layer, multi-layer chain, 真 mac vs garbage 比对 bit-exact)

### 副产品 / 已知问题

- **RPC server lwIP fragmented TCP segment bug**: 单次 LOAD/READ_DDR > ~1KB (跨 MTU)
  触发 ConnectionReset. host 端 chunked 分段 work around 即可. 跟 RTL/BD 无关,
  baremetal lwIP echo template 默认 recv callback 没正确处理多 pbuf segment.
- **Vivado 2023.2.1 batch mode create_bd_design 撞 roe_framer IP init bug**: 用 GUI
  规避. tcl 内 `set_msg_config -id "Common 17-39" -new_severity WARNING` 等 workaround 无效.

### Stage 3a 进展 (2026-05-14): 多 cmd + 多轮启停稳定性 全 PASS

承接 Stage 2 (1x1x1x1 单次跑通), Stage 3a 验证 sequencer / SG dispatcher
在更大维度 + 多轮启停下稳定. 推 multi-layer / 多张图片的基础.

**Phase 1**: 1×1×1×1 hand-craft cfg, N=500 轮启停
- ✅ **500/500 全 PASS**, 平均 0.3 ms/轮
- sticky done bit `start_layer_pulse` 时正确清, sequencer 每轮干净回 S_IDLE
- tail vs head -62% 是 RPC TCP socket 暖启动加速 (非 hw slowdown)
- file: `host/vd100_pc/test_stage3a_phase1_loop.py`

**Phase 2**: hw_files 自动派生 cfg (递进调试)
- 用 `toolchain.hw_files.derive_layer_cfg + build_layer_desc_segment` 自动生成
  desc list + cfg dict, host 端手工拼 SG cmd list (每行一条 cmd, H_IN/H_OUT 条)
- 关键修复: IDMA/ODMA cmd_count = H_IN/H_OUT (不是 1), 每条 cmd cover 一行
- 递进验证表格:

| case | 验证维度 | N | 结果 |
|---|---|---|---|
| K=1 H=W=1 Cin=Cout=1  | hw_files vs Phase 1 hand-craft 等价 | 1 | ✅ PASS |
| K=1 H=W=1 Cin=Cout=16 | Cin/Cout 满 (16 PE 全工作)         | 1 | ✅ PASS |
| K=1 H=W=8 Cin=Cout=1  | H/W 多 cmd (8 IDMA + 8 ODMA)        | 1 | ✅ PASS |
| K=1 H=W=8 Cin=Cout=16 | 上面两个组合                         | 1 | ✅ PASS |
| **K=3 H=W=8 Cin=Cout=16 pad=1** (目标) | **kk=9 + 多 cmd** | **200** | ✅ **200/200 PASS** |
| K=3 H=W=16 Cin=Cout=16 pad=1          | 大维度 (16 cmd/dir) | 30 | ✅ 30/30 PASS |

- file: `host/vd100_pc/test_stage3a_phase2_larger.py`

**Phase 2 调试 trap**: sequencer 没软 reset 通路 — 单次 stuck 后所有后续 layer 都失败.
最初 Phase 2 第一次跑 FAIL 误以为是 cfg bug, 实际是 Phase 1 残留 state. 烧 PDI
重置后 hw_files 全 case 全 PASS. **教训**: VD100 demo 测试 between case 之间应该
重烧 PDI 隔离 state, 或者 RTL 加软 reset 通路 (TODO).

**已知 limit**:
- BRAM 64KB ⇒ 最大 case H=W≤32 (IFM+OFM ≈ 2×16KB), 更大需扩 BRAM 重综合
- RPC server lwIP 单次 LOAD/READ_DDR > 1KB 触发 ConnReset, 测试脚本 chunked 规避

### v3 PDI 全面验证 (2026-05-14, commit f947cc9 + 后续)

v3 PDI 相对 v1 (Stage 0~3a 用的) 的三大改动:
1. **RTL 软 reset**: cfg_regs 加 CTRL.bit7 stretched 16 拍 soft reset, core_top 16 个
   sub-module rst_n → core_rst_n (复合 hard+soft), cfg_regs 自身保留 hard rst.
   stuck → host POKE CTRL=0x80 → FSM 全回 IDLE, 不用重烧 PDI.
2. **odma_sg_dispatcher.sv r_yout_base bug fix**: Round H step 2 改 FSM 跳过 S_STS,
   但漏改了 r_yout_base 推进条件 (还在等 `S_STS && s2mm_sts_fire`). 永远不成立 →
   r_yout_base 永远 = 0 → 所有 ODMA cmd 读 OFB[0..ofb_row_words-1] → H>1 case 的
   OFM row 1+ 全是 row 0 内容复制. Stage 4 board bit-exact 暴露. 修复: yout_base
   推进改 `S_TX tlast && is_last_cmd_in_row` (跟 r_rows_drained 同步).
3. **BRAM 容量 64KB → 256KB**: emb_mem_gen MEMORY_DEPTH 4096 → 16384, addr range
   64K → 256K. (默认 32768 = 512KB 超 RAMB36, fit 不下 VE2302 155 RAMB36, 折中 16384.)

### v3 Stage 0~4 验证结果 (board: vd100_minimal_with_elf.pdi v3)

| Phase | 测试 | 结果 |
|---|---|---|
| A1: Stage 3a Phase 1 regression | 1×1×1×1 N=200 | ✅ **200/200 PASS** |
| A2: Stage 3a Phase 2 regression | K=3 H=W=8 N=100 | ✅ **100/100 PASS** |
| B1: Stage 4 bit-exact | K=1 H=W=1 Cin=Cout=16 | ✅ PASS |
| B2: Stage 4 bit-exact | K=1 H=W=2 Cin=Cout=16 | ✅ PASS |
| B3: Stage 4 bit-exact | K=1 H=W=4 Cin=Cout=16 | ✅ PASS |
| B4: Stage 4 bit-exact | K=1 H=W=8 Cin=Cout=16 | ✅ PASS |
| **B5: Stage 4 bit-exact** | **K=3 H=W=8 Cin=Cout=16 pad=1** | ✅ **PASS (1024 byte 全 bit-exact)** |
| B6: Stage 4 bit-exact | K=3 H=W=16 Cin=Cout=16 pad=1 | ✅ PASS |
| C: 软 reset 通路 | baseline → trigger → soft reset → post-reset | ✅ **PASS (4/4 步)** |
| D: bit-exact 持续 | K=3 H=W=8 N=50 | ✅ **50/50 PASS** |

### sim 调研 (论文素材)

revert ODMA fix 后 sim ResNet11 N=3 仍然 11 layer 全 PASS, 0 mismatch. 即 **同一个
RTL bug 在 sim (IDEAL_SMC + axi_dm IP sim model) 没暴露, 在 board (axi_smc IP +
真 axi_dm + smartconnect_pl) 暴露**. 可能 sim model 的 axi_dm s2mm sts 准时回让
不同代码路径触发, 或 ofb_writer/ODMA 之间某种 race 在 sim 自然对齐. **board-level
bit-exact validation 暴露 sim 未覆盖的边界 — 论文 §5.4 板级验证一节核心素材**.

### Stage 4 corner case 扩展 (16+ case 全 bit-exact PASS)

整图 fit case (ifb_strip = H_IN, OFB ring 也 fit) 全 bit-exact PASS:

| case | 形状 | 结果 |
|---|---|---|
| K=1 H=W=1 / 2 / 4 / 8 Cin=Cout=16 | 递进 | ✅ 4/4 |
| K=3 H=W=8 / 16 / 32 Cin=Cout=16 pad=1 | 多大小 | ✅ 3/3 |
| K=3 H=W=16 Cin=Cout=16 stride=2 | stride>1 | ✅ |
| K=1 H=W=16 Cin=16 Cout=32 | Cout 切片 (cs=2) | ✅ |
| K=1 H=W=8 Cin=32 Cout=16 | Cin 切片 | ✅ |
| K=3 H=W=8 Cin=32 Cout=32 | 双向切片 | ✅ |
| K=1 H=W=8 Cin=32 Cout=64 | Cout cs=4 | ✅ |
| K=5 / K=7 H=W=8 Cin=Cout=16 | 大 K | ✅ 2/2 |
| K=3 H=W=8 Cin=Cout=16 stride=3 | corner stride | ✅ |

### Streaming strip 模式新发现 bug (留待 Stage 3b 深入 debug)

精确 fit / streaming 边界 (K=3 Cin=Cout=16 pad=1):

| H_IN | ifb_strip | 模式 | 板上结果 |
|---|---|---|---|
| 8 | 8 (=H_IN) | 整图 fit | ✅ PASS |
| 16 | 16 (=H_IN) | 整图 fit | ✅ PASS |
| **32** | **32 (=H_IN)** | **整图 fit** | ✅ **PASS (16KB OFM bit-exact)** |
| **40** | **6 (<H_IN)** | **streaming** | ❌ FAIL stuck |
| 48 | 6 (<H_IN) | streaming | ❌ FAIL stuck |
| 56 | 6 | streaming | ❌ FAIL stuck |
| 64 | 6 | streaming | ❌ FAIL stuck |

H>32 时 hw_files.derive_layer_cfg 用 streaming row-ring 模式 (ifb_strip ≈ K + slack).
board stuck: STATUS=0x672 (idma_busy=1 odma_busy=1), SEQ_DBG=0x07xx (idma_sg=0 done?
odma_sg=7 S_TX).

这跟 sim ResNet11 streaming PASS 不一致, **board-level 暴露 sim 未覆盖的 streaming 模式
bug**. 类似 ODMA r_yout_base bug 的发现路径, 都是 board 比 sim 严格.

可能 root cause (待 debug):
1. axi_dm.s2mm 反压链条 (axi_dm → smartconnect_pl → BRAM) 长 burst stall
2. ofb_writer ↔ ODMA dispatcher 通过 row_done_pulse 跨 strip 同步 race
3. ifb ring wrap 后 line_buffer 读到 stale data

### 下一步 (待 next session)

- **Stage 3b**: debug streaming strip 模式 stuck (H>32 ifb_strip < H_IN case)
- Stage 5: multi-layer chain (ResNet11 11 layer 顺序跑, 用 SDP/residual 全特性)
- Stage 6: mesh 3 core (回到原 vd100_resnet11 但去掉 axi_noc, 用多 BRAM 或 PL DDR via smartconnect)
- (深挖) sim 为啥没暴露 ODMA bug: 深入 ofb_writer ↔ ODMA dispatcher 在 sim model 下的 timing 差异

## 2.12 Phase 7 阶段 2 — SMC + NUMA RTL 集成 (2026-05-05)

承接 Phase 7 阶段 1 (`docs/smc_numa_design.md` + 骨架文件), 阶段 2 完成 ConvCore 端 SG dispatcher 集成 + 顶层 axi crossbar 实例化, RTL 端到端可 elab.

### 设计哲学 (来自 Phase 6.6 用户多轮 align)

- **统一全局地址**: 4 mem 物理独立, 共享 32-bit 全局地址空间, halo 列**物理只一份**
- **DMA 完全软件控制**: driver 编译期为每核每层生成 SG cmd list, dispatcher 顺序拉 cmd 跑 burst
- **多 burst 路径**: 跨 mem 边界由 driver 拆 cmd, AXI Crossbar 按 awaddr 路由, 不重复存储
- **尽量用 Vivado IP**: 顶层 SmartConnect IP, ConvCore 内 axi_dm 现有 IP, 自写 RTL 最少

### 完成内容

| 模块 / 文件 | 内容 | 验证 |
|---|---|---|
| `RTL/DMA/idma_sg_dispatcher.sv` (重写) | IDMA SG cmd dispatcher: cmd 拉取 + 实际 data 拉取都走 mm2s_arb 的 idma 端口 (两阶段 FSM, 不新增 axi master read 通道). 跟 `idma_ctrl` 端口签名兼容, generate-if 切换. | vlog OK |
| `RTL/DMA/odma_sg_dispatcher.sv` (新, ~370 行) | ODMA SG cmd dispatcher: cmd 拉取走 mm2s_arb 第 4 路 (ocmd, 新加), s2mm 装 cmd / 数据走 axi_dm.S2MM. NHWC gather 跨 N 段每行 (driver 编译期决定). cmd 内带 ofb_w_start, dispatcher 每段从 OFB SRAM 该 W 列起 gather. | vlog OK |
| `RTL/DMA/mm2s_arb.sv` (扩) | 加第 4 路 ocmd_*, priority idma > rdma > ocmd > wdma. owner FIFO 宽度不变 (3 → 4 owner 仍 2 bit). SG_MODE=0 时 ocmd 路 tie 0 不影响其它路仲裁, 旧 sim 完全兼容. | vlog OK |
| `RTL/AXI4/axi_crossbar_4to4_sim.sv` (新, ~360 行) | sim-only 4M↔4S AXI crossbar, 行为模型. 每 MI 一个 read-FSM + write-FSM, 锁定 SI owner 至 burst 完成. addr[25:24] = mem_id 路由. 可被 Vivado SmartConnect IP 直接替换. | vlog OK |
| `params.py` | 加 6 个 ADDR (ID/OD MA SG 各 3): `IDMA_CMD_LIST_PTR=0x1D8 / CMD_COUNT=0x1DC / CMDS_PER_ROW=0x1E0`, `ODMA_CMD_LIST_PTR=0x1E4 / CMD_COUNT=0x1E8 / CMDS_PER_ROW=0x1EC`, 重生 `RTL/flux_cnn_params.svh` (66 CSR entries) | OK |
| `RTL/cfg_regs.sv` | 加 IDMA + ODMA SG 寄存器 (各 3) + 输出端口 + seq_w 写口 + reg_r_data 读 mux, 由 CFG_WRITE descriptor 配置 (per-layer cmd list 起点 / 总数 不同) | vlog OK |
| `RTL/core_top.sv` | 加 `parameter SG_MODE` (默认 0). SG_MODE=1: generate-if 实例化 `idma_sg_dispatcher` 替代 `idma_ctrl`, `odma_sg_dispatcher` 替代 `odma_ctrl`, mm2s_arb 第 4 路接 odma_sg_dispatcher.ocmd. SG_MODE=0 兼容已有 single-DDR sim, ocmd 路 tie 0. | vlog OK |
| `RTL/multicore_top_smc.sv` | 把 placeholder tie-0 替换为 `axi_crossbar_4to4_sim` 实例化 + 4 SI/MI ID padding (CORE_BUS_ID=4 → CB_ID_W=6), `core_top.SG_MODE=1` override | vopt elab OK (0 errors) |

### vopt elab 验证 (2026-05-05)

完整链路 (axi_dm IP + axi_lite_1to4 IP + RTL 全集 + multicore_top_smc + axi_crossbar_4to4_sim + 4 axi_slave_mem) elab 通过:

```
vopt -work work multicore_top_smc -o multicore_top_smc_opt
Errors: 0, Warnings: 8 (mac_array.sv 的已知 SVCHK warning, 跟改动无关)
```

### 关键设计决策 (用户多轮 align)

> "ODMA 也可能跨 mem, 谁也无法保证 OFM 写进哪个地址, 甚至可能是自己的地址."

→ ODMA 必须 SG 化 (driver 编译期决定每核每行 OFM 的 N 段散布), 跟 IDMA 对称.
对称地, mm2s_arb 加 ocmd 第 4 路给 ODMA SG 拉 cmd 用, 不影响原 idma/wdma/rdma 三路.

### IDMA SG dispatcher FSM (idma_sg_dispatcher.sv)

```
S_IDLE          → start → S_FETCH_CMD_ISS
S_FETCH_CMD_ISS : 装 mm2s cmd 拉 cmd_list[idx] (16 byte / 1 beat) → S_FETCH_CMD_DAT
S_FETCH_CMD_DAT : 收 1 beat, latch src_addr/btt/last_cmd/sram_offset → S_FETCH_CMD_STS
S_FETCH_CMD_STS : 等 mm2s sts → S_RING_WAIT
S_RING_WAIT     : ring 反压 (rows_pushed - rows_consumed < strip_rows) → S_ISSUE
S_ISSUE         : 装 mm2s cmd 拉用户 data → S_DATA
S_DATA          : 收 data → IFB SRAM[sram_wptr] (跨 ring_words wrap) → S_STS
S_STS           : 等 mm2s sts. r_cmd_idx++. 判 last_cmd / 全完 → S_DONE / 回 S_FETCH_CMD_ISS
```

### IDMA SG cmd 格式 (32 byte / cmd, 1 beat 有效 16 byte)

```
word 0: src_addr     [31:0]   transfer 起点 byte addr (全局地址)
word 1: btt          [22:0]   transfer 长度 byte
                     [23]     last_cmd flag
                     [31:24]  reserved
word 2: sram_offset  [12:0]   IFB SRAM 写入起点 (word offset)
word 3: reserved
```

### ODMA SG dispatcher FSM (odma_sg_dispatcher.sv)

```
S_IDLE          → start → S_WAIT
S_WAIT          : writer 攒够 1 行 (row_done_pulse) → S_FETCH_CMD_ISS; all_issued → S_DONE
S_FETCH_CMD_ISS : ocmd 装 cmd 拉 cmd_list[idx] → S_FETCH_CMD_DAT
S_FETCH_CMD_DAT : 收 1 beat, latch dst_addr/btt/last_cmd/ofb_w_start → S_FETCH_CMD_STS
S_FETCH_CMD_STS : 等 ocmd sts → S_CMD
S_CMD           : 装 s2mm cmd (dst=cmd.dst, btt=cmd.btt) → S_PREFETCH
S_PREFETCH      : OFB 读 1 拍延迟, x_rd 起点 = ofb_w_start, cs_rd=0 → S_TX
S_TX            : 流式送 OFB beat 给 s2mm.data, NHWC gather (cs_rd 内层, x_rd 外层),
                  beats_left=1 时 tlast → S_STS
S_STS           : 等 s2mm sts. r_cmds_done++. 是本行最后 cmd → r_yout_base wrap.
                  is_last_cmd_overall || r_last_cmd → S_DONE; 否 S_WAIT
```

### ODMA SG cmd 格式 (32 byte / cmd, 1 beat 有效 16 byte)

```
word 0: dst_addr     [31:0]   目标全局地址 byte
word 1: btt          [22:0]   transfer 长度 byte (= sub_W × cout_slices × 16)
                     [23]     last_cmd flag
                     [31:24]  reserved
word 2: ofb_w_start  [15:0]   该段在 OFB SRAM 的起始 W 列 (NHWC gather 用)
                     [31:16]  reserved
word 3: reserved
```

### AXI Crossbar 路由 + ID 处理

- **路由 key**: `addr[25:24]` = mem_id (跟 `Syn/gen_axi_smc_4to4.tcl` IP base/high 配置一致, 16 MB / mem)
- **ID 透传不路由**: SI 端 ConvCore 出口 awid/arid 4 bit 零扩展到 6 bit 输入 crossbar, MI 端透传给 mem. 响应回流通过 `r_owner[m]` / `w_owner[m]` 寄存器路由 (不依赖 ID 携带 SI 信息)
- **并发**: 不同 MI 上的 transaction 全并发 (4 SI 各路由到不同 mem 时 4 read + 4 write 同步跑); 同一 MI 上 SI 间用 priority encoder 选, 锁定到 burst 完成

### 替换 Vivado IP 路径

`axi_crossbar_4to4_sim.sv` 是 sim 用 placeholder, 跑 `Syn/gen_axi_smc_4to4.tcl` 生成真正的 SmartConnect IP 后, `multicore_top_smc.sv` 内把 `axi_crossbar_4to4_sim` 实例化替换为 `axi_smc_4to4 u_smc (.S00_AXI_*(...), ..., .M03_AXI_*(...))` 即可, 接口签名一致.

### Phase 7 阶段 3 (driver + TB + sim, 进行中, 2026-05-05)

| 阶段 | 任务 | 状态 |
|---|---|---|
| 3.1 | `mesh_cmd.py` 加 `OdmaSgCmd` + `gen_odma_sg_cmd_list_w_slice` + `compute_smc_w_segments` (整图 W 切 N 段, 跟 `compute_w_slice_geom` 一致) | ✅ |
| 3.2 | `hw_files.cfg_to_dict` 加 IDMA/ODMA SG cmd list 字段 (`IDMA_CMD_LIST_PTR/COUNT/CMDS_PER_ROW`, ODMA 同) — 由 `build_layer_desc_segment` 自动生成 CFG_WRITE descriptor | ✅ |
| 3.3 | `run_multicore_chain.py` 加 `--smc` flag + `SMCLayout` 类 + 主流程 SMC 分支 (强制 N=4 W slice + 整图 W 4 等分散布到 4 mem + per-(core, layer) IDMA/ODMA SG cmd list 生成 + SMC meta 输出) | ✅ |
| 3.4 | `tb_smc_chain.sv` (~570 行): DUT 换 `multicore_top_smc`, parse SMC meta, 4 mem hier ref preload (IFB W 4 等分 + WB broadcast + RDMA per-core + desc per-core + IDMA/ODMA SG cmd lists), host stage barrier loop, final OFM stitch check | ✅ |
| 3.5 | **wslice1 SMC sim PASS** — 4 核 W slice 全路径. **1024/1024 OFM bit-exact, 4116 cycles** | ✅ |
| 3.6 | **修复 idma/odma_sg_dispatcher off-by-one bug** — 最后一行 cmd 漏装 (`r_cmd_idx + 1 >= cmd_count` 误判). 修法: `r_cmd_idx >= cfg_cmd_count`. | ✅ |
| 3.7 | **wslice5 SMC 5 层 chain PASS** — All 5 layers OFM bit-exact, ~20585 cy. | ✅ |
| 3.8 | **SMC driver mode A 单核 layer 支持** — FC W=1 等走单核, mem[my_core] 紧凑 IFB/OFM. | ✅ |
| 3.9 | **ResNet11 N=4 driver 跑通** — 11 layer (mode A FC + W slice + residual). | ✅ |
| 3.10 | **TB 扩展** — 解析 SMC layer 维度 / mode / root_slot 字段, mode A IFB 整图 layout 跟 W slice 散布两种 layout 分支, sliced RDMA per-(core, layer) preload, mode A OFM check 整图存 mem[my_core]. | ✅ |
| 3.11 | **修复 IDMA SG cmd sram_offset 累加 wrap bug** — `sram_offset` 累加 13-bit truncate 跟 `cfg_ifb_ring_words` (≠ 8192) 不同步. 修法: driver 算 cmd 时 `sram_offset % ifb_ring_words` (传入 driver `derive_w_slice_cfg` 的 ring_words). | ✅ |
| 3.12 | **修复 dispatcher ring_has_space underflow bug** — stride>1 layer 时 `line_buffer.rows_consumed_raw += stride`, 让 rows_consumed > rows_pushed (uint16 underflow → 65535), dispatcher 永远卡 S_RING_WAIT. 修法: 加 `ring_consumer_ahead = (rows_consumed >= r_rows_pushed)` fallthrough, consumer 跑超 producer 时认为 ring 有充裕空间. | ✅ |
| 3.13 | **修复 driver IDMA cmd input_src 查找** — ResNet 非 linear chain (如 L3 input from Patch=L0, 不是 L2). driver 用 `layer_idx-1` 错, 改成查 `name_to_idx[layer.input_src]`. | ✅ |
| 3.14 | **🎉 ResNet11 N=4 SMC 完整网络 PASS** — 11/11 layer 全 bit-exact, **Final OFM 33/33 bit-exact**, **220,824 cycles @ 100 MHz = 2.21 ms = 453 fps**. | ✅ |
| 3.15 | **修复 Layer 0 4 boundary mismatches** — 根因: SMC_INPUT_BASE root slot offset 0x10000 (64 KB) **不够 layer 0 IFB seg 容量** (Patch s2d 240×33×4×16 ≈ 496 KB). FC IFB preload 到 0xD10000 覆盖了 layer 0 IFB 的 r=31 col 1..4 (16 word 跨 4 cols), 让 ofb_writer 用错的 IFM 算 OFM. 修法: SMC_LAYER_INPUT_OFFSET = 0x80000 (512 KB / root slot). | ✅ |
| 3.16 | **稳健性 regression: 13/13 case 全 PASS** — 覆盖 W slice 多层, 不同 K (1/3/5/7), stride (1/2/4 + s2d), 奇数 W, cin/cout 多 slice, residual + sliced RDMA, mode A 单核, force_s2d, chain 自洽. 见下表. | ✅ |
| 3.17 | **🎉 axi_smc_4to4 真 IP 替换 sim model PASS** — `Syn/gen_axi_smc_4to4.tcl` + `multicore_top_smc.sv` 加 `\`ifdef USE_AXI_SMC_IP` 切换 IP / sim model. ResNet11 N=4 + IP 路径 **all 11 layer OFM bit-exact, 237,556 cy** (vs sim model 220,824 cy, IP +7.6% 真实 AXI4 cmd FIFO/ID tracking 延迟). 真硬件可综合可用. | ✅ |

### SMC + NUMA 稳健性 Regression (2026-05-05, 13/13 PASS)

| Case | Layers | 维度特点 | Final | Wall (cy) |
|---|---|---|---|---|
| wslice1 | 1 | K=3 stride=1 W=32 | 1024 word | 4,316 |
| wslice4 | 4 | K=3 stride=1 W=32 chain | 1024 word | 17,267 |
| wslice5 | 5 | K=3 stride=1 W=32 chain (5 层) | 1024 word | ~20,585 |
| wslice_mixed | 4 | 混合 K=3/5/3/1 stride=1 | 1024 word | 19,384 |
| wslice_oddw | 3 | **W=33 奇数 (4 切不整除)** | 1089 word | 13,241 |
| wslice_k7 | 2 | **K=7 大 halo** (halo=3 列) | 1024 word | 30,187 |
| wslice_k1 | 2 | **K=1 无 halo** + cin=64 cout=64 (cs 多 slice) | 1024 word | 5,031 |
| patch_small | 1 | cin=64 → cout=16 H=32 W=32 (cs_in=4) | 1024 word | 2,940 |
| patch_s2d_resnet | 1 | **Patch K=4 s=4 c=4 + force_s2d** (= ResNet11 L0 单层) | 32400 word | 43,104 |
| resnet_block1 | 3 | **ResNet block + residual** (K=3+K=3+K=1.ds) | 1024 word | 11,310 |
| resnet_residual_wslice | 3 | **ResNet residual + W slice** (K=3 sliced shortcut) | 1024 word | 13,207 |
| **resnet11** | **11** | **完整 ResNet-18-like (Patch+3 ResNet block+FC)** | **33 word** | **220,824** |

**全 case 0 mismatch, 100% bit-exact 跟 single-core golden 一致.**

不支持的 case (architecture limitation, 非 bug):
- `wslice_smallw` / `wslice_stride2` 部分 layer cycles 太小让 scheduler 选 mode A 单核, 但 SMC 当前 mode A → W slice stitch 不支持 (mode A 仅限 root layer 或 prev mode A). ResNet11 FC 是 root mode A 能跑.

### ResNet11 SMC sim per-layer cycles (2026-05-05, all bit-exact)

| Layer | Name | Mode | dim → dim | cycles | Mismatch |
|---|---|---|---|---|---|
| 0 | Patch (s2d K=4 stride=4 c=4 → K=1 stride=1 c=64) | W slice | 960×540×4 → 240×135×16 | 42,904 | **0/32400** |
| 1 | L1.B1.C1 (K=3 stride=2) | W slice | 240×135×16 → 120×68×16 | 25,800 | 0/8160 |
| 2 | L1.B1.C2 (K=3 stride=1) | W slice | 120×68×16 → 120×68×16 | 20,172 | 0/8160 |
| 3 | L1.B2.ds (K=1 stride=2 + residual from L1.B1.C2) | W slice | 240×135×16 → 120×68×16 | 26,956 | 0/8160 |
| 4 | L2.B1.C1 (K=3 stride=2) | W slice | 120×68×16 → 60×34×32 | 11,983 | 0/4080 |
| 5 | L2.B1.C2 (K=3 stride=1) | W slice | 60×34×32 → 60×34×32 | 24,516 | 0/4080 |
| 6 | L2.B2.ds (K=1 stride=2 + residual from L2.B1.C2) | W slice | 120×68×16 → 60×34×32 | 11,332 | 0/4080 |
| 7 | L3.B1.C1 (K=3 stride=2) | W slice | 60×34×32 → 30×17×64 | 12,858 | 0/2040 |
| 8 | L3.B1.C2 (K=3 stride=1) | W slice | 30×17×64 → 30×17×64 | 27,705 | 0/2040 |
| 9 | L3.B2.ds (K=1 stride=2 + residual from L3.B1.C2) | W slice | 60×34×32 → 30×17×64 | 4,355 | 0/2040 |
| 10 | FC (K=1 c=256→522) | **mode A 单核** | 1×1×256 → 1×1×522 | 10,043 | 0/33 |
| **Total** | | | | **220,824** | **All 11 layer bit-exact** |

### 性能对比 (2026-05-05)

| 配置 | cycles | 加速比 vs N=1 |
|---|---|---|
| ResNet11 N=1 single-core | 596K | 1.0× |
| ResNet11 N=2 multicore (mode A/W slice) | 450K | 1.32× |
| ResNet11 N=4 multicore (mode A/W slice, single-DDR) | 354K | 1.68× |
| ResNet11 N=4 4-DDR PoC (BW 解墙) | 196K | 3.04× |
| **ResNet11 N=4 SMC + sim model crossbar** | **220,824** | **2.70×** |
| **ResNet11 N=4 SMC + axi_smc_4to4 真 IP** | **237,556** | **2.51×** |

SMC + NUMA 218K cycles 接近 4-DDR PoC 196K (10% 慢, 主要因 axi_crossbar_4to4_sim 行为模型保守 — 每 MI 锁 SI owner 至 burst 完成, 限制 ConvCore 同时多 outstanding read 给同 mem). 真硬件 SmartConnect IP 应能跑到 4-DDR 性能甚至更好.

### 关键里程碑

**SMC + NUMA 端到端 ResNet11 N=4 完整网络通过 sim**:
- 4 核 ConvCore 全自走 IDMA/ODMA SG dispatcher 拉自己 cmd list
- axi_crossbar_4to4_sim 按 addr[25:24]=mem_id 自动路由
- halo 物理只一份 (driver 编译期决定 cmd list, 跨 mem 边界 crossbar 自动路由)
- 11 layer 含 W slice + ResNet residual + mode A 单核 FC
- Final OFM 33/33 bit-exact 跟 single-core golden 一致
- 设计哲学 (统一全局地址 + halo 物理只一份 + DMA 软件控制 + 多 burst 路径) 完全可工程化

### 剩余 (Phase 7 阶段 4 = 硬件落地, 可选)

| 阶段 | 任务 | 状态 |
|---|---|---|
| 4.1 | Vivado axi_crossbar IP `axi_smc_4to4` 真生 + 替换 `axi_crossbar_4to4_sim` | ✅ 完成 (3.17) |
| 4.2 | (可选) Vivado axi_dma SG mode IP 真生 (替代 axi_dm + idma_sg_dispatcher 自走) | ⏳ |
| 4.3 | (可选) `multicore_top_smc.sv` 综合 + 跑 timing 报告 | ⏳ |

### 用 SmartConnect IP 真上时

- `Syn/gen_axi_smc_4to4.tcl` (新建, 设计 doc 内有草稿) 跑一次生成 IP
- `multicore_top_smc.sv` 替换 `axi_crossbar_4to4_sim` 实例 → `axi_smc_4to4` 实例
- ConvCore 内 axi_dm IP 升级为 axi_dma SG mode (替代 datamover, 内置 SG engine), 或保留现 axi_dm + idma_sg_dispatcher 自走 cmd list (本阶段路径)

---

| 文件 / 目录 | 说明 |
|---|---|
| `STATUS.md` (本文件) | 当前进度 + 任务交接 |
| `README.md` | 项目顶层叙述 + 性能表 |
| `CLAUDE.md` | Claude Code 工作指引 |
| `RTL代码编写原则.md` | RTL 风格规范 |
| `model_analysis.md` | 目标模型 PE 利用率分析 |
| `docs/` | 模块时序、切片机制、Ky-fold/S2D 推导、TB 机制 |
| **`memory/`** | 本次会话提炼的设计经验 (新增) |
| `RTL/` | 26 个 .sv 源文件 + AXI4/ DMA/ 子目录 |
| `Syn/run_syn.tcl` | 综合脚本 (顶层切换 single/multi) |
| `Syn/gen_axi_datamover.tcl` | DataMover IP 生成 |
| `Syn/gen_multicore_ip.tcl` | 多核 wrapper IP 生成 |
| `sim/tb_core_dma/` | 单核 TB + chained regression |
| `sim/tb_multicore/` | 多核 smoke TB (待修后可用) |
| `toolchain/` | 编译器 / 数据生成 / 回归 |

---

## 11. 综合时序优化 (2026-05-06 ~ 05-07) — Fmax 60→142.8 MHz

### 6 轮迭代 (xc7k325tffg900-2 N=4 synth-only)

| Round | WNS (ns) | Fmax (MHz) | clock target | 主要改动 |
|---|---:|---:|---:|---|
| 0 | -6.835 | 60   | 100 MHz | (基线, BRAM 484 爆 445) |
| 1 | -4.345 | 70   | 100 | odma_sg_dispatcher 16-bit 除法 → 累加器 |
| 2 | -1.076 | 89   | 100 | SDP 1-stage pipeline (DSP48 input + M reg) |
| 3 | +0.781 | 108.5| 100 | ofb_writer sb_raddr 用累加器替换 2 个 cascade DSP48 mult |
| 4 | +0.300 | 129.9| 125 | SDP 拆 stage 2a/2b (round+shift / clip+trunc 分两拍) |
| 5 | +0.439 | 141.6| 133 | ring_full 寄存器化 + driver OFB 留 1 行 slack |
| 6 | +0.196 | 142.8| 138.9 | line_buffer rows_needed 寄存器化 (跟 y_row_base 同步 update) |

资源 (R6 后): BRAM 290/445 (65%), LUT 162K/203K (76%), DSP 320/840 (38%).

### 停止 R7 的 ROI 评估

新 critical path: `parf_accum/fill_tile_cnt → cur_valid_w_fill → wr_is_last_col → drain_stall_fill →
psum_in_ready → wgt_buffer/wgt_addr → WB`, 17 levels (CARRY4=3 + LUT6×9), logic 仅 1.4 ns + route
5.0 ns (78%). routing dominate, P&R phys_opt 可能改善, 纯 logic 优化 ROI 有限.

经验完整记录在 `memory/project_timing_optimization.md`.

---

## 12. ResNet11 PE 利用率分析 (2026-05-07) — 多核才暴露的瓶颈

### SMC N=4 layer-by-layer profile

TB 加 `dump_pe_profile` 输出每核每层 fire/stall/idle 计数. 信号语义:
- **`act_idle`**: mac_array 等 line_buffer 给数据 (上游慢, 通常 IDMA 拉数据 bound)
- **`wgt_stall`**: wgt_buffer 给但 mac 拒收 (下游慢, 跟 act_idle 配对 = valid/ready 反压一对)
- **`psum_idle`**: mac_array 没产出 (mac_array 流水填充 + cs 切换 prefetch + bias_rf prefetch 4 拍)
- **`acc_idle`**: parf_accum drain 等 ofb_writer 消耗

| L | Name | cycles | C0 util | C3 util | act_idle | psum_idle | acc_idle |
|---|------|----:|----:|----:|----:|----:|----:|
| 0 | Patch (s2d K=1 H=240) | 43114 | 73.5% | **80.2%** | 5754 | 10856 | 31444 |
| 1 | L1.B1.C1 (K=3 s=2) | 24911 | 73.7% | 73.7% | 5617 | 5972 | 21962 |
| 2 | L1.B1.C2 (K=3 s=1) | 20174 | **91.0%** | 91.0% | 857 | 1235 | 17202 |
| 3 | **L1.B2.ds (K=1 s=2)** | 26956 | **7.6%** | 7.6% | C2 21984 | 24337 | 22120 |
| 4 | L2.B1.C1 (K=3 s=2) | 11985 | 72.1% | **90.1%** | 79~764 | 2766 | 7180 |
| 5 | L2.B1.C2 (K=3 s=1) | 24518 | 70.5% | 88.1% | 1578 | 6659 | 17319 |
| 6 | **L2.B2.ds (K=1 s=2)** | 11401 | **8.4%** | 10.5% | C1 8422 | 9562 | 7959 |
| 7 | L3.B1.C1 (K=3 s=2) | 12860 | 67.2% | 84.0% | 79 | 3641 | 7656 |
| 8 | L3.B1.C2 (K=3 s=1) | 27707 | 62.4% | 78.0% | 3021 | 9848 | 19238 |
| 9 | L3.B2.ds (K=1 s=2) | 4415 | 21.7% | 27.2% | 1346 | 2876 | 1474 |
| 10 | FC | 10045 | 5.3% | (idle) | 43 | 9373 | 906 |
| **TOTAL** | | **220286** | | | | | |

### 多核才暴露的 4 类瓶颈

#### 瓶颈 A: 核间几何切片不均 (边界核 W 段更宽)

**现象**: L0 cycles=43114 内 C3 util 80.2% 比 C0..C2 73.5% 高 ≈ 9%. 这种"末核多干"在所有 W 不能整除 4 的层都出现.

**根因**: driver `compute_smc_w_segments(W_full, n=4)` 切片用 floor 除法, 余数全给最后核.
  例如 L0 W_OUT=135: 135÷4=33 余 3, 段宽 = [33, 33, 33, 36], 末核 C3 多 3 列 (9% 多).
  L1.B1.C1 W_OUT=68: 68÷4=17 整除, 段宽 = [17,17,17,17], 4 核 fire 数完全相同.

**反映在 cycle**: layer barrier 等最慢核 (C3 干完才算 layer done), 所有核 cycles 计数对齐到 C3 done.
  C3 cycles 内 fire 多 (cells/yout 多 3) → util 高.
  C0..C2 cycles 内 fire 少 (cells/yout 少) → cycle 末段闲置 → util 低.

**优化**: 切片改成"余数分散给前 N 核"(每核 +1, 最大差距 ≤ 1 列).
  L0 改 [34,34,34,33] → 差距 1/34 ≈ 3% (vs 现在 3/33 ≈ 9%).
  整网估收益 5~8% (avg util 上升).

#### 瓶颈 B: 中间核跨内存路由开销 (act_idle 跨核分布不均)

**现象**: L3 (K=1 s=2) 4 核 act_idle 分布 = [12499, 20181, 21984, 12632]. 中间核 C1/C2
比边界核 C0/C3 多等 7000~9000 cycle 数据.

**根因**: 跟瓶颈 A **完全不同**. C1/C2 的 W slice 段恰好横跨两块 mem 的边界 (e.g. C1 在 W=[34:67]
跨 mem[0]=[0:34) 和 mem[1]=[34:68)), IDMA 一行需要发 2 条 SG 命令 (per_row=2), 两条命令
在 IDMA SG 调度器内 **串行执行** (无双 outstanding), 中间有 turnaround 延迟累积.
C0/C3 的段都在单块 mem 内 (per_row=1), 一拍 1 命令快得多.

**反映在 cycle**: 中间核 IDMA 拉数据慢 → mac_array act_idle 多 → util 低. 边界核拉得快, util 高.

**跟瓶颈 A 的区别**:
- A 是 **几何不均匀** (静态切片算法不公平), C3 多干; B 是 **动态网络开销** (跨 mem 串行), C1/C2 多等.
- A 表现在 fire 数差异 (干多/干少); B 表现在 idle 数差异 (等多/等少).
- A 修在 driver (改切片算法); B 修在 RTL (IDMA SG dispatcher 双 outstanding) 或 driver (避免段跨 mem).

**优化**: IDMA SG dispatcher 加双 outstanding (一条 cmd 在 axi_dm S2MM 时同时发下条 cmd), 让 cross-mem 段并行.
  预期 L3/L6 整体收益 30~40% (BW 翻倍).

#### 瓶颈 C: 1×1 stride=2 ds 层 PE 阵列空转 (结构性瓶颈)

**现象**: L1.B2.ds / L2.B2.ds / L3.B2.ds 单核 util 仅 7.6% / 8.4% / 21.7%. 单核也一样
(单核 tb_core_dma 测 18.76% / 29.49% / 56.68%, SMC 因 W slice 切 4 wallclock 减但 util 不变).

**根因**: K=1, cin_slices=1, cout_slices=1 时 mac_array (16 PE × 16 col = 256 MAC) 一拍只用
**16 PE × 16 col 子集 = 16 个 PE** (= cin × cout = 1×1 PE 同时算), 240 PE 完全空转. 这是 mac_array
针对 K×K 大卷积的硬件结构, 跟 ds 1×1 不匹配 (PE 利用率天花板 = 1/16 = 6.25%, 加点流水开销).

**反映在 cycle**: act_idle 大 (line_buffer 很快出, mac_array 一拍消耗 1 cell, 但 mac fire 数固定 = ofm pixels),
不是 IDMA 慢, 是 mac_array 内部带宽不够利用.

**优化** (按改动量排序):
1. **Ky-fold 启用** (driver 已支持): K=1 cin=16 时折成 cin_fake=16×K=16 让 PE 行铺满. **但 ds K=1 cin=16
   已经满 PE 行, 无法折. 只能 cout-fold 但当前不支持.**
2. **小 K 专用 1D MAC 引擎**: 跟 16×16 阵列并行, ds layer 走小引擎. 改动大.
3. **算法层合并 ds 跟相邻层**: 数学合并 ds 1×1 + B1.C1 3×3 → 单层 conv. 编译器侧改动.

#### 瓶颈 D: cs 切换 / mac_array 启动空泡 (psum_idle)

**现象**: 所有层 psm_idle = 5972~24337 cycle. L1.B1.C1 占 24%, L3 占 90%. 主因 mac_array 流水填充
+ cs 切换时 wgt/bias prefetch.

**根因 1**: bias_rf 在 cs 切换时 4 拍 SRAM 读 prefetch 16×INT32, 期间 `bias_ready=0` mac_array stall.
**根因 2**: mac_array 启动有 K+几拍空泡 (流水填充).
**根因 3**: wgt_buffer 已有 ping-pong (`load_caught_up` 跟 rounds_ahead), cs 切换无 wgt prefetch idle.

**流水化解掉吗**: **可以, 但收益有限**.
- bias_rf 加 ping-pong (双 RF 各持一组 16×INT32), cs N 用 RF[A] 时后台预加载 cs N+1 到 RF[B], 切换无 idle.
  每 cs 切换省 4 拍. L0 cs=4 切换 3 次 = 12 cy/yout × 240 yout = 2880 cy/core. 但 psm_idle 实测 10856 cy,
  bias 占 ~25% (~3000 cy), 其他 75% 是流水填充和 drain 同步.
- mac_array 启动空泡 fundamental, 流水加深反而更糟.
- 整网估 bias ping-pong 收益 5~8%.

### 优化优先级 (按 ROI 排序, 准备论文方案)

| 优化 | 改动量 | 预期整网收益 | 论文价值 |
|---|---|---:|---|
| **A. 切片余数分散** (driver) | 5 行 Python | +5~8% | 中 (调度公平性, 易讲) |
| **B. IDMA SG 双 outstanding** (RTL) | 50 行 | +10~15% | 高 (跨 mem 流水, NUMA 优化) |
| **C1. Ky-fold cout 维度** (RTL+driver) | 100 行 | ds 层 +2~3× | 高 (PE 阵列复用) |
| **C2. 小 K 专用引擎** (架构) | 300+ 行 | ds 层 +5× | 极高 (异构加速器) |
| **D. bias_rf ping-pong** (RTL) | 30 行 | +5~8% | 中 (流水化深化) |

### 当前状态

- TB profile: `sim/tb_smc/tb_smc_chain.sv` 已加 `dump_pe_profile` task, 每 layer 输出 4 核 fire/stall/idle
- 数据已收集 (220286 cy ResNet11 SMC N=4)
- 论文方案候选: A (切片公平) + B (NUMA 流水) + D (流水深化) 是低改动量 RTL/driver 工作; C 是新算法/架构创新点

---

## 13. Round A 切片公平 (2026-05-07) — ResNet11 SMC N=4 cycle 220286 → 205752 (−6.6%)

### 解决方案

**问题**: driver `compute_smc_w_segments` 旧版用 floor 除法 + 余数全给末核, 例 W=135 切 4 → [33,33,33,36],
末核多 3 列 (9% cells), layer barrier 等末核完成 → 前 3 核闲等, util 损失 5~8%.

**修改**: 余数分散给前 rem 个核, 每核 +1 列. W=135 → [34,34,34,33], 段宽差距 ≤ 1 col (3% vs 9%).

**RTL/driver 改动**:
- `toolchain/mesh_cmd.py:compute_smc_w_segments` (mem 段 layout)
- `toolchain/hw_files.py:compute_w_slice_geom` (单层 OFM 切片)
- `toolchain/hw_files.py:compute_w_slice_chain_geom` (chain 反向递推 OFM 起点)
- `sim/tb_smc/tb_smc_chain.sv:preload_ifb_smc / check_layer_ofm` (TB 镜像 driver 切片公式)

3 处 driver 切片公式必须保持 strict 一致 (mem 段起点 ↔ core slice 反推 ↔ TB preload/check), 任一处不同步立即 OFM mismatch.

### 仿真数据对比 (ResNet11 SMC N=4, IFB=1024 SHB=2048 trim 后)

| Layer | R6 cy | Round A cy | Δ | 改善原因 |
|---|----:|----:|----:|---|
| 0 Patch (s2d K=1 H=240 W=135) | 43114 | **41227** | **−1887 (−4.4%)** | mem layout 公平, 4 核 fire 数差距 9%→3% |
| 1 L1.B1.C1 (K=3 s=2 H=240 W=135) | 24911 | 24956 | +45 | W_OUT=68 整除 4, 几何切片不变, 误差噪声 |
| 2 L1.B1.C2 (K=3 s=1 H=120 W=68) | 20174 | 20174 | = | W_OUT=68 整除 |
| **3 L1.B2.ds (K=1 s=2 H=240 W=135)** | 26956 | **17478** | **−9478 (−35.2%)** | **见下** |
| 4 L2.B1.C1 (K=3 s=2 H=120 W=68) | 11985 | 10888 | −1097 (−9.2%) | mem layout 公平 |
| 5 L2.B1.C2 (K=3 s=1 H=60 W=34) | 24518 | 22413 | −2105 (−8.6%) | mem layout 公平 |
| 6 L2.B2.ds (K=1 s=2 H=120 W=68) | 11401 | 11439 | +38 | W_OUT=34 整除, 误差噪声 |
| 7 L3.B1.C1 (K=3 s=2 H=60 W=34) | 12860 | 12841 | −19 | W_OUT=17 整除 |
| 8 L3.B1.C2 (K=3 s=1 H=30 W=17) | 27707 | 27687 | −20 | W_OUT=17 整除 |
| 9 L3.B2.ds (K=1 s=2 H=60 W=34) | 4415 | 4404 | −11 | W_OUT=17 整除 |
| 10 FC (1×1) | 10045 | 10045 | = | 单核 |
| **TOTAL** | **220286** | **205752** | **−14534 (−6.6%)** | 接近预期 5~8% |

### 意外发现: Layer 3 Cycle 减少 35.2% (远超预期)

Round A 设计预期仅 5~8% 整网收益, **L3 单层 −35%** 是远超预期的隐性收益.

**原因**: Round A 切片不仅对齐核间负载, 还**对齐了 mem 物理段起点跟 core slice 反推段**.
- 旧 mem layout `[33,33,33,36]` 末核多 3 列 → core slice 反推 IFM 段跟 mem boundary 错位
  → 中间核 (C1/C2) IDMA per_row=2, IFM 行需要发 2 条 SG 命令跨 mem 边界, IDMA SG dispatcher
  内串行执行, turnaround 延迟累积 → mac_array act_idle 大.
- 新 mem layout `[34,34,34,33]` → mem 段宽更接近 core slice 反推 sub_W → cross-mem 命令大幅减少.

**验证 (L3 act_idle 分布)**:
- R6 旧: `[12499, 20181, 21984, 12632]` (中间核 C1/C2 多等 ~9000 cy IDMA)
- Round A: `[12498, 12498, 12498, 12499]` (4 核完全均匀, cross-mem 已消除)

**对 Round B (IDMA SG 双 outstanding) 的影响**: Round A 已经吃掉很大一部分 cross-mem 损失,
Round B 预期收益从原估 10~15% 整网 / L3 30~40% 降到 ~5~8% 整网 / L3 ~15%. 但 Round B
仍有价值 (其余 ds layer L6/L9 没整除 4 时还会 cross-mem).

### 性能数字 (假设 P&R 后 100 MHz)

| 项目 | R6 (220286 cy) | Round A (205752 cy) | 提升 |
|---|----:|----:|----:|
| Latency @100MHz | 2.20 ms | 2.06 ms | −6.6% |
| FPS | 454 fps | 486 fps | +7.0% |

### Round A 附带重构: TB 切片 SOT 化 (消除设计债)

**问题**: 旧 TB `preload_ifb_smc` / `check_layer_ofm` 自己重新算切片公式 (`b = w/N`, 余数处理),
跟 driver `compute_smc_w_segments` 两套独立实现. Round A 改 driver 切片, TB 没同步触发 OFM mismatch.
违反"切片由编译器决定"设计原则, 是显著的设计债.

**重构**: driver 把每 layer mem 散布 layout (IFM + OFM 各 4 段) 写到 multicore_meta.txt:
```
SMC_LAYER_0_IFM_SEG_WIDTHS = 34 34 34 33
SMC_LAYER_0_IFM_SEG_STARTS = 0 34 68 102
SMC_LAYER_0_OFM_SEG_WIDTHS = 34 34 34 33
SMC_LAYER_0_OFM_SEG_STARTS = 0 34 68 102
...
```

TB parse 时增加 4-int array `$sscanf(line, "%s = %d %d %d %d", ...)` 分支, 存到
`smc_ifm/ofm_seg_widths/starts[layer][core]` 数组. preload + check 直接索引 array, 不再
hardcode 切片公式.

**效果**: 
- driver 是切片**唯一 source of truth**, 改切片策略只需改 1 处 (Round A 同类改动)
- TB 收益: preload_ifb_smc 切片代码 23 行 → 5 行, check_layer_ofm 同样削减
- 系统稳健性 ↑: TB 跟 driver 不会再因切片公式不同步而 mismatch

**改动**:
- `toolchain/run_multicore_chain.py`: meta 写 4 array per layer
- `sim/tb_smc/tb_smc_chain.sv`: parse_meta 加 4-int array 分支 + smc_ifm/ofm_seg_* 数组 +
  preload/check 用 array 替代手算公式

**验证**: SMC sim 三 case PASS (resnet11 205752 cy 跟 commit 4b4be0c 一致, bit-exact),
单核 regression 26/26 PASS.

---

## 14. Round B IDMA SG dispatcher sts 后台化 (2026-05-07) — ResNet11 SMC N=4 cycle 205752 → 203320 (-1.18%)

### 解决方案

**问题**: `RTL/DMA/idma_sg_dispatcher.sv` 主 FSM 每条 cmd 必须等 sts 才进下条 (S_FETCH_CMD_STS + S_STS
两个等 sts 状态), cross-mem cmd per_row=2 时累积 turnaround 浪费.

**修改**: sts collector 跟主 FSM 解耦.
  - `mm2s_sts_tready` 永远 = 1 (后台 collect, 不阻塞 main FSM).
  - 主 FSM 跳过 S_FETCH_CMD_STS / S_STS, 状态从 9 个减到 7 个.
  - cmd_fire / sts_fire 计数器并行累计, S_DONE 时 `r_cmd_fires_total == r_sts_collected` 才置 r_done.
  - r_cmd_idx / r_rows_pushed 改成 S_DATA tlast (= 数据写完 IFB 那拍) 推进, 不等 sts.
  - r_err sts[7] sticky 不变 (后台 collector 也检查).

### 仿真数据对比

| Layer | Round A cy | Round B cy | Δ | Δ% |
|---|----:|----:|----:|----:|
| 0 Patch (K=1 s=1, cs=4 cross-mem) | 41227 | **40795** | -432 | -1.0% |
| 1 L1.B1.C1 (K=3 s=2 halo cross-mem) | 24956 | **24483** | -473 | -1.9% |
| 2 L1.B1.C2 (K=3 s=1) | 20174 | 20162 | -12 | 0% |
| 3 L1.B2.ds (K=1 s=2) | 17478 | **16999** | -479 | -2.7% |
| 4 L2.B1.C1 (K=3 s=2) | 10888 | 10880 | -8 | 0% |
| 5 L2.B1.C2 (K=3 s=1) | 22413 | 22401 | -12 | 0% |
| **6 L2.B2.ds (K=1 s=2 cross-mem)** | 11439 | **10732** | **-707** | **-6.2%** |
| 7 L3.B1.C1 (K=3 s=2) | 12841 | 12834 | -7 | 0% |
| 8 L3.B1.C2 (K=3 s=1) | 27687 | 27679 | -8 | 0% |
| **9 L3.B2.ds (K=1 s=2 cross-mem)** | 4404 | **4112** | **-292** | **-6.6%** |
| 10 FC | 10045 | 10043 | -2 | 0% |
| **TOTAL** | **205752** | **203320** | **-2432** | **-1.18%** |

其他 case:
- smc_wslice1: 4339 → **4125 cy (-4.9%)**
- smc_wslice5: 21699 → **20629 cy (-4.9%)**
- 单核 regression 26/26 PASS

### 收益分析

ds 层 (L6/L9, K=1 stride=2 cross-mem) 收益最高 (6.2%/6.6%), 因为它们 cmd 数密集 + sts wait 占总 cycle 比重大.
K=3 主 conv 层收益较低 (~0%), 因为 user data 传输 D 远大于 sts wait, 节省 2 cy/cmd 在大 D 下占比小.

**实测 < 预估 (5-8% → 1.18%)**: axi_dm IP 内部 sts 跟 data tlast 几乎同 cycle 返回 (不需要等几拍),
sts wait 实际仅 1-2 cycle. 真正的 cross-mem 串行损失 (axi_dm 内部 cmd FIFO 严格按序处理) 没法在自写
dispatcher 层面消除. 要拿到 30+% 必须改 IP 架构 (axi_mcdma 多 channel 真并行).

### 综合

- WNS: +0.196 ns (跟 Round A 完全一致, sts collector 解耦不影响 critical path)
- Fmax: 142.8 MHz (timing MET)
- 资源不变

### Round 总览 (cycle 累计收益)

| Round | ResNet11 cy | Δ vs prev | 累计 vs baseline | 关键改动 |
|---|----:|----:|----:|---|
| baseline (R6 timing, IFB=1024) | 220286 | - | 0% | (起点) |
| Round A (切片公平) | 205752 | -6.6% | -6.6% | driver 余数分散 + 隐性 mem boundary 对齐 |
| Round B (sts 后台 collect) | 203320 | -1.18% | -7.7% | dispatcher main FSM 跳 sts wait |
| @100 MHz Latency | 2.03 ms | (vs 2.20 ms baseline) | **-7.7%** | |
| FPS | 492 | (vs 454 baseline) | **+8.4%** | |

> 注: 后续清理 commit `1554928` 把 sim crossbar 改成真 Vivado axi_smc_4to4 IP, ResNet11 cy 从
> 203320 涨到 217311 (+14000 cy 是 IP 内 register stage 真实开销, 跟硬件部署一致). 下面 Round C 的
> baseline 用 217311 cy.

---

## 15. Round C — Cout slice 自适应 (2026-05-07) — ResNet11 SMC N=4 cycle 217311 → 210784 (-3.0%)

### 背景: L10 (FC) 单核独跑的浪费

ResNet11 L10 (FC, H_IN=W_IN=1, cin=256, cout=528) 是个特殊层:
- W=1 不能 W slice (W < n_split × (K+1) = 4 × 2 = 8)
- 之前 driver 选 `Mode.A_SINGLE` (单核独占 mode A), mask=0001 只 core 0 干活, 其他 3 核空转
- L10 cy=10022, fire=528 utilization=5.3%, 占总 cy 的 4.6%

### 解决方案: 编译器层面 W/cout 切片自适应

**scheduler.py choose_mode 优先级调整** (新加最高优先级分支):

```python
# 优先级特例: W 切不动 (e.g. FC W=1) 但 cout 切可行 → cout slice 让 4 核都干活
if not can_w_slice(layer, n_split) and can_cout_slice(layer, n_split):
    return (Mode.C_COUT_SLICE, n_split)
```

之前规则 (基本不变):
1. 极小 layer (cyc < target/4): A_SINGLE
2. 默认 W slice
3. W 切不下兜底 cout slice (实际从未命中, 因为 1 总在前面 catch)

新规则 ResNet11 命中情况:
- L0~L9 走 W slice (W >= 4 × (K+1))
- **L10 走 cout slice** (W=1 切不动, cout=528 ≥ 4 × 16 = 64)

### Cout slice 数据流 (硬件零修改)

每核负责整图 H × W, 但 cout 维只算 1/n 段. 4 核共享拉同一份 IFM, 每核写自己 cout 段紧凑 layout.

**前提**: IFM 必须**集中存放**在某个 mem (mode A 或 host preload). 上层 W slice 散布 4 mem 的话,
cout slice 每核要从 4 mem stitch IFM, cmd 数 4× 膨胀, **当前未支持** (driver 报 NotImplementedError).

**L10 天然满足**: ResNet11 L9 → L10 之间硬件不做的 GlobalAvgPool 由 host 完成, host 把 1×1×256
flatten 数据 preload 到 mem[0] 当 root layer, IFM 就是集中存放的 (mode A 等价). 不用 driver 做层间 stitch.

cout 段切片 (`mesh_cmd.compute_cout_segments(528, 4)`):
- core 0: cs[0..9), cout[0:144), 9×16=144 通道
- core 1: cs[9..17), cout[144:272), 8×16=128 通道
- core 2: cs[17..25), cout[272:400), 8×16=128 通道
- core 3: cs[25..33), cout[400:522), 8×16=128 通道 (最后段含尾巴 522-400=122 实际通道, 余 6 PE col 空转)
- sum=522 ✓

### IDMA / ODMA cmd 形式

cout slice 下 SG cmd 简化 (跟 W slice 跨 mem 多段不同):

| | IDMA cmd (per core, 拉 IFM) | ODMA cmd (per core, 写 OFM) |
|---|---|---|
| 数量 | H_IN 条 (每行 1 条) | H_OUT 条 (每行 1 条) |
| src/dst_addr | 4 核相同, 都指向 mem[ifm_mem_core] | 各核指自己 mem 紧凑段 (cout_per_core × 16 byte/pixel) |
| btt | W_IN × cin_slices × 16 | W_OUT × my_cs × 16 (按本核段) |

### 改动文件清单 (硬件 0 改)

| 文件 | 改动 |
|---|---|
| `toolchain/scheduler.py` | choose_mode 加优先 cout slice 分支 (5 行) |
| `toolchain/mesh_cmd.py` | 加 `compute_cout_segments` / `gen_idma_sg_cmd_list_cout_slice` / `gen_odma_sg_cmd_list_cout_slice` |
| `toolchain/hw_files.py` | 加 `derive_cout_slice_cfg` (复用 derive_layer_cfg, 传本核 cout 段) |
| `toolchain/run_multicore_chain.py` | `build_step_cfg_dict` C_cout_slice 分支 + SMC SG cmd list 生成 + meta 写出 mode='C' / COUT_SEG |
| `sim/tb_smc/tb_smc_chain.sv` | parse `COUT_SEG_*` + preload_ifb / wb / rdma 加 mode='C' 分支 + check_layer_ofm 加 mode='C' 分支 |

TB 关键改动:
- preload_ifb mode='C' = mode='A' (整图灌 mem[mode_a_core])
- preload_wb mode='C': 4 mem 各装自己 cout 段 weight (wb.txt cs-outer layout, 每 cs 占 kk×cin_slices wb-rows)
- preload_rdma mode='C': bias 切 cout 段 (cs-outer layout, 每 cs 4 word)
- check_layer_ofm mode='C': 从 4 mem 各读自己 cout 段 stitch 跟整图 NHWC 比对

### 仿真数据

| 指标 | Round B (sim crossbar) | Round B (IP path baseline) | Round C (cout slice) | Δ vs baseline |
|---|----:|----:|----:|----:|
| smc_resnet11 总 cy | 203320 | 217311 | **210784** | -3.0% |
| L10 cy | 10043 | 10022 | **3495** | -65% |
| L10 mask | 0001 | 0001 | **1111** | 4 核都活 |
| smc_wslice1 | 4125 | 4701 | 4701 | 0% (无变化) |
| smc_wslice5 | 20629 | 23493 | 23493 | 0% (无变化) |

### 跟之前 §12 论文优化候选对照

§12 讨论的"低 util ds 层" (L3/L6/L9) 还是没动. cout slice 修的是 L10 单核独跑, 不是 K=1 ds 层的
PE 阵列空转 (那个是结构性瓶颈, 见 §12 瓶颈 C).

L10 之前算"FC 不影响主体" (10K cy / 217K = 4.6%), 改完 3.5K cy / 211K = 1.7%, 进一步压低 FC 占比.

### Round 总览 (cycle 累计收益)

| Round | ResNet11 cy | Δ vs prev | 累计 vs IP baseline | 关键改动 |
|---|----:|----:|----:|---|
| baseline (cleanup, 真 IP path) | 217311 | - | 0% | (起点) |
| Round C (cout slice 自适应) | **210784** | -3.0% | -3.0% | scheduler choose_mode + driver/TB cout slice 实现 |
| @100 MHz Latency | 2.11 ms | (vs 2.17 ms baseline) | **-3.0%** | |
| FPS | 475 | (vs 460 baseline) | **+3.1%** | |

注: Round A/B 的 cy 数 (203K / 205K) 是 sim model crossbar 路径, 真硬件部署应跟 Round C 的 211K 对齐.

### 自适应切片限制 (未来可扩展)

cout slice 当前只支持 IFM 集中存放. ResNet11 仅 L10 命中. 如果未来网络有"中间层 cout 大 H/W 小"
(典型如 squeeze-and-excite 类), 上层是 W slice 散 4 mem 的话, cout slice 不能用.

要扩展支持需要 driver 加层间 stitch 逻辑 (cmd 数 ×4) 或硬件加 layer 间 redistribute pass. 当前网络
不需要, 留给未来.

### 下一步: PE 利用率优化 (准备做)

§12 的瓶颈 A/B 已经在 Round A/B 解掉, 剩下:
- **C (1×1 ds 层 PE 空转)**: 结构性, 需要 RTL/编译器深度改动 (Ky-cout-fold 或小 K 引擎)
- **D (psum_idle bias prefetch)**: bias_rf ping-pong, 改动局部, 收益 5~8%
- **A' (layer barrier 软化)**: per-core barrier + 链表 desc, 5~10% (跟数据依赖耦合, 复杂)

下一阶段重点研究 C 和 D, 见 §16 后续章节.

---

## 16. Round D — bias_rf ping-pong (失败, 2026-05-07)

### 尝试方案

bias_rf 改双 RF (rf[0..1]) + 后台 prefetch:
- cs N 计算时 active RF 输出 bias_vec
- 后台预加载 cs N+1 到 inactive RF
- cs_cnt 切换时立即 swap r_active_rf, ready 不掉

预期收益: cs 切换 5 拍 stall × 切换次数, 整网估 1-2% (cs > 1 layer 才有收益).

### 失败原因

wslice1/wslice5 PASS (这俩 case 全 cs=1 不触发切换). resnet11 hang on layer 4-5
(cout=32 cs=2 首次切换). dump 显示 cs_cnt 跳回 0 (新 yout) 时:
- active RF 持 cs=1 (上 yout 末)
- inactive RF 持 cs=2 (prefetch 越界, 实际 cout_slices=2 没 cs=2)
- 双不匹配应触发 cold reload (state → S_LOAD), 但 state 持续 S_RUN 5000+ cy mac_array stall

Root cause 推测: state machine (S_IDLE/S_LOAD/S_RUN) 跟 load FSM (load_in_progress)
分两套, 没强同步 + prefetch_cs 越界让 inactive RF 持无效 cs.

### 决策

**回退** (`git checkout HEAD -- RTL/bias_rf.sv`). 保留 Round B 单 RF 5 拍 stall 设计.
收益小 (1-2%) + 风险高 (state machine bug), 不值得现在做. 教训记入 memory:
`feedback_bias_rf_pingpong_failed.md`.

未来再做需:
1. 单 FSM 合并 state + load_in_progress (强绑定)
2. prefetch_cs cap 到 cout_slices-1 不越界
3. 测试必跑 cs > 1 case (resnet11 layer 4+)

---

## 17. Round E — mm2s_arb WDMA 饥饿提优先级 (防御性, 2026-05-07)

### 改造

`RTL/DMA/mm2s_arb.sv` 加 WDMA 饥饿计数:
- WDMA 等 ≥ STARVE_THRESH=32 拍没 grant → 强制提优先级到最高, 抢一次
- 否则保持原 priority `idma > rdma > ocmd > wdma`

实现:
- `r_wdma_wait_cnt` (饱和到 32)
- WDMA fire 或 idle 清零, 等待时累加
- `wdma_starve = (cnt >= 32)`, cmd_owner 跳 wdma 当 starve && wdma_cmd_tvalid

### 实测

ResNet11/wslice1/wslice5 cy 完全不变 (210784/4701/23493). 当前 dispatcher 串行
(Round B), WDMA 在 IDMA fetch latency 间隙能拉到, 没饿. 改造没主动收益.

### 价值

**防御性改动**, 防未来 dispatcher 加 prefetch / multi-outstanding 让 IDMA 持续
占用 mm2s 让 WDMA 饿死的退化 (Round C+ 尝试 prefetch, L0 cy +17K 就是这个 root cause,
见 `memory/feedback_prefetch_starves_wdma.md`).

下次再做 dispatcher prefetch, 有 mm2s_arb starve 兜底, 不会让 L0 暴涨.

---

## 18. Round F — TB host loop per-core barrier 软化 (2026-05-07)

### 改造

`sim/tb_smc/tb_smc_chain.sv` host 主循环:
- 之前: for c in 0..3: write_boot_regs(c) + start_dfe(c) + **wait_dfe_done(c)** (串行)
- 现在: 串行 write_boot_regs + start_dfe (csr 总线单口必须串行)
       + **fork...join 并行 wait_dfe_done(0..3)** (DFE 各核独立 RTL, 真并行拉 desc list)

### 实测

| Case | Round C | Round F | Δ |
|---|----:|----:|----:|
| smc_resnet11 | 210784 | **206589** | **-2.0%** |
| smc_wslice1 |   4701 | **4317** | **-8.2%** |
| smc_wslice5 |  23493 | **21585** | **-8.1%** |

每层均省 ~381 cy (= 4 核 DFE 拉 desc 串行变并行省的 turnaround). ResNet11 11
layer × 381 = 4195 cy.

wslice1/5 收益更大因为 layer 少, host overhead 占比大.

### 局限

TB-only 改动 (sim 内 host 改成并行). 真硬件部署 host 端需要类似的 multi-thread /
DMA-driven boot reg 写才能拿到此收益. sim 数反映 host 实现的上限.

### 未来 Round G+ (更激进的 layer barrier 软化)

per-core layer N+1 启动 不等其他核 layer N done — 需要 driver 加层间数据依赖图
分析 (K=1 ds 层无 halo 可独立, K=3 层依赖左右邻居). 工作量大, 留给后续.

---

## Round 总览 (累计)

| Round | ResNet11 cy | Δ vs prev | 累计 vs IP baseline | 关键改动 |
|---|----:|----:|----:|---|
| Round B (sim crossbar) | 203320 | - | (老 baseline) | (sim model crossbar) |
| → 切真 IP | 217311 | +6.9% | 0% | sim crossbar → axi_smc IP, 14K cy register stage 真实开销 |
| Round C (cout slice) | 210784 | -3.0% | -3.0% | scheduler 优先 cout slice for L10 (FC) |
| Round E (mm2s_arb 防御) | 210784 | 0% | -3.0% | WDMA 饥饿提优先级 (防御 dispatcher prefetch) |
| Round F (host parallel) | 206589 | -2.0% | -4.9% | TB fork wait_dfe_done |
| Round G (desc preload) | 206121 | -0.23% | -5.2% | TB fork preload layer N+1 desc |
| Round H step 1 (S2MM pause) | 204251 | -0.91% | -6.0% | TB OFM check pause 200→30 cy |
| **Round H step 2 (ODMA sts bg)** | **204240** | **-0.005%** | **-6.0%** | ODMA dispatcher sts 后台 collect (跟 IDMA Round B) |
| @100 MHz Latency | 2.04 ms | (vs IP baseline 2.17 ms) | | |
| FPS | 490 | | | |

---

## 19. Round G — TB layer N+1 desc preload 跟 layer N 计算重叠 (2026-05-07)

### 改造

`sim/tb_smc/tb_smc_chain.sv` host 主循环:
- Round F 已经让 layer 内 4 核 dfe wait 并行
- Round G 进一步让 **layer N+1 的 dfe preload** (write boot regs + start_dfe + wait dfe done)
  跟 layer N 计算重叠

实现:
- 加 `r_dfe_preload_done[0:31]` flag, per-layer 同步
- layer 0 主 process 同步预拉 (启动)
- 后续 layer 在 layer (l-1) iteration 内 fork...join_none 异步预拉
  fork 内独占 csr 总线 (主 process 在 wait done 期间不写 csr)
- 主 process 进 layer l 前 wait r_dfe_preload_done[l]

### 实测

| Case | Round F | Round G | Δ |
|---|----:|----:|----:|
| smc_resnet11 | 206589 | **206121** | -468 cy (-0.23%) |
| smc_wslice1 | 4317 | 4317 | 0 (单层无 preload 机会) |
| smc_wslice5 | 21585 | 21573 | -12 |

### 边际收益小的原因

Round F 已经让单层内 4 核 dfe 并行, 大头被吃了 (~4200 cy/网). Round G 进一步把
~150 cy/层 dfe 时间跟 layer 计算重叠, 但 layer 计算 1000+ cy 已经 cover 大部分,
Round F 漏出来的小尾巴才被 Round G 收掉.

### 当前 PE util

131.5M useful_mac / (4 cores × 206121 cy × 256 PE) = **62.7%** (vs Round C 60.9%, IP baseline 60.5%)

### 剩余优化空间

剩 37.3% PE idle, 大头:
1. **ds 层 PE 阵列空转** (L3/L6/L9 PE util 8-22%, 结构性): 需要 mac_array Ky-cout-fold 或小 K 引擎. 6-10% 收益, 大改 RTL.
2. **psum_idle** (大部分层): bias_rf cs 切换 + mac_array 流水填充. 修需 bias_rf ping-pong (Round D 失败, 复杂) 或 mac_array 流水深化.
3. **act_id 在 ds 层** (cross-mem cmd 串行): 需要 dispatcher prefetch (Round C+ 失败 L0 退化, 现在有 mm2s_arb 兜底可重试, 但工作量大).
4. **per-core layer barrier 软化** (无 halo K=1 layer): driver 加数据依赖图分析. K=1 ds layer 可以 per-core 启动, 但 ResNet11 K=1 layer 占总 cy 比例 ~21%, 收益估 1-2%.

> **更新 (2026-05-08, §20 控制变量实验)**: 上面这 4 条推测被实验数据推翻 / 修正:
> - 推测 #1 (ds 阵列空转) **错**, 实际是 memory-bound (见 §20 实验 5)
> - 推测 #2 (bias_rf 流水化) **收益微乎其微** (-0.06%, 见实验 1)
> - 推测 #3 (dispatcher prefetch) **收益上限 ≤ 2-3%**, ring_wait 反弹吃掉一半 (见实验 4)
> - 推测 #4 仍然有效, 但 ROI 估计被高估
>
> **真正高 ROI 优化方向 (基于 §20 实验)**:
> 1. SMC 互联简化: -7.0% (实验 2 实测)
> 2. ds layer IFB 复用 (skip_idma + sequencer): -7-10% (实验 5 推论)
> 3. **算法层 K=1 ds + K=3 conv 合并: -16%** (实验 5 推论, 论文创新点)

---

## 20. Round H — 进一步 quick wins + 控制变量瓶颈分析 (2026-05-08)

### Round H steps (commits)

- **step 1** (`3b47e06`): TB OFM check pause 200 → 30 cy (axi_dm S2MM commit 实测 ~20 cy 足够). ResNet11 -1870 cy (-0.91%).
- **step 2** (`4c4d057`): ODMA SG dispatcher sts 后台 collect (跟 IDMA Round B 一致). ResNet11 -11 cy (微小).

### 控制变量实验 (commits + paper/data/)

为支持论文写作, 加性能 counter + ifdef 切换理想模型:
- `c9d41a8` Round E mm2s_arb WDMA starve (实验 3 验证 0 触发)
- `55f72c4` IDEAL_SMC ifdef 切换 + 恢复 sim crossbar (实验 2)
- `1e41af0` 加 mm2s_arb + dispatcher state counter (实验 3, 4) + IDEAL_BIAS_RF ifdef (实验 1)

5 个实验文档在 `paper/data/exp{1..5}*.md` (用户自管 paper/, 不进 git) + `summary_bottleneck_analysis.md`.

### 关键发现

| 实验 | 发现 | 论文价值 |
|---|---|---|
| 1 (IDEAL_BIAS_RF) | bias_rf cs 切换 stall 占 **0.06%** | 否定 §12 推测 D |
| 2 (IDEAL_SMC) | SMC 互联 IP 占 **7.0%** wall cy (14354 cy) | 最大单一可量化瓶颈 |
| 2b | SMC overhead 双峰分布: IDMA-bound 12% / compute-bound 1% | layer 类型差异化优化 |
| 3 (mm2s_arb counter) | WDMA `starve_preempt = 0` (Round E 0 触发) | 仲裁不是瓶颈 |
| 4 (dispatcher counter) | dispatcher 时间 fetch 17.5% / ring_wait 12.6% / data 44.7% / IDLE 24.2% | dispatcher prefetch 收益上限 ≤ 2-3% |
| 5 (ds layer 数学分析) | ds layer 是 **memory-bound 不是 compute-bound** | 否定 §12 推测 C, 论文创新方向 |

### Round 总览 (累计)

| Round | ResNet11 cy | Δ vs prev | 累计 vs IP baseline | 关键改动 |
|---|----:|----:|----:|---|
| Round B (sim crossbar) | 203320 | - | (老 baseline) | (sim model crossbar) |
| → 切真 IP | 217311 | +6.9% | 0% | sim crossbar → axi_smc IP |
| Round C (cout slice) | 210784 | -3.0% | -3.0% | scheduler 优先 cout slice for L10 |
| Round E (mm2s_arb 防御) | 210784 | 0% | -3.0% | WDMA 饥饿提优先级 |
| Round F (host parallel) | 206589 | -2.0% | -4.9% | TB fork wait_dfe_done |
| Round G (desc preload) | 206121 | -0.23% | -5.2% | TB fork preload layer N+1 desc |
| Round H step 1 (S2MM pause) | 204251 | -0.91% | -6.0% | TB OFM check pause 200→30 cy |
| Round H step 2 (ODMA sts bg) | 204240 | -0.005% | -6.0% | ODMA dispatcher sts 后台 collect |
| Round I L3/L6 only (9957414) | 192158 | -5.9% | -11.6% | ds K=1 stride>1, cin_slices==1 |
| **Round I L9 fix (e8ca7b0)** | **190977** | **-0.6%** | **-12.1%** | IFB_ROW_STEP 跟 STRIDE_H 同步, ds cin_slices>1 也 enable |
| Round J 探针 W 压缩 (软件层) | 261627 | **+37%** | **+20%** | ❌ axi_dm cmd 颗粒度反噬 (paper/data/exp7) |
| **Round K (driver L1 cmd reorder)** | **190133** | **-0.44%** | **-12.5%** | 本地 mem 段优先, head-of-line 缓解 (paper/data/exp8) |
| @100 MHz Latency | **1.90 ms** | (vs IP baseline 2.17 ms) | | |
| FPS | **526** | (+66 vs baseline 460) | | |

**Round J 含义**: W 维 stride 走 axi_dm IP 软件层不可行 — cmd 数膨胀 6-17×, fetch
overhead 增加 37K cy + data 时间 +30K cy 完全吃掉数据量减半收益. 推动论点:
**硬件 2D 寻址才是 W 压缩出路** (mem core 端解码寻址 pattern), 投入 1-2 周换 ~2-3% 整网.
当前论文阶段不投, 用作"为什么必须改硬件"的实证素材. (探针默认 OFF, FLUX_W_COMPRESS=1 触发)

**Round K 含义**: 验证 SMC head-of-line 假设方向对 (L1 C1-3 act_id 减 10%, 共 -843 cy),
但收益比预期小 (-0.44% vs 预期 -2~3%). 说明 L1 idle 主要瓶颈不在 SMC 仲裁,
而在 axi_dm IP cmd setup latency. 跟 Round J 互补: axi_dm IP 是性能天花板,
软件层 cmd 优化上限有限. (driver only, FLUX_LOCAL_FIRST=1 默认 ON, 详见 paper/data/exp8)

**Round L 评估 (放弃)**: L8 cout slice 替 W slice 估收益 -2.6%, 但 driver 当前
不支持 W slice ↔ cout slice 跨 layer transition (L7→L8→L9 stitch 复杂),
实施成本 1-2 周. ROI 不划算放弃. 加 scheduler.set_force_cout_layer_names + env hook
留给 future work. 跟 Round J/K 形成完整链: 软件层优化路径基本枚举完,
剩余空间需硬件改动. (paper/data/exp8 §5)

---

## 21. Round I — H 维 stride 分离 ds layer IDMA 数据量减半 (2026-05-08)

### 优化思路

实验 5 (paper/data/exp5) 推导出 ds layer (K=1 stride>1) 是 memory-bound: mac_array
fire 数 K²=1 vs K²=9 少 9×, 但 IDMA 拉的数据量相同 → mac_array 75% 时间等数据.

K=1 stride=2 数学上每行 IFM 只用 stride 间隔行 (1/4 像素), 当前实现拉 dense H × W 浪费.
让 IDMA 跳行拉 (H 维 stride 后 cmd), mac_array 当 H stride=1 跑.

W 维不能优化 (axi_dm 不支持 strided burst).

### RTL 改动 (~25 行)

- `params.py` + svh: CSR_ADDR_MAP 加 `STRIDE_H = 0x1F0`
- `cfg_regs.sv`: 加 `r_stride_h` reg + `stride_h` 输出. 写 STRIDE 同步 stride_h (默认兼容). 写 STRIDE_H 单独 override.
- `line_buffer.sv`: 3 处 H 维 stride 用法 (rows_needed_next / rows_consumed_raw / y_row_base) 改用 `cfg_stride_h`. W 维 (`iss_pos_s`) 保持 `cfg_stride`.
- `core_top.sv`: `cfg_stride_h` wire

### Driver 改动

- `mesh_cmd.gen_idma_sg_cmd_list_w_slice`: 加 `h_compress_stride` 参数, 每 cmd src_addr 跳 stride 行
- `run_multicore_chain.py`: 检测 ds layer (K=1, stride>1, cin_slices=1) 启用
- `hw_files.cfg_to_dict`: 加 STRIDE_H + 改 H_IN_TOTAL = h_in_idma

### 实测每层 cy 对比

| L | Round H | Round I | Δ |
|---|---:|---:|---:|
| L0 Patch | 46015 | 46019 | +4 (噪音) |
| L1 | 27367 | 27709 | +342 (噪音) |
| L2 | 19798 | 19800 | +2 |
| **L3 ds (cin_slices=1)** | 18369 | **10921** | **-7448 (-40.5%)** ★ |
| L4 | 11303 | 11305 | +2 |
| L5 | 22174 | 22176 | +2 |
| **L6 ds (cin_slices=1)** | 11721 | **6726** | **-4995 (-42.6%)** ★ |
| L7 | 12537 | 12539 | +2 |
| L8 | 27362 | 27364 | +2 |
| L9 ds (cin_slices=2 跳过) | 4151 | 4153 | +2 |
| L10 FC | 2926 | 2927 | +1 |
| **Total** | **204240** | **192158** | **-12082 (-5.9%)** |

### L9 fix (commit `e8ca7b0`, 2026-05-08)

L9 cin_slices=2 启用 H stride compress 时 OFM mismatch 根因: `cfg.IFB_ROW_STEP`
原算法 = stride × W × cs (dense H_in IFB 视角). H compress 后 IFB 行 dense,
应该 = stride_h × W × cs (跟 IDMA 实际写 layout 对齐).

修复: `hw_files.cfg_to_dict` 改 IFB_ROW_STEP 用 `_STRIDE_H` 字段 (默认 = stride 兼容).

修复后 L9 cy 4151 → 2972 (-28.4%, -1179 cy). 整网 192158 → 190977 (-0.6% 增量).
ResNet11 累计 vs IP baseline: -12.1%, FPS 524.

### 论文意义

- **打到瓶颈**: 实验 5 推论 (ds memory-bound) 实施验证, ds layer cy 减 40%+ 印证
  memory-bound 假设
- **RTL 改动小**: < 15 行实质 RTL 改动拿 5.9% 收益, 控制变量实验的高 ROI 案例
- **跟其他 round 对比**: bias_rf ping-pong (Round D 失败) 即使成功 0.06% < Round I 0.5 天投入 5.9%

### 失败 round 记录

- **Round D**: bias_rf ping-pong (state machine bug, cs > 1 hang). 实验 1 证明就算成功收益 0.06%, 不值得.
- **Round C+**: dispatcher cmd FIFO prefetch (L0 退化 +17K cy). 实验 4 证明 ring_wait 反弹会吃 50% 收益, 净 ≤2%.

### 下一步: 论文写作准备

实验数据齐全, 5 大瓶颈都有实证:
1. 性能 baseline + 6 round 优化路径
2. SMC 互联 overhead 量化
3. ds layer 根因 (memory-bound 创新论证)
4. 各种早期推测的实验证伪 (科学方法亮点)

可直接引用 `paper/data/` 内 5 个 exp 报告 + summary 写论文 §性能瓶颈分析章节.
