# FLUX_CNN 项目当前状态 (2026-04-30)

> 本文件是 **任务交接文档**, 记录截止 2026-04-30 的设计状态、未完成事项、下一步计划。
> M1.5 多核仿真死锁已修复 (`axi_m_mux` 字段 forward + `axi_arbiter` sel sticky), 见 §2.
> M2 跨核 SRAM 直送 双层 chain 仿真验证通过, 见 §2.5.
> M2.5 多核调度器 + 切片决策 + driver 框架已就位, 见 §2.6.
> 参数 single source of truth (`params.py`) 完成, 见 §2.7.
> P1 Mode C W slice 单层 + 多层 chain 全 PASS, 见 §2.8.
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
