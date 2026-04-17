# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FLUX_CNN is a CNN accelerator in SystemVerilog. A 16×16 int8 MAC array (256 MACs) is orchestrated by **5 个去中心化模块 + 共享配置寄存器**，模块间靠 valid-ready 握手串起流水。每个模块自带 counter 独立推进循环，不再有任何中心 FSM。

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
