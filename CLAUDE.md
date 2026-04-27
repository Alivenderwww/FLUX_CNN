# CLAUDE.md

Claude Code 工作指引。项目概览请见 `README.md`，细节分散在 `docs/`。

## 项目总览

FLUX_CNN：SystemVerilog CNN 加速器，16×16 int8 MAC 阵列（256 MAC）。顶层结构两层：

1. **Core pipeline**（5 模块 + 共享 `cfg_regs`，valid-ready 握手，无中心 FSM）
   `line_buffer → mac_array → parf_accum → ofb_writer`；`wgt_buffer` 侧路供 WRF。
   `parf_accum` 内部由 `parf_col` × NUM_COL 组成（每列独立 SRAM，外壳共享 wr_addr/we）。
2. **DMA 子系统**（`idma_ctrl / wdma_ctrl / odma_ctrl` + `mm2s_arb` + Xilinx `axi_dm`
   IP + `axi_m_mux` + `axi_lite_csr`）外部看是 1 个 AXI4 Master + 1 个 AXI-Lite
   Slave。idma_ctrl/wdma_ctrl 共享 axi_dm 的 MM2S 通道（mm2s_arb 串行仲裁），odma_ctrl
   占 S2MM 通道。axi_dm IP 由 `Syn/gen_axi_datamover.tcl` 生成 (Vivado 2023.1).
   ModelSim 仿真前需先跑 `Syn/compile_simlib.tcl` 一次性编译 Vivado simlib.

**运行模式**：统一为 **streaming row-ring**。原 batch 模式是 ring 容量覆盖整图的退化情形。支持任意 Cin/Cout（多 slice 切片）、任意 H×W（strip 粒度 ring）。

**PE 利用率优化**（仅处理 Cin 小的情况；Cout 小时硬件不复用，对应 PE 列空转）：
- `--ky-fold` : Cin<16 时把 Ky 折到 cin_fake，编译器侧输入 y 偏移复制
- `--s2d`     : stride≥2 时把 4 相位折到 cin，编译器侧重排无复制（DDR 友好）

## 仿真目录

- `sim/tb_core_dma/` — 端到端 descriptor-driven 测试（走 AXI-Lite 配 + AXI4 M 搬 IFB/WB/ODMA + DFE 拉 descriptor list）
- `sim/tb_axi_lite_csr/` `sim/tb_axi_m_mux/` — AXI 子模块单测
- `sim/tb_axi_dm_smoke/` — axi_dm IP 跨语言 elab smoke
- `sim/tb_idma_ctrl/` — idma_ctrl + axi_dm DDR→IFB 联合自测 (验证 DataMover 指令控制端)

## 常用命令

```bash
# 端到端回归（默认 22 case ResNet-18 风格）
cd toolchain
python run_regression.py                      # 无 fold 基线
python run_regression.py --fold               # Ky-fold
python run_regression.py --fold --s2d         # Ky-fold + S2D
python run_regression.py --case "C64C64"      # 只跑 name 含子串的 case
python run_regression.py --timeout-ns 2e9     # 手动 watchdog 超时

# 单 case 生成 (到 sim/tb_core_dma/)
cd toolchain
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --num_cin 8 --num_cout 8 --pad 1
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 --num_cin 4 --num_cout 8 --stride 2 --pad 4 --ky-fold
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 --num_cin 4 --num_cout 8 --stride 2 --pad 4 --s2d

# PyTorch 模型端到端部署
cd toolchain
python models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10
```

## 配置 & 代码约定

- **SystemVerilog IEEE 1800**；无 packages；packed struct + enum
- 核心参数：`NUM_COL=NUM_PE=16` / `WRF=ARF=PARF=32` / `DATA=8` / `PSUM=32` / `SRAM=8192` / `ADDR_W=20`
- AXI：内部 `BUS_DATA_W=128`，外部 1 主口 (axi_m_mux 聚合 axi_dm.MM2S/S2MM + DFE) + CSR `32` 位 slave
- `gen_isa_test.py` 是 derived 值的 source of truth；`hw_files.derive_layer_cfg()` 共享 cfg 派生
- Commit prefix 用中文：`Feat:` / `Perf:` / `Docs:` / `Fix:` / `Refactor:`
- RTL 风格规范详见 `RTL代码编写原则.md`（4 大原则 + 例外清单）
- 加新 RTL 要同步 `sim/<tb>/run.tcl` 和 `Syn/run_syn.tcl`

## 文档导航

- `README.md` — 顶层叙述 + 性能表
- `docs/modules/` — 每个 RTL 模块的时序与数据流（按文件名查找：line_buffer / mac_array / parf_accum / cfg_regs / sequencer / dfe / core_top / ... 等; idma_ctrl/wdma_ctrl/odma_ctrl/mm2s_arb 见 RTL 文件头注释）
- `docs/slicing/` — Cin/Cout > 16 时编译器、cfg 寄存器、硬件循环嵌套的切片机制
- `docs/pe-fold.md` — Ky-fold + Space-to-Depth 数学推导 + 实现位置
- `docs/simulation.md` — TB 机制、回归流程、CASE_RESULT / CASE_PROFILE 输出格式
- `docs/multi-layer-compilation.md` — PyTorch 多层编译
- `docs/roadmap.md` — 未来工作
- `docs/synthesis.md` — 综合（参考）
- `RTL代码编写原则.md` — RTL 风格指南
- `model_analysis.md` — 目标模型 PE 利用率分析
