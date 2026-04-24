# CLAUDE.md

Claude Code 工作指引。项目概览请见 `README.md`，细节分散在 `docs/`。

## 项目总览

FLUX_CNN：SystemVerilog CNN 加速器，16×16 int8 MAC 阵列（256 MAC）。顶层结构两层：

1. **Core pipeline**（5 模块 + 共享 `cfg_regs`，valid-ready 握手，无中心 FSM）
   `line_buffer → mac_array → parf_accum → psum_reshape → ofb_writer`；`wgt_buffer` 侧路供 WRF。
   `parf_accum` 内部由 `parf_col` × NUM_COL 组成，每列独立存储以支持 Kx-fold 的 per-col wr_addr 偏移。
2. **DMA 子系统**（`idma / wdma / odma` + `axi_m_mux` + `axi_lite_csr`）
   外部看是 1 个 AXI4 Master + 1 个 AXI-Lite Slave，内部 3 DMA 聚合。

**运行模式**：J-1 起统一为 **streaming row-ring**（`DMA_MODE=3`）。原 batch 模式是 ring 容量覆盖整图的退化情形。支持任意 Cin/Cout（多 slice 切片）、任意 H×W（strip 粒度 ring）。

**PE 利用率优化**（K 阶段）：
- `--ky-fold` : Cin<16 时把 Ky 折到 cin_fake，填满 PE 行
- `--kx-fold` : Cout<16 时把 Kx 折到 cout_fake（systolic），填满 PE 列
- 两者可叠加，stem 上 6.35M → 1.11M cycles（5.74×），PE util 99.9%

## 仿真目录

- `sim/tb_core_dma/` — 端到端 descriptor-driven 测试（走 AXI-Lite 配 + AXI4 M 搬 IFB/WB/ODMA + DFE 拉 descriptor list）
- `sim/tb_axi_lite_csr/` `sim/tb_axi_m_mux/` `sim/tb_idma/` `sim/tb_wdma/` `sim/tb_odma/` — AXI/DMA 子模块单测

## 常用命令

```bash
# 端到端回归（默认 22 case ResNet-18 风格）
cd toolchain
python run_regression.py                      # 无 fold 基线
python run_regression.py --fold               # Ky-fold
python run_regression.py --fold --kx-fold     # Ky + Kx fold 叠加 (最优)
python run_regression.py --case "C64C64"      # 只跑 name 含子串的 case
python run_regression.py --timeout-ns 2e9     # 手动 watchdog 超时

# 单 case 生成 (到 sim/tb_core_dma/)
cd toolchain
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --num_cin 8 --num_cout 8 --pad 1
python gen_isa_test.py --k 7 --h_in 960 --w_in 540 --num_cin 4 --num_cout 8 --stride 2 --pad 3 --ky-fold --kx-fold

# PyTorch 模型端到端部署
cd toolchain
python models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10
```

## 配置 & 代码约定

- **SystemVerilog IEEE 1800**；无 packages；packed struct + enum
- 核心参数：`NUM_COL=NUM_PE=16` / `WRF=ARF=PARF=32` / `DATA=8` / `PSUM=32` / `SRAM=8192` / `ADDR_W=20`
- AXI：内部 `BUS_DATA_W=128`，外部 1 主口（mux 聚合 IDMA/WDMA/ODMA）+ CSR `32` 位 slave
- `gen_isa_test.py` 是 derived 值的 source of truth；`hw_files.derive_layer_cfg()` 共享 cfg 派生
- Commit prefix 用中文：`Feat:` / `Perf:` / `Docs:` / `Fix:` / `Refactor:`
- RTL 风格规范详见 `RTL代码编写原则.md`（4 大原则 + 例外清单）
- 加新 RTL 要同步 `sim/<tb>/run.tcl` 和 `Syn/run_syn.tcl`
- Kx-fold 约束：`kxper % stride == 0`（软件端在 `compute_fold_params_kx` 里自动选最优参数）

## 文档导航

- `README.md` — 顶层叙述 + 性能表
- `docs/architecture.md` — 核流水模块层次 + 数据通路
- `docs/handshake-pipeline.md` — 握手协议、推进事件、气泡分析
- `docs/config-registers.md` — cfg 寄存器位宽 + 语义
- `docs/pe-fold.md` — Ky/Kx-fold 数学推导 + RTL 实现
- `docs/streaming-row-ring.md` — streaming 数据路径规范
- `docs/simulation.md` — TB 机制、回归流程
- `docs/multi-layer-compilation.md` — PyTorch 多层编译
- `docs/descriptor-sequencer.md` — descriptor + sequencer
- `docs/roadmap.md` — 未来工作
- `docs/synthesis.md` — 综合（参考）
- `RTL代码编写原则.md` — RTL 风格指南
- `model_analysis.md` — 目标模型 PE 利用率分析
