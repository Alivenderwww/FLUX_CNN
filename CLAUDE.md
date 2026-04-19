# CLAUDE.md

Claude Code 工作指引。项目概览请见 `README.md`，细节分散在 `docs/`。

## 项目总览

FLUX_CNN：SystemVerilog CNN 加速器，16×16 int8 MAC 阵列（256 MAC）。顶层结构两层：

1. **Core pipeline**（5 模块 + 共享 `cfg_regs`，valid-ready 握手，无中心 FSM）
   `line_buffer → mac_array → parf_accum → ofb_writer`；`wgt_buffer` 侧路供 WRF。
2. **DMA 子系统**（`idma / wdma / odma` + `axi_m_mux` + `axi_lite_csr`）
   外部看是 1 个 AXI4 Master + 1 个 AXI-Lite Slave，内部 3 DMA 聚合。

**两种运行模式**（由 cfg `DMA_MODE` 切换）：
- **Batch**（v1，`DMA_MODE=0`）：IDMA/WDMA 先把整图搬进 SRAM → core 跑 → ODMA 把 OFB 搬出。
- **Streaming row-ring**（v2，`DMA_MODE=3`）：IFB/OFB 作 row-level ring，IDMA/core/ODMA 并发，单次 start 处理远超 SRAM 的大图（已验证 480×640 VGA）。限制：Cin ≤ 16 且 Cout ≤ 16。

## 仿真目录

- `sim/tb_core_dma/` — 端到端 descriptor-driven 测试（走 AXI-Lite 配 + AXI4 M 搬 IFB/WB/ODMA + DFE 拉 descriptor list），batch + streaming 双模
- `sim/tb_axi_lite_csr/` `sim/tb_axi_m_mux/` `sim/tb_idma/` `sim/tb_wdma/` `sim/tb_odma/` — AXI/DMA 子模块单测

## 常用命令

```bash
# DMA 端到端回归（batch）
cd sim/tb_core_dma
python run_regression.py                                # 14 case

# Streaming 回归（单次 start，含大图 VGA 480×640）
python gen_isa_test.py --k 3 --h_in 480 --w_in 640 --num_cin 3 --num_cout 16 --streaming
python run_regression_stream.py                         # 11 case

# 单个 case + pad 测试
python gen_isa_test.py --k 3 --h_in 32 --w_in 60 --num_cin 8 --num_cout 8 --pad 1
vsim -c -do run.tcl

# Multi-strip descriptor 生成
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --num_cin 8 --num_cout 8 --strip_rows 16
```

## 配置 & 代码约定

- **SystemVerilog IEEE 1800**；无 packages（`core_isa_pkg` 已删）；packed struct + enum
- 核心参数：`NUM_COL=NUM_PE=16` / `WRF=ARF=PARF=32` / `DATA=8` / `PSUM=32` / `SRAM=8192` / `ADDR_W=20`
- AXI：内部 `BUS_DATA_W=128`，外部 1 主口（mux 聚合 IDMA/WDMA/ODMA）+ CSR `32` 位 slave
- `gen_isa_test.py` 是 derived 值 (step、round_len、OFM 维度、streaming ring cfg) 的 source of truth；改 cfg 语义要两端同步
- `--streaming` 要求 `cin_slices == 1 && cout_slices == 1`（Cin/Cout ≤ 16）
- Commit prefix 用中文：`Feat:` / `Perf:` / `Docs:` / `Fix:` / `Refactor:`
- RTL 风格规范详见 `RTL代码编写原则.md`（4 大原则 + 例外清单）
- 加新 RTL 要同步 `sim/<tb>/run.tcl` 和 `Syn/run_syn.tcl`

## 文档导航

- `README.md` — 顶层叙述 + 性能表
- `docs/architecture.md` — 核流水模块层次 + 数据通路
- `docs/handshake-pipeline.md` — 握手协议、推进事件、气泡分析
- `docs/config-registers.md` — cfg 寄存器位宽 + 语义 + 预计算
- `docs/streaming-row-ring.md` — v2 streaming 架构规范 + 实现结果
- `docs/simulation.md` — TB 机制、回归流程
- `docs/roadmap.md` — 未来工作（padding、残差、pooling、Cout > 16 切片）
- `docs/isa-legacy.md` / `docs/synthesis.md` — 历史 / 综合（参考）
- `RTL代码编写原则.md` — RTL 风格指南
- `model_analysis.md` — 目标模型 PE 利用率分析
