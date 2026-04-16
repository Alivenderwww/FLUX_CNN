# FLUX CNN 加速器

FLUX_CNN 是一个用 SystemVerilog 实现的 CNN 加速器：16×16 int8 MAC 阵列（256 个 MAC），由**配置寄存器驱动的自循环 FSM** 编排，支持任意 K×K 卷积核、任意步长、Cin/Cout 切片、WRF 轮次分块；行为级 SRAM 模型（IFB / WB / OFB）用于存储特征图和权重；综合目标 Xilinx UltraScale+ `xcku060`，100 MHz 时时序全满足。

---

## 架构要点

- **256 个 MAC 并行**：16 列 × 16 PE，列间广播激活值（Output Channel Broadcast），每列累加一个输出通道。
- **三层数据复用**：Weight Stationary（权重静止）、ARF 滑动窗口（stride=1 复用）、输出通道广播（同 `act_to_mac` 喂 16 列）。
- **配置寄存器 + 自驱动 FSM**：6 层嵌套循环 `cs → yout → tile → cins → round → (ky,kx)` 全在硬件内部推进，地址由运行指针 `+= STEP` 维护，硬件零乘除。
- **轮次分块**：K² > 32 时（K≥7），每 cins 内按 WRF_DEPTH=32 切分为多轮，轮间 PARF 不清零、ky/kx 继续推进。
- **20-bit 内部地址**：支持 1M 字寻址，单切片 224×224 特征图可全存 IFB。

---

## 目录导航

| 文档 | 内容 |
|------|------|
| [`docs/architecture.md`](docs/architecture.md) | 模块层次、数据通路、数据复用策略 |
| [`docs/config-registers.md`](docs/config-registers.md) | 配置寄存器完整列表（位宽、含义、软件预算规则） |
| [`docs/fsm-pipeline.md`](docs/fsm-pipeline.md) | FSM 状态机、循环嵌套、流水线时序对齐 |
| [`docs/simulation.md`](docs/simulation.md) | 仿真运行指南、TB 配置 poke 机制、回归测试 |
| [`docs/synthesis.md`](docs/synthesis.md) | Vivado OOC 综合结果（资源/功耗/时序） |
| [`docs/roadmap.md`](docs/roadmap.md) | 未来工作（padding、残差、多核、性能调优） |
| [`docs/isa-legacy.md`](docs/isa-legacy.md) | 已退役的 Macro-ISA 方案（历史参考） |
| [`model_analysis.md`](model_analysis.md) | 目标模型 PE 利用率分析 |
| [`history.md`](history.md) | 设计演进对话记录（含 ISA→cfg-driven 的决策讨论） |
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 使用指引 |

---

## 快速上手

```bash
cd sim/tb_core_isa

# 1. 生成测试数据 + 配置
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --stride 1 --num_cin 8 --num_cout 8

# 2. 运行仿真
vsim -c -do tb_core_isa.tcl

# 3. 跑完整回归
python run_regression.py --label cfg-driven --out regression_cfg_driven.txt
```

更多细节见 [`docs/simulation.md`](docs/simulation.md)。

---

## 当前状态

- **功能**：标准卷积（任意 K / stride / Cin / Cout）全通
- **不支持**：padding、残差、pooling（见 roadmap）
- **回归**：17/17 PASS（含 K=7 多轮次、224×224 大图），`regression_cfg_driven.txt`

| 代表用例 | 周期数 | MAC 利用率 |
|---|---|---|
| K=3 C8C8 66×118 s=1 | 80 267 | 87.32% |
| K=7 C8C8 62×114 s=1 | 377 706 | 91.69% |
| K=7 C3C8 218×218 s=1 | 2 525 750 | 92.20% |

对比原 ISA 方案（`regression_16x16.txt`），cfg-driven FSM 在同规模上消除了 fetch/decode/标量指令开销，K=7 68×120 用例从 378 211 降至 377 706 周期（-0.13%），利用率 91.57% → 91.69%。
