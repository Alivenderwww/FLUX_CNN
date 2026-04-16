# FLUX CNN 加速器

FLUX_CNN 是一个用 SystemVerilog 实现的 CNN 加速器：16×16 int8 MAC 阵列（256 个 MAC），由 **5 个去中心化模块 + valid-ready 握手流水**编排，支持任意 K×K 卷积核、任意步长、Cin/Cout 切片；行为级 SRAM 模型（IFB / WB / OFB）用于特征图和权重存储。

---

## 架构要点

- **256 个 MAC 并行**：16 列 × 16 PE，列间广播激活值（Output Channel Broadcast），每列累加一个输出通道。
- **去中心化握手流水**：5 个顶层模块（`line_buffer` / `wgt_buffer` / `mac_array` / `parf_accum` / `ofb_writer`） 各自跑自己的循环，模块间 `valid-ready` 握手串联。无中心 FSM。
- **cross-round pipeline**：`line_buffer` 的 issue counter 连续推进，ring buffer 读/写指针模运算永不重置，整层只在开头 2 拍 startup，round 边界零气泡。
- **FILL/DRAIN overlap**：`parf_accum` 的 tile N drain 隐藏在 tile N+1 fill 的 first_round 里，消除 DRAIN 对 MAC 的反压。
- **Elastic join 握手**：`mac_array` 两侧 ready 互相依赖对方 valid，保证 stall 下 in-flight 数据不丢失；`compute_en` 低时 PE/加法树寄存器保持。
- **20-bit 内部地址**：支持 1M 字寻址，单切片 224×224 特征图可全存 IFB。

---

## 目录导航

| 文档 | 内容 |
|------|------|
| [`docs/architecture.md`](docs/architecture.md) | 模块层次、数据通路、握手拓扑 |
| [`docs/handshake-pipeline.md`](docs/handshake-pipeline.md) | 握手协议、各模块推进机制、气泡分析 |
| [`docs/config-registers.md`](docs/config-registers.md) | 配置寄存器位宽 + 语义 + 软件预算规则 |
| [`docs/simulation.md`](docs/simulation.md) | 仿真运行指南、TB 配置 poke 机制、回归测试 |
| [`docs/roadmap.md`](docs/roadmap.md) | 未来工作（chunked、stride=1 滑窗、padding、残差、pooling） |
| [`docs/isa-legacy.md`](docs/isa-legacy.md) | 已退役方案历史（Macro-ISA + cfg-driven FSM） |
| [`docs/synthesis.md`](docs/synthesis.md) | Vivado OOC 综合（非主要关注点） |
| [`model_analysis.md`](model_analysis.md) | 目标模型 PE 利用率分析 |
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
python run_regression.py --label v3-handshake --out regression_v3.txt
```

更多细节见 [`docs/simulation.md`](docs/simulation.md)。

---

## 当前状态

- **功能**：标准卷积（任意 K / stride / Cin / Cout，packed 场景 `K²·cin_slices ≤ 32`）全通
- **不支持（v2 待办）**：chunked 模式（K≥7、Cin>32）、padding、残差、pooling；详见 roadmap
- **回归**：10/10 PASS，[regression_v3.txt](sim/tb_core_isa/regression_v3.txt)

### 代表用例性能（K=3 packed 区域）

| 用例 | Cycles | MAC 利用率 | vs 原 cfg-driven FSM |
|---|---:|---:|---:|
| K=3 C8C8 66×118 s=1 | 70,128 | 99.95% | **-12.6%** (快) |
| K=3 C32C8 66×118 s=1 | 140,229 | 99.97% | - |
| K=3 C32C32 66×118 s=1 | 280,433 | 99.98% | - |
| K=5 C16C16 30×56 s=2 | 43,556 | 99.87% | - |

**理论下限** = MAC fire 总拍数（同参数下等于 `cout_slices × h_out × sum_of_tile_widths × cin_slices × K²`）。v3 实测在 K=3 C8C8 用例上只比理论下限多 36 拍（layer 起始 startup + pipe drain）。

### 演进对比（K=3 C8C8 66×118 s=1）

| 版本 | Cycles | MAC 利用率 | 备注 |
|---|---:|---:|---|
| 原 Macro-ISA | 80,757 | 87.28% | 64-bit 指令 + 取指译码，已退役 |
| cfg-driven FSM | 80,267 | 87.32% | 单 FSM 自驱动 6 层循环，已退役 |
| v1 握手（串行 LOAD/STREAM） | 145,489 | 48.18% | 通路验证版本 |
| v2 (ring + parf overlap) | 74,878 | 93.61% | 消除 parf drain 反压 |
| **v3 (cross-round pipe)** | **70,128** | **99.95%** | 当前版本 |
