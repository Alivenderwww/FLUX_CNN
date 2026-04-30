# FLUX CNN 加速器

FLUX_CNN 是一个用 SystemVerilog 实现的 CNN 加速器：**16×16 int8 MAC 阵列（256 MAC）**，由去中心化 5 模块 + valid-ready 握手流水编排，外挂 AXI4 DMA 子系统（IDMA / WDMA / ODMA + `axi_m_mux` + AXI-Lite CSR），通过 **row-ring streaming 数据路径** 以单次 start 处理远超 SRAM 的大图。

---

## 架构要点

### 核流水（core pipeline）
- **256 个 MAC 并行**：16 列 × 16 PE，列间广播激活值（Output Channel Broadcast），每列一个输出通道
- **去中心化握手流水**：`line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer` 各自维护 counter，`valid-ready` 握手串联，无中心 FSM
- **PARF 拆分为 per-col 存储**：`parf_col` × NUM_COL + `parf_accum` 外壳，每列独立 SRAM、共享 wr_addr/we
- **cross-round pipeline**：line_buffer ring buffer 读/写指针模运算永不重置
- **FILL/DRAIN overlap**：parf_accum 的 tile N drain 隐藏在 tile N+1 fill 的 first_round 里
- **Elastic join 握手**：mac_array 两侧 ready 互相依赖对方 valid，stall 下 in-flight 数据不丢
- **统一 chunked 权重调度**：`wgt_buffer` 走 cins-ahead 流水路径；K=1 单 slot 场景用 1 拍 bubble 守护 2 拍流水 hazard

### DMA 子系统
- **AXI4 M（128-bit）外部 1 主口**：`axi_m_mux` 聚合 IDMA (MM2S IFB) + WDMA (MM2S WB，128→2048 bit packing) + ODMA (S2MM OFB) + DFE (descriptor fetch)
- **AXI-Lite Slave（32-bit CSR）**：`axi_lite_csr` + `cfg_regs` 寄存器组；host 下发配置 + DMA 描述符 + `CTRL` 启停位

### Streaming row-ring
- IFB / OFB 作 row-level ring buffer，按 `strip_rows` 取模
- **双向 row-credit 反压**：line_buffer 消费行 → 送 IDMA；ODMA 排空行 → 送 ofb_writer
- **forward-pressure**：line_buffer 仅在 `rows_available >= yout*stride + Ky` 时发射
- **并发**：sequencer 一次 dispatch 同启 IDMA / 核 / ODMA，写/算/读同拍进行
- **任意 Cin/Cout**：Cin/Cout > 16 走多 slice 切片

### PE 利用率优化（仅处理 Cin 小的情况；Cout 小时硬件不复用，对应 PE 列空转）
- **Ky-fold**：`Cin < PE_H` 时把 Ky 维折到 `cin_fake`（y 方向偏移复制到空闲 cin 行），PE 行填满。纯软件，零 RTL 改动。
- **Space-to-Depth (S2D)**：`stride ≥ 2` 时把 (kx%stride, ky%stride) 4 相位折到 `cin_new = stride² × Cin`。等价为 stride=1, K_new=ceil(K/stride) 的 conv。编译器侧重排不复制，DDR 友好。

---

## 目录导航

| 文档 | 内容 |
|------|------|
| [`STATUS.md`](STATUS.md) | **当前进度 + 任务交接** (2026-04-29 更新) |
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 工作指引（含快速命令） |
| [`memory/`](memory/) | 设计经验小文档 (踩过的坑 + 当时不明显的设计决策) |
| [`docs/modules/`](docs/modules/) | 每个 RTL 模块的时序与数据流 |
| [`docs/slicing/`](docs/slicing/) | Cin/Cout > 16 时编译器、cfg、硬件循环嵌套的切片机制 |
| [`docs/pe-fold.md`](docs/pe-fold.md) | Ky-fold + Space-to-Depth 数学推导 + 实现位置 |
| [`docs/simulation.md`](docs/simulation.md) | 仿真运行指南、TB 机制、回归测试 |
| [`docs/multi-layer-compilation.md`](docs/multi-layer-compilation.md) | PyTorch `nn.Sequential` 多层编译 |
| [`docs/roadmap.md`](docs/roadmap.md) | 未来工作 |
| [`docs/synthesis.md`](docs/synthesis.md) | Vivado OOC 综合（参考） |
| [`RTL代码编写原则.md`](RTL代码编写原则.md) | RTL 风格指南 |
| [`model_analysis.md`](model_analysis.md) | 目标模型 PE 利用率分析 |

---

## 快速上手

```bash
# 端到端回归：22 case (ResNet-18 风格)
cd toolchain
python run_regression.py                # 无 fold
python run_regression.py --fold         # Ky-fold
python run_regression.py --s2d          # Space-to-Depth (stride>=2)
python run_regression.py --fold --s2d   # 两者叠加

# 单 case 生成
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --num_cin 8 --num_cout 8 --pad 1
cd ../sim/tb_core_dma && vsim -c -do run.tcl

# PyTorch 模型端到端
cd toolchain
python models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10
```

---

## 当前状态

- **功能通路**：任意 K × stride × Cin × Cout 通过；多 slice 切片支持 Cin/Cout > 16
- **回归**：22/22 PASS（ResNet-18 风格 stem + L1-L4 + FC，含 Ky-fold / S2D 变体）
- **PyTorch 编译器**：`nn.Sequential` 多层端到端 bit-exact

### Cin / Cout 小时的处理

| 场景 | 处理 |
| --- | --- |
| Cin < 16, K > 1 | Ky-fold 编译器侧把 Ky 折到 cin |
| Cin < 16, stride ≥ 2 | S2D 优先（无数据复制） |
| Cout < 16 | 不复用，对应 PE 列空转，util ≈ Cout/16 |
| Cin ≥ 16, Cout ≥ 16 | 直跑，PE 阵列填满 |

### VGA 480×640 端到端

单图 4.9 MB，SRAM 只用 10 KB ring（`strip_rows=8 × W_IN=640`），一次 start 跑完。
