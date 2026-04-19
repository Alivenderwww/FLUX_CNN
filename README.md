# FLUX CNN 加速器

FLUX_CNN 是一个用 SystemVerilog 实现的 CNN 加速器：16×16 int8 MAC 阵列（256 MAC），由 **5 个去中心化模块 + valid-ready 握手流水** 编排，外包 **AXI4 DMA 子系统**（IDMA / WDMA / ODMA + `axi_m_mux` + AXI-Lite CSR），支持 batch 和 **streaming row-ring** 两种模式。

---

## 架构要点

### 核流水（core pipeline）
- **256 个 MAC 并行**：16 列 × 16 PE，列间广播激活值（Output Channel Broadcast），每列累加一个输出通道
- **去中心化握手流水**：`line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer` 各跑自己的 counter，`valid-ready` 握手串联，无中心 FSM
- **cross-round pipeline**：line_buffer ring buffer 读/写指针模运算永不重置，整层只 2 拍 startup
- **FILL/DRAIN overlap**：parf_accum 的 tile N drain 隐藏在 tile N+1 fill 的 first_round 里
- **Elastic join 握手**：mac_array 两侧 ready 互相依赖对方 valid，stall 下 in-flight 数据不丢
- **内部地址 20 bit**：1M 字寻址

### DMA 子系统
- **AXI4 M（128-bit）外部 1 主口**：`axi_m_mux` 聚合 IDMA (MM2S IFB) + WDMA (MM2S WB，128→2048 bit packing) + ODMA (S2MM OFB) + 1 reserved
- **AXI-Lite Slave（32-bit CSR）**：`axi_lite_csr` + `cfg_regs` 寄存器组，flat 32-bit 地址空间；host 下发配置 + DMA 描述符 + `CTRL` 启停位

### Streaming row-ring（v2，单次 start 跑超大图）
- IFB / OFB 都作 row-level ring buffer，按 `strip_rows` 取模
- **双向 row-credit 反压**：line_buffer 消费行 → 送 IDMA；ODMA 排空行 → 送 ofb_writer
- **forward-pressure**：line_buffer 只在 `rows_available >= yout*stride + K` 才发射
- **并发**：一次 `CTRL=0xB` 同启 IDMA/核/ODMA，IDMA 写 / 核算 / ODMA 读同拍进行
- **限制（v2）**：`cin_slices = 1 && cout_slices = 1`（Cin ≤ 16 && Cout ≤ 16）；cs 外层循环留 v3

---

## 目录导航

| 文档 | 内容 |
|------|------|
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 工作指引（含快速命令） |
| [`docs/architecture.md`](docs/architecture.md) | 核流水模块层次 + 数据通路 |
| [`docs/handshake-pipeline.md`](docs/handshake-pipeline.md) | 握手协议、推进事件、气泡分析 |
| [`docs/config-registers.md`](docs/config-registers.md) | cfg 寄存器位宽 + 语义 + 软件预算规则 |
| [`docs/streaming-row-ring.md`](docs/streaming-row-ring.md) | **streaming v2 架构规范 + 实现结果** |
| [`docs/simulation.md`](docs/simulation.md) | 仿真运行指南、TB 机制、回归测试 |
| [`docs/roadmap.md`](docs/roadmap.md) | 未来工作（Cout > 16 切片、padding、残差、pooling） |
| [`docs/isa-legacy.md`](docs/isa-legacy.md) | 已退役方案历史（Macro-ISA + cfg-driven FSM） |
| [`docs/synthesis.md`](docs/synthesis.md) | Vivado OOC 综合（非主要关注点） |
| [`RTL代码编写原则.md`](RTL代码编写原则.md) | RTL 风格指南（4 大原则 + 例外清单） |
| [`model_analysis.md`](model_analysis.md) | 目标模型 PE 利用率分析 |

---

## 快速上手

```bash
# DMA 端到端 descriptor-driven 回归
cd sim/tb_core_dma
python run_regression.py                        # batch 14 case
python run_regression_stream.py                 # streaming 11 case (含 VGA 480x640)
```

---

## 当前状态

- **功能通路**：任意 K × stride × Cin × Cout 通（packed `K²·cin_slices ≤ 32` + chunked K=7），padding / 残差 / pooling 见 roadmap
- **Batch 回归**（tb_core_dma）：14/14 PASS
- **Streaming 回归**（tb_core_dma）：11/11 PASS，小图 + VGA 480×640 全绿

### Streaming 代表用例性能

| 用例 | 尺寸 | 核周期 | MAC 利用率 |
|---|---|---:|---:|
| K=3 C16C16 | 66×118 | 70,482 | 99.45% |
| K=7 C16C16 | 62×114 | 347,202 | 99.75% |
| K=3 C3C8 | **128×128** | 143,306 | 99.71% |
| K=3 C3C16 | **240×320** | 682,157 | 99.85% |
| K=3 C3C16 | **480×640 (VGA)** | 2,746,640 | **99.93%** |

**关键**：VGA 480×640 单图 4.9 MB，SRAM 只用 10 KB ring（`strip_rows=8 × W_IN=640`），一次 start 跑完，MAC 利用率 99.93%。

### Batch 代表用例性能（K=3 packed 区域）

| 用例 | Cycles | MAC 利用率 |
|---|---:|---:|
| K=3 C8C8 66×118 s=1 | 70,128 | 99.95% |
| K=3 C32C32 66×118 s=1 | 280,433 | 99.98% |
| K=5 C16C16 30×58 s=2 | 43,556 | 99.87% |

### 演进对比（K=3 C8C8 66×118 s=1）

| 版本 | Cycles | MAC 利用率 | 备注 |
|---|---:|---:|---|
| 原 Macro-ISA | 80,757 | 87.28% | 64-bit 指令总线，已退役 |
| cfg-driven FSM | 80,267 | 87.32% | 单 FSM 6 层循环，已退役 |
| v1 握手（串行 LOAD/STREAM） | 145,489 | 48.18% | 通路验证 |
| v2 (ring + parf overlap) | 74,878 | 93.61% | 消除 parf drain 反压 |
| **v3 batch (cross-round pipe)** | **70,128** | **99.95%** | 当前 batch 基线 |
| v3 streaming（+DMA 并发） | 70,482 | 99.45% | 单次 start，含 DMA 搬运气泡 |
