# FLUX CNN 加速器

FLUX_CNN 是一个用 SystemVerilog 实现的 CNN 加速器：**16×16 int8 MAC 阵列（256 MAC）**，由去中心化 5 模块 + valid-ready 握手流水编排，外挂 AXI4 DMA 子系统（IDMA / WDMA / ODMA + `axi_m_mux` + AXI-Lite CSR），通过 **row-ring streaming 数据路径**以单次 start 处理远超 SRAM 的大图。

---

## 架构要点

### 核流水（core pipeline）
- **256 个 MAC 并行**：16 列 × 16 PE，列间广播激活值（Output Channel Broadcast），每列一个输出通道
- **去中心化握手流水**：`line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer` 各自维护 counter，`valid-ready` 握手串联，无中心 FSM
- **PARF 拆分为 per-col 存储**：`parf_col` × NUM_COL + `parf_accum` 外壳；为 Kx-fold 的每列独立 wr_addr 偏移做准备
- **psum_reshape 归约级**：Kx-fold 时在 parf 和 SDP 之间做组合式 cout-group 求和，1-16 路灵活切换
- **cross-round pipeline**：line_buffer ring buffer 读/写指针模运算永不重置，整层只 2 拍 startup
- **FILL/DRAIN overlap**：parf_accum 的 tile N drain 隐藏在 tile N+1 fill 的 first_round 里
- **Elastic join 握手**：mac_array 两侧 ready 互相依赖对方 valid，stall 下 in-flight 数据不丢
- **统一 chunked 权重调度**（J-3）：`wgt_buffer` 只保留 cins-ahead 流水路径，packed 分支废弃；K=1 单 slot 场景用 1 拍 bubble 守护 2 拍流水 hazard

### DMA 子系统
- **AXI4 M（128-bit）外部 1 主口**：`axi_m_mux` 聚合 IDMA (MM2S IFB) + WDMA (MM2S WB，128→2048 bit packing) + ODMA (S2MM OFB) + 1 reserved
- **AXI-Lite Slave（32-bit CSR）**：`axi_lite_csr` + `cfg_regs` 寄存器组；host 下发配置 + DMA 描述符 + `CTRL` 启停位

### Streaming row-ring
- IFB / OFB 作 row-level ring buffer，按 `strip_rows` 取模
- **双向 row-credit 反压**：line_buffer 消费行 → 送 IDMA；ODMA 排空行 → 送 ofb_writer
- **forward-pressure**：line_buffer 仅在 `rows_available >= yout*stride + Ky` 时发射
- **并发**：一次 `CTRL=0xB` 同启 IDMA/核/ODMA，写/算/读同拍进行
- **任意 Cin/Cout**：J-1 起 batch/streaming 数据路径合并；Cin/Cout > 16 走多 slice 切片

### PE 利用率优化（K 阶段）
- **Ky-fold**：`Cin < PE_H` 时把 Ky 维折到 `cin_fake`（y 方向偏移复制到空闲 cin 行），PE 行填满。纯软件，零 RTL 改动。
- **Kx-fold**：`Cout < PE_W` 时把 Kx 维折到 `cout_fake`（systolic psum 按列组偏移写）。约束 `kxper % stride == 0` 保证所有列组 parity 对齐。
- **两者可叠加**，Ky-fold 用 PE 行、Kx-fold 用 PE 列，互不争抢资源。

---

## 目录导航

| 文档 | 内容 |
|------|------|
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 工作指引（含快速命令） |
| [`docs/architecture.md`](docs/architecture.md) | 核流水模块层次 + 数据通路 |
| [`docs/handshake-pipeline.md`](docs/handshake-pipeline.md) | 握手协议、推进事件、气泡分析 |
| [`docs/config-registers.md`](docs/config-registers.md) | cfg 寄存器位宽 + 语义 |
| [`docs/pe-fold.md`](docs/pe-fold.md) | Ky/Kx-fold 数学推导 + RTL 实现 |
| [`docs/streaming-row-ring.md`](docs/streaming-row-ring.md) | streaming 数据路径规范 |
| [`docs/simulation.md`](docs/simulation.md) | 仿真运行指南、TB 机制、回归测试 |
| [`docs/multi-layer-compilation.md`](docs/multi-layer-compilation.md) | PyTorch `nn.Sequential` 多层编译 |
| [`docs/descriptor-sequencer.md`](docs/descriptor-sequencer.md) | Descriptor + Sequencer 协议 |
| [`docs/roadmap.md`](docs/roadmap.md) | 未来工作（padding / 残差 / pooling / 多核） |
| [`docs/synthesis.md`](docs/synthesis.md) | Vivado OOC 综合（参考） |
| [`RTL代码编写原则.md`](RTL代码编写原则.md) | RTL 风格指南（4 大原则 + 例外清单） |
| [`model_analysis.md`](model_analysis.md) | 目标模型 PE 利用率分析 |

---

## 快速上手

```bash
# 端到端回归：22 case (ResNet-18 风格)
cd toolchain
python run_regression.py                      # 无 fold
python run_regression.py --fold               # Ky-fold 启用
python run_regression.py --fold --kx-fold     # Ky + Kx fold 叠加

# 单 case 生成
python gen_isa_test.py --k 3 --h_in 68 --w_in 120 --num_cin 8 --num_cout 8 --pad 1
cd ../sim/tb_core_dma && vsim -c -do run.tcl

# PyTorch 模型端到端
cd toolchain
python models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10
```

---

## 当前状态

- **功能通路**：任意 K × stride × Cin × Cout 通（J-3 起 packed/chunked 统一，走单一 cins-ahead 路径）
- **回归**：22/22 PASS (ResNet-18 风格 stem + L1-L4 + FC, 含 fold 变体)
- **PyTorch 编译器**：`nn.Sequential` 多层端到端 bit-exact；4 层 MNIST 全卷积仿真通过

### ResNet-18 风格端到端性能（22 层，960×540 输入）

| 配置 | 总 cycles | @100MHz | 加速 |
|---|---:|---:|---:|
| 原始（无 fold） | 8,454,897 | 84.5 ms (11.8 fps) | 1.0× |
| Ky-fold | 3,495,088 | 35.0 ms (28.6 fps) | 2.42× |
| **Ky + Kx fold** | **2,565,616** | **25.7 ms (39 fps)** | **3.29×** |

@ 400 MHz 外推：**6.4 ms / 156 fps**

### 关键层对比（Ky + Kx fold）

| 层 | 原始 cycles | Fold 后 | 加速 | MAC util |
|---|---:|---:|---:|---:|
| Stem K=7 C4C8 960×540 s=2 | 6,353,678 | 1,107,469 | 5.74× | **99.9%** |
| L1 K=3 C8C8 ×4 | 1,197,584 | 578,080 | 2.07× | 96.3% |
| L2 K=3 C16C16 120×68 | 75,988 | 75,988 | 1.00× | 96.7% |
| L3 K=3 C32C32 60×34 | 77,154 | 77,154 | 1.00× | 95.2% |
| L4 K=3 C64C64 30×17 | 78,934 | 78,934 | 1.00× | 93.0% |
| FC1 C256C512 1×1 | 9,843 | 9,843 | 1.00× | 5.2% |

### PE 利用率演进

| 阶段 | 加权平均 MAC util |
|---|---:|
| 原始（stem + L1 空间利用率瓶颈） | 22.1% |
| Ky-fold | 59.9% |
| **Ky + Kx fold** | **~77%** |

### VGA 480×640 端到端

单图 4.9 MB，SRAM 只用 10 KB ring (`strip_rows=8 × W_IN=640`)，一次 start 跑完，MAC 利用率 **99.93%**。
