# FLUX_CNN 项目工作梳理 (Phase 1 v2)

## 项目元信息

- **完成时间**：2026-05-06（v2 重跑，与 v1 archive 独立）
- **数据快照对应 commit**：`5fe16b2` (Feat: Phase 7 SMC+NUMA 主线 + IFB ring bug fix + xc7k325t N=4 综合通过)
- **STATUS.md 快照日期**：2026-05-05
- **literature.md 沿用**：v1 已通过 reviewer，未重写
- **标记符号语义**：
  - `[CHECK: 描述]` = 数据/事实待补（实测/查文档/查 RTL）
  - `[TBD: 描述]` = 写作路径/取舍待决策

## 一句话定位

FLUX_CNN 是一款 **Xilinx 7-series FPGA 上的 INT8 CNN 加速器**，核心特征是**去中心化 valid-ready 握手 + row-ring streaming 数据通路 + 编译器侧 PE 利用率优化（Ky-fold / S2D）**，端到端从 PyTorch 经自研轻量编译器编译至硬件 cfg + DDR 数据布局，单 start 跑完任意 H×W 图像，已扩展到 4 核 SMC + NUMA 多核架构在 ResNet11 上 sim 实测 2.51× 加速（真 Vivado SmartConnect IP）。

> 与 literature.md §C 的 streaming line-buffer 类（fpgaConvNet / Snowflake / Angel-Eye / Alwani Fused-layer / Kang AoCStream / Liu Full-Stack）同属 FPGA streaming 谱系，差异在于：(a) 单核 layer-serial 共用硬件而非 layer-pipelined 多 block；(b) row-level granularity 而非 fully-on-chip 或 layer-fused；(c) 编译器侧 Ky-fold + S2D 把 PE 利用率优化从硬件转移到编译器（与 literature.md §B 的 MAERI / Eyeriss-v2 硬件可重构路线对照）。

---

## 贡献清单

### 一、硬件架构层（Architecture）

#### C1.1 — 16×16 INT8 MAC 阵列 + 输出通道广播 (OC-broadcast) 数据流
- **层级**：架构级
- **内容描述**：256 个 INT8 MAC 排成 16 列 × 16 PE，列间广播激活值（每列对应一个输出通道，列内 16 PE 沿 Cin 方向并行）。每周期可消耗 1 拍 act 向量 + 16 列 weight，理论峰值 256 ops/cy × Fmax。
- **代码定位**：`RTL/mac_array.sv` + `RTL/mac_pe.sv`；常量 `NUM_COL=NUM_PE=16`，参数集中在 `params.py`。
- **验证途径**：sim/tb_core_dma 单核 26 case 全 bit-exact PASS（STATUS §1）。
- **实现状态**：✅ 已完成
- **量化数据**：综合 DSP 用量 82/256 PE（DSP 推断率低，详见 STATUS §1 已知问题，论文需点出）。
- **与 prior art 差异化**：与 literature.md §A 系统阵列类（Eyeriss / TPU 等）相似的 OC-broadcast 数据流，但通道宽度只有 16 而非 256；与 §B 可重构 PE 类（MAERI / Eyeriss-v2）路线不同——本工作选择**固定 16×16 + 编译器侧折叠**（见 C2.x）作为 Cin/Cout < 16 的低利用率应对方案。
- **诚实自评强度**：中（数据通路不算 novel，但与编译器折叠耦合后形成完整方案）。

#### C1.2 — 去中心化 valid-ready 5 模块流水（无中心 FSM）
- **层级**：架构级
- **内容描述**：核流水分 5 个模块（line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer），各模块自己维护 counter，前后通过 valid-ready 串联，**全程无中心 FSM**。stall 下 in-flight 数据不丢（elastic join），fill/drain overlap（tile N drain 隐藏在 tile N+1 first_round），cross-round pipeline（line_buffer 读/写指针模运算永不 reset）。
- **代码定位**：`RTL/line_buffer.sv` / `RTL/mac_array.sv` / `RTL/parf_accum.sv` / `RTL/ofb_writer.sv` / `RTL/wgt_buffer.sv`；docs/modules/ 各对应文档。
- **验证途径**：单核 11-case ResNet-18-like chain 仿真 + 24-case robust smoke 全 PASS（STATUS §1 / §2.5）。
- **实现状态**：✅ 已完成
- **与 prior art 差异化**：literature.md §C 中 fpgaConvNet / Snowflake 风格 streaming 流水通常每层一个独立 dataflow block，本工作把"去中心化"用在**单核 layer-serial 共用硬件**的内部 5 模块中，单层硬件可处理任意 H×W。
- **诚实自评强度**：中-强（去中心 + elastic join + cross-round pipe 三者组合是工程亮点，单独看每条都有 prior art）。

#### C1.3 — Streaming row-ring 行级环形缓冲（IFB / OFB）
- **层级**：架构级
- **内容描述**：IFB 8192 word / OFB 2048 word 都做成 **strip_rows × W_IN 粒度的环形 ring**，两端通过 row-credit 反压（line_buffer 消费行 → 释放给 IDMA；ODMA 排空行 → 让 ofb_writer 继续写）。Forward-pressure：line_buffer 仅在 `rows_available >= yout*stride + Ky` 时发射。一次 start 可处理远超 SRAM 容量的图（VGA 480×640 单图 4.9 MB，SRAM 仅用 10 KB ring，strip_rows=8 跑完整张）。
- **代码定位**：`RTL/line_buffer.sv` ring 指针 + `RTL/ofb_writer.sv` strip-aware 写出；`RTL/idma_ctrl.sv` / `RTL/odma_ctrl.sv` row-credit。
- **验证途径**：单核 26 case，含 W=8 极小 / W=33 奇数 / VGA 大图（STATUS §2.5 / §2.8）；run_robust_smoke 24/24 PASS。
- **实现状态**：✅ 已完成
- **量化数据**：strip 粒度可由编译器决定（hw_files.py `ofb_strip_rows_max = (OFB-1)//row_words`）；ResNet11 Patch 层 W=240 / H=135 也走单 start。
- **与 prior art 差异化**：literature.md §C Alwani Fused-layer（多层共享 line buffer 但需要硬件层流水绑定）、Kang AoCStream（buffer 容量 = activation tile）—— 本工作把 streaming 退化到**行级**而非整图或多层 fused，硬件不绑定层数，编译器决定 strip 粒度。**这是 Alwani / Kang 威胁项的主要差异化点**。
- **诚实自评强度**：中-强（核心叙事，论文必写）。

#### C1.4 — 分列累加器 PARF（parf_col × NUM_COL，外壳共享 wr_addr/we）
- **层级**：机制级
- **内容描述**：PARF（partial-sum register file）拆分为 16 个 per-column 实例（`parf_col` × NUM_COL），每列独立 SRAM、共享 wr_addr/we，避免 16 列共用一个大 SRAM 的端口冲突。每列容量 PARF=32（对应 cout slice 内 32 个 tile_w 的累加），支持 multi-slice cin 累加无需回写 DDR。
- **代码定位**：`RTL/parf_accum.sv` 外壳 + `RTL/parf_col.sv`。
- **验证途径**：包含在单核 26 case 中（多 slice cin 累加路径）。
- **实现状态**：✅ 已完成
- **与 prior art 差异化**：与 Eyeriss 系列的"per-PE psum register"不同，本工作 psum 累加在**per-column shared SRAM**，颗粒度介于 per-PE 和全局之间，匹配 OC-broadcast 数据流。
- **诚实自评强度**：中（实现细节，论文可作为机制级亮点。是否单独成节 [TBD: 看 paper 篇幅]）。

#### C1.5 — SDP 后处理融合（bias / shift / clip / residual）
- **层级**：机制级
- **内容描述**：在 ofb_writer 入口处 fuse SDP（Scale-Down + Post-process）链：bias 加（来自 bias_rf 经 RDMA 拉自 DDR）→ shift（量化）→ clip（饱和 INT8）→ residual fusion（shortcut_mult / shortcut_shift，shortcut bank 直读上一层 OFM）。打通 ResNet residual 通路无需 host 介入。
- **代码定位**：`RTL/ofb_writer.sv` 内 SDP 子模块；`bias_rf` + `Shortcut Bank` 在 STATUS §1 R.1/R.2 列出已完成。
- **验证途径**：链式 CASES 三层 ds 启用残差 bit-exact PASS（STATUS §1）；ResNet11 11 层 N=1/2/4 全 PASS 含 3 个 residual block（STATUS §2.8）。
- **实现状态**：✅ 已完成
- **已知问题**：SDP 量化组合链是 critical path，导致 Fmax 仅 68 MHz（未切流水），STATUS §1 已记录；论文需诚实写出。
- **诚实自评强度**：中（融合 SDP + residual + bias 是工程一致性，单独看 novelty 中等）。

#### C1.6 — 7 层循环嵌套硬件实现（cout_slice / cin_slice / yout / xout / kx-ky / pe）
- **层级**：架构级
- **内容描述**：硬件直接实现 conv 7 层循环（含 multi-slice cin/cout 切片层），各层 counter 由对应模块自维护：line_buffer 走 yout/xout，mac_array/wgt_buffer 走 kx/ky/cin_pe，parf_accum 走 cin_slice 累加，ofb_writer 走 cout_slice。无中心 FSM 协调。
- **代码定位**：`docs/slicing/` 下 cin/cout slice 机制文档；各模块 `RTL/*.sv` 内 counter 实现。
- **验证途径**：单核 26 case 覆盖 K∈{1,2,3,5,7}, stride∈{1,2,3,4}, cin/cout 任意（含 cin<16 unfold 路径）。
- **实现状态**：✅ 已完成
- **与 prior art 差异化**：与 §B 可重构数据流加速器需要 reconfig PE 网络不同，本工作循环嵌套**完全静态 mapping 到固定硬件**，编译器只调 counter 边界。
- **诚实自评强度**：弱-中（多数 dataflow 加速器都做循环嵌套，论文里更多是"完整性"陈述）。

### 二、编译器优化层（Compiler）

#### C2.1 — Ky-fold：Cin<16 时把 Ky 维折到 cin_fake（编译器侧零 RTL 改动）
- **层级**：机制级
- **内容描述**：当 Cin < PE_H=16 时，把 Ky（卷积 Y 方向）维度折叠到 cin_fake 维，每个 ky 偏移在编译器侧做"y 方向偏移复制"到空闲 cin 行。硬件视角看到的还是 cin_fake = Cin × Ky 的卷积，PE 行填满。**纯软件，零 RTL 改动**。
- **代码定位**：`toolchain/gen_isa_test.py --ky-fold` + `toolchain/hw_files.py` derive_layer_cfg；docs/pe-fold.md 数学推导。
- **验证途径**：run_regression.py --fold 跑 22-case 全 PASS（CLAUDE.md 中常用命令）；触发条件 scheduler.Layer.force_s2d/fold 自动判定（STATUS §2.8）。
- **实现状态**：✅ 已完成
- **量化数据**：[CHECK: Ky-fold 单独使能 vs baseline 的 PE 利用率提升数字，需跑 model_analysis.md 对应数据]
- **与 prior art 差异化**：据已知文献，**Ky 维折叠到 Cin 这种纯编译器侧、零硬件代价的 PE 利用率优化方案**在 FPGA streaming CNN 加速器中较少见。literature.md §B 路线（MAERI / Eyeriss-v2）通过硬件 reconfig 解决相同问题，需要 NoC 或 mesh 互连开销。本工作把这个权衡推到编译器侧，硬件保持最简固定阵列。
- **诚实自评强度**：偏强（**论文核心 novelty 候选之一**，可作为 PE-fold 单独章节；写作时使用"据已知文献..."而非"first to..."）。

#### C2.2 — Space-to-Depth (S2D)：stride≥2 时把 4 相位折到 cin（DDR 友好重排）
- **层级**：机制级
- **内容描述**：当 stride ≥ 2 时，把 (kx % stride, ky % stride) 的 stride² 个相位折到 `cin_new = stride² × Cin`，等价转换为 stride=1, K_new=ceil(K/stride) 的卷积。**编译器侧重排无需数据复制（DDR 友好）**。当 stride≥3 ∧ K≥stride 时 scheduler 自动启用。
- **代码定位**：`toolchain/gen_isa_test.py --s2d`；scheduler.Layer.force_s2d() / s2d_eff()；docs/pe-fold.md。
- **验证途径**：run_regression.py --s2d / --fold --s2d 全 PASS；ResNet11 Patch K=4 stride=4 → K=1 stride=1 cin=64 自动启用（STATUS §2.8）。
- **实现状态**：✅ 已完成
- **量化数据**：ResNet11 Patch 层单层 cycles：654,404 → 129,594（**5.05× 加速**），整网 N=1：1,115K → 596K（**1.87×**），N=4：750K → 355K（**2.11×**），FPS 250 → 564（STATUS §2.8）。
- **与 prior art 差异化**：[CHECK: S2D 在加速器领域的引用谱系待补查 — Pixel-Shuffle / Sub-pixel 是 super-resolution 文献，加速器侧近似 idea 是否被 prior art 覆盖需要查]。已知 literature.md 三处威胁项（Alwani / Kang / Liu）均不直接覆盖此变换。
- **诚实自评强度**：中-强（**配合 Patch 层 5× 数据非常有说服力**，但需要补查谱系再写 novelty 强度）。

#### C2.3 — Ky-fold + S2D 联合触发与编译器自动决策
- **层级**：机制级
- **内容描述**：scheduler.Layer 在编译期按 (Cin, K, stride, H, W) 自动决定 `force_s2d` / `force_fold`（stride≥3 ∧ K≥stride 走 S2D；Cin<16 ∧ K>1 走 Ky-fold；两者可叠加）。`build_step_cfg_dict` 用 `s2d_eff()` 返回的等效维度算 cfg，跟 gen_isa_test 内部 s2d 重排 ifb/wb 一致，确保编译器/硬件视角对齐。
- **代码定位**：`toolchain/scheduler.py` Layer.force_s2d() / s2d_eff()；`toolchain/run_multicore_chain.py` build_step_cfg_dict。
- **验证途径**：22 case × 3 mode（无 fold / --fold / --fold --s2d）× 单/多核全 PASS。
- **实现状态**：✅ 已完成
- **诚实自评强度**：中（联合触发是工程接合点，单独看不算强 novelty，但论文里需写以保证 C2.1/C2.2 故事完整）。

#### C2.4 — PyTorch → ISA 端到端编译流（nn.Sequential 多层 bit-exact）
- **层级**：工程级
- **内容描述**：`toolchain/models/run_model.py` 接受 PyTorch `nn.Sequential` 模型 + 输入图片，端到端编译至硬件 cfg + DDR 数据布局，**bit-exact 仿真验证**。覆盖 mnist_allconv 等模型（CLAUDE.md 常用命令）。`hw_files.derive_layer_cfg()` 是 cfg 派生 single source；`gen_isa_test.py` 是 derived 值的 source of truth。
- **代码定位**：`toolchain/models/run_model.py`、`toolchain/hw_files.py`、`toolchain/gen_isa_test.py`；docs/multi-layer-compilation.md。
- **验证途径**：mnist_allconv 等模型 bit-exact PASS（README "当前状态"）。
- **实现状态**：✅ 已完成
- **与 prior art 差异化**：literature.md §D 工具链类（fpgaConvNet 等）多采用 ONNX 中间表示，本工作直接 PyTorch nn.Module 走 hw_files cfg 派生，省去中间 IR；轻量但功能闭环。
- **诚实自评强度**：中（工程级亮点，论文用于支撑"端到端可部署"叙事）。

#### C2.5 — 链式 CASES 验证基础设施（DSL builder + 跨层 DDR FM 共享）
- **层级**：工程级
- **内容描述**：`run_regression.py` 用 Chain / _Node / resnet_block 7 行 DSL 写完 11 层 ResNet-18-like 网络。跨层 FM 共享 DDR region（不回写主存），bias / shortcut / wgt / desc 各走独立 DDR 段。Wall_us 端到端报告（host 启动 → host 可读 DDR）+ FPS 上限。
- **代码定位**：`toolchain/run_regression.py`；hw_files.write_multilayer_desc_list / DDRPlanner。
- **验证途径**：22 case × 3 mode 全 PASS；51 case ResNet residual + W slice + ResNet11 全 bit-exact（STATUS §2.8）。
- **实现状态**：✅ 已完成
- **诚实自评强度**：中（infra，论文 evaluation 章节支撑）。

### 三、系统集成层（System Integration）

#### C3.1 — AXI / DMA 子系统集成（Vivado axi_dm IP + 自研 ctrl + arb + mux + CSR）
- **层级**：工程级
- **内容描述**：DMA 子系统由 Xilinx `axi_dm` IP（Vivado 2023.1 生成）+ 自研 `idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl` + `mm2s_arb`（idma/wdma 串行仲裁）+ `axi_m_mux`（聚合 MM2S/S2MM/DFE）+ `axi_lite_csr`（host 寄存器侧）组成。**外部仅暴露 1 个 AXI4 Master + 1 个 AXI-Lite Slave**，方便集成到 Zynq / SoC PL 侧。
- **代码定位**：`RTL/DMA/idma_ctrl.sv` / `wdma_ctrl.sv` / `odma_ctrl.sv` / `rdma_ctrl.sv` / `mm2s_arb.sv`；`RTL/AXI4/axi_m_mux.sv`；`RTL/axi_lite_csr.sv`；`Syn/gen_axi_datamover.tcl`。
- **验证途径**：DataMover 性能与原版自写 DMA 持平 +0.5%（STATUS §1）；端到端 sim/tb_core_dma 全 PASS。
- **实现状态**：✅ 已完成
- **诚实自评强度**：弱-中（系统集成工程，单独看 novelty 弱，但是论文 system overview 必写）。

#### C3.2 — CFG_WRITE descriptor 配置流（host 仅 4 boot 寄存器写 + 1 start）
- **层级**：工程级
- **内容描述**：把 50+ 个 layer cfg 寄存器写打包进 DDR descriptor list，DFE（descriptor fetch engine）自动拉取并写 cfg_regs。**host AXI-Lite 写从 ~50 / 层降到 4 / 层**。多层 chain 时 host 一次 start_dfe + start_layer 跑完整层。新增 TYPE_CFG=0x3，desc_fifo 32→128 容纳一层完整 cfg。
- **代码定位**：`RTL/dfe.sv`（descriptor fetch engine）；`toolchain/hw_files.write_multilayer_desc_list`。
- **验证途径**：单核 / 多核 chain TB 全部用 CFG_WRITE descriptor 路径（STATUS §1 / §2.8）。
- **实现状态**：✅ 已完成
- **诚实自评强度**：中（host 介入降低对 deployment 友好，工程亮点）。

#### C3.3 — Done sticky 寄存器 + 双口 cfg_regs（csr_w + seq_w）
- **层级**：工程级
- **内容描述**：layer_done 上升沿 set + start_layer 自清，host poll STATUS[0] 或接 GIC level IRQ 都干净，避免 race。cfg_regs 双写口：host CSR 走 csr_w（boot 寄存器），DFE 走 seq_w（layer cfg），互不干扰。
- **代码定位**：`RTL/cfg_regs.sv` 双 port + done_sticky 逻辑。
- **验证途径**：单核 / 多核所有 TB（STATUS §1）。
- **实现状态**：✅ 已完成
- **诚实自评强度**：弱（工程细节，论文里 1 段带过即可）。

#### C3.4 — 多核 W 切片扩展（multicore_top + halo + 跨核 SRAM 直送 push）
- **层级**：架构级
- **内容描述**：`multicore_top` N=2/4 wrapper 包多核 + Xilinx axi_2to3 / axi_4to5 crossbar IP + axi_lite_1to4 host CSR fanout。**Mode C W slice**：把 W 维分给多核，每核处理 sub_W 段 + halo（computed redundancy halo, K=3 时 N=2 重叠 2 列）+ asymmetric pad。**跨核 SRAM 直送 (M2)**：producer ODMA 写 consumer IFB region（`0x8000_0000 + i × 0x1000_0000`），通过 `ifb_axi_slave` 写入 ring，无需经 DDR 中转。
- **代码定位**：`RTL/multicore_top.sv` / `RTL/multicore_top_smc.sv`；`RTL/AXI4/ifb_axi_slave.sv`（新）；`RTL/cfg_regs.sv` ADDR_SKIP_IDMA；`toolchain/scheduler.py` analyze_slicing。
- **验证途径**：20 个 W 切片 case 全 bit-exact PASS（N=2/4 各 10 case，覆盖 K∈{1,3,5,7}, stride∈{1,2}, W∈{8,32,33}, 单/多层；STATUS §2.8）；ResNet11 N=1/2/4 全 PASS（STATUS §2.8）。
- **实现状态**：✅ 已完成（Phase 7 SMC + NUMA 主线已合入：commit 5fe16b2）
- **量化数据**：ResNet11 N=2 1.32×、N=4 1.68× 加速（s2d 后；STATUS §2.8）；4-DDR PoC 4-核能到 1.81× / mac_pipe% 36→63.5%（STATUS §2.10）。
- **与 prior art 差异化**：literature.md §C 多数 streaming 加速器是单核或层间流水多核，本工作是**单层内 W 切片多核**，halo 用 computed redundancy（重叠输入列）而非数据 push（避免跨核同步开销）。
- **诚实自评强度**：中-强（论文核心 scaling 故事，与 1-DDR BW 上限的诚实数字配合很有说服力）。

#### C3.5 — 单源参数 params.py（RTL + Python 双向消费）
- **层级**：工程级
- **内容描述**：`params.py`（项目根）是 RTL/Python 唯一参数源，含 64 个 `\`FLUX_*` 宏（核心尺寸 + SRAM 容量 + AXI/CSR + 全局地址映射 + 58 CSR addr）。`python params.py` 自动生成 `RTL/flux_cnn_params.svh`；RTL `\`include` + `\`FLUX_*` 宏，Python `from params import *`。改硬件常量在一处改，工具链自动 propagate。
- **代码定位**：`params.py` + `RTL/flux_cnn_params.svh`；docs/params.md。
- **验证途径**：cfg_regs / core_top / multicore_top / hw_files / scheduler 已用，仿真全 PASS。
- **实现状态**：✅ 已完成
- **诚实自评强度**：弱-中（工程实践，论文 1 段带过；可作为 reproducibility 加分项）。

#### C3.6 — 多核 TB 结构化 profile 报告（LAYER_PROFILE / CORE_RESULT / DDR_PROFILE）
- **层级**：工程级
- **内容描述**：`tb_multicore_chain.sv` / `tb_multicore_4ddr_chain.sv` 加机读 profile 行（per-layer / per-core / per-DDR），跟 tb_core_dma 的 CASE_RESULT 同口径。axi_slave_mem 加 5 个计数器（aw_fire / w_beats / ar_fire / r_beats / busy_cyc），TB hier ref 拿 DDR 利用率。**`mac_pipe_pct = mac_fire / cycles`**（per-core stage-0 join 占比）。
- **代码定位**：`sim/tb_multicore/tb_multicore_chain.sv`；`sim/axi_slave_mem.sv`。
- **验证途径**：4-DDR PoC 数据（STATUS §2.10）就靠这套 profile 读出。
- **实现状态**：✅ 已完成
- **诚实自评强度**：中（论文 evaluation 章节工具支撑，是论文 BW 分析数字可信的基础）。

### 四、实测结果与综合数据（Measured Results）

> **数字标注约定**：来自 STATUS.md 的写明 §X.X 行号；论文最终需要重新跑一次回归对齐 commit `5fe16b2` 后再固化。

#### C4.1 — Vivado 综合（XC7K325T-FFG900-2 @ 100 MHz target）
- **层级**：量化级
- **数据**（单核，STATUS §1）：
  | 资源 | 用量 | FPGA 容量 | 占比 |
  |------|------|-----------|------|
  | LUT | 36,942 | 203,800 | 18.1% |
  | FF | 13,167 | 407,600 | 3.2% |
  | BRAM36 | 128 (+1 RAMB18) | 445 | 28.8% |
  | DSP48E1 | 82 | 840 | 9.8% |
  | Fmax | **68.4 MHz** (WNS=-4.618 ns) | — | **未达 100 MHz** |
- **代码定位**：`Syn/run_syn.tcl` / `Syn/run_syn_smc.tcl`；reports 在 `Syn/reports_smc/`。
- **实现状态**：✅ 已完成
- **诚实陈述**：(a) Fmax 仅 68 MHz，critical path = SDP 量化组合链；(b) DSP 推断率低（82/256），mac_pe 大部分综合到 LUT，加 `(* use_dsp = "yes" *)` 估能省 17K LUT。**论文必须诚实写出，不要藏**。
- **量化数据来源**：STATUS §1 单核综合表
- **诚实自评强度**：N/A（实测数据，无 novelty 评分）

#### C4.2 — 多核综合（XC7K325T，N=2 装得下，N=4 BRAM-bound 需缩 SB）
- **层级**：量化级
- **数据**（STATUS §2 / §1）：
  | 资源 | 单核 | 2 核 wrapper | 推算 3 核 | 推算 4 核 | 上限 |
  |------|------|--------------|-----------|-----------|------|
  | LUT | 36,942 | 74,386 (36.5%) | 109K (54%) | 146K (72%) | 203K |
  | BRAM36 | 128 | 256 (57.5%) | 384 (86%) | **512 ❌** | 445 |
  | DSP | 82 | 164 (19.5%) | 246 | 328 | 840 |
- **结论**：3 核是 XC7K325T 不动 SRAM 的硬上限（BRAM-bound）；commit `5fe16b2` 已通过 N=4 综合（缩 shortcut_bank 8192→2048）。
- **代码定位**：`Syn/run_syn_smc.tcl`；commit message "xc7k325t N=4 综合通过"。
- **实现状态**：✅ 已完成
- **量化数据**：[CHECK: 4 核 SMC 综合后的最终 LUT/FF/BRAM/DSP 实测数字，需查 `Syn/reports_smc/utilization_synth.rpt` 最新版]
- **诚实自评强度**：N/A

#### C4.3 — 单层 + 多层 chain bit-exact 回归（51 case 全 PASS）
- **层级**：量化级
- **数据**（STATUS §2.8 末尾合计）：
  - 26 单核 corner case（K∈{1,2,3,5,7}, stride∈{1,2,3,4}, pad∈{0,1,2,3}, cin∈{4,8,12,16,32}, cout∈{16,24,32}, H/W∈{1×1,15×17,28×28,33×33,64×32,120×68} 等）
  - 16 个 N=2/4 W slice case（10 N=2 + 10 N=4 单层/多层混合，不去重为 16 见 STATUS）
  - 6 ResNet residual chain case
  - 3 ResNet11 N=1/2/4 完整网络
- **合计**：[CHECK: STATUS §2.8 末尾自报 51 cases；最新 Phase 7 commit 自报 13/13 regression case + 11/11 layer 全 PASS。具体合计数最终核对：26+10(N=2)+10(N=4)+6+3=55，与 STATUS 自报 51 不一致，需用户最终核对]
- **代码定位**：`toolchain/run_regression.py`、`toolchain/run_robust_smoke.py`、`toolchain/run_multicore_chain.py`。
- **实现状态**：✅ 已完成
- **诚实自评强度**：N/A

#### C4.4 — ResNet11 整网 cycles / FPS（多核 + S2D 加速分解）
- **层级**：量化级
- **数据**（STATUS §2.8）：
  | 配置 | wall cycles | 加速 | FPS @ 100 MHz |
  |------|-------------|------|---------------|
  | N=1 baseline (no S2D) | ~1,115K | 1.00× | 168 |
  | N=1 + S2D | 596,088 | 1.87× | **313** |
  | N=2 + S2D | 450,469 | 1.32× (vs N=1+S2D) | 444 |
  | N=4 + S2D | 354,555 | 1.68× (vs N=1+S2D) | **564** |
- **关键拆解**（STATUS §2.8）：S2D 单 Patch 层 5.05×（654K → 130K cycles），整网 N=1 1.87× / N=4 vs no-S2D 2.11×。
- **代码定位**：`sim/tb_multicore/tb_multicore_chain.sv` LAYER_PROFILE / CORE_RESULT 输出；`toolchain/run_multicore_chain.py`。
- **实现状态**：✅ 已完成
- **诚实陈述**：FPS 用 100 MHz 假设算（实测 Fmax=68 MHz，所以**实际 FPS ≈ 0.68× 表中数字**，论文必须明示），STATUS §1 已自承 timing 未达 100 MHz。
- **诚实自评强度**：N/A
- **量化数据来源**：STATUS §2.8 ResNet11 N=1/2/4 表

#### C4.5 — N=4 多核近线性加速（1-DDR vs 4-DDR PoC 对比）
- **层级**：量化级
- **数据**（STATUS §2.10）：
  | 配置 | wall cycles | mac_pipe% | DDR busy |
  |------|-------------|-----------|----------|
  | 1-DDR baseline | 354,566 | 36.2% | 84.7% (单 DDR) |
  | 4-DDR PoC | 237,986 | 54.0% | 29-44% per DDR |
  | **4-DDR + force_multicore (BW 上限)** | **196,271** | **63.5%** | 40-44% per DDR |
- **关键 insight**：1-DDR 下 N=4 仅 1.68× 加速（mac_pipe% 仅 36%）= **DDR BW bound**；4-DDR 解墙后 wall -45%、mac_pipe% +27pp。论文应诚实陈述 BW 是限制并标记多 DDR 板 ROI 倾向（见 memory/project_4ddr_poc_result.md）。
- **代码定位**：`RTL/multicore_top_4ddr.sv`；`sim/tb_multicore/tb_multicore_4ddr_chain.sv`。
- **实现状态**：✅ 已完成（PoC 结论为不投多 DDR 板，4-DDR 仅 sim PoC）
- **量化数据来源**：STATUS §2.10
- **诚实自评强度**：N/A

#### C4.6 — Phase 7 SMC + NUMA ResNet11 N=4 完整网络 sim
- **层级**：量化级
- **数据**（STATUS 顶部 + commit 5fe16b2）：220,824 cycles，**453 fps**，11/11 layer 全 bit-exact，13/13 regression case 全 PASS。XC7K325T N=4 综合通过。
- **代码定位**：`RTL/multicore_top_smc.sv`；commit `5fe16b2`。
- **实现状态**：✅ 已完成（最新主线，commit `5fe16b2`）
- **量化数据**：[CHECK: 220,824 cy / 453 fps 是单一 commit 数字，需要用户确认是否作为论文最终数。453 fps 假设 100 MHz，实测 Fmax 待重新综合后给]
- **诚实自评强度**：N/A
- **量化数据来源**：STATUS §2.12 + commit `5fe16b2`

#### C4.7 — PE 利用率三模式对比（baseline / Ky-fold / S2D）
- **层级**：量化级
- **内容描述**：22-case ResNet-18 风格 case 在三种编译器 mode 下分别跑回归，对比 PE 利用率提升。
- **数据**：[CHECK: model_analysis.md 给出 ResNet11 整网 PE 利用率分析，但与 mac_pipe% 数字的口径需确认（mac_pipe% 是硬件 pipe 占比 ≠ PE 利用率）]
- **代码定位**：`toolchain/run_regression.py --fold` / `--s2d`；`model_analysis.md`。
- **实现状态**：[CHECK: 三 mode 是否都有最新数据]
- **诚实自评强度**：N/A
- **量化数据来源**：[CHECK: 等用户从 model_analysis.md 确认对应表格]

---

## 实现状态汇总

| 模块 / 子系统 | 完成度 | 风险 / 待决项 |
|---------------|--------|---------------|
| Core pipeline (line_buffer/mac_array/parf_accum/wgt_buffer/ofb_writer) | ✅ | Fmax 仅 68 MHz；DSP 推断率低 |
| DMA 子系统 (idma/wdma/odma/rdma_ctrl + axi_dm IP + mm2s_arb + axi_m_mux) | ✅ | DataMover 性能与原版自写 DMA 持平 +0.5% |
| AXI-Lite CSR + cfg_regs (双口 csr_w + seq_w) | ✅ | — |
| Done sticky / CFG_WRITE descriptor / desc_fifo 32→128 | ✅ | — |
| SDP 后处理 (bias / shift / clip / residual fusion) | ✅ | SDP 量化组合链是 critical path |
| Shortcut Bank + bias_rf (R.1 + R.2) | ✅ | N=4 综合 BRAM-bound 已缩 SB 8192→2048 |
| 编译器 Ky-fold / S2D / 联合触发 | ✅ | — |
| PyTorch 端到端编译流 | ✅ | mnist_allconv 等 bit-exact PASS |
| 链式 CASES / Wall_us 报告 / FPS 上限 | ✅ | — |
| 多核 W slice (Mode C) + halo + asymmetric pad | ✅ | — |
| 多核 cross-core SRAM 直送 (M2 push) | ✅ | — |
| Phase 7 SMC + NUMA ResNet11 N=4 | ✅ | commit `5fe16b2`，453 fps @ 100 MHz 假设 |
| 4-DDR PoC sim (BW 解墙验证) | ✅ (PoC) | ROI 决定不投多 DDR 板 |
| Mesh + AXIS NoC PoC (AIE-ML 风格) | 🔄 | Phase 6 Step E.1 PASS，Step F 多核 W slice + ResNet11 待做 |
| Mode C cout slice cfg gen | ⏳ | 未做（设计就绪） |
| Stage barrier 多 stage 调度 | ⏳ | 未做 |
| 片上 push 链 P2 (computed redundancy halo + 跨 stage push) | ⏳ | 设计就绪，2-3 天 |
| 单源参数 params.py (RTL/Python 双向消费) | ✅ | — |
| TB 结构化 profile (LAYER_PROFILE / CORE_RESULT / DDR_PROFILE) | ✅ | mac_pipe_pct 与 mac_util 是不同口径，论文写作需明示 |

## 关键性能数据（候选）

| 指标 | 数值 | 来源 | 置信度 |
|------|------|------|--------|
| 单核 LUT | 36,942 (18.1%) | STATUS §1 | 高（已综合）|
| 单核 BRAM36 | 128+1 (28.8%) | STATUS §1 | 高 |
| 单核 DSP48E1 | 82 (9.8%) | STATUS §1 | 高（但 DSP 推断率低，论文需点出）|
| 单核 Fmax | 68.4 MHz | STATUS §1 | 高（**未达 100 MHz 目标**）|
| 2 核 LUT | 74,386 (36.5%) | STATUS §2 | 高 |
| 2 核 BRAM36 | 256 (57.5%) | STATUS §2 | 高 |
| 4 核 综合通过 (XC7K325T) | ✅ commit 5fe16b2 | STATUS §0 + commit | [CHECK: 最新 LUT/FF/BRAM/DSP 数字] |
| ResNet11 N=1 + S2D | 596,088 cy / 313 fps | STATUS §2.8 | 中-高（FPS 用 100 MHz 假设）|
| ResNet11 N=2 + S2D | 450,469 cy / 444 fps | STATUS §2.8 | 中-高 |
| ResNet11 N=4 + S2D (1-DDR) | 354,555 cy / 564 fps | STATUS §2.8 | 中-高 |
| ResNet11 N=4 + S2D + 4-DDR PoC | 196,271 cy / 1015 fps | STATUS §2.10 | 中（仅 sim PoC）|
| Phase 7 SMC+NUMA N=4 | 220,824 cy / 453 fps | STATUS §2.12 + commit 5fe16b2 | [CHECK: 最新数字稳定性] |
| S2D Patch 单层加速 | 5.05× (654K→130K cy) | STATUS §2.8 | 中-高 |
| S2D 整网 N=1 加速 | 1.87× (1,115K→596K) | STATUS §2.8 | 中 |
| 24-case robust smoke | 24/24 PASS | STATUS §2.5 | 高 |
| ResNet11 N=1/2/4 含 residual | 51 case 全 bit-exact | STATUS §2.8 末尾合计 | [CHECK: 51 vs 55 数字不一致需核对] |
| mac_pipe% (1-DDR vs 4-DDR) | 36.2% → 63.5% | STATUS §2.10 | 高 |

> 任何含 [CHECK] 的数据，论文写作时需用户确认或重跑回归对齐 commit `5fe16b2`。

## 论文取舍建议

- **C2.1 Ky-fold 单独成节**[TBD]：是论文最强 novelty 候选。建议独立 1 节，配合 model_analysis.md 数据展示。措辞用"据已知文献..."。
- **C2.2 S2D**[TBD]：novelty 强度依赖谱系查证。Patch 层 5.05× 数字非常有说服力，但 prior art 关系需要 paper-literature-scout 补查（Pixel-Shuffle / Sub-pixel 在加速器的引用谱系）。
- **C3.4 多核 W slice**[TBD]：是否单独成节取决于会议偏向：工程偏向（FPL / FCCM）值得单写，架构偏向（HPCA / ISCA）则与 BW 分析合并。
- **DMA 子系统**[TBD]：axi_dm 集成方案是否单列一节，取决于目标会议是工程偏向还是架构偏向。架构偏向论文可以仅 1 段带过。
- **Fmax 68 MHz / DSP 推断率低**：**必须诚实写出**，作为 future work（SDP 切流水可拉到 100+ MHz / `(* use_dsp *)` 综合属性）。
- **4-DDR PoC**[TBD]：是否写进论文取决于篇幅。可作为 limitation analysis（"BW 是 N=4 加速比下降的原因"）的支撑数据。
- **Mesh + AXIS NoC PoC**[TBD]：Phase 6 仍在 Step F，**论文中不要 claim 完成**，可作为 future work 或附录的 forward-looking 章节。
- **`mac_pipe%` 与 `mac_util` 口径区分**：论文 evaluation 节必须明示口径定义（per-core stage-0 join ratio vs ops/(cy×256)），STATUS §2.9 已澄清。

## 待决问题清单

| ID | 描述 | 类型 | 处置建议 |
|----|------|------|---------|
| Q1 | C2.1 Ky-fold 是否单独成节 | TBD | 用户决策；倾向单独成节 |
| Q2 | C2.2 S2D 在加速器领域的引用谱系 | CHECK | 请 paper-literature-scout 补查 Pixel-Shuffle / Sub-pixel 在 FPGA 加速器中的近似 idea |
| Q3 | C4.2 4 核 SMC 综合最新 LUT/FF/BRAM/DSP 数字 | CHECK | 查 `Syn/reports_smc/utilization_synth.rpt` 最新版 |
| Q4 | C4.3 51 vs 55 case 合计数不一致 | CHECK | 用户最终核对 STATUS §2.8 末尾自报 |
| Q5 | C4.6 Phase 7 SMC+NUMA 220,824 cy / 453 fps 是否作为论文最终数 | CHECK | 用户确认；可能需要重新跑回归对齐 commit `5fe16b2` |
| Q6 | C4.7 三模式 PE 利用率对比的具体数字 | CHECK | 从 model_analysis.md 确认对应表格 |
| Q7 | Fmax 实测 vs 100 MHz 假设 FPS 的写作策略 | TBD | 倾向论文双标：100 MHz 假设给 ceiling，68 MHz 实测给 actual |
| Q8 | C2.1 / C2.2 与 literature.md §B 可重构 PE 路线（MAERI/Eyeriss-v2）的对比深度 | TBD | 视目标会议；架构偏向论文可深入对比，工程偏向可 1 段带过 |
| Q9 | Mesh + AXIS NoC PoC 是否进论文 | TBD | 倾向 future work / 附录，不进 main contributions |
| Q10 | 多 DDR 板 ROI 决策（不投）是否进论文 | TBD | 可作为 limitation 章节数据支撑 |
| Q11 | DMA 子系统是否单独成节 | TBD | 视会议偏向 |
| Q12 | C3.4 多核 W slice 是否单独成节 | TBD | 视会议偏向 |
