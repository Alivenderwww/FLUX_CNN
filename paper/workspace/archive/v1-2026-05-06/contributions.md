# FLUX_CNN 项目贡献清单 (Phase 1)

> Phase 1 工作梳理。每条贡献必须可追溯到代码 + 实测数据；强度自控避免过强 claim。
> **快照对应**：`STATUS.md` 时间戳 2026-04-30；当前 git branch `main`，HEAD 见 git log `b158cab`（链式 CASES + CFG_WRITE descriptor + done sticky + 墙钟报告）。

---

## 项目元信息
- 完成时间：2026-04-30
- 项目根：`c:/_Project/FLUX_CNN/`
- 数据快照对应 commit：`b158cab` (`Refactor: 链式 CASES + CFG_WRITE descriptor + done sticky + 墙钟报告`)
- STATUS.md 快照日期：2026-04-30

## 一句话定位

FLUX_CNN 是一个 **16×16 INT8 MAC 阵列 + 去中心化 valid-ready 流水 + row-ring streaming + AXI4 DataMover IP 子系统** 的 CNN 加速器，硬件保持简洁、由编译器（Ky-fold + S2D）填满 PE，单次 `start` 跑完任意 H×W；当前 N=1/2/4 多核版在 XC7K325T 综合通过且仿真 bit-exact。

> 与 literature.md A 类（Gemmini@DAC'21、NVDLA、Eyeriss@ISCA'16）以及 B 类（MAERI@ASPLOS'18）共谱系；与 C 类 row-streaming 工作（Alwani@MICRO'16、Kang AoCStream@arXiv'22 / Sensors'23、Liu Full-Stack@TNNLS'21）属同一类 streaming 解法但具体取舍不同。

---

## 1. 硬件架构贡献

### C1.1：去中心化 valid-ready 流水（核内无中心 FSM）

- **层级**：架构级 / 工程级
- **内容描述**：核流水（`line_buffer → wgt_buffer → mac_array → parf_accum → ofb_writer`）共 5 个模块，**每模块自维护独立 counter**，模块间用 `valid-ready` 双向握手串联。无中心 controller / scheduler，模块间不靠主时序图同步。`elastic join` 握手保证 stall 下 in-flight 数据不丢；`FILL/DRAIN overlap` 让 `parf_accum` tile N 的 drain 隐藏在 tile N+1 fill 的 first_round 里。
- **代码定位**：`RTL/line_buffer.sv` / `RTL/mac_array.sv` / `RTL/parf_accum.sv` / `RTL/parf_col.sv` / `RTL/ofb_writer.sv` / `RTL/wgt_buffer.sv`；`RTL/sequencer.sv` 仅做 cross-block 启动同步而非中心调度
- **验证途径**：`sim/tb_core_dma/` 22-case 链式回归全 PASS（STATUS §1）
- **实现状态**：✅ 已完成
- **量化数据**：22-case ResNet-18 风格链 86.6% MAC%（来自 STATUS line 355；[CHECK: 单层 vs 整网 MAC% 口径区分待用户确认]）
- **与文献的差异**：与 Gemmini@DAC'21（中心 RoCC scheduler）/ NVDLA（CDMA→CMAC→CACC→SDP 显式 controller）/ VTA@IEEE Micro'19（4 段 fetch/load/compute/store + two-level ISA）形成对比；Buffets@ASPLOS'19 提供"buffer + counter + handshake"的形式化语言，FLUX_CNN 是该思想的 RTL 落地。**新颖性强度自评：工程化贡献，并非首次提出，但作为完整 5 模块端到端验证的开源参考有价值**

### C1.2：Streaming row-ring 数据路径 + 双向 row-credit 反压

- **层级**：架构级
- **内容描述**：IFB / OFB 作 row-level ring buffer，按 `strip_rows` 取模；line_buffer 仅在 `rows_available >= yout·stride + Ky` 时发射（forward-pressure），ODMA 排空行 → 送 ofb_writer credit；sequencer 一次 dispatch 同启 IDMA/核/ODMA，三阶段并发。这让**整图不必装进片上 SRAM**（VGA 480×640 / 4.9 MB 单图只用 ~10 KB ring，`strip_rows=8 × W=640`）。"统一 streaming row-ring" 模式同时覆盖原 batch 模式（环容量 ≥ 整图的退化情形）。
- **代码定位**：`RTL/line_buffer.sv`（ring wptr/rptr 模运算永不 reset）、`RTL/ofb_writer.sv`（ring writer）、`RTL/DMA/odma_ctrl.sv`（rows_consumed credit 回送）、`RTL/sequencer.sv`（三段同启）
- **验证途径**：22-case 回归 + `toolchain/run_robust_smoke.py` 24 case（H/W 含 1×1 FC、15×17 奇数、120×68 大图，全 PASS @ 100k cycles 内，STATUS line 313–321）
- **实现状态**：✅ 已完成
- **量化数据**：VGA 480×640 单图 4.9 MB / SRAM 仅 10 KB ring（README）；22+24=46 case 全 bit-exact PASS
- **与文献的差异**：**最近邻 prior art 是 Kang AoCStream@arXiv'22/Sensors'23（line buffer 大小线性于图宽）+ Alwani Fused-layer@MICRO'16（跨层 fusion 95% 流量节省）+ Liu Full-Stack@TNNLS'21（streaming + residual）**。FLUX_CNN 的差异点：(a) **row-level 而非 layer-level granularity**——只缓几行而不是几层 feature map；(b) **layer-serial 单核共用硬件**而非 layer-pipelined 多 block；(c) 承认外存必然存在并优化 DDR 流量（Kang 强调全片上）。**新颖性强度自评：claim 偏弱——streaming 思想已属公开技术，差异化在工程取舍上，论文必须在 Related Work 精细论证不当独创点**

### C1.3：PARF 拆分为 per-col SRAM（parf_col × NUM_COL + 共享外壳）

- **层级**：机制级
- **内容描述**：`parf_accum` 内部不是单一 SRAM，而是 `parf_col` × 16 列独立 SRAM，每列独立寻址但共享 wr_addr/we/rd_addr 的外壳逻辑。这降低了"16 个 cout 列同时累加"的 SRAM 端口数压力，让每列可用 single-port BRAM。
- **代码定位**：`RTL/parf_accum.sv` (外壳) + `RTL/parf_col.sv` (单列)
- **验证途径**：综合通过 + 22-case 回归
- **实现状态**：✅ 已完成
- **量化数据**：[CHECK: parf BRAM 数量贡献单独估算待补——单核 128 BRAM 总量见 STATUS line 35]
- **与文献的差异**：常规做法是单一 PSUM 大 SRAM（NVDLA / Gemmini）；FLUX_CNN 选择 per-col 拆分以匹配"列广播激活 + 列独立 cout 输出"的数据流。**新颖性强度自评：工程化优化点，作为单元级技巧值得记录，论文中应作为"实现细节"出现而非主贡献**

### C1.4：SDP 后处理：Bias / Residual / Shift / ReLU / Clip 融合（NVDLA-inspired，R.1+R.2 重构）

- **层级**：机制级
- **内容描述**：Single Data Point processor 把 5 步后处理融合在一段组合链：`pipe_psum_reg → mult → shift → +zp → ReLU → clip → trunc`。R.1 把 bias 从 mac_array 内移到 SDP 段（解 MAX_COUT_SLICES=32 限制并解放 bias_rf 容量）；R.2 加 `shortcut_mult / shortcut_shift` 可编程量化因子做残差融合。Shortcut Bank（8192×128 SRAM）专用驻留残差源。
- **代码定位**：`RTL/sdp.sv` / `RTL/bias_rf.sv` / `RTL/DMA/rdma_ctrl.sv`（残差读 DMA）；`memory/sdp_residual_fusion.md` + `memory/bias_to_sdp_refactor.md` 设计文档
- **验证途径**：链式 CASES 三层 ds 启用残差，bit-exact 通过（STATUS §1，line 24）；R.1+R.2 上线后 22-case 全过
- **实现状态**：✅ 已完成
- **量化数据**：8192×128 Shortcut Bank → 32 BRAM（STATUS line 43）
- **与文献的差异**：NVDLA 的 SDP 段命名/分段法直接借用（README 与 literature.md E 类已点明）；Liu Full-Stack@TNNLS'21 也是 streaming + residual fusion；He@CVPR'16 ResNet 是该 fusion 必然性的算法侧动机。**新颖性强度自评：偏增量——但 R.1 的 bias 重定位（解 32 cout slice 限制）+ R.2 的 shortcut_mult/shift 可编程量化因子是有价值的具体优化。论文中作为 "system 完整性" 论据**

### C1.5：路径上 Critical Path 已知问题（写进论文需诚实标注）

- **层级**：实现状态披露
- **内容描述**：当前 SDP 量化组合链未流水化，Fmax = 68.4 MHz @ XC7K325T-2，未达 100 MHz target；mac_pe 未加 `(* use_dsp = "yes" *)` 属性，82/256 个 PE 实际综合到 LUT 而非 DSP。STATUS §4 列出修复路径（"加 use_dsp 属性 → 释放 17K LUT"、"切 SDP 流水线 → 100+ MHz"）。
- **代码定位**：`RTL/sdp.sv` / `RTL/mac_pe.sv`；`memory/dsp_inference.md`
- **验证途径**：综合报告（`Syn/run_syn.tcl` 输出，timing WNS=-4.618 ns）
- **实现状态**：🔄 已识别未修复
- **与文献的差异**：诚实披露这一项有助于评审人理解工程取舍——"Fmax 未达 100 MHz"会被审稿人挑战，必须在论文中给出"修复路径已知"+"目标会议受众接受 work-in-progress 程度"的说明


## 2. 编译器优化贡献

### C2.1：Ky-fold（Cin<16 时把 Ky 折到 cin_fake，零 RTL 改动）

- **层级**：机制级 / 编译器
- **内容描述**：当卷积 Cin<16 让 PE 行不能填满时，编译器侧把 Ky 维按 `groups_y = HW_PE/Cin` 分组折到 cin 维，定义虚拟 cin 通道 `g·Cin + c`、Ky 缩到 `kyper = ceil(K/groups_y)`、cin_fake = groups_y · Cin。**输入做 y-方向偏移复制 + 权重做相应 reshape**，硬件按普通 conv 跑（K_new=kyper, Cin=cin_fake），完全无感。代价是 IFB 占用 × groups_y。
- **代码定位**：`toolchain/hw_files.py::compute_fold_params(K, Cin, HW_PE)` / `fold_input()` / `fold_weights()`；`toolchain/gen_isa_test.py --ky-fold`；`docs/pe-fold.md` §1 完整数学推导
- **验证途径**：`run_regression.py --fold` 22 case 全 PASS；`docs/pe-fold.md` 表格列出受益层（K>1 AND Cin<16）
- **实现状态**：✅ 已完成
- **量化数据**：`model_analysis.md` Layer 1 (K=7, Cin=4, Cout=8) 原 PE 利用率 12.5%；`run_regression.py --fold` 启用后接近 100%（[CHECK: 实测 PE 利用率百分比待 project-analyst 跑 fold 模式 case 时取 cycles 算出]）
- **与文献的差异**：与 cuDNN@arXiv'14 im2col 同源但**更轻量**——只折 Ky 不展开 Kx，保留 Cin 通道并行，避免 im2col 的全展开内存炸裂。与 MAERI@ASPLOS'18（硬件 ART 网络）/ Eyeriss-v2@JETCAS'19（硬件 NoC reconfiguration）形成"compiler-only vs hardware-reconfigurable"对比。**新颖性强度自评：claim 偏强**——literature.md §D 检索未命中"FPGA 上把 Ky 折到 cin 作为 PE 利用率优化"的具体 prior art

### C2.2：Space-to-Depth (S2D)（stride≥2 时 4 相位折到 cin，编译器侧重排不复制）

- **层级**：机制级 / 编译器
- **内容描述**：stride≥2 的卷积，按 `(kx%stride, ky%stride)` 把 stride² 个相位折到 cin 维。等价为 stride=1, K_new=ceil(K/stride), Cin_new=stride²·Cin 的卷积。**编译器侧只重排不复制**，DDR 友好；启用后 stride=1 + ARF reuse_en=1 滑动窗口复用，IFB 读次数大幅下降。K 被 stride 整除时 pad_waste=0（如 K=8 stride=2，4 相位 = 4×4 sub-kernel 完美对齐）。
- **代码定位**：`toolchain/hw_files.py::compute_s2d_params(K, Cin, stride)` / `s2d_input()` / `s2d_weights()`；`toolchain/gen_isa_test.py --s2d`；`docs/pe-fold.md` §2 完整数学推导
- **验证途径**：`run_regression.py --s2d` 22 case 全 PASS；K=8 / stride=4 / pad=4 等极端 case 通过
- **实现状态**：✅ 已完成
- **量化数据**：相比 Ky-fold 的 groups_y 倍 IFB inflation，S2D 等量重排（不复制），**多核场景 DDR 带宽节省明显**（具体节省比例 [CHECK: 待跑对照 run]）
- **与文献的差异**：S2D 的"sub-pixel rearrangement"算法侧来自 Shi et al.@CVPR'16（Sub-Pixel Convolutional Networks）；FLUX_CNN 的贡献是把它**作为 compiler-pass 应用到任意 stride≥2 的预训练 conv**——不需要重训模型，与"训练时设计 sub-pixel layer"的根本区别。**`literature.md` 已标注 [CHECK: S2D 在加速器领域的引用谱系不清晰]**——FLUX_CNN 的"compiler-pass S2D"看起来是新点，但 reviewer 阶段需补查 ASPLOS/HPCA 中 architecture-side S2D 的更早出处。**新颖性强度自评：claim 中等偏强**——但需 reviewer 阶段补查防 prior art 突袭

### C2.3：自动决策 + 受益层判定逻辑

- **层级**：编译器策略
- **内容描述**：`run_regression.py` 自动决策 fold 启用条件：Ky-fold 触发于 `K>1 AND Cin<16`；S2D 触发于 `stride≥2 AND K≥stride`；S2D 启用后 Cin 变为 stride²·Cin，重新判定 Ky-fold（多数情况 S2D 后 Cin'≥16 不再需 Ky-fold）。
- **代码定位**：`toolchain/run_regression.py`；`docs/pe-fold.md` §5 触发表
- **验证途径**：22-case 链式回归，三种模式（无 fold / `--fold` / `--fold --s2d`）皆 PASS
- **实现状态**：✅ 已完成
- **新颖性强度自评：工程化贡献，作为"compiler 自动选择"细节出现**

### C2.4：6 + 1 层硬件循环嵌套 + cin/cout 多 slice 切片（任意 Cin/Cout）

- **层级**：编译器机制
- **内容描述**：物理 16×16 阵列处理任意 Cin/Cout，靠两层切片：`cin_slices = ⌈Cin/16⌉`（时间维累加，靠 PARF 合并）+ `cout_slices = ⌈Cout/16⌉`（时间维 OFB 行内段 NHWC 拼接）。整层硬件循环是 `for yout > for cs > for tile > for cins > for ky > for kx > for iss_pos`（7 层嵌套）。`derive_layer_cfg` 做容量校验（IFB / WB / OFB），超容量自动切 strip（H 方向行环）+ tile（W 方向）。
- **代码定位**：`toolchain/hw_files.py::derive_layer_cfg`；`docs/slicing/README.md`（完整文档）；硬件侧 `RTL/sequencer.sv` 嵌套计数器
- **验证途径**：22-case 含 Cin∈{4..256}, Cout∈{8..256}（model_analysis 表 §2 全部 14 层都覆盖了），全 PASS
- **实现状态**：✅ 已完成
- **与文献的差异**：与 Interstellar@ASPLOS'20（7 nested loops over DNN）形式化天然同构；Ma et al.@FPGA'17（4-6 维循环 systematic study）是同语言的早期工作。FLUX_CNN 给出一个具体硬件落地点。**新颖性强度自评：偏增量**——loop tiling 已属公开技术，FLUX_CNN 的具体 7 层嵌套方案与文献基本一致，可作"具体实现"出现

### C2.5：PyTorch nn.Sequential 多层端到端编译（bit-exact）

- **层级**：工具链
- **内容描述**：`compile_layer.py`（PyTorch Conv2d → 硬件 cfg + 数据文件）+ `compile_model.py`（多层链式编译，`_plan_ddr` 分配 FM-shared DDR 区）+ `run_regression.py`（DSL builder：Chain / _Node / resnet_block 7 行写完 11 层 ResNet block）。整网 bit-exact 通过。
- **代码定位**：`toolchain/compile_layer.py` / `compile_model.py` / `run_regression.py` / `hw_files.py`（DDR 文件 I/O 共享层 + CFG_ADDR_MAP）
- **验证途径**：11-case ResNet-like chain 全 PASS @ 593K cycles, 86.6% MAC%, 端到端 5.95 ms (≈168 fps 上限 @ 100 MHz target)（STATUS line 355）
- **实现状态**：✅ 已完成
- **量化数据**：11 cases 全 PASS, 整网 593K cycles, 86.6% MAC%, 5.95 ms @ 100 MHz target → 168 fps 上限 [CHECK: target vs Fmax 68.4 MHz 实际 fps 应为 168×0.684 ≈ 115 fps，论文须统一两口径]
- **与文献的差异**：TVM@OSDI'18 / VTA@IEEE Micro'19 / fpgaConvNet@TNNLS'19 提供更通用的端到端编译栈；FLUX_CNN 的 hand-rolled mini-compiler scope 远窄于 TVM，但配 22-case 链式回归提供完整可复现验证。**新颖性强度自评：工程化贡献**——作为 system completeness 论据出现而非主贡献


## 3. 系统集成贡献

### C3.1：DMA 子系统采用 Xilinx axi_dm IP + 轻量 *_ctrl 控制器

- **层级**：工程级 / 系统集成
- **内容描述**：原项目自写 IDMA/WDMA/ODMA RTL（~3000 行）替换为 Xilinx `axi_dm` (AXI DataMover) IP + 轻量 `idma_ctrl / wdma_ctrl / odma_ctrl / rdma_ctrl`（仅做 cmd 生成 + done 检测的控制端）+ `mm2s_arb`（IDMA/WDMA 共享 MM2S 的串行仲裁）。外部看是 1 个 AXI4 Master + 1 个 AXI-Lite Slave。
- **代码定位**：`RTL/DMA/idma_ctrl.sv` / `wdma_ctrl.sv` / `odma_ctrl.sv` / `rdma_ctrl.sv` / `mm2s_arb.sv` / `dfe.sv`（descriptor fetch engine）；`Syn/gen_axi_datamover.tcl`（IP 生成）；`RTL/AXI4/axi_m_mux.sv`（聚合 axi_dm.MM2S/S2MM + DFE 到外部单 master 口）
- **验证途径**：`sim/tb_axi_dm_smoke/`（IP smoke）+ `sim/tb_idma_ctrl/`（cmd 控制端联合自测）+ `sim/tb_core_dma/` 端到端
- **实现状态**：✅ 已完成
- **量化数据**：DataMover 性能与原版自写 DMA 持平 (+0.5%)（STATUS line 21；commit dd114ab "Refactor+Perf: *_dm → *_ctrl 重命名 + mm2s_arb 流水化 + 关 DataMover SF"）；DataMover burst_size 16→256 后性能基本回到原版（commit f238de1）
- **与文献的差异**：vendor IP 在 FPGA-CNN 加速器集成中的作用经常被低估；FLUX_CNN 的 system view 显示，把 DMA 全部交给 vendor IP、自己只做轻量 controller 就能拿到 vendor-grade 吞吐。Xilinx PG022 是 vendor doc 引用方式 [CHECK: vendor doc 引用方式]。**新颖性强度自评：工程化贡献**，作为"system 实用性"的论据

### C3.2：CFG_WRITE descriptor + descriptor-driven 配置流（CFG burst 4/层）

- **层级**：工程级 / 系统集成
- **内容描述**：原架构 host 通过 AXI-Lite 写 ~50/层 cfg 寄存器；R 重构后 host 只写 4 个 boot regs，50 个 layer cfg 全走 **TYPE_CFG=0x3 descriptor**（CFG_WRITE descriptor）由 DFE 拉 descriptor list 自动写。`desc_fifo` 32→128。
- **代码定位**：`RTL/desc_fifo.sv` / `RTL/DMA/dfe.sv`（descriptor fetch engine）/ `RTL/cfg_regs.sv`（双端写口：csr_w + seq_w）/ `RTL/AXI4/axi_lite_csr.sv`；`memory/cfg_write_descriptor.md` 设计文档
- **验证途径**：22-case 回归 + 链式 CASES 跨层 cfg 切换 PASS（STATUS line 26）
- **实现状态**：✅ 已完成
- **量化数据**：host AXI-Lite 写从 ~50/层 降到 4/层
- **与文献的差异**：NVDLA 类似有"register list" descriptor 模式；FLUX_CNN 的具体实现把 descriptor 类型扩展到 CFG_WRITE 而非仅 IFB/WB/ODMA buffer 描述符。**新颖性强度自评：工程化贡献**

### C3.3：链式 CASES（DSL builder + 跨层 DDR FM 共享 + 墙钟报告）

- **层级**：工程级 / 验证基础设施
- **内容描述**：`run_regression.py` 内 `Chain / _Node / resnet_block` DSL，7 行 Python 写完 ResNet 11-layer 风格 chain；`validate_chain` + `plan_chain_ddr` 自动分配 FM-shared DDR 区域；regression report 加 `Wall_us` 列（host 启动 → host 可读 DDR 端到端时延）+ footer FPS 上限。
- **代码定位**：`toolchain/run_regression.py` / `toolchain/hw_files.py::write_multilayer_desc_list`；commit `b158cab` "链式 CASES + CFG_WRITE descriptor + done sticky + 墙钟报告"；`memory/chained_cases_design.md`
- **验证途径**：11-case 链全 PASS @ 593K cycles
- **实现状态**：✅ 已完成
- **新颖性强度自评：工程化贡献**——验证基础设施完整性的体现

### C3.4：Done sticky 寄存器 + 双端 cfg 写口

- **层级**：机制级
- **内容描述**：`layer_done` 上升沿 set 到 STATUS[0] sticky 位，`start_layer` 下降沿自清；host 用 polling 或 GIC level IRQ 都干净。`cfg_regs` 双端写口（csr_w 给 host AXI-Lite + seq_w 给 sequencer 内部 ack）解 race。
- **代码定位**：`RTL/cfg_regs.sv`；`memory/done_sticky_pattern.md` 设计文档
- **验证途径**：22-case 链式 chain 跨层切换不丢 done
- **实现状态**：✅ 已完成

### C3.5：多核 wrapper（multicore_top）+ 跨核 SRAM 直送 + N=4 综合通过

- **层级**：架构级 / 系统集成
- **内容描述**：M1 阶段实现 `multicore_top.sv` 参数化 N 核 wrapper，N=2 综合通过、仿真 bit-exact PASS @ 9057 cycles（vs 单核 8808 cycles 的 +2.8% AXI 仲裁开销）。M2 阶段升级 `axi_2to1` IP → `axi_2to3`（NUM_SI=2 NUM_MI=3，MI[0]=DDR + MI[i+1]=Core[i] IFB）实现**跨核 SRAM 直送**（producer ODMA 写 0x8000_0000 + consumer_id × 0x1000_0000，crossbar 路由到 consumer 的 ifb_axi_slave 模块经 ring 反压写 IFB SRAM）。M2.5 阶段加 N=4（`axi_4to5` + `axi_lite_1to4`），20/20 W slice case 全 bit-exact PASS（K∈{1,3,5,7}, stride∈{1,2}, W∈{8,32,33}, 单/多层, mode A/W slice 混合, LPT 并行 stage）。
- **代码定位**：`RTL/multicore_top.sv` / `RTL/AXI4/axi_arbiter.sv` / `axi_m_mux.sv` / `ifb_axi_slave.sv`（M2 新增 AXI4 SI → SRAM 写口）；`Syn/gen_multicore_ip.tcl`；`sim/tb_multicore/`；`memory/cross_core_design.md` / `m2_cross_core_pipeline.md`
- **验证途径**：M1.5 multicore DDR-mode TB PASS @ 9057 cycles；M2 双层 chain (32×32×16 → conv3×3 → 32×32×16 → conv3×3 → 32×32×16) PASS @ 11006 cycles, Layer 1 OFB 1024 word bit-exact 匹配 golden；M2.5 N=4 W slice 20/20 PASS（详见 STATUS §2.8 表）
- **实现状态**：✅ N=2/N=4 已完成；🔄 ResNet 11 layer 多核适配 (residual + h/w 维度变化) 待做（STATUS line 254 估 1-2 天）
- **量化数据**：N=2: 36.5% LUT / 57.5% BRAM @ XC7K325T；N=3 推算 BRAM 86%（边缘）；N=4 推算 BRAM 超容需缩 shortcut bank。Scheduler 估算 ResNet 11-layer N=2 1.7×, N=4 3.64× speedup（STATUS line 126，[CHECK: 实测 multi-core full ResNet 还没跑]）
- **与文献的差异**：与 Simba@MICRO'19 chiplet × 36 形成"多 chiplet ASIC vs 多核 FPGA"对照；FLUX_CNN 的多核版在 XC7K325T 上是 BRAM-bound 而非通信 bound，对应不同 scaling 域。**新颖性强度自评：工程化贡献，N=4 跨核 SRAM 直送的具体仿真验证有完整性价值**

### C3.6：参数 single source of truth (`params.py`)

- **层级**：工程级 / 工具链
- **内容描述**：`params.py`（项目根）作为 RTL/Python 唯一参数源，64 个 `\`FLUX_*` 宏覆盖核心尺寸 + SRAM 容量 + AXI/CSR + 全局地址映射 + 58 CSR addr。`python params.py` 自动生成 `RTL/flux_cnn_params.svh`；RTL `\`include`，Python `from params import *`。
- **代码定位**：项目根 `params.py` → `RTL/flux_cnn_params.svh`（generated）；`docs/params.md`
- **验证途径**：cfg_regs.sv / core_top.sv / multicore_top.sv 已用；hw_files / scheduler 已用
- **实现状态**：✅ 已完成
- **新颖性强度自评：工程化贡献**——避免参数 drift 的实用工程实践

### C3.7：W slice (Mode C) — 任意 H×W 输入的多核切分

- **层级**：机制级 / 系统集成
- **内容描述**：N 核处理同一图的不同 W 段，每核独立从 DDR 读自己 W 段（offset = w_in_start × cin × 16），各核 ODMA 写 DDR 的对应 W 段，DDR 上自然合并。`computed redundancy halo` 机制（如 N=2 K=3 pad=1 W=32：Core 0 处理 W[0..17), pad_l=1 pad_r=0；Core 1 处理 W[15..32), pad_l=0 pad_r=1，重叠 2 列 halo）。RTL 改动轻量（idma_ctrl.sv 加 `cfg_ddr_ifm_row_stride` ~20 行，让 cur_addr 用 row_stride 推进而不是 cmd_btt）。
- **代码定位**：`RTL/DMA/idma_ctrl.sv`（cur_addr += cfg_ddr_ifm_row_stride）；`toolchain/hw_files.compute_w_slice_geom` / `derive_w_slice_cfg`；`toolchain/scheduler.py`（LPT + W/cout slice 决策 + 兼容矩阵）；`memory/mode_c_w_slice_design.md`
- **验证途径**：20/20 W slice case 全 bit-exact PASS（STATUS §2.8）；wslice_oddw (W=33 不被 4 整除) / wslice_smallw (W=8 极小) 等 corner 也通过
- **实现状态**：✅ N=2/4 W slice 已完成；🔄 cout slice 未做；🔄 stage barrier 多 stage 调度未做；⏳ 片上 push 链 (P2 完成态) 设计就绪未实施（STATUS line 256，估 2-3 天）
- **量化数据**：N=4 wslice1 3833 cycles vs N=2 5569 cycles = 1.45× speedup（[CHECK: 是否可视为接近线性，要看 N=2→N=4 的 baseline 差异如何归一化]）
- **与文献的差异**：传统 multi-core CNN 加速器（Simba 等）按 channel 切；FLUX_CNN 在 streaming 路径上做 W slice，与 Liu Full-Stack@TNNLS'21 / Kang AoCStream 同属 streaming 加速谱系但 multi-core 切分维度选择不同。**新颖性强度自评：claim 偏强**——具体的"computed redundancy halo + asymmetric pad + 整层 stride 标记 + DDR row_stride 解耦 cmd_btt"组合在 literature.md 中没找到完全对应的 prior art


## 4. 实测结果（量化指标）

> **数据口径声明**：以下数据中 "100 MHz target" 是设计目标，"68.4 MHz Fmax" 是 XC7K325T-2 OOC 综合实测。论文必须统一口径，避免审稿人挑战。

### 4.1 综合资源（XC7K325T-FFG900-2，OOC，Vivado 2023.1）

| 资源 | 单核 | 2 核 wrapper | 推算 3 核 | 推算 4 核 | 容量 |
|------|------|-------------|----------|----------|------|
| LUT | 36,942 (18.1%) | 74,386 (36.5%) | 109K (54%) | 146K (72%) | 203,800 |
| FF | 13,167 (3.2%) | 26,927 (6.6%) | 40K | 53K | 407,600 |
| BRAM36 | 128 (28.8%) | **256 (57.5%)** | **384 (86%)** | **512 ❌** | 445 |
| DSP48E1 | 82 (9.8%) | 164 (19.5%) | 246 | 328 | 840 |
| **Fmax** | **68.4 MHz** (WNS=-4.618 ns) | 68 MHz | — | — | — |

**BRAM 明细（单核）**：WB SRAM 1024×2048 = 57；IFB SRAM 8192×128 = 32；Shortcut Bank 8192×128 = 32；OFB SRAM 2048×128 = 7 + 1 RAMB18。

**结论**：3 核是 XC7K325T 不动 SRAM 的硬上限（BRAM-bound）；4 核需把 shortcut_bank 8192→2048。

**来源**：STATUS §1（单核）+ §2（多核），commit b158cab 时点综合报告。

### 4.2 仿真回归（22+24+20 case）

| 套件 | 数量 | 状态 | cycles 范围 | 来源 |
|------|------|------|-------------|------|
| 单核 ResNet-18 风格链式 | 11 cases × 3 模式（无 fold / `--fold` / `--fold --s2d`）= 22 | 22/22 PASS | 整网 593K cycles, 86.6% MAC% | STATUS line 355 |
| 鲁棒性 corner case | 24 (K∈{1,2,3,5,7}, stride∈{1..4}, pad∈{0..3}, cin∈{4..32}, cout∈{16..32}, H/W ∈ {1×1, 15×17, 28×28, 33×33, 64×32, 120×68}) | 24/24 PASS @100k cycles 内 | STATUS §2.5 line 313–321 |
| 多核 N=2/4 W slice | 20 (10 cases × N=2 + 10 cases × N=4) | 20/20 PASS bit-exact | STATUS §2.8 |
| 多核 N=2 DDR mode (M1.5) | 1 (K=3 C8C16 30×30) | PASS | 9057 cycles，单核同 case 8808 cycles → +2.8% AXI 仲裁开销 |
| 多核 N=2 跨核 SRAM 直送 (M2) | 1 (双层 chain 32×32×16) | PASS | 11006 cycles, Layer 1 OFB 1024 word bit-exact |

**总计**：约 67 个独立 case 全 PASS。

### 4.3 端到端延迟（ResNet-18 风格 11 层链）

| 指标 | 值 | 备注 |
|------|----|------|
| 总 cycles | 593K | STATUS line 355 |
| MAC% (整网，含 IDMA/ODMA stall) | 86.6% | STATUS line 355 |
| 端到端时间 @ 100 MHz **target** | 5.95 ms | → 168 fps 上限（target 口径）|
| 端到端时间 @ 68.4 MHz **Fmax** | 8.69 ms | → 115 fps 上限（实测 Fmax 口径）|
| Wall_us (host 启动 → host 可读 DDR) | [CHECK: 等加 Wall_us 实测列] | regression report 里有该列但具体数字需要重跑取 |

### 4.4 PE 利用率（model_analysis.md §2 表）

| 层 | K | s | Cin | Cout | 有效 MAC (无 fold) | PE 利用率 (无 fold) | 启用 Ky-fold/S2D 后 [CHECK: 实测百分比] |
|---|---|---|-----|------|------|--------|--------|
| Layer 1 (C0) | 7 | 2 | 4 | 8 | 32/256 | **12.5%** | [CHECK: 跑 fold 模式 case 取 cycles 算] |
| Layer 3/4 | 3 | 1 | 8 | 8 | 64/256 | **25.0%** | [CHECK] |
| Layer 5a/5c | 3/1 | 2 | 8 | 16 | 128/256 | **50.0%** | [CHECK] |
| Layer 5b 起 | 3 | 1 | 16 | 16 | 256/256 | **100%** | 100% (无需 fold) |
| FC_xy | 1 | — | 256 | 2 | 32/256 | **12.5%** | Cout 小不复用，硬件保持简洁 |

**结论**：浅层（Cin<16）才是 fold 受益核心；Cout<16 当前不优化（PE 列空转，util=Cout/16）。

### 4.5 Multi-core 估算 vs 实测

| 网络 | N=1 | N=2 | N=4 |
|------|-----|-----|-----|
| ResNet 11-layer (131M MAC) 估算 | 593K cycles, 168 fps | ~302K, 331 fps (1.7×) | ~141K, 709 fps (3.64×) |
| YOLOv3-tiny (2.7G MAC) 估算 | 10.87M, 9.2 fps | ~5.45M, 18.3 fps | ~3.12M, 32 fps |
| **实测 W slice wslice1 (1 层 K=3)** | — | 5569 cycles | 3833 cycles (1.45×) |

来源：STATUS line 126（估算）+ STATUS §2.8（实测）。

**[CHECK: ResNet 完整 11-layer 多核 chain 实测 cycles 还没跑（STATUS line 254 估 1-2 天）]**

### 4.6 已知缺口（必须诚实标注）

| 项 | 状态 | 说明 |
|---|------|------|
| Fmax 100 MHz | 未达成 | 实测 68.4 MHz；critical path 在 SDP 量化组合链；STATUS §4 给出修复路径 |
| `(* use_dsp = "yes" *)` 属性 | 未加 | 82/256 PE 综合到 LUT；加属性估能省 17K LUT；STATUS line 50 |
| Pooling 硬件 | 未做 | 软件代替；model_analysis.md §5 列入算子缺口 |
| Depthwise Conv | 未做 | 16×16 broadcast 不直接支持；同上 |
| 稀疏 / 剪枝 | 未做 | 选择 dense + streaming 路线 |
| ResNet 完整 multi-core chain | 🔄 适配中 | residual 路径 + chain h/w 维度变化（STATUS line 254）|
| 跨层 WDMA 预取 / K=1 stride=2 IDMA 跳行 | 路线图 | STATUS §4 ROI 排序 |


## 5. 与 prior art 的对比矩阵

> 仅列与 FLUX_CNN 直接对位的工作；完整文献位置在 `literature.md`。

### 5.1 性能维度

| 维度 | FLUX_CNN | TPU v1 | Eyeriss JSSC'17 | Gemmini | Snowflake | Angel-Eye | Aydonat DLA | Lu Winograd | Liu Full-Stack | VTA |
|------|----------|--------|-----------------|---------|-----------|-----------|-------------|-------------|----------------|-----|
| 平台 | XC7K325T (FPGA) | 28nm ASIC | 65nm ASIC | TSMC 16nm/Intel 22FFL ASIC | Zynq XC7Z045 | Zynq XC7Z045 | Arria 10 | ZCU102 | Arria 10 GX1150 | Zynq XC7Z020/045 |
| 阵列 | 16×16 (256) | 256×256 (65k) | 14×12 (168) | 16×16 默认 | 128 GOPS peak | 不详 | 不详 | 不详 | 不详 | 256 PE 默认 |
| 数据类型 | INT8 | INT8 | INT16 | INT8 | INT8 | INT8 | FP16/Winograd | Winograd | INT8 | INT8 |
| 峰值算力 | 51.2 GOPS @100MHz target / **35 GOPS @68.4MHz Fmax** | 92 TOPS | — | 106.1 GOPS/W (22nm) | 128 GOPS | 6× 同期 FPGA | 1382 GFLOPS | 854.6 GOPS (AlexNet) | >1.3 TOPS |  256 PE @100MHz |
| 整网 MAC% | **86.6%** (ResNet-18 风格 11 层) | — | — | — | 91% 平均 | — | — | — | **97%** | — |
| Fmax | 68.4 MHz 实测 | — | — | — | 100 MHz | — | — | — | — | 100 MHz |

**口径声明**：FLUX_CNN 行的算力数字必须明确"100 MHz target"和"68.4 MHz Fmax"两个口径；论文中应一致使用其中一个。

### 5.2 功能 / 设计取舍

| 维度 | FLUX_CNN | NVDLA | Gemmini | VTA | Eyeriss-v2 | MAERI | Alwani Fused | Kang AoCStream | Liu TNNLS'21 |
|------|----------|-------|---------|-----|-----------|-------|--------------|----------------|--------------|
| 中心 controller | **去中心 valid-ready** | CDMA→CMAC→CACC→SDP 显式 | RoCC scheduler | 4 段 fetch/load/compute/store | 不详 | 中心 + ART | 不详 | 不详 | 不详 |
| 任意 H×W | ✅ row-ring streaming | tile-based | tile-based | tile-based | 同上 | 同上 | layer fusion | layer-pipelined | streaming |
| Cin<16 PE 行不足 | ✅ **compiler-only** Ky-fold/S2D | 硬件 | 硬件 | 硬件 | 硬件 NoC reconfig | 硬件 ART 网络 | — | — | — |
| Cout<16 | ❌ 列空转 | 硬件 | 硬件 | — | 硬件 | 硬件 | — | — | — |
| Residual fusion | ✅ SDP内 shortcut_mult/shift | ✅ SDP | — | — | — | — | — | — | ✅ |
| Pooling 硬件 | ❌ 软件 | ✅ | ✅ | — | — | — | — | — | — |
| Depthwise Conv | ❌ | ✅ | — | — | ✅ | — | — | — | — |
| 稀疏 | ❌ dense | — | — | — | ✅ | — | — | — | — |
| Layer fusion (跨层片上) | ❌ layer-serial | — | — | — | — | — | ✅ 95% 流量节省 | ✅ 全片上 | ✅ |
| 多核 scaling | ✅ N=2/4 W slice | 多 instance | — | — | — | — | — | — | — |

### 5.3 prior art 威胁评估（对应 literature.md §C 三处）

| Prior art | 威胁度 | FLUX_CNN 差异点 | 论文 Related Work 必谈 |
|-----------|--------|-----------------|------------------------|
| Alwani Fused-layer @ MICRO'16 | 🔴 高 | row-level granularity（只缓几行而非几层 FM）+ layer-serial 单核共用硬件 | ✅ 必须精细论证 |
| Kang AoCStream @ arXiv'22 / Sensors'23 | 🔴 高 | 承认外存必然 + 优化 DDR 流量（Kang 强调全片上）；layer-serial vs Kang layer-pipelined 多 block | ✅ 必须精细论证 |
| Liu Full-Stack @ TNNLS'21 | 🟡 中 | 单核 layer-serial 不做 layer fusion；规模差（中型 7-series vs Arria 10 GX1150）；97% MAC% vs 86.6% MAC% 是同思路不同规模 | ✅ 列对照 |

### 5.4 可量化优势 candidate（按 claim 强度排列）

| # | Claim | 强度 | 主要 prior art 威胁 |
|---|-------|------|---------------------|
| 1 | **流式任意 H×W 单 start 处理 VGA**（10 KB ring vs 4.9 MB 整图）| 中 | Alwani / Kang / Liu 是近邻——差异在 row-level + layer-serial |
| 2 | **编译器侧 PE 利用率优化（Ky-fold + S2D）零 RTL 改动** | 强 | literature.md 检索未命中"FPGA 上把 Ky 折到 cin"的 prior art；S2D 在加速器领域引用谱系不清 |
| 3 | **去中心化握手流水（5 模块无中心 FSM）** | 弱-中 | Buffets@ASPLOS'19 提供形式化语言；属"工程美学"claim |
| 4 | **SDP residual fusion 可编程 shortcut_mult/shift** | 弱 | NVDLA-inspired，Liu TNNLS'21 也支持；增量改进 |
| 5 | **Multi-core W slice + computed redundancy halo + DDR row_stride 解耦 cmd_btt** | 中-强 | 在 streaming 谱系内的多核切分维度选择，literature.md 中没找到完全对应 prior art |



## 6. 实现状态汇总

| 模块 / 功能 | 状态 | 验证 | 风险 / 备注 |
|---|---|---|---|
| Core pipeline (line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer) | ✅ | 22-case 链式回归 + 24-case smoke | — |
| DMA 子系统（idma/wdma/odma/rdma_ctrl + axi_dm IP + mm2s_arb + axi_m_mux）| ✅ | sim/tb_axi_dm_smoke + sim/tb_idma_ctrl + sim/tb_core_dma | DataMover 性能 +0.5% vs 自写 DMA |
| AXI-Lite CSR + cfg_regs（双端 csr_w/seq_w）| ✅ | 22-case | — |
| R.1 bias→SDP + Shortcut Bank + bias_rf | ✅ | STATUS line 24 | — |
| R.2 SDP residual fusion（shortcut_mult/shift）| ✅ | 链式 CASES 三层 ds | — |
| Done sticky 寄存器 | ✅ | host poll / GIC IRQ 都干净 | — |
| CFG_WRITE descriptor (TYPE_CFG=0x3) | ✅ | desc_fifo 32→128，host AXI-Lite 4 写/层 | — |
| 链式 CASES DSL builder | ✅ | 7 行 Python 写完 11 层 | — |
| Wall_us 端到端报告 | ✅ | regression report 加列 | [CHECK: 实测 Wall_us 数字待重跑取] |
| Ky-fold 编译器 | ✅ | `--fold` 22-case PASS | — |
| Space-to-Depth 编译器 | ✅ | `--s2d` 22-case PASS | — |
| Cin/Cout 多 slice 切片 | ✅ | model_analysis 14 层全覆盖 | — |
| PyTorch nn.Sequential 多层编译 | ✅ | run_model.py mnist_allconv | — |
| **Multi-core M1 (N=2 wrapper)** | ✅ | 综合 + sim PASS @ 9057 cycles | — |
| **Multi-core M1.5 (axi_m_mux 全字段 forward + axi_arbiter sticky sel)** | ✅ | DDR mode TB PASS | — |
| **Multi-core M2 跨核 SRAM 直送 (axi_2to3 + ifb_axi_slave)** | ✅ | 双层 chain PASS @ 11006 cycles bit-exact | — |
| **Multi-core M2.5 调度器 + driver 框架（params.py / scheduler.py / run_multicore_chain.py）** | ✅ | simple2 demo PASS @ 20253 cycles | — |
| **Multi-core P1 W slice (Mode C) N=2/4** | ✅ | 20/20 W slice case bit-exact | — |
| Multi-core W slice cfg corner cases | ⚠ WIP | wslice_smallw / wslice_oddw 已过 | `memory/mode_c_w_slice_design.md` 列细节 |
| Multi-core cout slice (Mode C) | ❌ 未做 | — | STATUS line 119, 估 1 天 |
| Multi-core stage barrier 多 stage 调度 | ❌ 未做 | — | STATUS line 120 |
| Multi-core 片上 push 链 (P2) | ❌ 未做 | — | STATUS line 120, 设计就绪估 2-3 天 |
| ResNet 11 layer multicore chain 适配 | 🔄 进行中 | — | STATUS line 254, residual 路径 + chain h/w 维度变化 |
| `(* use_dsp = "yes" *)` 属性 | ❌ 未加 | — | STATUS line 50, 1 行修改估省 17K LUT |
| SDP 流水线化 | ❌ 未做 | — | Fmax 拉到 100+ MHz, ~30 行 |
| AvgPool 硬件 | ❌ 暂停 | — | 等用户确认有无 host CPU |
| Pooling / Depthwise / 稀疏 | ❌ | — | 路线图未排 |



## 7. 论文取舍建议

### 7.1 主线 narrative 候选

| narrative | 主贡献组合 | 适合的会议 | 风险 |
|-----------|-----------|------------|------|
| A. **"compiler-side PE utilization for fixed-array CNN accelerators"** | C2.1 Ky-fold + C2.2 S2D + C2.4 切片 + C2.5 编译器栈作为支撑 | FCCM / TCAD / TVLSI / TRETS（FPGA 系工程偏重）；ASPLOS 也可（compiler-hardware co-design）| Cout<16 不优化 + S2D 引用谱系不清需 reviewer 阶段补查 |
| B. **"row-streaming + multi-core W slice for arbitrary H×W INT8 CNN"** | C1.2 row-ring + C3.5 multicore + C3.7 W slice | FPGA / FCCM / FPL | Alwani / Kang / Liu prior art 威胁高，必须精细差异化 |
| C. **"去中心化握手流水的工程实践"** | C1.1 + C1.3 PARF per-col + C1.4 SDP fusion + C3 系统集成 | TVLSI / TRETS / 期刊 | 工程美学 claim 弱 |
| D. **"end-to-end open-source 16×16 INT8 streaming CNN accelerator with compiler"** | 全部 C1-C3 | FPGA / FCCM 期刊扩展 | 偏 system 完整性论文 |

**[TBD: 主 Agent 决定 narrative 走 A/B/C/D 哪条；建议 A（vs B 的 prior art 威胁更小，vs C 的 claim 更具体）+ 部分 B 作为 system context]**

### 7.2 取舍点

- **去中心化 vs 中心 controller**：narrative A 不要硬塞这条；narrative C 才作为主 claim 出现
- **Multi-core W slice**：narrative B 主推，narrative A 作为"system 完整性"二级出现
- **PE utilization 实测百分比**：narrative A 的核心数据，**必须由 project-analyst 跑回归取实测百分比填 4.4 节**
- **整网 MAC% 86.6% vs 单层 PE 利用率**：两口径需文中明确区分
- **100 MHz target vs 68.4 MHz Fmax**：建议全文统一用 Fmax 实测口径并标注修复路径在 Future Work
- **是否开源 [TBD]**：narrative D 高度依赖开源，narrative A/B/C 不强依赖

### 7.3 不建议作为主 claim 的项

- **SDP residual fusion** (C1.4)：只作为 system 完整性 argument 出现，不主推；NVDLA-inspired + Liu 已支持
- **PARF per-col 拆分** (C1.3)：作为实现细节，不作为顶层贡献
- **CFG_WRITE descriptor** (C3.2)：工程化，作为"system 实用性"二级
- **params.py** (C3.6)：辅助工程实践，不写入 contribution



## 8. 待决问题清单（[CHECK]/[TBD] 汇总）

> 论文写作前必须由用户 / project-analyst 处理这些项；按"测一下/查一下/引一下" vs "用户决定"分类。

### 8.1 [CHECK]：可通过实测/查文献解决的数据缺口

| # | 项 | 来自哪条 contribution | 行动 |
|---|----|----------------------|------|
| 1 | Layer 1 (Cin=4, Cout=8) 启用 Ky-fold 后实测 PE 利用率百分比 | C2.1 / 4.4 表 | 跑 `run_regression.py --fold` 取 cycles 反算 PE 利用率 |
| 2 | Layer 3/4 (Cin=8) 启用 Ky-fold 后 PE 利用率 | C2.1 / 4.4 表 | 同上 |
| 3 | 启用 S2D 后 DDR 带宽节省比例 | C2.2 量化数据 | 跑对照 run，统计 IDMA 读次数 |
| 4 | "100 MHz target → 168 fps" vs "68.4 MHz Fmax → ~115 fps" 文中口径统一 | 4.3 表 | 用户决定用哪个口径 |
| 5 | Wall_us 端到端实测数字 | C3.3 / 4.3 表 | 重跑回归取 Wall_us 列 |
| 6 | 单层 PE 利用率 vs 整网 MAC% (86.6%) 的关系 | 4.4 / 5.1 表 | 在论文中文字论证 |
| 7 | parf BRAM 数量贡献单独估算（per-col 16 SRAM 各占多少 BRAM）| C1.3 | 看综合报告 cell utilization |
| 8 | ResNet 11-layer multicore N=2/4 实测 cycles vs 估算 | 4.5 表 | STATUS line 254 ResNet 适配后跑 |
| 9 | N=4 wslice1 1.45× speedup 是否近线性的 baseline 归一化 | C3.7 | 与 N=1/N=2 对比时分子分母统一 |
| 10 | 各 baseline 整网 MAC% (Snowflake 91% / Liu 97% 已知；Angel-Eye / Aydonat / Lu / VTA 未知) | 5.1 表 | reviewer 阶段查原文 |
| 11 | 同器件 (XC7K325T 量级) 其他工作 Fmax 对照 | 5.1 表 | reviewer 阶段查原文 |
| 12 | Angel-Eye 是否原生支持 SDP residual fusion | 5.2 表 | 查 Guo et al.@TCAD'18 |
| 13 | 同器件 Angel-Eye / DPU / VTA 资源占用对照 | 5.2 表 | reviewer 阶段查 |
| 14 | 各工作 verification (TB / 回归) 公开度 | 5.2 表 | 一般查 GitHub |
| 15 | S2D (space-to-depth) 在加速器领域的引用谱系（reviewer 风险）| C2.2 | reviewer 阶段在 ASPLOS / HPCA / MICRO 中补查"architecture-side S2D" |
| 16 | NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN 引用方式（vendor doc 5 处）| literature.md | 主 Agent 决定 |

### 8.2 [TBD]：需要用户决定的事项

| # | 项 | 涉及 narrative |
|---|----|----------------|
| 1 | **目标会议** | FPGA / FCCM / TCAD / TVLSI / TRETS / ASPLOS / MICRO 候选 |
| 2 | **narrative 主线** | 7.1 表 A / B / C / D |
| 3 | **是否开源** | narrative D 高度依赖 |
| 4 | **写作时是否同步把 mac_pe `(* use_dsp = "yes" *)` 加上重综合更新数字** | 影响 4.1 资源数据 |
| 5 | **是否同步把 SDP 流水线化重综合更新 Fmax** | 影响 4.1 / 4.3 / 5.1 数据 |
| 6 | **是否单列 axi_dm IP 集成方案** | 工程偏向会议（FCCM）适合，架构偏向（ISCA / MICRO）不适合 |
| 7 | **PE-fold 是否单独成章** | narrative A 主推，narrative B/C 二级 |

### 8.3 修订路径（按风险 排序）

1. **最先做**：跑 Ky-fold case 实测 PE 利用率（CHECK #1, #2），给 narrative A 配上核心数字
2. **次之**：对 NVDLA / Xilinx / cuDNN 引用方式的决策（CHECK #16），不解决会被审稿人挑剔
3. **再次之**：Liu / Kang / Alwani 的 Related Work 差异化论证（5.3 表），narrative B 高威胁
4. **最后**：N=4 ResNet 完整 multicore 实测（CHECK #8），影响 narrative B 的 multi-core 论据强度

---

## 附录：重要文件路径速查

| 类别 | 路径 |
|------|------|
| 项目根 | `c:/_Project/FLUX_CNN/` |
| 文献综述 | `paper/workspace/literature.md` |
| 本文档 | `paper/workspace/contributions.md` |
| 当前状态 | `STATUS.md` |
| 顶层叙述 | `README.md` |
| RTL 总入口 | `RTL/core_top.sv` / `RTL/multicore_top.sv` |
| 核流水模块 | `RTL/{line_buffer,wgt_buffer,mac_array,parf_accum,parf_col,ofb_writer,sdp,bias_rf,sequencer}.sv` |
| DMA 子系统 | `RTL/DMA/{idma_ctrl,wdma_ctrl,odma_ctrl,rdma_ctrl,mm2s_arb,dfe}.sv` |
| AXI 子系统 | `RTL/AXI4/{axi_m_mux,axi_arbiter,axi_lite_csr,ifb_axi_slave}.sv` |
| 工具链 | `toolchain/{gen_isa_test,hw_files,compile_layer,compile_model,run_regression,scheduler,run_multicore_chain,gen_cross_core_test}.py` |
| Single source params | 项目根 `params.py` → `RTL/flux_cnn_params.svh` |
| 设计文档 | `docs/{pe-fold,slicing/README,modules/*,multicore_scheduling,roadmap,simulation}.md` |
| 设计经验 | `memory/{cross_core_design,m2_cross_core_pipeline,mode_c_w_slice_design,sdp_residual_fusion,bias_to_sdp_refactor,cfg_write_descriptor,done_sticky_pattern,axi_m_mux_quirk,dsp_inference,resource_budget,bram_layout,chained_cases_design,ip_gen_workflow}.md` |
| 仿真 | `sim/{tb_core_dma,tb_axi_lite_csr,tb_axi_m_mux,tb_axi_dm_smoke,tb_idma_ctrl,tb_multicore}/` |
| 综合 | `Syn/{run_syn,gen_axi_datamover,gen_multicore_ip}.tcl` |
| 模型分析 | `model_analysis.md` |

