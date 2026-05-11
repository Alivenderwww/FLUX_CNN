### 判定：PASS

# 技术评审报告 Phase 5 §6 System Integration

## 第 1 次评审

### 评审范围
- paper.md §6 System Integration and Toolchain（line 302-360）
- 1 chapter intro + 5 节（§6.1-§6.5）共 23 段正文
- 对照 paragraph-skeleton.md line 230-268（§6 骨架）
- 不评 §1-§5（已 PASS）/ §7-§8（占位）

### 段落骨架对齐
骨架要求：§6.1=5 段 / §6.2=5 段 / §6.3=5 段 / §6.4=5 段 / §6.5=3 段，合计 23 段 + chapter intro。

实际：
- §6.1 line 308 / 310 / 312 / 314 / 316 = 5 段（claim / evidence×2 / comparison / transition）✓
- §6.2 line 320 / 322 / 324 / 326 / 328 = 5 段（claim / evidence / claim / evidence / comparison）✓
- §6.3 line 332 / 334 / 336 / 338 / 340 = 5 段 ✓
- §6.4 line 344 / 346 / 348 / 350 / 352 = 5 段 ✓
- §6.5 line 356 / 358 / 360 = 3 段 ✓
- chapter intro line 304 = 1 段 ✓

合计 23 段 + 1 intro。完全对齐。

### 架构准确性（4 处抽样均通过）

1. §6.1 PyTorch toolchain（line 308-310）
   - 提到 `toolchain/models/run_model.py` / `compile_layer.py` / `compile_model.py` / `run_regression.py` / `hw_files.derive_layer_cfg()` / `_plan_ddr` / `Chain` / `_Node` / `resnet_block`
   - 与 CLAUDE.md "PyTorch 模型端到端部署" 段、`gen_isa_test.py` 是 derived 值 source of truth、`hw_files.derive_layer_cfg()` 共享 cfg 派生 一致 ✓

2. §6.2 DMA 子系统（line 320-326）
   - 列举 `axi_dm` IP + `idma_ctrl/wdma_ctrl/odma_ctrl/rdma_ctrl` + `mm2s_arb`（IDMA/WDMA 共享 MM2S）+ `axi_m_mux`（聚合 MM2S/S2MM/DFE）+ `axi_lite_csr`
   - 与 CLAUDE.md "DMA 子系统"段（`idma_ctrl / wdma_ctrl / odma_ctrl + mm2s_arb + axi_dm IP + axi_m_mux + axi_lite_csr`）+ 1 AXI4 Master + 1 AXI-Lite Slave 一致 ✓
   - `rdma_ctrl` 第 4 控制器：CLAUDE.md DMA 段未列入三巨头列表但 commit 44d49be / 141d031 显式 "加 rdma_ctrl"，骨架 §6.2 段 2 也写了四件套，前后一致 ✓
   - `CFG_WRITE` (TYPE_CFG=0x3) + DFE + 50→4 写次数 + done sticky + 双端 cfg 写口 与 commit b158cab "链式 CASES + CFG_WRITE descriptor + done sticky" 一致 ✓

3. §6.4 Multicore（line 344-348）
   - `multicore_top.sv` N=2/4 W-slice + computed redundancy halo + asymmetric pad（具体例 N=2 K=3 pad=1 W=32：Core0 W[0..17) pad_l=1 pad_r=0 / Core1 W[15..32) pad_l=0 pad_r=1，2 列 halo）+ `axi_2to3`/`axi_4to5` + 跨核 SRAM 直送（producer ODMA → crossbar → consumer `ifb_axi_slave`）
   - git status 显示 `?? RTL/multicore_top.sv` / `?? RTL/AXI4/ifb_axi_slave.sv` / `?? sim/tb_multicore/`（未追踪但存在），与正文模块名 / 仿真路径一致 ✓

4. §6.5 params.py（line 356-358）
   - `params.py` 64 个 `FLUX_*` 宏 + 58 CSR addr → 生成 `RTL/flux_cnn_params.svh` + Python `from params import *`
   - 与 CLAUDE.md 最新条目"params.py 作为单源" 一致；具体数字（64 / 58）骨架同样写了，未自创 ✓

### C3.3 tag 缺失评估（关键项）

骨架 §6.1 段 5 (line 238) 显式标注 `[依赖: C3.3 链式 CASES]`。

实际正文：
- §6.1 段 5 transition (line 316) "The compiler's output is ultimately a stream of descriptors that flow into hardware over DMA, which the next subsection describes." — **无 C3.3 标记**
- §6.1 段 1 (line 308) "...emits a chained descriptor list that the hardware replays without host intervention" — 实质上覆盖 C3.3 链式 CASES 的核心语义，但未带 (C3.3) tag
- §6.2 段 3 (line 324) "Descriptor-driven configuration further reduces host involvement... CFG_WRITE descriptor type... lets the chained descriptor list write cfg_regs directly" — 同样实质覆盖但未带 C3.3 tag
- 全 §6 grep 不到 "C3.3" 字面量（人工核对 line 302-360）

**评估**：C3.3 的实质内容（链式 descriptor / 主机零干预 / chained descriptor list）在 §6.1 段 1、段 5 transition、§6.2 段 3 三处均已展开，并非内容遗漏；缺失的仅是 `(C3.3)` 形式标记。这与 §6.1 段 1 的 `(C2.5)`、§6.2 段 1 的 `(C3.1)`、§6.2 段 3 的 `(C3.2)`/`(C3.4)`、§6.3 段 1 的 `(C1.4)`、§6.4 段 1 的 `(C3.5, C3.7)`、§6.5 段 1 的 `(C3.6)` 形成不一致的引用风格。

按评审约定（"C3.3 实质遗漏直接 FAIL；C3.3 仅 tag 形式问题判轻微"），此处属**轻微**。建议在 §6.1 段 1 的 "chained descriptor list" 处或段 5 transition 末尾补 `(C3.3)` 标签使引用风格统一，但不阻塞 PASS。

### 其他 contribution 引用核对

骨架要求 vs 正文实际：

| 编号 | 骨架位置 | 正文位置 | 状态 |
|------|---------|---------|------|
| C2.5 | §6.1 段 1 | §6.1 段 1 line 308 `(C2.5)` | ✓ |
| C3.3 | §6.1 段 5 | 无显式 tag（实质覆盖 §6.1 段 1+5、§6.2 段 3） | 轻微 |
| C3.1 | §6.2 段 1 | §6.2 段 1 line 320 `(C3.1)` | ✓ |
| C3.2 | §6.2 段 3 | §6.2 段 3 line 324 `(C3.2)` | ✓ |
| C3.4 | §6.2 段 3 | §6.2 段 3 line 324 `(C3.4)` | ✓ |
| C1.4 | §6.3 段 1 | §6.3 段 1 line 332 `(C1.4)` | ✓ |
| C3.5 | §6.4 段 1 | §6.4 段 1 line 344 `(C3.5, C3.7)` | ✓ |
| C3.7 | §6.4 段 1 | §6.4 段 1 line 344 `(C3.5, C3.7)` | ✓ |
| C3.6 | §6.5 段 1 | §6.5 段 1 line 356 `(C3.6)` | ✓ |

任务清单中 "C3.4" 为骨架原文 "C3.4 done sticky 双端 cfg 写口"，正文 line 324 写 "A done-sticky bit plus a dual-port cfg write path (C3.4)"，语义/编号双重对齐 ✓。

### Claim 强度（无 over-claim）

- chapter intro line 304："These items are necessary engineering work rather than primary novelty claims; they are reported here to establish that the architectural contributions of §3-§5 are validated end to end on real workloads rather than in isolation." — 主动降级 ✓
- §6.1 段 4 line 314："FLUX_CNN's compiler is deliberately a hand-rolled mini-compiler scoped to its own ISA: it does not aim to be a target backend for arbitrary frontends" — 主动让出 generality ✓
- §6.3 段 3 line 336："we acknowledge as 'NVDLA-inspired' rather than rederived" + "residual fusion exists because of ResNet [He-CVPR'16], not as a hardware contribution of ours" — 主动让出 SDP/residual 算法侧 ✓
- §6.3 段 5 line 340："The novel content here is limited to the R.1 bias relocation and the R.2 programmable quantization factors; we present these as system-completeness evidence rather than as primary architectural novelty" — 显式收窄 novelty ✓
- §6.4 段 4 line 350："We deliberately do not claim near-linear scaling without that audit, and we do not yet report multi-core numbers on the full ResNet chain" — 主动避免 1.45× → 线性 over-claim ✓
- §6.5 段 3 line 360："This is a pure engineering practice rather than an architectural contribution" — 主动降级 ✓

System completeness 章节定位拿捏得很稳，无 over-claim。

### 数据真实性

| 数字 | 位置 | 标记 | 评估 |
|------|------|------|------|
| 593K cycles / 86.6% MAC% / 5.95 ms @ 100 MHz / 8.69 ms @ 68.4 MHz | §6.1 段 3 line 312 | `[CHECK: chain 实测口径统一]` | ✓ |
| ~3000 行手写 DMA RTL | §6.2 段 1 line 320 | 无显式 [CHECK] | 工程量级估算，骨架同样未标，可接受 |
| 50 → 4 register writes | §6.2 段 3 line 324 | 无 | 与 commit b158cab 一致，可接受 |
| +0.5% gap | §6.2 段 4 line 326 | `[CHECK: 0.5% gap measured...]` | ✓ |
| burst_size 16→256 | §6.2 段 4 line 326 | 无 | 与 commit f238de1 "burst_size 16→256" 一致 ✓ |
| 8192×128 SRAM ≈ 32 BRAMs | §6.3 段 2 line 334 | 无 | 与骨架 `8192×128 SRAM = 32 BRAM` 一致 ✓ |
| MAX_COUT_SLICES=32 | §6.3 段 2 line 334 | 无 | R.1 commit 注解一致 ✓ |
| SDP combinational chain timing | §6.3 段 1 line 332 | `[CHECK: SDP combinational chain timing closure post-resynthesis]` | ✓ |
| N=2/4 综合通过 + 20/20 W-slice PASS | §6.4 段 1 line 344 | `[CHECK: 20/20 W-slice cases PASS confirmation in STATUS.md / sim/tb_multicore/]` | ✓ |
| 1.45× speedup (3833 vs 5569 cycles) | §6.4 段 4 line 350 | `[CHECK: 1.45× number — verify baseline normalization...]` | ✓ |
| 64 FLUX_* 宏 / 58 CSR addr | §6.5 段 1 line 356 | 无 | 骨架同样写了具体数字，与 CLAUDE.md params.py 条目一致 ✓ |
| flux_cnn_params.svh 提交 + Vivado 拾取 | §6.5 段 2 line 358 | `[CHECK: confirm flux_cnn_params.svh is currently committed...]` | ✓ |

合计 6 处 [CHECK]（与 Writer 自报 6 处一致），所有相对/不确定数字均已标记。无编造。

### [CHECK]/[TBD] 合规性

[CHECK] ×6（line 312, 326, 332, 344, 350, 358）— 全部位于具体数字或工艺/工具链状态宣称处，位置合理 ✓
[TBD] ×2：
- line 352 §6.4 段 5 末："[TBD: whether §6.4 should be promoted to its own top-level chapter once multicore evaluation matures]" — 与骨架 §6.4 段 5 末 `[TBD: §6.4 是否升格独立章]` 完全对应 ✓
- line 360 §6.5 段 3 末："[TBD: whether §6.5 deserves its own subsection or should be folded into §6.2]" — 与骨架 §6.5 段 3 `[TBD: §6.5 是否值得占节位]` 对应 ✓

无误标，无漏标（已对全部具体数字与工艺状态逐项核对）。

### 回归性（无新引用 / 无新数据）

引用文献清单：TVM-OSDI'18 / VTA-Micro'19 / fpgaConvNet-TNNLS'19 / NVDLA / He-CVPR'16 / Liu-TNNLS'21 / Jacob-CVPR'18 / Simba-MICRO'19 / Kang-Sensors'23 — 全部在骨架 line 237 / 246 / 252 / 253 / 262 出现过 ✓

未出现骨架以外的新文献、新数字或新模块名。

### 通过-失败汇总

- 段落骨架对齐：通过（23+1，结构精确匹配 5+5+5+5+3）
- 架构准确性：通过（4 处抽样全部对齐 CLAUDE.md / git commit / RTL 实物）
- C3.3 tag：轻微（实质覆盖完整，仅形式标记缺失）
- 其他 contribution 引用：通过（8/9 显式 tag，1/9 仅形式缺失即 C3.3）
- claim 强度：通过（章节、节、段三层均主动收窄 novelty）
- 数据真实性：通过（6 处 [CHECK] 全部覆盖相对/不确定数字）
- [CHECK]/[TBD] 合规性：通过（6+2 位置全部合理）
- 回归性：通过（无骨架外新增）

无严重 / 中等问题。仅 1 项轻微（C3.3 tag）。

### 修订建议（非阻塞，PASS 通过）

1. （轻微）建议在 §6.1 段 1 line 308 末尾或段 5 transition line 316 处补 `(C3.3)` 形式标签，以与同章其他 contribution 引用风格保持一致。最自然的落点是 line 308 "...emits a chained descriptor list that the hardware replays without host intervention (C3.3)."。
