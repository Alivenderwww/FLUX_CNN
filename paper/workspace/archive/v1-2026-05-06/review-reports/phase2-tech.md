# 技术评审报告 Phase 2

## 第 1 次评审

### 判定
PASS

### 评审范围

- 待评审文件：`paper/workspace/outline.md`（8 章 / 混合 narrative A+B / 17 条贡献全归位）
- 上下文参考：`paper/workspace/contributions.md` (17 条 C)、`paper/workspace/literature.md` (35 篇 A-G 七类)
- 项目根抽样校验：`RTL/`、`RTL/AXI4/`、`RTL/DMA/`、`toolchain/hw_files.py`、`docs/pe-fold.md`、`docs/slicing/README.md`、`Syn/`、`STATUS.md`、`README.md`

### 贡献覆盖完整性

PASS。outline.md §叙事一致性检查 - 贡献-章节映射表（line 254-271）逐条核对：

| 段 | 贡献编号 | 在 contributions.md 中存在 | 主章节归位 |
|---|---|---|---|
| C1 类 | C1.1 / C1.2 / C1.3 / C1.4 / C1.5 | ✓ | §4.2 / §4.3 / §4.4 / §6.3 / §7.7 |
| C2 类 | C2.1 / C2.2 / C2.3 / C2.4 / C2.5 | ✓ | §5.1 / §5.2 / §5.3 / §4.5 / §6.1 |
| C3 类 | C3.1 / C3.2 / C3.3 / C3.4 / C3.5 / C3.6 / C3.7 | ✓ | §6.2 / §6.2 / §7.1 / §6.2 / §6.4 / §6.5 / §6.4 |

- 总数 = 5+5+7 = **17 条全部归位**，与 contributions.md 实际 grep 数一致
- **无遗漏**：未发现 contributions.md 中的贡献未出现在 outline 任一章节
- **无重复主章归类**：每条贡献只有一个"主章节"，二级章节为"被引用"而非主推
- outline 已主动揭示主 Agent 提示词"18 条"为 off-by-one 错（line 273 + line 392），且按实际 17 条对齐

**轻微瑕疵**（非阻断，仅提示）：line 306 写"✅ **18 条 contribution 全部归位**"与 line 273 的"17 条"自我打架（line 273 是修正后的真实数字）。建议 Writer 把 line 306 改为 "17 条" 以保持文档内部一致；这处不算编造、不影响 PASS 判定。

### 3 章抽样核（§4 / §5 / §6）

#### §4 Architecture（line 120-143）

抽样核对模块名 vs `RTL/` 实存：
- `line_buffer / mac_array / parf_accum / parf_col / ofb_writer / wgt_buffer / sequencer / sdp / bias_rf` — `ls c:/_Project/FLUX_CNN/RTL/` 全部命中 ✓
- §4.1 "5-module core pipeline + DMA subsystem" — 与 CLAUDE.md "Core pipeline (5 模块)" 一致 ✓
- §4.2 去中心化 valid-ready：每模块独立 counter，无中心 FSM — 与 contributions.md C1.1 + CLAUDE.md "valid-ready 握手，无中心 FSM" 完全一致 ✓
- §4.3 row-ring + bidirectional credit backpressure：`rows_available ≥ yout·stride+Ky` — 与 contributions.md C1.2 完全一致 ✓
- §4.4 per-col PARF：`parf_col × 16 列独立 SRAM` — `RTL/parf_col.sv` + `RTL/parf_accum.sv` 两个文件实存 ✓
- §4.5 7 层硬件循环嵌套，对位 Interstellar / Ma FPGA'17 — 与 contributions.md C2.4 + `docs/slicing/README.md` 7 层嵌套一致 ✓

判定：**§4 全部描述能在代码 / 文档中找到事实支撑。** 没有把"未实现"写成"已实现"。

#### §5 Compiler Optimizations（line 145-168）

抽样核对路径名 vs `toolchain/hw_files.py` 实存：
- §5.1 Ky-fold：`compute_fold_params / fold_input / fold_weights` — `Grep` 在 hw_files.py line 177/196/223 全部命中 ✓
- §5.2 S2D：`compute_s2d_params / s2d_input / s2d_weights` — line 91/109/147 全部命中 ✓
- §5.3 触发条件：Ky-fold = K>1 AND Cin<16；S2D = stride≥2 AND K≥stride — 与 `docs/pe-fold.md` §5 受益层判定一致 ✓
- 章节引用 `docs/pe-fold.md §1 (Ky-fold) / §2 (S2D)` — `Grep` 头确认 §1/§2 章节确实存在 ✓
- §5.1 输入做"y-方向偏移复制"+ 权重 reshape，硬件按普通 conv 跑 — 与 docs/pe-fold.md §1 Ky-fold 数学描述一致 ✓
- §5.4 与 cuDNN im2col / MAERI ART / Eyeriss-v2 NoC / Sub-pixel CVPR'16 对位 — literature.md 这 4 条均存在 ✓

判定：**§5 全部描述能在 toolchain/ 与 docs/pe-fold.md 找到事实支撑。**

#### §6 System Integration（line 170-195）

抽样核对：
- §6.1 PyTorch 编译栈：`compile_layer / compile_model / DSL builder Chain/_Node/resnet_block` — `toolchain/{compile_layer,compile_model,run_regression}.py` 实存（toolchain ls 已确认）+ contributions.md C2.5/C3.3 ✓
- §6.2 axi_dm IP + 轻量 *_ctrl + mm2s_arb：`RTL/DMA/{idma_ctrl,wdma_ctrl,odma_ctrl,rdma_ctrl,mm2s_arb,dfe}.sv` — Bash ls 全部命中 ✓
- §6.3 SDP fusion + Shortcut Bank 8192×128：`RTL/sdp.sv` / `RTL/bias_rf.sv` / `RTL/DMA/rdma_ctrl.sv` 实存 ✓；与 contributions.md C1.4 R.1+R.2 + memory/sdp_residual_fusion.md 一致 ✓
- §6.4 multicore_top + axi_2to3/4to5 + ifb_axi_slave：
  - `RTL/multicore_top.sv` 实存 ✓
  - `RTL/AXI4/ifb_axi_slave.sv` 实存（Grep 命中）✓
  - `axi_2to3 / axi_4to5 / axi_lite_1to4` 出现在 `Syn/ip_multicore_summary.txt` + `RTL/multicore_top.sv` + `sim/tb_multicore/` ✓（Grep 命中 6 个文件）
  - `cfg_ddr_ifm_row_stride` 出现在 `RTL/DMA/idma_ctrl.sv` + `RTL/core_top.sv` + `RTL/AXI4/ifb_axi_slave.sv` ✓（Grep 命中 3 个文件）
- §6.5 params.py → flux_cnn_params.svh — 项目根 `params.py` + `RTL/flux_cnn_params.svh` 实存（CLAUDE.md 已说明，本评审多处文件列表确认）✓

判定：**§6 全部描述能在 RTL/ 与 toolchain/ 找到事实支撑。** 没有提及未实现的模块。

### [CHECK]/[TBD] 标记合规性

PASS。outline 全文 grep `[CHECK:]` ≈14 处 + `[TBD:]` ≈15 处，分类表 line 350-380 列了 13 个独立 CHECK 行动项 + 10 个独立 TBD 决策项。逐类抽核：

**[CHECK] 标记位置正确性**（重点核"该标未标"）：
- §1.4 PE 利用率提升数字（12.5%→~99%）— ✓ 标了 [CHECK]，与 contributions.md §8.1 #1/#2 + literature.md line 503 三处 [CHECK] 共指同一数据 gap
- §5.1/§5.2/§5.4 实测 PE 利用率 / DDR 节省比 — ✓ 标了，与 contributions.md §4.4 表的 [CHECK] 一致
- §7.4 Wall_us 实测 + 单层 vs 整网 MAC% 论证 — ✓ 标了，对应 contributions.md §8.1 #5/#6
- §4.4 parf BRAM 数量贡献单独估算 — ✓ 标了，对应 contributions.md §8.1 #7
- §6.4 N=4 wslice1 1.45× speedup baseline 归一化 + ResNet 11-layer multi-core 实测 — ✓ 标了，对应 STATUS line 254 + contributions.md §8.1 #8/#9
- §7.6 各 baseline 整网 MAC% / 同器件 Fmax / Angel-Eye SDP / 同器件资源 — ✓ 标了，对应 contributions.md §8.1 #10-#13

**[TBD] 标记位置正确性**：
- 元信息：目标会议（FPGA/FCCM/TCAD/...）+ 论文标题候选 — ✓ 决策项，标 [TBD] 合理
- §1.4 fps 口径选择（target 168 vs Fmax 115）— ✓ 用户决策项，标 [TBD] 合理
- §3.5 单独成节 vs 并入 §6.3 / §4.4 与 §4.2 是否合并节 / §6.4 是否升格 / §6.5 params.py 是否值得占节 — ✓ 篇幅决策项，标 [TBD] 合理
- §7 mac_pe `(* use_dsp *)` + SDP 流水线化是否同步综合更新 — ✓ 工程决策项，标 [TBD] 合理

**未发现"该标未标"的具体数字**：所有具体数字（86.6% MAC%, 593K cycles, 5.95 ms, 168 fps, 36.9k LUT, 128 BRAM, 82 DSP, 68.4 MHz Fmax, 20/20 W slice case, 1.45× speedup）均能在 contributions.md §4 直接追溯到 STATUS.md / 综合报告 / regression report，**不是新冒出的、需要 [CHECK] 的数字**。

**未发现"误标 [CHECK]"**：无已知确定的事实被误标为 [CHECK]。

### 文献映射抽样核

抽 5 处 outline 引用 vs literature.md 验证：

1. **MAERI @ ASPLOS'18**（outline §3.2 + §5.4）— literature.md line 184-191 ✓ DOI 10.1145/3173162.3173176，作者 Kwon et al.
2. **Eyeriss-v2 @ JETCAS'19**（outline §3.2 + §5.4）— literature.md line 137-145 ✓ DOI 10.1109/JETCAS.2019.2910232
3. **Buffets @ ASPLOS'19**（outline §4.2 + §3.2）— literature.md line 204-211 ✓ DOI 10.1145/3297858.3304025（已修正过会议归属）
4. **Snowflake @ ISCAS'17**（outline §3.4 + §7.6）— literature.md line 280-287 ✓（Phase 0 已修正 CVPRW'14→ISCAS'17）
5. **Liu Full-Stack @ TNNLS'21**（outline §3.4 + §7.6 + §6.4）— literature.md line 319-327 ✓ DOI 10.1109/TNNLS.2021.3055240，第一作者 Shuanglong Liu（Phase 0 已修正作者格式）
6. **Tangram @ ASPLOS'19**（outline §8 future work 对照）— literature.md line 224-231 ✓（Phase 0 已修正 MICRO'19→ASPLOS'19）
7. **Sub-pixel @ CVPR'16**（outline §5.4）— literature.md line 345-353 ✓
8. **Kang AoCStream @ arXiv'22 / Sensors'23**（outline §3.4 + §6.4）— literature.md line 309-317 ✓（双 venue 表述与 literature.md "Kang@arXiv'22 / Kang & Yang@Sensors'23" 一致）

**张冠李戴检查**：无。outline 引用均匹配 literature.md 中的作者 + 会议 + DOI 元组。

**新引用检查**：outline 提及"NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report"5 处 vendor doc，全部在 literature.md 已标 [CHECK: vendor doc 引用方式] — outline 没引入新文献，只是把 literature.md 里已存在的 vendor doc [CHECK] 透传到 §3.4 末尾的"未决问题"中。

### 回归性

PASS — **未发现 outline 引入 Phase 0/1 没有的新数字 / 新引用 / 新 claim**。

逐项核对：
- **数字**：86.6% MAC%, 593K cycles, 5.95 ms / 8.69 ms, 168 fps / 115 fps, 36.9k LUT, 128 BRAM, 82 DSP, 68.4 MHz, 22+24+20 case PASS, 9057 cycles N=2, 11006 cycles N=2 双层, 3833 vs 5569 N=4/N=2 1.45×, 8192×128 Shortcut Bank, 4.9 MB / 10 KB ring — 全部在 contributions.md §4 已经存在
- **引用**：14 篇核心 prior art (Alwani / Kang / Liu / MAERI / Eyeriss-v2 / cuDNN / Sub-pixel / NVDLA / TPU / Eyeriss / Gemmini / Buffets / Snowflake / Angel-Eye) + 21 篇 framing 引用 — 全部在 literature.md A-G 七大类内
- **claim**：narrative A+B 混合 / "硬件保持简洁固定 + 编译器侧填满 PE" / "row-level vs layer-level granularity" / "compiler-only vs hardware-reconfigurable" — 与 contributions.md §7.1 narrative 表 + literature.md "可量化优势 candidate" line 537-540 完全一致

outline 严格做到了"只组织已有内容"。

### 通过原因

- **17 条贡献全部归位**且无重复主章归类，已主动揭示主 Agent 提示词的 off-by-one 错并对齐
- **§4/§5/§6 抽样核 3 章**所有具体模块 / 路径 / 描述都能在 RTL/ + toolchain/ + docs/ 找到一一对应；无"声称已实现但代码不存在"的情况
- **[CHECK]/[TBD] 标记位置全部合理**：所有未确定数字（PE 利用率实测、DDR 节省比、ResNet multi-core 实测）都标了 [CHECK]，所有用户决策项（目标会议、narrative、开源、节归并）都标了 [TBD]，无误标也无漏标
- **5+ 处文献引用抽样**全部匹配 literature.md 元数据，无张冠李戴，无幻觉引用
- **回归性**：outline 严格做到"只组织 Phase 0/1 已有内容"，未引入新数字 / 新引用 / 新 claim

### 给 Writer 的轻微建议（非阻断，仅提示）

1. line 306 "✅ **18 条 contribution 全部归位**" 与 line 273 "17 条" + line 392 "17 条" 自相矛盾。建议把 line 306 的 "18" 改为 "17" 保持文档内部一致。这处不算编造、不影响 PASS。

2. outline §6.4 line 188 写 "[CHECK: §6.4 N=4 wslice1 1.45× speedup 是否近线性需 baseline 归一化（contributions.md §8.1 #9）]" — 1.45× 数字本身（3833 vs 5569 cycles）是已确定的实测数据，不需要标 [CHECK]；标的是"是否可视为接近线性"的论证文字。建议在 Phase 5 写作时把 [CHECK] 表达精化为 "[CHECK: 是否接近线性 scaling 的论证 — 与 N=2 baseline 对比时分子分母统一]"，避免读者误解为数字本身待测。这处也不算阻断问题。
