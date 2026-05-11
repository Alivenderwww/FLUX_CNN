# 技术评审报告 Phase 1（contributions.md）

## 第 1 次评审（2026-04-30）

### 判定
PASS

### 评审范围

- 抽样的 6 条贡献编号：**C1.1（去中心化 valid-ready 流水）**、**C1.3（PARF per-col 拆分）**、**C1.4（SDP residual fusion）**、**C2.1（Ky-fold）**、**C3.1（DMA + axi_dm IP）**、**C3.7（W slice）** + 加测 **§4.1 综合资源表 与 §4.3 端到端延迟一致性**（覆盖 4 大分类 + 实测口径校验）
- 工具表自检：本阶段未需 `mcp__elicit__*`（literature.md 在 Phase 0 已 PASS，本次仅做 contributions ↔ literature.md 对位检查）；`Read / Grep / Glob / Bash` 全可用；`check-citations` skill 本次未触发（无引用真实性疑点出现，所有提到的 Author@Conf'YY 引用都能在 literature.md 第 4/5 次启动后的 entry 中找到）

### 6 条抽样核验结果

#### C1.1 去中心化 valid-ready 流水（核内无中心 FSM）

- **代码追溯**：✅ 全部对得上
  - `RTL/line_buffer.sv` / `RTL/mac_array.sv` / `RTL/parf_accum.sv` / `RTL/parf_col.sv` / `RTL/ofb_writer.sv` / `RTL/wgt_buffer.sv` 全部存在（`Glob RTL/**/*.sv` 验证）
  - `RTL/sequencer.sv` 头注释（line 5–14）写"从 desc_fifo pop 一条 descriptor，解析字段，发 start pulse 给核流水 + IDMA + ODMA"——与 contributions"仅做 cross-block 启动同步而非中心调度"描述一致
  - `parf_accum.sv` 头注释（line 4–7, 20–23）明确写 FILL/DRAIN overlap 与 first_round 跨 tile 衔接规则，与 contributions"FILL/DRAIN overlap 让 parf_accum tile N 的 drain 隐藏在 tile N+1 fill 的 first_round 里"完全对得上
- **数字追溯**：✅ "22-case 链式回归 86.6% MAC%" 来源 STATUS line 355 已验证
- **描述准确性**：✅ 与代码一致

#### C1.3 PARF 拆分为 per-col SRAM

- **代码追溯**：✅ `RTL/parf_accum.sv` 头注释 line 5–8 直接写"内部把 PARF 拆成 NUM_COL 个独立 parf_col 子模块, 每列各自一块 SRAM, 外壳负责 FILL/DRAIN 计数、握手、地址生成. 所有列共享 wr_addr / rd_addr / we"——与 contributions 描述一字不差
- **数字追溯**：[CHECK] 标记位置合理（"parf BRAM 数量贡献单独估算待补"未编造数字，标记得当）
- **描述准确性**：✅

#### C1.4 SDP 后处理 Bias / Residual / Shift / ReLU / Clip 融合（R.1+R.2）

- **代码追溯**：✅ `RTL/sdp.sv` 头注释 line 7–17 给出 11 步流水（contributions 简化为 7 步表述但与代码方向一致）；line 19–24 明确写"bias 已经从 mac_array 的 first_round seed 路径迁移到这里 (post-PARF)"对应 R.1，"shortcut_mult / shortcut_shift 做量化 scale 对齐"对应 R.2
- **设计文档**：✅ `memory/sdp_residual_fusion.md` + `memory/bias_to_sdp_refactor.md` 都存在
- **数字追溯**：✅ "8192×128 Shortcut Bank → 32 BRAM" 与 STATUS line 43 一致
- **描述准确性**：✅

#### C2.1 Ky-fold（Cin<16 时把 Ky 折到 cin_fake）

- **代码追溯**：✅ `Grep` 命中 `toolchain/hw_files.py` line 177 `def compute_fold_params(K, NUM_CIN, HW_PE=16)` / line 196 `fold_weights` / line 223 `fold_input`——三个函数全存在
- **轻微差异**：contributions line 80 写 `compute_fold_params(K, Cin, HW_PE)` 而代码参数名是 `NUM_CIN`，属于功能等价的命名简化，不构成事实错误
- **数字追溯**：[CHECK: 实测 PE 利用率百分比待 project-analyst 跑 fold 模式 case 时取 cycles 算出] 标记得当，未编造具体数字
- **描述准确性**：✅

#### C3.1 DMA 子系统采用 Xilinx axi_dm IP + 轻量 *_ctrl 控制器

- **代码追溯**：✅ 所有引用文件都存在（`RTL/DMA/{idma_ctrl, wdma_ctrl, odma_ctrl, rdma_ctrl, mm2s_arb, dfe}.sv` + `RTL/AXI4/axi_m_mux.sv` 全部命中）
- **idma_ctrl.sv 头注释 line 4–7** 明确写"这个模块本身**不是** DataMover, 是 DataMover MM2S 通道的指令控制端"——与 contributions"仅做 cmd 生成 + done 检测的控制端"完全一致
- **数字追溯**：✅ "+0.5%" 来源 STATUS line 21 已验证；"burst_size 16→256" commit f238de1 在 git log 中能查到（HEAD 之前的近期 commit）
- **Syn 脚本**：✅ `Syn/gen_axi_datamover.tcl` 存在
- **描述准确性**：✅

#### C3.7 W slice (Mode C)

- **代码追溯**：✅
  - `RTL/DMA/idma_ctrl.sv` line 68 输入 `cfg_ddr_ifm_row_stride`、line 206 `cur_addr <= cur_addr + cfg_ddr_ifm_row_stride`（Grep 命中）——与 contributions"加 cfg_ddr_ifm_row_stride ~20 行，让 cur_addr 用 row_stride 推进而不是 cmd_btt"完全一致
  - `toolchain/hw_files.py` line 808 `compute_w_slice_geom`、line 859 `derive_w_slice_cfg` 都存在
- **数字追溯**：✅ "20/20 W slice case bit-exact PASS" + "wslice1 N=4 3833 cycles vs N=2 5569 cycles = 1.45× speedup" 与 STATUS §2.8 表中 line 178/192 完全对得上
- **N=4 IP 实测来源**：`Syn/ip_multicore_summary.txt`（时间戳 2026-04-30 21:16:46）给出 axi_4to5 + axi_lite_1to4，与 STATUS §2.8 line 233–235 描述的 IP gen 一致
- **[CHECK: 1.45× 是否近线性] 标记合理**——线性归一化口径确实有解释空间
- **描述准确性**：✅

#### 加测：§4.1 综合资源表 + §4.3 端到端延迟自洽

- **N=2 多核数据**（routed）：`Syn/reports/utilization.rpt` 的 multicore_top 行：LUT 74386 / FF 26927 / BRAM36 256+2 RAMB18 / DSP 164——与 contributions 表 100% 一致 ✅
- **单核数据**：contributions 写 36,942 LUT / 13,167 FF / 128 BRAM / 82 DSP / Fmax 68.4 MHz / WNS=-4.618 ns；当前 reports/ 已被多核综合覆盖（utilization.rpt 中的 gen_core[0].u_core 是 34666 LUT / 13187 FF），但**单核独立综合是历史快照**，STATUS.md line 34–38 完整给出该快照数字（与 contributions 完全一致）。这不是编造，是有 STATUS 来源支撑
- **派生数字自洽性**：
  - 593K cycles / 100 MHz = 5.93 ms（contributions 写 5.95 ms，舍入差 ✅）；1/5.95ms ≈ 168 fps ✅
  - 593K cycles / 68.4 MHz = 8.67 ms（contributions 写 8.69 ms，舍入差 ✅）；1/8.69ms ≈ 115 fps ✅
  - 两口径相互一致，无矛盾

### 实测数据自洽性

- 所有"实测数字 / 派生数字"相互一致，无矛盾：
  - 593K cycles / 100 MHz / 168 fps / 5.95 ms 互相能反算
  - 593K cycles / 68.4 MHz / 115 fps / 8.69 ms 互相能反算
  - N=2 综合数字 (74386 LUT / 256 BRAM / 164 DSP) = 单核 (37K LUT / 128 BRAM / 82 DSP) × 2 + 顶层 ~4600 LUT + crossbar IP（与 utilization.rpt 顶层一致）
  - 22-case 链式 + 24-case smoke + 20-case W slice = 66（4.2 表写"约 67 个"，含 M1.5 1 case 和 M2 1 case = 67 ✅）
- **唯一需关注的点**：单核 LUT/FF 数字来自历史综合（不是当前 reports/ 的多核 routed），但已通过 STATUS line 34–38 完整记录，可追溯，不算编造

### [CHECK]/[TBD] 标记合规性

✅ 总体合规

抽查 16 处 [CHECK]：
- C1.1 line 31 [CHECK: 单层 vs 整网 MAC% 口径区分待用户确认] — 合理（确实是口径选择）
- C1.3 line 51 [CHECK: parf BRAM 数量贡献单独估算待补] — 合理（未编造数字）
- C2.1 line 83 [CHECK: 实测 PE 利用率百分比待 project-analyst 跑 fold 模式 case 时取 cycles 算出] — 合理（明确未实测）
- C2.2 line 93 [CHECK: 待跑对照 run] — 合理
- C2.2 line 94 [CHECK: S2D 在加速器领域的引用谱系不清晰] — 合理
- C2.5 line 121 [CHECK: target vs Fmax 实际 fps 应为 168×0.684 ≈ 115 fps] — 合理（明确两口径冲突）
- C3.1 line 135 [CHECK: vendor doc 引用方式] — 合理
- C3.5 line 171 [CHECK: 实测 multi-core full ResNet 还没跑] — 合理
- C3.7 line 190 [CHECK: 是否可视为接近线性] — 合理
- 4.3 表 line 234 [CHECK: 等加 Wall_us 实测列] — 合理
- 4.4 表 line 240–242 多列 [CHECK] — 合理
- 4.5 表 line 258 [CHECK: ResNet 完整 11-layer 多核 chain 实测 cycles 还没跑] — 合理
- §8.1 列出 16 项 [CHECK] 与全文标记一一对应

抽查 7 处 [TBD]：
- §7.1 line 370 [TBD: 主 Agent 决定 narrative 走 A/B/C/D 哪条] — 决策项，标 [TBD] 正确
- §7.2 line 379 [TBD: 是否开源] — 决策项 ✅
- §8.2 列 7 项 [TBD] — 全部是用户决策项，标 [TBD] 正确

**未发现"该标的没标"或"不该标的乱标"**：
- 检查"5.95 ms / 168 fps / 86.6% MAC% / 593K cycles" 等核心数字：均有 STATUS line 355 实测来源，未标 [CHECK] 正确
- 检查"36,942 LUT / 128 BRAM / 82 DSP / 68.4 MHz / WNS -4.618 ns"：均有 STATUS line 34–38 来源，未标 [CHECK] 正确
- 检查"+2.8% AXI 仲裁开销 / 9057 cycles / 11006 cycles / 8808 cycles 单核 / 5569 cycles wslice1 N=2 / 3833 cycles wslice1 N=4"：均有 STATUS §2 / §2.5 / §2.8 来源 ✅

### prior art 对比真实性

✅ 全部能对位 literature.md，无张冠李戴

抽查 12 处对位（按 contributions.md 出现顺序）：
- C1.1 引 Gemmini@DAC'21 / NVDLA / VTA@IEEE Micro'19 / Buffets@ASPLOS'19 — literature.md §A line 147 / line 157 / line 436 / §B line 203 全有
- C1.2 引 Kang AoCStream@arXiv'22/Sensors'23 / Alwani@MICRO'16 / Liu@TNNLS'21 — literature.md §C line 299 / 309 / 319 全有，且 venue 与 Phase 0 第 5 次启动修正一致
- C1.4 引 NVDLA / Liu@TNNLS'21 / He@CVPR'16 — 对应 line 157 / 319 / 462
- C2.1 引 cuDNN@arXiv'14 / MAERI@ASPLOS'18 / Eyeriss-v2@JETCAS'19 — 对应 line 335 / 183 / 137
- C2.2 引 Shi et al.@CVPR'16 — 对应 line 345 ✅
- C2.4 引 Interstellar@ASPLOS'20 / Ma et al.@FPGA'17 — 对应 line 426 / 366
- C2.5 引 TVM@OSDI'18 / VTA / fpgaConvNet@TNNLS'19 — VTA 已在 line 436 找到
- C3.5 引 Simba@MICRO'19 — literature.md 中存在
- 5.1 表对比的 TPU v1 / Eyeriss / Snowflake / Angel-Eye / Aydonat / Lu Winograd / Liu Full-Stack / VTA — 全部对应 literature.md entry

**所有引用格式 `Author@Conf'YY` 与 literature.md entry header 一致**。Kang AoCStream 双 venue 形式（arXiv'22 + Sensors'23）在 contributions.md 与 literature.md 都用了，吻合 Phase 0 第 5 次启动修正后的格式。

### 通过原因

1. **6 条抽样贡献的代码定位 100% 可追溯**：所有引用的 RTL 文件、Python 模块、函数名、参数名均经 `Glob`/`Grep`/`Read` 验证存在，描述与代码注释/语义一致
2. **所有数字相互自洽**：593K cycles / 100 MHz target / 68.4 MHz Fmax / 5.95 ms / 8.69 ms / 168 fps / 115 fps / 86.6% MAC% / 67 个 case 都能交叉反算且一致
3. **N=2 多核综合数字（74386 LUT / 256 BRAM / 164 DSP）与 utilization.rpt 100% 匹配**；单核数字（36,942 LUT / 128 BRAM / 82 DSP）虽然 reports/ 已被覆盖但 STATUS.md 完整记录
4. **[CHECK]/[TBD] 标记位置合规**：16 处 [CHECK] 全部为"待实测"或"待决策"，未发现"未实测却写成实测"的情况；7 处 [TBD] 全部是用户决策项
5. **所有 prior art 引用都能在 literature.md 找到对应 entry**，无张冠李戴或幻觉，引用格式与 Phase 0 修正后一致
6. **contributions §1–§3 的"差异化论证"部分诚实——多次出现"新颖性强度自评：偏弱/工程化贡献/增量"等克制表述**，避免过强 claim；core claim（C2.1 Ky-fold / C2.2 S2D / C3.7 W slice）也明确标注 reviewer 阶段需补查 prior art 的风险

### 给 Writer 的修订建议（轻微 / 非阻塞）

虽 PASS，但提示两个**轻微级**可优化点（不影响通过判定，留作后续 Phase 微调时参考）：

1. **§4.1 表单核列的来源备注更显式**：当前 `Syn/reports/` 下的 utilization.rpt 是 N=2 多核 routed 报告（gen_core[0] 是 34666 LUT 而不是单核独立综合的 36942 LUT）。建议在表脚注里加一行 "单核数据来自历史 N=1 独立综合，详见 STATUS.md line 34–38；当前 reports/ 已被 N=2 多核综合覆盖，多核 routed 中单核子模块 = 34,666 LUT 略低（因 multicore wrapper 优化）"，避免审稿人问"为什么单核 + 单核 ≠ 多核"。
2. **C2.1 line 80 的函数签名表述**：`compute_fold_params(K, Cin, HW_PE)` 与代码实际签名 `compute_fold_params(K, NUM_CIN, HW_PE=16)` 仅参数名差异，建议改写成 `compute_fold_params(K, NUM_CIN, HW_PE)` 与代码一致，纯命名同步事项。
