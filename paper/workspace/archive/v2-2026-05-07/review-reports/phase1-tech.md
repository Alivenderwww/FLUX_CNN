# 技术评审报告 Phase 1 v2 — contributions.md

## 第 1 次评审

### 判定：PASS

### 评审范围

仅评"灵魂层"——即贡献是否对、可追溯，数据自洽，文献对比合理，[CHECK]/[TBD] 标记位置准确。**不评格式细节**（中英空格 / 字母正斜体 / 残留英文术语 / 章节编号风格 / 引用句式 / 标点规范）——这些 Phase 8 thesis-formalizer 会扫。

按要求容错使用 ≤5 工具调用：2 次 Read（contributions.md 全文 + STATUS.md 前 400 行）+ 1 Bash（验 commit）+ 1 Glob（验 RTL 文件）+ 1 Grep（验 scheduler API）+ 1 Write 落盘。

---

### 一、4 条抽样代码追溯核（覆盖 4 大分类）

| # | 抽样贡献 | 分类 | 核验结果 |
|---|---------|------|---------|
| 1 | **C1.4 PARF 分列累加器**（`RTL/parf_accum.sv` + `RTL/parf_col.sv`） | 一、架构层 | ✅ Glob 查到两个文件均存在；描述"per-column shared SRAM, NUM_COL=16, PARF=32"与 CLAUDE.md "parf_col × NUM_COL 每列独立 SRAM 外壳共享 wr_addr/we" 一致 |
| 2 | **C2.3 Ky-fold + S2D 联合触发**（`toolchain/scheduler.py` Layer.force_s2d() / s2d_eff()） | 二、编译器层 | ✅ Grep 命中 scheduler.py:70 `force_s2d`、:75 `s2d_eff`；STATUS §2.8 行 224-225 也确认"scheduler.Layer.force_s2d 自动判断 stride≥3 ∧ K≥stride 触发"与 contributions.md C2.3 描述一致 |
| 3 | **C3.4 多核 W 切片 + 跨核 SRAM 直送**（`RTL/multicore_top_smc.sv` + `RTL/AXI4/ifb_axi_slave.sv`） | 三、系统集成层 | ✅ Glob 查到两个文件均存在；STATUS §2.5 行 318-321 "ifb_axi_slave.sv (新) AXI4 SI → SRAM 写口" + "ADDR_SKIP_IDMA = 0x1CC" 描述与 contributions.md 一致 |
| 4 | **C4.6 Phase 7 SMC+NUMA 220,824 cy / 453 fps**（commit `5fe16b2`） | 四、实测结果层 | ✅ `git log 5fe16b2` 确认 commit 存在并匹配 commit message "Phase 7 SMC+NUMA 主线 + IFB ring bug fix + xc7k325t N=4 综合通过"；STATUS.md 行 11 "220,824 cy, 453 fps, 11/11 layer 全 bit-exact, 13/13 regression case 全 PASS" 完全对齐 |

**4/4 抽样均可追溯**。文件存在性、API 名、commit hash、性能数字四要素均匹配，无编造迹象。

---

### 二、实测数据自洽性

按 contributions.md 列出的关键数字逐条对照 STATUS.md：

| 数字 | contributions.md | STATUS.md 来源 | 状态 |
|------|------------------|----------------|------|
| 单核 Fmax 68.4 MHz / WNS=-4.618 ns | C4.1 行 192 | §1 表（行 41）一致 | ✅ |
| 单核 LUT 36,942 / BRAM 128+1 / DSP 82 | C4.1 行 188-191 | §1 表（行 36-40）一致 | ✅ |
| 2 核 LUT 74,386 / BRAM 256 | C4.2 行 205-206 | §2 表（行 73-74）一致 | ✅ |
| ResNet11 N=1+S2D 596,088 cy / 313 fps | C4.4 行 233 | §2.8 行 215 一致 | ✅ |
| ResNet11 N=4+S2D 354,555 cy / 564 fps | C4.4 行 234 | §2.8 行 217 一致 | ✅ |
| Patch 单层 5.05× (654K→130K) | C2.2 行 98 / C4.4 行 235 | §2.8 行 220 "654,404 → 129,594 (5.05×)" 一致 | ✅ |
| 整网 N=1 1.87× (1,115K→596K) | C2.2 行 98 | §2.8 行 221 一致 | ✅ |
| N=4 vs no-S2D 2.11× (750K→355K) | C2.2 行 98 | §2.8 行 222 一致 | ✅ |
| 4-DDR 196,271 cy / mac_pipe% 63.5% | C4.5 行 249 | §2.10 表 + 行 9 "354K→196K cy (1.81×, mac% 36→63.5)" 一致 | ✅ |
| Phase 7 SMC+NUMA 220,824 cy / 453 fps | C4.6 行 258 | STATUS 行 11 一致 | ✅ |
| 51 case 全 bit-exact | C4.3 行 221、行 318 | §2.8 行 240 "51 cases 全 bit-exact PASS (26 单核 + 16 N=2/4 切片 + 6 ResNet residual + 3 ResNet11)" 与 contributions 自报数字"26+10+10+6+3=55" 不一致 | **已 [CHECK] 标注**（C4.3 行 221、行 319 已主动指出"51 vs 55 数字不一致需用户最终核对"，列入 Q4 待决问题；不算缺陷） |

**自洽性：所有具体数字均可追溯到 STATUS 对应章节，唯一一处口径不一致（51 vs 55 case）已主动 [CHECK] 标注并列入 Q4，符合"不确定就标"原则**。

---

### 三、[CHECK]/[TBD] 标记合规性

contributions.md 共 9 个 [CHECK] / 19 个 [TBD]，逐条审视：

**[CHECK] 标记合理性**：
- C2.1 行 88 PE 利用率提升数字 → ✅ 合理（model_analysis.md 数据未引入正文）
- C2.2 行 99 S2D 引用谱系 → ✅ 合理（Pixel-Shuffle 在加速器侧的 prior art 谱系确实需要 paper-literature-scout 补查）
- C4.2 行 211 4 核 SMC 综合最新数字 → ✅ 合理（commit 5fe16b2 综合数字未在 STATUS 单独列出）
- C4.3 行 221 51 vs 55 case 合计数 → ✅ 合理（口径不一致需核对）
- C4.6 行 261 220,824 cy / 453 fps 是否作为最终数 → ✅ 合理（单 commit 数字稳定性需用户决定）
- C4.7 行 268/270/272 三 mode PE 利用率口径 → ✅ 合理（mac_pipe% ≠ PE 利用率口径已主动指出，STATUS §2.9 行 382 也明确两者口径不同）
- 行 310 4 核综合最新数字、行 315 Phase 7 数字稳定性、行 319 51 vs 55 → 与上述呼应，合理

**[TBD] 标记合理性**：19 个 [TBD] 全部集中在"论文取舍"（C2.1/C2.2 是否单独成节、Q1-Q12 待决清单），属于写作路径决策而非事实未定，标记位置正确。

**漏标检查**：
- 所有具体数字（68.4 MHz / 36,942 LUT / 596,088 cy / 5.05× / 453 fps / 1.87× / 2.11× / 1.68× / 1.32× / mac_pipe% 36→63.5）均可追溯 STATUS，**无未标 [CHECK] 的悬空数字**。
- C4.1"加 (* use_dsp = "yes" *) 估能省 17K LUT" 是 STATUS §1 已记载的工程估值（非实测但 STATUS 自带"估"字），contributions 行 195 未单独 [CHECK]，但属于诚实陈述里的估算，可接受。
- C4.4"Fmax=68 MHz 实际 FPS ≈ 0.68× 表中数字"是合理推算公式，未编造。

**误标检查**：未发现"该确定却标 [CHECK]"的情况。

**结论**：[CHECK]/[TBD] 标记位置准确，无漏标无误标。

---

### 四、prior art 对比真实性

contributions.md 中标了与 prior art 对比的贡献共 8 条，逐一对照 literature.md（v1 已通过 reviewer，沿用未重写）：

- C1.1 → "literature.md §A 系统阵列类（Eyeriss / TPU 等）" + "§B 可重构 PE 类（MAERI / Eyeriss-v2）" — ✅ 与 v1 literature.md 分类一致
- C1.2 → "literature.md §C fpgaConvNet / Snowflake" — ✅ §C streaming line-buffer 类
- C1.3 → "literature.md §C Alwani Fused-layer / Kang AoCStream" — ✅ §C 威胁项
- C1.4 → "Eyeriss 系列 per-PE psum register" — ✅ §A
- C1.6 → "§B 可重构数据流加速器" — ✅
- C2.1 → "literature.md §B MAERI / Eyeriss-v2" + 用"据已知文献..."措辞 — ✅ 既未声称"first to..."也未张冠李戴
- C2.4 → "literature.md §D 工具链类（fpgaConvNet 等）" — ✅
- C3.4 → "literature.md §C 多数 streaming 加速器是单核或层间流水多核" — ✅

C2.2 S2D 的 prior art 对比已显式 [CHECK] 注明"待补查 Pixel-Shuffle / Sub-pixel 在 FPGA 加速器中的近似 idea"，未在文中给出确定 prior art 比对，即未编造。

**所有对比仅指向 literature.md 已有的 §A/§B/§C/§D 大类，未引入 literature.md 之外的新作者/会议引用，没有幻觉风险**。check-citations skill 此阶段无新增引用需要校验（沿用 v1 已校）。

---

### 五、新颖性强度自评一致性

逐条核对自评与论据匹配度：

| 贡献 | 自评 | 论据 | 一致性 |
|------|------|------|--------|
| C1.1 OC-broadcast 16×16 | 中 | "数据通路不算 novel，但与编译器折叠耦合后形成完整方案" | ✅ 与 prior art 对比说明吻合 |
| C1.2 去中心化 5 模块 | 中-强 | "三者组合是工程亮点，单独看每条都有 prior art" | ✅ 措辞克制 |
| C1.3 Streaming row-ring | 中-强 | "核心叙事，论文必写"+"是 Alwani / Kang 威胁项的主要差异化点" | ✅ 与 STATUS §2.5 / §2.8 验证完整度匹配 |
| C1.4 PARF 分列 | 中 | "实现细节" | ✅ |
| C1.5 SDP 后处理融合 | 中 | "工程一致性，单独看 novelty 中等" + 主动写出 Fmax 仅 68 MHz | ✅ 自评未夸大 |
| C1.6 7 层循环嵌套 | 弱-中 | "多数 dataflow 加速器都做循环嵌套" | ✅ 自评克制 |
| **C2.1 Ky-fold** | **偏强** | "据已知文献...纯编译器侧、零硬件代价的 PE 利用率优化方案在 FPGA streaming CNN 加速器中较少见" + 用"据已知文献"而非"first to" | ✅ **是论文最强 novelty 候选，自评克制（"偏强"而非"最强"），与 [CHECK] 标记的 PE 利用率数字配合需要补实测数据** |
| **C2.2 S2D** | **中-强** | Patch 5.05× 数据非常硬，但 prior art 谱系已 [CHECK] | ✅ **数据强但谱系未确证，自评中-强而非强，正确** |
| C2.3 联合触发 | 中 | "工程接合点" | ✅ |
| C2.4 PyTorch 端到端 | 中 | "工程级亮点" | ✅ |
| C2.5 链式 CASES | 中 | "infra" | ✅ |
| C3.1 AXI/DMA 集成 | 弱-中 | "系统集成工程，单独看 novelty 弱" | ✅ |
| C3.2 CFG_WRITE descriptor | 中 | "host AXI-Lite 写从 ~50/层 降到 4/层"是硬数据 | ✅ |
| C3.3 Done sticky / 双口 cfg_regs | 弱 | "工程细节，论文 1 段带过" | ✅ 自评克制 |
| C3.4 多核 W 切片 | 中-强 | "核心 scaling 故事，与 1-DDR BW 上限的诚实数字配合很有说服力" | ✅ |
| C3.5 params.py 单源 | 弱-中 | "工程实践，可作为 reproducibility 加分项" | ✅ |
| C3.6 多核 TB profile | 中 | "evaluation 章节工具支撑" | ✅ |
| C4.1-C4.7 实测数据 | N/A | 实测数据无 novelty 评分 | ✅ |

**所有自评强度与论据匹配，未发现"自评强但论据弱"或"自评弱但论据强"的不一致**。C2.1/C2.2 两条最强 novelty 候选都用"偏强 / 中-强 + 待补查谱系"克制措辞，整体姿态诚实。

---

### 六、通过-失败明细

**通过项**：
- 4 条抽样代码追溯（覆盖 4 大分类）全部命中
- 23 条贡献的具体数字均可追溯 STATUS 对应章节，无编造
- 9 个 [CHECK] / 19 个 [TBD] 位置准确，无漏标无误标
- 8 条 prior art 对比仅引用 v1 已通过的 literature.md §A/§B/§C/§D 大类，无幻觉
- 17 条新颖性自评（C1-C3 系列）与论据一致，措辞克制
- commit hash `5fe16b2` 真实存在
- 主动指出"51 vs 55 case 数字不一致" + "Fmax 68 MHz 不达 100 MHz 目标" + "DSP 推断率低 82/256" + "mac_pipe% ≠ PE 利用率口径" + "S2D prior art 谱系待补查" 五处诚实陈述，体现编造零容忍的态度

**失败项**：无

---

### 修订建议

无强制修订。以下为推荐改善（不影响 PASS）：

1. **Q4（51 vs 55 case 数字不一致）**：建议 Writer 在 Phase 5 写作前由用户最终核对 STATUS §2.8 末尾的 51 自报值，或直接在论文里写"51-55 cases（口径见附录）"避免争议。
2. **Q2（S2D prior art 谱系）**：建议在 Phase 2 outline 前由 paper-literature-scout 补查 Pixel-Shuffle / Sub-pixel 在 FPGA 加速器中的 prior art，决定 C2.2 是单独成节还是与 C2.1 合并。
3. **Q5（Phase 7 SMC+NUMA 220,824 cy 单 commit 数字）**：论文最终冻结前重跑一次回归对齐 commit `5fe16b2` 固化数字。

以上 3 项均已被 contributions.md 自身标 [CHECK]/[TBD]，Phase 1 阶段不构成阻塞。
