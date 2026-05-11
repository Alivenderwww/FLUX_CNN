# 技术评审报告 Phase 2 v2 — 论文大纲

## 第 1 次评审

### 判定：PASS

---

## 评审范围声明

按 `_AGENT_ROSTER.md` 灵魂层 vs 格式层分工，本评审只管：
- 贡献覆盖完整性
- 章节内容与代码事实匹配（抽 §3 / §4 / §5）
- prior art 三处威胁的应对到位与否
- `[CHECK]` / `[TBD]` 标记位置合理性
- 章节结构是否符合 `chinese-thesis-spec.md` §11.1 本科论文范式
- 大纲是否引入 Phase 0/1 没有的新事实/新引用

不管：中英空格、字母正斜体、残留英文术语、引用句式（属 Phase 8 polisher 范畴）。

---

## 1 本科论文结构合规

对照 `chinese-thesis-spec.md` §11.1 强制范式逐项核：

| 范式要求项 | outline.md 体现 | 状态 |
|---|---|---|
| 中文摘要 + 关键词 | 元信息节"一句话摘要"已铺垫；关键词在元信息层未直接列，但属 Phase 5 写作时落地，大纲层已声明 6 章 + 附属节包含摘要 | ✅ 合规（大纲层"6 个正文章 + 摘要/Abstract/主要符号表/参考文献等附属节" 第 11 行明示） |
| Abstract + Key words | 同上 | ✅ |
| 主要符号表（独立成节） | 第 11 行附属节明示包含 | ✅ |
| 1 绪论（含 1.1 背景 / 1.2 国内外现状 / 1.3 研究目标研究内容 / 1.4 论文组织结构） | §1 完整含 1.1 / 1.2.1-1.2.5 / 1.3 / 1.4 | ✅ 完全匹配（1.2 综述 ≈4500 字符合"~8000 字"中的综述主体） |
| 独立的理论基础章 | §2 "卷积神经网络与FPGA硬件加速技术原理" | ✅ |
| 设计章（总体方案与选择论证） | §3 "固定阵列CNN加速器整体方案" | ✅ |
| 核心实现章（通常最长） | §4 "硬件实现与编译器优化" 9000 字最长 | ✅ |
| 验证章（功能/板级/性能/对比） | §5 "系统验证与实验分析" 含 5.3 功能 / 5.4 板级综合 / 5.5 性能 / 5.7 对比 | ✅ 完全匹配 |
| 结论与展望（强制拆 6.1/6.2/6.3） | §6 含 6.1 结论 / 6.2 创新点 / 6.3 展望 | ✅ 完全匹配 |
| 章号格式（阿拉伯无"第 N 章"前缀） | §11 第 33 行明示"一级章号用阿拉伯数字（无第 N 章前缀）" | ✅ |
| 参考文献（独立无章号） | 元信息附属节明示包含 | ✅ |
| 章末小结 | §2.6 / §3.6 / §4.6 / §5.8 全部有"本章小结" | ✅ |
| 章首段范式 | §3.1 / §4.1 / §5.1 全部声明"章首段范式" | ✅ |

**结论**：6 章正文结构 + 附属节齐全，与 §11.1 范式 1:1 对应。

---

## 2 贡献覆盖

contributions.md v2 共 24 条贡献（C1.x 6 条 / C2.x 5 条 / C3.x 6 条 / C4.x 7 条 = 24）。outline.md "贡献-章节映射表"逐条核对：

| 类 | 贡献 ID | 主章节 | 是否归位 |
|---|---|---|---|
| C1.x 架构（6 条） | C1.1-C1.6 | §4.2.1-§4.2.6 | ✅ 全部 |
| C2.x 编译器（5 条） | C2.1-C2.5 | §4.3-§4.4 | ✅ 全部 |
| C3.x 系统集成（6 条） | C3.1-C3.6 | §3.5 / §4.4 / §4.5 / §5.2.3 / §5.6 | ✅ 全部 |
| C4.x 量化结果（7 条） | C4.1-C4.7 | §5.3-§5.6 | ✅ 全部 |

**抽样核验**：
- C2.3（Ky 折叠 + S2D 联合触发）→ outline §4.3.4 + §3.5.3（编译器/硬件视角对齐），与 contributions.md "scheduler.Layer.force_s2d() / s2d_eff()" 描述一致 ✅
- C3.6（多核 TB 结构化 profile）→ outline §5.2.3 一段带过 + §5.6 数据来源，符合 contributions.md "C3.6 是论文 BW 分析数字可信的基础"定位 ✅
- C4.5（4-DDR PoC）→ outline §5.6 全节 + §6.3.1 已知不足，符合 contributions.md "可作为 limitation analysis 的支撑数据"建议 ✅
- C1.4（PARF 分列累加器）→ outline §4.2.4 主节，无次章节呼应，符合 contributions.md "实现细节 / 是否单独成节 [TBD]" 建议 ✅

**结论**：24/24 全部归位，无遗漏。outline.md 中"23 条"的口径来自任务文本，§259 行已明示"用户任务文本中 23 条为概数"，并在覆盖率验证段落同时引用 23 与 24，标注未引入新概念，仅是文本计数。

---

## 3 抽 3 章 vs 代码事实匹配

### §3.3 总体架构（代码事实抽样）

outline §3.3.1 "两层结构：核流水（5 模块 + cfg_regs）+ DMA 子系统（idma_ctrl / wdma_ctrl / odma_ctrl + axi_dm IP + mm2s_arb + axi_m_mux + axi_lite_csr）"
- 与 CLAUDE.md 项目总览第 1 段"核流水 5 模块 + 共享 cfg_regs"完全一致 ✅
- 与 contributions.md C3.1 列出的模块名（idma/wdma/odma/rdma_ctrl + mm2s_arb + axi_m_mux + axi_lite_csr + axi_dm IP）一致 ✅
- 注：outline 没列 `rdma_ctrl`（contributions.md C3.1 提到了），但 outline 是高层架构表述，3.3 简提级别允许省略，不算编造

outline §3.3.2 "外部接口：1 主 1 从（AXI4 Master + AXI-Lite Slave）"
- 与 CLAUDE.md "外部看是 1 个 AXI4 Master + 1 个 AXI-Lite Slave"一致 ✅

outline §3.3.3 "多核拓扑（multicore_top + axi_2to3 / axi_4to5 + axi_lite_1to4 fanout）"
- 与 contributions.md C3.4 描述完全一致 ✅

### §4.3 编译器侧 PE 利用率优化（narrative A 主章）

outline §4.3.2 "Ky 折叠：将 Ky 维折叠到 cin_fake 的数学推导（cin_fake = Cᵢₙ × Kᵧ）"
- 与 contributions.md C2.1 描述（"把 Ky 维度折叠到 cin_fake 维"）一致 ✅
- 与 CLAUDE.md "--ky-fold : Cin<16 时把 Ky 折到 cin_fake"一致 ✅

outline §4.3.3 "S2D：stride² 相位折叠的等价变换（cin_new = stride² × Cᵢₙ）"
- 与 contributions.md C2.2 描述（"把 (kx % stride, ky % stride) 的 stride² 个相位折到 cin_new = stride² × Cin"）一致 ✅
- 与 CLAUDE.md "--s2d : stride≥2 时把 4 相位折到 cin"一致 ✅

outline §4.3.4 "Ky 折叠 + S2D 联合触发与编译器自动决策（scheduler.Layer.force_s2d() / force_fold()）"
- 与 contributions.md C2.3 函数名（`scheduler.Layer.force_s2d()` / `s2d_eff()` / `build_step_cfg_dict`）一致 ✅

### §5 系统验证与实验分析（数据事实抽样）

outline §5.4.1 "单核综合（XC7K325T，LUT 36,942 / FF 13,167 / BRAM36 128+1 / DSP 82 / Fmax 68.4 MHz）"
- 与 contributions.md C4.1 表格逐字一致 ✅

outline §5.5.2 "ResNet-11 整网 cycles / FPS（N=1 596K cy / 313 fps；N=2 450K cy / 444 fps；N=4 354K cy / 564 fps；S2D 单 Patch 层 5.05× — 对应 C4.4）"
- 与 contributions.md C4.4 表格一致（596,088 / 313；450,469 / 444；354,555 / 564）✅
- "Patch 层 5.05×" 与 contributions.md "654K → 130K cycles, 5.05×" 一致 ✅

outline §5.5.4 "Phase 7 SMC + NUMA 主线最新数（N=4 220,824 cy / 453 fps — 对应 C4.6，[CHECK: 是否作为论文最终数 — Q5]）"
- 与 contributions.md C4.6 数字一致（220,824 cy / 453 fps），且诚实保留了 [CHECK]，与 contributions.md "需要用户确认是否作为论文最终数"一致 ✅

outline §5.5.3 "多核扩展加速比（N=2 1.32× / N=4 1.68×）"
- 与 contributions.md C4.4 数字一致 ✅

outline §5.6.1 "1-DDR 与 4-DDR PoC 对比（mac_pipe% 36.2% → 63.5% / wall cycles 354K → 196K）"
- 与 contributions.md C4.5 表格一致（354,566 / 36.2%；196,271 / 63.5%）✅

**抽 3 章结论**：所有具体数字、模块名、函数名、文件名均能在 contributions.md / CLAUDE.md / STATUS.md 中找到对应来源，**未发现编造**。

---

## 4 prior art 三处威胁的三层防御

| 威胁项 | §1 综述 | §3 差异化 | §5 数据兑现 | 三层到位？ |
|---|---|---|---|---|
| Alwani Fused-layer@MICRO'16 | §1.2.3 列入 | §3.4.3 论证差异化（行级 vs 跨层 fused） | §5.7.2 横向对比表 | ✅ 三层全 |
| Kang AoCStream@arXiv'22+Sensors'23 | §1.2.3 列入 | §3.4.3 论证差异化（行级 vs 整图 / activation tile 粒度） | §5.7.2 横向对比表 | ✅ 三层全 |
| Liu Full-Stack@TNNLS'21 | §1.2.3 + §1.2.5 工具链综述 | §3.4.3 论证差异化（单核 layer-serial vs 跨层 pipelined 多 block） | §5.7.2 + §5.7.1 工具链对比 | ✅ 三层全 |

差异化措辞与 contributions.md C1.3 给出的差异化要点（"行级粒度 vs 整图或多层 fused / 硬件不绑定层数 / 编译器决定 strip 粒度"）一致，未引入大纲层不该出现的新差异化数据。

§3.4.3 / §1.2.3 两处都明确标 [CHECK: 写作时落到具体段落]，把"差异化的具体措辞细化"留给 Phase 5，**不在大纲阶段过早承诺数字或具体段落字句**，符合 paper-outliner 工作纪律。

---

## 5 [CHECK] / [TBD] 标记合理性（19 处全核）

逐条审视：

| 标记 ID | 位置 | 合理性 |
|---|---|---|
| TBD-1 | 标题候选 | ✅ 合理（用户决策） |
| TBD-2 | §1.3.3 主要贡献条数（4 vs 5） | ✅ 合理（写作压缩决策） |
| TBD-3 | §2.5.2 是否含 S2D 推导 | ✅ 合理（与 §4.3.3 不重复决策） |
| TBD-4 | §3.5 是否独立成节 | ✅ 合理 |
| TBD-5 | §4.3 是否拆两节 | ✅ 合理 |
| TBD-6 | §4.5 多核是否独立成章 | ✅ 合理 |
| TBD-7 | Q7 — Fmax vs 100 MHz 双标策略 | ✅ 合理（与 contributions.md C4.4 诚实陈述要求一致） |
| TBD-8 | Q9 — Mesh PoC 是否进 | ✅ 合理（与 contributions.md "倾向 future work / 附录"一致） |
| TBD-9 | Q10 — 多 DDR 板 ROI | ✅ 合理 |
| TBD-10 | §6.2 与 §1.3.3 对应 | ✅ 合理 |
| CHECK-1 | Q3 — 4 核 SMC 综合数字 | ✅ 该标（contributions.md C4.2 也标了 [CHECK]） |
| CHECK-2 | Q4 — 51 vs 55 case 数 | ✅ 该标（contributions.md C4.3 也标了不一致） |
| CHECK-3 | Q5 — Phase 7 SMC+NUMA 数字 | ✅ 该标（contributions.md C4.6 也标了 [CHECK]） |
| CHECK-4 | Q6 — 三模式 PE 利用率 | ✅ 该标（contributions.md C4.7 也标了 [CHECK]） |
| CHECK-5 | C2.1 量化数据 | ✅ 该标（contributions.md C2.1 也标了） |
| CHECK-6 | Q2 — S2D 谱系 | ✅ 该标（contributions.md C2.2 也标了） |
| CHECK-7 | literature.md 文献对照数据 5 处 | ✅ 该标 |
| CHECK-8 | vendor doc 引用方式 | ✅ 合理 |
| CHECK-9 | 三处威胁项差异化措辞 | ✅ 合理（写作粒度的标记） |

**结论**：19 处标记位置全部合理，**没有一处"该标却未标"或"不该标却标"的情况**。所有 [CHECK] 都能上溯到 contributions.md 对应条目的 [CHECK]；所有 [TBD] 都属用户决策或写作策略层面，不是事实问题。

---

## 6 文献-章节引用映射

抽 5 处 outline.md 引用，回查 literature.md 是否真有：

| outline 引用 | literature.md 是否存在 | 状态 |
|---|---|---|
| Alwani Fused-layer@MICRO'16 | §C 第 299 行 ⭐ 第 4 次启动新增 | ✅ |
| Kang AoCStream@arXiv'22+Sensors'23 | §C 第 309 行 ⭐ 第 5 次启动 venue 修正 | ✅ |
| Liu Full-Stack@TNNLS'21 | §C 第 319 行 ⭐ 第 4 次启动新增 | ✅ |
| MAERI@ASPLOS'18 | §B 第 183 行 Kwon et al.@ASPLOS'18 | ✅ |
| Eyeriss-v2@JETCAS'19 | §A 第 137 行 Chen et al.@JETCAS'19 | ✅ |
| Pixel-Shuffle / Sub-pixel @CVPR'16 | §D 第 345 行 Shi et al.@CVPR'16 | ✅ |
| Winograd Lu@FCCM'17 | §D 第 356 行 Lu et al.@FCCM'17 | ✅ |
| VTA | §F 第 436 行 Moreau et al.@IEEE Micro'19 | ✅ |
| im2col / cuDNN | §D 第 335 行 Chetlur et al.@arXiv'14 | ✅ |
| ResNet | §G 第 462 行 He et al.@CVPR'16 | ✅ |

**1 处轻微 gap**：outline §1.2.2 / §5.7.1 引用 "Caffeine" 作为 FPGA streaming 横向对比对象，但 literature.md §C 当前条目为：fpgaConvNet / DnnWeaver / Roofline-VGG (Zhang FPGA'15) / Aydonat DLA / Snowflake / Angel-Eye / Alwani / Kang / Liu — 没有 Caffeine 专门条目。

判定：
- 这是**轻微问题**，不算编造（Caffeine = Zhang et al. ICCAD'16 是真实论文）
- outline 阶段引用的是名字而非数据
- 可在 Phase 5 写作前由 paper-literature-scout 补 literature 条目，或将 Caffeine 改为 literature.md 已有的 Roofline-VGG (Zhang FPGA'15)

**不构成 FAIL 条件**（轻微度，且 Phase 8 polisher / Phase 5 写作均可修复）。建议在评审备注中传给 Writer：Phase 5 之前请 literature-scout 补一条 Caffeine 或改为 Zhang FPGA'15。

其他 §1.2.x 引用清单（TPU / Eyeriss / Eyeriss-v2 / Gemmini / NVDLA / MAERI / Tangram / fpgaConvNet / Snowflake / Angel-Eye / Aydonat / im2col / Winograd / Sub-pixel / VTA / NVDLA 编译器）全部能在 literature.md 中找到对应条目。

---

## 7 回归性检查（是否引入新事实/新数据）

逐节扫描 outline.md，与 contributions.md / literature.md / Phase 0 综述比对：

- ✅ 所有性能数字（596K / 450K / 354K / 220,824 / 5.05× / 1.87× / 2.11× / 1.68× / 1.32× / 36.2% / 63.5% / 36,942 LUT / 68.4 MHz / 82 DSP）全部来自 contributions.md C4.x，未引入新数字
- ✅ 所有引用文献全部来自 literature.md（Caffeine 例外见 §6 已说明）
- ✅ 所有模块/函数/文件名（line_buffer / mac_array / parf_accum / wgt_buffer / ofb_writer / parf_col / scheduler.Layer.force_s2d / hw_files.derive_layer_cfg / multicore_top / multicore_top_smc / ifb_axi_slave / axi_m_mux）全部来自 contributions.md / CLAUDE.md
- ✅ "narrative A 主轴 / narrative B 支撑"是 outline 阶段的写作策略选择，非新事实
- ✅ "三层防御"应对策略是 outline 阶段的章节布局策略，非新事实

**结论**：outline.md 没有引入 Phase 0/1 没有的新数据 / 新引用 / 新主张，**回归性合规**。

---

## 8 通过-失败汇总

| 评审维度 | 结论 |
|---|---|
| 本科论文结构合规（§11.1） | ✅ PASS |
| 24 条贡献覆盖完整 | ✅ PASS（全部归位） |
| §3 / §4 / §5 章节内容与代码事实匹配 | ✅ PASS（无编造） |
| prior art 三处威胁三层防御 | ✅ PASS（Alwani / Kang / Liu 均三层全） |
| [CHECK] / [TBD] 标记位置（19 处） | ✅ PASS（合理） |
| 文献-章节引用映射 | ✅ PASS（10/10 抽样命中，1 处轻微 gap "Caffeine" 留给 Phase 5 修复） |
| 回归性（无新事实） | ✅ PASS |

**总判定：PASS**

---

## 9 给 Writer 的非阻塞建议（不影响 PASS）

以下事项不构成 FAIL，但建议在 Phase 5 写作进入前关注：

1. **轻微：Caffeine 条目缺失**——outline §1.2.2 / §5.7.1 引用 Caffeine 但 literature.md §C 无专条。建议 Phase 5 启动前由 paper-literature-scout 补一条（Zhang et al. ICCAD'16 Caffeine: Towards Uniformed Representation and Acceleration for Deep Convolutional Neural Networks），或将 §5.7.1 横向对比改为 literature.md 已有的 Roofline-VGG (Zhang FPGA'15) + DnnWeaver 等。

2. **轻微：23 vs 24 贡献条数口径**——outline §259 行明示用户任务文本"23 条"为概数、实际是 24 条，并在 §385 文字写"23 条贡献全部归位"但表格列了 24 条。建议 Phase 5 写作时全文统一为"24 条"，避免歧义。

3. **轻微：§3.3 总体架构第 RTL 模块清单**——outline §3.3.1 列了 idma_ctrl / wdma_ctrl / odma_ctrl 但漏了 rdma_ctrl（contributions.md C3.1 含）。Phase 5 写作时建议把 rdma_ctrl 也列上（与 bias_rf / SDP 残差融合的数据通路相关）。

以上 3 条均属"写作期完善"事项，不影响大纲层判定。

---

## 评审结论

outline.md（Phase 2 v2）作为本科论文范式大纲，**结构合规、贡献全归位、prior art 三层防御到位、19 处标记位置合理、无编造数据/引用、无回归性破坏**。可进入 Phase 3。

---

## 抽样验证记录

- §3.3 5 模块流水描述 → CLAUDE.md 项目总览 + contributions.md C1.2 验证一致 ✅
- §4.3.3 S2D 公式 cin_new = stride² × Cᵢₙ → contributions.md C2.2 一致 ✅
- §5.4.1 单核综合 LUT 36,942 / Fmax 68.4 MHz → contributions.md C4.1 表格逐字一致 ✅
- §5.5.2 ResNet-11 N=4 354K cy / 564 fps → contributions.md C4.4 一致 ✅
- §5.6.1 1-DDR vs 4-DDR mac_pipe% 36.2% → 63.5% → contributions.md C4.5 一致 ✅
- §6.2.1 创新点措辞"据已知文献..."→ contributions.md C2.1 "诚实自评强度：偏强（写作时使用'据已知文献...'而非'first to...'）"一致 ✅
- 文献抽样 10 条（Alwani / Kang / Liu / MAERI / Eyeriss-v2 / Pixel-Shuffle / Winograd / VTA / im2col / ResNet）→ literature.md 全部存在 ✅
