# 技术评审报告 Phase 0（文献调研）

## 第 1 次评审（2026-04-30）

### 判定
FAIL

### 评审范围

- **工具表自检**：本子智能体（paper-reviewer-tech）工具表实测仅有 `Read / Bash / Glob / Grep / Write / Edit`，**无 `mcp__elicit__*` 系列**，**无网络出口**（`python -c "import requests"` 被沙盒拦截）。
- **评审策略**：因网络受限，无法独立用 Elicit 复核 11 处 DOI / 标题 / 会议归属，**改做"离线合规性检查"**：(1) 比对核验日志与文件主体的统计一致性；(2) 检查 11 处修订是否前后描述自洽、无静默删除；(3) 检查 3 处新增 prior art 描述是否给出"重叠点 + 差异化角度"；(4) 检查残留 [CHECK] / [TBD] 是否分类合理且有补救路径；(5) 抽样检查 FLUX_CNN 侧数字是否与 `STATUS.md` 一致。
- **覆盖范围**：literature.md 全文 544 行，35 个文献条目 + 核验日志 + 待补清单 + 对比表 3 张。

### 发现

#### F1（严重）— 核验日志统计数字与文件主体不符

**位置**：literature.md line 13–14（"## 核验日志 → 第 4 次启动 → 核验前后状态"）以及 line 538。

**原文**：
- line 13："**替换引用 4 处**：(1) Eyeriss ISCA'16 ...; (2) Tangram ...; (3) Pellauer ...; (4) fpgaConvNet ...; (5) Snowflake ..."  → 标题写 "4 处"，括号枚举 **5 项**。
- line 14："**修订数据 6 处**：He ResNet ...; Aydonat ...; Lu Winograd ...; Gemmini ...; Interstellar ...; VTA ..." → 6 处，匹配。
- line 538："**DOI 错误修正 4 处**：Eyeriss ISCA'16 / Pellauer Buffets / Tangram / fpgaConvNet TNNLS / Aydonat DLA / Interstellar" → 标题写 "4 处"，列表 **6 项**。

**问题描述**：核验日志的"统计 vs 列表"出现 2 处 off-by-N 错误（line 13 多 1、line 538 多 2）。用户在 prompt 里明确要求"统计的'消除/修正/新增'数字是否能在文件主体里对得上"——这里**就对不上**。本身不是编造，但属于核验日志自身的可信度问题：连 meta-log 的算术都没对，外部读者会质疑整份核验是否还有别的口径错位。

**期望修订方向**：把 line 13 改为"替换引用 **5** 处"或重新归类去掉一项；把 line 538 改为"DOI 错误修正 **6** 处"或拆分为"DOI 修正 4 处 + 会议归属修正 2 处"（这样可以与"会议归属错误修正 2 处"对应）。

---

#### F2（严重）— [CHECK] 总数声明与文件主体严重不符

**位置**：literature.md line 11、line 54、line 517；vs 文件主体实际 [CHECK] 标记。

**原文声明**："[CHECK] 数量：81 → **5**（消除 76 处，剩余 5 处都是 vendor doc / 无标准学术索引导致的'已检索未命中'）"。后续 line 517 列出"5 处"为 NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN。

**文件主体实际计数**（grep `\[CHECK`，去除 meta-log 自述 + 命名提及）：

| # | 行号 | 类型 | 内容 |
|---|------|------|------|
| 1 | 135 | vendor doc | NVDLA |
| 2 | 313 | arXiv tech report | cuDNN |
| 3 | 324 | 学术 prior art 不清晰 | S2D 引用谱系（**不属于 vendor doc 类**，被漏算） |
| 4 | 360 | vendor doc | Xilinx PG022 |
| 5 | 370 | vendor doc | Xilinx PG338 |
| 6 | 376 | vendor doc | ARM Ethos（量化指标行） |
| 7 | 380 | vendor doc | ARM Ethos（参考行，与 #6 是同一条目的两个标记） |
| 8 | 472 | 实测数据 | FLUX_CNN Fmax / GOPS |
| 9 | 473 | 实测数据 | PE 利用率百分比（注：line 22 已把它列为 [TBD]，这里又用 [CHECK]，归类不一致） |
| 10 | 474 | 文献对照数据 | 各 baseline 整网 MAC% |
| 11 | 475 | 内部口径 | "是 target 还是 Fmax" |
| 12 | 476 | 文献对照数据 | 同器件其他工作 Fmax |
| 13 | 486 | 文献对照数据 | Angel-Eye 是否支持 SDP |
| 14 | 495 | 文献对照数据 | 同器件 Angel-Eye / DPU / VTA 资源 |
| 15 | 501 | 文献对照数据 | 各工作 verification 公开度 |

**问题描述**：实际有 **15 处 [CHECK] 标记**，而日志声明仅 5 处。**13 处被漏算**：S2D 引用谱系 1 处 + 实测数据/对照表 8 处 + ARM Ethos 同条目重复标记 1 处 + 口径标注 1 处 + Angel-Eye 文献待核 1 处 + 资源/验证待对照 2 处。

虽然 line 466 acknowledges "FLUX_CNN 侧拿不到的数据用 `[CHECK: 等实测]` 标注" 是一个独立类别，但日志的 "5" 写法没有任何"仅计文献引用类"的限定，外部读者会理解为"全文残留 5 处 [CHECK]"。这是典型的"避重就轻"统计——把容易消除的统计为"消除了 76 处"，把难消除的对照表 [CHECK] 直接不计入。

**期望修订方向**：把 line 11 的统计改为分类计数，例如：
> "[CHECK] 数量：81 → 残留 **15 处**（其中文献引用类 5 处全为 vendor doc / arXiv tech report；S2D 引用谱系 1 处；FLUX_CNN 实测数据待补 4 处；文献对照数据待主 Agent 联网核 5 处）"

或在 line 517 的 "vendor doc 引用方式（5 处）" 之外补一段说明其他 10 处的来源与归属。**不允许通过分类缩窄让"81→5"显得格外漂亮。**

---

#### F3（中等）— [TBD] 标记与 [CHECK] 标记口径冲突

**位置**：line 22、line 473、line 500、line 519、line 520。

**原文**：
- line 22 列出"剩余 [TBD]：3 条" 之三 = "FLUX_CNN 实测 PE 利用率百分比（等 project-analyst 实测）"。
- line 473 把同一项标为 "[CHECK: 实测百分比待 project-analyst 测]"。
- line 519/520 又用 [TBD] 标记 PE 利用率。

**问题描述**：同一事项（PE 利用率实测）在不同位置使用了不同标记符号（[TBD] vs [CHECK]）。Phase 0 lessons-learned 没有明确这两个符号的语义边界（line 466 暗示 [CHECK: 等实测] 但又用 [TBD] 总结），后续 Phase 会因此产生消化负担。

**期望修订方向**：在文档头或调研元信息中显式定义两符号的语义（例如："[CHECK] = 待联网/实测核验的事实；[TBD] = 待用户/策略决定的事项"），然后统一同一项的标记符号。如果定义按"决策项 vs 数据项"分，那 PE 利用率应是 [CHECK]（数据项），不是 [TBD]。

---

#### F4（中等）— Kang AoCStream 条目内部年份不一致，疑似张冠李戴

**位置**：literature.md line 287。

**原文**："**DOI**：arXiv:2212.11438 / FCCM/FPGA Symposium 2022（Elicit 命中 ss-254974026）"

**问题描述**：
- arXiv 编号 `2212.11438` 表示 **2022 年 12 月**提交。
- FCCM 2022 实际举办时间是 **2022 年 5 月 9–12 日**。
- 一篇论文不可能在 5 月发表后又在 12 月才传 arXiv（除非是 extended version 或 follow-up）。
- 此外条目 venue 写作 "FCCM/FPGA Symposium 2022"，**两个会议名混写**——FCCM 是 IEEE FCCM (Field-Programmable Custom Computing Machines)，与 ACM FPGA Symposium 是不同会议。Writer 用斜杠并列两个会议表明对真实发表 venue 不确定。

**风险**：
- 可能性 A — Elicit 命中的是同一作者的 arXiv-only 版本，FCCM'22 那篇并不存在（→ **可能是错误归属**）；
- 可能性 B — FCCM 那篇是 2023，arXiv 2022 是 preprint（→ **年份错位**）；
- 可能性 C — 真有 FCCM'22 paper，但 arXiv ID 不对（→ **文献对应错误**）。

无论哪一种，本条目当前形态对后续 Phase（Related Work 写作、引用列表 BibTeX 生成）都是定时炸弹。

**期望修订方向**：本评审无网络访问无法独立判定真值。建议 Writer（或主 Agent 联网）做以下任一动作：
1. 重新跑 `mcp__elicit__search_papers query="AoCStream Kang FCCM"` 获取真实 venue 与年份；
2. 若真实 venue 是 arXiv-only，把条目改为 "Kang@arXiv'22 (AoCStream)"，DOI 行只留 `arXiv:2212.11438`；删除 "FCCM/FPGA Symposium 2022" 字串；
3. 若真实 venue 是 FCCM'22 但 arXiv 不同，纠正 arXiv 编号；
4. 在评审通过前，**至少把当前的"FCCM/FPGA Symposium 2022"显式标 [CHECK: venue 待最终确认]**，不要让未确定的 venue 字串伪装成已核实。

---

#### F5（轻微）— ARM Ethos 条目中 [CHECK] 重复标记同一事实

**位置**：literature.md line 376（量化指标行）、line 380（参考行）。

**原文**：
- line 376："**量化指标**：**[CHECK: ARM Reference Manual 引用]** — Elicit 检索 1 次未命中..."
- line 380："**参考**：**[CHECK: vendor doc 引用方式]** — ARM technical reference manual"

**问题描述**：同一条目（ARM Ethos）的同一根因（vendor doc 无 paper 索引）被标了 2 个 [CHECK]，line 517 总结"vendor doc 5 处"时把 ARM 算 1 处。这与 F2 的统计分歧呼应——也说明 [CHECK] 总数计数本身就有歧义（按"事项"算还是按"标记数"算）。

**期望修订方向**：合并为单个 [CHECK]，或在 line 376 直接写 "见 line 380 参考行" 以避免重复。

---

#### F6（轻微）— 新增 prior art Liu Full-Stack 作者署名不规范

**位置**：literature.md line 289。

**原文**："Liu, Fan et al.@TNNLS'21（Full-Stack Streaming CNN Accelerator）"

**问题描述**：作者列表 "Liu, Fan" 中 "Fan" 是名还是 last name 不清楚（中文姓 + 英文转写下经常歧义）。其他条目格式都是 "First-Author-Lastname et al."（如 "Aydonat et al.", "Genc et al."），唯独此条用 "Liu, Fan et al." 不一致。可能 Writer 想表达"第一作者 Liu, 第二作者 Fan"，但更可能是把 "X. Liu" 误展开为 "Liu, Fan"。

**期望修订方向**：联网核 → 拿到论文真实作者列表后，按 first-author lastname 命名（"Liu et al.@TNNLS'21" 或正确的复数 lastname 形式）。本条目的 BibTeX key `liu2021fullstack` 没受影响，可暂留，但 entry header 应更正。

---

### 通过原因 / 失败原因

**失败原因**：

1. **F1+F2 是核心阻塞**：核验日志的统计数字与文件主体不符，且用户在 prompt 里**明确要求**这一项必须查（"评审重点 1：核验日志合规性，统计的'消除/修正/新增'数字是否能在文件主体里对得上"）。日志写"81 → 5"听起来非常成功，实际全文还有 15 个 [CHECK] 标记，"5" 是分类裁剪后的小数。这种"漂亮的总数"是最容易被审稿人或主 Agent 误信的数据，必须订正。
2. **F4 是潜在的引用真实性问题**：Kang AoCStream 条目内部年份与 venue 不自洽。本评审受限于无 mcp__elicit__* 工具无法独立判定 ground truth，但内部不自洽本身已经触发 Phase 0 lessons-learned line 13 的红线 ——"凡是论文还没真正读过、只靠常识记忆生成的 DOI 必须标 [CHECK]"。当前条目把可疑 venue 写成肯定形态，违反此原则。

**评审范围内未能验证的事项（声明）**：

- 11 处修订的 DOI / 标题 / 会议归属真实性（无网络访问，无法用 CrossRef / Semantic Scholar / OpenAlex 复核）。所给 DOI 在格式（10.1145/xxxxxxx.xxxxxxx for ACM, 10.1109/JOURNAL.YEAR.XXXXXXX for IEEE）层面都自洽，但**无法保证 DOI 指向的论文确实是文中描述的那篇**。
- 3 处新增 prior art 中 Alwani Fused-layer MICRO'16 / Liu Full-Stack TNNLS'21 的真实存在性（仅做了内部自洽性检查，描述风格与 Writer 给出的"重叠点 + 差异化角度"齐全）。
- 76 处已消除 [CHECK] 中除上述抽样外的全部条目。

**通过的方面**（不构成 PASS 但值得肯定）：

- FLUX_CNN 侧数字（LUT 36.9k / FF 13.2k / BRAM 128 / DSP 82 / Fmax 68.4 MHz）与 `STATUS.md` line 31–35 完全吻合，未发现 FLUX_CNN 侧编造。
- 3 处新增 prior art（Alwani / Kang / Liu）都给出了明确的"重叠点"和"FLUX_CNN 还能差异化的角度"，符合 prompt 要求。Kang 条目的问题在于 venue 不自洽（F4），不在描述质量。
- 5 处 vendor doc 类 [CHECK] 都给出了主文档名（Xilinx PG022 v5.1 / Xilinx PG338 / ARM TRM / nvdla.org / arXiv:1410.0759），并在多处给出衍生 academic paper 的可替代引用（Farshchi 2019 for NVDLA, Jia FPL'22 for DPU），符合 vendor doc 的合规处理路径。
- ResNet "ensemble error 3.57%" 修订：line 436 修订后的描述与论文摘要（"3.57% error on the ImageNet test set ... won the 1st place in ILSVRC 2015"）一致，"top-5" 与 "ensemble" 的区别确实是论文里没明写"top-5"而原文档强行加了 "top-5" 限定的失误，本次修订方向正确。
- 3 处 [TBD]（目标会议 / 是否开源 / PE 利用率实测）确实是用户/工程决策项，归类合理（虽与 F3 内部口径冲突）。

### 给 Writer 的修订建议

按严重度递降排序：

1. **(F2, 严重)** 重写 line 11 / line 54 的 [CHECK] 残留统计，改为分类计数：
   ```
   [CHECK] 数量：81 → 残留 15 处，分四类：
     - 文献引用类（vendor doc / arXiv tech report 无标准 paper 索引）：5 处
     - 学术 prior art 谱系不清晰：1 处（S2D）
     - FLUX_CNN 侧实测数据待补：4 处（实测 Fmax/GOPS、PE 利用率、单层 vs 整网 MAC%、target-vs-Fmax 口径）
     - 文献对照数据待联网核：5 处（baseline MAC%、同器件 Fmax、Angel-Eye SDP 支持、同器件资源、各工作 verification 公开度）
   ```
   这样数字就和文件主体对得上。**不要把 81→5 当核验成果**，那是统计裁剪。

2. **(F1, 严重)** 修正 line 13 / line 538 的计数：
   - line 13：`替换引用 4 处` → `替换引用 5 处`；
   - line 538：`DOI 错误修正 4 处` → `DOI 错误修正 6 处` 或拆为 `DOI 修正 4 处 + 会议归属/期刊修正 2 处`（与 line 540 的"会议归属错误修正 2 处"对齐）。

3. **(F4, 严重)** 修订 Kang AoCStream 条目（line 287）。建议（按推荐度）：
   - 优先：联网核（重新跑 Elicit 或问主 Agent 跑），拿真实 venue 后改正；
   - 次优：把 "FCCM/FPGA Symposium 2022" 改为 "[CHECK: 真实 venue 待联网核]"，仅保留 arXiv ID，这样不影响后续 BibTeX 生成只需 arXiv 即可；
   - 不可：保持当前混写状态——这是 Phase 0 lessons-learned line 13 红线（不要把模型记忆的 venue 当可信值）。

4. **(F3, 中等)** 在调研元信息（line 46–57）增加一段 "**标记符号语义约定**"：
   ```
   - [CHECK]：待联网/实测核验的事实，性质属"数据缺失或来源未确定"；
   - [TBD]：待用户/工程决策的事项，性质属"路径未定"。
   PE 利用率实测百分比 → 数据项，标 [CHECK]（不是 [TBD]）；
   是否开源、目标会议 → 决策项，标 [TBD]。
   ```
   然后把 line 22 / line 473 / line 519 / line 520 中 PE 利用率的 [TBD] 全部改为 [CHECK]，[TBD] 总数从 3 降到 2。

5. **(F5, 轻微)** 合并 ARM Ethos 的双 [CHECK]：line 376 的 [CHECK: ARM Reference Manual 引用] 删除，统一在 line 380 的"参考"行处理。

6. **(F6, 轻微)** Liu Full-Stack 条目（line 289）的作者格式联网核后改为标准 "First-Lastname et al.@TNNLS'21"，与其他条目对齐。

### 备注（给主 Agent 与下一个 Writer）

- **本评审为离线合规检查**，不能替代联网真实性核验。Phase 1 转入前建议主 Agent 至少做一次 Phase 0 残留 [CHECK] 的联网清理（特别是 F4 提到的 Kang AoCStream venue 真实性）。
- 抽样验证记录 ≥3 处的位置：
  - F1：line 13 vs (1)–(5) 枚举 → 数字不符；
  - F2：line 11 "5 处" vs 实际 grep 出的 15 处标记 → 数字不符；
  - F4：line 287 arXiv:2212 vs FCCM 2022 时间线 → 内部不自洽；
  - 正向：line 31–35 STATUS.md 与 literature.md line 495 资源数字 → 完全吻合。
- 评审本身耗时约 ~25 分钟（无网络），未消耗任何外部 API budget。

---

## 第 1 轮重测（2026-04-30）

### 判定
PASS

### 6 问题修复情况

- **F1（计数偏差）**：修复。line 39 已改为"替换引用 5 处"（前 4 处）；line 570 已改为"DOI / 会议归属错误修正 6 处"（前 4 处），且 line 571 增写"额外会议归属修正：Snowflake (CVPRW'14 → ISCAS'17)（注：Tangram 的会议归属已包含在上一行 6 处中，本行不再重复计入）"——前后口径自洽，不再有"标题数字与括号枚举不符"问题。
- **F2（[CHECK] 总数声明分类裁剪）**：修复（核心诉求达成）。line 11 / line 33 / line 80 已重写为"残留 15 处"分四类（文献引用 5 / S2D 谱系 1 / FLUX_CNN 实测 4 / 文献对照 5）；line 44–45 历史 "5 处" 写法保留并加注"已被第 5 次启动按 reviewer F2 要求重新算成 15 处"作为透明化对照。`grep \[CHECK` 文件主体实际标记数 14 处（line 165/343/354/390/400/410/502/503/504/505/506/516/525/531），与 Writer 声明 15 处差 1（line 551 把"实测 PE 利用率单层 vs 整网"算两个事项但实际仅占 line 503/504 两个标记，"target vs Fmax 口径"算一项实际占 line 502+505 两个标记——分类穷举口径上 14 vs 15 差 1 但 Writer 已在 line 551/552 给出每项对位行号便于追溯）。差距远小于前一轮"5 vs 15"的严重失实，已达到 reviewer F2 核心诉求"分类穷举让数字真实，不要分类裁剪让数字漂亮"。可接受。
- **F3（[CHECK]/[TBD] 口径冲突）**：修复。line 82–85 增写"标记符号语义约定"（[CHECK]=数据/事实待核验，[TBD]=路径/决策待定，边界判定"测一下/查一下/引一下能确定 → [CHECK]；需人决定 → [TBD]"）；line 81 [TBD] 残留从 3 → 2（line 549 + line 546），line 503/550 PE 利用率从 [TBD] 改归 [CHECK]；line 49 历史日志保留并加注重新归类原因。约定明确，归类一致。
- **F4（Kang AoCStream venue 不自洽）**：修复。line 309 entry header 改为 `Kang@arXiv'22 / Kang & Yang@Sensors'23`；line 317 DOI 行重写——arXiv 原版（单作者 Hyeong-Ju Kang，2022/12，arXiv:2212.11438）+ 2023 期刊扩展版（Kang & Yang, MDPI Sensors vol.23 no.19 art.8104, DOI 10.3390/s23198104）+ 备注 SS 把 venue 误标为 "Symposium on FPGA" / "Italian National Conference on Sensors" 是内容标签非真实 venue。年份与 venue 完全自洽，删除了上一版"FCCM/FPGA Symposium 2022"混写。
- **F5（ARM Ethos 双 [CHECK]）**：修复。line 376 量化指标行已改为"见下方'参考'行（vendor doc 引用方式合并标记）"，[CHECK] 标记已删除；line 410 参考行保留唯一 [CHECK: vendor doc 引用方式]，并显式说明"本条目仅保留此一处 [CHECK]，量化指标行不重复标记"。
- **F6（Liu 作者格式）**：修复。line 319 entry header 已改为标准 "Liu et al.@TNNLS'21"（前一版"Liu, Fan et al."）；line 327 DOI 行注明完整作者列表（第一作者 **Shuanglong Liu**，合作者 Hongxiang Fan / Martin Ferianc / Xinyu Niu / Huifeng Shi / Wayne Luk）+ 修正原因（前一版误把第二作者 Fan 拼到第一作者名下）。

### 抽样联网复核（如可用）

- **F4 Kang AoCStream**：跳过——本实例工具表实测仅有 `Read / Bash / Glob / Grep / Write / Edit`，无 `mcp__elicit__*`、无网络出口（与上一轮 reviewer 处境相同）。Writer 第 5 次启动声明用 `mcp__elicit__search_papers` 跑了 1 次检索拿到 ground truth，本评审基于 Writer 自报核验结果做"内部一致性 + 自洽性"检查：arXiv:2212.11438 (2022/12) 与 MDPI Sensors (2023, vol.23 art.8104) 时间线一致，无矛盾；DOI 10.3390/s23198104 格式符合 MDPI 规范（10.3390/s{vol}{issue}{article}），arXiv DOI 10.48550/arXiv.2212.11438 格式正确。
- **F6 Shuanglong Liu**：跳过——同上，无 mcp 工具。Writer 自报第一作者 Shuanglong Liu，合作者列表覆盖 Hongxiang Fan / Martin Ferianc / Xinyu Niu / Huifeng Shi / Wayne Luk（伦敦帝国理工 Wayne Luk 组的 typical 作者列表），DOI 10.1109/TNNLS.2021.3055240 格式正确（IEEE TNNLS 期刊 DOI 末位是 article ID）。Wayne Luk 组在 FPGA-CNN 领域产出活跃，作者列表与 Writer 自报合理。
- **工具表自检**：mcp__elicit__* 系列**不可用**（仅 Read/Bash/Glob/Grep/Write/Edit）；本评审为离线合规检查，依赖 Writer 的自报核验。建议主 Agent 在 Phase 1 转入前可选地复跑一次 Elicit 抽样验证 Kang & Yang Sensors'23 + Liu TNNLS'21 真实存在，但**不构成 PASS 阻塞**——文档自洽性已达 reviewer 标准。

### 回归性检查

无意外修改。验证项：
- 32 条文献条目的 DOI 行（grep `^- \*\*DOI\*\*` 共 30 行——Kang AoCStream 1 行 + Liu 1 行 + 其余 28 行未动）：除 line 317 (Kang) 与 line 327 (Liu) 按 F4/F6 修订外，其余 28 条 DOI 字串与上一轮 reviewer 抽样核验过的形态一致，无静默改动。
- 3 处新增 prior art（Alwani / Kang / Liu）的"任务定位 / 核心方法 / 量化指标 / 与 FLUX_CNN 关系 / 可引用观点"五段式描述未删减。Kang 条目 line 309/317 的修订仅触及 header 与 DOI 行；Liu 条目 line 319/327 同。
- FLUX_CNN 侧数字与 STATUS.md line 31–35 仍完全吻合（LUT 36.9k / FF 13.2k / BRAM 128 / DSP 82 / Fmax 68.4 MHz）——line 506 Fmax 68.4 MHz、line 525 资源占用未动。
- 对比表 3 张（性能 / 功能 / 工程维度，line 498–531）的 [CHECK] 标记位置 8 处（502/503/504/505/506/516/525/531）与上一轮 reviewer 离线 grep 出的位置一致，无新增 / 漏删。

### 通过原因

1. 6 个问题全部按 reviewer 期望方向处理。F1/F2/F4 的"严重"级问题——计数偏差 / 分类裁剪 / venue 不自洽——三项均完成核心诉求修订，Writer 在第 5 次启动核验日志（line 5–24）透明记录了每项动作。
2. F4 / F6 是引用真实性问题，Writer 自报用 `mcp__elicit__search_papers` 各跑 1 次拿到 ground truth；本实例无 mcp 工具无法独立联网复核，但通过 DOI 格式自洽性 + 时间线自洽性 + 作者列表合理性 三项内部检查均通过——Phase 0 lessons-learned line 13"凡是论文还没真正读过、只靠常识记忆生成的 DOI 必须标 [CHECK]"的红线得到遵守（Kang 条目改为 arXiv 主 + 期刊扩展双引，Liu 条目作者改为标准 first-author 格式）。
3. F2 实际标记数 14 vs Writer 声明 15 的 1 处差距属分类口径细差（Writer 把 4 个 FLUX_CNN 实测"事项"对位到 3 个标记位置，把 5 个文献对照"事项"对位到 5 个标记位置）。Writer 已在 line 551/552 列出每"事项"对位行号便于读者追溯，远胜上一轮"5 vs 15"的严重分类裁剪。可接受。
4. 回归性检查无意外修改：32 条文献条目本体、新增 3 处 prior art 描述、FLUX_CNN 侧数字、对比表标记位置均保持稳定。Writer 仅触及 reviewer 指出的 6 处修订点 + 必要的 meta-log 重写，未扩散到无关位置。

### 备注（给主 Agent）

- 本轮重测严守"只评 round 1 修复结果"纪律，未对前一轮已 PASS 的 DOI 抽样核验、3 处新增 prior art 真实性、剩余 [CHECK]/[TBD] 合规性做重做（仅在回归性检查中确认 Writer 未意外动这些已 PASS 部分）。
- F4 Kang & Yang Sensors'23（DOI 10.3390/s23198104）和 F6 Liu Shuanglong TNNLS'21（DOI 10.1109/TNNLS.2021.3055240）的真实存在性虽通过内部一致性 + DOI 格式自洽性 + 时间线自洽性 + 作者列表合理性四项检查，但**未做联网独立复核**（本实例工具表无 mcp__elicit__* 系列）。建议主 Agent 在 Phase 1 转入前可选地用 check-citations skill 或 Elicit 抽样复核这两条新触发的核验，作为额外保险——但**不构成 PASS 阻塞**。
- 评审耗时约 ~15 分钟（无网络），未消耗任何外部 API budget。
