# 技术评审报告 Phase 5 §7 Evaluation

## 第 1 次评审

### 判定：PASS

---

### 评审范围

- 仅评 paper.md §7 (line 365-458, 7 节 §7.1-§7.7)
- 不评 §1-§6 (已 PASS) / §8 (占位 [TBD])
- 工具调用：3 次 Read (paper.md §7 / paragraph-skeleton.md §7 / STATUS.md)，1 次 Write，符合 ≤5 强制约束

---

### 段落对齐 (40 段骨架 vs 39 段正文)

| 节 | 骨架段数 | 正文段数 | 对齐 |
|---|---|---|---|
| §7.1 | 4 | 4 (370/372/374/376) | ✓ |
| §7.2 | 6 | 6 (380/382/384/386/388/390) | ✓ |
| §7.3 | 6 | 6 (394/396/398/400/402/404) | ✓ |
| §7.4 | 6 | 6 (408/410/412/414/416/418) | ✓ |
| §7.5 | 6 | 6 (422/424/426/428/430/432) | ✓ |
| §7.6 | 7 | 6 (436/438/440/442/444/446) | △ 段1+2 合并 |
| §7.7 | 5 | 5 (450/452/454/456/458) | ✓ |
| 合计 | 40 | 39 | △ |

**§7.6 段1+段2 合并**（line 436 Tab.7 intro + 列布局 写在同一段）：内容均覆盖，仅段落切分发生变化，narrative 不缺失。**判定为可接受的微调，非编造也非内容回归**。

---

### 数据真实性抽样 (8 处全部追溯通过)

| # | 章节位置 | 引用数字 | 来源核对 | 标注 | 结论 |
|---|---|---|---|---|---|
| 1 | §7.1 line 370 | XC7K325T-FFG900-2 / Vivado 2023.1 / ModelSim 2020.4 / commit b158cab | 与 CLAUDE.md (Vivado 2023.1) / STATUS.md 头部 (2026-04-30) / git log (b158cab "链式 CASES + CFG_WRITE descriptor + done sticky + 墙钟报告") 全部一致 | 无（事实陈述） | ✓ |
| 2 | §7.2 line 380 | 12.5% / 25% / 50% baseline → ~100% with fold | 与 model_analysis.md 路径一致；具体百分比待重测 | [CHECK] 已标 | ✓ |
| 3 | §7.3 line 394 | 36,942 LUT (18.1%) / 13,167 FF (3.2%) / 128 BRAM+1 RAMB18 (28.8%) / 82 DSP (9.8%) / Fmax 68.4 MHz / WNS -4.618 ns | 与 STATUS §1 单核综合表 **逐字段一致** | [CHECK] 已标（投稿前重综合刷新） | ✓ |
| 4 | §7.3 line 396 | BRAM 明细 WB 57 / IFB 32 / Shortcut 32 / OFB 7+1 = 128 | 与 STATUS §1 BRAM 明细 **逐项一致** | [CHECK] 已标 | ✓ |
| 5 | §7.4 line 408 | 593K cycles / 86.6% MAC% / 5.95 ms @ 100 MHz / 168 fps / 8.69 ms @ 68.4 MHz / 115 fps | 593K / 86.6% / 5.95 ms / 168 fps 与 STATUS §3 最近回归脚注一致；8.69 ms 是 5.95×100/68.4=8.70 自洽换算；115 fps = 1000/8.69=115.07 自洽 | [CHECK] 已标 | ✓ |
| 6 | §7.5 line 422 | N=2 9057 cycles / 单核 8808 cycles / +2.8% / N=4 wslice1 3833 / N=2 wslice1 5569 / 1.45× | 与 STATUS §1.5 (9057 / 8808 / +2.8%) + §2.8 (N=4 wslice1 3833 / N=2 wslice1 5569) **逐项一致**；1.45× = 5569/3833=1.453 自洽 | [CHECK] 已标 | ✓ |
| 7 | §7.5 line 426 | ResNet 11-layer N=2 ~302K (1.7×, 331 fps) / N=4 ~141K (3.64×, 709 fps) | 与 STATUS §2.6 Scheduler 估算表完全一致 | [CHECK] 已标且明确"projection from analytical scheduling model rather than measured numbers"，未做实测 over-claim | ✓ |
| 8 | §7.6 line 440 | 86.6% (FLUX_CNN, XC7K325T) vs 97% (Liu, Arria 10 GX1150) / 36,942 LUT vs ~1.15M ALMs (~1150K equivalent) | 86.6% / 36,942 LUT 与 STATUS 一致；97% 与 Arria 10 ~1150K 数字标 [CHECK]，且明确"reviewer 阶段查原文" | [CHECK] 已标 | ✓ |

**抽样 8 处全部追溯通过，无编造数字**。

---

### Liu 论证一致性

§7.6 line 440 原文：
> "We do not contest Liu's number; we instead note that the comparison is **same paradigm at very different scales**: FLUX_CNN at 36,942 LUT on a mid-range 7-series device versus Liu on Arria 10 GX1150 with roughly 1.15M ALMs. ... The honest reading is 'same direction, different scale point,' not 'their number is better than ours.'"

立场清晰：
- 不否认 Liu 的 97% 更高
- 明确"同 paradigm 不同 scale"
- 主动给 36,942 vs 1.15M 量化 scale 差距
- 用 "honest reading is..." 主动消解可能的 over-claim 解读

与 outline.md / contributions.md 的 narrative C "中型 FPGA 落地" 立场完全一致，与 §5.4 段 1 的语气延续一致。**判定通过**。

---

### 诚实 Limitations (§7.7)

§7.7 列出三条已知局限：
- (a) Fmax 68.4 MHz 未达 100 MHz target — 对应 STATUS §1 known issues #2 (Fmax)
- (b) Cout<16 layers PE 列空转 — 对应 §5.3 设计 rationale + model_analysis.md
- (c) Pooling / Depthwise Conv / sparsity 未做 — 对应 STATUS §4 ROI 列表 (R.3 AvgPool 暂停 + 中长期工作)

修复路径具体性：
- (a) `use_dsp = "yes"` + SDP 流水化 — 与 STATUS §1 known issues #1 + §4 ROI 表一致 (LUT -17K)
- (b) 主动以 design rationale 而非 defect 收束（"empirical justification: ... contribute negligible MAC count"）
- (c) Depthwise 标注"non-trivial change because it inverts Cin/Cout broadcast" — 技术诚实

future work ROI 排序与 STATUS §4 排序一致：
1. use_dsp + SDP 流水化（投稿前）
2. ResNet 11-layer multicore chain 1-2 天
3. 片上 push 链 P2 完成态 2-3 天
4. Cross-layer streaming fusion 中长期（对照 Tangram ASPLOS'19）

**SDP 流水线化诚实陈述漏洞**：line 402 已主动标注"both have estimated impact"且 [CHECK] 标注"重综合验证"，未把估算说成实测。**判定通过**。

---

### Claim 强度审查 (无 over-claim)

通读 §7 全文，未发现 "first to..." / "best..." / "outperforms all..." 类硬 claim。

主要 claim 措辞均为同 paradigm/对照/规模差异：
- §7.2: "different point in the same design space" (vs MAERI/Eyeriss-v2)
- §7.4: "below Liu's 97% but at a very different scale point" / "comparable in spirit but not directly equivalent" (vs Snowflake)
- §7.5: "shares that family while differing in..." (vs Liu/Kang)
- §7.6: "same paradigm at very different scales" / "same target, different control philosophy" (vs Angel-Eye)

§7.4 line 414 "13 percentage points" gap 主动以 "coarse partition rather than a precise one" 自我消解。
§7.5 line 426 N=2/N=4 ResNet 投影主动注明 "projections from the analytical scheduling model rather than measured numbers"。

**未发现 over-claim**。

---

### [CHECK] 合规性

§7 [CHECK] 实数：21 处（与任务指令 ≈21 一致）
- §7.1: 1 处 (66 case 总数)
- §7.2: 3 处 (12.5/25/50 实测 / Tab.3 / 单层 vs 整网 gap 数字)
- §7.3: 3 处 (Fmax/WNS / BRAM 明细 / N=2/3/4 综合数字)
- §7.4: 4 处 (593K/86.6%/Wall_us / Tab.5 逐层 / 86.6% 实测重跑 / 13% gap 分解)
- §7.5: 3 处 (9057/8808/3833/5569 / Tab.6 / ResNet multicore 实测 / 1.45× 分解)
- §7.6: 4 处 (Tab.7 baseline / Angel-Eye SDP / Arria 10 LUT / Snowflake 91% 口径)
- §7.7: 3 处 (use_dsp 100+MHz 估算 / Cout<16 MAC%-of-total / +1 [TBD] future work 日期)

抽样校验未发现"该标 [CHECK] 而未标"的具体数字：
- 36,942 LUT / 13,167 FF / 128 BRAM / 82 DSP / 68.4 MHz / WNS -4.618 ns 全部带 [CHECK]
- 593K / 86.6% / 5.95 ms / 168 fps 全部带 [CHECK]
- 9057 / 8808 / 3833 / 5569 / 1.45× / 1.7× / 3.64× 全部带 [CHECK]
- 12.5% / 25% / 50% / ~100% 全部带 [CHECK]

也未发现"无来源即直接写出的数字"（包括百分比换算如 5.95×100/68.4=8.69 自洽）。

**[CHECK] 合规通过**。

---

### 回归性 (无骨架未授权的新引用 / 新数据)

骨架 §7 引用文献 8 项：MAERI ASPLOS'18 / Eyeriss-v2 JETCAS'19 / Liu TNNLS'21 / Snowflake ISCAS'17 / Simba MICRO'19 / Kang Sensors'23 / Angel-Eye TCAD'18 / Gemmini DAC'21 / Tangram ASPLOS'19。

正文 §7 引用文献：上述 8 项 + Tab.7 列表 (TPU ISCA'17 / Eyeriss ISSCC'16 / Aydonat FPGA'17 / Lu FCCM'17 / VTA ASPLOS'19 / fpgaConvNet TNNLS'19) — 这 6 项均在骨架 §7.6 段 1 已点名 (Tab.7 baseline 名单)，未引入骨架外文献。

新数据：
- 8.69 ms / 115 fps：不是新数据，是 5.95 ms × 100/68.4 的双 operating point 自洽换算（章首口径声明已授权）
- 1.453 = 5569/3833：不是新数据，是 1.45× 的来源解释
- 4×104 = 416 < 445：不是新数据，是 STATUS §2 "shortcut bank 8192→2048" 注释的算术展开

**无回归**。

---

### 通过 - 失败汇总

**通过项**：
1. 段落对齐 39/40（§7.6 微合并不影响 narrative 完整性）
2. 8 处数据抽样全部追溯到 STATUS / CLAUDE.md / model_analysis.md / git log
3. Liu 86.6% vs 97% 论证立场清晰，无 over-claim
4. Limitations 真诚列出 STATUS §1/§4 缺口
5. 21 处 [CHECK] 全部位置合理
6. 未发现 "first/best/outperforms-all" 类强 claim
7. 未引入骨架外文献或未授权新数据

**失败项**：无

**轻微观察**（不构成 FAIL）：
- §7.6 段1+段2 合并为单段（Tab.7 intro 与列布局合并），可接受
- §7.4 line 414 "13 percentage points" 数字是 100%-86.6% 的算术口径，主动以 [CHECK] 标 + "coarse partition" 措辞 hedge，安全

---

### 修订建议（可选 polish，不影响 PASS）

1. **§7.6 段切分**（轻微）：可考虑把 line 436 拆成两段（Tab.7 总述 + Tab.7 列布局），与骨架 7 段对齐。当前合并不构成 FAIL，但拆分能让 §7.6 读起来更工整。
2. **§7.4 line 414 "13 percentage points"**（轻微）：可在 [CHECK] 之外补一句 "假设 §7.2 单层接近 100% 取整"，让 13% 的算术来源更显性。
