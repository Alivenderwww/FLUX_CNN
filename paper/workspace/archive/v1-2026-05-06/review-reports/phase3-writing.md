# 写作评审报告 Phase 3

## 第 1 次评审

### 判定
PASS

### 评审范围

- 主产出：`paper/workspace/section-summary.md`（655 行 / 38KB / §1-§8 共 36 节）
- 上一阶段参考：`paper/workspace/outline.md`
- 历史经验：`paper/workspace/lessons-learned.md`
- 评审角度：写作质量（结构 / 衔接 / 风格 / 术语 / 篇幅），不审事实正确性、不审新颖性

### 抽样 8 节核心信息清晰度

抽 §1.1 / §1.4 / §2.3 / §3.4 / §4.3 / §5.2 / §6.4 / §7.6 各节的"核心命题"段。

| 节 | "核心命题"一句话能否传递主信息 | 说明 |
|---|---|---|
| §1.1 | ✅ | "12.5%-50% 利用率 + VGA 4.9MB vs 端侧 SRAM" 双约束清晰，紧扣 motivation |
| §1.4 | ✅ | 列 5 条贡献 + quantified results 直接落到位，"with documented optimization roadmap" 防御性表述明确 |
| §2.3 | ✅ | "445 BRAM ≈ 1.6 MB vs 4.9 MB 单图 → row-ring 仅需 ~10 KB" 数字对照法表达力强 |
| §3.4 | ✅ | 三处 prior art (Alwani / Kang / Liu) 各自 trade-off 一句话精准点出，差异化清晰 |
| §4.3 | ✅ | "rows_available ≥ yout·stride+Ky 触发 + 三阶段并发 + ring ~10 KB" 机制+证据双线 |
| §5.2 | ✅ | S2D 数学 + 与 Ky-fold 关键差异（不复制） + Sub-Pixel 算法侧出处 + compile-pass 新意点 |
| §6.4 | ✅ | W slice + halo 几何用 N=2 K=3 W=32 具体例子 + 跨核 SRAM 直送 AXI 路径，工程感足 |
| §7.6 | ✅ | "86.6% vs Liu 97% = same paradigm at different scale" 防御口径明确 |

**结论**：8 节抽样无水节，核心命题段都做到了"一句话说清这节传递什么"。

### 节内三要素完整性

抽查 36 节是否都包含：核心命题 / 关键内容 / 承上启下 / 依赖。

- ✅ **§1-§8 全部 36 节**结构齐整，每节都有"核心命题 / 关键内容 / 承上启下 / 依赖"四段式格式
- ✅ §3.6 / §5.4 / §7.7 / §8.1 等收束节也保留了"承上启下"段（非首节亦非末节才能承接）
- ✅ §1.5 / §2.4 / §3.6 / §4.5 / §5.4 / §6.5 / §7.7 章末节都明确写了"为下一章铺垫"，§8.1 写"全文收束 / 不引出后续章节"亦明确
- 轻微：§4.5 / §5.3 等节的"关键内容"段较长（~150 字），与 §4.4 / §6.5 等较短节（~80-100 字）形成轻微不平衡，但仍在 100-200 字框架内，可接受

**结论**：三要素完整，无格式缺陷。

### 跨章衔接

| 接缝 | 评估 | 说明 |
|---|---|---|
| §1.5 → §2.1 | ✅ | §1.5 "引出 §2 Background"，§2.1 "作为 Background 首节给术语" 自然 |
| §2.4 → §3.1 | ✅ | §2.4 把 motivation 转 design goals，"自然过渡到 §3 Related Work 对每条 goal 的 prior art 对位"，§3.1 直接给硬件谱系坐标 |
| §3.6 → §4.1 | ✅ | §3.6 "防御总结表 → §4 Architecture 详细架构论证"，§4.1 "承 §3.6 防御总结收束"显式回应 |
| §5.4 → §6.1 | ✅ | §5.4 "narrative A 主章收束 → §6 把 narrative A 论据落到完整系统"，§6.1 "承 §5 narrative A 主章收束"对位 |
| §6.5 → §7.1 | ✅ | §6.5 "本章给系统能跑，下一章给系统跑得多好"，§7.1 "承 §6 系统能跑" 这是最自然的一处衔接，写得很 punchy |
| §7.7 → §8.1 | ✅ | §7.7 "缺口 + 修复路径 → §8 Conclusion 高 level 总结"，§8.1 "承 §7.7 已知局限" 对位 |

**结论**：6 个跨章接缝全部自然，无突兀话题切换。

### 风格一致性（§1-§3 vs §4-§8）

> 重点：§1-§3 由第一个 Writer 写，§4-§8 由第二个 Writer 写。两段是否风格分裂是本次评审的关键。

#### 用词风格
- ✅ 两段都使用"narrative A 主轴 / narrative B 系统侧 / system 完整性论据"等同一套元语言
- ✅ 学术化程度一致（"工程谱系 / 同语言对照 / 反例对照 / 形式化对位"等表达两段都用）
- ✅ 句长分布相近（短句 + 数字注解为主，~25-35 字平均）
- 轻微：§1-§3 偏好用"——"做主从句切分（如 §3.4 "三处 streaming 近邻 prior art 在此精细差异化是 narrative B 防御主战场"），§4-§8 偶尔用"；"分号衔接稍多（如 §6.4 "用一段话给 multicore wrapper + W slice 几何 + halo 机制；用一段话给跨核 SRAM 直送的 AXI crossbar 路由；引 N=4 wslice1 1.45× speedup..."），但这是文体差异不是分裂

#### 摘要结构格式
- ✅ 36 节统一采用"核心命题 / 关键内容 / 承上启下 / 依赖"四段式，无任一节走自由形式
- ✅ "依赖"段下挂"贡献 / 文献 / 图表占位 / 不确定项"四子项的格式两段一致
- ✅ §5 / §7 章首段都加了"口径声明"灰底引文（"> **章首段必述**：..."），格式手法一致

#### 引用 contribution 编号格式
- ✅ 全文统一用 `C1.1 / C2.1 / C3.7` 格式（不是 `C-1.1` 或 `c2.1`）
- ✅ 文献条目格式统一：`MAERI ASPLOS'18 / Eyeriss ISCA'16/JSSC'17 / Liu TNNLS'21`，作者+venue+年份缩写一致
- ✅ DOI / arXiv 编号未直接写入 summary（合规——核验日志另行管理，避免编造）

#### 篇幅习惯
- §1.1-§1.5：每节 ~100-150 字，集中
- §2.1-§2.4：每节 ~100-130 字
- §3.1-§3.6：§3.4 偏长（~250 字，因为承担三处 prior art 防御主战场，篇幅是合理的章节内倾斜）
- §4.1-§4.5：每节 ~120-180 字
- §5.1-§5.4：每节 ~150-200 字（narrative A 主章，篇幅倾斜合理）
- §6.1-§6.5：每节 ~130-180 字（§6.5 略短 ~110 字，符合工程实践收束节定位）
- §7.1-§7.7：每节 ~130-200 字
- §8.1：~120 字（单节 conclusion 不分小节，符合定位）

✅ 两段 Writer 篇幅习惯接近，章节内的篇幅倾斜（§3.4 / §5.x / §7.6）都符合 outline 给出的预估页数权重。

**结论**：两段 Writer 风格统一度高，无可见分裂。

### narrative 主线一致性

逐节追踪 narrative A（compiler-side PE utilization）+ narrative B（row-streaming + multi-core）的连续性：

| 节 | narrative 标签 | 是否对位 |
|---|---|---|
| §1.1 motivation | A 数字 + B 数字共同导出 | ✅ |
| §1.3 our approach | A 主轴 + B 系统侧 + C 元素三层 framing | ✅ |
| §1.4 contributions | 5 条贡献按 A/B 加权排序 | ✅ |
| §3.2 hardware-reconfigurable | narrative A 反向 alternative | ✅ 标签明确 |
| §3.3 compiler co-design | narrative A 同语言谱系 | ✅ 标签明确 |
| §3.4 streaming | narrative B 主对照集 | ✅ 标签明确 |
| §4.3 row-ring | C1.2 narrative B 系统侧主推 | ✅ |
| §5 章 | narrative A 主章 | ✅ 章首口径声明 |
| §6.3 SDP | system 完整性（narrative D 元素） | ✅ 明确不主推 |
| §6.4 multi-core | C3.7 narrative B 系统侧主推 | ✅ |
| §7.2 PE util | narrative A 核心数据 | ✅ |
| §7.5 multi-core scaling | narrative B 系统侧主数据 | ✅ |
| §7.6 prior art comparison | narrative A/B 防御主战场 | ✅ |
| §8.1 conclusion | A 主轴 + B 系统侧回顾 | ✅ |

✅ 在 §6 system 章节，A/B 区分清晰：§6.1 / §6.2 / §6.5 是 system completeness（narrative D 元素），§6.3 是 system 完整性（narrative D），§6.4 才是 narrative B 系统侧主推——三类 system 元素混合 narrative 但分工清晰，未出现主线模糊。

**结论**：narrative A+B 主线全文跟得住，混合 narrative 在 §6 system 章得到清晰区分。

### 术语一致性

抽查关键术语在 §1-§3 vs §4-§8 是否统一：

| 术语 | §1-§3 用法 | §4-§8 用法 | 一致性 |
|---|---|---|---|
| PE / mac_array | "PE 利用率" / "mac_array" | "PE 利用率" / "mac_array" | ✅ |
| Ky-fold / Space-to-Depth (S2D) | "Ky-fold" / "Space-to-Depth" / "S2D" | 同 | ✅ |
| streaming row-ring | "streaming row-ring" / "row-ring" | "row-ring" / "row-level ring buffer" | ✅ 短长式都有，但语义清楚 |
| SDP | "Single Data Point processor" / "SDP" | "SDP" / "Single Data Point processor" | ✅ §6.3 第一次出现完整展开 + 缩写 |
| mm2s_arb | (§3 未提) | "mm2s_arb（IDMA/WDMA 共享 MM2S 串行仲裁）" | ✅ |
| axi_dm | "Xilinx axi_dm IP" | "Xilinx axi_dm IP" | ✅ |
| narrative A/B/C/D | A/B/C/D 全字母 | 同 | ✅ |
| 16×16 INT8 阵列 | "16×16 INT8 阵列" / "fixed 16×16 阵列" | "16×16 INT8 阵列" / "physical 16×16 阵列" | ✅ |
| per-col PARF / parf_col | "per-col PARF" | "per-col PARF" / "parf_col × 16 列独立 SRAM" | ✅ |

✅ 全文未发现 §1-§3 用 A 名字、§4-§8 用 B 名字的术语分裂。

**轻微提示**：
- "mm2s_arb" 直到 §6.2 才首次出现（§1-§3 不展开 DMA 子系统细节是合理的），首次出现处用括号给了职责说明，符合术语首现展开规范。
- "DFE"（descriptor frontend）在 §6.2 出现但未展开（"CFG_WRITE descriptor + DFE 让 host AXI-Lite 写..."），这是 summary 阶段的轻微缺陷——但 paragraph-skeleton 阶段会补，对当前 phase 不算缺陷。

**结论**：术语一致性达标。

### 避免重复检查

§538-548 跨小节一致性记录已自查 5 处重复风险（PE 利用率 motivation / streaming / vs Liu / SDP / loop nest），并都标注了"分工"——表明 Writer 主动管理重复。
- ✅ §1.1 与 §5.1 与 §7.2 三处 PE 利用率数字：分工明确（motivation 一句话 / fold 数学详述 / 三列对比表）
- ✅ §3.4 与 §7.6 两处 vs Liu 论证：分工明确（精细差异化 / 数字对位）
- ✅ §3.5 与 §6.3 两处 SDP：分工明确（framing / 详述）

**结论**：重复管理到位，无概念在多节重复定义。

### 通过原因

1. **结构整齐**：36 节统一采用"核心命题 / 关键内容 / 承上启下 / 依赖"四段式，无格式分裂
2. **核心信息清晰**：8 节抽样无水节，每节核心命题段都能传递明确主信息
3. **风格统一**：两段 Writer (§1-§3 vs §4-§8) 在用词 / 结构 / 引用格式 / 篇幅习惯上达到高度一致，未见分裂
4. **跨章衔接自然**：6 个跨章接缝全部对位（§1.5↔§2.1 / §2.4↔§3.1 / §3.6↔§4.1 / §5.4↔§6.1 / §6.5↔§7.1 / §7.7↔§8.1），无突兀话题切换
5. **narrative 主线连贯**：A+B 混合 narrative 在 §6 system 章得到清晰区分，§5 / §7 章首口径声明承担 narrative 锚点
6. **术语一致**：关键术语（PE / Ky-fold / S2D / row-ring / SDP / axi_dm 等）全文统一，首次出现处展开 + 后续缩写
7. **重复管理**：§538-548 跨节一致性记录已自查 5 处重复风险并明确分工
8. **覆盖完整**：17 条 contribution 全部归位（§552-571 校验日志），35 篇文献全覆盖

### 给 Writer 的轻微建议（不影响 PASS，可在 Phase 4 段落骨架阶段顺便处理）

1. **§4.4** "[CHECK: per-col PARF 的 BRAM 数量贡献单独估算需查综合报告 cell utilization]" 嵌在"关键内容"段的句末，紧跟在"仅作单元级机制记录"之后稍显突兀；建议 Phase 4 起草段落骨架时把 [CHECK] 移到"不确定项"列中（§4.4 节其实已经在"不确定项"重复列了，可以删去内嵌的那一处）。
2. **§6.1** 末尾的 "[CHECK: 5.95 ms target vs 68.4 MHz Fmax 实际 fps 应为 168×0.684 ≈ 115 fps，论文须统一两口径]" 含具体数字推算，作为 summary 阶段的 [CHECK] 描述偏长；建议 Phase 4 起草时拆为"两口径如何统一"的简短 [TBD] 决策项 + 数字算式移到正文 footnote。
3. **§3.6** "依赖 / 文献" 字段写"本章涉及全部 32 篇综合"，与 §552-583 的"35 篇全覆盖"数字（literature.md 实际 35 篇）口径不一致——建议统一为 35 篇。这是 lessons-learned.md 第 10 条 "穷举计数 vs 分类裁剪" 的提示提醒，避免数字 drift（**轻微，写作角度提示**）。

### 提示 tech / novelty 评审复核（不影响本次 writing 评审 PASS/FAIL 判定）

- §3.6 / §552 都写"32 篇 / 35 篇全覆盖"——具体数字真实性建议 tech 评审复核 literature.md 实际条数。
- §4.5 "Interstellar ASPLOS'20（7 nested loops over DNN）的形式化天然同构" 这种"天然同构"的 framing 较强，建议 novelty 评审复核与 prior art 的 framing 强度是否合适。

---

**写作评审结论**：本次产出在结构整齐度、风格统一度、跨章衔接、术语一致性、narrative 主线连贯性五个维度全部达标，两段 Writer 协作未引入风格分裂。Phase 3 写作质量 PASS。
