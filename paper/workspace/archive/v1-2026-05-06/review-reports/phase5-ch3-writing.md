# 写作评审报告 Phase 5 §3 Related Work

## 第 1 次评审

### 判定
PASS

### 评审范围
仅 §3 Related Work（§3.1–§3.6，21 段，paper.md line 119–181）。§1/§2 已 PASS，作为风格基线参考；§4–§8 为占位章节，不在评审范围内。

### 段落长度合规（6 段抽样）

抽样以词数为准（中文字数粗略对应于英文 1.5–2 倍）。窗口为 50–160 词。

| 抽样位置 | 词数（粗估） | 判定 |
|---------|-------------|------|
| §3.1 段 1（line 123） | ~75 | OK |
| §3.1 段 2（line 125, TPU/Eyeriss/Gemmini 列举段） | ~150 | OK，接近上限但单段聚焦"array scale 跨 3 数量级"主题成立 |
| §3.3 段 2（line 145, Timeloop / Interstellar / TVM / Ma / cuDNN） | ~140 | OK |
| §3.4 段 2（line 155, fpgaConvNet/Snowflake/Angel-Eye/Aydonat/Lu） | ~150 | OK，6 work 罗列但未爆段 |
| §3.4 段 5（line 161, Liu et al. 威胁段） | ~135 | OK |
| §3.6 段 2（line 177, 五轴 enumeration） | ~165 | 略超上限 5%，但 enumeration 结构清晰，每条 (1)–(5) 自带 "anchored against …" 平行句法，作为收尾段可接受 |

无 <50 词短段堆叠，无 >200 词超长段。

### 段落主题句结构

每段开头主题句清晰，列举其中几例验证：
- §3.1 段 1："Spatial array accelerators ... we organize related work along three axes..." — 元结构句，立刻给出三轴框架。
- §3.2 段 1："A first family of responses to shape-mismatch ... introduces hardware-reconfigurable interconnect..." — 直接命名"first family"，主题清晰。
- §3.4 段 1："The streaming / line-buffer family is the most directly comparable family ... and three works within it ... represent the closest prior art" — 同时设定威胁段 framing。
- §3.5 段 1："INT8 quantization and post-processing fusion ... are by now standard ingredients ... we touch on them only as background" — scope-limit 句，巧妙降权这一节。
- §3.6 段 1："We close this chapter by placing FLUX_CNN explicitly along the five axes..." — 收束 framing 显式。

无主题句埋中段的情况。

### 段间承接

- §2.4 末段 → §3.1 首段：§2.4 末"three families and identifies the specific points"，§3.1 首"organize related work along three axes" — 衔接顺，从"三家族"自然过渡到"三轴坐标"。
- §3.1 末 → §3.2 首：§3.1 末"the first of the two design choices that distinguish FLUX_CNN ... how to recover PE utilization on layers whose Cin is below the array width"；§3.2 首"A first family of responses to shape-mismatch" — 主语对齐。
- §3.2 末 → §3.3 首：§3.2 末"the language in which compiler-side approaches are usually expressed — loop-nest formalisms..."；§3.3 首"Compiler-side and loop-nest co-design forms the natural sibling family" — "natural sibling" 衔接显式。
- §3.3 末 → §3.4 首：§3.3 末"The next subsection moves to the streaming family proper..."；§3.4 首"The streaming / line-buffer family is the most directly comparable..." — 衔接精确。
- §3.4 末 → §3.5 首：§3.4 末"briefly addresses the algorithmic-side concerns — quantization and post-processing fusion — that frame FLUX_CNN's SDP block"；§3.5 首"INT8 quantization and post-processing fusion ... are by now standard ingredients" — 衔接顺。
- §3.5 末 → §3.6 首：§3.5 末未显式承接（以 [TBD] 收尾），§3.6 首"We close this chapter by placing..." — 衔接靠 §3.6 自带 "close this chapter" 元词。轻微，不构成断层。
- §3.6 末 → §4 占位："The next chapter develops the architecture (axis 1) in detail; §5 ... §6 ... §7" — 已显式铺垫 §4–§7，恰当。

### 与 §1+§2 风格一致性

| 维度 | §1+§2 基线 | §3 | 一致性 |
|------|-----------|----|----|
| 段长 | ~80–160 词 | ~75–165 词 | 一致 |
| Short tag 引用 | `[Eyeriss-ISCA16]`、`[Gemmini-DAC21]` | 同（`[MAERI-ASPLOS18]`、`[Liu-FullStack-TNNLS21]`） | 一致 |
| "we" 频率 | 每节 1–3 次（"We adopt this taxonomy", "We treat this as..."） | §3 各节均 1–3 次（"We organize ... ", "We treat ... ", "We close ... "） | 一致 |
| Claim 强度 | 偏 hedged（"to our knowledge", "deliberate scope decision"） | 同（"to our knowledge ... distinct from", "we are deliberate about not over-claiming"） | 一致，甚至更克制 |
| HTML 注释密度 | §1+§2 共 ~6 处 | §3 共 4 处（§3.1, §3.3, §3.4 ×2） | 一致 |
| 元结构句（"this section / next subsection"） | 多见 | 多见，未滥用 | 一致 |

§3 与 §1+§2 的笔触统一度很高，看不出多 Writer 实例的痕迹。

### 学术英文腔调

- 无 buzzword（"revolutionary"/"groundbreaking"/"state-of-the-art" 等未出现）。
- Claim 全部 hedged：例 §3.4 "We read this comparison as a same-philosophy point on a different Pareto frontier rather than as a head-to-head comparison" — 主动降权而不是夸耀。
- 第一人称 "we" 使用得当，集中在元结构句与判断句，不滥用。
- 时态：方法/工作描述多用现在时（"FLUX_CNN occupies", "Alwani et al. fuse"），评述用现在时（"is the canonical RS reference"），一致。
- 无情绪词、无营销腔。

### 句长与节奏

抽查若干长句：
- §3.1 line 125（TPU/Eyeriss/...）—— 通过分号分句、逗号 + dash 把六个 work 的列举铺成一段，每个 work 平均 1.5 句，节奏可读。
- §3.4 line 157（Alwani 威胁段）—— 5 句，长短交替（"Alwani et al. fuse the first 5 conv layers..."短；"The differentiation point for FLUX_CNN is granularity and resource regime: ..."长 split by 冒号），不存在 60+ 词从句套娃。
- §3.6 line 177（五轴 enumeration）—— 用 "(1) ... ; (2) ... ; (3) ... ; (4) ... ; and (5) ..." 平行结构承担长度，每条 ~30 词，节奏稳。

未发现 ≥3 连续相同句式。

### 术语一致性

抽查关键术语：
- "spatial array" / "spatial PE utilization" — §1, §2, §3 一致使用。
- "OS-with-column-broadcast" — §2.1 line 83 首次出现（无显式定义，但上下文已说明），§3.1 line 127 / §3.6 line 177 重用，一致。
- "row-ring" / "row-level streaming" — §1.3, §2.3, §3.4, §3.6 全部使用 "row-level" 或 "row-ring"，未与 "row streaming"/"row-stream" 混用。
- "Ky-fold" / "S2D"（"Space-to-Depth"）—— §1, §2, §3 一致；首次展开 S2D 在 §1.3 line 45。
- "PE" 缩写：§1.1 line 19 首次使用前已用 "MAC" 与 "16×16 INT8 grid"，line 21 "spatial PE utilization" 已展开为完整缩写形式（"PE" 在硬件加速器领域属于约定俗成缩写，未显式 expansion 可接受，与基线 §2.2 处理一致）。
- "WS / OS / RS"：§2.1 line 79 显式展开，§3 各处复用，一致。
- "ART" 在 §1.2 line 33 与 §3.2 line 135、§3.6 line 177 一致使用，且 line 33/135 首次出现都展开为 "Augmented Reduction Tree (ART)"——按 §3.2 line 135 仍展开来说有微小冗余，但属"在 Related Work 单独再展一次"的可接受惯例。
- "BRAM36"、"SRAM" — 与 §1, §2 用法一致。

无术语漂移。

### 三大威胁段落可读性

§3.4 段 3 (Alwani, line 157)、段 4 (Kang, line 159)、段 5 (Liu, line 161) 每段均按"prior work 描述 → 差异化点命名 → FLUX_CNN 立场 → 不踩对方"四步：
- Alwani 段："The differentiation point for FLUX_CNN is granularity and resource regime: ..."；段尾 "not a strict improvement ... but a different point on the streaming-vs-fusion design space" 立场克制。
- Kang 段："The differentiation here is layer-pipelined-multi-block versus layer-serial-single-core, and the position taken on external memory: ..."；段尾"AoCStream achieves higher peak throughput at the cost of more replicated MAC hardware" 公正承认对方优势。
- Liu 段："The differentiation here is principally one of scale rather than approach: ..."；段尾 "same-philosophy point on a different Pareto frontier rather than as a head-to-head comparison" 显式拒绝 head-to-head 对比，避免 over-claim。

三段格式平行，读者能并列对照三家差异；论证密集但被结构化压低了认知负担。可读性达标。

### §3.6 Positioning 收尾

§3.6 三段：
- 段 1：定调（"close this chapter by placing FLUX_CNN explicitly along the five axes"）。
- 段 2：枚举五轴坐标，每条 "FLUX_CNN is X, anchored against Y" 格式平行。
- 段 3：把 §3.2/§3.3/§3.4 的差异化论证回挂到 (2)/(3)/(4)/(5) 轴上，并将 §3.5 一节降权为"轴外背景"（隐含），最后给 §4–§7 铺路。

读完此节，读者得到一个明确的"五轴坐标 + FLUX_CNN 在每轴的位置"心智模型，效果到位。niche 描述清楚，没有"FLUX_CNN 是首个 X" 这类 unhedged 自夸。

### HTML 注释处理

§3 共 4 处 `<!-- 来自 ... -->`：
- line 127: `<!-- 来自 contributions.md C1.1 + RTL/mac_array.sv -->`
- line 149: `<!-- 来自 contributions.md §4.3 / STATUS.md line 355 -->`
- line 161 (×2): `<!-- 来自 contributions.md §5.3 -->` 与 `<!-- 来自 contributions.md §4.3 -->`

密度 4/21 段 ≈ 19%，与 §1+§2 大致相当（§2.1–§2.4 间约 6 处 ≈ 35–40%，§1 约 15%）。位置全部在数字 / 资源 claim 处，使用得当。无过密。

### 通过原因

- 段落长度全合规（最长段 §3.6 段 2 ~165 词，仅超上限 5%，但作为结构化收尾段可接受）。
- 段间承接 6 个跨节衔接全部到位，仅 §3.5→§3.6 衔接靠 §3.6 自带元词承担，未达断层级。
- 与 §1+§2 风格一致度高（多 Writer 痕迹不可见）。
- 学术腔调克制，无 buzzword、无 over-claim。
- 三大威胁段落（Alwani/Kang/Liu）格式平行、论证清晰。
- §3.6 五轴收尾给出明确坐标系，niche 立场清楚。
- 术语一致，HTML 注释使用得当。

### 轻微建议（可选，不影响 PASS）

1. §3.6 段 2（line 177）轻微超长（~165 词），如终稿压力允许，可在 axis (3) 与 axis (4) 之间 break 一次（"... on a much larger device. (4) The multi-core scaling dimension..."），更易读。非必需。
2. §3.5 末段以 `[TBD]` 收尾后无显式 transition 句到 §3.6，目前靠 §3.6 段 1 自带的 "We close this chapter" 承担。可考虑在 §3.5 段 2 末尾补一句 "The next subsection consolidates the foregoing discussion into a five-axis positioning of FLUX_CNN." 之类。非必需。
3. §3.4 段 2 列举 6 个 work（fpgaConvNet/Snowflake/Angel-Eye/Aydonat/Lu）共 ~150 词，已是该段上限；如后续 reviewer 阶段补加新 work，建议拆为两段。当前不改。

### 提示给主 Agent（非 writing 评审范围）

- §3.3 line 149 引用 "86.6% network-level MAC utilization on a 16× smaller mid-range FPGA" 与 §3.4 line 161 同句重复；事实上无误，但措辞紧贴重复。tech reviewer 已应能查实数值真伪；writing 角度建议第二处可改为 "(see §3.3)" 引回，不必再列同一数字 — 此为 stylistic suggestion，不影响 PASS。
- §3.4 line 159 "AoCStream achieves higher peak throughput at the cost of more replicated MAC hardware" 是对对方的事实判断（"higher peak throughput"），建议 tech 评审复核此 claim 是否准确（如果 Kang 论文中 throughput 较 FLUX_CNN 实际并不更高，需要修；writing 上该句结构 OK）。
