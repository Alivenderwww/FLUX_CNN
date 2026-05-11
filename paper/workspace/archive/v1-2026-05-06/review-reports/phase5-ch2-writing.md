# 写作评审报告 Phase 5 §2

## 第三次评审

### 判定：PASS

---

## 评审范围

paper.md line 76-116（§2 Background and Motivation 全章），含四节：
- §2.1 Spatial Array Dataflows: WS / OS / RS Taxonomy（line 78-86，5 段）
- §2.2 PE Utilization Pathology in Shallow Layers（line 88-98，5 段）
- §2.3 Streaming vs Tiled: Memory Footprint of Large-Image Inference（line 100-108，4 段）
- §2.4 Design Goals（line 110-116，3 段）

风格基线对照：§1（line 18-72，已 PASS）。

---

## 段落长度

抽样 6 段（节首段 + 节末段 + 中段）字数：

| 位置 | 字数（英文 word） | 评价 |
|------|---------|------|
| §2.1.p1（line 80, "Spatial accelerators... taxonomy"） | ~95 | OK |
| §2.1.p3（line 84, "FLUX_CNN's 16×16... streaming"） | ~115 | OK |
| §2.2.p1（line 90, "A 16×16 spatial array..."） | ~80 | OK |
| §2.2.p3（line 94, "Two distinct utilization quantities..."） | ~110 | OK |
| §2.3.p2（line 104, "A row-ring streaming..."） | ~125 | OK |
| §2.4.p2（line 114, "Three goals..."） | ~135 | OK |

最长段 §2.4.p2（135 词）仍在 80-200 词区间内；最短 §2.2.p1（80 词）刚好达标。无 >250 词长段，亦无 <50 词短段堆叠。整体节奏与 §1 一致（§1 正文段普遍 90-160 词）。

---

## 主题句

逐段抽查节首主题句：

- §2.1.p1 主题句"Spatial accelerators for convolution are conventionally classified by which tensor remains stationary inside the array"——清晰统领"taxonomy 介绍"。
- §2.2.p1 主题句"A 16×16 spatial array sized for the bulk of a CNN systematically underutilizes its PEs on the shallow layers"——直接给出本节核心论点。
- §2.3.p1 主题句"Edge FPGAs in the 7-Series mid-range present a hard ceiling on resident feature-map storage"——清楚定位约束。
- §2.4.p1 主题句"We now consolidate the two motivations of §2.2 and §2.3 into a small set of design goals"——明示本节角色。

每段均以主题句开头，支持句紧随其后展开数据/例证，结构稳定。无主题句埋在段中段尾的情形。

---

## 段间承接

| 衔接点 | 评价 |
|--------|------|
| §2.1 末 → §2.2 首 | line 86 末句"...the next subsection turns to the quantitative pathology that motivates narrative A — the chronic underutilization of a 16-wide array on shallow ResNet layers"，§2.2.p1 直接接住"systematically underutilizes its PEs on the shallow layers"。承接顺畅，narrative A 标签复用一致。 |
| §2.2 末 → §2.3 首 | line 98 末句"...a second constraint... the on-chip SRAM footprint required to hold a single image of realistic resolution"，§2.3.p1 接"Edge FPGAs... hard ceiling on resident feature-map storage"。承接清晰，"orthogonal"伏笔在 §2.3.p4（line 108）回收。 |
| §2.3 末 → §2.4 首 | line 108 末句"The next subsection rolls the two constraints up into a compact set of design goals"，§2.4.p1"We now consolidate the two motivations of §2.2 and §2.3"——直接对接，连约束编号都列出。 |
| §2.4 末 → §3 首（占位） | line 116 末句"The next chapter walks through these three families in turn and identifies the specific points at which FLUX_CNN diverges from each"，明确铺垫 §3 Related Work。 |

四处衔接句结构一致（"the next ... [verb] ..."），与 §1.1 末/§1.2 末/§1.3 末/§1.4 末的承接句风格一致。无断层。

---

## 与 §1 风格一致性

| 维度 | §1 基线 | §2 实测 | 一致 |
|------|---------|---------|------|
| 句长中位数 | 长复合句为主，节奏 25-45 词 | 同（如 line 92 单句 ~55 词，line 102 单句 ~50 词） | 一致 |
| 段长 | 90-160 词 | 80-135 词 | 一致 |
| `[Tag-Conf'YY]` 引用格式 | `[TPU-ISCA17]`、`[Eyeriss-v2-JETCAS19]` | `[TPU-ISCA17]`、`[Eyeriss-ISCA16]`、`[ShiDianNao-ISCA15]`、`[NVDLA]`、`[Alwani-MICRO16]`、`[Kang-AoCStream-arXiv22, Kang-Sensors23]` | 一致 |
| "we" 使用 | 节制（§1.1 不用"we"，§1.3-§1.4 偶用） | §2.1.p1"We adopt this taxonomy"、§2.2.p3"we keep them separate"、§2.4.p1"We now consolidate"——节制使用 | 一致 |
| 章首段 | §1.1.p1 给定义 + 双约束铺陈，无 chapter-level 简介段 | §2 直接进 §2.1，无 chapter intro paragraph | 一致（§1 也是直进 §1.1） |
| 数据脚注样式 | `<!-- 来自 contributions.md ... -->` HTML 注释 | 同样 8 处 HTML 注释 | 一致 |
| `[CHECK: ...]` 标记句式 | 长描述 + 来源指引 | §2.2.p2 line 92 末"[CHECK: post-Ky-fold 实测百分比待 fold-mode 回归 cycle count 反算，contributions.md §8.1 #1, #2]" | 一致 |
| en-dash 使用 | 大量"— ... —"插入语 | 同（line 90、94、102、108、114 均见） | 一致 |

§2 与 §1 同人称、同句式骨架、同标记体例，未见风格分裂。

---

## 学术英文腔调

- 无 buzzword（无 "intelligent / smart / novel / cutting-edge / state-of-the-art" 等空洞词）。
- 过强 claim 已被规避：line 96"We treat this as a deliberate scope decision rather than a missed optimization"——主动 hedge 而非夸张；line 86"closest to OS"而非"the optimal choice"。
- 第一人称使用克制：全章"we"出现 4 次（§2.1×1, §2.2×1, §2.4×2），均为方法论标注（"We adopt"/"We treat"/"We now consolidate"），非冗余。
- 学术口吻稳定：使用"conventionally classified""canonical""systematically""by construction"等正式短语，与 §1 风格匹配。

---

## 句长节奏

抽查段内句长变化：

- §2.1.p2（line 82）：3 个 example 句（TPU/Eyeriss/ShiDianNao），各 35-45 词，长度相当但结构平行（"X is the canonical Y design: a Z..."），属合理排比，未触发"连续 ≥3 句相同句式"红线（视为列举段，可接受）。
- §2.2.p2（line 92）：长枚举句（"Layer 1... Layers 3 and 4... the early tiles of Layer 5... only from Layer 5b onward..."），分号串联，节奏紧凑。
- §2.3.p1（line 102）：长句（~60 词）+ 中句（~30 词）+ 短句（~25 词，"the platform simply cannot host..."），节奏有起伏。
- §2.4.p2（line 114）：(a)(b)(c) 三段式枚举，每条 30-45 词，结构平行但内容差异化。

无连续 ≥3 句完全同句式的情形。整体节奏与 §1.1（line 20-28）相当。

---

## 术语一致

| 术语 | §1 用法 | §2 用法 | 一致 |
|------|---------|---------|------|
| 16×16 INT8 array | §1.1, §1.3 | §2.1.p3, §2.2.p1, §2.4.p2 | ✅ |
| spatial PE utilization / network-level MAC utilization | §1.1, §1.4 | §2.2.p3 显式定义并区分 | ✅（§2.2.p3 增强了定义） |
| narrative A / narrative B | §1.3, §1.4 | §2.2.p1, §2.3.p3, §2.4.p3 | ✅ |
| row-ring / row-level streaming | §1.3 "row-level streaming"/"row-ring buffer" | §2.1.p3 "row-ring streaming feature-map path"、§2.3.p2 "row-ring streaming data path" | ✅ |
| Cin / Cout（大小写） | §1.1 "Cin = 4" | §2.2.p2 "Cin = 4, K = 7"、§2.4 同 | ✅ |
| line_buffer → mac_array → parf_accum → ofb_writer 模块名 | §1.3.p1 | §2.1.p4（line 86）chain 同序 | ✅ |
| `strip_rows` | §1.3.p3 "strip_rows = 8" | §2.3.p2 "`strip_rows`" | ✅ |
| BRAM36 / 1.6 MB / 4.9 MB / 10 KB | §1.1.p4 | §2.3.p1-p2 同口径 | ✅ |

未发现别名混用。

---

## HTML 注释处理

§2 共出现 6 处 `<!-- 来自 ... -->` 注释（line 92, 96, 102, 104, 114×3）。检查：

1. 全部位于句末或段末，渲染后不打断阅读节奏。
2. 与 §1 同体例（§1 也大量使用 `<!-- 来自 contributions.md ... -->`）。
3. Markdown 渲染不可见，不污染读者视图。
4. 标注精确（含具体小节号 / 行号），便于 tech 评审追溯。

处理得当，无需调整。

---

## 提示（不影响判定）

以下为可能的事实性疑点，**写作评审不审，仅提示 tech 评审复核**：

- line 102 "approximately 4.9 MB"（480×640×16 ≈ 4.92 MB）与 line 26 §1.1 数字一致，建议 tech 复核此一致性是否有意。
- line 104 "8 × 640 = 5120 bytes per channel and on the order of 10 KB total"——5120 字节×多通道与 10 KB 总量是否兼容，请 tech 验算。
- line 92 "Layer 1 (Cin = 4, K = 7)... 4 of the 16 column lanes... 12.5% PE utilization"——12.5% = 32/256 而非 4/16，行内描述只提了 column lanes 维度，与 §1.1 line 24"32 of 256 useful MACs"口径需 tech 确认是否一致表述。

---

## 通过原因

1. **段落结构稳健**：所有段以主题句开头，长度在 80-135 词区间，无失衡。
2. **段间承接顺畅**：四处节间衔接句句式一致、回收上节关键概念，无断层。
3. **与 §1 风格高度一致**：人称、引用格式、HTML 注释体例、`[CHECK]` 标记句式、en-dash 用法、章首段处理全部对齐——**未见 Writer 实例切换导致的风格分裂**。
4. **学术腔调克制**：无 buzzword、无 over-claim，hedge 用词得当。
5. **术语零别名**：narrative A/B、PE utilization 双义、模块名链、内存数字全章一致。
6. **HTML 注释处理得当**：渲染层与审计层分离，与 §1 一致。

无 FAIL 触发项（无连续 ≥3 段 evidence 无 claim、无 ≥2 处术语不统一、无段长严重失衡、轻微问题 < 3）。

---

（无 FAIL，无修订建议项。）
