# 技术评审报告 Phase 5 §2 (Background and Motivation)

## 第 2 次评审（前次 SSL 错重启）

### 判定
FAIL

### 评审范围
仅 §2（paper.md line 76-116）。§1 已 PASS 不复评，§3-§8 仍为 [TBD] 占位不评。

---

### 段落骨架对齐

paragraph-skeleton.md §2 共 16 段（§2.1 4 段 / §2.2 5 段 / §2.3 4 段 / §2.4 3 段）。
逐段比对：

| skeleton 段 | paper.md 段 | 对齐 |
|---|---|---|
| §2.1 段 1 setup (WS/OS/RS 定义) | §2.1 paragraph 1 | ✅ |
| §2.1 段 2 evidence (TPU/Eyeriss/ShiDianNao) | §2.1 paragraph 2 | ✅ |
| §2.1 段 3 claim (FLUX_CNN OS-like) | §2.1 paragraph 3 | ⚠️ 见下"背景知识"#1 |
| §2.1 段 4 transition (NVDLA 命名) | §2.1 paragraph 4 | ✅ |
| §2.2 段 1 claim (chronic underutilization) | §2.2 paragraph 1 | ✅ |
| §2.2 段 2 evidence (per-layer 数字) | §2.2 paragraph 2 | ✅ |
| §2.2 段 3 setup (单层 vs 整网口径) | §2.2 paragraph 3 | ✅ |
| §2.2 段 4 claim (Cout<16 不优化) | §2.2 paragraph 4 | ✅ |
| §2.2 段 5 transition | §2.2 paragraph 5 | ✅ |
| §2.3 段 1 setup (1.6 MB vs 4.9 MB) | §2.3 paragraph 1 | ✅ |
| §2.3 段 2 claim (row-ring streaming 必要) | §2.3 paragraph 2 | ✅ |
| §2.3 段 3 comparison (Alwani / Kang) | §2.3 paragraph 3 | ✅ |
| §2.3 段 4 transition + [TBD] 图 | §2.3 paragraph 4 | ✅ |
| §2.4 段 1 setup | §2.4 paragraph 1 | ✅ |
| §2.4 段 2 claim (3 design goals) | §2.4 paragraph 2 | ✅ |
| §2.4 段 3 transition (3 谱系定位) | §2.4 paragraph 3 | ✅ |

段落数 / 顺序 / 角色（setup/evidence/claim/transition）对齐良好。

---

### 背景知识准确性（4 处抽样）

**#1 §2.1 paragraph 3 FLUX_CNN OS-like 数据流描述 — ❌ 严重错误（行/列方向反了）**

paper.md line 84 原文：
> "each column is dedicated to a distinct Cout channel and accumulates its partial sum locally inside `parf_accum`, **while each row sees the same activation broadcast and a per-row weight from the WRF**."

事实（来自 `RTL/mac_array.sv` line 36-58 + `docs/modules/mac_array.md` line 25-33, 96-99 + `docs/modules/mac_col.md` line 24, 33, 60）：

- `act_in_vec[NUM_PE×8]` = "16 个 cin 通道在某像素位置的拼接"
- `docs/modules/mac_col.md:60`: "16 列共享 `wrf_raddr / act_in_vec / compute_en`（**cin 维输入广播给所有 cout 列**）"
- 即：**所有 16 列共享同一个 act_in_vec**（列广播激活）；**每列内部 16 个 PE 对应不同 cin 通道**（row=cin 维），每个 PE 取 `act_in_vec` 的对应 8-bit 片段
- WRF 是 per-PE（256 PE × WRF_DEPTH=32），不是 "per-row"

正文写法把行/列方向搞反了：
- "each row sees the same activation broadcast" ❌ — 实际是 each **column** sees the same act_in_vec; rows correspond to different cin channels
- "per-row weight from the WRF" ❌ — WRF 是 per-PE，不是 per-row

紧接的下一句"This 'column-broadcast activation, per-column output' arrangement"反而是对的——这与前一句自相矛盾。

**这是与 RTL 直接冲突的事实错误，必须改。**

**#2 §2.1 paragraph 2 TPU / Eyeriss / ShiDianNao 描述 — ✅**
- TPU v1 256×256 INT8 systolic / WS：与 literature.md line 109-115 一致
- Eyeriss 14×12 RS：与 literature.md line 117-125 一致（核心方法 14×12 = 168 PE）
- ShiDianNao OS：与 literature.md line 193-199 一致

**#3 §2.2 paragraph 2 利用率百分比 — ✅**
- "Layer 1 (Cin=4, K=7) → 12.5%" 与 model_analysis.md line 81 一致 (32/256=12.5%)
- "Layer 3-4 (Cin=8) → 25%" 与 model_analysis.md line 96 一致 (64/256=25%)
- "Layer 5a-c (Cin=8, Cout=16) → 50%" 推算 128/256=50% 合理（与 model_analysis.md line 167 "Layer 3,4,5a,5c... 25-50% PE" 一致）
- "FC_xy (Cout=2) 2/16 column lanes" 与 model_analysis.md line 67, 94 一致 (12.5%)
- HTML 来源注释 "来自 contributions.md §4.4 / model_analysis.md §2" 可追溯

**#4 §2.3 paragraph 1 BRAM 与 image footprint — ✅**
- "445 BRAM36 tiles ≈ 1.6 MB" 与 STATUS.md line 36 (445 容量) 一致；445 × 36 kbit / 8 / 1024 / 1024 ≈ 1.96 MB（"roughly 1.6 MB" 略保守但量级正确，且未声称精确）
- "an FLUX_CNN core already commits 128 BRAM36" 与 STATUS.md line 36 (单核 BRAM36 用量 128) 一致
- "VGA 480×640 INT8 16 channels ≈ 4.9 MB"：480×640×16 = 4,915,200 bytes ≈ 4.69 MB，"approximately 4.9 MB" 合理
- "strip_rows=8 × W=640 = 5120 bytes/channel ≈ 10 KB total" 量级合理

---

### 引用真实性（4 处抽样）

**已不联网核（按指令）**，仅核对"paper 引用 key 是否在 literature.md 内有对应条目"：

| paper.md 引用 key | literature.md 是否有条目 | 备注 |
|---|---|---|
| `[TPU-ISCA17]` | ✅ literature.md line 107-115（Jouppi et al.@ISCA'17，DOI 10.1145/3079856.3080246）| 名称对位 |
| `[Eyeriss-ISCA16]` | ✅ literature.md line 117-125（Chen et al.@ISCA'16，DOI 10.1145/3007787.3001177）| 名称对位 |
| `[ShiDianNao-ISCA15]` | ✅ literature.md line 193-201（Du et al.@ISCA'15，DOI 10.1145/2749469.2750389）| 名称对位 |
| `[NVDLA]` | ✅ literature.md line 157-165（vendor doc，已标 [CHECK]）| 名称对位 |
| `[Alwani-MICRO16]` | ✅ literature.md line 299-307（Alwani et al.@MICRO'16，DOI 10.1109/MICRO.2016.7783725）| 名称对位 |
| `[Kang-AoCStream-arXiv22]` | ✅ literature.md line 309（已修正 venue）| 名称对位 |
| `[Kang-Sensors23]` | ✅ literature.md line 309 同条（Sensors'23 复述）| 名称对位 |

> 注：`[Kang-AoCStream-arXiv22, Kang-Sensors23]` 在 §2.3 paragraph 3 同时出现，对应同一作者 / 主题的 arXiv 与期刊版，引用方式合理。

未发现幻觉引用 / 张冠李戴。所有引用 key 都能在 literature.md 找到对应条目。**离线情况下仍建议在 Phase 6 之前用 `check-citations` skill 跑一次 .bib，覆盖 ChrossRef/Semantic Scholar 真实性核验。**

---

### claim 强度

抽样审查 §2 内 claim 是否在 contributions.md / literature.md / model_analysis.md / STATUS.md 备书：

- §2.1 "FLUX_CNN's 16×16 INT8 array adopts a hybrid that is closest to OS" — 在 literature.md line 112 ("FLUX_CNN 是 OS 列广播") 备书 ✅
- §2.1 "OS introduces the least coupling between the array and the line buffer" — 这是定性论证，未引文献，但作为设计选择动机的说理是允许的（不算硬 claim）✅
- §2.2 "the layers that are most expensive in raw cycle count, because their H×W is largest, are precisely the layers at which the array runs at a fraction of its nominal throughput" — model_analysis.md line 195 "Layer 1 是瓶颈，k=7 Conv 占 62% 计算量" 备书 ✅
- §2.2 "the cycle cost of such layers in a ResNet-18-style network is negligible relative to the cycle cost of the early convolutional layers" — model_analysis.md line 67 (FC_xy <0.01% 算量) 备书 ✅
- §2.3 "Streaming is therefore not an optimization that one could choose to skip on this platform; it is the only data path that fits" — 由 BRAM/image 数字直接推论，备书在前 ✅
- §2.4 三条 design goals — HTML 注释指向 contributions.md C1.1 / C2.1 / C2.2 / C1.2，对位合理 ✅

**未发现 §2 内未备书的过强 claim（除 #1 已被 RTL 直接证伪的事实错误）。**

---

### [CHECK]/[TBD] 合规性

- §2.2 paragraph 2 末尾 `[CHECK: post-Ky-fold 实测百分比待 fold-mode 回归 cycle count 反算，contributions.md §8.1 #1, #2]` — 位置合理：原文给出的 12.5/25/50/100% 是 **unfolded（无 fold）** 利用率，由 Cin/16 推得，是无需实测就能给出的结构性数字；而 fold 之后的实测利用率确实需要实测才能确认。**[CHECK] 范围划分得当。**
- §2.3 paragraph 4 末尾 `[TBD: 是否在此处插入 SRAM-vs-image-size 曲线图作为 Fig.2，待用户决定]` — 合理，是图表决策悬挂项 ✅
- 7 处 HTML `<!-- 来自 ... -->` 来源注释 — 抽查 §2.2 p2、§2.2 p4、§2.3 p1、§2.3 p2、§2.4 p2 (3 处) 都能在指向位置找到（contributions.md / model_analysis.md / STATUS.md），追溯链条完整 ✅

**[CHECK]/[TBD] 标记位置无问题。**

---

### 回归性

paragraph-skeleton.md §2 列出的依赖：TPU / Eyeriss / ShiDianNao / NVDLA / Alwani / Kang。
paper.md §2 引用集合：TPU-ISCA17 / Eyeriss-ISCA16 / ShiDianNao-ISCA15 / NVDLA / Alwani-MICRO16 / Kang-AoCStream-arXiv22 / Kang-Sensors23。
**完全一致，无骨架外新引用。**

skeleton 中提到的所有数字（12.5% / 25% / 50% / 100% / 1.6 MB / 4.9 MB / 10 KB / strip_rows=8）在 paper 中都有对应表述，**未引入骨架外的新数字**。

---

### 失败原因

§2.1 paragraph 3 第二句对硬件数据流方向的描述与 RTL 直接冲突（行/列搞反）。这是技术评审零容忍项——对一个声称要"developing the dataflow taxonomy"的章节，把自己的硬件数据流方向写错是审稿人致命攻击点。

虽然只有 1 处事实错误，但：
1. 错的就是本章核心 claim（FLUX_CNN 自身数据流分类）
2. 同段下一句的 "column-broadcast activation, per-column output" 是对的——上下两句互相矛盾，审稿人会立刻发现
3. 这个错误延伸了 §2.1 paragraph 2 关于 RS / WS / OS 的对比叙事——如果作者自己把行/列方向写错，那整个 §2.1 的可信度受质疑

---

### 给 Writer 的修订建议

**唯一必修点 (P0)**：§2.1 paragraph 3 第二句改写。

原文（line 84）：
> "FLUX_CNN's 16×16 INT8 array adopts a hybrid that is closest to OS: each column is dedicated to a distinct Cout channel and accumulates its partial sum locally inside `parf_accum`, while each row sees the same activation broadcast and a per-row weight from the WRF."

建议改为（与 RTL 一致）：
> "FLUX_CNN's 16×16 INT8 array adopts a hybrid that is closest to OS: each column is dedicated to a distinct Cout channel and accumulates its partial sum locally inside `parf_accum`; all 16 columns share the same 16-channel activation vector `act_in_vec` (a column-broadcast of activations across one pixel position), while within each column the 16 PEs correspond to distinct Cin channels and read per-PE weights from the WRF."

—— 这样下一句 "This 'column-broadcast activation, per-column output' arrangement maps naturally onto..." 就有了正确的承接。

**修订关键点**：
1. `each row sees the same activation broadcast` → 改为 `all columns share the same act_in_vec`（**列广播**，不是行广播）
2. `per-row weight from the WRF` → 改为 `per-PE weights from the WRF`（WRF 是 per-PE，不是 per-row）
3. 补一句 "within each column the 16 PEs correspond to distinct Cin channels" 说清行=cin 维，避免后续 §2.2 讲 "Cin/16 PE utilization" 时读者无法把 Cin 与硬件维度对应

无其它必修点。修订后重测应可 PASS。

---

## 第 1 轮重测（2026-05-01）

### 判定
PASS

### 评审范围
仅核第 2 轮 FAIL 报告中唯一 P0 问题（§2.1 paragraph 3 行/列方向 + WRF 粒度）的修复落实情况。其他抽样（#2-#4）/ 引用真实性 / claim 强度 / [CHECK]/[TBD] / 段落骨架对齐 / 回归性已在第 2 轮 PASS，本轮不复评。

### 修复确认

**§2.1 paragraph 3（paper.md line 84）当前文本**：
> "FLUX_CNN's 16×16 INT8 array adopts a hybrid that is closest to OS: each column is dedicated to a distinct Cout channel and accumulates its partial sum locally inside `parf_accum`; all 16 columns share the same 16-channel activation vector `act_in_vec` (a column-broadcast of activations across one pixel position), while within each column the 16 PEs correspond to distinct Cin channels and read per-PE weights from the WRF <!-- 来自 RTL/mac_array.sv L36-58 + docs/modules/mac_array.md L25-33,96-99 + docs/modules/mac_col.md L24,33,60 -->."

逐项核对：

| 修订关键点 | 修后文本 | RTL/docs 证据 | 判定 |
|---|---|---|---|
| 列广播激活（不是行广播）| "all 16 columns share the same 16-channel activation vector `act_in_vec` (a column-broadcast of activations across one pixel position)" | `docs/modules/mac_col.md` L60: "16 列共享 `wrf_raddr / act_in_vec / compute_en`（cin 维输入广播给所有 cout 列）"；`RTL/mac_array.sv` L52 信号声明 `act_in_vec[NUM_PE×8]` 单端口广播 | ✅ 修复 |
| 行 = Cin 维 | "within each column the 16 PEs correspond to distinct Cin channels" | `docs/modules/mac_array.md` L24,33: "`act_in_vec[NUM_PE×8]` = 16 个 cin 通道的输入像素拼接 / 每 PE 的 `act_in` 取 `act_in_vec` 的对应 8-bit 片段（即第 i 个 cin 通道的像素）" | ✅ 修复 |
| WRF 粒度（per-PE 而非 per-row）| "read per-PE weights from the WRF" | `RTL/mac_array.sv` L47 `wrf_we[NUM_COL*NUM_PE-1:0]`（256 位 per-PE 写使能）；`mac_col.md` L20 "每 PE 独立写使能" | ✅ 修复 |
| 来源注释 | `<!-- 来自 RTL/mac_array.sv L36-58 + docs/modules/mac_array.md L25-33,96-99 + docs/modules/mac_col.md L24,33,60 -->` | 三处指向均存在且对应内容支撑正文 | ✅ 合理 |

**与上下句承接**：
- 前句（同段开头）"each column is dedicated to a distinct Cout channel and accumulates its partial sum locally" — 与新文本"per-column output / Cout"一致 ✅
- 后句（同段第二句）"This 'column-broadcast activation, per-column output' arrangement maps naturally onto a row-ring streaming feature-map path" — 现在"column-broadcast activation"恰好承接新文本中的"a column-broadcast of activations across one pixel position"，"per-column output"承接"each column is dedicated to a distinct Cout channel"。**自洽。** ✅
- 与 §2.2 paragraph 1 后续"Cin/16 上界"叙事一致：现在读者已知"列=Cout，行=Cin"，§2.2 讲 "Cin much smaller than the array width" 时硬件维度对应清晰 ✅

### 回归性

逐段对比第 2 轮 FAIL 报告中引用的 §2 各段文本与本次 paper.md line 76-116 的当前文本：
- §2.1 paragraph 1（line 80）：未变
- §2.1 paragraph 2（line 82, TPU/Eyeriss/ShiDianNao 段）：未变
- §2.1 paragraph 3（line 84）：仅第二个分句改写 + 补 HTML 来源注释（即 Writer 自报范围）
- §2.1 paragraph 4（line 86, NVDLA 段）：未变
- §2.2 paragraph 1-5（line 90-98）：全部未变
- §2.3 paragraph 1-4（line 100-108）：全部未变
- §2.4 paragraph 1-3（line 110-116）：全部未变

**Writer 自称"只动 1 处"属实，无意外回归。**

### 通过原因

第 2 轮唯一 P0 严重问题（"each row sees the same activation broadcast" + "per-row weight from the WRF"——行/列方向反 + WRF 粒度错）已按建议修订到位：

1. 列广播 / 行=Cin / WRF=per-PE 三个事实点都改对了，且每一点都能在 RTL/mac_array.sv + docs/modules/mac_array.md + docs/modules/mac_col.md 中找到直接证据
2. HTML 来源注释明确指向上述三处证据，可追溯
3. 同段下一句"column-broadcast activation, per-column output"现在不再与本句矛盾，反而由本句直接铺垫
4. §2 其他段落未被意外改动，无回归

§2 现在没有未解决的事实错误。技术维度判定 PASS。
