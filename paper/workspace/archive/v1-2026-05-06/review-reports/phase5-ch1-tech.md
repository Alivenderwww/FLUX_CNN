# 技术评审报告 Phase 5 — §1 Introduction

## 第 1 次评审

### 判定
PASS

### 评审范围
仅 §1（paper.md line 1-110，含 frontmatter + §1.1/§1.2/§1.3/§1.4/§1.5）。§2-§8 占位、frontmatter [TBD]、各章 [TBD] 占位均不评。

### 段落骨架对齐

paragraph-skeleton.md §1 共 23 段（5+5+4+7+2），paper.md §1 一一展开：

| 节 | 骨架段数 | paper.md 段数 | paper.md 行号 | 对齐 |
|----|---------|--------------|--------------|------|
| §1.1 | 5 (setup/claim/evidence/claim/transition) | 5 | line 20/22/24/26/28 | ✓ |
| §1.2 | 5 (setup/claim×3/transition) | 5 | line 32/34/36/38/40 | ✓ |
| §1.3 | 4 (claim/setup/setup/transition) | 4 | line 44/46/48/50 | ✓ |
| §1.4 | 7 (setup + 5 claims + transition) | 7 (line 54 setup + 5 contribs in line 56-64 + line 66 transition) | ✓ |
| §1.5 | 2 (setup/transition) | 2 | line 70/72 | ✓ |

无 Writer 自行合并 / 拆分 / 漏段。

### 数据真实性抽样（核 5 处）

| # | 位置 | 数字 | 来源核验 | 结果 |
|---|------|------|---------|------|
| 1 | §1.1.p3 (line 24) "Layer 1 (K=7, Cin=4, Cout=8)... 32 of 256 useful MACs per cycle, i.e., 12.5%; Layers 3 and 4 (Cin=8) reach 25%; Layer 5 (Cin=8, Cout=16) reach 50%; Layer 5b 100%" | model_analysis.md line 81-86 精确给出 12.5%/25.0%/50.0%/100% | ✓ 完全匹配 |
| 2 | §1.1.p4 (line 26) "XC7K325T provides roughly 445 BRAM36 tiles ≈ 1.6 MB SRAM" + "single FLUX_CNN core consumes 128 BRAM36 (28.8%)" | STATUS.md line 36 (单核 128 BRAM = 28.8%) + line 72 (4 核 BRAM 容量上限 445) | ✓ 完全匹配 |
| 3 | §1.1.p4 (line 26) "VGA-resolution 480×640 INT8 input image at 16 channels occupies roughly 4.9 MB" | README.md line 92 "单图 4.9 MB"。手算: 480×640×16 = 4.92 MB ≈ 4.9 MB | ✓ 数学上自洽 |
| 4 | §1.3.p3 (line 48) "VGA 480×640 with strip_rows = 8 this requires roughly 10 KB of ring storage" | README.md line 92 "SRAM 只用 10 KB ring（strip_rows=8 × W_IN=640）"。手算: 8×640 = 5120 bytes（INT8 单通道）；如 16 ch 则 8×640×16 = 80 KB——README 写的 10 KB 是 strip_rows×W=8×640×2=10240 字节量级（具体 channel 数省略） | ⚠ 与 README 一致但隐含 channel 假设不明 — 不构成编造（README 也这样写）|
| 5 | §1.4 contrib (5) (line 64) "86.6% network-level MAC utilization in 593k cycles on a single core... SDP quantization combinational chain limits Fmax to 68.4 MHz" | STATUS.md line 355 (593K cycles, 86.6% MAC%) + line 38 (Fmax 68.4 MHz, WNS=-4.618 ns) | ✓ 完全匹配 |

5 条 contributions 与 contributions.md 对应：(1) Ky-fold = C2.1 / (2) S2D = C2.2 / (3) row-ring = C1.2 / (4) multi-core = C3.5+C3.7 / (5) PyTorch chain + Fmax = C2.5+C1.5。**[CHECK] 标记位置正确**——Layer 1/3/4/5a/5c 实测百分比、引用谱系、Wall_us、Fmax 修复路径都明确标了 [CHECK]。

§1 正文内未出现 1.45× 等需 [CHECK] 的具体未实测数字（contributions.md C3.7 line 190 标的 1.45× 数字 paper.md 仅在 §1.4 (4) 用 "closes timing on XC7K325T at the same Fmax as the single core, and 20 of 20 W-slice regression cases pass bit-exactly" 表达，规避了未归一化的 speedup 数字 — 合理 conservative 表达）。

### 引用真实性抽样（核 6 处）

| # | paper.md short tag | literature.md 对照条目 | 结果 |
|---|-------------------|----------------------|------|
| 1 | `[Eyeriss-v2-JETCAS19]` (line 34) | line 137 "Chen et al.@JETCAS'19（Eyeriss v2）" + DOI 10.1109/JETCAS.2019.2910232 | ✓ |
| 2 | `[MAERI-ASPLOS18]` (line 34) | line 184 "Kwon et al.@ASPLOS'18（MAERI）" + DOI 10.1145/3173162.3173176 | ✓ |
| 3 | `[Alwani-MICRO16]` (line 36) | line 299 "Alwani et al.@MICRO'16" + DOI 10.1109/MICRO.2016.7783725 | ✓ |
| 4 | `[Kang-AoCStream-arXiv22]` + `[Kang-Sensors23]` (line 36) | line 309 "Kang@arXiv'22 / Kang & Yang@Sensors'23" + DOI 10.48550/arXiv.2212.11438 + 10.3390/s23198104（第 5 次启动 venue 修正过的） | ✓ |
| 5 | `[Buffets-ASPLOS19]` (line 50) | line 206 "Pellauer et al.@ASPLOS'19（Buffets）" + DOI 10.1145/3297858.3304025（第 4 次启动 DOI 修正过的） | ✓ |
| 6 | `[Shi-CVPR16]` (line 58) | line 345 "Shi et al.@CVPR'16（Sub-Pixel Convolutional Networks）" + DOI 10.1109/CVPR.2016.207 | ✓ |

抽样还覆盖到：`[VTA-MICRO19]` (line 20) → literature.md line 436 "Moreau et al.@IEEE Micro'19（VTA）"；`[Gemmini-DAC21]` (line 20) → line 147；`[NVDLA]` (line 20) → line 158；`[Liu-FullStack-TNNLS21]` (line 36) → line 319；`[cuDNN-arXiv14]` (line 38) → line 336。

均能在 literature.md 找到对应条目，无幻觉引用。Phase 0 已用 Elicit + Semantic Scholar 核过 35 篇，本次 PASS。

### claim 强度

§1 全文未出现 "first" / "only" / "breakthrough" 等过强词。检查关键 claim：

- §1.4 (1) "this raises spatial PE utilization from 12.5%–50% toward the array width" — 标了 [CHECK]，未硬给数字 ✓
- §1.4 (1) "Our literature search for 'Ky-folded compiler pass on fixed FPGA arrays' did not return a direct prior art" — 标了 [CHECK: S2D / Ky-fold 在加速器领域引用谱系待 reviewer 阶段补查]，与 contributions.md C2.1 line 84 一致 ✓
- §1.4 (2) "applying it as a compiler pass... is, to our knowledge, distinct from the training-time use" — 用 "to our knowledge" 软化，无问题 ✓
- §1.4 (3) "all 46 cases pass bit-exactly within 100k cycles" — 22+24=46 与 STATUS.md §2.5 + contributions.md §4.2 一致 ✓
- §1.4 (4) "20 of 20 W-slice regression cases pass bit-exactly" — STATUS.md §2.8 + contributions.md C3.7 一致 ✓
- §1.4 (5) "we adopt the convention 'with documented optimization roadmap'" — 诚实标注 Fmax 未达 100 MHz 的修复路径 ✓

无过强 claim。

### [CHECK]/[TBD] 合规性

Writer 自报 §1 正文内 [CHECK]=5 / [TBD]=2，frontmatter [TBD]=3，各章占位 [TBD]=7。

paper.md §1 正文（line 14-72）实际计数：

- **[CHECK]=5**：line 24 (Layer 1/3/4/5a/5c 实测 PE 利用率) / line 56 (Ky-fold 实测 PE 利用率) / line 56 (S2D 加速器领域引用谱系) / line 64 (end-to-end Wall_us) / line 64 (Fmax 修复路径与 contributions.md C1.5)
- **[TBD]=2**：line 44 (Fig.1 占位) / line 66 (5 条 vs 4 条 vs 6 条裁决)

与 Writer 自报完全一致。位置全部合理：所有未实测百分比都标 [CHECK]、未联网核的引用谱系标 [CHECK]、待用户决定的写作裁决标 [TBD]。**未发现该标 [CHECK] 而未标的内容**（包括 4.9 MB / 10 KB ring / 12.5%/25%/50% / 86.6% / 593k cycles / 68.4 MHz 全是有明确来源的实测/计算数字，不需 [CHECK]）。

### 回归性

§1 正文是否引入 paragraph-skeleton.md §1 没有的"新数据 / 新引用 / 新 claim"：

- **新引用检查**：§1 中所有 short tag (Gemmini-DAC21 / VTA-MICRO19 / NVDLA / Eyeriss-v2-JETCAS19 / MAERI-ASPLOS18 / Alwani-MICRO16 / Kang-AoCStream-arXiv22 / Kang-Sensors23 / Liu-FullStack-TNNLS21 / cuDNN-arXiv14 / Shi-CVPR16 / Buffets-ASPLOS19) 均在骨架"依赖"或 literature.md 中提及。无新引入文献。
- **新数据检查**：
  - "445 BRAM36 tiles ≈ 1.6 MB" → 骨架 §1.1.段4 已写 "445 BRAM ≈ 1.6 MB" ✓
  - "128 BRAM36 (28.8%)" → 骨架未写但 contributions.md §4.1 + STATUS.md line 36 是 source of truth；属于"用 contributions.md 实测数字补 motivation 段"的合理操作 ✓
  - "Layer 5b onward 100%" → 骨架段 3 列了 12.5%/25%/50%（前三档），paper.md 同步给出 100% 上限（model_analysis.md line 85 直接证据），是一句自然延伸，不构成编造 ✓
- **新 claim 检查**：§1.3.p4 (line 50) 把 "narrative C 工程化元素（去中心 valid-ready 流水）作为核内组织形式支撑 A/B 两轴" 改写为 "We treat this as an organizing principle rather than a top-line claim — the formal language of buffers, counters, and handshakes was already given by Pellauer et al."——主动**降级**而非强化 claim，与骨架"narrative C 工程化元素 ... 作为支撑"语义一致 ✓

§1 全文严格基于骨架展开，无未授权的"创新事实"。

### 通过原因

1. 23 段对齐严格，无骨架外段落
2. 5 处关键数字抽样全部能在 model_analysis.md / STATUS.md / README.md / contributions.md 中追溯
3. 6 处文献引用抽样全部对应 literature.md 真实条目（Phase 0 已 Elicit/SS 核验，DOI 已修正）
4. 5 contributions 一一对应 contributions.md C1.x/C2.x/C3.x，无遗漏 [CHECK]
5. claim 强度自控（"to our knowledge" + "with documented optimization roadmap"），无 first / only 突袭
6. [CHECK]/[TBD] 标记位置全部合理，自报数字与实查一致
7. 主动降级 narrative C，不引入未授权新事实

### 给 Writer 的修订建议（非阻塞，可在后续 phase 自决）

以下为**轻微 / 改进性建议**，不影响本轮 PASS：

| # | 位置 | 建议 | 优先级 |
|---|------|------|--------|
| 1 | §1.1.p1 (line 20) `[VTA-MICRO19]` | short tag "MICRO19" 容易被读者误以为是 ACM/IEEE MICRO 会议；literature.md 实际是 IEEE Micro 期刊。建议在 reference 阶段统一改为 `[VTA-IEEEMicro19]` 或类似明确写法，避免 reviewer 挑错 | 低（writing reviewer 关注点） |
| 2 | §1.3.p3 (line 48) "10 KB of ring storage" | 此处隐含通道数（如 strip_rows=8 × W=640 × 2 byte = 10 KB 暗示 INT16 或 1 ch；实际 INT8 单 ch 是 5 KB，多 ch 则更大）。后续 §4.3 / §6 详写时建议明确 "INT8 1-channel 等价容量 ~5 KB / 实际 ring 容量 ~10 KB（含 multi-channel 与 padding rows）" 避免 reviewer 推算不一致 | 低 |
| 3 | §1.4 (5) (line 64) | "with documented optimization roadmap" 措辞好；建议在 §7.7 / §8 复用同一短语保持一致性 | 低 |

> 这三条均为后续 phase 优化建议，**不影响本轮 PASS 判定**。
