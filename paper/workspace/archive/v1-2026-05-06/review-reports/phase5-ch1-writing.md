### 判定
PASS

### 评审范围
仅 §1（line 1-110 of paper.md，共 §1.1 / §1.2 / §1.3 / §1.4 / §1.5 五节，约 23 段 / 2950 词）。frontmatter 的 [TBD] 与正文 [CHECK] 标记按约定不计入 writing 缺陷。§2-§8 占位不评。

### 段落长度合规（8 段抽样）

| 抽样段 | 估算词数 | 判定 | 备注 |
|---|---|---|---|
| §1.1 P1 (line 20) 开篇 fixed-array 谱系 | ~140 词 | OK | 略偏上限但内容承担"开篇 + 双约束 framing"双任务 |
| §1.1 P3 (line 24) 量化 breakdown | ~95 词 | OK | 数字密集但分号节奏合理 |
| §1.1 P5 (line 28) 节内收束 | ~50 词 | OK | 短段做 roadmap 收束合理 |
| §1.2 P3 (line 36) layer-pipelined 路线 | ~125 词 | OK | 承载 Alwani+Kang+Liu 三引用，密度合理 |
| §1.3 P1 (line 44) approach 总述 | ~115 词 | 边界 OK | 含一处 60+ 词长句（见后段评） |
| §1.4 P3 (line 58) S2D 贡献 | ~115 词 | OK | 含 prior art 区分论证 |
| §1.4 P6 (line 64) PyTorch 端到端 | ~150 词 | 偏长 OK | 因 Fmax 长括号说明吃了字数，但仍 ≤200 |
| §1.5 P2 (line 72) 自包含说明 | ~70 词 | OK | 收束节短段合理 |

**结论**：23 段全部落在 50-200 词目标区间内，无 >300 词超长段，无 <30 词散段堆叠。

### 段落主题句结构

主题句基本都在段首或第二句，且每段 claim 单一，未发现一段塞多主题：

- §1.1 P2 "The first constraint is that, on ResNet-18-style networks..." — claim 直接置首，后续句为 evidence
- §1.1 P4 "The second constraint is on-chip memory." — 极简主题句开头，标准学术结构
- §1.2 P2/P3/P4 三段以 "The first family / The second family / The third family" 开头，主题平行结构清晰
- §1.3 三段分别以 "The core design choice of FLUX_CNN..." / "Narrative A — compiler-side PE utilization — is realized by..." / "Narrative B — system-side support — is realized by..." 开头，承担"总-分-分"三段式
- §1.4 (1)-(5) 每条 contribution 都有粗体小标题作为视觉主题句，正文首句紧扣

唯一弱主题句：**§1.4 P1**（line 54）作为 contributions 节引言，主题分布在两句话——前句讲 narrative 组织、后句讲 quantification policy，可读但稍混。轻微，不构成失败项。

### 段间承接（4 个边界）

| 边界 | 衔接句 | 判定 |
|---|---|---|
| §1.1 → §1.2 | §1.1 末段："The remainder of this section reviews how prior approaches address each constraint individually (§1.2)..." | 显式列出 §1.2-§1.5 路线图，承接清晰 |
| §1.2 → §1.3 | §1.2 末段："FLUX_CNN therefore selects a fourth path. ...The next section makes this approach concrete." | "fourth path" 巧妙呼应前文三个 family，过渡自然 |
| §1.3 → §1.4 | §1.3 末段："The next section enumerates the resulting concrete contributions." | 一句话直白桥接，可接受（虽朴素） |
| §1.4 → §1.5 | §1.4 末段："These five contributions are detailed in §4–§7." + "alternative groupings (e.g., merging (1) and (2)...) are equally defensible [TBD]" | 转入 §1.5 时无显式衔接句，但 §1.5 自身首句"The remainder of this paper..."自然承接 |

四个边界均承接自然，无断层。

### 学术英文腔调

- **人称 "we"**：全文 8 次，集中在 §1.4 contributions（"We introduce..." × 2、"We split..." × 1、"We adopt..." × 1）+ §1.3 narrative C（"We treat this as..."）+ §1.4 引言/收束（"We deliberately..." × 2、"We note that..."）。频率合理，无滥用。
- **Buzzword 检查**：grep `revolutionary|unprecedented|breakthrough|novel|first to|only system|state-of-the-art` 全文 0 命中。Writer 自报"未用 CNN have revolutionized cliché"属实。
- **Claim 强度**：
  - §1.4 (1) "Our literature search... did not return a direct prior art [CHECK: ...谱系待 reviewer 阶段补查]" —— 双重 hedge，工整
  - §1.4 (2) "is, to our knowledge, distinct from the training-time use in the original work" —— 标准学术 hedge，未 over-claim
  - §1.4 (5) "with documented optimization roadmap" —— 主动承认未修复 critical path，反向 hedge 防御得当
  - §1.3 P3 narrative C "We treat this as an organizing principle rather than a top-line claim" —— 明示降权，避免 over-claim
- **时态**：方法描述 "We introduce" / "FLUX_CNN selects" / "Hardware is kept simple" 一致用一般现在时；引用既有工作 "Alwani et al.'s ... keeps inter-layer feature maps" 用一般现在时（描述 design fact），未与方法句冲突。无时态混乱。

### 句长与节奏

整体长短句搭配良好。**唯一边界长句**：

- **§1.3 P1**（line 44）"The core design choice of FLUX_CNN is to keep a fixed 16×16 INT8 array, fold shallow-layer convolutions onto that array entirely in software via Ky-fold and Space-to-Depth (S2D) compiler passes, and stream feature maps through the pipeline in a row-ring buffer driven by bidirectional row-credit backpressure, so that a single host-issued `start` runs an arbitrary H×W image to completion." —— 约 65 词，但是 "to keep A, fold B, and stream C" 三段并列 + 一个目的状语，结构对称清楚，**未达"60+ 词从句套娃读不下去"程度**。轻微建议：可拆为"The core design choice of FLUX_CNN is threefold: ..." + 三句短列举，但当前形式可接受。

§1.1 P1 也接近 50 词长句但同样为列举对称结构，节奏不卡。其他段无 60+ 词单句问题。

### 术语一致性

抽查全节关键术语：

| 术语 | 首次出现 | 缩写形式 | 一致性 |
|---|---|---|---|
| PE (Processing Element) | §1.1 P2 | "PE utilization" 直接用 | OK，未与 "MAC unit" 混用（"MAC" 用于 "MAC array" / "useful MACs per cycle" 是 multiply-accumulate 操作语义，非 PE 别名） |
| Ky-fold | §1.3 P2 | 无展开但已是常用记号 | OK，全节统一 |
| Space-to-Depth (S2D) | §1.3 P2 首次出现处展开"Space-to-Depth (S2D) compiler passes" | S2D | OK，规范 |
| row-ring | §1.3 P1 | 无另名 | OK；偶有"ring storage / row-level streaming"作描述性变体，语义无歧义 |
| narrative A / B / C | §1.1 P2 / §1.2 P1 / §1.3 P3 | 每次重提附带说明 | OK |
| line buffer / line_buffer | §1.3 P1 用 `line_buffer`（代码字体）作模块名 | §1.2 用 "line-buffer granularity" 描述模式 | 区分得当（代码记号 vs 概念词），无冲突 |
| BRAM / BRAM36 | §1.1 P4 "445 BRAM36 tiles" / "128 BRAM36" / "BRAM blow-up" / "BRAM budget" | 一致 | OK |
| SDP | §1.4 (5) "SDP quantization combinational chain" | 首次出现处未展开 | 轻微 — 见下方建议 |

### §1.1 开篇 cliché 检查

§1.1 P1 开头："Fixed two-dimensional MAC arrays — typically organized as a 16×16 INT8 grid on mid-range edge FPGAs — have become the dominant building block for embedded CNN inference accelerators, including open-source designs such as Gemmini [Gemmini-DAC21] and VTA [VTA-MICRO19], as well as industrial reference architectures such as NVDLA [NVDLA]."

检查项：
- 未用 "CNN have revolutionized..." / "Deep learning has transformed..." 类俗套 ✓
- 未用 "In recent years..." 时间套话 ✓
- 直接进入"fixed-array 加速器谱系定位"+ 三个具名 prior art（Gemmini / VTA / NVDLA）✓
- 立即收束到本工作的双约束 framing ✓

**判定：cliché 规避到位，开篇直进谱系定位**。"have become the dominant building block" 是事实性陈述（且后接具体引用佐证），非空泛宣称。

### §1.4 contributions 列表可读性

- **编号 / 排版**：(1)-(5) 阿拉伯数字编号 + **粗体小标题** + 段首正文，格式工整一致
- **句式开头多样化**：
  - (1) "We introduce a compiler transformation..."
  - (2) "We introduce a second compiler transformation..."
  - (3) "The IFB, weight, and OFB buffers cooperate..."
  - (4) "We split the W dimension..."
  - (5) "A small Python compiler maps..."
  
  5 条不全用 "We propose..." 开头，(2) 与 (1) 句式接近（都以 "We introduce..." 开头）但加了 "second" 区分，可接受。(3) 改用主语为硬件子系统、(5) 改用主语为编译器，节奏被打破得恰到好处。
- **每条均带 quantified evidence**：12.5%-50%→近 100%、~10 KB ring、N=4、20/20 cases、46/46 cases、86.6% MAC%、68.4 MHz Fmax 等，遵循引言段宣告的"each claim quantified"
- **末段（line 66）"the count of contributions is itself a writing decision... [TBD: 5 vs 4 vs 6]"**：自我指涉，略显 meta，但因 [TBD] 不计入 writing 缺陷且明确说明是占位，可接受

### 通过原因

1. **段落长度全合规**：23 段全部落 50-200 词区间，无超长 / 超短堆叠
2. **段落主题句聚焦**：每段单一主题，主题句普遍位于段首或第二句
3. **段间衔接自然**：4 个小节边界均显式承接，"fourth path"、"The remainder..."等桥段地道
4. **学术腔调到位**：0 buzzword，hedge 工整（"to our knowledge" / "with documented roadmap" / "did not return a direct prior art"）
5. **§1.1 cliché 规避成功**：直接进入 fixed-array 谱系定位，伴具名 prior art 锚点
6. **§1.4 contributions 排版工整**：粗体编号 + 句式多样 + 每条带量化证据
7. **术语全节统一**：Ky-fold / S2D / PE / row-ring / narrative A/B/C 无别名混用
8. **时态统一**：方法句一致 PR、引文事实陈述 PR，无 PR/PT 来回切换

### 给 Writer 的修订建议（轻微，不影响 PASS）

以下三处属"打磨级"建议，Writer 可选择采纳或忽略：

1. **§1.4 (5) "SDP" 首次出现处建议补全展开**（line 64）。当前 "SDP quantization combinational chain" 中 SDP 在 §1 内是首次出现。虽然 §4.4 / §6 占位提到 SDP，但 §1 自身应自包含。建议改为 "the Single Data Processor (SDP) quantization combinational chain" 或 "the SDP (single-data-processor post-quantization) combinational chain"。

2. **§1.3 P1 长句拆分（可选）**（line 44）。当前 65 词列举式长句虽对称可读，但若想进一步降低审稿人阅读阻力，可改为 "The core design of FLUX_CNN rests on three choices: (i) keep a fixed 16×16 INT8 array; (ii) fold shallow-layer convolutions onto that array via Ky-fold and Space-to-Depth (S2D) compiler passes; (iii) stream feature maps through a row-ring buffer driven by bidirectional row-credit backpressure. A single host-issued `start` then runs an arbitrary H×W image to completion."

3. **§1.4 末段（line 66）meta 语句考虑迁移到注释**。"the count of contributions is itself a writing decision — alternative groupings... are equally defensible [TBD]" 当前是正文段，正式发表时通常会移除。建议在 [TBD] 决议后改为 HTML 注释 `<!-- ... -->` 形式而非正文段。

### 提示其他 reviewer

- **§1.4 contributions 中 "12.5%–50% toward the array width [CHECK]"、"86.6% network-level MAC utilization in 593k cycles [CHECK]"、"68.4 MHz" 等具体数字** 已带 [CHECK] 标记，但建议 tech reviewer 复核 593k cycles / 86.6% / 68.4 MHz / 10 KB / 4.9 MB / 1.6 MB / 445 BRAM36 等数字与 contributions.md / STATUS.md / model_analysis.md 是否一致。
- **§1.4 (1) "Our literature search ... did not return a direct prior art" 与 (2) "to our knowledge, distinct from the training-time use in the original work"** 涉及 novelty claim 防御，建议 novelty reviewer 复核谱系防御充分性。

writing 评审仅就语言 / 结构判定 PASS，不审上述事实 / 新颖性。
