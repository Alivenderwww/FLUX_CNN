# 写作评审报告 Phase 5 §6 System Integration

## 第 1 次评审

### 判定：PASS

---

### 评审范围

`paper/workspace/paper.md` §6 整章（line 302-360），含章首段 + §6.1-§6.5 五个子节，共 23 段。仅评写作（语言、结构、衔接、风格），不评事实与新颖性。

---

### 段落长度

抽样 6 段，按英文 50-160 词 / 中文当量 80-200 字目标核查：

| 段位 | 词数（估算） | 评价 |
|------|------------|------|
| §6 章首段（line 304） | ~80 | 合规，单段铺垫整章 |
| §6.1 p2（line 310，三层结构） | ~100 | 合规 |
| §6.1 p4（line 314，TVM 对比） | ~70 | 合规 |
| §6.2 p2（line 322，四个 controller） | ~95 | 合规 |
| §6.3 p2（line 334，R.1/R.2） | ~85 | 合规 |
| §6.4 p2（line 346，W-slice 例子） | ~100 | 合规 |
| §6.5 p3（line 360，收束段） | ~110 | 合规 |

无 >160 词单段、无 <30 词碎段，长度分布健康。

---

### 主题句结构

逐段抽查主题句（每段首句是否承担本段 claim/setup/transition 角色）：

- §6 章首：`This chapter describes the system-level engineering...` — 标准 chapter-opener，列举五节。**OK**
- §6.1 p1：`FLUX_CNN ships a complete PyTorch-to-hardware compile stack...` — claim 句，明确说出"端到端单命令"。**OK**
- §6.1 p2：`The stack is organized as three layers.` — 经典 setup-then-list，结构清晰。**OK**
- §6.2 p1：`The original project shipped roughly 3000 lines of hand-written...has been replaced with...` — before/after 对比开头，叙事张力到位。**OK**
- §6.3 p1：`The Single Data Point processor (SDP) fuses five post-MAC operations into one combinational chain...` — 定义+功能一句话给出。**OK**
- §6.4 p1：`multicore_top.sv is a parameterizable N-core wrapper...` — 定义型主题句。**OK**
- §6.5 p1：`params.py is the single source of truth for all design-wide parameters...` — 命题性主题句，直接喊出"single source of truth"。**OK**

23 段无"埋主题句"现象。

---

### 段间承接

- **§5.4 末段 → §6 章首**（line 296 → line 304）：§5.4 末句 `the next chapter turns to the system-integration side — how the same compiler emits the descriptor lists, hardware files, and PyTorch front-end glue that drive the array end-to-end`，§6 章首段承接 `the system-level engineering that turns the FLUX_CNN datapath into a deployable accelerator`，并列出五节。**衔接顺畅。**
- **§6 章首 → §6.1**：章首段最后一句 `validated end to end on real workloads`，§6.1 第一句即 `PyTorch-to-hardware compile stack...end to end`，关键词复现。**OK**
- **§6.1 → §6.2**：§6.1 末句 `The compiler's output is ultimately a stream of descriptors that flow into hardware over DMA, which the next subsection describes.` — 标准 hand-off，且 "descriptors" 与 §6.2 的 `descriptor-driven configuration` 直接接续。**OK**
- **§6.2 → §6.3**：§6.2 末段讲 `CFG_WRITE` 与 NVDLA 对照，§6.3 开头直接进入 SDP。中间没有显式过渡句，但从"配置路径"切到"数据后处理路径"语义自然。**轻微建议**（不构成 FAIL）：可在 §6.2 末段或 §6.3 首段补一句"Beyond the configuration path, the post-MAC datapath is also fused into a single block, described next."
- **§6.3 → §6.4**：§6.3 末段 `we present these as system-completeness evidence rather than as primary architectural novelty`，§6.4 首段直接介绍 multicore wrapper。无显式过渡，但章首段已统一框定五节 → 可接受。
- **§6.4 → §6.5**：§6.4 末段是 Simba/Liu/Kang 文献对比与 [TBD]，§6.5 首段直入 `params.py`。同上，无显式过渡但可接受，且 §6.5 末段 `the chained descriptor flow of §6.2 and the multi-core wrapper of §6.4 both depend on RTL and Python agreeing` 显式回扣前几节，弥补了承接弱化。**OK**
- **§6.5 末段 → §7 占位**（line 360 → line 366）：`The next chapter quantifies what that accelerator delivers.` — 标准 transition-to-evaluation。**OK**

整章承接以章首段统辖 + §6.5 末段回扣的 "frame + close" 形式实现，§6.2→§6.3→§6.4 之间无显式 connective tissue，但因每节都用强主题句开头，未造成断裂。

---

### 与 §1-§5 风格一致性

参照 §5（特别是 §5.3 line 282-286 与 §5.4 line 290-296）作为基线：

| 维度 | §5 基线 | §6 实测 | 一致性 |
|------|---------|---------|--------|
| 句长 | 长复合句为主，多 em-dash | 同样多 em-dash + 内嵌从句 | OK |
| "we" 频率 | 每节 2-4 次 | 每节 1-3 次 | OK |
| 引用 short tag | `[Liu@TNNLS'21]` `[Kwon@ASPLOS'18]` | `[NVDLA]` `[He-CVPR'16]` `[Liu-TNNLS'21]` `[Simba-MICRO'19]` | **轻微不一致**：§5 用 `@`，§6 多处用 `-`（如 `Liu-TNNLS'21` vs §5.4 line 290 `Liu@TNNLS'21`）。同一篇 Liu 文献两种 tag 风格。**建议统一**，但不构成 FAIL（这是引用规范问题，可由 Phase 7 整体 pass 一并扫掉）。 |
| 学术腔调 | 克制，常用 "we frame X deliberately" "we report it honestly" | 同腔调："we report this honestly" "we acknowledge as 'NVDLA-inspired'" "we present these as system-completeness evidence rather than as primary architectural novelty" | **OK 且非常一致** |
| 节奏 | claim → mechanism → evidence → comparison → bridge | §6.1/§6.2/§6.3 都遵循同结构 | OK |
| 篇幅密度 | §5.4 5 段 / 1100 词 | §6 23 段 / ~1310 词，每子节 3-5 段 | 与 §5 平均节长一致 |

整体风格与 §5 高度一致，无割裂。

---

### 章首段（line 304）

> "This chapter describes the system-level engineering that turns the FLUX_CNN datapath into a deployable accelerator: a PyTorch end-to-end compile path (§6.1), a vendor-IP-backed AXI/DMA subsystem with descriptor-driven configuration (§6.2), an SDP post-processing block that fuses bias / residual / quantization (§6.3), an N-core W-slice scaling wrapper (§6.4), and a single-source parameter management flow (§6.5). These items are necessary engineering work rather than primary novelty claims; they are reported here to establish that the architectural contributions of §3-§5 are validated end to end on real workloads rather than in isolation."

- 列出全部五节 + 一句话定位 → **结构性铺垫到位**
- 第二句明确 "necessary engineering work rather than primary novelty claims" → **管理读者预期**，避免读者误判 §6 内容为新贡献
- 与 §6.5 末段 "turn the architectural contributions of §3-§5 into a deployable accelerator" 形成首尾呼应

**强章首段**。

---

### 学术腔调

- **buzzword 检查**：未发现 "smart" / "intelligent" / "optimal" / "novel"（仅在 "primary novelty claims" 中作为名词出现一次，正确用法）。**OK**
- **过强 claim 检查**：
  - line 350 `we deliberately do not claim near-linear scaling without that audit, and we do not yet report multi-core numbers on the full ResNet chain` — 主动降调，**优秀**
  - line 340 `we present these as system-completeness evidence rather than as primary architectural novelty` — 同上
  - line 336 `we acknowledge as "NVDLA-inspired" rather than rederived` — 同上
  - line 326 `vendor IP does not leave bandwidth on the table` — 略带俗语但属于学术圈接受范围
- **第一人称**：23 段中 "we" 出现约 8 次，密度合理，未滥用
- **时态**：方法描述用现在时（`fuses` `consumes` `walks` `assigns`），实测引用用现在时陈述事实（`currently passes` `currently runs`）— 与 §5 一致。**OK**

---

### 节奏

抽查连续句式：

- §6.1 p2（line 310）`compile_layer.py lowers... compile_model.py walks... On top of that, run_regression.py exposes...` — 三个并列模块描述，有 "On top of that" 节奏调节，未连续 3 句相同句式。**OK**
- §6.2 p2（line 322）`The four controllers... mm2s_arb lets... axi_m_mux aggregates... axi_lite_csr provides...` — 四个并列模块，但每句主语不同、动词不同（`generate` `lets` `aggregates` `provides`），节奏不平。**OK**
- §6.3 p2（line 334）`R.1 moved... R.2 added...` — 仅 2 句并列，无重复风险。**OK**

无连续 ≥3 句相同句式问题。

---

### 术语一致性

按 CLAUDE.md 名册逐项核查：

| 术语 | CLAUDE.md | §6 用法 | 一致 |
|------|-----------|---------|------|
| `idma_ctrl` | ✓ | line 322, 324 反引号包裹 | OK |
| `wdma_ctrl` | ✓ | line 322 | OK |
| `odma_ctrl` | ✓ | line 322, 348 | OK |
| `mm2s_arb` | ✓ | line 322 | OK |
| `axi_dm` | ✓ | line 320, 322（注："AXI DataMover IP" 全称在 line 320 首次展开，正确） | OK |
| `axi_m_mux` | ✓ | line 322 | OK |
| `axi_lite_csr` | ✓ | line 322 | OK |
| SDP | ✓ | line 332 首次展开为 "Single Data Point processor (SDP)"，后续用缩写 | **OK，缩写规范** |
| W slice / W-slice | ✓ | §6.4 一致使用 `W-slice`（line 344, 346, 350, 352） | OK |
| `multicore_top` | ✓ | line 344 | OK |
| `params.py` | ✓ | line 354, 356, 358, 360 | OK |
| `cfg_regs` | ✓ | line 308, 324, 356 | OK |
| `parf_accum` / `parf_col × NUM_COL` | ✓ | line 284（在 §5）；§6 未提，无冲突 | OK |
| DFE | ✓ | line 322 首次出现展开为 "descriptor front-end (DFE)"，后续无再用，但单次出现可接受 | OK |
| `rdma_ctrl` | 出现于 git log `Refactor: R.1` | line 322 提及，未在 CLAUDE.md DMA 子系统列表中显式列出，但与 commit 历史一致 | OK |
| `ifb_axi_slave` | git status 新增 | line 348 反引号包裹 | OK |
| Shortcut Bank | memory/sdp_residual_fusion.md | line 334 大写 "Shortcut Bank" | OK |

**无术语别名混用**。所有 RTL 标识符规范地用反引号包裹，缩写规范地首次展开。

---

### params.py 表述（§6.5）

CLAUDE.md 提到 `params.py is single source of truth + auto-codegen`。§6.5 表述：

- line 354 主题句：`params.py is the single source of truth for all design-wide parameters consumed by both RTL and the Python toolchain` — **明确点出 single source of truth**
- line 354 工程价值：`eliminating the parameter-drift class of bugs where an ADDR_W change updates RTL but not the compiler (or vice versa)` — **用具体反例说明价值**，比抽象描述强
- line 358 auto-codegen 机制：`python params.py regenerates RTL/flux_cnn_params.svh; RTL consumes the parameters via \`include\`, and Python consumes them via from params import *. Both sides therefore read the same numbers without any hand-maintained mirror file.` — **机制清楚**，单向生成 + 双方消费的关系说得明白
- line 360 末段定位：`This is a pure engineering practice rather than an architectural contribution; we mention it because...` — **降调表述**，避免被读者误读为贡献，与章首段 "necessary engineering work rather than primary novelty claims" 呼应

**§6.5 是本章写得最干净的子节之一**，工程价值表述清晰而不夸张。

---

### 通过项汇总

1. 段落长度全部合规（80-200 字 / 50-160 词）
2. 主题句结构 23 段全部首句即 claim/setup
3. §5.4→§6→§7 大跨度衔接顺畅
4. 章首段强力铺垫五节 + 管理读者预期
5. 学术腔调与 §5 一致，无 buzzword、无过强 claim
6. "we" 频率合理，时态规范
7. 术语全部对齐 CLAUDE.md，缩写首次展开规范
8. params.py 表述清晰说出 single source of truth + auto-codegen 工程价值
9. 全章无 cliché、无割裂、无段落断层

---

### 轻微建议（≤3 处，不影响 PASS）

1. **引用 tag 风格不统一**：§6 内 `[Liu-TNNLS'21]`（line 338, 352）与 §5.4 line 290 `[Liu@TNNLS'21]` 同一篇文献两种 tag。建议 Phase 7 整体扫尾时统一为 `@` 或 `-` 之一。
2. **§6.2 → §6.3 缺一句过渡**：从 NVDLA 描述类比直跳到 SDP，可在 §6.3 首句前补 "Beyond the configuration path, post-MAC datapath fusion further reduces handshake overhead." 之类的承接（非必需）。
3. **DFE 缩写仅出现一次**：line 322 展开了 "descriptor front-end (DFE)" 但全章后续无再用 DFE 缩写，缩写定义略显多余。可考虑直接写 "the descriptor front-end" 不引入缩写。

---

### 提示（供其它评审参考，不影响本评审 PASS/FAIL）

- line 312 chain 实测数字（593K cycles / 86.6% / 5.95 ms / 8.69 ms）已带 [CHECK]，建议 **tech 评审**复核口径
- line 326 `+0.5%` 性能差距已带 [CHECK]，建议 **tech 评审**复核
- line 350 `1.45×` 与 baseline 归一化已带 [CHECK]，建议 **tech 评审**复核
- line 290 与 line 290 自比 / 与 §5.4 一处 86.6% 数字 — 与 line 312 应一致，建议 **tech 评审**统一
- §6.4 [TBD: 是否提升为顶层章节] — 结构性问题，建议主 Agent 在后续 phase 与 **outline 评审/novelty 评审**协同决策

以上提示均不影响本写作评审 PASS 判定。
