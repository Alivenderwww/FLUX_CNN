### 判定
PASS

### 评审范围
仅 §4 Architecture（line 186-242）。共 5 节 + 1 章首 framing 段，合计 22 段。未审 §1/§2/§3（已 PASS）/ §5-§8（占位）。

### 段落长度合规（6 段抽样）

抽样：§4 章首 framing（line 188）/ §4.1 p1（line 192）/ §4.2 p2 elastic-join（line 204）/ §4.3 p1 forward-pressure（line 214）/ §4.4 p3 NVDLA contrast（line 230）/ §4.5 p1（line 236）。

| 段 | 词数估计 | 评估 |
|---|---|---|
| 章首 framing | ~75 词 | 合规（用户自报 80 词，实测略低，但不影响） |
| §4.1 p1 | ~65 词 | 合规 |
| §4.2 elastic-join | ~70 词 | 合规 |
| §4.3 forward-pressure | ~110 词 | 合规 |
| §4.4 NVDLA contrast | ~70 词 | 合规 |
| §4.5 p1 | ~75 词 | 合规 |

全章无 >160 词的超长段，无 <30 词的碎屑段。最长段是 §4.3 p1（约 110 词，含数学公式 `rows_available ≥ y_out · stride + Ky`），仍在合规区间。

### 段落主题句结构

每段以聚焦的主题句开头：
- §4.1 p1："organised as two layers..." 直接给出顶层切分。
- §4.2 p1："each maintain their own counters... no central FSM"——主题句即 C1.1 命题。
- §4.3 p1："physically organised as row-level ring buffers... interlocks at row granularity"——主题句即 C1.2 命题。
- §4.4 p1："not a single monolithic PSUM SRAM; instead..."——以否定句强主题，结构清晰。
- §4.5 p1："handles arbitrary Cin and Cout through two layers of slicing"——主题句即 C2.4 命题。

无埋藏主题句的情况。

### 段间承接

- §3.6 末段（"§4 develops the architecture (axis 1) in detail"）→ §4 章首段（"develops the architecture along axis (1) of the five-axis frame established in §3.5"）：明确回扣 axis (1) 锚点，承接干净。
- §4.1 末段 → §4.2 首段："The next subsection first treats the inter-module handshake that underpins the entire pipeline." → "The five core modules each maintain their own counters..." 承接顺。
- §4.2 末段 → §4.3 首段："without elastic backpressure at every boundary, the row-ring would not be able to stall cleanly..." → "The IFB and OFB are physically organised as row-level ring buffers..." 强逻辑衔接（"backpressure→row-ring 是它的下游应用"）。
- §4.3 末段 → §4.4 首段："The accumulator unit that sits inside this ring deserves its own subsection..." → "parf_accum is not a single monolithic PSUM SRAM..." 承接顺。
- §4.4 末段 → §4.5 首段：§4.4 末段以"unit-level implementation choice... reported here for completeness"收尾，§4.5 首段直接进入 loop-nest，承接稍弱（无显式过渡句），但在可接受范围——§4.4 是 unit-level dive，§4.5 是 nest-level resurfacing，章节级跨度合理。
- §4.5 末段 → §5 占位："The next chapter develops the compiler side, where Ky-fold and S2D are shown to be transformations on the inputs to this same nest rather than new modes of the nest itself." 显式铺垫 §5，过渡到位。

### 与 §1+§2+§3 风格一致性

- **句长 / 节奏**：§4 与 §3 节奏接近，多用中长句 + 偶现长复合句（如 §4.2 末段的 Gemmini/NVDLA/VTA 三引）。比 §1 略短（§1 偶有 60+ 词单句，§4 控制在 50 词以内），但仍属同一调性。
- **"we" 频率**：§4 共 6 处 we（"we have deliberately... / we adopt this taxonomy / we do not claim... / we frame... / we classify... / we claim"），均为方法论立场表述，与 §1.4/§3 用法一致，无滥用。
- **引用格式**：`[NVDLA]` `[Gemmini-DAC21]` `[Snowflake-ISCAS17]` `[Buffets-ASPLOS19]` `[Interstellar-ASPLOS20]` `[Ma-FPGA17]` `[VTA-Micro19]` 全部沿用 §3 风格。
- **`<!-- 来自 ... -->` 来源注释**：§4 出现 4 处，与 §1-§3 频率近似。
- **学术腔调**：§4 与 §2/§3 同温层，无突兀口语。

风格一致性 PASS。

### 学术英文腔调

- 无明显 buzzword（无 "smart" / "intelligent" / "cutting-edge" / "novel approach"）。
- 第一人称"we"用于立场表述，未滥用。
- 时态一致：方法描述用现在时（"is organised as" / "addresses" / "broadcasts"），实验/历史用过去时（"R.1 refactor relocated bias..." 在 §3.5，§4 内无实验过去时矛盾）。
- 偶用 hedging（"we do not claim... we claim that..." / "we classify this as a unit-level implementation choice rather than..."），姿态克制。

### 句长与节奏

- 最长单句出现在 §4.1 p2（"the DMA subsystem comprises three lightweight controllers..."）约 50 词，三个 m-dash 切分，仍可读。
- 无 60+ 词从句套娃。
- §4.3 p1 用了一个分号 + 解释从句的复合结构，节奏稍紧但未失控。
- §4.5 末段有 "with no fold-aware paths, no reconfigurable interconnect, and no special cases for small Cin" 三排比，节奏好。

### 术语一致性

核对 CLAUDE.md / §1-§3 / 骨架术语：
- `line_buffer` / `mac_array` / `parf_accum` / `wgt_buffer` / `ofb_writer` ✅
- `idma_ctrl` / `wdma_ctrl` / `odma_ctrl` / `mm2s_arb` / `axi_dm` / `axi_m_mux` / `axi_lite_csr` ✅（与 CLAUDE.md 一致）
- `WRF` / `IFB` / `OFB` / `PARF` ✅
- `row-ring` / `streaming` / `valid-ready` ✅
- `SDP` ✅（§4.1 末出现，与 §3.5 一致）
- `parf_col` ✅
- `strip_rows` ✅
- C1.1 / C1.2 / C1.3 / C2.4 contribution 编号 ✅（与 §1.4 一致）

**轻微观察**（不影响 PASS）：
- §4.2 引用 `[VTA-Micro19]`，但 §1.2 / §3.3 使用 `[VTA-MICRO19]`（大小写不一致：Micro19 vs MICRO19）。建议全文统一为 `MICRO19`，由 polisher 阶段顺手修。
- §4.3 末段引用 `[Liu-TNNLS21]`，但 §1.2/§2.3/§3.4 使用 `[Liu-FullStack-TNNLS21]`。建议统一为后者。
- §4.3 末段引用 `[Kang-Sensors23]`，但 §1.2/§2.3/§3.4 一并使用 `[Kang-AoCStream-arXiv22, Kang-Sensors23]`。§4.3 这里引用单一变体可读性不影响，但若全文检索 ID 会少一处。

以上三点均为引用 ID 大小写 / 简化变体问题，由 Phase 7 全局 polishing 处理即可，不构成 §4 写作 FAIL。

### 章首 framing 段质量

章首 framing 段（line 188，约 75 词）做到三件事：
1. 显式锚定到 §3.5 的 axis (1)，建立"读者已经看过五轴坐标"的预设；
2. 列出 §4 要 ground 的 4 条 contribution（C1.1 / C1.2 / C1.3 / C2.4），与 §1.4 编号对齐；
3. 提前声明诚实立场——"deliberately kept the architecture conventional in places where novelty would not justify the verification cost"。

framing 段没有 over-claim，没有冗余 setup。质量好。

### narrative 一致性

- §4.2 → C1.1（decentralized valid-ready）：首段直接挂 `(C1.1)`，与 §1.4 contribution (3) 的 narrative B 衔接。
- §4.3 → C1.2（streaming row-ring）：首段直接挂 `(C1.2)`，§1.4 contribution (3) 同源。
- §4.4 → C1.3（per-column PARF）：首段挂 `(C1.3)`。
- §4.5 → C2.4（loop-nest realization）：首段挂 `(C2.4)`。

每节 narrative 钩子明确。读者从 §1.4 跳到 §4 任一节都能立即定位对应 contribution。

### 诚实立场是否到位

用户自报"硬件不太新颖"立场要在 3 处明确。逐一核对：

1. **§4 章首**："We have deliberately kept the architecture conventional in places where novelty would not justify the verification cost; the design choices that we do claim are flagged at the section level rather than buried in microarchitectural lists." ✅ 到位。
2. **§4.4 末段**："We classify this as a unit-level implementation choice rather than a top-level contribution; it is one of the engineering details that makes the rest of the pipeline cheap to clock-close, and we report it here for completeness rather than as a novelty claim." ✅ 到位。
3. **§4.5 末段**："We do not claim a new loop-nest taxonomy; we claim that this particular nest is the target surface that the compiler in §5 maps Ky-fold and S2D into." ✅ 到位。

另外 §4.2 末段也有诚实立场："Buffets [Buffets-ASPLOS19] supplies the formal `buffer + counter + handshake` vocabulary that we adopt informally; we do not claim a contribution at the level of the Buffets formalism, only that the FLUX_CNN core pipeline is a faithful instance of that pattern at the module-chain level rather than the storage-element level." 这是 §4 内第 4 处诚实立场（章节级），实质增强了立场一致性。

诚实立场表达不 over-claim 也不 under-sell：未把 row-ring / per-column PARF 写得像新发明，但也明确指出"这是 axis (1) 上的实现选择"——读者能 distinguish "engineering 选择"和"算法贡献"。

### 通过原因

1. 段落长度全部合规（80-200 字 / 50-160 词内），抽样 6 段无超长无碎屑。
2. 主题句结构清晰，每段首句即论点。
3. 段间承接顺畅，§3.6 → §4 → §5 三章衔接均有显式过渡句。
4. 与 §1-§3 风格一致，无突兀分裂。
5. 学术腔调克制，无 buzzword、无 over-claim。
6. 术语一致性总体良好，仅 3 处引用 ID 大小写/简化变体问题（全局 polishing 处理）。
7. 章首 framing 段紧凑有效，4 条 contribution 锚点明确。
8. narrative 一致性清晰，C1.1/C1.2/C1.3/C2.4 与 §1.4 锚定。
9. 诚实立场在 4 处明确表达（§4 章首 + §4.2 末 + §4.4 末 + §4.5 末），既不夸大也不贬低。

### 给 Writer 的修订建议（轻微，不影响 PASS）

1. **引用 ID 统一**（建议在 Phase 7 polisher 处理）：
   - `[VTA-Micro19]`（§4.2 末段）→ 改为 `[VTA-MICRO19]`，与 §1.2 / §3.3 对齐。
   - `[Liu-TNNLS21]`（§4.3 末段）→ 改为 `[Liu-FullStack-TNNLS21]`，与 §1.2 / §2.3 / §3.4 对齐。
   - `[Kang-Sensors23]`（§4.3 末段）→ 建议补全为 `[Kang-AoCStream-arXiv22, Kang-Sensors23]` 或在 §3.4 末已展开后此处单引可接受。

2. **§4.4 → §4.5 过渡稍弱**（可选）：§4.4 末段以"unit-level... reported for completeness"收尾后，§4.5 直接跳到 loop-nest。可在 §4.5 首段增加一句过渡（如"with the per-stage hardware described, we now turn from microarchitectural choices back to the loop-nest surface that the compiler will target"），但当前形态也可接受，由 polisher 阶段定。

3. **提示 tech 评审**：§4.3 引用 "10 KB ring storage at strip_rows = 8, W = 640"，与 §1.3 / §2.3 / §1.4 contribution (3) 数字一致；但 contribution (3) 中加了限定语 "across the active channels"，§4.3 这里写得更紧凑。建议 tech 评审复核 strip_rows = 8 × W = 640 × active_channels 的 10 KB 数字推导是否在所有引用处口径一致。本评审不审事实正确性。

---

评审结论：§4 写作质量达到 PASS 标准。结构清晰、段落均衡、术语一致、诚实立场到位、与 §1-§3 风格无缝衔接。仅 3 处引用 ID 大小写/简化问题，由 Phase 7 polisher 顺手处理即可，不构成阻塞。
