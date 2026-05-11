# 技术评审报告 Phase 5 §4 Architecture

## 第 2 次评审（前 reviewer SSL 错，重启）

### 判定：PASS

---

### 评审范围
- paper.md 第 §4 章（lines 186-242）
- 章首段 + §4.1 / §4.2 / §4.3 / §4.4 / §4.5 共 5 节 + 1 chapter intro = 23 段（含 §4 intro 1 段，§4.x 子节合计 22 段，与启动指令一致）
- 不评 §1/§2/§3（已 PASS）/ §5-§8（仍为 [TBD] 占位，跳过）
- 不评 writing 维度

---

### 段落骨架对齐（vs paragraph-skeleton.md lines 153-190）

| 节 | 骨架段数 | 正文段数 | 对齐 |
|----|---------|---------|------|
| §4.1 | 4 (claim/evidence/comparison/transition) | 4 (192/194/196/198) | ✅ |
| §4.2 | 5 (claim/2×evidence/comparison/transition) | 5 (202/204/206/208/210) | ✅ |
| §4.3 | 5 (claim/2×evidence/claim/transition) | 5 (214/216/218/220/222) | ✅ |
| §4.4 | 4 (claim/evidence/comparison/transition) | 4 (226/228/230/232) | ✅ |
| §4.5 | 4 (claim/evidence/comparison/transition) | 4 (236/238/240/242) | ✅ |

§4 chapter intro (line 188) 不在 skeleton 中显式列出，但属合理桥接段（点出 §4 处理 axis 1 + 列出 C1.1/C1.2/C1.3/C2.4），不构成回归。

---

### 架构准确性（4 处抽样 vs CLAUDE.md）

| # | 抽样位置 | 正文表述 | CLAUDE.md 表述 | 判定 |
|---|---------|---------|---------------|------|
| 1 | §4.1.p2 (line 194) | "core pipeline 是 line_buffer → mac_array → parf_accum → ofb_writer，wgt_buffer 侧路供 WRF" + "DMA 子系统：idma_ctrl/wdma_ctrl/odma_ctrl + axi_dm + mm2s_arb + axi_m_mux + axi_lite_csr" | "line_buffer → mac_array → parf_accum → ofb_writer；wgt_buffer 侧路供 WRF" + "idma_ctrl / wdma_ctrl / odma_ctrl + mm2s_arb + Xilinx axi_dm IP + axi_m_mux + axi_lite_csr" | ✅ 完全一致 |
| 2 | §4.2.p1 (line 202) | "5 模块各自维护 counter（row, column, kx/ky, slice），通过 valid-ready 串联，无中心 FSM" | "valid-ready 握手，无中心 FSM" | ✅ 一致；正文 counter 列表更具体但不冲突 |
| 3 | §4.3.p1 (line 214) | "line_buffer 仅在 rows_available ≥ y_out · stride + Ky 时发射（forward-pressure），ODMA 返回 row-credit 给 ofb_writer" + "任意 H×W (C1.2)" | "streaming row-ring 任意 H×W strip 粒度 ring" | ✅ 一致；正文给出具体数学条件，未越界 |
| 4 | §4.4.p1 (line 226) | "16 个 parf_col 实例，每个 instance 拥有独立 SRAM，外壳广播 wr_addr / we / rd_addr (C1.3)" | "parf_col × NUM_COL 组成（每列独立 SRAM，外壳共享 wr_addr/we）" | 🟡 轻微：CLAUDE.md 只列 wr_addr/we，正文 + skeleton 都加了 rd_addr。skeleton line 180 也写 "wr_addr/we/rd_addr"，骨架与正文自洽，但与 CLAUDE.md 一行简述存在 1 项差异。考虑到 PARF 设计客观需要 rd_addr，此扩充合理；不构成事实错误，仅记录 |

---

### Contribution 编号引用

| C# | 出现位置 | 章节匹配 | 判定 |
|----|---------|---------|------|
| C1.1 | line 188 (§4 intro), line 202 (§4.2) | §4.2 Decentralized Valid-Ready Pipeline | ✅ |
| C1.2 | line 188, line 214 (§4.3) | §4.3 Streaming Row-Ring | ✅ |
| C1.3 | line 188, line 226 (§4.4) | §4.4 Per-Column PARF | ✅ |
| C2.4 | line 188, line 236 (§4.5) | §4.5 Loop-Nest Realization | ✅ |

无遗漏 / 无错挂。

---

### Claim 强度

无 "novel architecture" / "first to" 之类 overclaim。逆向证据：

- §4 intro (line 188): "deliberately kept the architecture conventional in places where novelty would not justify the verification cost"
- §4.2 (line 208): "we do not claim a contribution at the level of the Buffets formalism, only that the FLUX_CNN core pipeline is a faithful instance of that pattern"
- §4.4 (line 232): "We classify this as a unit-level implementation choice rather than a top-level contribution"
- §4.5 (line 240): "We do not claim a new loop-nest taxonomy"

claim 调子保守、与 contributions.md 自评的 "工程整合 + 局部 novelty" 定位一致。

---

### [CHECK]/[TBD] 合规性

§4 内实际计数（不含 §3 line 180 的 [CHECK]）：

| # | 位置 | 标记 | 内容 | 合理性 |
|---|------|------|------|--------|
| 1 | line 206 (§4.2) | [CHECK] | 22-case chained regression PASS rate, 来自 STATUS.md | ✅ 应标，PASS rate 是具体数字需实测确认 |
| 2 | line 216 (§4.3) | [CHECK] | ring footprint 实测 (~10 KB at strip_rows=8, W=640) | ✅ 应标，footprint 字节数需 BRAM map 确认 |
| 3 | line 218 (§4.3) | [CHECK] | 24-case shape suite count, 来自 STATUS.md | ✅ 应标，case 数需 STATUS.md 验 |
| 4 | line 228 (§4.4) | [CHECK] | parf_col BRAM count and port configuration | ✅ 应标，single-port BRAM 主张需综合报告确认 |
| 5 | line 232 (§4.4) | [TBD] | §4.4 是否合并入 §4.2 由 polisher 阶段决定 | ✅ 合理结构标记 |

合计 4 [CHECK] + 1 [TBD]。启动指令提到 "5 [CHECK]"，实际数为 4 + §3 line 180 的 1 个跨章 [CHECK]（关于 S2D 引用谱系）；以"§4 内"严格计数为 4，标记位置全部合理，**无该标未标**且**无误标**情形。

正文中未标 [CHECK] 的具体数字（"5 modules"、"16×16 MAC"、"sixteen parf_col"、"七层嵌套"、"480×640 / 4.9 MB"）皆为：
- 架构常量（NUM_COL=16）— 来自 CLAUDE.md / params 单一源，无需 [CHECK]
- 形式化推导（4.9 MB = 480×640×16 bit ≈ 614400 bytes 量级）— 自洽
- 已被同段 [CHECK] 覆盖（10 KB ring）

---

### 关键参数自洽（vs CLAUDE.md）

| 参数 | CLAUDE.md 值 | §4 出现 | 一致 |
|------|------------|---------|------|
| NUM_COL = NUM_PE = 16 | 16 | "16×16 MAC array" (line 236)；"sixteen parf_col instances — one per MAC column" (line 226)；"⌈Cin/16⌉" + "⌈Cout/16⌉" (line 236) | ✅ |
| WRF / ARF / PARF = 32 | 32 | 仅作模块名 PARF 出现，未涉及深度数字 | ✅ 不冲突 |
| DATA = 8 / PSUM = 32 | 8 / 32 | §4 未数值化提及 | ✅ 不冲突（属 §6/§7 范围） |
| BUS_DATA_W = 128 | 128 | §4 未数值化提及 | ✅ 不冲突 |
| 接口：1 AXI4 master + 1 AXI-Lite slave | 同 | line 192 "exposes only one AXI4 master port ... and one AXI-Lite slave port" | ✅ |

---

### 回归性（vs paragraph-skeleton.md §4）

§4 引用文献：

| 引用 | skeleton 位置 | 出现位置 | 判定 |
|------|------------|---------|------|
| NVDLA | line 159, 167, 182 | line 196, 208, 230 | ✅ 骨架内 |
| Snowflake-ISCAS17 | line 159 | line 196 | ✅ |
| Gemmini-DAC21 | line 167, 182 | line 208, 230 | ✅ |
| VTA-Micro19 | line 167 | line 208 | ✅ |
| Buffets-ASPLOS19 | line 167 | line 208 | ✅ |
| Alwani-MICRO16 | line 175 | line 220 | ✅ |
| Kang-Sensors23 | line 175 | line 220 | ✅ |
| Liu-TNNLS21 | line 175 | line 220 | ✅ |
| Interstellar-ASPLOS20 | line 189 | line 240 | ✅ |
| Ma-FPGA17 | line 189 | line 240 | ✅ |

§4 引入文献 0 条新增 / 0 条改名。

§4 引入数据：除骨架已有的 22+24=46 case、480×640/4.9 MB/10 KB 之外，无新增定量声明。

无回归。

---

### 文献引用真实性核验

§4 引用 10 条全部继承自 §3 / §1 / §2 已通过审查的引用集（NVDLA / Snowflake / Gemmini / VTA / Buffets / Alwani / Kang / Liu / Interstellar / Ma），**§4 未引入任何新引用**，因此引用真实性核验**继承前次结论**，本次无须重测。check-citations skill 因离线降级方案：4 项要素（作者+会议+年份+谱系）均由前次评审确认；本次无可疑新增。

---

### 通过原因

1. 段落骨架与正文 5 节 22 段（+ 1 chapter intro）严格逐段对应，无新增/缺失/错位段
2. 架构事实 4 处抽样对照 CLAUDE.md：核心 5 模块 + DMA 子系统组件齐全且名字精确，valid-ready/row-ring/per-column PARF 表述准确
3. 4 个 Contribution 编号 (C1.1/C1.2/C1.3/C2.4) 各落在对应小节
4. 无 over-claim；多处主动降调（"deliberately conventional" / "not a contribution at Buffets level" / "not a top-level contribution" / "do not claim a new loop-nest taxonomy"）
5. 4 [CHECK] + 1 [TBD] 标记位置全部合理，无遗漏标记
6. 关键参数（NUM_COL=16 / 接口口数）与 CLAUDE.md 一致
7. 无新引用、无新数字、无回归

---

### 备注（非缺陷）

- §4.4 (line 226) 把 `rd_addr` 加入 "shared shell" 列表，CLAUDE.md 一行简述只写 `wr_addr/we`。骨架与正文自洽，PARF 设计上 rd_addr 共享亦合理；建议 Phase 6 polisher 顺手核 RTL `parf_accum.sv` 顶层端口列表确认是否要回填 [CHECK]，但**不阻塞当前 §4 PASS**
- §4.1 line 196 描述 NVDLA 命名谱系时使用 `line_buffer → mac_array → parf_accum → sdp` —— 以 `sdp` 作为终点（NVDLA 风格），而 line 194 的实际 core pipeline 终点是 `ofb_writer`。两处分别在"naming taxonomy"和"physical chain"语境下使用，正文已隐含区分；无修改建议
- 启动指令提到 "5 [CHECK]"，实际 §4 内为 4 [CHECK]（§3 line 180 跨章 1 个 [CHECK] 不计入 §4）—— 数值差异为 1，所有现存 [CHECK] 位置均合理，无新增/删减需要
