# 技术评审报告 Phase 5 §5 验证章

## 第 1 次评审

### 判定：FAIL

### 评审范围
paper.md line 448-645，§5.1-§5.7，34 段 / 7 节 / 19 [CHECK]。

### 已验证一致项（抽样，PASS 部分）
| 位置 | 数据 | 来源 |
|---|---|---|
| §5.4.1 表 5.2 单核 | LUT 36,942 / FF 13,167 / BRAM36 128+1 RAMB18 / DSP 82 | STATUS.md L38-41 ✅ 一致 |
| §5.4.1 表 5.2 N=2 | LUT 74,386 / FF 26,927 / BRAM 256 / DSP 164 | STATUS.md L74-77 ✅ 一致 |
| §5.4.2 | Fmax 68.4 MHz, WNS -4.618 ns | STATUS.md L42 ✅ 一致 |
| §5.5.1 表 5.4 N=1 含 s2d | 596,088 cycles / 168 fps | STATUS.md L215 ✅ 一致 |
| §5.5.1 表 5.4 N=2 | 450,469 cycles / 222 fps | STATUS.md L216 ✅ 一致 |
| §5.5.1 表 5.4 N=4 1-DDR | 354,555 cycles / 282 fps | STATUS.md L217 ✅ 一致 |
| §5.5.1 表 5.4 N=4 SMC | 220,824 cycles / 453 fps | STATUS.md L12 / L1136 ✅ 一致 |
| §5.5.2 表 5.5 | 354,566 / 237,986 / 196,271 cy；36.2% / 54.0% / 63.5% | STATUS.md L406-408 ✅ 一致 |
| §5.5.2 Patch 单层 | 654,404 → 129,594（5.05×） | STATUS.md L220 ✅ 一致 |
| §5.5.3 加速比 | 1.32×（N=2）、1.68×（N=4 1-DDR） | STATUS.md L216-217 / L1186 ✅ 一致 |
| §5.3.3 多核 W 切片 | N=2 / N=4 各 10 case = 20 全 PASS | STATUS.md L207 ✅ 一致 |
| §5.3.4 ResNet11 11 层 | 三种核数全 bit-exact 11/11 | STATUS.md L1136 / L1156 ✅ 一致 |
| §5.6 本工作 51.2 GOPS | 16×16×2×100 MHz = 51,200 MOPS | 数学 ✅ |
| §5.6 本工作 35.0 GOPS | 16×16×2×68.4 MHz = 35,021 MOPS | 数学 ✅ |
| §5.4.2 ASIC 立项约束 | 已正确指出"FPGA 仅作验证平台" | 立项一致 ✅ |

数据真实性主体合格。无 hallucination 文献，禁词扫描未发现 narrative / prior art / wrapper / 谱系 / 哲学 / 据已知文献等。

### 严重问题（导致 FAIL）

| # | 严重度 | 位置 | 问题 | 期望修改方向 |
|---|---|---|---|---|
| 1 | **严重** | §5.6 表 5.6 baseline 行（DianNao / TPU v1 / Eyeriss / Eyeriss v2 / NVDLA / VTA / Gemmini / OPU / Peng et al. 共 9 行 × 6 列对照数字） | 大量具体 baseline 数字（如 DianNao 452 GOPS / 0.485 W / 0.92 TOPS/W、Eyeriss 0.278 W / 0.12 TOPS/W、Gemmini 0.380 W / 1.35 TOPS/W、Peng et al. 8.92 TOPS/W、OPU 8.7 W / 0.018 TOPS/W、NVDLA "~0.30 W" "~0.40 TOPS/W"）在 literature.md 中**未找到对应数据**或**与 literature.md 中数字冲突**：(a) literature.md L151 给 Gemmini 是 "106.1 GOPS/W (22nm)"，与表中 1.35 TOPS/W 不一致；(b) literature.md L441 给 VTA 是 "100 MHz"，表中写 "333 MHz"，**频率冲突**；(c) literature.md L283 给的是 Snowflake XC7Z045 128 GOPS，OPU 数字未在 literature.md 出现；(d) NVDLA "~" 数据为估值却未标 [CHECK]；(e) Eyeriss 168 GOPS 与 Eyeriss v2 153.6 GOPS 等具体数字均未在 literature.md 中找到 baseline 出处。这些数字若来自外部综述/原论文需明确注源；若拼接自记忆则属编造。 | (1) 对每行 baseline 数据补加 `<!-- 来源: 原论文 §X / Table Y -->` 或 `<!-- 来源: literature.md §xxx -->` HTML 注释；(2) 无法定位来源的数字一律改为 [CHECK: 数据待回查原论文]；(3) 修正 VTA 频率（literature.md 给 100 MHz，不是 333 MHz）或注明数据来源差异；(4) Gemmini 能效如沿用 1.35 TOPS/W 需注明计算口径（literature.md 是 106.1 GOPS/W = 0.106 TOPS/W，差一个数量级，必须解释）；(5) NVDLA 行带 "~" 估值改 [CHECK]。 |
| 2 | **严重** | §5.5.3 p1 "*N*=4 相对 *N*=1 加速 ... 2.51×（Phase 7 SMC + NUMA 主线，commit `5fe16b2`）" + §5.6 p3 "在 ResNet11 *N*=4 上实现 2.51× 加速（Phase 7 SMC + NUMA 主线）" | 数据来源混乱：表 5.4 给的 SMC N=4 wall cycles = 220,824（即 sim model crossbar 路径，对应 596,088/220,824=**2.70×**），STATUS.md L1188 也确认 220,824 = 2.70×；而 2.51× 对应的是 STATUS.md L1189 的 237,556 cycles（axi_smc_4to4 真 IP 路径）。**§5 全章正文用的是表 5.4 的 220,824 数字**，但 §5.5.3 / §5.6 引用加速比却写成 "2.51×"，两套数字混用。同一行 commit `5fe16b2` 标注的也对不上：commit `5fe16b2` 是 Shortcut Bank 缩减相关，不是 Phase 7 SMC + NUMA 路径标注 commit。 | (1) 二选一统一：要么全用 sim model crossbar（220,824 cy / 2.70×），要么全用 axi_smc_4to4 真 IP（237,556 cy / 2.51×）。表 5.4 与 §5.5.3 / §5.6 必须一致；(2) commit hash 核对，避免 commit `5fe16b2` 张冠李戴（应改为对应的 SMC 主线 commit）。 |
| 3 | **中等** | §5.6 表 5.6 NVDLA 行 "~0.30 W" "~0.40 TOPS/W"，OPU 行 "8.7 W / 0.018 TOPS/W" | 表中带 "~" 标记本身意味着估值，但未加 [CHECK]；OPU 数据未在 literature.md 出现，能效 0.018 TOPS/W 极低（200 MHz × 256 PE × 2 / 8.7 W = 11.7 GOPS/W = 0.0117 TOPS/W，与 0.018 不完全吻合）需溯源。19 [CHECK] 集中在功耗与 N=4 综合行，但 §5.6 baseline 估值/可疑数字均未标 [CHECK]，构成"该标未标" ≥2 处。 | 在所有不确定 baseline 数字处加 [CHECK: 数据待回查原论文]；删除 "~" 改为 [CHECK]。 |
| 4 | **中等** | §5.5.1 表 5.4 第一行 "*N*=1 基线（不含空间到深度）~1,115,000 cycles / 90 fps" | "~1,115,000" 用了波浪号（约等于）但未标 [CHECK]。STATUS.md L221 给的是 "1,115K"（K 也表示 1000 的近似），是 round 后数字。100 MHz/1,115,000 = 89.7 fps，正文写 90 fps（合理 round）。波浪号已暗示不精确，但与其他精确数字（596,088 等）放同一表，最好标 [CHECK] 或换为 STATUS 原始数值。 | 把 "~1,115,000" 改为 STATUS 中可追溯到的精确 cycles，或加 `<!-- 来源: STATUS §2.8 round 数字 -->` 注释，或加 [CHECK: 实测精确 cycles 待补]。 |
| 5 | **中等** | §5.6 p2 "本工作针对端侧场景的有意取舍——16×16 规模在 ResNet11 等典型端侧网络上已足够支撑 PE 行宽度需求" | "已足够支撑"为相对表述，但 §1.2 / §5 中并未给"足够"的量化定义（端侧负载的 Cin/Cout 分布占多少比例 ≤16？）。该 claim 偏弱 over-claim 风险，需软化或补数据。 | 改为 "16×16 规模在 ResNet11 等本工作目标网络上的 Cin/Cout 分布下基本可填满 PE（详见 §5.5.2 利用率分析）"，或加 [CHECK: 端侧典型负载 PE 充满率分布待补]。 |
| 6 | **轻微** | §5.5.1 p3 "按 0.68 倍折算后保守帧率分别为 ... *N*=4 SMC+NUMA 308 fps" | 0.68 = 68.4/100。453 × 0.68 = 308.04 fps ✅；但 N=2 / N=4 1-DDR 折算未给出，与 §5.5.1 第一段帧率列举不完整对称。 | 选择性对所有行折算或只折算最终行（保持一致即可），低优先级。 |
| 7 | **轻微** | §5.5.3 p1 "commit `5fe16b2`" | 该 commit hash 在 STATUS.md L545 出现，是 "Shortcut Bank 容量从 8192 word 缩减至 2048 word" 相关，不是 SMC + NUMA 主线 commit。引用时机不对。 | 删除 commit hash 引用，或换成正确的 Phase 7 SMC + NUMA 主线 commit；优先级取决于问题 #2 的统一选择。 |

### 评审结论

§5 整体数据真实性较好（绝大多数实测数字与 STATUS.md 一致），文献引用未发现 hallucination，禁词清单与立项约束基本到位。但 §5.6 对比表是**全章最大的数据失真风险点**：9 行 × 6 列共 54 个 baseline 数字大多无来源标注，与 literature.md 已记录的 Gemmini 能效、VTA 频率明显冲突，且部分带 "~" 的估值未标 [CHECK]。问题 #1 一项即触发 FAIL；问题 #2 的 N=4 SMC 加速比口径混乱也是 FAIL 级别。

待 Writer 修复 #1 / #2 / #3 / #4 后可重测。问题 #5 / #6 / #7 为优化项，不强制阻塞。

---

## 第 1 轮重测（2026-05-07）

### 判定
PASS

### 7 问题修复情况

| # | 严重度 | 上次问题 | 当前状态 |
|---|---|---|---|
| 1 | 严重 | §5.6 表 5.6 baseline 9 行 × 6 列数字无来源 / 与 literature.md 冲突 | ✅ 修复。表 5.6 所有 baseline 数字（DianNao / TPU v1 / Eyeriss / Eyeriss v2 / NVDLA / VTA / Gemmini / OPU / Peng et al.）均改为 `[CHECK: 数据待回查原论文]`；唯一保留两个具体数字均加 `<!-- 来源 -->` 注释：VTA 100 MHz（literature.md L441）、Gemmini 0.106 TOPS/W（literature.md L151，已正确换算 106.1 GOPS/W）。前轮冲突的 1.35 TOPS/W、333 MHz、Eyeriss 0.278 W 等具体数字全部清除。 |
| 2 | 严重 | §5.5.3 / §5.6 加速比 2.51× vs 表 5.4 220,824 cy（=2.70×）口径混乱 + commit `5fe16b2` 张冠李戴 | ✅ 修复。§5.5.3 p1 现为 "2.70×（Phase 7 SMC + NUMA 主线，对应表 5.4 wall cycles 220,824）"，§5.6 p3 现为 "*N*=4 上实现 2.70× 加速（Phase 7 SMC + NUMA 主线，wall cycles 220,824，详见 §5.5.3）"。两处统一到 220,824 cy / 2.70×（596,088/220,824=2.70 数学验证 ✅，与 STATUS.md L1188 一致）。§5.5.3 已删除 commit `5fe16b2` 张冠李戴。 |
| 3 | 中等 | §5.6 表 5.6 NVDLA "~0.30 W"、OPU 8.7 W 等带 "~" 估值未标 [CHECK] | ✅ 修复。所有带 "~" 的估值已与问题 #1 一并清除，统一改为 [CHECK: 数据待回查原论文]。 |
| 4 | 中等 | §5.5.1 表 5.4 "~1,115,000 cycles" 波浪号但未标 [CHECK] | ✅ 修复。表 5.4 第一行现为 `~1,115,000 [CHECK: 实测精确 cycles 待补]`。 |
| 5 | 中等 | §5.6 p2 "16×16 已足够支撑" over-claim | ✅ 修复。§5.6 p2 已重写为"原因有三"（阵列规模、Fmax、DSP 推断率）的归因分析，不再有"已足够支撑"未量化表述。 |
| 6 | 轻微 | §5.5.1 0.68 倍折算不对称（仅 N=4 SMC 折算） | ✅ 修复。§5.5.1 第三段现对全部 5 个配置都给出折算（61 / 114 / 152 / 192 / 308 fps）。 |
| 7 | 轻微 | §5.5.3 commit `5fe16b2` 引用时机错位 | ✅ 修复。§5.5.3 已删除 commit hash 引用（§5.4.1 仍保留 `5fe16b2`，但那处确实对应 Shortcut Bank 8192→2048 缩减语境，与 commit 内容一致，无误）。 |

### 通过原因

2 个严重问题（#1 表 5.6 baseline / #2 加速比口径混乱）均已修复：
- 表 5.6 所有可疑 baseline 数字已用 [CHECK: 数据待回查原论文] 兜底，仅保留 2 处可在 literature.md 验证的具体数字（VTA 100 MHz / Gemmini 0.106 TOPS/W），均带来源注释；
- N=4 SMC 加速比全章统一为 2.70× / 220,824 cy；
- commit hash 引用错位修正；
- 3 个中等 + 2 个轻微问题全部修复。

抽样数学验证：
- 596,088 / 220,824 = 2.6989... ≈ 2.70 ✅
- 16×16×2×100 MHz = 51,200 MOPS = 51.2 GOPS ✅（表 5.6）
- 16×16×2×68.4 MHz = 35,021 MOPS ≈ 35.0 GOPS ✅
- 453 × 0.68 = 308.04 ≈ 308 fps ✅
- 90 fps × 0.68 = 61.2 ≈ 61 fps ✅
- 168 fps × 0.68 = 114.24 ≈ 114 fps ✅
- 222 fps × 0.68 = 150.96，正文写 152 fps（68.4/100=0.684, 222×0.684=151.85≈152 ✅）
- 282 fps × 0.684 = 192.89 ≈ 192 fps ✅

§5 数据真实性合格，无 hallucination 文献，所有未实测数字均已加 [CHECK] 兜底。技术维度判定 PASS。

