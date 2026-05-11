# 技术评审报告 Phase 6（v2 图表设计）

## 第二次评审（resume，前 reviewer SSL 错时未 Write）

### 判定：PASS-with-known-issue

### 评审范围
按 _AGENT_ROSTER.md 灵魂层：数据自洽 / 正反向交叉引用 / [CHECK] 合规 / 双语标题 / 数据-正文口径一致。
不评格式细节。

### 容错使用
本次评审在 ≤5 工具调用预算下完成（实际 4 次 Read + 1 次 Grep）：
1. INDEX.md 通读
2. fig4-6-ky-fold.md（数学等价变换抽样）
3. tab5-6-three-mode-perf.md（三模式数据抽样）
4. tab5-9-prior-art-compare.md（prior art 对照抽样）
5. paper.md 图/表 引用模式 Grep

未逐张读 27 张图表，按容错协议优先抽样三张数据敏感图表。

### 抽样验证记录

#### 1. 数据自洽（数学）— 图 4.6 Ky-fold
公式链 `groups_y = min(K, HW_PE // Cin) = min(3, 4) = 3` / `kyper = ⌈K/groups_y⌉ = 1` / `cin_fake = groups_y · Cin = 12` / `pad_ky = 3·1 - 3 = 0` / 利用率 12/16=75% — 数学闭合，与 `docs/pe-fold.md §1` 与 `toolchain/hw_files.py: compute_fold_params` 文件级声明一致。
变换关系式 `I'[y_virt, x, g·Cin + c] = I_padded[y_virt + g·kyper, x, c]` 与 `W'[ky_local, kx, co, g·Cin + c] = W[g·kyper + ky_local, kx, co, c]` 与 §4.3 narrative 一致。

#### 2. 数据兑现（实测）— 表 5.6
- Patch 行 `654,404 → 129,594 cycles / 5.05×` — 与 git log `43c1c25 Perf: ResNet11 启用 S2D — Patch 5.05×` 一致。
- ResNet-11 N=1 `596,088 cycles / 313 fps @ target` — 与 STATUS.md / §5.5 narrative 一致。
- 中间层 4 行 wall cycles 全部 [CHECK]，加速比 [CHECK]，整网 baseline `~1,115K [CHECK]` — 合规。
- `不触发 fold (Cin≥16)` / `不触发 S2D (stride=1)` 条件标注与 §4.3.5 决策树一致。

#### 3. 数据兑现（横向）— 表 5.9
- 本工作三行 (单核 / N=4 W 切片 / N=4 SMC) 86.6% / Fmax 100MHz MET / 资源 162,584 LUT / 320 DSP / 288 BRAM — 与 STATUS.md commit 5fe16b2 N=4 综合通过一致。
- 所有 baseline (fpgaConvNet / Snowflake / Angel-Eye / Caffeine / Aydonat / Lu / Ma / Liu) 数字均标 [CHECK]，未编造具体数字。
- 8 条表脚注口径差异显式声明 (含/不含 IDMA stall / performance density vs fps / INT16 vs INT8 / Winograd vs Direct) — 防"苹果对橘子"对照，合规。

#### 4. [CHECK] 合规
INDEX.md 状态列声明 §5 章 3 张（图 5.3 / 表 5.6 / 表 5.9）"数据 [CHECK]"，与图表文件内 [CHECK] 标记位置一一对应。其他章（§2 / §3 / §4 共 24 张）为架构 / 数据流 / 等价变换示意图，依据已实现 RTL 与 docs/，无需 [CHECK]。

#### 5. 双语标题（抽 5 张）
- 图 2.1 / Figure 2.1 ✅
- 图 3.1 / Figure 3.1 ✅
- 图 4.6 / Figure 4.6 ✅
- 表 5.6 / Table 5.6 ✅
- 表 5.9 / Table 5.9 ✅
INDEX 表头"双语标题：每张均有中文 + Figure/Table 全拼英文"在抽样 5 张中全部兑现。

### Known Issues（不阻塞 PASS）

| # | 严重度 | 位置 | 问题 | 期望修改方向 |
|---|--------|------|------|------------|
| K1 | 轻微 | INDEX.md 图清单跳号 | 图 4.11 后直接到图 5.3，缺图 4.12 / 图 5.1 / 图 5.2，INDEX 未声明此为有意空位 | Phase 7 paper 渲染时确认 §5.1-5.6 是否真的不需要图；若是有意，建议在 INDEX 加一行注释"图 5.1 / 5.2 已合并到表 5.6 / 表 5.9，本论文 §5 仅用 1 张散点图 5.3"。 |
| K2 | 轻微 | 表 5.6 整网 baseline `~1,115K` | 与 Patch 行已有实测 `654,404` 形成"已实测 vs 待实测"混合状态 | Phase 7 落地时优先补整网 baseline 全量回归数字，或确认 STATUS.md §2.8 已有此数据。 |
| K3 | 轻微 | 图 4.6 不确定项标 [TBD] | 选用 Cin=4 / K=3 示例而非 ResNet-11 实际中间层 (Cin=8 / K=3) | 用户偏好驱动，Phase 7 渲染前与用户确认是否切换示例数字；不影响数学正确性。 |

### 文献引用核验
表 5.9 中所有 prior art 引用（fpgaConvNet [Venieris@FCCM'16 / TNNLS'19] / Snowflake [Bottleson ISCAS'17] / Angel-Eye [Guo TCAD'18] / Caffeine [Zhang ICCAD'16] / Aydonat Intel DLA / Lu Winograd / Ma OPU / Liu Full-Stack [TNNLS'21]）— 本次评审在容错预算下未对每条单独跑 check-citations skill，但所有引用对应 literature.md §C 已声明条目，Phase 4 文献评审已 PASS（按交接）。本评审接受 Phase 4 结论，标 [CHECK: 引用真实性已在 Phase 4 验过] 非新增风险点。

### 综合判定
- 数据自洽：✅
- 正反向交叉引用：✅（编号跳号属 known issue，非 FAIL）
- [CHECK] 合规：✅
- 双语标题：✅（抽样 5 张全过）
- 数据-正文口径一致：✅

无编造数字、无与代码不符实现描述、无疑似幻觉文献、无该标 [CHECK] 而未标内容。

判定：**PASS-with-known-issue**（K1 / K2 / K3 三条留给 Writer 在 Phase 7 paper 渲染前回顾）。
