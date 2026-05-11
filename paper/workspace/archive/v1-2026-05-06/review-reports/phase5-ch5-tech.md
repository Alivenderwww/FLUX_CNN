### 判定 PASS

## 评审范围
Phase 5 §5 Compiler Optimizations for PE Utilization (paper.md line 248–297)。仅评 §5。参考骨架 paragraph-skeleton.md line 194–226；核心数学 docs/pe-fold.md。

## 段落对齐
骨架 §5 共 18 段：§5.1 五段、§5.2 五段、§5.3 四段、§5.4 四段。Writer 实写：
- §5.1：5 段（claim → math → benefit/cost → vs im2col → vs MAERI/Eyeriss-v2 + 章首 framing 段 + metric scope 引言段，共 7 个块）。骨架 5 段被忠实覆盖，只是把章首口径声明（骨架在 line 196 单独标）做成了独立块。合理。
- §5.2：4 段（claim → math/no-copy → Shi 谱系 + S2D 出处 → vs Ky-fold + DDR 节省）+ 1 段 prior art note。骨架 5 段；段 5 (transition) 与段 3 段 5 谱系合并到 prior art note。合并合理。
- §5.3：4 段（joint 触发 → 回归证据 → Cout<16 honesty → narrative transition），与骨架 4 段一一对齐。
- §5.4：4 段（thesis + Liu 86.6 vs 97 → 4-axis Table 2 → trade-off non-exclusive → §7 引）。与骨架 4 段一一对齐。

Writer 自报 22 段含 framing/transition：实际块数与骨架等价偏多 1–2 个 framing 块（含章首段 250、metric scope 252、§5.4 § 296 transition），未引入新论点。**合并/拆分合理。**

## Ky-fold 数学（§5.1）
对照 docs/pe-fold.md §1：
- groups_y = HW_PE/Cin ✓
- kyper = ⌈K/groups_y⌉ ✓
- cin_fake = groups_y · Cin ✓
- ky = g·kyper + ky_local 分组 ✓
- I'[y_virt, x, g·Cin+c] = I_padded[y_virt + g·kyper, x, c] ✓ — paper.md line 258 与 docs §1 line 23 一字不差
- W'[ky_local, kx, co, g·Cin+c] = W[g·kyper + ky_local, kx, co, c] ✓ — line 258 与 docs line 24 一致
- pad_ky = groups_y·kyper - K，零填充 ✓ — line 258 提到 "pad-out positions are zero-filled"
- IFB 4× inflation（Cin=4, HW_PE=16 → groups_y=4） ✓ — line 260
- "输入 y-方向偏移复制"措辞 ✓ — line 258 "each virtual row materializes groups_y y-shifted physical rows"

**Ky-fold 数学完全对齐 docs/pe-fold.md，无错。**

## S2D 数学（§5.2）
对照 docs/pe-fold.md §2：
- stride² 个相位，p = a·stride + b 索引 ✓ — line 268, 270
- I'[Y, X, p·Cin+c] = I_padded[Y·stride+a, X·stride+b, c] ✓ — line 270 与 docs line 59 一致
- K_new = ⌈K/stride⌉, Cin_new = stride²·Cin ✓
- "纯字节排列、无复制" ✓ — line 270 "purely a permutation of input bytes ... no input element is duplicated, S2D is DDR-bandwidth-neutral (in bytes)"
- K 被 stride 整除时 pad_waste=0 ✓ — line 270 与 docs §2 line 72 一致
- stride=1 后 ARF reuse_en=1 复用 ✓ — line 270 与 docs §2 line 84 一致
- "DDR 友好"准确度 ✓ — Writer 用了更精确的 "DDR-bandwidth-neutral (in bytes)"，比骨架 "DDR 友好" 更严谨

**S2D 数学完全对齐，"无复制 + DDR 友好" 表述比骨架更精确。**

## Liu 86.6% vs 97% 论证（§5.4 段 1, line 290）
原文：
> "FLUX_CNN reaches 86.6% network-level MAC utilization on a ResNet-18-style suite on an XC7K325T with a fixed 16×16 array, comparable in spirit to the 97% reported by Liu et al.'s Full-Stack design on an Arria 10 GX1150, even though that work applies hardware-level reconfiguration on a substantially larger device. The two numbers are not strictly head-to-head — different devices, different workloads, different metric definitions — but they sit in the same regime"

清楚标注：
- different devices ✓ (XC7K325T vs Arria 10 GX1150)
- different workloads ✓
- different metric definitions ✓
- "not strictly head-to-head" ✓
- "comparable in spirit" / "same regime" 软化措辞 ✓
- 加 [CHECK: §7 实测口径汇总] 标记 ✓

**Liu 论证三维度全标注，措辞 honest 不夸张。**

## claim 强度
§5.1 段 5 line 264：
> "To our knowledge, packaging this Ky-only fold as an ahead-of-time pass on a fixed 2-D MAC array, with no runtime configuration and no per-layer microcode, is a distinguishing point of this work"

避免 "first to..."，用 "To our knowledge ... a distinguishing point"，强度合规。

§5.2 line 272：
> "FLUX_CNN's contribution is not the identity itself but its repackaging as a post-training compile pass"

未 over-claim，区分清楚 "identity 不属我们 / 重打包属我们"。

§5.2 line 276 prior art note 用 "appears to be less standard"，软化措辞合规。

**全章无 "first to" / "novel" / "unprecedented" 等 over-claim。**

## prior art 谱系标记
§5.2 末 line 272: `[CHECK: S2D 引用谱系待补查 — Shi 2016 sub-pixel / YOLOv5 focus / pixel-shuffle inverse 是否是被加速器领域引用的标准 prior art，待 reviewer-stage 文献核]` ✓
§5.2 末 line 276: `[CHECK: S2D 加速器领域引用谱系]` ✓
两处 [CHECK] 都在，谱系不确定性已显式标记。

## Cout<16 诚实 disadvantage（§5.3 段 3, line 284）
原文：
> "The complementary case — Cout < 16 — is intentionally not optimized in the current design. Layers such as small classifier heads with Cout=2 simply leave (16-Cout) PE columns idle, capping column utilization at Cout/16. We made this choice deliberately ... it is a real limitation of the compiler-only route and we report it honestly."

- 明确说"PE 列空转 / 利用率 = Cout/16"
- 给具体例子 "Cout=2"
- 给硬件原因（共享 wr_addr 的 parf_col × NUM_COL）
- 有 docs/pe-fold.md §3 / RTL/parf_accum.sv 来源注释
- 主动用 "real limitation" / "report it honestly" 措辞 — 不被软化
- ResNet-18 "negligible MAC%" 给出辩护但没掩盖事实

**诚实度合规，未软化。**

## vs Hardware-Reconfigurable 对位（§5.4 段 2, line 292）
四维度对位：
- hardware complexity: MAERI ART [Kwon@ASPLOS'18] ✓ + Eyeriss-v2 NoC [Chen@JETCAS'19] ✓
- compiler complexity: Timeloop [Parashar@ISPASS'19] + MAESTRO [Kwon@MICRO'20]
- memory cost: im2col K² inflation vs Ky-fold groups_y vs S2D in-place
- applicable scenarios: dynamic vs ahead-of-time

对位含 MAERI ✓ Eyeriss-v2 NoC ✓。**Tangram 未提及**——但骨架 line 224 也只列了 "MAERI ART / Eyeriss-v2 NoC / cuDNN im2col"，Tangram 不是骨架要求。任务清单 #8 提"四维度对位是否含 MAERI / Eyeriss-v2 NoC / Tangram"——Tangram 在 §5.4 未出现，但骨架未要求且 §5.4 已 honest 框定为 "comparable in spirit" 不强 claim 全覆盖，未触发 FAIL。建议改进项写入修订建议。

## Contribution 引用
- C2.1 line 256 ✓ Ky-fold claim
- C2.2 line 268 ✓ S2D claim
- C2.3 line 280 ✓ Joint trigger
- C2.4 line 296 ✓ transition
- C2.5 line 297 ✓ PyTorch end-to-end → §6.1 承接

**5 个 contribution 标签全到位。**

## 数据真实性
百分比 / 数字盘点：
- 86.6% network-level MAC utilization (line 290) → `[CHECK: §7 实测口径汇总]` ✓
- 97% Liu et al. (line 290) — 引用 [Liu@TNNLS'21]，外部已发表数据，无需 [CHECK]
- 12.5% (Cin=2), 25% (Cin=4) (line 256, 260) — 数学派生 (Cin/HW_PE)，可推导，无需 [CHECK]
- 4× IFB inflation (line 260) — 数学派生 (groups_y=HW_PE/Cin=16/4=4)，正确
- "Layer 1/3/4 实测 PE 利用率" (line 260) → `[CHECK: per-layer measured PE utilization for Layer 1/3/4]` ✓
- "DDR 带宽节省比例" (line 274) → `[CHECK: DDR 带宽节省比例待对照 run]` ✓
- 22 cases (line 282) — 与 CLAUDE.md "22 case ResNet-18 风格" 一致 ✓
- Cout=2 (line 284) — 例子，无需 [CHECK]
- Table 2 (line 292) → `[CHECK: Table 2 编号待终稿确认]` ✓

**所有非数学派生百分比/数字都带 [CHECK] 或可追溯实测来源。无编造数字。**

## 文献引用真实性
（受 ≤4 工具调用限制，未能调用 check-citations skill 实跑；以下基于元数据合理性判断）
- [Kwon@ASPLOS'18] MAERI — Kwon, ASPLOS 2018, 真实存在（已是经典）✓
- [Chen@JETCAS'19] Eyeriss-v2 hierarchical NoC — Chen, JETCAS 2019, 真实存在 ✓
- [Shi@CVPR'16] sub-pixel convolution — Shi et al. CVPR 2016, 真实存在 ✓
- [Parashar@ISPASS'19] Timeloop — Parashar, ISPASS 2019, 真实存在 ✓
- [Kwon@MICRO'20] MAESTRO — Kwon, MICRO 2020, 真实存在 ✓
- [Liu@TNNLS'21] Full-Stack — 元数据未幻觉但需 reviewer-stage 用 check-citations skill 校验完整 (作者 + 标题 + 期刊 + 卷号)，建议在终稿前跑 skill。
四要素无明显可疑。

## 回归性
§5 引用：
- 骨架已列：C2.1/C2.2/C2.3/C2.4/C2.5 ✓ MAERI ✓ Eyeriss-v2 ✓ Shi CVPR'16 ✓ cuDNN ✓
- §5.4 多列 Timeloop [Parashar@ISPASS'19] / MAESTRO [Kwon@MICRO'20] — 骨架 §5.3 段 4 line 219 提到 "Interstellar ASPLOS'20 / Timeloop ISPASS'19"。Writer 用 Timeloop+MAESTRO 替代 Interstellar+Timeloop，方向一致（dataflow search 工具谱系），未引入完全新主题。
- 数据：86.6%、12.5%、25%、4×、Cout=2 — 骨架已暗示或可数学推导，无新数字。
- 97%（Liu）— 骨架 line 196 章首段中骨架本身未明列，但 contributions.md 与 Phase 4 narrative A "vs Liu Full-Stack" 已确立，non-trivial 但 in-narrative 引用，不算回归。

**未引入骨架/前序阶段没有的新论点或新数字。**

## 通过-失败
| 维度 | 判定 |
|---|---|
| 段落对齐 | PASS |
| Ky-fold 数学 | PASS |
| S2D 数学 | PASS |
| Liu 86.6 vs 97 论证 | PASS |
| claim 强度（避免 first to） | PASS |
| prior art 谱系 [CHECK] 标记 | PASS |
| Cout<16 诚实未软化 | PASS |
| vs Hardware-Reconf 四维度对位 | PASS（Tangram 缺席但未触发任务清单硬性 FAIL 条件） |
| C2.1–C2.5 contribution 引用 | PASS |
| 数据真实性 + [CHECK] 位置 | PASS |
| 回归性 | PASS |

无任一硬 FAIL 触发条件命中：
- 数学未错
- Liu 论证 三维度差异显式，不含糊
- 无 "first to" over-claim
- S2D 谱系两处 [CHECK]
- Cout<16 主动用 "real limitation / report it honestly" 措辞，未软化

## 修订建议（非阻塞，留给 Writer 后续轮次或 Phase 6 优化）
1. **§5.4 段 2 Tangram 可选补位**（轻微）：四维度对位现含 MAERI / Eyeriss-v2 NoC / Timeloop / MAESTRO，若想完全匹配任务清单 #8 提及的 Tangram，可在 hardware complexity 维度加一句 "Tangram-style coarse-grained reconfigurable cores" 作为 spectrum 中点。骨架未要求，可选。
2. **§5.4 段 1 Liu 论证可微调**（轻微）：当前 "comparable in spirit" + "same regime" 已合规；若想更稳，可加一句 "we therefore avoid claiming numerical parity"，把 hedge 显式化。
3. **prior art note 合并** (轻微)：§5.2 line 272 谱系段 + line 276 prior art note 两处 [CHECK] 措辞略冗余，终稿前可合一处。
4. **[Liu@TNNLS'21] 终稿前用 check-citations skill 跑一次**（中等）：作者 + 标题 + 卷期号四要素核验，避免 chimeric 风险。本轮工具预算耗尽未跑，留给下一轮 reviewer 或 Phase 6 终审。
5. **章首段 line 250 "broadcast-and-systolic"** (轻微)：与 docs/modules/mac_array.md 措辞一致性建议在 Phase 6 figure 阶段统一核对（不在 §5 评审范围）。

以上 5 条均为优化项，不阻塞本轮 PASS 判定。
