# 新颖性评审报告 Phase 2 v2 — 论文大纲（中文本科论文范式 6 章）

## 第 1 次评审

### 判定

PASS

---

### 评审范围

按 _AGENT_ROSTER.md 灵魂层 vs 格式层分工，本评审**只管灵魂**：narrative 选择合理性 / §4 实现章主章定位 / §1.2.3 + §3.4.3 + §5.7.2 三处 prior art 应对 / 整体定位连贯 / claim 强度自评一致 / Reject 风险（X+Y 拼凑 / 缺 SOTA / [CHECK] 过载）。**不评格式细节**（小节字号 / 字数估算逻辑 / 术语统一约束等）。

待评审文件：`paper/workspace/outline.md`（v2，6 章本科论文范式，2026-05-06 完成）。
上下文核读：`contributions.md` v2（24 条贡献分 4 大类）。`literature.md` v1（沿用未重写）。

---

### 1. narrative 选择合理性

**结论**：narrative A（编译器侧 PE 利用率，C2.1/C2.2/C2.3）为主轴 + narrative B（系统集成，C1.2/C1.3/C3.4）为支撑的选择**合理且经得起 reviewer 挑战**。

**支撑论证**：

| 维度 | A 主轴（编译器侧 PE 利用率） | B 支撑（系统集成） |
|---|---|---|
| contributions.md 自评强度 | C2.1 偏强（"据已知文献..."措辞）/ C2.2 中-强（Patch 5.05× 数据撑） | C1.2 中-强 / C1.3 中-强 / C3.4 中-强 |
| 量化数据 | Patch 5.05× / 整网 N=1 1.87× / N=4 2.11× | 51-case bit-exact / N=4 综合通过 / 1-DDR vs 4-DDR PoC |
| 与 prior art 差异化清晰度 | 与 §B（MAERI/Eyeriss-v2 硬件可重构）路线对比，"硬件最简 + 编译器侧填满"取舍清晰 | 与 §C（Alwani/Kang/Liu）三处差异化在 §3.4.3 已布防 |
| 风险点 | C2.2 与 Pixel-Shuffle/Sub-pixel 谱系待 scout 补查（CHECK-6） | C1.2 单独看 prior art 多（Snowflake / fpgaConvNet），需"组合三者 + 单核 layer-serial 共用"立差 |

**reviewer A（系统派）假想质疑**："这两条 narrative 是不是 X+Y 拼凑？"
**回应**：§3.5 单独立"编译器与硬件协同设计原则"小节，把 params.py 单源参数 + cfg 派生 single source + s2d_eff 编译器/硬件视角对齐三件事作为接合点写出，narrative A 与 B 在 §3.5 显式 coupling，不是平行拼凑。

---

### 2. §4 实现章主章定位

**结论**：§4.3（编译器侧 PE 利用率优化，约 2500 字）作为 narrative A 主战场位置合理，但**字数体量略弱于 §4.2**（3000 字硬件数据通路），存在 reviewer 误判"主轴是硬件实现"的风险。**未到 FAIL 程度**，建议 Phase 5 在写作时通过强化 §4.3.5（与 MAERI/Eyeriss-v2 取舍对比）和 §4.3.1（PE 利用率塌陷量化分析）来加重 §4.3 篇幅，把 §4.3 拉到 3000 字以上。

**§4.3 5 个子节布局质量**：
- 4.3.1 量化分析：必要的 motivation 铺垫
- 4.3.2 Ky-fold 数学推导：narrative A 核心 claim 1，公式编号 4.x
- 4.3.3 S2D 等价变换：narrative A 核心 claim 2，公式编号 4.y
- 4.3.4 联合触发：C2.3 兑现，避免 reviewer 问"两个 trick 怎么协同"
- 4.3.5 与 MAERI/Eyeriss-v2 取舍：**narrative A 与 §B 路线的关键防御**，必写

**reviewer B（算法派）假想质疑**："Ky-fold 是不是只是 im2col 的一种？S2D 是不是只是 Pixel-Shuffle 在硬件上的复用？"
**回应**：§2.5（基础变换）→ §4.3.2/4.3.3（数学推导）→ §6.2.2（Pixel-Shuffle 谱系诚实标注）三处布防。措辞已用 lessons-learned 第 7 条要求的"据已知文献..."而非"first to..."，避免 over-claim。

---

### 3. §1.2.3 + §3.4.3 + §5.7.2 prior art 三层防御

**结论**：Alwani / Kang / Liu 三处威胁项的**三层防御**结构经得起审稿挑战，差异化口径具体到一句话级（行级 vs 跨层 fused / 行级 vs 整图 activation tile / 单核 layer-serial vs 跨层 pipelined）。

| 威胁项 | 第 1 层（§1.2.3 综述定位） | 第 2 层（§3.4.3 差异化论证） | 第 3 层（§5.7.2 数据兑现） | 防御质量 |
|---|---|---|---|---|
| Alwani Fused-layer@MICRO'16 | 列入综述并明示工作范围 | "FLUX_CNN 是单核 layer-serial 共用硬件 vs Alwani 多层硬件流水绑定" | 横向对比表（资源 + 灵活性） | ✅ 经得起挑战 |
| Kang AoCStream@arXiv'22+Sensors'23 | 列入综述并明示 buffer 容量线性于图宽 | "FLUX_CNN 行级粒度 strip_rows × W_IN vs Kang 整图 activation tile 粒度" | 横向对比表 | ✅ 经得起挑战 |
| Liu Full-Stack@TNNLS'21 | §1.2.3 + §1.2.5 工具链综述列入 | "FLUX_CNN 单核 layer-serial vs Liu 跨层 pipelined 多 block" | §5.7.2 + §5.7.1 工具链对比 | ✅ 经得起挑战 |

**reviewer C（应用派）假想质疑**："你的行级行环跟 Kang AoCStream 'streaming line-buffer' 有什么实质区别？都是流式啊？"
**回应**：§3.4.3 已布防——"buffer 容量 = activation tile（线性于图宽）vs 行级粒度（strip_rows × W_IN）"。CHECK-9 标记需写作时落到具体段落，paper-literature-scout 已论证差异化要点。

**潜在风险**：CHECK-6（S2D 在加速器领域的引用谱系，Pixel-Shuffle/Sub-pixel）是 narrative A 主轴的"软肋"——如果 scout 补查发现某 FPGA 工作已经在 stride 卷积上做过 phase folding，C2.2 novelty 强度需要从"中-强"降到"中"，§6.2.2 措辞需进一步加诚实标注。**该风险在大纲已用 CHECK-6 + §6.2.2 "诚实标注"双标记，未到 FAIL**。

---

### 4. 整体定位连贯

**结论**：§1.3.3（5 创新声明）→ §4.3+§4.2+§4.5（5 实现章节）→ §5.5+§5.7（数据兑现）→ §6.2（5 创新点回应）一气呵成。**叙事一致性检查表已自验证 5 创新声明全部映射到主章节 + 验证章节**，无悬空 claim 也无 unsupported promise。

| 链条节点 | 状态 |
|---|---|
| §1 抛 5 条 claim → §4 5 个主章节展开 | ✅ 1-to-1 对应 |
| §4 实现 → §5 数据兑现 | ✅ §5.5.1（Ky-fold/S2D 三模式）/ §5.5.2（5.05×）/ §5.5.3（多核加速比）/ §5.3（51-case bit-exact）逐一兑现 |
| §1 promise → §6 收束闭环 | ✅ §6.2 5 创新点对应 §1.3.3 5 主要贡献，描述角度不同但 1-to-1 |
| 诚实陈述承接链 | ✅ Fmax 68 MHz / DSP 推断率低 / 1-DDR BW bound / Mesh PoC 未完成 4 处都有 §5（数据）→ §6.3（路径）双承接 |

**reviewer 综合质疑**："Intro 的 promise 在正文是否真的兑现？"
**回应**：本大纲提供"贡献-章节映射表"+ "文献-章节映射表"+ "叙事一致性检查表"三张表自验证，覆盖 24 条贡献全部归位 + 35 条文献全部有归属章节 + 5 创新与 5 创新点 1-to-1。

---

### 5. claim 强度自评一致

**结论**：大纲在每章描述中**显式继承**了 contributions.md 的强度评级，措辞 calibration 一致，无 over-claim 也无 over-hedging。

| 创新点 | contributions.md 强度 | 大纲措辞 | 一致性 |
|---|---|---|---|
| §1.3.3 创新 1 / §6.2.1 Ky-fold | C2.1 偏强 | "据已知文献，纯编译器侧、零硬件代价的 Ky 维折叠到 *C*ᵢₙ 的 PE 利用率优化方案在 FPGA streaming CNN 加速器中较少见" | ✅ 完全继承 lessons-learned 第 7 条要求 |
| §1.3.3 创新 2 / §6.2.2 S2D | C2.2 中-强 | Patch 5.05× / 整网 1.87× 实测数据撑 + "诚实标注与 Pixel-Shuffle/Sub-pixel 在超分辨率领域的引用谱系关系" | ✅ 强度合理 + 谱系标注完整 |
| §1.3.3 创新 3 / §6.2.3 行环行流水 | C1.2 中-强 + C1.3 中-强（合并） | "与 Alwani/Kang/Liu 三处近邻 prior art 的差异化在第 3 章已论证" | ✅ 不夸 |
| §1.3.3 创新 4 / §6.2.4 多核 W 切片 | C3.4 中-强 | 引"halo 计算冗余 + 跨核 SRAM 直送 M2 push" | ✅ 不夸 |
| §1.3.3 创新 5 / §6.2.5 端到端编译流 | C2.4 中 + C3.2 中（合并） | "host AXI-Lite 写从 ~50/层降到 4/层" 工程亮点 | ✅ 工程级 claim 不混淆为架构创新 |

**无 over-claim 检查**：未见"首次 / 独有 / SOTA / 显著优于"等无支撑强词。
**无 over-hedging 检查**：Patch 5.05× / N=4 2.51× 加速等明显优势没被措辞削弱。

---

### 6. Reject 风险

| 风险类型 | 状态 | 说明 |
|---|---|---|
| "X+Y 拼凑" | ✅ 低 | narrative A+B 在 §3.5 单独立"编译器与硬件协同设计原则"小节作为接合点（params.py 单源 + cfg 派生 + s2d_eff 视角对齐）。reviewer 不易把 narrative A 与 B 视为平行拼凑 |
| "缺 SOTA 比较" | ✅ 低 | §5.7 三类对比都安排：5.7.1 FPGA streaming（Angel-Eye/Snowflake/fpgaConvNet/Caffeine 同器件横向）+ 5.7.2 三处威胁项数据兑现 + 5.7.3 ASIC 数量级差距与工艺归一化 |
| "[CHECK] 太多影响 claim" | ⚠ 中（可接受） | 19 处 [TBD]/[CHECK] 中，影响 claim 强度的关键 3 处：CHECK-6（S2D 谱系，已委托 scout）/ CHECK-5（Ky-fold 单独使能数字）/ CHECK-7（baseline 整网 MAC%）。**Phase 5 之前必须补完，否则会拖累 §1.3.3 创新 1/2 的强度**。但当前 Phase 2 大纲阶段不算阻断 |
| "Mesh PoC over-claim" | ✅ 低 | TBD-8 + §6.3.2 已明示 Mesh + AXIS NoC PoC 进 future work / 附录，不进 main contributions，避免半成品被 reviewer 抓 |
| "Fmax 68 MHz 藏起来" | ✅ 低 | §5.4.3 + §5.5.5 + §6.3.1 三处诚实陈述链完整，FPS 双标策略（100 MHz ceiling + 68 MHz actual）已规划在 TBD-7 |

---

### 通过-失败 总结

**通过**：
- narrative A+B 选择合理，主轴强度撑得起 §4.3 主章定位
- prior art 三处威胁三层防御到位（综述 + 论证 + 数据），差异化口径具体
- §1 promise → §4 实现 → §5 数据 → §6 收束闭环完整，24 条贡献全部归位
- claim 强度继承 contributions.md，措辞 calibration 一致，无 over-claim 无 over-hedging
- 三类 reject 风险（X+Y 拼凑 / 缺 SOTA / Mesh over-claim）均已布防

**未失败**：无 claim 过强 / 无 prior art 应对漏 / 无 narrative 切换混乱 / 无 over-hedging 削弱定位

**软肋（可接受范围内）**：
- §4.3 字数（2500）略弱于 §4.2（3000），建议 Phase 5 写作时把 §4.3 拉到 3000+ 字加重 narrative A 主轴
- CHECK-6 / CHECK-5 / CHECK-7 三处影响 claim 强度的数据 / 谱系待补，**Phase 5 之前必须解决**，否则进 Phase 7 仍未补会触发 reviewer 质疑

---

### 修订建议

无强制修订（PASS）。以下为 Phase 5 写作时的**软建议**，不影响 Phase 2 大纲落定：

1. **§4.3 篇幅微调**：把 §4.3.1（PE 利用率塌陷量化分析）扩到 600 字 + §4.3.5（与 MAERI/Eyeriss-v2 取舍）扩到 600 字，把 §4.3 整节从 2500 字拉到 3000+ 字，体量上压过 §4.2，凸显 narrative A 主轴地位。
2. **CHECK-6 优先级**：S2D 与 Pixel-Shuffle/Sub-pixel 谱系是 narrative A 主轴的关键防御，建议在 Phase 3（段落级骨架）展开前由 paper-literature-scout 补查并落到 §1.2.4 / §4.3.3 / §6.2.2 三处的具体措辞。
3. **§5.7.1 同器件对比表**：CHECK-7 标记的 5 处文献对照数据（baseline 整网 MAC% / 同器件 Fmax / Angel-Eye SDP 支持 / 同器件资源 / verification 公开度）建议在 Phase 4（数据落锤）阶段补齐，否则 §5.7.1 会成为薄弱章。
4. **§4.3.5 取舍对比的具体口径**：与 MAERI / Eyeriss-v2 的取舍对比是 narrative A 与 literature.md §B 可重构 PE 路线的关键差异化，Phase 5 写作时建议落到"硬件可重构 NoC 开销 vs 编译器侧零硬件代价"具体一段，避免泛泛而谈。
