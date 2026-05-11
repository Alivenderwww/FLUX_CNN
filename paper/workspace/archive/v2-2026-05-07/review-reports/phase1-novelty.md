# 新颖性评审报告 Phase 1 v2

## 第 1 次评审

### 判定：PASS

---

### 评审范围声明

按 _AGENT_ROSTER.md "灵魂层 vs 格式层"分工，本评审**只管灵魂层**：

- ✅ 评 claim 强度、与 prior art 差异化、整体定位连贯性、reject 风险
- ❌ 不评中英空格 / 残留英文术语 / 章节编号 / 引用句式（writing reviewer 管）
- ❌ 不评数据真伪 / 代码追溯 / [CHECK] 标记位置（tech reviewer 管）
- ❌ 不联网检索（Phase 0 v1 已 PASS）

复核目标：让"假想审稿人"无法用 incremental / overclaim / 漏文献三招否掉本工作。

---

### 一、claim 强度全文一致性评估

#### 强 claim 词汇盘查（"first" / "最早" / "独创" / "唯一" 类）

通读 23 条贡献 + 一句话定位 + 论文取舍建议，**强 claim 控制良好**：

| 检查项 | 出现位置 | 评估 |
|--------|---------|------|
| "first to..." / "最早提出" | **零处** | 优秀 |
| "独创" / "独有" | **零处** | 优秀 |
| "唯一" | **零处** | 优秀 |
| "据已知文献..." 软化措辞 | C2.1 / 取舍建议 Q1 | ✅ 写法合规 |
| "较少见" 软化措辞 | C2.1 | ✅ 安全 |
| "**直接借用 NVDLA 的命名**" | C1.5 ↔ literature.md NVDLA 条 | ✅ 主动承认引用谱系，反向防御 reject |

C2.1 Ky-fold 已正确避开"first to"硬 claim，改用"据已知文献..."—— 这是 Phase 1 v1 reviewer 经验沉淀的直接体现，writer 学到位。

#### 自评强度分布合理性

| 自评强度 | 数量 | 贡献 | 评估 |
|---------|------|------|------|
| 偏强 | 1 | C2.1 | ✅ 合理（核心 novelty 候选，软措辞 + 谱系待查 [CHECK] 已配） |
| 中-强 | 3 | C1.2 / C1.3 / C2.2 / C3.4 | ✅ 合理（C1.3 streaming row-ring + C2.2 S2D 是核心叙事） |
| 中 | 7 | C1.1 / C1.4 / C1.5 / C2.3 / C2.4 / C2.5 / C3.2 / C3.6 | ✅ 工程级与机制级兼有，未夸大 |
| 弱-中 | 3 | C1.6 / C3.1 / C3.5 | ✅ 诚实（"循环嵌套完整性陈述"、"系统集成必写"、"reproducibility 加分项"——不是核心新颖性） |
| 弱 | 1 | C3.3 | ✅ "1 段带过即可"——分寸感到位 |
| N/A | 7 | C4.1–C4.7 | ✅ 实测数据无 novelty 评分，符合 |

**关键发现**：自评分布形成清晰金字塔——只有 1 条"偏强"，1 条"中-强"是核心叙事，其余都是中或弱，整体非常**克制**。这种克制是对 reject 风险（"X+Y 拼凑"、"overclaim"）的最佳预防。

#### claim 一致性检查

- 一句话定位（line 14）vs C2.1 + C1.3 + C3.4：**一致**——三大支柱（row-ring streaming / 编译器 PE 利用率优化 / 多核 W slice）三处叙事吻合
- 一句话定位"sim 实测 2.51× 加速"对应 C4.4 N=4 + S2D vs N=1 + S2D = 596K / 354K ≈ 1.68×（同 baseline 内对比）或 vs N=1 baseline = 1115K / 354K ≈ 3.15×
  - **疑点**：2.51× 这个数字在表 C4.4 / C4.5 / C4.6 中均找不到精确出处（最接近是 N=4 + S2D vs N=1 baseline = 1115K / 354K ≈ 3.15×，或 N=4 + S2D + 4DDR PoC vs N=1 baseline ≈ 5.7×）
  - 但这是**数字真伪**问题，归 tech reviewer 管。本评审仅记录提请 tech 复核

---

### 二、prior art 威胁完整性评估

#### 已识别威胁项对位检查

| 威胁项 | literature.md 出处 | contributions.md 对位 | 差异化论证强度 |
|--------|-------------------|----------------------|----------------|
| Alwani Fused-layer @MICRO'16 | §C ⭐第 4 次启动新增 | C1.3 末段直接点名 | ✅ "row-level 而非整图或多层 fused，硬件不绑定层数" — 论证清晰 |
| Kang AoCStream @arXiv'22 / Sensors'23 | §C ⭐第 4 次启动新增 | C1.3 末段直接点名 | ✅ "退化到行级 vs buffer 容量 = activation tile" — 论证清晰 |
| Liu Full-Stack @TNNLS'21 | §C ⭐第 4 次启动新增 | 一句话定位 + 取舍建议 Q8 | ⚠️ **未在 C1.x / C2.x 任一条贡献的"与 prior art 差异化"段直接点名** |

**Liu Full-Stack 对位空缺细查**：Liu 2021 是 streaming + residual fusion 同时支持的 FPGA 工作（Arria 10 GX1150 / 1.3 TOP/s / 97% MAC 效率），与 C1.5 SDP 后处理 + residual fusion 直接对照。但 C1.5 的"与 prior art 差异化"行**未提 Liu**，只笼统说"工程一致性，单独看 novelty 中等"。

- **严重度**：中等（不上升到 FAIL，因为一句话定位与 Q8 已暗指 Liu 是 prior art，且 C1.5 自评强度已主动降到"中"）
- **建议修订**：C1.5 末行加一句"与 Liu et al.@TNNLS'21 的 layer-fusion + residual streaming 路线相比，本工作 SDP 在 layer-serial 单核场景中以更简的形式（无 layer fusion）实现同类 residual 通路融合"

#### 潜在 prior art 威胁补盘（按 literature.md §A/§B/§F 横扫）

| 文献（literature.md） | 是否在 contributions 显式对位 | 评估 |
|---------------------|-----------------------------|------|
| **§A Gemmini @DAC'21**（同 16×16 INT8） | ✅ 隐含在 C1.1 + C1.2 "去中心化 vs 中心 FSM" | ✅ 但**未在 C1.1 直接点名 Gemmini** — 这是论文里 reviewer 100% 会拿来比的最近邻同尺度对手，建议 C1.1 / C1.2 显式提及 |
| **§A NVDLA**（SDP 命名同源） | ✅ C1.5 隐含借用 SDP 命名（literature.md NVDLA 条说"FLUX_CNN 沿用 NVDLA 命名脉络"） | ⚠️ contributions.md C1.5 **未主动声明 SDP 命名借用 NVDLA** — 让审稿人自己挖会显得"藏"。建议 C1.5 加一句"SDP 命名沿用 NVDLA 谱系" |
| **§A Eyeriss-v1/v2**（OS vs RS dataflow） | ✅ C1.1 / C1.4 多处对位 | ✅ "PARF 颗粒度介于 per-PE 和全局之间" 是好的差异化论证 |
| **§A Simba @MICRO'19**（多 chiplet scaling） | ⚠️ contributions.md **完全未提 Simba** | ✅ 但合理 — Simba 是 chiplet-NoP 而非单板多核，与 C3.4 W slice 不直接竞争 |
| **§A TPU v1**（256×256 systolic） | 一句话定位隐含尺度对照 | ✅ 不需要在 C1.x 直接对位（数据中心 vs 端侧场景错位） |
| **§B MAERI @ASPLOS'18** | ✅ C2.1 直接对位 | ✅ "硬件可重构 vs 编译器侧重映射" 论证清晰 |
| **§B Eyeriss-v2** | ✅ C2.1 直接对位 | ✅ 同 MAERI |
| **§B Tangram @ASPLOS'19**（cross-layer pipelining） | ⚠️ contributions.md **完全未提 Tangram** | ⚠️ Tangram 的 row-level cross-layer pipelining 是 FLUX_CNN row-ring 未来扩展的直接对照（literature.md 自承"将来工作的直接对照"）。**Reviewer 可能问**："你的 row-ring 与 Tangram 的 row-level inter-layer 关系？" 建议 C1.3 末段加一句"当前为 layer-serial，Tangram@ASPLOS'19 的 row-level cross-layer 是 future work 方向" |
| **§B Buffets @ASPLOS'19** | ⚠️ contributions.md **完全未提** | ✅ literature.md 已说 FLUX_CNN 是"buffet 思想的去中心化 RTL 落地"——这是 C1.2 的有力对照。建议 C1.2 加一句"我们的 valid-ready + counter 模式可视为 Pellauer Buffets 思想的去中心化 RTL 落地" |
| **§B ShiDianNao**（OS dataflow 端侧 sensor coupled） | ⚠️ contributions.md **未直接提 ShiDianNao** | ✅ 但合理 — ShiDianNao 假设全图入片上，与 FLUX_CNN row-ring 大图场景错位 |
| **§D im2col / cuDNN @arXiv'14** | ✅ C2.1 隐含（Ky-fold "与 im2col 同源但更轻量"）但 contributions 未直说 | ⚠️ literature.md 明说 Ky-fold 是"轻量化 im2col 变体"——这是 C2.1 novelty 论证的关键反向锚点。**审稿人会立刻问**："这不就是 im2col 的子集吗？" 建议 C2.1 显式说"区别于 im2col 全展开的 Kx×Ky×Cin 内存炸裂，Ky-fold 仅折 Ky 保留 Cin 通道并行" |
| **§D Sub-pixel Convolution @CVPR'16**（S2D 谱系） | ✅ C2.2 已用 [CHECK] 标谱系待查 | ✅ 处理合规 |
| **§D Winograd Lu @FCCM'17** | ⚠️ contributions.md **未在 C2.x 中提及为何不选 Winograd** | ⚠️ "为什么不用 Winograd"是审稿人必问。literature.md 已给好答案（"Cin<16 PE 行不足，Winograd 不解"）。建议 Related Work 章节而非 contributions.md 处理 — **不算 contributions.md 缺陷** |
| **§F TVM / Interstellar / VTA**（编译器对照） | ✅ C2.4 提及 ONNX 中间表示对比；VTA 通过一句话定位隐含 | ✅ 处理合理 |
| **§G EIE 稀疏路线** | ✅ contributions 未涉稀疏路线 | ✅ literature.md 已给"正交方向" — 不算缺陷 |

**§F 去中心化路线的 prior art**：literature.md §F 标题虽是"编译器/ISA-level 切片"，但本工作"去中心化握手"的对照对手实际散在 §A（Gemmini / NVDLA / VTA 是中心 FSM 代表）。已盘查清楚，不重复列。

#### prior art 完整性结论

- 已识别威胁（Alwani / Kang）对位**清晰**
- Liu Full-Stack 对位**有空缺但 C1.5 自评已降中**——不上升到 FAIL
- Gemmini / NVDLA / Tangram / Buffets / cuDNN 是 5 处**未显式对位**的 prior art，但 literature.md 都已有对照线索，作为**修订建议**列出
- **未发现需要回 Phase 0 补查的漏文献**——literature.md 35 篇覆盖充分

---

### 三、整体定位连贯性

#### "本科毕业论文" 视角下能否撑起一篇

23 条贡献分布：
- 一类（架构层 C1.x）：6 条 — 硬件主干
- 二类（编译器层 C2.x）：5 条 — 优化 + 工具链
- 三类（系统集成 C3.x）：6 条 — DMA + 多核 + 参数 + profile
- 四类（实测 C4.x）：7 条 — 综合 + 仿真 + 多核加速

**判断**：
- ✅ 主线清晰：(a) 16×16 OS 阵列 + (b) row-ring streaming + (c) 编译器 PE 利用率（Ky-fold + S2D）+ (d) 多核 W slice — 四点形成一个完整故事
- ✅ 主角配角分布合理：C1.3 + C2.1 + C2.2 + C3.4 = 4 条核心新颖性贡献占 17%；其余 19 条是支撑工程
- ✅ 配角不掩盖主角：C3.3 / C3.5 / C1.6 自评弱 — 体现"一段带过"的分寸感
- ✅ 体量合适：23 条对本科毕业论文完全够用，对会议论文则按 Q1 / Q11 / Q12 取舍

#### "X + Y 拼凑" 风险

最容易被审稿人 reject 的场景是"两个 idea 各自不够强、硬拼"。盘查：

- **C1.3 row-ring streaming + C2.x 编译器优化**：一句话定位明确说"硬件保持简洁、编译器侧填满 PE"——这本身就是**一个统一论点**而非两个 idea 拼凑。✅ 安全
- **C1.x 单核 + C3.4 多核**：N=4 加速 1.68× 是被 1-DDR BW 限制（C4.5 mac_pipe% 36% → 63.5% 解墙数据），多核章节有"诚实陈述 BW 是瓶颈"——这是**limitation analysis 而非额外新颖性**。✅ 处理诚实
- **C2.1 Ky-fold + C2.2 S2D**：C2.3 显式说"两者可叠加 / 联合触发"是工程接合点 —— ✅ 处理合规

**结论**：**无 X+Y 拼凑风险**。

#### "narrative 切换" 风险

23 条覆盖架构/编译器/系统/实测四层 — 是否会让 reviewer 觉得"前后不在说一个故事"？

- 一句话定位先抛"端侧 INT8 加速器 + Ky-fold/S2D + row-ring streaming + N=4 SMC"四个关键词
- 后续 23 条按四层组织，每层都映射回这四个关键词的一部分
- ✅ Narrative 一致

---

### 四、强弱评级合理性 + 处理建议

#### 弱-中评级贡献的处理建议

| 贡献 | 自评 | 建议处置 | 理由 |
|------|------|---------|------|
| C1.6 7 层循环嵌套 | 弱-中 | **保留 + 改写为完整性陈述** | literature.md Ma FPGA'17 / Interstellar 已建立"7 层循环"的形式语言，C1.6 是该语言的具体落地点 — 论文里 1-2 段说明 cfg 寄存器映射即可，不必单独成节 |
| C3.1 AXI/DMA 集成 | 弱-中 | **保留 + 视会议偏向决策**（Q11） | FPGA / FCCM 偏好工程细节可单写一节；ISCA / HPCA 偏好架构则 1 段带过 |
| C3.3 done sticky + 双口 cfg_regs | 弱 | **保留 + 1 段带过** | writer 已自评 "1 段带过即可" — ✅ 处置合规 |
| C3.5 params.py 单源 | 弱-中 | **作为 reproducibility 加分项放附录** | writer 已自评 — ✅ |

**所有弱评级贡献保留 + 降权处理是正确判断**。无需砍掉。

#### 偏强评级贡献的反向 sanity check

| 贡献 | 自评 | sanity check |
|------|------|-------------|
| C2.1 Ky-fold | 偏强 | ✅ 措辞用"据已知文献"软化 + Q2 标 [CHECK] 谱系待查 + 与 MAERI/Eyeriss-v2 显式对位 — 三重防御充分 |

---

### 五、Reject 风险预判

#### Risk 1: "X+Y 拼凑"
- **评估**：低
- **原因**：一句话定位把 row-ring + 编译器优化 + 多核三件事统一到"硬件简洁 + 编译器填 PE"主论点

#### Risk 2: "缺 SOTA 比较"
- **评估**：中
- **风险点**：
  - C4.4 ResNet11 N=4 + S2D = 564 fps @ 100 MHz 假设 — **未与 Angel-Eye / Snowflake / VTA / DPU 任一同器件 baseline 直接数字比较**
  - C4.1 单核 36,942 LUT / 82 DSP — **未与同尺度对手（VTA 256-PE on Zynq）资源占用对比**
- **缓解**：literature.md 第 5 次启动已识别"FLUX_CNN 实测 vs 文献对照数据"为待补 [CHECK] 4 处 + 5 处。**论文写作阶段必须补对照表**，否则 FCCM / FPGA reviewer 会直接 reject
- **建议**：在 contributions.md 取舍建议章节加一条"Risk: SOTA 对照表必须在 evaluation 章节给出，至少覆盖 Angel-Eye / VTA / DPU 同器件 INT8 数字"
- **当前判定**：不上升到 FAIL — 这是 evaluation 章节工作而非 contributions 列表工作

#### Risk 3: "[CHECK] 太多影响 claim 可信度"
- **评估**：低-中
- **数字盘查**：6 [CHECK] + 19 [TBD]（writer 自报）— 实际 grep contributions.md 得 [CHECK] 出现 9 次（C2.1/C2.2/C4.2/C4.3/C4.6/C4.7 + 关键性能数据表 3 处），[TBD] 出现 11 次
  - **轻微偏差**：writer 自报 6 + 19 = 25 vs 实际 9 + 11 = 20。属计数误差，归 writer 自审 — 不影响新颖性判定
- **风险点**：
  - C2.2 S2D 的"在加速器领域引用谱系" [CHECK] —— 这是**核心 novelty 候选**的依据待查，**最关键** [CHECK]
  - C4.7 三模式 PE 利用率对比 [CHECK] —— evaluation 数据待补，影响 C2.1 + C2.2 强度论证
- **缓解**：Q2 已显式 dispatch 给 paper-literature-scout — ✅ 处置合规
- **当前判定**：[CHECK] 集中在数据层（C4.x）而非 claim 层（C1-C3.x），属于"诚实留白"非"过度声明前置"

#### Risk 4: "narrative 切换"
- **评估**：低
- **原因**：见上节"整体定位连贯性"

#### Risk 5: "工程量太重，architecture novelty 不足"
- **评估**：中
- **风险点**：23 条中 11 条是工程级 + 实测级（C2.4 / C2.5 / C3.1 / C3.2 / C3.3 / C3.5 / C3.6 / C4.1-C4.7）—— architecture novelty 集中在 C1.3 + C2.1 + C2.2 + C3.4 这 4 条
- **缓解**：FPGA / FCCM / TCAD 接受"工程实证 + 实测 + 编译器栈"的论文气质（一句话定位也指向这条路线）；如目标 ISCA / HPCA 则需要把 C2.1 作为单节核心
- **建议**：保留现状 + 论文写作阶段按目标会议（Q1 / Q8）调权重
- **当前判定**：不上升到 FAIL — Q1 / Q8 已 dispatch 用户决策

---

### 六、通过 / 失败总结

| 评估项 | 结论 |
|--------|------|
| claim 强度全文一致性 | ✅ PASS（无强 claim 词，软措辞合规，自评分布金字塔） |
| prior art 威胁完整性 | ✅ PASS（已识别威胁对位清晰，5 处未显式对位的 prior art 列为修订建议而非 FAIL） |
| 整体定位连贯性 | ✅ PASS（四层主线统一，无拼凑/切换风险） |
| 强弱评级合理性 | ✅ PASS（自评分布合理，弱评级处理建议得当） |
| Reject 风险 | ⚠️ 中等但可控（SOTA 对照在 evaluation 阶段补；[CHECK] 集中数据层） |

**判定 PASS** —— writer 在 Phase 1 v2 表现出对 reject 风险的高度自觉性，软措辞 + 诚实自评 + 主动标 [CHECK] 三件套到位。

---

### 七、修订建议（非 FAIL，仅供 writer 在 Phase 2 outline 阶段或论文撰写阶段择优采纳）

| # | 严重度 | 位置 | 建议 |
|---|--------|------|------|
| 1 | 中等 | C1.5 末行 | 加一句对位 Liu Full-Stack@TNNLS'21 layer-fusion + residual streaming，明示 FLUX_CNN 是 layer-serial 单核场景下的更简实现 |
| 2 | 中等 | C1.5 内容描述 | 加一句"SDP 命名沿用 NVDLA 谱系"——主动承认引用，反向防御 reject |
| 3 | 中等 | C2.1 与 prior art 差异化 | 加一句对位 cuDNN im2col：本工作"轻量化 im2col 变体"，仅折 Ky 保留 Cin 通道并行，避免 im2col 全展开内存炸裂 |
| 4 | 中等 | C1.1 / C1.2 与 prior art 差异化 | 显式点名 Gemmini@DAC'21（同 16×16 INT8）—— 这是审稿人最可能拿来比的最近邻同尺度对手 |
| 5 | 轻微 | C1.2 末行 | 加一句"valid-ready + counter 模式可视为 Pellauer Buffets@ASPLOS'19 思想的去中心化 RTL 落地" |
| 6 | 轻微 | C1.3 末段 | 加一句"row-level cross-layer pipelining（Tangram@ASPLOS'19 思想）是本工作 row-ring 的 future work 方向" |
| 7 | 轻微 | 取舍建议章节 | 增补一条 "Risk: SOTA 对照表 — evaluation 章节必须给 Angel-Eye / VTA / DPU 同器件 INT8 数字直接对比，否则 FPGA 系审稿人会以缺 SOTA 比较 reject" |
| 8 | 轻微 | 一句话定位 | "sim 实测 2.51× 加速" 数字与 C4.4 / C4.5 / C4.6 表内任一项对不上（最接近是 1.68× / 2.11× / 3.15×），请 tech reviewer 复核数字出处或 writer 自审改正 |

**全部为非阻塞性建议**。建议 1-4 在 Phase 2 outline 之前选择性补；建议 5-8 可在论文写作中再处理。

