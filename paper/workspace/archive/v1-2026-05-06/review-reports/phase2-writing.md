# 写作评审报告 Phase 2

## 第 1 次评审

### 判定
PASS

### 评审范围
- 主产出：`paper/workspace/outline.md`（397 行；8 章 IEEE 会议风格 + narrative A 主轴 / B 支撑混合）
- 上一阶段产出：`paper/workspace/literature.md` / `paper/workspace/contributions.md`
- 历史经验：`paper/workspace/lessons-learned.md`（重点关注 lesson "[CHECK] 总数声明禁分类裁剪" 在 outline 待决清单中的执行情况）

评审重点（章节齐全 / 章节顺序逻辑 / narrative 一致性 / 章节衔接 / 标题措辞 / Related Work-Evaluation 布局），不审事实正确性、不审新颖性强度。

### 章节顺序逻辑性

8 章顺序（Introduction / Background and Motivation / Related Work / Architecture / Compiler Optimizations / System Integration / Evaluation / Conclusion）符合 IEEE 工程会议主流叙事路径：

- **§1 → §2 → §3** 是经典的 "what / why / who-already-tried" 链，§2 Background 为 §3 Related Work 提供 dataflow taxonomy 术语，§3 为后续 §4/§5 的 contribution 章节做 prior art 防御铺垫，逻辑闭合。
- **§4 Architecture → §5 Compiler Optimizations → §6 System Integration** 三章按"硬件骨架 → 编译器侧 narrative A 主章 → 完整系统"递进，符合 narrative A 主轴下"硬件保持简洁 / 编译器填满 PE / 落到完整系统"的论证链。
- **§4.5 Loop-Nest Realization → §5 Compiler** 的强依赖在 §IX 章节依赖图里显式声明，避免 §5 突然引入 7 层嵌套术语造成读者断点。
- **§7 Evaluation → §8 Conclusion** 的 quantified-validate-then-收束 流程标准。
- 没有"前向引用"陷阱：§5 Compiler 用到的 PE 利用率口径在 §2.2 motivation 章已经引入；§6 System Integration 用到的 row-ring 在 §4.3 已经定义；§7 Evaluation 引用的所有 claim 在 §1.4 contributions 中预申明。

### 章节粒度均衡

预估篇幅分布：§1 (1.5p) / §2 (1p) / §3 (1.5p) / §4 (1.5p) / §5 (2p) / §6 (2p) / §7 (2.5-3p) / §8 (0.5p)，总计约 12-12.5 页（IEEE 会议双栏，元信息标的目标是 8-10 页正文 + 0.5-1 页 references）。

- **粒度均衡度**：合理。narrative A 主章 §5 (2p) 和数据章 §7 (2.5-3p) 是两个篇幅高峰，分别承担"机制论证"和"数据验证"双引擎，符合工程论文常见配重。
- **总篇幅超标提示（轻微）**：12 页 vs 8-10 页 target 有约 2-3 页 buffer 压力。Writer 在元信息中已经标注 [TBD] 期刊版可扩到 14-16 页 + 章节内多处 [TBD: 视篇幅压力定]，说明取舍预案已经留好。**这是预估值，不是结构问题，不影响 PASS。**
- §2 (1p) / §8 (0.5p) 是最轻的两章，但都是功能性章节（motivation 铺垫、conclusion 收束），不需要篇幅高，没有"章很轻"的失衡。

### narrative 一致性

混合 A+B narrative 在 8 章里有清晰主线：

- **§27-39 行**显式陈述 narrative 选择（A 主轴 + B 支撑），并给出"为什么不选 C / D"的反向取舍理由。这种显式陈述对 Phase 3 段落骨架起草是有价值的锚点。
- **§253-275 行 贡献-章节映射表** 给每条 contribution 标了 narrative 角色（"narrative A 核心" / "narrative B 主推" / "narrative A 实现细节" / "system 完整性" 等），17 条贡献全部归位，0 条遗漏。这是 narrative 一致性的强证据。
- **§277-298 行 文献-章节映射** 给 14 篇核心 prior art 标了威胁度 + 主对位章节 + 应对策略，3 处高威胁 prior art (Alwani / Kang / Liu) 在 §3.4 + §7.6 双层精细差异化。
- **§313-339 行 章节依赖图** 给出 5 条阅读路径（审稿人快读 / 架构 reviewer / 编译器 reviewer / system reviewer / 可独立阅读章节），证明 narrative 链条可被多角度审稿人跟住。

读者从 §1 Introduction 读到 §7 Evaluation 能跟住"主张（PE 利用率提升 + row-streaming 必要性）→ 方法（Ky-fold/S2D + row-ring + 5 模块去中心化）→ 验证（22-case + multi-core + head-to-head）"链条。

### 章节衔接

每章描述均预设了与前后章的衔接点：

- §1.4 Contributions 的 5 条贡献 → §7.2-§7.5 quantified data 一一映射（在 §301 行显式声明）
- §2.3 末尾要点明"为什么不用 Alwani-style layer fusion 解决 streaming 问题" → 预告 §3 Related Work 展开（§89 行）
- §4.5 Loop-Nest Realization → §5 Compiler 的形式化框架基础（§332 行强依赖声明）
- §6.4 Multi-Core W slice 强依赖 §4.3 Streaming Row-Ring（§333 行）
- §7 首段口径声明（100 MHz target vs 68.4 MHz Fmax）→ §7.7 Future Work 修复路径（§223 行）

无明显"各章自说自话"的结构断层。

### 标题措辞

8 章标题全部清晰、避免 buzzword、5 秒可判：

- "Background and Motivation" / "Related Work" / "Architecture" / "Compiler Optimizations for PE Utilization" / "System Integration and Toolchain" / "Evaluation" — 全部用功能性 / 学术化措辞，无 "smart" / "efficient" / "novel" 这类自夸词（与 lessons-learned `feedback_no_abstract_names` 经验对齐）。
- §5 标题副词 "for PE Utilization" 直接锁定 narrative A 主张，比单写 "Compiler Optimizations" 更精确。
- §6 标题 "and Toolchain" 把 PyTorch 编译栈 + DMA + 多核 wrapper 都包进来，避免 "System" 太宽泛。
- 子节标题（如 §4.3 "Streaming Row-Ring Datapath with Bidirectional Credit Backpressure" / §6.4 "Multi-Core Scaling: W-Slice with Computed Redundancy Halo"）信息密度高但不冗长，可读。

3 个候选论文标题（§17-21 行）措辞均合规；候选 2（narrative A+B 平衡）的"Compiler-Driven PE Utilization"是 narrative A 直白翻译，没有过度营销。

### Related Work / Evaluation 布局

**§3 Related Work（1.5 页 / 6 节布局）**：

- §3.1 Spatial Array Accelerators / §3.2 Hardware-Reconfigurable PE Utilization / §3.3 Compiler Loop-Nest Co-Design / §3.4 Streaming Line-Buffer Accelerators / §3.5 Quantization and Fusion / §3.6 Positioning of FLUX_CNN — 6 节覆盖了 literature.md A-G 类中的 A/B/C/D/E/F 6 类（G 类是 dataflow taxonomy / Roofline，已并入 §2.1 / §2.3 motivation 章，不重复在 §3）。**布局合理，未压缩也未拉伸。**
- 3 处 prior art 威胁（Alwani / Kang / Liu）均在 §3.4 单独差异化（§114-116 行），且 Liu 在 §7.6 head-to-head 比较表二次出现（§116 行），双层防御。
- §3.6 Positioning of FLUX_CNN 用一段总结表把 5 维度 placement 钉死（dataflow / PE-utilization 路线 / streaming granularity / multi-core 切分维度 / 编译栈 scope），是 §3 章末的有力收束。

**§7 Evaluation（2.5-3 页 / 7 节布局）**：

- 7.1 Experimental Setup / 7.2 PE Utilization / 7.3 Resource and Fmax / 7.4 End-to-End Latency / 7.5 Multi-Core Scaling / 7.6 Comparison with Prior Art / 7.7 Discussion: Known Limitations and Future Work
- baseline 比较（§7.6）/ ablation（§7.2 PE 利用率即是 fold on/off ablation）/ 端到端 case study（§7.4 ResNet-18 风格 11 层 chain）三类 evaluation 都有专节。
- §7.7 Known Limitations 单独成节是个亮点——主动暴露 Fmax 68.4 MHz 未达 100 MHz target / Cout<16 不优化 / Pooling DW 未做，符合工程会议审稿人对"诚实标注弱点"的预期。

### 通过原因

1. 8 章顺序逻辑闭合，无前向引用陷阱，无章节空洞。
2. narrative A+B 显式陈述 + 17 条 contribution 100% 归位 + 14 篇核心 prior art 全部对位 + 3 处高威胁 prior art 双层防御。
3. 章节标题全部学术化、无 buzzword、5 秒可判。
4. §3 Related Work 6 节布局覆盖 A-F 6 类，§7 Evaluation 7 节布局含 setup/ablation/baseline/case-study/limitations 五件套。
5. 章节衔接预设清晰（依赖图 + 5 条阅读路径 + 强弱依赖标注）。
6. [CHECK]/[TBD] 描述清晰可执行（行动 + 关联章节 + 来源），符合 lessons-learned 的"穷举计数"原则（§347 / §367 行均给出 grep 总数）。

### 给 Writer 的修订建议（轻微，不影响 PASS）

仅 3 处轻微建议，Writer 可在 Phase 3 段落骨架起草时顺手处理：

1. **§307 行措辞自相矛盾（轻微）**：原文写 "**18 条 contribution 全部归位**，无 [TBD: 是否要写] 标记"，但 §273 行明确说"实际 17 条 = C1.1-C1.5 / C2.1-C2.5 / C3.1-C3.7；主 Agent 提示词的'18 条'为 off-by-one"。建议把 §307 行的 "18 条" 改为 "17 条"，与 §273 / §392 行保持一致，避免读者第二遍读到 "18 条" 时困惑。

2. **§347 行括号嵌套描述偏复杂（轻微）**：原文 "全文 grep `[CHECK:]` 共 **14 处**（章节内嵌 [CHECK] 与下表条目去重后归并）。下表列 13 项行动条目（其中 1 处 §4.4 parf BRAM 与 §7.3 的资源数据可视为同一行动）" — 三层嵌套口径（grep 总数 / 表项数 / 合并项）容易让读者抓不到主数。建议改为分两行：第 1 行只报 grep 总数 14 处，第 2 行说"为可执行性，下表合并为 13 项行动条目"。

3. **元信息 §10 行候选会议表述（轻微）**：原文 "FPGA / FCCM / TCAD / TVLSI / TRETS 优先" 列了 5 个选项；contributions.md §7.1 narrative A 推荐的是 "FCCM / TCAD / TVLSI / TRETS（FPGA 系工程偏重）；ASPLOS 也可"。两份文档候选会议清单不完全一致（contributions 多了 ASPLOS，少了 FPGA Symposium）。建议在 outline 元信息处加一句 "（与 contributions.md §7.1 narrative A 推荐口径对齐时，FCCM/TCAD/TVLSI/TRETS 为优先候选；FPGA Symposium 与 ASPLOS 为备选，待用户决定）"。

### 提示（不属于写作评审范围，仅供主 Agent 参考）

- §347 行 "[CHECK] 14 处" 的总数声明是否真等于全文 grep 数，建议 tech 评审复核（lessons-learned 第 8 条要求 grep 实测）。**写作评审已确认表达方式合规，但实际计数准确性属事实层。**
- §10 行预估篇幅 "8-10 页双栏正文" 与各章预估累加 12-12.5 页之间的差距是否需要在元信息处主动标注 "若超 10 页，§5/§7 是首选压缩点"，建议 tech 评审根据数据完整度判断。
