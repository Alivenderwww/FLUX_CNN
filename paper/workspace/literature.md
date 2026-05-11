# 文献综述材料 (Phase 0)

## 核验日志

### 第 5 次启动 — 第 1 轮修正（2026-04-30）

- **任务意图**：tech reviewer 对第 4 次启动产出 FAIL，本轮按 phase0-tech.md 列出的 6 个问题做最小改动修复
- **本实例工具表实测**：✅ `mcp__elicit__search_papers` 可用（前任已验证父进程已重启，本实例直接复用）
- **检索预算使用**：`mcp__elicit__search_papers` 共 **2 次**（≤10 次预算）
- **处理的问题**（编号对应 phase0-tech.md）：
  - **F1（严重，文档合规）— 计数偏差**：line 13 "替换引用 4 处" 与括号枚举 5 项不符 → 改为 "替换引用 5 处"；line 538 "DOI 错误修正 4 处" 与列表 6 项不符 → 改为 "DOI / 会议归属错误修正 6 处"，并在 line 547 改写"额外会议归属修正"避免与上行 Tangram 项重复计入
  - **F2（严重，文档合规）— [CHECK] 总数声明分类裁剪**：line 11 / line 54 / line 517 原写"81→5"是仅算"文献引用类"的小数 → 重写为分类计数 **15 处**（文献引用类 5 / S2D 谱系 1 / FLUX_CNN 实测 4 / 文献对照 5）；同时在"待补清单"补两类（FLUX_CNN 其他实测数据 + 文献对照数据）以与 grep 出的 15 处对位
  - **F3（中等，标记口径）— [CHECK]/[TBD] 口径冲突**：在调研元信息处增补"标记符号语义约定"段（[CHECK]=数据/事实待核验，[TBD]=路径/决策待定）；按此把 PE 利用率实测从 [TBD] 改归 [CHECK]，[TBD] 残留数从 3 → 2；line 22-27 历史日志保留并加注"已被第 5 次启动重新归类"
  - **F4（严重，引用真实性）— Kang AoCStream venue 不自洽**：跑 1 次 `mcp__elicit__search_papers query="AoCStream all-on-chip..."` → 真实情况 = arXiv:2212.11438 是 2022年12月 single-author preprint（Hyeong-Ju Kang，DOI 10.48550/arXiv.2212.11438），SS 把 venue 自动归类为 "Symposium on FPGA" 是内容标签非真实发表 venue；2023 期刊扩展版 (Kang & Yang) 发于 **MDPI Sensors**, vol.23 art.8104, **DOI 10.3390/s23198104**（SS 的 "Italian National Conference on Sensors" 也是误标）。entry header 改为 `Kang@arXiv'22 / Kang & Yang@Sensors'23`，DOI 行同时给出两版 DOI 并备注 SS venue 误标
  - **F5（轻微，文档合规）— ARM Ethos 双 [CHECK] 重复**：line 376 量化指标行的 [CHECK: ARM Reference Manual 引用] 删除，统一保留在 line 380 参考行（该项在 [CHECK] 总数中按"事项"算 1 处）
  - **F6（轻微，引用真实性）— Liu Full-Stack 作者格式**：跑 1 次 `mcp__elicit__search_papers query="Liu Full-Stack streaming CNN..."` → 真实第一作者是 **Shuanglong Liu**（不是 "Liu, Fan"——前一版把第二作者 Hongxiang Fan 误拼到第一作者名下）。entry header 改为标准 "Liu et al.@TNNLS'21" 格式，DOI 行注明完整作者列表
- **核验前后状态**：
  - [CHECK] 数量声明：5 → **15**（按 reviewer F2 改成分类计数，**实质未变**——只是把分类裁剪数字改回真实总数）
  - [TBD] 数量声明：3 → **2**（按 reviewer F3 把"PE 利用率实测"重新归类）
  - **修正引用 2 处**：Kang AoCStream venue 重写、Liu Full-Stack 作者格式
  - **删除引用**：0 处
- **本轮无法处理的问题**：无；所有 6 个问题均已处理
- **方法论备注**：F2/F3 的核心是"统计口径透明化"——把分类裁剪（让数字漂亮）改成分类穷举（让数字真实）；这与 lessons-learned 第 7 条（"不要把模型记忆当作可信值"）的精神一致——核验日志的"统计漂亮"和文献的"DOI 漂亮"都是同一类陷阱

---

### 第 4 次启动（2026-04-30，新实例 + 父进程已重启）

- **任务意图**：父进程重启后 `mcp__elicit__*` 工具进入子智能体工具表，按"批量核验任务清单 §批 1–§批 8"做最终核验
- **本实例工具表实测**：✅ `mcp__elicit__search_papers` 可用（首次调用 query="test" 返回正常 papers 字段），其他 `mcp__elicit__list_reports / get_report / create_report` 因本任务不需要未实测
- **检索预算使用**：`mcp__elicit__search_papers` 共 **18 次**（≤30 次预算）
- **核验前后状态**：
  - [CHECK] 数量：81 → 残留 **15 处**（**第 5 次启动按 reviewer 要求重算**——前一版 "5" 是仅算"文献引用类"的分类裁剪数字，非全文残留），分四类：
    - **文献引用类** (vendor doc / arXiv tech report，无标准 paper 索引，必由主 Agent 决定引用方式)：5 处 = NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report
    - **学术 prior art 谱系不清晰**：1 处 = S2D 在加速器领域的引用谱系
    - **FLUX_CNN 侧实测数据待补**（需要 project-analyst 实测）：4 处 = 实测 Fmax 下 GOPS / PE 利用率百分比 / "target vs Fmax" 口径标注 / 单层 vs 整网 MAC% 区分
    - **文献对照数据待联网核**（需要补做对照表跨工作核验）：5 处 = 各 baseline 整网 MAC% / 同器件其他工作 Fmax / Angel-Eye 是否支持 SDP / 同器件 Angel-Eye/DPU/VTA 资源 / 各工作 verification 公开度
  - [TBD] 数量：4 → **2**（**第 5 次启动按 reviewer F3 要求重新归类**：前一版的 "PE 利用率实测" 在 line 22/473/519/520 标记口径冲突，按"决策项 vs 数据项"约定，PE 利用率属数据项应标 [CHECK]，已归并到上方第三类；剩余 [TBD] 仅 = 目标会议、是否开源 这两个决策项）
  - **替换引用 5 处**：(1) Eyeriss ISCA'16 真实 DOI 与作者列表（之前 DOI 错、作者多了 Krishna）；(2) Tangram 会议归属（之前写 MICRO'19，真实是 ASPLOS'19）；(3) Pellauer Buffets DOI（之前完全错）；(4) fpgaConvNet TNNLS DOI（之前末尾数字错）；(5) Snowflake 会议（之前写 CVPRW'14，真实是 ISCAS'17 + arXiv 2017）
  - **修订数据 6 处**：He ResNet "top-5 3.57%" → "ensemble error 3.57%"; Aydonat DLA DOI 末四位修正; Lu Winograd 是 FCCM'17 单篇（非 FPGA'17 + FCCM'17 二选一）；Gemmini 论文出处更精确（arXiv 2019 / DAC 2021 fab + ESSCIRC 2021）；Interstellar DOI 末三位修正；VTA 默认 256-PE @ 100 MHz on Zynq（不是 16×16）
  - **新增 §C 三条 prior art**：Alwani Fused-layer MICRO'16 / Kang AoCStream FCCM'22 / Liu Full-Stack TNNLS'21 — Phase 0 §批 8 检索发现 row-streaming 任意 H×W 在文献中已有近邻 prior art，必须列入避免审稿人指出遗漏
  - **删除引用**：0 处（所有"经典"引用均经 Elicit 命中确认存在）
- **方法论备注**：本次不直接调用 check-citations skill（依赖 CrossRef/Semantic Scholar 联网），但 Elicit 检索本身使用 Semantic Scholar 后端（命中条目带 `ss-` 前缀），作用等价
- **剩余 [CHECK] 分类**（**第 4 次启动写法，已在第 5 次启动按 reviewer F2 要求重新算成 15 处**，详见上文新增分类；本节保留作历史记录）：
  - 5 条都是 **vendor doc（NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report 引用方式）** —— Elicit 未索引或仅索引衍生研究，必须由主 Agent 在 Phase 1+ 决定引用形式（vendor white paper 还是引衍生 paper）
- **剩余 [TBD]**（**第 4 次启动写法，已在第 5 次启动按 reviewer F3 重新归类成 2 处**，本节保留作历史记录）：
  - 目标会议未定（Phase 1 或主 Agent 决定）
  - FLUX_CNN 是否开源（取决于发表策略）
  - ~~FLUX_CNN 实测 PE 利用率百分比（等 project-analyst 实测）~~ → 第 5 次启动按数据项归类改为 [CHECK]

---

### 第 3 次启动（2026-04-30，新实例非 resume）— 工具表问题已修复

[历史记录保留——本节因第 4 次启动已使工具就绪，说明已过期]

- 在第 3 次启动时实测子智能体工具表无 `mcp__elicit__*`，根因是 sub-agent tools 白名单文件改动需父进程重启才生效
- 第 3 次启动产出："批量核验任务清单"（按主题 8 批），交由主 Agent 主线程跑核验或 fresh restart 后 resume 新 Writer
- 第 4 次启动确认父进程已重启 → 工具到位 → 完成核验

---

### 第 2 次 resume（2026-04-30）— 工具表问题已发现

[历史记录保留]

- 第 2 次 resume 时主 Agent 通知 Elicit MCP 已加载，但子智能体工具表实测无 `mcp__elicit__*` 系列
- 沉淀经验："MCP 工具的可用性必须以 Writer 自身工具表为准，不能信主 Agent 的口头确认"

---

## 调研元信息

- **调研方式（实际）**：**Elicit MCP（mcp__elicit__search_papers）+ 项目本地资料**
  - 第 4 次启动跑 18 次 Elicit 检索，每次 maxResults=5–15（默认 semantic 模式）
  - 检索后端：Semantic Scholar 全索引（200M+ papers）+ CrossRef DOI metadata
  - 无标准学术索引的 vendor doc / GitHub-only 项目（NVDLA / Xilinx PG / ARM Ethos）保留 [CHECK] 并注明"已检索：vendor doc 无 paper 索引"
- **完成时间**：2026-04-30（第 4 次启动 + 第 5 次启动修正）
- **总文献数**：35（A–G 七大类）
- **[CHECK] 残留**：**15 处**（详见核验日志第 4 次启动条目内分类，**第 5 次启动按 reviewer F2 要求改为分类计数**，前一版 "5" 是分类裁剪数字）
- **[TBD] 残留**：**2 处**（**第 5 次启动按 reviewer F3 要求归并**：目标会议 / 是否开源；PE 利用率实测属数据项已改归 [CHECK]）
- **标记符号语义约定**（第 5 次启动按 reviewer F3 增补）：
  - `[CHECK: 描述]` = 待联网/实测核验的**事实**，性质属"数据缺失或来源未确定"。例：FLUX_CNN 实测 PE 利用率百分比、文献对照数据、vendor doc 引用方式
  - `[TBD: 描述]` = 待用户/工程**决策**的事项，性质属"路径未定"。例：目标会议、是否开源、发表策略
  - 边界判定：若该项最终通过"测一下 / 查一下 / 引一下"就能确定 → 是 [CHECK]；若该项需要人决定才有答案 → 是 [TBD]
- **本地参考资料**：`README.md` / `STATUS.md` / `CLAUDE.md` / `docs/pe-fold.md` / `docs/slicing/README.md` / `docs/roadmap.md` / `model_analysis.md`

---

## 领域分类与本工作定位预判

CNN 加速器文献大致分两条主线：(1) **ASIC 系统脉动 / 可重构阵列**（TPU / Eyeriss / Gemmini / NVDLA / MAERI / Simba / Tangram），关注每个 PE 的数据流取舍（WS / OS / RS）、PE 利用率、片上互联可重构性；(2) **FPGA streaming / line-buffer 风格加速器**（fpgaConvNet / DnnWeaver / Snowflake / Caffeine / Angel-Eye / AoCStream / Fused-layer 等），关注端侧 / 实时推理、片上 SRAM 受限下的层间 dataflow、AXI/DMA 集成、HLS vs 手写 RTL 取舍。FLUX_CNN 横跨两条线：硬件层面是 ASIC 风格的 16×16 spatial array（Cout 列广播 + Cin 行累加，本质是 **output-stationary（OS） + 列广播激活**的混合数据流），但系统集成层面是 FPGA streaming 风格——**整图不必装进片上 SRAM**，靠 row-ring 反压让 IDMA / 计算 / ODMA 三阶段并发，单次 `start` 跑完任意 H×W。

FLUX_CNN 与现有工作有三个**潜在对比轴**：
1. **PE 利用率优化的位置**：是放在硬件（MAERI 的 ART 网络、Tangram 的 tiled inter-layer scheduling、Eyeriss-v2 的可重构 hierarchical mesh NoC）还是放在编译器（FLUX_CNN 的 Ky-fold + Space-to-Depth）。FLUX_CNN 走"硬件保持简洁、编译器侧填满 PE"路线。
2. **任意 H×W 输入的处理方式**：是按 tile/block 切分整图（绝大多数加速器）还是 row-streaming ring buffer（FLUX_CNN）。row-ring 反压配合"forward-pressure"语义在文献中**有近邻 prior art**：Alwani'16 Fused-layer (跨层 fusion 减少 off-chip 流量), Kang AoCStream'22 (stream-based line-buffer, line-buffer 大小线性于图宽), Liu Full-Stack'21 TNNLS (streaming + residual concatenative)。FLUX_CNN 的差异点需要在 Related Work 精细论证。
3. **去中心化握手流水 vs 中心 FSM**：FLUX_CNN 的 5 个核心模块各自维护 counter，靠 valid-ready 串联，无中心 controller。这与 Gemmini / NVDLA / VTA 显式的 controller-driven scheduler 形成对比。

> 目标会议 **[TBD]**。基于检索观察："FPGA + streaming + INT8 + RTL + compiler" 这种气质的工作集中在 **FPGA / FCCM / TCAD / TVLSI / TRETS**；"systolic array + dataflow + 体系" 的工作集中在 **ISCA / MICRO / HPCA / ASPLOS**。FLUX_CNN 的工程偏重（详细 RTL + 实测综合 + 编译器栈）+ Xilinx 7-series 平台导向，建议 **FPGA / FCCM / TCAD** 优先。

---

## A. 系统脉动阵列（ASIC，2D MAC array）

> 本类对比 FLUX_CNN 的 **16×16 阵列拓扑**和**列广播激活**数据流。

### Jouppi et al.@ISCA'17（TPU v1）

- **任务定位**：数据中心推理加速器，首次系统披露 256×256 INT8 systolic array 的规模化部署
- **核心方法**：Weight-stationary systolic array；超大 unified buffer（28 MiB）；CISC 指令集（matrix multiply / convolve / activate）
- **量化指标**：256×256 = 65,536 MAC（8-bit）；峰值 92 TeraOps/s（已通过 Elicit 确认）；28 MiB 软件管理片上内存
- **与 FLUX_CNN 的关系**：尺度对照（FLUX_CNN 256 MAC vs TPU 65k MAC，差 256×）；同样 INT8；都用 systolic 思路但 FLUX_CNN 是 OS 列广播（不是 WS 行流动）
- **可引用观点**：TPU v1 论文确立了 INT8 + systolic + huge unified buffer 的"数据中心规格"参考点，端侧加速器（如 FLUX_CNN）面对的是相反约束——SRAM 不可能容下 input/weight，必须 streaming
- **BibTeX key（暂定）**：`jouppi2017tpu`
- **DOI**：10.1145/3079856.3080246（ISCA'17，arXiv:1704.04760，Elicit 命中 ss-4202768）

### Chen et al.@ISCA'16（Eyeriss spatial architecture，第一篇）

- **任务定位**：能效优先的 CNN spatial 加速器；提出 row-stationary（RS）数据流以最小化片上 SRAM/RF 访问能量
- **核心方法**：RS 数据流——每个 PE 持有一行 weight，按行展开 ifmap/psum；spatial array 上做 PE 内 / PE 间 / 全局三级 reuse
- **量化指标**：RS 比已有 dataflow 节能 1.4×–2.5× (CONV layers)
- **与 FLUX_CNN 的关系**：阵列形态相似（spatial 2D），但 RS vs OS 的 trade-off 是 Related Work 必谈对比；RS 强调 weight reuse，FLUX_CNN 强调 ifmap 列广播 + Cout 列并行
- **可引用观点**：Eyeriss 把"data movement is dominant energy cost"作为核心论点；FLUX_CNN 在端侧 FPGA 场景里把这条转写为"DDR 流量主导，所以需要 row-ring streaming + 编译器侧重排（S2D）"
- **BibTeX key（暂定）**：`chen2016eyeriss_isca`
- **DOI**：10.1145/3007787.3001177（ISCA'16，作者 3 人：Chen / Emer / Sze；Elicit 命中 ss-3291270）

### Chen et al.@JSSC'17（Eyeriss chip）

- **任务定位**：Eyeriss 65 nm 流片，end-to-end 系统验证 RS 数据流
- **核心方法**：14×12 = 168 PE；可重构架构以支持不同 CNN 形状
- **量化指标**：65 nm CMOS；AlexNet **35 fps @ 278 mW** (batch=4)；VGG-16 0.7 fps @ 236 mW (batch=3)；DRAM access/MAC = 0.0029 (AlexNet)
- **与 FLUX_CNN 的关系**：JSSC 版本提供具体能效数字，是端侧 CNN 加速器的经典对照点
- **可引用观点**：Eyeriss 把端侧 CNN 加速器的 SOTA 数字钉死在了"百毫瓦级 + 几十 fps"这个量级；FLUX_CNN 在 FPGA 上的对标是 Angel-Eye / Snowflake，能效会差 1–2 个数量级（因 FPGA vs ASIC 工艺）
- **BibTeX key（暂定）**：`chen2017eyeriss_jssc`
- **DOI**：10.1109/JSSC.2016.2616357（JSSC'17，作者 4 人：Chen / Krishna / Emer / Sze；Elicit 命中 ss-207882941）

### Chen et al.@JETCAS'19（Eyeriss v2）

- **任务定位**：稀疏 + 紧凑模型（MobileNet 等）的端侧加速器
- **核心方法**：可重构 hierarchical mesh NoC（多播 / 单播 / 广播 mode 切换）；支持稀疏数据 on-the-fly 处理；RS+ 数据流（在 RS 基础上扩展空间 tiling 维度）
- **量化指标**：65 nm；sparse MobileNet 上 1470.6 inferences/s, 2560.3 inferences/J（对比 v1 跑 MobileNet 速度 12.6×, 能效 2.5×）
- **与 FLUX_CNN 的关系**：v2 的 reconfigurable NoC 是为了支持不同形状的层（Cin/Cout 失衡）；FLUX_CNN 的同类问题用编译器侧 Ky-fold/S2D 解决，硬件保持广播 mesh 不变
- **可引用观点**：Eyeriss v2 论证了"层形状失衡时硬件可重构 NoC 是一条路"；FLUX_CNN 提出"另一条路"——编译器侧重映射，硬件不动
- **BibTeX key（暂定）**：`chen2019eyerissv2`
- **DOI**：10.1109/JETCAS.2019.2910232（JETCAS'19，arXiv:1807.07928；Elicit 命中 ss-131771552）

### Genc et al.@DAC'21（Gemmini）

- **任务定位**：开源、参数化、生成式 DNN 加速器（基于 Chipyard）；用于 system-level evaluation；TSMC 16nm 和 Intel 22FFL 实际流片
- **核心方法**：Output-stationary 或 weight-stationary 可选；RoCC ISA 接 RISC-V；显式中心 controller / scheduler；可生成不同尺寸阵列（典型 8×8 / 16×16 / 32×32 / 64×64）
- **量化指标**：默认 16×16 INT8 systolic（Elicit 命中确认 16×16 是常用配置之一，arXiv 2019:1911.09925）；Gemmini 流片版本（与 BOOM/Hwacha 异构）报告 106.1 GOPS/W (22nm)
- **与 FLUX_CNN 的关系**：**最贴近的对比对象之一**——同样 16×16 默认配置、同样 INT8。差别：Gemmini 是中心 FSM/scheduler，FLUX_CNN 是去中心化 valid-ready 握手；Gemmini 跑在 SoC 集成场景，FLUX_CNN 是独立 AXI4 IP
- **可引用观点**：Gemmini 提供了一个开源 baseline，让端到端比较成为可能；FLUX_CNN 可在 Related Work 中说明"我们与 Gemmini 同尺度，但走 streaming row-ring 而不是 tiled scratchpad"
- **BibTeX key（暂定）**：`genc2021gemmini`
- **DOI**：arXiv:1911.09925（2019 arXiv 公开），DAC'21 流片论文最早完整版本 + ESSCIRC'21 (DOI 10.1109/ESSCIRC53450.2021.9567768) 测试 SoC（Elicit 命中 ss-208247987）

### NVDLA（NVIDIA, 2017–）

- **任务定位**：开源工业级 DLA IP；Jetson Xavier 等量产 SoC 中实际部署
- **核心方法**：可配置 MAC array（典型 64–2048 MAC，nv_small / nv_medium / nv_large 变体）；显式 SDP（Single Data Point processor）做 bias / BN / ReLU / 量化 fusion；分离 CDMA/CMAC/CACC/SDP 子模块
- **量化指标**：nv_small 在 50–100 MHz 下能效 0.6–1.5 TOPS/W（来自 Wolff 2019 stage report，Elicit 命中 ss-209959222）
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 SDP（bias + residual fusion + shift + ReLU + clip）**直接借用 NVDLA 的命名**；架构粒度也类似（CDMA→CMAC→CACC→SDP 对应 line_buffer→mac_array→parf_accum→sdp）。这是一个**重要的命名脉络要在文中点明**
- **可引用观点**：NVDLA 把"feature processing pipeline"模块化为 4 段，FLUX_CNN 沿用了这条思路并在每段上做了优化（去中心 FSM、per-col PARF、row-ring streaming）
- **BibTeX key（暂定）**：`nvdla2018`
- **DOI/参考**：**[CHECK: vendor doc 无标准学术 paper]** — 已 Elicit 检索 2 次，命中均为基于 NVDLA 的衍生研究 (Veronesi 2020 / Farshchi 2019 / Wolff 2019)，无 NVDLA 第一作者 paper。建议引用方式为 `nvdla.org` 官方 hardware spec + GitHub `nvdla/hw` 或衍生论文（如 Farshchi 2019 DOI 10.1109/EMC249363.2019.00012）

### Shao et al.@MICRO'19（Simba）

- **任务定位**：可扩展多 chiplet DNN 加速器，用 multi-chip-module (MCM) 包装 36 个 chiplet
- **核心方法**：每 chiplet 16 PE × 8 vector lanes = 128 MAC（4 TOPS peak）；ground-referenced signaling (GRS) NoP 互联；非均匀通信下的 tiling 优化（3 种）
- **量化指标**：单 chiplet 4 TOPS peak；36-chiplet MCM 128 TOPS / 6.1 TOPS/W；ResNet-50 batch=1 1988 fps / 0.50 ms latency；16nm
- **与 FLUX_CNN 的关系**：Simba 是"多 chiplet 多核"的代表，FLUX_CNN 多核 M1 阶段（`multicore_top`，N=2 已综合通过）也是同方向；可作为 system-level scaling 对照
- **可引用观点**：Simba 论证了 chiplet 级别的 NoP 通信是可解决的瓶颈；FLUX_CNN 多核版（XC7K325T 上 2 核）则是 BRAM-bound 而非通信 bound，对应不同 scaling 域
- **BibTeX key（暂定）**：`shao2019simba`
- **DOI**：10.1145/3352460.3358302（MICRO'19；Elicit 命中 ss-202547735）

---

## B. 数据流体系与 PE 利用率优化（编排）

> 本类围绕 FLUX_CNN 的"数据流取舍"和"硬件如何吸收层形状失衡"展开。

### Kwon et al.@ASPLOS'18（MAERI）

- **任务定位**：可重构 DNN 加速器；让单一硬件支持不同 DNN 形状下的高利用率
- **核心方法**：Augmented Reduction Tree（ART）+ Distribution Tree；PE 之间的连接拓扑可在运行时配置（多个虚拟数据流）
- **量化指标**：相比 rigid NoC baseline，提升 8–459% PE utilization（覆盖多种 dataflow 映射）
- **与 FLUX_CNN 的关系**：MAERI 用**硬件可重构**解层形状失衡问题，FLUX_CNN 用**编译器 Ky-fold + S2D**解同一问题。对比维度："硬件复杂度 vs 编译复杂度"
- **可引用观点**：MAERI 提出"任意层形状下高 PE 利用率"是可重构硬件的目标；FLUX_CNN 反向论证：在 Cin<16 的层形状下，可重构网络不是必需的——通过编译器侧把 Ky / spatial 维折到 Cin 上，固定 16×16 阵列就能填满
- **BibTeX key（暂定）**：`kwon2018maeri`
- **DOI**：10.1145/3173162.3173176（ASPLOS'18；Elicit 命中 ss-3990189）

### Du et al.@ISCA'15（ShiDianNao）

- **任务定位**：嵌入式视觉场景下的 CNN 加速器，与 image sensor 紧耦合，无 DRAM 访问
- **核心方法**：2D PE 阵列直接消费 sensor pixel stream；CNN 权重共享允许整网装进片上 SRAM；output-stationary 数据流降低能耗
- **量化指标**：65 nm；4.86 mm² die；320 mW；30× 高于高端 GPU（同期）；60× 高于已有 NN 加速器
- **与 FLUX_CNN 的关系**：与 FLUX_CNN 同样是 OS 数据流；ShiDianNao 假设全图能放进片上 SRAM（小图/sensor coupled），FLUX_CNN 假设大图（VGA 480×640 走 row-ring streaming），约束不同
- **可引用观点**：ShiDianNao 论证 OS 在 small-image sensor coupled 场景的高能效；FLUX_CNN 把 OS 推广到 large-image streaming 场景
- **BibTeX key（暂定）**：`du2015shidiannao`
- **DOI**：10.1145/2749469.2750389（ISCA'15；Elicit 命中 ss-11504619）

### Pellauer et al.@ASPLOS'19（Buffets）

- **任务定位**：通用片上 buffer 抽象，重新审视 reuse 模式
- **核心方法**：把 staging buffer 形式化为"buffet"（FIFO + counter + skip + decoupled fills/accesses），提供数据流通用化的语言；hierarchical composition + multi-cast
- **量化指标**：相比 8KB RAM 仅 2% 控制开销；vs DMA-managed double-buffered scratchpad / cache，能效 EDP 改善 1.53× / 5.39×
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 line_buffer / wgt_buffer / parf_col 都可以套 buffet 模式描述；Related Work 中借此引出"valid-ready 握手 + counter-driven 各模块"是 buffet 思想的一种 RTL 落地
- **可引用观点**：Buffets 提供了"buffer + counter + handshake"的形式化语言；FLUX_CNN 是该模式的去中心化实现示例
- **BibTeX key（暂定）**：`pellauer2019buffets`
- **DOI**：10.1145/3297858.3304025（ASPLOS'19，**注意**：之前文档错写为 10.1145/3307650.3322241/ISCA'19，已修正；Elicit 命中 ss-102346098）

### Parashar et al.@ISPASS'19（Timeloop）

- **任务定位**：系统化探索 DNN 数据流空间的建模工具
- **核心方法**：把 dataflow 形式化为 mapping over loop nests；对接 Accelergy 估能耗
- **量化指标**：N/A（建模工具）
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 6 层硬件循环嵌套（`yout / cs / tile / cins / ky / kx / iss_pos`，见 `docs/slicing/README.md`）天然能用 Timeloop 表示；做 PE 利用率分析、DSE 时可作为参考工具，**不一定需要在论文里跑 Timeloop，但应在 Related Work 提及**
- **可引用观点**：Timeloop 提供了 DNN 加速器 design space 的 systematic 描述；FLUX_CNN 给出一个具体点
- **BibTeX key（暂定）**：`parashar2019timeloop`
- **DOI**：10.1109/ISPASS.2019.00042（ISPASS'19；Elicit 命中 ss-113404930）

### Gao et al.@ASPLOS'19（Tangram）

- **任务定位**：多层 NN 在 tiled accelerator 上的层间流水（cross-layer pipelining）+ 层内 buffer-sharing 优化
- **核心方法**：Buffer sharing dataflow（intra-layer：分布式 buffer 表现为单一共享 buffer）+ Alternate layer loop ordering（inter-layer：fine-grained forward 中间数据，减小 buffer 需求）；支持复杂 DAG 结构
- **量化指标**：tiled NN accelerator 性能 +2×, 能耗 -45%
- **与 FLUX_CNN 的关系**：FLUX_CNN 单核当前是层串行（链式 CASES），但 streaming row-ring 已经为 row-level 跨层流水准备好（见 `docs/roadmap.md` 中"行流式跨层融合"项）。Tangram 的"row-level pipelining"思想是**将来工作的直接对照**
- **可引用观点**：Tangram 论证 cross-layer fine-grained forwarding 的能效收益；FLUX_CNN 的硬件 row-ring 反压机制为这条扩展路径提供了基础
- **BibTeX key（暂定）**：`gao2019tangram`
- **DOI**：10.1145/3297858.3304014（**ASPLOS'19**，**注意**：之前文档错写为 MICRO'19 / DOI 末尾 .3358301，已修正；Elicit 命中 ss-102347057）

---

## C. Streaming / line-buffer 风格 CNN 加速器（FPGA 端，与 FLUX_CNN 最贴近）

> 本类是 FLUX_CNN 系统级定位的**主要对照集**。第 4 次启动时通过 Phase 0 §批 8 检索发现 3 篇近邻 prior art（Alwani / Kang / Liu），新增进本节。

### Venieris & Bouganis@FCCM'16 / TNNLS'19（fpgaConvNet）

- **任务定位**：把 CNN 网络映射成 streaming hardware on FPGA；Synchronous Dataflow (SDF) 形式化 + 自动 design space exploration
- **核心方法**：每层一个 streaming module；HLS 生成；定义 SDF graph 上的 transformation 集探索 throughput / latency / multi-objective 优化
- **量化指标**：vs SOTA FPGA / DSP / 嵌入式 GPU，performance density 提升 6×, performance efficiency 提升 4.49×；vs SOTA FPGA-based ConvNet 架构，performance density +2.94×
- **与 FLUX_CNN 的关系**：fpgaConvNet 是"layer-pipelined streaming"路线代表；FLUX_CNN 走的是"single core + row-streaming layer-serial"，对比维度是 BRAM 占用 vs 灵活性
- **可引用观点**：fpgaConvNet 论证"per-layer streaming module"在 FPGA 上的可行性；FLUX_CNN 论证"统一核 + row-ring"在端到端 latency 上的优势
- **BibTeX key（暂定）**：`venieris2016fpgaconvnet` / `venieris2019fpgaconvnetjournal`
- **DOI**：FCCM'16 (10.1109/FCCM.2016.22；**注意是 FCCM 不是 FPGA**) + TNNLS'19 (**10.1109/TNNLS.2018.2844093**，**注意**：之前文档末尾 .2884591 错；Elicit 命中 ss-6758524 / ss-51614444)

### Sharma et al.@MICRO'16（DnnWeaver）

- **任务定位**：从高层 DNN 描述（Caffe）自动生成 FPGA RTL
- **核心方法**：Tiled MACC array + 模板化 RTL；macro dataflow ISA + 编译器 tile/schedule/batch；FSM-driven controller
- **量化指标**：覆盖 Xilinx Zynq / Altera Stratix V / Arria 10 三器件，与 ARM Cortex A15/Xeon E3 CPU 和 K1/650Ti/K40 GPU 对比性能/能效优势
- **与 FLUX_CNN 的关系**：DnnWeaver 是"模板生成"路线，FLUX_CNN 是手写优化 RTL；这反映了"灵活性 vs PPA"的取舍
- **可引用观点**：DnnWeaver 论证 RTL generation 的 productivity 收益；FLUX_CNN 论证 hand-tuned 16×16 + streaming 在固定阵列下能拿到更好 PE 利用率
- **BibTeX key（暂定）**：`sharma2016dnnweaver`
- **DOI**：10.1109/MICRO.2016.7783720（MICRO'16；Elicit 命中 ss-525898）

### Zhang et al.@FPGA'15（Roofline-based VGG accelerator）

- **任务定位**：用 Roofline 模型在 FPGA 上 DSE 出 VGG-style CNN 加速器
- **核心方法**：Loop tiling + on-chip buffer 配置 + Roofline 寻找最优 design point
- **量化指标**：**61.62 GFLOPS @ 100 MHz on VC707 (Virtex-7)** ✓
- **与 FLUX_CNN 的关系**：经典 FPGA-CNN 起点工作；FLUX_CNN 在 Kintex-7 同 100 MHz、INT8、256 MAC 下产生不同的算力点（256 × 0.99 × 2 = ~507 GOPS 峰值）
- **可引用观点**：Zhang FPGA'15 把 Roofline 引入 FPGA-CNN 设计；FLUX_CNN 没单独跑 Roofline 但 `model_analysis.md` 给了类似的 layer-by-layer compute / bandwidth 分析
- **BibTeX key（暂定）**：`zhang2015fpga`
- **DOI**：10.1145/2684746.2689060（FPGA'15；Elicit 命中 ss-207220904）

### Aydonat et al.@FPGA'17（Intel/Altera DLA）

- **任务定位**：Intel Arria 10 上的高性能 CNN 加速器
- **核心方法**：1D 数据流 + Winograd 优化；OpenCL HLS 实现
- **量化指标**：Arria 10 上 **AlexNet 1020 img/s / 23 img/s/W / 1382 GFLOPS**；vs SOTA FPGA 同期工作，10× faster, 8.4× more GFLOPS, 5.8× efficiency
- **与 FLUX_CNN 的关系**：商业 FPGA vendor 路线对照；FLUX_CNN 是 Xilinx 7-series 上的开源/自主实现
- **可引用观点**：Aydonat FPGA'17 展示了 vendor-specific HLS + Winograd 的高峰值；FLUX_CNN 不依赖 Winograd，靠直接卷积 + streaming
- **BibTeX key（暂定）**：`aydonat2017dla`
- **DOI**：**10.1145/3020078.3021738**（FPGA'17，**注意**：之前文档末四位 .3021727 错，已修正；Elicit 命中 ss-7272247）

### Gokhale et al.@ISCAS'17（Snowflake）

- **任务定位**：嵌入式 FPGA CNN 加速器，强调高 PE 利用率与跨网络通用性
- **核心方法**：vector engine + control pipeline + 自定义 ISA 编译；line buffer 持续滑动消费 ifmap
- **量化指标**：**Xilinx Zynq XC7Z045**，128 GOPS peak；AlexNet 100 fps / 120 GOPS / GoogLeNet 36 fps / ResNet-50 17 fps；91% 平均计算效率
- **与 FLUX_CNN 的关系**：line-buffer 模式的代表之一；FLUX_CNN 的 line_buffer 模块名沿用此传统，但加了 row-ring + forward-pressure（rows_available ≥ yout·stride + Ky）反压
- **可引用观点**：Snowflake 把 line-buffer 模式推广到端侧 CNN 多模型场景；FLUX_CNN 在此基础上加 row-level credit 反压让任意 H×W 单 start 跑完
- **BibTeX key（暂定）**：`gokhale2017snowflake`
- **DOI**：10.1109/ISCAS.2017.8050809（ISCAS'17；**注意**：之前文档错写为 CVPRW'14，实际是 2017 年 ISCAS + arXiv tech report；Elicit 命中 ss-20249920 / ss-8877990）

### Guo et al.@TCAD'18（Angel-Eye）

- **任务定位**：FPGA 上的 INT8 量化 CNN 加速器；Xilinx Zynq 平台；end-to-end 设计流（量化策略 + 编译工具）
- **核心方法**：固定 PE 阵列 + 数据 quantization-aware 流水；Caffe → 硬件 mapper；INT8 量化方案使精度损失 <1%
- **量化指标**：Zynq XC7Z045 上 vs 同器件其他 FPGA 实现，6× faster, 5× better power efficiency；Zynq XC7Z020 vs NVIDIA TK1/TX1，多 16× 能效
- **与 FLUX_CNN 的关系**：**最直接对照之一**——同 Xilinx 7-series、INT8、整张图 streaming。差异：Angel-Eye 是 controller-driven + 固定 tile，FLUX_CNN 是 row-ring + handshake-driven
- **可引用观点**：Angel-Eye 是 Zynq INT8 CNN 的代表；FLUX_CNN 的 PE 利用率和 streaming 抽象是 Related Work 必谈对照
- **BibTeX key（暂定）**：`guo2018angeleye`
- **DOI**：10.1109/TCAD.2017.2705069（TCAD'18；Elicit 命中 ss-206628725）

### Alwani et al.@MICRO'16（Fused-layer CNN accelerators）⭐ 第 4 次启动新增

- **任务定位**：跨层 fusion 减少 off-chip feature map 流量
- **核心方法**：通过修改 input data 引入芯片的顺序，让多层 conv 的中间数据在 on-chip 缓存而不写回 DRAM
- **量化指标**：VGGNet-E 前 5 conv 层，使用 362 KB on-chip 存储，把 off-chip feature map 流量从 77 MB 减到 3.6 MB / image (95%)
- **与 FLUX_CNN 的关系**：**直接近邻 prior art**！FLUX_CNN 的 row-ring streaming 也是用片上 buffer 缓 inter-layer 数据；差异在 FLUX_CNN 是单核 row-streaming layer-serial，Alwani 是 multi-layer 同时驻留计算
- **可引用观点**：Alwani 论证了 inter-layer feature map 缓存可以省掉 95% off-chip 访问；FLUX_CNN 把这个思路推到 row-level（缓存仅几行而不是几层 feature map），节省 BRAM
- **BibTeX key（暂定）**：`alwani2016fusedlayer`
- **DOI**：10.1109/MICRO.2016.7783725（MICRO'16；Elicit 命中 ss-5804465）

### Kang@arXiv'22 / Kang & Yang@Sensors'23（AoCStream）⭐ 第 4 次启动新增 / 第 5 次启动 venue 修正

- **任务定位**：所有特征图都驻留 on-chip 的 CNN 加速器，目标是 low-end FPGA 不需外存就能跑物体检测
- **核心方法**：stream-based line-buffer 架构（取代传统 frame-based）；line buffer 大小线性于图宽（而非平方于图大小）；每层 dedicated block，pipelined input/output streams；accelerator-aware pruning
- **量化指标**：在 low-end FPGA 上跑完整 object detection CNN，无需外存；2023 期刊扩展版在 MobileNetV1/V2 + SSDLite 变体上验证；相比同精度 FPGA 加速器，吞吐更高且 LUT/FF/DSP 更少
- **与 FLUX_CNN 的关系**：**最近邻 prior art**！同样的"stream + 仅几行 line buffer"思路。差异：(1) Kang 是 layer-pipelined 多 block 同时跑，FLUX_CNN 是 layer-serial 共用一组硬件；(2) Kang 强调"全片上"，FLUX_CNN 则承认外存必然存在并优化 DDR 流量
- **可引用观点**：Kang 论证 stream-based line-buffer 在 low-end FPGA 上节省 BRAM；FLUX_CNN 在更大模型 / DDR 受控场景下论证 row-ring 反压让 IDMA/计算/ODMA 三阶段并发
- **BibTeX key（暂定）**：`kang2022aocstream` (arXiv) / `kang2023aocstream_sensors` (期刊扩展)
- **DOI**：原版为 arXiv-only preprint **arXiv:2212.11438** (单作者 Hyeong-Ju Kang，2022年12月提交；DOI 10.48550/arXiv.2212.11438；Elicit 命中 ss-254974026，note: Semantic Scholar 把 venue 自动归类为 "Symposium on Field Programmable Gate Arrays" 系内容标签而非真实发表 venue)；2023 期刊扩展版 Kang & Yang "AoCStream: ... and Accelerator-Aware Pruning" 发表于 **MDPI Sensors**, vol. 23, no. 19, art. 8104, **DOI 10.3390/s23198104**（Elicit 命中 ss-263248239；Semantic Scholar 误标 venue 为 "Italian National Conference on Sensors"，实际是 MDPI Sensors 期刊）。引用建议：优先用 2023 期刊版（peer-reviewed），arXiv'22 作 preprint 备注

### Liu et al.@TNNLS'21（Full-Stack Streaming CNN Accelerator）⭐ 第 4 次启动新增 / 第 5 次启动作者格式修正

- **任务定位**：Intel Arria 10 上的 streaming hardware 架构 + full-stack 加速器
- **核心方法**：把 conv / deconv 层映射到统一模块；高效实现 residual / concatenative 连接（支持 ResNet / DenseNet 拓扑）；layer fusion + 多级 parallelism + 充分利用 DSP
- **量化指标**：Arria 10 GX1150 上 >1.3 TOP/s 吞吐, 97% MAC 计算效率
- **与 FLUX_CNN 的关系**：另一个 streaming + residual 同时支持的工作；FLUX_CNN 同样支持 residual fusion (R.2 SDP)，但放在 layer-serial 单核场景，无 layer fusion；可对照"layer fusion 的复杂度 vs FLUX_CNN 的简洁性"
- **可引用观点**：Liu 2021 表明 streaming + residual fusion 在大型 FPGA 上可达 97% MAC 效率；FLUX_CNN 在中型 7-series 上的 86.6% 整网 MAC% 是同思路在不同规模下的可比数字
- **BibTeX key（暂定）**：`liu2021fullstack`
- **DOI**：10.1109/TNNLS.2021.3055240（TNNLS'21；第一作者 **Shuanglong Liu**，合作者 Hongxiang Fan / Martin Ferianc / Xinyu Niu / Huifeng Shi / Wayne Luk；Elicit 命中 ss-231910267。前一版误把第二作者 "Fan" 拼到第一作者名下，本次修正为标准 first-author "Liu et al." 格式）

---

## D. PE 利用率优化技巧（kernel folding / Space-to-Depth / 通道复用）

> 本类专门支撑 FLUX_CNN 最独特的两个优化（Ky-fold + S2D）。

### Chetlur et al.@arXiv'14（cuDNN）

- **任务定位**：把卷积转成大矩阵乘以喂 systolic array（im2col + GEMM 路线，GPU 通用做法）
- **核心方法**：im2col 把 ifmap 平铺成大矩阵；K×K×Cin 折到一个维度；BLAS-style 接口
- **量化指标**：把 Caffe 跑 AlexNet 训练时间提速 36%，同时减小内存占用
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 Ky-fold 与 im2col 思想**同源但更轻量**——只把 Ky 维折到 Cin（不展开 Kx），保留 Cin 通道并行，避免 im2col 的全展开内存炸裂；S2D 进一步在 stride≥2 时**不复制数据**只重排
- **可引用观点**：im2col 是经典做法但内存膨胀严重；FLUX_CNN 的 Ky-fold + S2D 是"轻量化的 im2col 变体"，针对端侧 PE 行不足场景
- **BibTeX key（暂定）**：`chetlur2014cudnn`
- **DOI**：arXiv:1410.0759（无标准 DOI；Elicit 命中 ss-12330432）；**[CHECK: arXiv tech report 引用方式]**

### Shi et al.@CVPR'16（Sub-Pixel Convolutional Networks）

- **任务定位**：Real-time single image / video super-resolution；sub-pixel convolution layer（即 pixel shuffle / depth-to-space）作为高效上采样方法
- **核心方法**：在 LR 空间提取 feature，用 depth-to-space layer 重排到 HR；学习 upscaling filter 而不是手工 bicubic
- **量化指标**：图像 +0.15dB / 视频 +0.39dB；比已有 CNN-based 方法快一个数量级
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 S2D 是 **sub-pixel convolution 的逆操作（space-to-depth）**用于硬件 PE 利用率优化的**编译器侧自动重写**，不需要重训模型——这是与"训练时设计 sub-pixel layer"的根本区别
- **可引用观点**：Shi 2016 把 sub-pixel rearrangement 引入 CNN architecture；FLUX_CNN 把 inverse sub-pixel（space-to-depth）作为 compiler-pass 应用到任意 stride≥2 的预训练 conv，让 PE 利用率提升而不重训
- **BibTeX key（暂定）**：`shi2016subpixel`
- **DOI**：10.1109/CVPR.2016.207（CVPR'16，arXiv:1609.05158；Elicit 命中 ss-7037846）
- **[CHECK: 该 S2D 在加速器领域的引用谱系不清晰]** — Elicit 检索 1 次未命中"FPGA 上把 space-to-depth 作为 PE 利用率优化"的具体 prior art，FLUX_CNN 的"compiler-pass S2D"看起来是新点，但 reviewer 可能要求补查 ASPLOS / HPCA 中是否有 architecture-side S2D 的更早出处

### Lu et al.@FCCM'17（Winograd-based FPGA accelerator）

- **任务定位**：用 Winograd minimal filtering 减少 conv 乘法次数
- **核心方法**：Winograd algorithm + line buffer 结构（feature map data 在 tile 间复用）+ 流水化 PE engine + 多 PE 并行；analytical model 指导 design space exploration
- **量化指标**：Xilinx ZCU102 平台，AlexNet 854.6 GOP/s（整网）/ 1006.4 GOP/s（CONV layers）；VGG16 2940.7 GOP/s / 3044.7 GOP/s
- **与 FLUX_CNN 的关系**：另一种"减少有效计算量"的路线；FLUX_CNN 不用 Winograd（直接 conv，简洁），但**应在 Related Work 中说明为什么不选 Winograd**——因为在 Cin<16 这种 PE 行不足场景，问题不在乘法数而在 PE 利用率，Winograd 不解
- **可引用观点**：Winograd 减计算量但加 PE 行 / 列的复杂度，与 FLUX_CNN 解决的问题正交
- **BibTeX key（暂定）**：`lu2017winograd`
- **DOI**：10.1109/FCCM.2017.64（**确认 FCCM'17 单篇**，**注意**：之前文档 [CHECK FPGA 还是 FCCM] 已确认是 FCCM；Elicit 命中 ss-267806049）；TCAD'20 期刊扩展 DOI 10.1109/TCAD.2019.2897701

### Ma et al.@FPGA'17 (Loop optimization for CNN on FPGA)

- **任务定位**：系统化研究 CNN loop tiling / unrolling / interchange 对 FPGA PPA 的影响
- **核心方法**：对 4 层 conv 嵌套循环（N, M, R, C 的子集，加 K_y / K_x = 6 维）逐一分析展开方向；提出特定 dataflow 最大化 resource utilization
- **量化指标**：Arria 10 GX 1150 上 VGG-16 645.25 GOPS / 47.97 ms 延迟；vs SOTA FPGA-VGG 实现 >3.2× 提升；TVLSI'18 期刊扩展（DOI 10.1109/TVLSI.2018.2815603）覆盖到 NiN/VGG-16/ResNet-50/152，Stratix V 348 GOPS, Arria 10 715 GOPS
- **与 FLUX_CNN 的关系**：Ma FPGA'17 的 6 层循环模型与 FLUX_CNN `docs/slicing/README.md` 的 7 层循环嵌套（多了 cs / tile）几乎一一对应；可作为 loop-level dataflow 描述的统一语言
- **可引用观点**：Ma FPGA'17 给出了 FPGA-CNN loop unrolling 的 design space；FLUX_CNN 在此基础上明确了：Cin/Cout 维（M, N）展开到 PE 阵列，Ky/Kx 时序展开，并通过 Ky-fold 把 Ky 强制并行进 N 维
- **BibTeX key（暂定）**：`ma2017loopopt`
- **DOI**：10.1145/3020078.3021736（FPGA'17；Elicit 命中 ss-5262122）

---

## E. AXI / DMA / DataMover 在 FPGA CNN 加速器中的设计

> 本类对应 FLUX_CNN 的 IDMA / WDMA / ODMA + Xilinx axi_dm IP + axi_m_mux 子系统。

### Xilinx AXI DataMover (PG022)

- **任务定位**：Xilinx 官方提供的 high-throughput AXI4 MM ↔ AXI4-Stream 数据搬运 IP
- **核心方法**：Command / Status FIFO 接口；MM2S / S2MM 双通道；burst length / data width 可配
- **量化指标**：burst_size 可达 256；data width 可达 128 bit (FLUX_CNN 配置)
- **与 FLUX_CNN 的关系**：FLUX_CNN 直接用 axi_dm IP（替代之前手写 DMA），mm2s_arb 在 IDMA / WDMA 之间串行仲裁。这是 system-level 集成的关键决策——**比手写 DMA 节省 ~3000 行 RTL 且性能持平**
- **可引用观点**：vendor IP 在 FPGA-CNN 加速器集成中的作用经常被低估；FLUX_CNN 的 system view 显示，把 DMA 全部交给 vendor IP，自己只做轻量 controller（`*_ctrl`）就能拿到 vendor-grade 吞吐
- **BibTeX key（暂定）**：`xilinx_pg022`
- **参考**：**[CHECK: vendor doc 引用方式]** — Xilinx PG022 (AXI DataMover v5.1) 是 vendor product guide，无 paper 索引；引用方式为 `Xilinx Inc., "AXI DataMover v5.1 LogiCORE IP Product Guide (PG022)"`，需查最新版本号

### Xilinx Vitis AI / DPU (PG338)

- **任务定位**：Xilinx 官方 CNN 推理 IP（DPU = Deep-learning Processor Unit）
- **核心方法**：可配置 PE 阵列（B512 ~ B4096，名称中数字代表 ops/cycle）；INT8；集成 Vitis AI compiler
- **量化指标**：B4096 配置（Versal AI Engine 上 96-AIE-core 实现）32.76 TOPS peak；ResNet50 on VCK190 1653 fps（来自 Jia 2022 XVDPU paper）
- **与 FLUX_CNN 的关系**：商业对照——FLUX_CNN 是开源/自主实现，DPU 是闭源 vendor IP；都跑在 Zynq/Kintex 上的 INT8。**性能数字对比应非常谨慎**，因为 DPU 后端有 Vivado 深度优化
- **可引用观点**：DPU 代表 FPGA-CNN 加速器的工业最高水准；FLUX_CNN 在 system 简洁性（256 MAC、5 模块、~37k LUT）上具有对照价值，但峰值算力差一个数量级
- **BibTeX key（暂定）**：`xilinx_dpu_pg338`
- **参考**：**[CHECK: vendor doc 引用方式]** — Xilinx PG338，引用方式同上；可补引 Jia et al. FPL'22 "XVDPU" (DOI 10.1109/FPL57034.2022.00041) 作为可引用学术替代

### ARM Ethos-N78 / Ethos-U55（ARM 商业 NPU IP）

- **任务定位**：移动 / 嵌入式 SoC NPU，与 Cortex-A/M 集成
- **核心方法**：固定 PE 阵列；INT8 推理；Ethos-N 系列面向手机/汽车，Ethos-U 系列面向超低功耗 microcontroller
- **量化指标**：见下方"参考"行（vendor doc 引用方式合并标记）
- **与 FLUX_CNN 的关系**：商业 ASIC NPU 对照；同样面向端侧 INT8 推理
- **可引用观点**：Ethos 系列是 ASIC NPU 的工业基准，FLUX_CNN 在 FPGA 平台对应的"开源等价物"
- **BibTeX key（暂定）**：`arm_ethos_n78` / `arm_ethos_u55`
- **参考**：**[CHECK: vendor doc 引用方式]** — ARM Technical Reference Manual / ARM 白皮书。Elicit 检索 1 次未命中（vendor doc 无 paper 索引，本条目仅保留此一处 [CHECK]，量化指标行不重复标记）

---

## F. 编译器 / ISA-level 切片机制（multi-slice / tiling）

### Chen et al.@OSDI'18（TVM）

- **任务定位**：端到端 deep-learning compiler stack
- **核心方法**：Halide-style scheduling primitives + AutoTVM 自动调优；支持多硬件后端；graph-level + operator-level 优化（fusion / memory hiding）
- **量化指标**：与 SOTA hand-tuned libraries 性能持平 (low-power CPU / mobile GPU / server GPU)；可 target 新加速器（如 FPGA-based VTA）
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 `compile_layer.py` / `compile_model.py` / `_plan_ddr` 是 hand-rolled mini-compiler，scope 远窄于 TVM；但思想同源——把 PyTorch Conv2d 编译成硬件 cfg + 数据布局
- **可引用观点**：TVM 提供通用 DL compiler 框架；FLUX_CNN 提供一个固定后端的轻量编译器作为该谱系的具体落地
- **BibTeX key（暂定）**：`chen2018tvm`
- **DOI**：OSDI'18 conference proceedings + arXiv:1802.04799（Elicit 命中 ss-52939079）

### Yang et al.@ASPLOS'20（Interstellar）

- **任务定位**：让 dataflow accelerator 的 mapping 用 Halide scheduling 语言表达
- **核心方法**：把 dataflow 形式化为 7 nested loops over DNN 的 loop transformation；通过 Halide compiler 修改让其生成 hardware；提供创建 formal taxonomy of all existing dense DNN accelerators 的能力
- **量化指标**：固定 throughput 下，CNN 能效 +4.2×, LSTM/MLP +1.6×/+1.8×
- **与 FLUX_CNN 的关系**：FLUX_CNN 的 7 层硬件循环（yout / cs / tile / cins / ky / kx / iss_pos）天然能映射到 Interstellar 的 schedule 语言
- **可引用观点**：Interstellar 强调"compiler-hardware co-design"；FLUX_CNN 的 Ky-fold + S2D 是 compiler-hardware co-design 的具体例子
- **BibTeX key（暂定）**：`yang2020interstellar`
- **DOI**：**10.1145/3373376.3378514**（ASPLOS'20，**注意**：之前文档错写 .3378494，已修正；Elicit 命中 ss-212688928）

### Moreau et al.@IEEE Micro'19（VTA）

- **任务定位**：开源、参数化的 DNN 加速器 + 编译栈，用于 ResNet 等模型在 edge FPGA 上推理
- **核心方法**：4 段 pipeline（fetch / load / compute / store）；two-level ISA（task-ISA + microcode-ISA）；JIT compiler；对接 TVM
- **量化指标**：典型示范配置 **256-PE @ 100 MHz on Zynq XC7Z020 / XC7Z045**（Moreau 2018 ReQuEST@ASPLOS workshop, DOI 10.1145/3229762.3229766）；INT8；ResNet-18 推理
- **与 FLUX_CNN 的关系**：**与 FLUX_CNN 默认配置相近的开源对照**（VTA 256 PE = 16×16, 100 MHz, INT8）；差别在 compiler 栈的丰富程度和硬件灵活性。VTA 是 FLUX_CNN 最直接的开源对照
- **可引用观点**：VTA 论证 DNN accelerator + compiler 协同设计的价值；FLUX_CNN 是该思想的另一具体落地，专注 FPGA streaming 而非 tiled scratchpad
- **BibTeX key（暂定）**：`moreau2019vta`
- **DOI**：IEEE Micro'19 期刊版 **10.1109/MM.2019.2928962** "A Hardware–Software Blueprint for Flexible Deep Learning Specialization"（最常引用）+ arXiv:1807.04188；早期 ReQuEST@ASPLOS'18 workshop DOI 10.1145/3229762.3229766（Elicit 命中 ss-128358750 / ss-49672003）

---

## G. 量化 + Bias / Residual fusion (SDP-style 后处理)

> 对应 FLUX_CNN 在 R.1 / R.2 重构中把 bias / residual / shift / ReLU / clip 融合进 SDP。

### Jacob et al.@CVPR'18（Quantization-aware training）

- **任务定位**：INT8 量化训练框架，让 INT8 推理与 FP32 精度差<1%
- **核心方法**：Quantize-dequantize fake nodes during training；asymmetric per-channel quantization；integer-only inference (训练 + 推理协同设计)
- **量化指标**：MobileNets 等模型 8-bit + QAT 实现 ImageNet 分类 / COCO 检测 SOTA 精度
- **与 FLUX_CNN 的关系**：FLUX_CNN 假设输入模型已经过 INT8 quantization；Jacob 2018 是该假设的训练侧基础
- **可引用观点**：QAT 让 INT8 推理可行；FLUX_CNN 在硬件侧落地（INT8 MAC + SDP fixed-point shift + ReLU + clip）
- **BibTeX key（暂定）**：`jacob2018quantization`
- **DOI**：10.1109/CVPR.2018.00286（CVPR'18，arXiv:1712.05877；Elicit 命中 ss-39867659）

### He et al.@CVPR'16（ResNet）

- **任务定位**：残差连接首次系统化引入 deep CNN
- **核心方法**：identity shortcut + element-wise add；让训练 152+ 层成为可能；学习 residual functions wrt layer inputs
- **量化指标**：**ImageNet ensemble test set error 3.57%**（**注意**：之前文档错写为"top-5 3.57%"，论文摘要是 plain "error"，已修正）；100/1000 层 CIFAR-10 分析；COCO object detection +28% 相对提升
- **与 FLUX_CNN 的关系**：FLUX_CNN R.2 阶段加的 SDP residual fusion（`shortcut_mult / shortcut_shift`）正是为了硬件原生支持 ResNet 的 skip connection
- **可引用观点**：ResNet 让 residual 成为现代 CNN 标配；硬件加速器（FLUX_CNN）必须原生支持 residual fusion 才能避免 host CPU 介入
- **BibTeX key（暂定）**：`he2016resnet`
- **DOI**：10.1109/CVPR.2016.90（CVPR'16，arXiv:1512.03385；Elicit 命中 ss-206594692）

### Han et al.@ISCA'16（EIE）

- **任务定位**：稀疏 + 量化 DNN 加速器，针对压缩后的 FC/CNN
- **核心方法**：weight pruning + sparse encoding + run-length skip-zero + 4-bit weight sharing；直接在 compressed network 上跑 inference
- **量化指标**：102 GOPS sparse / 3 TOPS dense 等价；AlexNet FC layer 1.88×10^4 fps @ 600 mW；vs DaDianNao 2.9× throughput / 19× energy / 3× area efficiency
- **与 FLUX_CNN 的关系**：稀疏路线代表；FLUX_CNN 暂未做稀疏（dense INT8），可作为正交方向 Related Work
- **可引用观点**：EIE 论证稀疏 + 量化协同的高能效；FLUX_CNN 选择 dense INT8 + streaming，避免稀疏带来的 indexing 复杂度
- **BibTeX key（暂定）**：`han2016eie`
- **DOI**：10.1145/3007787.3001163（ISCA'16，arXiv:1602.01528；Elicit 命中 ss-1663491）

### Han et al.@FPGA'17（ESE）

- **任务定位**：稀疏 LSTM 在 FPGA 上的语音识别加速
- **核心方法**：load-balance-aware pruning（20× 压缩，10× 来自 pruning + 2× 来自 quantization）+ scheduler + 直接在 sparse 模型上的硬件
- **量化指标**：Xilinx XCKU060 @ 200 MHz，sparse LSTM 282 GOPS / 等价 dense 2.52 TOPS；41W 功耗；vs Core i7 / Pascal Titan X，43× / 3× faster, 40× / 11.5× more energy efficient
- **与 FLUX_CNN 的关系**：与 EIE 同思路在 FPGA 上的延伸；FLUX_CNN 选择 dense INT8 + streaming
- **可引用观点**：ESE 把 EIE 的稀疏思路推到 FPGA + LSTM；FLUX_CNN 处于"dense + streaming + CNN"的不同 trade-off 点
- **BibTeX key（暂定）**：`han2017ese`
- **DOI**：10.1145/3020078.3021745（FPGA'17；Elicit 命中 ss-3351553）

---

## 对比维度建议

> FLUX_CNN 侧拿不到的数据用 `[CHECK: 等实测]` 标注。基础数字来自 README / STATUS / model_analysis.md。

### 性能维度

| 维度 | FLUX_CNN（当前可获取） | 对照工作 | 备注 |
|------|------------------------|----------|------|
| 峰值 INT8 算力 (GOPS) | 256 MAC × 100 MHz × 2 = **51.2 GOPS** [CHECK: 单核 100 MHz target，实测 Fmax 68 MHz → 34.8 GOPS 实际可达] | TPU v1: 92 TOPS（数据中心，差 1800×）/ Zhang FPGA'15: 61.62 GFLOPS / Snowflake Zynq XC7Z045: 128 GOPS / Angel-Eye Zynq: 6× 同期 / Aydonat Arria 10: 1382 GFLOPS / Lu Winograd ZCU102: 854.6 GOPS / Liu TNNLS'21 Arria 10: >1.3 TOPS | 量级差异显著，不应直接绝对比较，应按 "GOPS / DSP" 或 "GOPS / W" 归一化 |
| PE 利用率 (浅层 Cin<16) | 无 fold: Layer1 12.5%, Layer3/4 25% / 启用 Ky-fold: ~99% [CHECK: 实测百分比待 project-analyst 测] | MAERI ART: 提升 8–459% PE utilization (vs rigid baseline) / Eyeriss-v2 NoC: 12.6× MobileNet 速度 / Liu TNNLS'21: 97% MAC efficiency | 这是 FLUX_CNN 的**核心宣称点**，需精确测量 |
| PE 利用率 (Cin≥16 标准层) | 22-case 链式回归 86.6% MAC% (整网，含 idle) | [CHECK: 各 baseline 整网 MAC%] | "整网 MAC%"包含 IDMA/ODMA stall，比"单层 PE 利用率"更严格 |
| 端到端 latency | ResNet-18 风格 11 层链 5.95 ms @ 100 MHz target → 168 fps 上限 [CHECK: 是 target 还是 Fmax] | Simba ResNet-50 batch=1: 0.50 ms (但 36 chiplet 16nm); Snowflake AlexNet: 100 fps / ResNet-50: 17 fps; Aydonat AlexNet: 1020 img/s | 单图 4.9 MB → 一次 start 跑完是关键差异 |
| Fmax (实测综合) | XC7K325T-2 OOC: **68.4 MHz** (WNS=-4.618 ns) | [CHECK: 同器件其他工作 Fmax] | 关键路径在 SDP 量化组合链，已知优化路径见 STATUS §4 |

### 功能维度

| 维度 | FLUX_CNN | 对照工作 |
|------|----------|----------|
| 任意 H × W 输入 | ✅ row-ring streaming（VGA 480×640 单 start）| TPU/Eyeriss/Gemmini: 通常 tile-based，需多次 invocation；**Alwani Fused-layer / Kang AoCStream / Liu Full-Stack: 也是 streaming/fused，row-level granularity 不同** |
| 任意 Cin / Cout | ✅ multi-slice 切片（cin_slices × cout_slices）| 大多数加速器支持，差别在编译器自动化程度 |
| Cin < 16 PE 行不足 | ✅ Ky-fold（编译器侧）/ S2D（编译器侧）| MAERI: 硬件 ART / Eyeriss-v2: NoC reconfiguration / FLUX_CNN: compiler-only |
| Cout < 16 | ❌ 不复用，PE 列空转 util=Cout/16 | MAERI: 列也可重映射 / FLUX_CNN 选择保持简洁 |
| Residual / Bias / ReLU fusion | ✅ SDP 内 fixed-point shift+add+ReLU+clip | NVDLA: 同样 SDP / Liu TNNLS'21: 也支持 / Angel-Eye: [CHECK] |
| Pooling | ❌ 暂无硬件，软件做 | Most accelerators: 硬件 pool unit |
| Depthwise Conv | ❌ 16×16 broadcast 不直接支持 | MobileNet 加速器（Eyeriss-v2 等）: 有 dedicated DW 路径 |
| 稀疏 / 剪枝 | ❌ 不支持 | EIE/Eyeriss-v2/ESE: 原生支持 |

### 工程维度

| 维度 | FLUX_CNN | 对照工作 |
|------|----------|----------|
| 资源占用 (XC7K325T 单核) | LUT 36.9k (18.1%), FF 13.2k (3.2%), BRAM 128 (28.8%), DSP 82 (9.8%) | [CHECK: 同器件 Angel-Eye / DPU / VTA] |
| 多核 scaling | M1.5 N=2 综合 + 仿真通过；3 核是 BRAM 上限 (86%) | Simba: chiplet × 36 / NVDLA: 多 instance 嵌入 SoC |
| 编译器栈 | PyTorch nn.Sequential → bit-exact，hand-rolled，~5 文件 | TVM/VTA: 通用栈 / fpgaConvNet: HLS 模板生成 / Vitis AI: vendor 闭源 |
| AXI / DMA 集成 | 1 AXI4 M (128b) + 1 AXI-Lite S (32b)，descriptor-driven | DPU: 类似 / VTA: AXI Stream / Gemmini: RoCC |
| 中心 controller vs 去中心 | **去中心化 valid-ready 握手，无中心 FSM** | NVDLA/Gemmini/VTA: 显式中心 controller |
| 是否开源 | [TBD: 取决于发表策略] | NVDLA/Gemmini/VTA/Eyeriss(部分)/AoCStream/Snowflake: 开源 / Angel-Eye/DPU/Ethos: 闭源 |
| 仿真 / 验证 | ModelSim 22-case 链式回归 + 多核 smoke TB | [CHECK: 各工作 verification 公开度] |

### 可量化优势 candidate（用于 abstract / contribution claim）

下列 claim 强弱由强到弱排列，**每条都需要在 Phase 1（project-analyst）阶段配实测数据**：

1. **流式任意 H×W**：单次 start 处理 VGA 480×640（4.9 MB）只用 10 KB ring buffer (`strip_rows=8 × W=640`)。**审稿人威胁**：Alwani'16 Fused-layer / Kang'22 AoCStream 已有近邻 streaming 思想；FLUX_CNN 的真正差异点是 (a) row-level granularity（不是 layer-fused 也不是 fully-on-chip）+ (b) layer-serial 单核共用硬件（不是 layer-pipelined 多 block）。需要在 Related Work 精细论证，不要把它当独创点
2. **编译器侧 PE 利用率优化**：Ky-fold + S2D 把浅层 PE util 从 12.5%（Cin=4, Cout=8）提到接近 100%，**零 RTL 改动**。**审稿人威胁较弱**：MAERI 是硬件路线，Eyeriss-v2 是 NoC 路线；Ky-fold + S2D 作为"compiler-only"路线在已检索文献中**无明显近邻**——这是 FLUX_CNN 较强 claim 候选
3. **去中心化握手流水**：5 模块 + valid-ready，无中心 FSM。属于"工程美学"claim，需评估是否被审稿人认可为贡献。Buffets (ASPLOS'19) 提供了相关的形式化抽象，可以作为 framing 工具
4. **SDP residual fusion**：NVDLA-inspired，但加上了 `shortcut_mult / shortcut_shift` 的可编程量化因子。需要审视是否足够新颖（Liu TNNLS'21 也是 streaming + residual fusion）

---

## 待补清单

- **[TBD]** 目标会议 / 期刊未定。基于检索观察建议 **FPGA / FCCM / TCAD / TVLSI / TRETS** 优先（FLUX_CNN 的工程偏重 + Xilinx 7-series 平台导向）；如果着重 systolic / dataflow taxonomy 一面，可投 ISCA/MICRO/HPCA/ASPLOS——但需配合更系统的对比实验
- **[CHECK]** vendor doc 引用方式（5 处）：NVDLA、Xilinx PG022、Xilinx PG338、ARM Ethos-N78/U55、cuDNN tech report — 这些都是 vendor doc / arXiv tech report，无标准学术 paper 索引。建议在 Phase 1 决定：(a) 直接引 vendor doc（如 `Xilinx Inc. PG022, "AXI DataMover v5.1 LogiCORE IP Product Guide"`），或 (b) 引衍生 academic paper（如 NVDLA → Farshchi 2019, DPU → Jia 2022 XVDPU）
- **[CHECK]** S2D（space-to-depth）在硬件加速器领域的引用谱系不清晰 — Phase 0 §批 4 检索 1 次未命中"FPGA 上把 S2D 作为 PE 利用率优化"的具体 prior art；Sub-pixel CVPR'16 是最近邻但属于 ML 算法侧。FLUX_CNN 的"compiler-pass S2D"看起来是新点，建议 Phase 1+ 在 reviewer 阶段补查 ASPLOS/HPCA 中是否有 architecture-side S2D 的更早出处
- **[TBD]** FLUX_CNN 是否开源 — 取决于发表策略
- **[CHECK]** FLUX_CNN 实测 PE 利用率百分比 — 等 project-analyst 实测填充（**第 5 次启动按 reviewer F3 改归 [CHECK]：属"测一下就能确定"的数据项，非决策项**）
- **[CHECK]** FLUX_CNN 侧其他实测数据待补（**第 5 次启动按 reviewer F2 增补**）：(a) 实测 Fmax 68.4 MHz 下 GOPS 真实值 / (b) "100 MHz target vs 68.4 MHz Fmax" 两个口径在文中应统一标注 / (c) 单层 PE 利用率 vs 整网 MAC% 两数据需口径区分。**对位行**：line 481 / 482 / 484
- **[CHECK]** 文献对照数据待联网核（**第 5 次启动按 reviewer F2 增补**，需 Phase 1+ 主 Agent 跑 Elicit 或查原文补齐）：(a) 各 baseline 整网 MAC% / (b) 同器件 (XC7K325T 量级) 其他工作 Fmax / (c) Angel-Eye 是否支持 SDP residual fusion / (d) 同器件 Angel-Eye / DPU / VTA 资源占用 / (e) 各工作 verification (TB / 回归) 公开度。**对位行**：line 481 / 485 / 495 / 504 / 510

---

## 已完成核验（保留作记录）

> 第 4 次启动按 8 批跑完 Elicit 检索，共 18 次 `mcp__elicit__search_papers` 调用：
>
> - 批 1（系统脉动阵列：TPU/Eyeriss/Gemmini/NVDLA/Simba）：3 次
> - 批 2（可重构数据流/Buffets/Timeloop/Tangram）：1 次
> - 批 3（FPGA streaming：fpgaConvNet/DnnWeaver/Angel-Eye/Snowflake/Aydonat）：4 次
> - 批 4（Im2col/Winograd/Loop/S2D）：4 次
> - 批 5（vendor IP：Xilinx/ARM）：1 次（vendor doc 命中少符合预期）
> - 批 6（编译器：TVM/Interstellar/VTA）：3 次
> - 批 7（量化/残差/稀疏）：4 次（Jacob/ResNet/EIE/ESE 各 1 次）
> - 批 8（streaming row-ring 强化）：1 次（**命中 3 个近邻 prior art**：Alwani Fused-layer, Kang AoCStream, Liu Full-Stack；已加入 §C）
>
> 关键修订（详见各条目）：
> - **DOI / 会议归属错误修正 6 处**：Eyeriss ISCA'16 (DOI) / Pellauer Buffets (DOI) / Tangram (会议 ASPLOS'19 + DOI) / fpgaConvNet TNNLS (DOI 末四位) / Aydonat DLA (DOI 末四位) / Interstellar (DOI 末三位)
> - **额外会议归属修正**：Snowflake (CVPRW'14 → ISCAS'17)（注：Tangram 的会议归属 MICRO'19 → ASPLOS'19 已包含在上一行的 6 处中，本行不再重复计入）
> - **数据/事实错误修正 1 处**：He ResNet "top-5 3.57%" → "ensemble error 3.57%"（论文摘要原文）
> - **新增 prior art 3 条**：§C 新增 Alwani Fused-layer / Kang AoCStream / Liu Full-Stack，因为它们对 FLUX_CNN claim 1 ("流式任意 H×W") 构成 prior art 威胁，必须在 Related Work 论证差异点
> - **Eyeriss ISCA'16 vs JSSC'17 拆为两条**：之前合并为一条不符合学术习惯（两者作者列表/数据不同）
> - **Lu Winograd 是 FCCM'17 单篇** (之前 [CHECK FPGA 还是 FCCM] 已确认)
> - **VTA 主要引用版本**：Moreau IEEE Micro 2019 (DOI 10.1109/MM.2019.2928962) 而不是只 arXiv tech report
