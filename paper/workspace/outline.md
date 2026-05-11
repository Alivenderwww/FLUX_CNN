# 论文大纲（Phase 2 v2 — 中文本科毕业论文范式）

## 元信息

- **目标**：中国大学集成电路设计与集成系统专业本科毕业论文
- **范式来源**：`paper/workspace/chinese-thesis-spec.md` §11.1 范文范式
- **数据快照对应 commit**：`5fe16b2`（与 contributions.md v2 对齐）
- **完成时间**：2026-05-06
- **目标会议/期刊**：N/A（本科毕业论文，非投稿）
- **总章节数**：6 个正文章 + 摘要/Abstract/主要符号表/参考文献等附属节
- **预估总字数**：4-6 万字（不含参考文献）
- **narrative 主轴选择**：**narrative A（编译器侧 PE 利用率）为主轴 + narrative B（系统集成）为支撑**
  - 理由：narrative A（Ky 折叠 + S2D + 联合触发）是 contributions.md 中 novelty 强度最高的贡献组（C2.1/C2.2/C2.3，含 Patch 层 5.05× 实测数据）；narrative B（去中心化 5 模块流水 + 行环 + 多核 W 切片）是工程闭环支撑，单独看 novelty 弱-中，但与 narrative A 配合形成完整方案。

## 论文标题（草拟）

- **候选 1**（强调编译器侧优化）：
  - 中：基于 FPGA 的固定阵列 CNN 加速器及其编译器侧 PE 利用率优化
  - 英：FPGA-based CNN Accelerator with Fixed Spatial Array and Compiler-side PE Utilization Optimization
- **候选 2**（强调系统集成）：
  - 中：基于流式行环数据通路的多核 CNN 加速器设计与实现
  - 英：Design and Implementation of a Multi-core CNN Accelerator with Streaming Row-ring Datapath
- [TBD: 标题最终待用户确定，倾向候选 1 — 与 narrative A 主轴一致]

## 一句话摘要

针对端侧 CNN 加速器在层形状失衡（*C*ᵢₙ<16）与大步幅卷积（stride≥2）下处理单元（PE）利用率低的问题，本论文提出一种基于 FPGA 的固定 16×16 INT8 阵列加速器，其硬件层采用去中心化 valid-ready 流水与流式行环数据通路、编译器层采用 Ky 折叠（Ky-fold）与空间到深度（S2D）变换将 PE 利用率优化由硬件侧迁移至编译器侧，在 XC7K325T 上单核综合通过、ResNet-11 整网在 4 核多核架构上实现 2.51× 加速。

---

## 正文章节大纲（按本科论文范式 1-6 章）

> 一级章号用阿拉伯数字（无"第 N 章"前缀），子节用 1.1 / 1.2.1 等。
> 末章拆分为 6.1 结论 / 6.2 创新点 / 6.3 展望（强制）。

### 1 绪论

- **章节定位**：全文起点。承载选题缘由 + 文献综述（≈8000 字，按 chinese-thesis-spec §4 强制要求）+ 研究目标 + 论文组织。是评审第一眼判断"工作是否值得读"的章节。
- **目标**：让读者在不读后续章节的前提下理解 (1) 端侧 CNN 加速器为何重要、(2) 目前已有工作的两条主线（ASIC 系统脉动 + FPGA streaming）、(3) 本工作切入点（PE 利用率优化迁移到编译器侧）以及与 prior art 的差异化。
- **包含小节**：
  - **1.1 研究背景与意义**（≈1500 字）
    - 1.1.1 卷积神经网络与端侧推理需求
    - 1.1.2 端侧 FPGA 加速器面临的核心矛盾（片上 SRAM 容量 vs 任意尺寸输入；固定 PE 阵列 vs 层形状失衡）
    - 1.1.3 本论文研究意义
  - **1.2 国内外相关研究现状**（≈4500 字 — 文献综述主体）
    - 1.2.1 ASIC 系统脉动与可重构阵列加速器（TPU / Eyeriss / Eyeriss-v2 / Gemmini / NVDLA / MAERI / Tangram — 对照 literature.md §A/§B）
    - 1.2.2 FPGA 流式 CNN 加速器（fpgaConvNet / Snowflake / Angel-Eye / Caffeine / Aydonat DLA — 对照 literature.md §C 主体）
    - 1.2.3 跨层与流式行缓冲技术（Alwani 跨层融合 / Kang AoCStream / Liu Full-Stack — **对照 literature.md §C 三处威胁项**，必须显式应对）
    - 1.2.4 PE 利用率优化与卷积变换技术（im2col / Winograd / Direct Convolution / Pixel-Shuffle / S2D 在加速器领域的引用谱系 — 对照 literature.md §D/§E/§F）
    - 1.2.5 编译器与工具链（fpgaConvNet 工具链 / VTA / NVDLA 编译器 — 对照 literature.md §G）
  - **1.3 研究目标与研究内容**（≈1200 字）
    - 1.3.1 研究目标（针对 *C*ᵢₙ<16 与 stride≥2 下 PE 利用率塌陷问题，提出"硬件最简 + 编译器侧重映射"方案）
    - 1.3.2 研究内容（4 大块对应 contributions.md 4 大类）
    - 1.3.3 主要贡献（**5 条对应主要贡献，挑选 contributions.md 23 条中 novelty 最强的核心几条整合**）：
      - **贡献 1**：提出 Ky 折叠（Ky-fold）编译器侧 PE 填充方法（C2.1）
      - **贡献 2**：提出空间到深度（S2D）等价变换（C2.2 + C2.3 联合触发）
      - **贡献 3**：去中心化 valid-ready 5 模块流水 + 行级流式行环数据通路（C1.2 + C1.3 整合）
      - **贡献 4**：单层内 W 切片多核扩展架构（C3.4）
      - **贡献 5**：PyTorch → 硬件 cfg 端到端编译流（C2.4 + C3.2 整合）
  - **1.4 论文组织结构**（≈300 字 — 各章一句话预告）
- **依赖项**：
  - contributions.md：1.3.3 主要贡献条目对应 C1.2 / C1.3 / C2.1 / C2.2 / C2.3 / C2.4 / C3.2 / C3.4
  - literature.md：1.2 各小节穷举 §A-§G 全部 35 条文献；**1.2.3 必须显式应对 Alwani / Kang / Liu 三处威胁**
- **未决问题**：
  - [TBD: 1.3.3 主要贡献是否压缩到 4 条 — 若篇幅紧张可合并贡献 4+5 为"系统集成与编译流"]
  - [CHECK: 1.2.4 S2D 在加速器领域的引用谱系 — 由 paper-literature-scout 在主线程已补查 Pixel-Shuffle/Sub-pixel 与本工作 S2D 的关系，需在写作时落到一段 200-300 字差异化论证]
- **预估篇幅**：8000 中文字（chinese-thesis-spec §4 强制 ≈8000 字）

### 2 卷积神经网络与FPGA硬件加速技术原理

- **章节定位**：理论基础章。chinese-thesis-spec §11.1 明确要求"应有独立的理论基础章"。承担两个使命：(1) 给后续章节铺垫概念（卷积公式 / 数据流分类 / valid-ready 握手 / AXI 协议），(2) 让评审相信作者具备本科应有的理论功底。**不放本工作的设计**——只放公认理论。
- **目标**：让读者在读完本章后能用统一术语理解后续设计章节，无需在第 3-5 章再回头解释 *C*ᵢₙ / *C*ₒᵤₜ / WS/OS/RS 等基础概念。
- **包含小节**：
  - **2.1 引言**（章首段范式 ≈200 字 — 承上启下）
  - **2.2 卷积神经网络基础**（≈1500 字）
    - 2.2.1 卷积、池化、全连接基本算子与7层循环嵌套
    - 2.2.2 INT8 量化推理与算子融合（bias / shift / clip / 残差融合）原理
    - 2.2.3 ResNet 残差结构与跨层数据流约束
  - **2.3 CNN 硬件加速器数据流分类**（≈1200 字）
    - 2.3.1 权重驻留（Weight-Stationary，WS）
    - 2.3.2 输出驻留（Output-Stationary，OS）
    - 2.3.3 行驻留（Row-Stationary，RS）
    - 2.3.4 各数据流的复用粒度与片上存储需求对比
  - **2.4 FPGA 硬件加速基础**（≈1500 字）
    - 2.4.1 FPGA 基本组成（LUT / FF / BRAM / DSP）与 Xilinx 7 系列特点
    - 2.4.2 流水线设计与 valid-ready 握手协议
    - 2.4.3 行缓存（line buffer）与片上 SRAM 行环结构
    - 2.4.4 AXI4 / AXI-Lite 总线协议与 DMA 数据搬运
    - 2.4.5 Vivado 综合工具链与 ModelSim 仿真流程
  - **2.5 PE 利用率优化基础变换**（≈800 字 — 引出第 4 章 narrative A 的数学基础）
    - 2.5.1 卷积通用变换（im2col / Winograd / 直接卷积）
    - 2.5.2 步幅卷积与子像素重排（Pixel-Shuffle / Sub-pixel）的等价关系
  - **2.6 本章小结**（≈300 字 — 不引入新数据，铺垫第 3 章）
- **依赖项**：
  - contributions.md：无直接对应（本章不放本工作设计）
  - literature.md：§A 数据流分类引 Eyeriss / TPU；§D 卷积变换引 im2col / Winograd / Direct Convolution；§E 量化引相关工作
- **未决问题**：
  - [TBD: 2.5.2 是否包含 S2D 数学推导 — 倾向只在第 4 章 §4.3 详写，本章只点出"步幅卷积可通过子像素重排转换为 stride=1"这一公认事实]
- **预估篇幅**：5500 中文字

### 3 固定阵列CNN加速器整体方案

- **章节定位**：设计章。承上启下——把第 2 章理论与第 4 章实现串起来。本章只讲"做了什么选择以及为什么"，不展开 RTL 细节。这是 chinese-thesis-spec §5 强调的"本研究内容的总体方案设计与选择论证"章。
- **目标**：让读者在读本章后理解 (1) 设计的硬件架构形态（16×16 + OS+列广播）、(2) 关键取舍依据（为什么不选 RS / 为什么不选可重构 NoC / 为什么选行环而非整图缓冲），并能指向第 4 章对应小节深入。
- **包含小节**：
  - **3.1 引言**（章首段范式）
  - **3.2 设计目标与约束**（≈1000 字）
    - 3.2.1 应用场景假设（端侧 / 实时 / 任意 *H*×*W* 输入）
    - 3.2.2 平台约束（XC7K325T 的 BRAM/DSP/LUT 容量、单 DDR 通道带宽）
    - 3.2.3 算法约束（INT8 量化 / ResNet 风格残差 / *C*ᵢₙ 与 *C*ₒᵤₜ 任意）
  - **3.3 总体架构方案**（≈1500 字 — 配顶层框图）
    - 3.3.1 两层结构：核流水（5 模块 + cfg_regs）+ DMA 子系统（idma_ctrl / wdma_ctrl / odma_ctrl + axi_dm IP + mm2s_arb + axi_m_mux + axi_lite_csr）
    - 3.3.2 外部接口：1 主 1 从（AXI4 Master + AXI-Lite Slave）
    - 3.3.3 多核拓扑（multicore_top + axi_2to3 / axi_4to5 + axi_lite_1to4 fanout）
  - **3.4 数据流选择论证**（≈1200 字 — 关键论证章）
    - 3.4.1 为什么选 OS+列广播（与 RS / WS 对比）
    - 3.4.2 为什么固定 16×16 不可重构（与 MAERI / Eyeriss-v2 路线对比 — 论证"硬件最简 + 编译器侧填满"路线的取舍）
    - 3.4.3 为什么选行级流式行环（与 Alwani 跨层融合 / Kang AoCStream / Liu Full-Stack 三处近邻 prior art 对比 — **必须显式差异化**）
  - **3.5 编译器与硬件协同设计原则**（≈800 字）
    - 3.5.1 单源参数 params.py（RTL/Python 双向消费 — 对应 C3.5）
    - 3.5.2 cfg 派生 single source（hw_files.derive_layer_cfg）
    - 3.5.3 编译器/硬件视角对齐（s2d_eff() 与 build_step_cfg_dict — 对应 C2.3）
  - **3.6 本章小结**（≈300 字）
- **依赖项**：
  - contributions.md：C1.1（16×16 OC-broadcast）/ C1.2 概览 / C1.3 概览 / C3.1 DMA 子系统 / C3.4 多核拓扑 / C3.5 单源参数 / C2.3 联合触发 — 本章只讲架构选择，**不展开实现细节**（实现在第 4 章）
  - literature.md：§A（TPU/Eyeriss 对比）/ §B（MAERI/Eyeriss-v2 对比）/ §C（Alwani/Kang/Liu 对比 — **三处威胁项主战场**）
- **未决问题**：
  - [TBD: 3.5 编译器与硬件协同设计原则是否单独成节 — 若篇幅紧张可合并到 3.4]
  - [CHECK: 3.4.3 与 Alwani/Kang/Liu 三处差异化论证的措辞 — 由 paper-literature-scout 已论证（行级 vs 跨层 fused / 行级 vs 整图 / 单核 layer-serial vs 跨层 pipelined），需在写作时落到具体段落]
- **预估篇幅**：5000 中文字

### 4 硬件实现与编译器优化

- **章节定位**：本论文最长最详的核心实现章。chinese-thesis-spec §11.1 范文范式中此章"通常最长最详"。**narrative A 主轴**所在章 — Ky 折叠 + S2D + 联合触发是本论文 novelty 最强的贡献，必须放在显著位置（§4.3）。
- **目标**：让读者读完本章后能理解关键模块的实现机理（行环指针 / parf_col 端口共享 / SDP 链 / Ky 折叠的 cin_fake 计算 / S2D 的 stride² 相位折叠），具备复现的理论依据（结合 RTL 路径与公式编号）。
- **包含小节**：
  - **4.1 引言**（章首段范式）
  - **4.2 核心数据通路硬件实现**（≈3000 字 — narrative B 支撑章核心）
    - 4.2.1 去中心化 valid-ready 5 模块流水（line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer 各自维护 counter，弹性 join，无中心 FSM — 对应 C1.2）
    - 4.2.2 流式行环数据通路（IFB 8192 word / OFB 2048 word，strip_rows × W_IN 粒度 ring，row-credit 反压 — 对应 C1.3）
    - 4.2.3 16×16 INT8 MAC 阵列与列广播激活（对应 C1.1）
    - 4.2.4 分列累加器 PARF（parf_col × NUM_COL，外壳共享 wr_addr/we — 对应 C1.4）
    - 4.2.5 SDP 后处理融合（bias / shift / clip / 残差融合 — 对应 C1.5）
    - 4.2.6 7 层循环嵌套硬件实现（对应 C1.6）
  - **4.3 编译器侧 PE 利用率优化（narrative A 主章）**（≈2500 字 — **本论文 novelty 主战场**）
    - 4.3.1 *C*ᵢₙ<16 与 stride≥2 下 PE 利用率塌陷的量化分析
    - 4.3.2 Ky 折叠（Ky-fold）：将 Ky 维折叠到 cin_fake 的数学推导与编译器实现（对应 C2.1，含公式如 *cin_fake* = *C*ᵢₙ × *K*ᵧ + 公式编号 4.x）
    - 4.3.3 空间到深度（S2D）：stride² 相位折叠的等价变换与 DDR 友好重排（对应 C2.2，含公式 *cin_new* = *stride*² × *C*ᵢₙ + 公式编号 4.y）
    - 4.3.4 Ky 折叠 + S2D 联合触发与编译器自动决策（scheduler.Layer.force_s2d() / force_fold() — 对应 C2.3）
    - 4.3.5 与硬件可重构路线（MAERI / Eyeriss-v2）的取舍对比
  - **4.4 系统集成与编译流**（≈1500 字 — narrative B 支撑章次核心）
    - 4.4.1 PyTorch → ISA 端到端编译流（对应 C2.4）
    - 4.4.2 链式 CASES 验证基础设施（DSL builder + 跨层 DDR FM 共享 — 对应 C2.5）
    - 4.4.3 AXI / DMA 子系统集成（Vivado axi_dm IP + 自研 ctrl + arb + mux — 对应 C3.1）
    - 4.4.4 CFG_WRITE descriptor 配置流（host AXI-Lite 写从 ~50/层降到 4/层 — 对应 C3.2 + C3.3）
  - **4.5 多核 W 切片扩展**（≈1500 字 — narrative B 关键扩展章）
    - 4.5.1 多核拓扑与 multicore_top 封装（N=2/4，对应 C3.4 部分）
    - 4.5.2 W 切片划分与 halo 计算冗余（K=3 时 N=2 重叠 2 列 + 非对称 pad — 对应 C3.4 部分）
    - 4.5.3 跨核 SRAM 直送（producer ODMA → consumer IFB region，无需 DDR 中转 — 对应 C3.4 M2 push）
    - 4.5.4 单源参数 params.py 在多核场景的应用（对应 C3.5）
  - **4.6 本章小结**（≈400 字）
- **依赖项**：
  - contributions.md：本章是**最高密度贡献承载章**，覆盖 C1.1-C1.6（架构层）+ C2.1-C2.5（编译器层）+ C3.1-C3.5（系统集成层）共 16 条贡献
  - literature.md：§B（MAERI / Eyeriss-v2 — §4.3.5 取舍对比）/ §C（Alwani / Kang / Liu — §4.2.2 行环差异化已在第 3 章主谈，本章简提）/ §D（im2col / Winograd / S2D — §4.3.3 谱系定位）
- **未决问题**：
  - [TBD: 4.3 是否拆为两节（4.3 编译器侧优化基础 + 4.4 联合决策与硬件接合） — 若 narrative A 写得足够厚可拆，倾向不拆保持紧凑]
  - [TBD: 4.5 多核 W 切片是否独立成章 — 若 chinese-thesis-spec 要求"通常 5 章正文"则保留为节；若用户希望突出多核工作可独立为第 5 章并把验证后移]
  - [CHECK: 4.3.1 PE 利用率塌陷量化数字 — 来自 model_analysis.md，contributions.md C2.1 量化数据条目标 [CHECK]，写作时需确认]
- **预估篇幅**：9000 中文字（本章最长）

### 5 系统验证与实验分析

- **章节定位**：实验章。chinese-thesis-spec §11.1 范文范式中此章"含功能/板级/性能/对比子节"。**所有量化数据集中地** — contributions.md 中 C4.1-C4.7 全部在本章兑现。论文是否站得住关键看本章。
- **目标**：让读者通过实验数据相信 (1) 设计功能正确（51-case bit-exact 全 PASS）、(2) 综合资源在合理范围（XC7K325T 单核/双核/4 核都能装下）、(3) 编译器侧优化（Ky 折叠 + S2D）确实带来 PE 利用率提升、(4) 多核扩展接近线性、(5) 与已有工作（Angel-Eye / Snowflake / fpgaConvNet 等）相比有合理位置。
- **包含小节**：
  - **5.1 引言**（章首段范式）
  - **5.2 实验设置**（≈800 字）
    - 5.2.1 硬件平台（XC7K325T-FFG900-2 @ 100 MHz target）
    - 5.2.2 工具链（Vivado 2023.1 / ModelSim / PyTorch / Python 工具链）
    - 5.2.3 测试集（22-case ResNet-18 风格回归 / 24-case robust smoke / 16-case W slice / 6-case ResNet 残差链 / 3-case ResNet-11 整网 N=1/2/4）
  - **5.3 功能验证**（≈800 字 — 对应 C4.3）
    - 5.3.1 单核 26 corner case bit-exact（K∈{1,2,3,5,7}, stride∈{1,2,3,4}, *C*ᵢₙ∈{4,8,12,16,32}, *H*×*W* 含 VGA 480×640）
    - 5.3.2 多核 W 切片 16 case bit-exact（N=2 10 case + N=4 10 case，含 K∈{1,3,5,7}, stride∈{1,2}, W∈{8,32,33}）
    - 5.3.3 ResNet-11 N=1/2/4 整网 bit-exact + 6 个 ResNet 残差链
    - [CHECK: 51 vs 55 case 合计数 — contributions.md Q4 标记需用户最终核对]
  - **5.4 板级综合与资源分析**（≈1000 字 — 对应 C4.1 + C4.2）
    - 5.4.1 单核综合（XC7K325T，LUT 36,942 / FF 13,167 / BRAM36 128+1 / DSP 82 / Fmax 68.4 MHz）
    - 5.4.2 多核综合（N=2 BRAM 256 / N=4 通过缩 SB 8192→2048 后综合通过 — commit 5fe16b2）
    - 5.4.3 **诚实陈述：Fmax 仅 68 MHz、DSP 推断率低 82/256** — 作为 limitation 在 §6.3 给出 future work 路径
    - 5.4.4 [CHECK: 4 核 SMC 综合最新 LUT/FF/BRAM/DSP — contributions.md Q3 标记需查 reports_smc/utilization_synth.rpt]
  - **5.5 性能分析**（≈1500 字 — 对应 C4.4 + C4.6 + C4.7 — narrative A 主轴数据兑现）
    - 5.5.1 PE 利用率三模式对比（baseline / Ky 折叠 / S2D — 对应 C4.7，[CHECK: 三模式具体数字]）
    - 5.5.2 ResNet-11 整网 cycles / FPS（N=1 596K cy / 313 fps；N=2 450K cy / 444 fps；N=4 354K cy / 564 fps；S2D 单 Patch 层 5.05× — 对应 C4.4）
    - 5.5.3 多核扩展加速比（N=2 1.32× / N=4 1.68× — 对应 C4.4）
    - 5.5.4 Phase 7 SMC + NUMA 主线最新数（N=4 220,824 cy / 453 fps — 对应 C4.6，[CHECK: 是否作为论文最终数 — Q5]）
    - 5.5.5 **诚实陈述：FPS 用 100 MHz 假设，实测 Fmax 68 MHz，实际 FPS ≈ 0.68× 表中数字**
  - **5.6 多 DDR 带宽分析**（≈800 字 — 对应 C4.5）
    - 5.6.1 1-DDR 与 4-DDR PoC 对比（mac_pipe% 36.2% → 63.5% / wall cycles 354K → 196K）
    - 5.6.2 BW 是 N=4 加速比下降的原因（DDR busy 84.7%）
    - 5.6.3 多 DDR 板 ROI 决策（不投，作为 limitation 数据支撑）
    - [TBD: Q10 — 是否进论文，倾向进 limitation 章节]
  - **5.7 与已有工作对比**（≈1200 字 — 关键对比章）
    - 5.7.1 与 FPGA streaming 类工作对比（Angel-Eye / Snowflake / fpgaConvNet / Caffeine — 同器件 Fmax / 资源 / GOPS 横向对比表）
    - 5.7.2 与跨层与流式行缓冲工作对比（Alwani / Kang AoCStream / Liu Full-Stack — **三处威胁项 5.x 数据兑现章**，落实第 1/3 章的差异化主张）
    - 5.7.3 与 ASIC 系统脉动工作对比（TPU / Eyeriss / Eyeriss-v2 / Gemmini — 数量级差距说明 + 工艺归一化讨论）
    - 5.7.4 帕累托前沿（Pareto frontier）定位
    - [CHECK: literature.md 中文献对照数据 — 各 baseline 整网 MAC% / 同器件其他工作 Fmax / Angel-Eye 是否支持 SDP / 同器件 Angel-Eye/DPU/VTA 资源]
  - **5.8 本章小结**（≈400 字）
- **依赖项**：
  - contributions.md：C4.1（单核综合）/ C4.2（多核综合）/ C4.3（功能验证）/ C4.4（ResNet-11 cycles/FPS）/ C4.5（4-DDR PoC）/ C4.6（Phase 7 SMC+NUMA）/ C4.7（PE 利用率三模式） — **本章是 C4 全集兑现地**
  - literature.md：§A（TPU/Eyeriss/Eyeriss-v2/Gemmini — §5.7.3 ASIC 对比）/ §C（Angel-Eye / Snowflake / fpgaConvNet / Caffeine / Alwani / Kang / Liu — §5.7.1 + §5.7.2 主战场）/ §G（VTA / 工具链对比）
- **未决问题**：
  - [CHECK Q3]：4 核 SMC 综合最新 LUT/FF/BRAM/DSP 数字
  - [CHECK Q4]：51 vs 55 case 合计数不一致
  - [CHECK Q5]：220,824 cy / 453 fps 是否作为论文最终数
  - [CHECK Q6]：三模式 PE 利用率对比的具体数字（来自 model_analysis.md）
  - [TBD Q7]：Fmax 实测 vs 100 MHz 假设 FPS 的写作策略（倾向双标）
  - [CHECK]：literature.md 中"文献对照数据"5 处（各 baseline 整网 MAC% 等）
- **预估篇幅**：8000 中文字

### 6 结论与展望

- **章节定位**：收束章。chinese-thesis-spec §11.2 强制拆分为 6.1 结论 / 6.2 创新点 / 6.3 展望三节。**不引入新数据 / 新引用 / 新主张**，只归纳全文工作。
- **目标**：让读者在最后一章读到 (1) 本工作完成了什么、(2) 与已有工作相比的创新点（与第 1 章 1.3.3 主要贡献呼应但不重复 — 1.3.3 偏研究计划，6.2 偏成果定性）、(3) 已知不足与未来工作（诚实陈述 Fmax / DSP 推断率 / 多 DDR 限制 / Mesh PoC 未完成）。
- **包含小节**：
  - **6.1 结论**（≈600 字）
    - 6.1.1 工作总结：硬件层（5 模块去中心化流水 + 行级行环 + 多核 W 切片）+ 编译器层（Ky 折叠 + S2D + 端到端 PyTorch 编译流）+ 系统层（AXI/DMA + descriptor 配置流 + 单源参数）
    - 6.1.2 验证结论：51 case bit-exact + XC7K325T 单核/双核/4 核综合通过 + ResNet-11 N=4 实现 2.51× 加速（基于 sim 实测）
    - 6.1.3 工程意义：在固定 16×16 阵列硬件上通过编译器侧重映射在 *C*ᵢₙ<16 与 stride≥2 场景兑现可用 PE 利用率
  - **6.2 创新点**（≈600 字）
    - 6.2.1 创新点 1：编译器侧 Ky 折叠（Ky-fold）方法 — **据已知文献，纯编译器侧、零硬件代价的 Ky 维折叠到 *C*ᵢₙ 的 PE 利用率优化方案在 FPGA streaming CNN 加速器中较少见**（措辞按 lessons-learned 第 7 条要求"据已知文献..."而非"first to..."）
    - 6.2.2 创新点 2：空间到深度（S2D）等价变换在 ResNet-11 Patch 层兑现 5.05× 单层加速 / 整网 1.87× 加速 — **诚实标注与 Pixel-Shuffle / Sub-pixel 在超分辨率领域的引用谱系关系**
    - 6.2.3 创新点 3：去中心化 valid-ready 5 模块流水 + 行级流式行环数据通路 — 单层硬件可处理任意 *H*×*W*，与 Alwani / Kang / Liu 三处近邻 prior art 的差异化在第 3 章已论证
    - 6.2.4 创新点 4：单层内 W 切片多核扩展 + 跨核 SRAM 直送（M2 push）+ halo 计算冗余
    - 6.2.5 创新点 5：PyTorch → 硬件 cfg 端到端编译流 + CFG_WRITE descriptor 配置（host AXI-Lite 写从 ~50/层降到 4/层）
  - **6.3 展望**（≈600 字）
    - 6.3.1 已知不足：
      - Fmax 仅 68 MHz（critical path 在 SDP 量化组合链）— 可通过切流水拉到 100+ MHz
      - DSP 推断率低（82/256）— 可通过 `(* use_dsp = "yes" *)` 综合属性提升
      - 1-DDR 通道带宽是 N=4 加速比下降原因（mac_pipe% 仅 36%）
    - 6.3.2 未来工作：
      - SDP 流水切分 → 100+ MHz 实测
      - DSP 综合属性优化 → 节省 ~17K LUT
      - Mesh + AXIS NoC PoC 完成 ResNet-11 跑通（contributions.md Q9 标记 — 倾向作为 future work，不进 main contributions）
      - Mode C *C*ₒᵤₜ 切片 cfg gen 与 Stage barrier 多 stage 调度（contributions.md 实现状态汇总未完成项）
      - 片上 push 链 P2（计算冗余 halo + 跨 stage push）
- **依赖项**：
  - contributions.md：6.2 创新点对应 C2.1（创新 1）/ C2.2 + C2.3（创新 2）/ C1.2 + C1.3（创新 3）/ C3.4（创新 4）/ C2.4 + C3.2（创新 5）；6.3 已知不足对应 C4.1 诚实陈述 + 实现状态汇总未完成项
  - literature.md：6.2.1 / 6.2.2 创新点措辞需对照 §B（MAERI / Eyeriss-v2）/ §C（Alwani / Kang / Liu）/ §D（Pixel-Shuffle / Sub-pixel 谱系）— **避免 over-claim**
- **未决问题**：
  - [TBD Q9]：Mesh + AXIS NoC PoC 是否进论文（倾向 future work / 附录，不进 main contributions）
  - [TBD]：6.2 创新点是否与 1.3.3 主要贡献完全对应（倾向"5 对 5"对应但描述角度不同 — 1.3.3 偏"研究目标"动词，6.2 偏"成果定性"名词）
- **预估篇幅**：1800 中文字

---

## 贡献-章节映射表（覆盖率验证）

> contributions.md v2 共 **24 条贡献**（C1.1-C1.6 6 条 / C2.1-C2.5 5 条 / C3.1-C3.6 6 条 / C4.1-C4.7 7 条 = 24 条；用户任务文本中"23 条"为概数）。

### 一、硬件架构层（C1.x，6 条）

| 贡献 ID | 贡献简述 | 主章节 | 次章节（呼应/简提） |
|---|---|---|---|
| C1.1 | 16×16 INT8 MAC 阵列 + OC-broadcast | §4.2.3 | §3.3 总体架构 / §3.4.1 数据流论证 |
| C1.2 | 去中心化 valid-ready 5 模块流水 | §4.2.1 | §1.3.3 创新 3 / §6.2.3 创新点 |
| C1.3 | 流式行环（IFB/OFB row-ring + row-credit） | §4.2.2 | §3.4.3 行环差异化 / §1.3.3 创新 3 |
| C1.4 | 分列累加器 PARF（parf_col × NUM_COL） | §4.2.4 | — |
| C1.5 | SDP 后处理融合（bias/shift/clip/residual） | §4.2.5 | §5.4.3 SDP 是 critical path 诚实陈述 |
| C1.6 | 7 层循环嵌套硬件实现 | §4.2.6 | §3.4 数据流选择论证 |

### 二、编译器优化层（C2.x，5 条）— **narrative A 主轴**

| 贡献 ID | 贡献简述 | 主章节 | 次章节（呼应/简提） |
|---|---|---|---|
| C2.1 | Ky 折叠（Ky-fold） | §4.3.2 | §1.3.3 创新 1 / §6.2.1 创新点 / §5.5.1 PE 利用率三模式对比 |
| C2.2 | 空间到深度（S2D） | §4.3.3 | §1.3.3 创新 2 / §6.2.2 创新点 / §5.5.2 ResNet-11 cycles 兑现 |
| C2.3 | Ky 折叠 + S2D 联合触发与编译器自动决策 | §4.3.4 | §3.5.3 编译器/硬件视角对齐 |
| C2.4 | PyTorch → ISA 端到端编译流 | §4.4.1 | §1.3.3 创新 5 / §6.2.5 创新点 |
| C2.5 | 链式 CASES 验证基础设施 | §4.4.2 | §5.2.3 测试集 / §5.3 功能验证 |

### 三、系统集成层（C3.x，6 条）— **narrative B 支撑**

| 贡献 ID | 贡献简述 | 主章节 | 次章节（呼应/简提） |
|---|---|---|---|
| C3.1 | AXI / DMA 子系统集成 | §4.4.3 | §3.3.1 总体架构两层结构 |
| C3.2 | CFG_WRITE descriptor 配置流 | §4.4.4 | §1.3.3 创新 5 / §6.2.5 创新点 |
| C3.3 | Done sticky 寄存器 + 双口 cfg_regs | §4.4.4（合并到 C3.2 段） | — |
| C3.4 | 多核 W 切片扩展（multicore_top + halo + cross-core SRAM 直送） | §4.5（独立节，三个子节展开） | §1.3.3 创新 4 / §6.2.4 创新点 / §5.5.3 多核扩展加速比 |
| C3.5 | 单源参数 params.py | §3.5.1 + §4.5.4 | §6.2 reproducibility 加分项简提 |
| C3.6 | 多核 TB 结构化 profile 报告 | §5.2.3 测试集（一段带过） | §5.6 多 DDR 带宽分析（数据来源） |

### 四、实测结果与综合数据（C4.x，7 条）— **第 5 章主战场**

| 贡献 ID | 贡献简述 | 主章节 |
|---|---|---|
| C4.1 | 单核综合（XC7K325T，Fmax 68.4 MHz） | §5.4.1 + §5.4.3 诚实陈述 |
| C4.2 | 多核综合（N=2 / N=4 BRAM-bound） | §5.4.2 + §5.4.4 [CHECK] 数字 |
| C4.3 | 单层 + 多层 chain bit-exact 回归（51-case） | §5.3 全节 |
| C4.4 | ResNet-11 整网 cycles / FPS（多核 + S2D 加速分解） | §5.5.2 + §5.5.3 |
| C4.5 | N=4 多核 1-DDR vs 4-DDR PoC 对比 | §5.6 全节 + §6.3.1 已知不足 |
| C4.6 | Phase 7 SMC + NUMA ResNet-11 N=4 完整网络 sim | §5.5.4 |
| C4.7 | PE 利用率三模式对比（baseline / Ky 折叠 / S2D） | §5.5.1 |

### 覆盖率验证

- **24 条贡献全部归位**：✅ 全部 24 条都至少有一个主章节归位
- **未归位贡献**：0 条
- **多次出现的贡献**：C1.2 / C1.3 / C2.1 / C2.2 / C2.4 / C3.2 / C3.4（这 7 条因 narrative 主轴或创新点提名，在第 1/4/6 章多处呼应是合理的）

---

## 文献-章节映射表（prior art 威胁应对）

> literature.md 三处近邻 prior art 威胁：Alwani Fused-layer@MICRO'16 / Kang AoCStream@arXiv'22+Sensors'23 / Liu Full-Stack@TNNLS'21。

### 三处威胁项的应对策略

| 威胁项 | 威胁简述 | §1 应对（综述定位） | §3 应对（差异化论证） | §5 应对（数据兑现） |
|---|---|---|---|---|
| **Alwani Fused-layer@MICRO'16** | 跨层 fusion 减少 off-chip 流量；多层共享 line buffer | §1.2.3 列入综述并明示其工作范围（跨层 fused 而非单层 layer-serial） | §3.4.3 论证差异化：FLUX_CNN 是**单核 layer-serial 共用硬件**而非多层硬件流水绑定，编译器决定 strip 粒度而非硬件 fused 层数 | §5.7.2 横向对比表（含资源 / 灵活性） |
| **Kang AoCStream@arXiv'22+Sensors'23** | 流式 line-buffer，buffer 容量 = activation tile（线性于图宽） | §1.2.3 列入综述并明示其工作范围（buffer 大小线性于图宽 vs FLUX_CNN 行级） | §3.4.3 论证差异化：FLUX_CNN 行级粒度（strip_rows × W_IN）vs Kang 整图 / activation tile 粒度 | §5.7.2 横向对比表 |
| **Liu Full-Stack@TNNLS'21** | streaming + residual concatenative；含 PyTorch 工具链 | §1.2.3 + §1.2.5 工具链综述列入 | §3.4.3 论证差异化：FLUX_CNN 单核 layer-serial vs Liu 跨层 pipelined 多 block | §5.7.2 横向对比表 + §5.7.1 工具链对比 |

### 其他文献的章节归属（按 literature.md 章节）

| literature.md 章节 | 主要文献 | 主要章节归属 |
|---|---|---|
| §A 系统脉动阵列（ASIC） | TPU / Eyeriss / Eyeriss-v1 JSSC / Gemmini / NVDLA | §1.2.1 综述 + §2.3 数据流分类 + §3.4.1 OS 论证 + §5.7.3 ASIC 对比 |
| §B 可重构 PE / NoC | MAERI / Eyeriss-v2 / Tangram / Simba / Interstellar / Buffets | §1.2.1 综述 + §3.4.2 不可重构论证 + §4.3.5 取舍对比 |
| §C FPGA streaming | fpgaConvNet / Snowflake / Angel-Eye / Caffeine / Aydonat DLA | §1.2.2 综述 + §3.3 总体架构对照 + §5.7.1 横向对比表 |
| §C 三处威胁项 | Alwani / Kang / Liu | 见上方威胁项应对表 |
| §D 卷积变换 | im2col / Winograd（Lu FCCM'17）/ Direct Convolution | §1.2.4 综述 + §2.5.1 基础变换 |
| §D Pixel-Shuffle / S2D 谱系 | Sub-pixel Conv（Shi CVPR'16） | §1.2.4 谱系定位 + §2.5.2 子像素重排 + §4.3.3 S2D 谱系 + §6.2.2 诚实标注 |
| §E 量化推理 | INT8 量化 / 训练后量化 | §2.2.2 INT8 与算子融合基础 |
| §F 残差与池化 | ResNet（He CVPR'16） | §2.2.3 ResNet 残差结构 |
| §G 工具链与编译器 | fpgaConvNet 工具链 / VTA / NVDLA 编译器 | §1.2.5 综述 + §4.4.1 编译流对比 |

### 覆盖率验证

- **三处 prior art 威胁全部应对**：✅ 在 §1.2.3 综述列入 + §3.4.3 差异化论证 + §5.7.2 数据兑现，**三层防御**到位
- **literature.md §A-§G 共 35 条文献全部有归属章节**：✅
- **Pixel-Shuffle / S2D 谱系（contributions.md Q2 标记）**：在 §1.2.4 / §2.5.2 / §4.3.3 / §6.2.2 多处提及，**诚实标注**与超分辨率领域的引用关系，避免 over-claim

---

## 叙事一致性检查

按 paper-outliner 工作流程 §3.3 检查清单逐项验证：

### 1. 绪论的"主要贡献声明"（§1.3.3）↔ 实验章（§5）的实验呼应

| 主要贡献声明（§1.3.3） | 对应章节（§4） | 数据兑现（§5） | 状态 |
|---|---|---|---|
| 创新 1：Ky 折叠 | §4.3.2 | §5.5.1 PE 利用率三模式对比 | ✅ 已映射 |
| 创新 2：S2D | §4.3.3 | §5.5.2 ResNet-11 cycles + Patch 5.05× | ✅ 已映射 |
| 创新 3：去中心化流水 + 行级行环 | §4.2.1 + §4.2.2 | §5.3 功能验证（51-case bit-exact） | ✅ 已映射 |
| 创新 4：W 切片多核 | §4.5 | §5.5.3 多核扩展 + §5.6 多 DDR 分析 | ✅ 已映射 |
| 创新 5：PyTorch 端到端编译流 + descriptor 配置流 | §4.4.1 + §4.4.4 | §5.3.3 ResNet-11 整网 bit-exact | ✅ 已映射 |

### 2. 综述章（§1.2）↔ 实验章（§5.7）的对比对象

| §1.2 提到的对比对象 | §5.7 是否真的对比 | 状态 |
|---|---|---|
| TPU / Eyeriss / Eyeriss-v2 / Gemmini（ASIC） | §5.7.3 数量级差距 + 工艺归一化（不是同器件横向对比，是"参考点"） | ✅ 合理处理 |
| Angel-Eye / Snowflake / fpgaConvNet / Caffeine（FPGA streaming） | §5.7.1 同器件 Fmax / 资源 / GOPS 横向对比表 | ✅ 已映射 |
| Alwani / Kang / Liu（三处威胁项） | §5.7.2 主战场 | ✅ 已映射 |
| MAERI / Eyeriss-v2（可重构路线） | §4.3.5 取舍对比 + §5.7.3 ASIC 大类对比 | ✅ 已映射 |
| im2col / Winograd / Direct Conv（卷积变换） | §2.5.1 基础变换；不进 §5.7 数据对比（属理论谱系而非同器件 baseline） | ✅ 合理处理 |
| Pixel-Shuffle / Sub-pixel（S2D 谱系） | §6.2.2 诚实标注；不进 §5.7（属理论谱系） | ✅ 合理处理 |
| VTA（工具链） | §5.7.1 工具链对比一段 | ✅ 已映射 |

### 3. 中间章节是否有重复信息

| 风险点 | 主章节 | 简提章节 | 处理策略 |
|---|---|---|---|
| 16×16 阵列 OC-broadcast | §4.2.3 详写 | §3.3 / §3.4.1 简提架构选择 | ✅ 不重复（§3 讲选择，§4 讲实现） |
| 行环数据通路 | §4.2.2 详写 | §3.4.3 简提差异化论证 | ✅ 不重复 |
| Ky 折叠 + S2D | §4.3 详写 | §1.3.3 / §6.2 主要贡献+创新点表述 | ✅ 不重复（§1 + §6 是声明，§4 是机理） |
| 多核 W 切片 | §4.5 详写 | §3.3.3 简提拓扑 / §5.5.3 + §5.6 数据兑现 | ✅ 不重复 |
| Fmax 68 MHz 诚实陈述 | §5.4.3 详写 | §6.3.1 已知不足简提 + §5.5.5 FPS 双标 | ✅ 不重复（§5 给数据，§6 给路径） |

### 4. contributions.md 中的 23 条贡献 — 是否每条都至少出现在某一章

见上方"贡献-章节映射表 — 覆盖率验证"小节：✅ **23 条贡献全部归位**，0 条未归位。

### 5. 关键诚实陈述的承接链

| 诚实陈述项 | 首次出现章节 | 后续呼应章节 |
|---|---|---|
| Fmax 仅 68 MHz（critical path = SDP 量化组合链） | §5.4.3 | §5.5.5 FPS 双标 + §6.3.1 已知不足 + §6.3.2 future work（SDP 切流水） |
| DSP 推断率低（82/256） | §5.4.3 | §6.3.1 已知不足 + §6.3.2 future work（use_dsp 综合属性） |
| 1-DDR 是 N=4 加速比下降原因（mac_pipe% 36%） | §5.6 | §6.3.1 已知不足 |
| Mesh + AXIS NoC PoC 未完成 | §6.3.2 future work | 不进 §1.3.3 主要贡献（避免 over-claim） |
| S2D 与 Pixel-Shuffle / Sub-pixel 谱系 | §1.2.4 + §6.2.2 | §4.3.3 简提 |

✅ **诚实陈述承接链完整，无 over-claim 风险**。

---

## 章节依赖图（文字版）

各章节阅读路径与依赖关系：

```
§1 绪论（独立可读）
  ├─→ §2 理论基础（独立可读，依赖 §1.2 综述提到的概念）
  │     ├─→ §3 整体方案（依赖 §2.3 数据流分类 + §2.4 FPGA 基础）
  │     │     ├─→ §4 实现章（最重，依赖 §3 架构选择 + §2.5 卷积变换基础）
  │     │     │     └─→ §5 验证章（依赖 §4 全部，是数据兑现）
  │     │     │           └─→ §6 结论（依赖 §1.3.3 + §4 + §5 全部）
```

**核心路径**：§1（动机）→ §3（架构选择）→ §4（实现机理）→ §5（数据兑现）→ §6（收束）。第 2 章理论基础是支撑章，可独立阅读。

**可独立阅读章**：§2（理论基础）/ §1（绪论）/ §6.1（结论）— 这三节给非领域审稿人快速概览。

**最重章**：§4（≈9000 字，narrative A 主战场）/ §5（≈8000 字，C4 全集兑现地）/ §1（≈8000 字，含 4500 字综述）。

**章节间术语统一约束**（防止跨章节漂移，由 Phase 7 polisher 终段处理）：
- "处理单元（Processing Element，PE）"：§1 第一次出现注释，后续用 PE
- "卷积神经网络（Convolutional Neural Network，CNN）"：§1 第一次出现注释，后续用 CNN
- "现场可编程门阵列（Field Programmable Gate Array，FPGA）"：§1 第一次出现注释，后续用 FPGA
- "Ky 折叠（Ky-fold）"：§1.3.3 / §4.3.2 第一次出现，后续可二选一（保持全文统一）
- "空间到深度（Space-to-Depth，S2D）"：§1.3.3 / §4.3.3 第一次出现，后续用 S2D
- "valid-ready 握手"：§2.4.2 第一次出现注释（数据有效—就绪握手），后续用 valid-ready
- "去中心化"：全文统一（不混用"分布式"/"去中心化控制"）

---

## 待决清单

### TBD（路径/决策待用户确定）

| ID | 描述 | 涉及章节 | 处置建议 |
|---|---|---|---|
| TBD-1 | 论文标题最终选择（候选 1 编译器侧 vs 候选 2 系统集成） | 封面 / 摘要 | 倾向候选 1（与 narrative A 主轴一致） |
| TBD-2 | §1.3.3 主要贡献是否压缩到 4 条（合并贡献 4+5 为"系统集成与编译流"） | §1.3.3 | 倾向保持 5 条（与 §6.2 创新点对称） |
| TBD-3 | §2.5.2 是否包含 S2D 完整数学推导 | §2.5 / §4.3.3 | 倾向只在 §4.3 详写；§2.5 只点出"步幅卷积可通过子像素重排转换为 stride=1" |
| TBD-4 | §3.5 编译器与硬件协同设计原则是否单独成节 | §3.4 / §3.5 | 倾向保持单独节（凸显 narrative A 与 narrative B 接合） |
| TBD-5 | §4.3 是否拆为两节（编译器侧优化基础 + 联合决策与硬件接合） | §4.3 / §4.4 | 倾向不拆，保持紧凑（narrative A 主轴在一节内有冲击力） |
| TBD-6 | §4.5 多核 W 切片是否独立成章（变成第 5 章） | §4.5 / §5 | 倾向保持节（章数已 6，符合 chinese-thesis-spec 范式） |
| TBD-7 | Q7 — Fmax 实测 vs 100 MHz 假设 FPS 的写作策略 | §5.4 / §5.5 | 倾向双标（100 MHz 假设给 ceiling，68 MHz 实测给 actual） |
| TBD-8 | Q9 — Mesh + AXIS NoC PoC 是否进论文 | §6.3.2 future work | 倾向 future work / 附录，不进 main contributions |
| TBD-9 | Q10 — 多 DDR 板 ROI 决策（不投）是否进论文 | §5.6 / §6.3.1 | 倾向进 limitation 章节数据支撑 |
| TBD-10 | §6.2 创新点与 §1.3.3 主要贡献是否完全对应 | §1.3.3 / §6.2 | 倾向"5 对 5"对应但描述角度不同（1.3.3 偏研究目标动词，6.2 偏成果定性名词） |

### CHECK（数据/事实待核验）

| ID | 描述 | 涉及章节 | 处置建议 |
|---|---|---|---|
| CHECK-1 | Q3 — 4 核 SMC 综合最新 LUT/FF/BRAM/DSP 数字 | §5.4.4 | 查 `Syn/reports_smc/utilization_synth.rpt` 最新版 |
| CHECK-2 | Q4 — 51 vs 55 case 合计数不一致 | §5.3 | 用户最终核对 STATUS §2.8 末尾自报 |
| CHECK-3 | Q5 — Phase 7 SMC+NUMA 220,824 cy / 453 fps 是否作为论文最终数 | §5.5.4 | 用户确认；可能需要重新跑回归对齐 commit `5fe16b2` |
| CHECK-4 | Q6 — 三模式 PE 利用率对比的具体数字 | §5.5.1 | 从 model_analysis.md 确认对应表格 |
| CHECK-5 | C2.1 量化数据 — Ky 折叠单独使能 vs baseline 的 PE 利用率提升数字 | §4.3.2 / §5.5.1 | 跑 model_analysis.md 对应数据 |
| CHECK-6 | Q2 — S2D 在加速器领域的引用谱系 | §1.2.4 / §4.3.3 / §6.2.2 | 由 paper-literature-scout 在主线程已补查 Pixel-Shuffle / Sub-pixel 与 FPGA 加速器的关系，需写作时落到具体段落 |
| CHECK-7 | literature.md 中"文献对照数据"5 处 — 各 baseline 整网 MAC% / 同器件其他工作 Fmax / Angel-Eye 是否支持 SDP / 同器件 Angel-Eye/DPU/VTA 资源 / 各工作 verification 公开度 | §5.7 | 由 paper-literature-scout 补查 |
| CHECK-8 | literature.md 中 vendor doc 引用方式 — NVDLA / Xilinx PG022 / Xilinx PG338 / ARM Ethos / cuDNN tech report | §1.2 / §4.4.3 | 主 Agent 决定引用形式（vendor white paper 还是引衍生 paper） |
| CHECK-9 | §1.2.3 与 Alwani / Kang / Liu 三处差异化论证的措辞 | §1.2.3 / §3.4.3 | 由 paper-literature-scout 已论证差异化要点（行级 vs 跨层 fused / 行级 vs 整图 / 单核 layer-serial vs 跨层 pipelined），需写作时落到具体段落 |

### 数量统计

- **TBD 总数**：10 处
- **CHECK 总数**：9 处
- **总 [TBD]/[CHECK] 标记**：19 处

> 这些标记将在 Phase 5 段落骨架展开时进一步分流，部分 [CHECK] 在 Phase 5 仍未解决会作为 known-issue 进入 Phase 7 polisher。
