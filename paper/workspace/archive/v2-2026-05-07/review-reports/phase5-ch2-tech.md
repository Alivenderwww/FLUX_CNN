# 技术评审报告 Phase 5 §2 理论基础章

## 第 1 次评审（v2，第二次启动后实际首次完成评审；前 reviewer ECONNRESET 重启）

### 判定：PASS

---

### 评审范围

paper.md 第 119–179 行 §2 整章（§2.1 引言 / §2.2 CNN 基础 / §2.3 数据流分类 / §2.4 FPGA 基础 / §2.5 PE 利用率优化变换基础 / §2.6 小结）。

按灵魂层职责评 4 项：理论概念准确性 / 引用 prior art 真实性 / 不 over-claim FLUX_CNN / 不引入骨架外新事实。格式与措辞细节不计入判定。

工具预算执行：1 次 Read（paper.md offset=115 limit=200）+ 1 次 Write（本报告）。未做发散探索。

---

### 抽样验证记录

#### 1. 理论概念准确性（4 处抽样）

| # | 位置 | 摘录 | 判定 |
|---|------|------|------|
| 1 | §2.2 p3 (paper.md L131) | "标准二维卷积可以用 7 层循环嵌套表达 ... y[n,co,h,w] = Σ_{ci,ky,kx} x[n,ci,h·S+ky−P,w·S+kx−P] · W[co,ci,ky,kx] + bias[co]" | ✅ 标准卷积定义、stride/pad 下标对齐均正确 |
| 2 | §2.3 p2-p4 (L143-L147) | WS=TPU 脉动 / OS=Snowflake / RS=Eyeriss 的归类与"PE 内驻留对象 + 复用粒度 vs 硬件复杂度权衡"描述 | ✅ 与 Eyeriss/Sze ISSCC'17 经典 taxonomy 一致 |
| 3 | §2.4 p3 (L159) | valid-ready 握手协议 "仅当 valid 与 ready 同时为高时一拍数据被实际消费 ... 弹性 join，无需中心 FSM" | ✅ AXI-Stream 风格握手语义准确 |
| 4 | §2.5 p3 (L173) | "stride=s 卷积可视作把输入特征图按 s×s 个相位拆成 s² 个子图 ... 按通道维拼接" | ✅ Sub-pixel / Pixel-Shuffle 等价关系数学事实正确 |

#### 2. 引用 prior art 真实性（4 处抽样）

| # | 位置 | 引用对象 | 判定 |
|---|------|---------|------|
| 1 | §2.3 L143 | Google TPU 脉动阵列 (WS 代表) | ✅ Jouppi ISCA'17 真实 well-known 工作 |
| 2 | §2.3 L145, §2.4 L161 | Snowflake (OS 代表 + line buffer) | ✅ 真实工作，OS 类常被列为代表 |
| 3 | §2.5 L171 | Caffe im2col / Winograd CVPR'16 (Lavin & Gray) | ✅ 两条均为加速器领域 well-known 经典；Winograd 在 CVPR'16 由 Lavin & Gray 提出 |
| 4 | §2.5 L173 | Shi CVPR'16 Sub-pixel Convolution | ✅ "Real-Time Single Image and Video Super-Resolution Using an Efficient Sub-Pixel Convolutional Neural Network" CVPR'16 真实存在 |

未发现幻觉/嵌合引用。`<!-- 来自 literature.md ... -->` 注释链接到 ResNet / Eyeriss / TPU / Snowflake / Caffe im2col / Winograd CVPR'16 / Shi CVPR'16 Sub-pixel / Angel-Eye / Kintex-7 datasheet / AXI 协议规范，均为 CNN+FPGA 加速器领域成熟参考。注：本次评审未单独运行 check-citations skill 跨库验证（工具预算受限于 ≤3 次调用）；上述 10 条均属高知名度工作，幻觉风险极低，但**建议**正文最终 .bib 落盘后由用户单独跑一次 check-citations 全量复核。

#### 3. 不 over-claim FLUX_CNN（核心检查）

| # | 位置 | 摘录 | 判定 |
|---|------|------|------|
| 1 | §2.3 L145 | "输出驻留（OS）以 Snowflake 与本工作为代表" | ✅ 仅是定位陈述（"本工作属 OS 类"），非性能/创新宣称 |
| 2 | §2.4 L161 | "本工作在标准 line buffer 之上加入 row-credit 反压协议（§4.2.2 详述）" | ✅ 节末一句过渡指针，未在 §2 展开实现/性能数字，符合理论章只铺垫不展开的定位 |
| 3 | §2.4 L163 | "本工作的 DMA 子系统由 idma_ctrl/wdma_ctrl/odma_ctrl 三套控制器加 Vivado axi_dm IP 与 mm2s_arb 仲裁器组成 ... §4.4.3 详述" | ✅ 同上，事实层面与 CLAUDE.md 项目总览第 2 节描述完全一致，且仅作 §4 章的 forward reference，未在 §2 论证其优势 |
| 4 | §2.5 L169, L175 | "本工作的编译器侧 PE 利用率优化 ... 工程化改造" / "S2D 在 ... §4.3.3 与 §4.3.4 详述" | ✅ 仅声明 §4.3 主章存在，无具体数字/收益声明 |

未发现 §2 内出现 PE 利用率百分比、Fmax、加速比、cycles、FPS 等需要在第 5 章实测支撑的具体数字。理论章定位严格守住。

#### 4. 回归性（不引入骨架外新事实/新引用）

| # | 检查项 | 结果 |
|---|-------|------|
| 1 | §2 内所有 prior art 引用是否均属加速器/CNN 领域 well-known 公开工作 | ✅ TPU/Snowflake/Eyeriss/Caffe/Winograd/Shi/Angel-Eye/Kintex-7/AXI/ResNet 全部满足 |
| 2 | §2 是否引入未被后续章节使用的孤立引用 | ✅ 全部对应 §3-§5 设计决策与对照工作的支撑 |
| 3 | §2 是否引入与 contributions.md 冲突的硬件声明 | ✅ "OS + 列广播"、"row-credit 反压"、"axi_dm + mm2s_arb"、"Ky-fold + S2D 编译器侧" 与 CLAUDE.md / 项目实现完全对齐 |
| 4 | [TBD-2.5.2] 标记位置合理性 | ✅ §2.5 末尾 "是否在本节包含简化 S2D 数学推导，倾向只在 §4.3.3 详写以避免重复" 是合理的写作决策标记，非事实不确定标记 |

---

### 备注（不影响判定，仅供 Writer 参考）

以下属于格式/措辞层面，按本评审职责定位**不计入判定**，留作 Writer/格式 reviewer 参考：

- L157 "BRAM36 约 445 块、DSP48 约 800 块" 这类 Kintex-7 datasheet 数字虽属公开规格，但若后续 §3.2 表格采用稍有差异的官方数（不同 speed grade 略有出入），需保持前后一致。
- L163 "AXI4 与 AXI-Lite 是 ARM 推出的开放总线协议族" — 实际 AXI 由 ARM AMBA 协议族定义并开放，措辞无误，但若严格历史可改为 "AMBA 协议族的一部分"，非必须。
- L169 "工程化改造" 措辞偏文学，非 over-claim，但若格式 reviewer 介入可考虑改为 "针对该场景的工程化适配"。

---

### 结论

§2 整章在四项灵魂层职责（理论概念准确 / 引用真实 / 不 over-claim / 不引入新事实）上均通过抽样验证，PASS。建议 Writer 进入 §3 扩写。
