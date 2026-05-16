# FLUX_CNN 论文配图设计方案（v2, 2026-05-12）

## 0. 总体目标

- **覆盖率**：paper.md 共有 38 个需配图的 `### x.y` 二级小节（第 6 章 3 个结论小节按用户要求排除），目标平均 ≥0.8 张/节 → **至少 31 张图**。
- **风格统一**：严肃学术论文风格（IEEE / JSSC 期刊），白底黑字，配色限于深蓝 `#1f4e79` / 深绿 `#2e7d32` / 深金 `#b8860b` / 学术深灰 `#404040`；禁用卡通、阴影、彩色渐变、3D 立体。
- **现状**：已有示意稿 13 张（[figures/](workspace/figures)）+ 已渲染数据图 9 张（[figures-rendered/](figures-rendered)）+ 7 张 markdown 表 = **22 图 + 7 表**。距 31 张图还差 **≥9 张图 + 若干算法**。
- **工具优先级**（用户共识）：**手搓效率比不过工具使用**——能用现成工具自动生成的不手画。

| 图类 | 默认工具 | 说明 |
|---|---|---|
| 数据分析图 | **matplotlib** | 复用 `figures-rendered/_style.py`，统一字体 / 配色 / 300 DPI |
| 时序图、波形图 | **WaveDrom** | JSON 描述 → SVG，浏览器或 wavedrom-cli 渲染 |
| 抽象示意图 | **Claude Design**（默认）/ TikZ（数学示意）/ draw.io | 灵活选 |
| 架构图 | **Claude Design**（默认）/ draw.io | 灵活选 |
| 代码图（伪代码） | **LaTeX `algorithm2e`**（`\SetAlgoVlined` 竖线缩进 + `\KwIn/\KwOut`） → 编译 PDF → `pdftoppm` 转 PNG | 渲染为图像嵌入，**不用裸 markdown 代码块**；优先与示意图组合（DepFiN Fig. 4 风格） |

---

## 1. 小节统计 + 图覆盖矩阵

下表只数实质性 `### x.y` 小节（不含"引言"和"本章小结"——它们按惯例不放图），命中规则：现有 ✅ / 待新增 ➕ / 共享上一节 ↩。

| 章 | 小节 | 当前图 | 状态 | 新图方案 | 工具 |
|---|---|---|---|---|---|
| **1 绪论** | 1.1 研究背景与意义 | — | ➕ | **图 1.1** CNN 端侧应用场景全景（视频监控/工业视觉/消费电子/自动驾驶四象限） | Claude Design |
|  | 1.2 国内外相关研究工作进展 | 表 1.1 | ➕ | **图 1.2** 加速器演进谱系树（DianNao→Eyeriss→TPU→VTA→NVDLA→本工作） | Claude Design |
|  | 1.3 本论文的主要研究内容 | — | ➕ | **图 1.3** 本论文五大创新点思维导图（分层调度/固定阵列任意形状/Ky-fold+S2D/W 切片+SMC/DSP 跨列） | Claude Design |
|  | 1.4 论文组织结构 | — | ➕ | **图 1.4** 章节关系导航图（6 章树形） | Claude Design |
| **2 理论基础** | 2.2 卷积神经网络基础 | 图 2.1（fig2-1） | ✅ | 已有示意稿 | Claude Design 出图 |
|  | 2.3 数据流分类与硬件复用策略 | — | ➕ | **图 2.2** WS / OS / RS 三种数据流对比示意（统一坐标系示数据复用方向） | Claude Design / TikZ |
|  | 2.4 硬件加速基础技术 | — | ➕ | **图 2.3** 流水线 + valid-ready 握手时序图 | TikZ (timing) |
|  | 2.5 非一致性内存访问架构 | — | ➕ | **图 2.4** NUMA 拓扑 + 跨节点访问延迟示意 | Claude Design |
|  | 2.6 存储层级与访存功耗差异 | — | ➕ | **图 2.5** 存储能耗金字塔（Horowitz [16] 数据：寄存器 1pJ → SRAM 5pJ → DRAM 640pJ） | matplotlib（柱图，已知数据） |
|  | 2.7 模型量化基础 | 图 2.2（fig2-2） | ✅ | 已有示意稿 | Claude Design 出图 |
| **3 总体方案** | 3.2 设计目标与约束 | — | ➕ | **图 3.1** 设计目标雷达图（吞吐/能效/灵活性/资源/可演进 5 维 × 端侧/数据中心两类目标对比） | matplotlib（雷达） |
|  | 3.3 总体架构 | 图 3.1（fig3-1） | ✅ | 已有示意稿 → 重命名为 **图 3.2**；补 **图 3.3** 核内数据通路（fig3-2 已有）、**图 3.4** DMA 子系统（fig3-3 已有） | Claude Design 出图（共 3 张） |
|  | 3.4 调度方案 | — | ➕ | **图 3.5** 分层调度示意（核内 6 级 FSM 嵌套树 + 核间 Task Descriptor 链） | Claude Design |
|  | 3.5 数据流选择 | — | ➕ | **图 3.6** 本工作三层数据复用叠加图（权重静止 + 滑窗复用 + 输出广播） | Claude Design |
| **4 加速器实现** | 4.2 行缓存模块 | 图 4.1（fig4-1） | ✅ |  | Claude Design |
|  | 4.3 MAC 阵列模块 | 图 4.2（fig4-2） | ✅ |  | Claude Design |
|  | 4.4 部分和累加模块 | 图 4.3（fig4-3） | ✅ |  | Claude Design |
|  | 4.5 SDP 后处理模块 | 图 4.4（fig4-4） | ✅ |  | Claude Design |
|  | 4.6 权重缓存模块 | 图 4.5（fig4-5） | ✅ |  | Claude Design |
|  | 4.7 配置寄存器与 FSM | 图 4.6（fig4-6） | ✅ |  | Claude Design |
|  | 4.8 Y 维折叠 | 图 4.9（fig4-9） | ✅ |  | Claude Design |
|  | 4.9 空间到深度 | — | ➕ | **图 4.10** S2D 重排示意（输入像素 4 相位 → Cin 通道 4 倍扩张） | Claude Design |
|  | 4.10 DMA 子系统 | 图 4.7（fig4-7） | ✅ |  | Claude Design |
|  | 4.11 多核 W 切片 | 图 4.8（fig4-8） | ✅ | 建议补 **图 4.12** 跨核 SRAM 直送时序示意 | TikZ (timing) |
|  | 4.12 DSP 跨列复用 | 图 4.10（DSP map） | ✅ |  | Claude Design |
| **5 验证与性能分析** | 5.2 验证环境与方法 | — | ➕ | **图 5.0** 验证流程图（PyTorch 浮点参考 → 编译器 → TB 仿真 → 逐字节比对） | Claude Design |
|  | 5.3 功能仿真验证 | 表 5.1 | — | 表足够 | — |
|  | 5.4 板级实现验证 | 表 5.2 / 5.3 | ➕（可选） | **图 5.0b** Vivado 布线后 FPGA die-shot 占用示意（LUT/BRAM/DSP 分布） | Claude Design（可选） |
|  | 5.5 性能分析 | **图 5.1~5.7**（fig5-4/5-7~12） | ✅ | 已有 9 张 matplotlib | matplotlib（已完成） |
|  | 5.6 与已有工作对比 | 表 5.6 + **图 5.8/5.9**（fig5-13/16） | ✅ | 已有 2 张 | matplotlib（已完成） |
| **6 结论与展望** | 6.1 结论 | — | ➕（可选） | **图 6.1** 核心指标摘要雷达（PE 利用率/吞吐/能效/资源占用/Fmax 5 维度） | matplotlib |
|  | 6.2 创新点 | — | ➕（可选） | **图 6.2** 5 大创新点 vs 业界基线对比柱图 | matplotlib |
|  | 6.3 展望 | — | ➕（可选） | **图 6.3** 短/中/长期工作路线时间轴 | Claude Design |

### 1.1 数量复核

| 类别 | 数量 | 备注 |
|---|---|---|
| 已有示意稿（待 Claude Design 出图） | 13 | figures/ |
| 已渲染 matplotlib 数据图 | 9 | figures-rendered/ |
| **新增必做** | **8** | 图 1.1–1.4、图 2.2–2.5、图 4.10（部分共享 §2 范围）见下表 |
| **新增可选** | **5** | 图 4.12、图 5.0、图 5.0b、图 6.1–6.3 |
| **合计**（必做） | **30 图** | 距 33 张缺 3 张 → 把可选里的图 5.0 + 图 6.1 + 图 6.3 升级为必做即满足 |

### 1.2 最终建议执行清单（33 图）

1. **复用现有 figures 示意稿（13）**：fig2-1 fig2-2 fig3-1 fig3-2 fig3-3 fig4-1 fig4-2 fig4-3 fig4-4 fig4-5 fig4-6 fig4-7 fig4-8 fig4-9 fig4-10 → 共 15 张（数错一下，下面重数）

实际清单（按 paper.md 章节编号顺序）：

| 章 | 论文图号 | 标题 | 来源 | 工具 | 状态 |
|---|---|---|---|---|---|
| 1 | 图 1.1 | CNN 端侧应用场景全景 | ➕新增 | **Claude Design** | 待绘 |
| 1 | 图 1.2 | CNN 加速器演进谱系 | ➕新增 | **Claude Design** | 待绘 |
| 1 | 图 1.3 | 本论文五大创新点思维导图 | ➕新增 | **Claude Design** | 待绘 |
| 1 | 图 1.4 | 论文章节关系导航 | ➕新增 | **Claude Design** | 待绘 |
| 2 | 图 2.1 | 卷积运算原理 | fig2-1 | **Claude Design** | 示意稿在 |
| 2 | 图 2.2 | WS/OS/RS 三种数据流对比 | ➕新增 | **TikZ / Claude Design** | 待绘 |
| 2 | 图 2.3 | 流水线+握手时序 | ➕新增 | **WaveDrom** | 待绘 |
| 2 | 图 2.4 | NUMA 拓扑示意 | ➕新增 | **Claude Design** | 待绘 |
| 2 | 图 2.5 | 存储能耗金字塔 | ➕新增 | **matplotlib** | 待绘 |
| 2 | 图 2.6 | INT8 量化推理流程 | fig2-2 | **Claude Design** | 示意稿在 |
| 3 | 图 3.1 | 设计目标雷达图 | ➕新增 | **matplotlib** | 待绘 |
| 3 | 图 3.2 | 加速器系统总体架构 | fig3-1 | **Claude Design** | 示意稿在 |
| 3 | 图 3.3 | 核内数据通路 | fig3-2 | **Claude Design** | 示意稿在 |
| 3 | 图 3.4 | DMA 子系统结构 | fig3-3 | **Claude Design** | 示意稿在 |
| 3 | 图 3.5 | 分层调度示意 | ➕新增 | **Claude Design** | 待绘 |
| 3 | 图 3.6 | 三层数据复用叠加 | ➕新增 | **Claude Design** | 待绘 |
| 4 | 图 4.1 | 行缓存模块 | fig4-1 | **Claude Design** | 示意稿在 |
| 4 | 图 4.2 | MAC 阵列 | fig4-2 | **Claude Design** | 示意稿在 |
| 4 | 图 4.3 | 部分和累加 | fig4-3 | **Claude Design** | 示意稿在 |
| 4 | 图 4.4 | SDP 后处理 | fig4-4 | **Claude Design** | 示意稿在 |
| 4 | 图 4.5 | 权重缓存 | fig4-5 | **Claude Design** | 示意稿在 |
| 4 | 图 4.6 | cfg_regs+6 层 FSM | fig4-6 | **Claude Design** | 示意稿在 |
| 4 | 图 4.7 | DMA 子系统详图 | fig4-7 | **Claude Design** | 示意稿在 |
| 4 | 图 4.8 | 多核扩展层 | fig4-8 | **Claude Design** | 示意稿在 |
| 4 | 图 4.9 | Y 维折叠 | fig4-9 | **Claude Design** | 示意稿在 |
| 4 | 图 4.10 | DSP 跨列复用 | fig4-10 | **Claude Design** | 示意稿在 |
| 4 | 图 4.11 | 空间到深度 (S2D) | ➕新增 | **Claude Design** | 待绘 |
| 5 | 图 5.0 | 验证流程图 | ➕新增 | **Claude Design** | 待绘 |
| 5 | 图 5.1 | 端到端时延与帧率（演进图） | fig5-4 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.2 | Cin × Ky-fold 利用率 | fig5-7 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.3 | K 扫频 | fig5-8 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.4 | stride 扫频 | fig5-9 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.5 | H 步长分离 | fig5-10 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.6 | N–W 扫频加速比 | fig5-11 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.7 | ResNet11 分层瓶颈 | fig5-12 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.8 | 多网络 ASIC 折算性能 | fig5-13 | **matplotlib** | ✅已渲染 |
| 5 | 图 5.9 | vs 代表性加速器 | fig5-16 | **matplotlib** | ✅已渲染 |

**合计：37 图**（用户决定结论章不配图，覆盖率 = 37 / 38 ≈ 0.97 张/节，超过 0.8 目标）

另附 **4 段算法伪代码（独立编号 算法 X.Y）**，不计入图数但贡献小节信息密度：

| 编号 | 标题 | 章节 | 形式 |
|---|---|---|---|
| 算法 3.1 | 核内 6 级嵌套自循环 FSM 主循环 | §3.4 调度方案 | listings 伪代码（cs → yout → tile → cins → round → pos） |
| 算法 4.1 | Y 维折叠编译器变换 | §4.8 Ky-fold | listings 伪代码（含 cin_fake 推导） |
| 算法 4.2 | 空间到深度（S2D）重排 | §4.9 S2D | listings 伪代码（4 相位拼接到 Cin） |
| 算法 5.1 | 字节级比对回归脚本主流程 | §5.2.2 验证方法 | listings 伪代码（参考 vs RTL 输出） |

---

## 2. 工具分派逻辑

| 工具 | 适用类型 | 张数 | 理由 |
|---|---|---|---|
| **matplotlib**（含 `_style.py`） | 数据扫频曲线、性能对比、雷达图、能耗金字塔 | 11 | 已有 `figures-rendered/_style.py` 统一字体(Microsoft YaHei + CM math) + 配色 + 300 DPI PNG/SVG 输出；9 张已就绪，2 张待写脚本（图 2.5 / 3.1） |
| **WaveDrom** | 时序波形 | 1（+1 可选） | JSON 描述驱动，浏览器版可在 [wavedrom.com/editor.html](https://wavedrom.com/editor.html) 直出 SVG；图 2.3 握手时序必做，图 4.12 跨核 SRAM 直送时序可选 |
| **Claude Design** | 架构框图、概念示意、流程图、思维导图 | 25 | 已有示意稿（13）含完整 IEEE 风 image prompt + ASCII 草图；新增（12）按相同 prompt 模板编写后丢给 [claude.ai/design](https://claude.ai/design) |
| **LaTeX `algorithm2e`** → PNG | 伪代码片段 | 4 段 | `\SetAlgoVlined`（竖线缩进）+ `\KwIn/\KwOut/\For/\If`（关键字粗体）+ `\tcp*[r]{...}`（右对齐斜体注释），编译 PDF 再 `pdftoppm` 转 300 DPI PNG；视觉风格匹配正文图，避免裸代码块"视觉降级" |

---

## 3. 各章节配图密度核查

| 章 | 实质小节数 | 配图数 | 算法 | 密度 | 达标 |
|---|---|---|---|---|---|
| 1 绪论 | 4 (1.1/1.2/1.3/1.4) | 4 图 + 1 表 | — | 1.00 | ✅ |
| 2 理论基础 | 6 (2.2/2.3/2.4/2.5/2.6/2.7) | 6 图 | — | 1.00 | ✅ |
| 3 总体方案 | 4 (3.2/3.3/3.4/3.5) | 6 图 | 算法 3.1 | 1.50 | ✅ |
| 4 加速器实现 | 11 (4.2–4.12) | 11 图 | 算法 4.1 / 4.2 | 1.00 | ✅ |
| 5 验证与性能 | 5 (5.2/5.3/5.4/5.5/5.6) | 10 图 + 5 表 | 算法 5.1 | 2.00 | ✅ |
| 6 结论与展望 | 3 (6.1/6.2/6.3) | 0 图 | — | — | — 排除（用户决定） |
| **加权平均（前 5 章）** | **30** | **37** | 4 段 | **1.23** | ✅ |

---

## 4. 新增内容设计要点（速览）

### 4.1 Claude Design 类（10 张）— 概念示意 / 架构图 / 流程图

| 图号 | 关键元素 | 复杂度 |
|---|---|---|
| 图 1.1 CNN 端侧应用场景全景 | 4 象限：监控/工业/消费/汽车，中央 FLUX_CNN 芯片图标 | 低 |
| 图 1.2 加速器演进谱系树 | 时间轴 2014→2024，分支：ASIC/FPGA/Hybrid；本工作高亮 | 中 |
| 图 1.3 五大创新点思维导图 | 中心节点 FLUX_CNN，5 辐射节点：分层调度/任意形状/Ky-fold+S2D/W切片+SMC/DSP跨列 | 低 |
| 图 1.4 论文章节关系导航 | 6 章圆角矩形，箭头表依赖；与§3-§5 串联突出 | 低 |
| 图 2.2 WS/OS/RS 数据流对比 | 三 panel：每 panel 一种数据流，标静止张量与流动张量 | 中 |
| 图 2.4 NUMA 拓扑示意 | 2-4 个 NUMA 节点，跨节点访问延迟标注 | 低 |
| 图 3.5 分层调度示意 | 上：核间 Task Descriptor 链表；下：核内 6 级 FSM 嵌套 (cs/yout/tile/cins/round/pos) | 高 |
| 图 3.6 三层数据复用叠加 | 阵列 16×16 PE 网格，三色箭头：权重静止(红)/滑窗(蓝)/广播(绿) | 中 |
| 图 4.11 S2D 重排示意 | 输入像素 4 相位 → 4×Cin 通道扩张，stride=2 case | 中 |
| 图 5.0 验证流程图 | PyTorch FP32 → 编译器 → TB → 仿真 vs 参考比对 → PASS/FAIL | 中 |

### 4.2 matplotlib 类（2 张新增 + 9 张已就绪）— 数据图

| 图号 | 数据来源 | 图类型 | 状态 |
|---|---|---|---|
| 图 2.5 存储能耗金字塔 | Horowitz ISSCC'14 [16]：寄存器 1pJ / L1 5pJ / SRAM 10pJ / DRAM 640pJ | 横柱图 + log 坐标 | ➕ 待写 |
| 图 3.1 设计目标雷达 | 5 维度：吞吐 / 能效 / 灵活性 / 资源 / 可演进；端侧 vs 数据中心两曲线 | 雷达图 | ➕ 待写 |
| 图 5.1–5.9 性能图（9 张） | figures-rendered/ 现有脚本 | 扫频/散点/演进 | ✅ 已就绪 |

### 4.3 WaveDrom 类（1 张必做 + 1 张可选）— 时序波形

| 图号 | 关键信号 | 复杂度 |
|---|---|---|
| 图 2.3 流水线 + valid-ready 握手时序 | clk / valid / ready / data / pipe[0..4]，呈现 5 级流水 + 反压堵塞场景 | 低 |
| 图 4.12（可选）跨核 SRAM 直送时序 | core[i].ofm_wr / push_to_core[i+1] / core[i+1].ifb_wr / ready，呈现"上一层写 OFM 同时下一层 IFB 已可用"的零拷贝接力 | 中 |

WaveDrom JSON 范例（先写后由用户在 [wavedrom.com/editor.html](https://wavedrom.com/editor.html) 渲染）：

```json
{ "signal": [
  { "name": "clk",    "wave": "p........" },
  { "name": "valid",  "wave": "01.0.1..." },
  { "name": "ready",  "wave": "1...0.1.." },
  { "name": "data",   "wave": "x3.x.4..x", "data": ["D0", "D1"] }
]}
```

### 4.4 代码图（算法伪代码）— 4 段，**LaTeX 渲染为图像**

参考 DepFiN [JSSC'23] Listing 1 与 Fig. 4 的呈现风格：竖线缩进、关键字加粗、注释右对齐斜体。**两种组合策略**：

- **独立模式**（DepFiN Listing 1 风格）：单独成一张图，自带 caption "算法 X.Y"。适合算法本身就是核心贡献的场景。
- **嵌入模式**（DepFiN Fig. 4 风格）：把伪代码作为示意图的局部元素，伪代码与图形示意并列在同一张组合图中，配色与连接线呼应。适合"算法需配合示意才能讲清"的场景。

| 编号 | 标题 | 章节 | 行数 | 组合策略 |
|---|---|---|---|---|
| 算法 3.1 | 核内 6 级嵌套自循环 FSM 主循环 | §3.4 | ~25 | **嵌入图 3.5** 分层调度示意（伪代码 + 6 级嵌套树状图并列） |
| 算法 4.1 | Y 维折叠（Ky-fold）编译器变换 | §4.8 | ~20 | **嵌入图 4.9** Ky-fold 原理示意（伪代码 + 相位重排可视化并列） |
| 算法 4.2 | 空间到深度（S2D）重排 | §4.9 | ~15 | **嵌入图 4.11** S2D 重排示意（伪代码 + 4 相位拼接示意并列） |
| 算法 5.1 | 字节级比对回归脚本主流程 | §5.2.2 | ~15 | **嵌入图 5.0** 验证流程图（伪代码 + 数据流箭头并列） |

**LaTeX 渲染模板**（每段算法一个独立 `.tex`，编译为 `standalone` PDF 后转 PNG）：

```latex
\documentclass[border=2pt,varwidth]{standalone}
\usepackage[T1]{fontenc}
\usepackage{xeCJK}                          % 中文支持（XeLaTeX 编译）
\usepackage[linesnumbered,vlined,boxed]{algorithm2e}
\SetAlFnt{\small\sffamily}
\SetCommentSty{\small\itshape}
\DontPrintSemicolon

\begin{document}
\begin{algorithm}[H]
\KwIn{cfg\_regs $\theta$（卷积参数 + 地址步进量）}
\KwOut{OFM 写完成信号}
\For{$cs \in [0, n_{cs})$}{
  \For{$y_{out} \in [0, H_{out})$}{
    \For{$tile \in [0, n_{tile})$}{
      \For{$cins \in [0, n_{cins})$}{
        \For{$round \in [0, n_{round})$}{
          \For{$pos \in [0, K^2)$}{
            发射 1 拍 MAC\tcp*[r]{16×16 PE 全并行}
          }
        }
      }
    }
  }
}
\caption{核内 6 级嵌套自循环 FSM 主循环}
\end{algorithm}
\end{document}
```

**编译命令**（一次性脚本）：

```bash
cd C:/_Project/FLUX_CNN/paper/figures-rendered
xelatex algo3-1-fsm.tex
pdftoppm -png -r 300 algo3-1-fsm.pdf algo3-1-fsm
# 输出 algo3-1-fsm-1.png，重命名为 algo3-1-fsm.png
```

**风格约定**：
- 关键字（`\For`、`\If`、`\KwIn`、`\KwOut`、`\Return`）自动粗体
- 注释用 `\tcp*[r]{...}` 右对齐斜体（参考 DepFiN Listing 1）
- 中文用 xeCJK + 思源黑体；英文 / 数学用 Computer Modern
- 单段 ≤ 30 行；超出则拆子流程或简化抽象
- 行号开启（`linesnumbered` 选项），便于正文行号引用

---

## 5. paper.md 与 Word 的同步差异（必须处理）

| 类型 | 当前 paper.md | Word 期望 | 处理 |
|---|---|---|---|
| 图 1.1 | 不存在 | "图1.1我顶我顶我顶"占位 | **加图 1.1 占位 + 引用** |
| 第 5 章 §5.5.1 表 5.4 | 表格 | 图 5.1（演进图 fig5-4） | **改为图 5.1 引用 PNG** |
| 第 5 章 §5.5.5 表 5.7 | 表格 | 图 5.2（fig5-7） | **改为图 5.2 引用** |
| 第 5 章 §5.5.5 表 5.8 | 表格 | 图 5.3（fig5-8） | **改为图 5.3 引用** |
| 第 5 章 §5.5.5 表 5.9 | 表格 | 图 5.4（fig5-9） | **改为图 5.4 引用** |
| 第 5 章 §5.5.5 表 5.10 | 表格 | 图 5.5（fig5-10） | **改为图 5.5 引用** |
| 第 5 章 §5.5.5 表 5.11 | 表格 | 图 5.6（fig5-11） | **改为图 5.6 引用** |
| 第 5 章 §5.5.5 表 5.12 | 表格 | 图 5.7（fig5-12） | **改为图 5.7 引用** |
| 第 5 章 §5.5.6 表 5.13 | 表格 | 图 5.8（fig5-13） | **改为图 5.8 引用** |
| 第 5 章 §5.6 表 5.16 | 表格 | 图 5.9（fig5-16） | **改为图 5.9 引用** |
| 第 5 章 §5.4.3 表 5.3 | 表格（数值待填） | 同 paper.md（占位） | 不变 |
| 第 5 章 §5.5.4 调度策略 | 文字 | Word 已扩 4 段 | **整段同步** |
| 附录 A / 修改记录 / 致谢 | 不存在 | 大工模板 3 章节 | **新增空骨架** |

> Word 中"表 5.3 = 1-DDR vs 4-DDR 流水占空比"（paper.md 中编号为表 5.5）属表号微调，不影响整体结构。Word 中"表 5.5 = CPU vs ASIC 加速比"（paper.md 中为表 5.15）同理。Word 的章号体系是平铺式（图/表全章顺编），paper.md 是层级式（图 X.Y 含章号）—— **建议保留 paper.md 层级编号**，导出 docx 时由 Word 字段重新生成。

---

## 6. 执行顺序建议

1. **立即可做**（已完成 / 已有数据）：
   - ✅ figures-rendered/ 的 9 张 PNG 嵌入 paper.md 第 5 章对应位置
   - ✅ §1.1 末尾插入图 1.1 占位 + caption
   - ✅ 附录 A / 修改记录 / 致谢骨架

2. **本周完成**（写设计稿 + 脚本）：
   - 给图 1.1–1.4 / 图 2.2 / 图 2.4 / 图 3.5 / 图 3.6 / 图 4.11 / 图 5.0 写完整 image prompt（按 fig3-1-system-arch.md 模板）—— 10 张 Claude Design 设计稿；**注意图 3.5 / 4.9 / 4.11 / 5.0 的 prompt 中预留伪代码嵌入位置**（左半 / 右半 / 上半的版面留白）
   - 给图 2.5 存储能耗金字塔 / 图 3.1 设计目标雷达 写 matplotlib 脚本（参考 _style.py）—— 2 个脚本
   - 给图 2.3 流水线+握手 / 图 4.12 跨核 SRAM 直送 写 WaveDrom JSON —— 2 个 json
   - 给算法 3.1 / 4.1 / 4.2 / 5.1 写 LaTeX `.tex` 源码（algorithm2e + xeCJK），编译为 PNG —— 4 段算法

3. **下周完成**（生成图像）：
   - 把 Claude Design 类的 23 张图（13 旧 + 10 新）依次扔到 [claude.ai/design](https://claude.ai/design) 出图，导出 SVG / PNG
   - 跑 matplotlib 脚本生成图 2.5 / 3.1
   - 在 [wavedrom.com/editor.html](https://wavedrom.com/editor.html) 渲染图 2.3 / 4.12 为 SVG
   - 跑 `render_algorithms.sh` 用 XeLaTeX 编译算法 3.1 / 4.1 / 4.2 / 5.1 → PNG
   - **组合阶段**：把 4 段算法 PNG 与对应的 Claude Design 示意图（图 3.5 / 4.9 / 4.11 / 5.0）在 Inkscape 或 Affinity Designer 中拼合成最终组合图（DepFiN Fig. 4 风格）

4. **最终入稿**：
   - 在 Word 文档中替换占位图
   - paper.md 的图引用按 v2 编号刷新

---

## 7. 风险与备选

| 风险 | 影响 | 备选 |
|---|---|---|
| Claude Design 出图风格不一致 | 视觉不统一 | 用同一个 prompt 模板（已有，见 fig3-1 等）+ 同一会话内连续出图 |
| matplotlib 雷达图数据点过少（5 维不够丰满） | 信息密度低 | 图 3.1 可降级为分组柱图（吞吐/能效/灵活性 vs 端侧 / 数据中心两类对比） |
| WaveDrom 浏览器版手输 JSON 易错 | 出图慢 | 用 [wavedrom-cli](https://github.com/wavedrom/cli) 本地 npm 安装，命令行 batch 渲染 |
| Word 占位与 paper.md 编号脱钩 | 答辩时图号乱 | 最终交付以 Word 为准；paper.md 仅做内容索引 |

---

## 8. 交付清单

### 已完成
- [x] figure-plan.md（本文档）
- [x] paper.md §1.1 加图 1.1 占位
- [x] paper.md §5.5.1 / §5.5.5.1-5 / §5.5.6 / §5.6 插入 9 张数据图引用
- [x] paper.md 文末附录 A / 修改记录 / 致谢骨架

### 待产出（共 16 项）

**Claude Design 设计稿（10 张）** — 写 `figures/figX-Y-NAME.md`，按 `fig3-1-system-arch.md` 模板包含 ASCII 草图 + 中英文 image prompt：
- [ ] figures/fig1-1-cnn-edge-scenarios.md
- [ ] figures/fig1-2-accelerator-genealogy.md
- [ ] figures/fig1-3-contributions-mindmap.md
- [ ] figures/fig1-4-thesis-navigation.md
- [ ] figures/fig2-3-dataflow-comparison.md
- [ ] figures/fig2-4-numa-topology.md
- [ ] figures/fig3-5-hierarchical-scheduling.md
- [ ] figures/fig3-6-reuse-overlay.md
- [ ] figures/fig4-11-s2d-transform.md
- [ ] figures/fig5-0-verification-flow.md

**matplotlib 脚本（2 个）** — 参考 `figures-rendered/_style.py` 字体 / 配色 / 输出格式：
- [ ] figures-rendered/fig2-5-memory-energy.py
- [ ] figures-rendered/fig3-1-design-radar.py

**WaveDrom JSON（2 个）** — 写 `figures/figX-Y-NAME.json`，浏览器版渲染：
- [ ] figures/fig2-3-pipeline-handshake.json
- [ ] figures/fig4-12-cross-core-push.json（可选）

**算法伪代码（4 段，LaTeX 渲染为 PNG）** — 写 `.tex` 源码 + 编译脚本，与对应示意图组合呈现：
- [ ] figures-rendered/algo3-1-fsm.tex（嵌入图 3.5 分层调度示意）
- [ ] figures-rendered/algo4-1-kyfold.tex（嵌入图 4.9 Ky-fold 示意）
- [ ] figures-rendered/algo4-2-s2d.tex（嵌入图 4.11 S2D 重排示意）
- [ ] figures-rendered/algo5-1-bytewise-cmp.tex（嵌入图 5.0 验证流程）
- [ ] figures-rendered/render_algorithms.sh（一键编译 + pdftoppm 批量转 PNG）

确认方案后我可以批量产出上述 19 个文件 / 段落（10 + 2 + 2 + 4 + 1 编译脚本）。
