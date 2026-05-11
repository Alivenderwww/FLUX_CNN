# 子智能体名册 / Agent Roster

本文件是论文撰写流水线中所有子智能体的统一名册。每个子智能体的提示词都会引用此文件，让子智能体知道有哪些"兄弟智能体"可调用。

主智能体把这个文件的内容（或路径）传给每个被启动的子智能体。

---

## 分工原则（重要）

**灵魂层 vs 格式层**：
- **灵魂层**（Phase 0–7 所有 Writer / Reviewer）—— 关心"内容对不对、结构合不合理、贡献清不清晰、语言流不流畅"；**不死磕格式细节**
- **格式层**（Phase 8 paper-thesis-formalizer）—— 关心"中英文有无空格、字母正斜体、章节编号风格、引用句式、引导句模板、双语图表标题、残留英文术语"

**前期 agents 不要管格式**——子任务管辖范围越窄越容易做好。如果发现自己在纠结"narrative A 该不该译为'技术路线 A'"，停下来直接保留 narrative，等 formalizer 统一处理。

**评审纪律**：
- Phase 0–7 的 reviewer-writing **不据格式细节 FAIL**（残留英文术语、中英空格、字母正斜体等不影响 PASS）
- Phase 8 的 reviewer-writing 才激活"中文工科规范"维度，对格式做严格审查

## 名册总览

| 类别 | 名称 | 阶段 | 一句话职责 | 你可以请它做什么 |
|------|------|------|-----------|----------------|
| Writer | `paper-literature-scout` | Phase 0 | 文献调研，调用 Elicit API + check-citations 核验 | 补 X 类文献 / 补某具体论文细节 / 验证某引用是否真实 / 提取某文献的具体数据 |
| Writer | `paper-project-analyst` | Phase 1 | 项目工作梳理，从代码/文档提炼贡献清单 | 查 FLUX_CNN 中 X 模块是否实现 / 某个性能数据有无可信来源 / 某 contributions 条目的代码定位 |
| Writer | `paper-outliner` | Phase 2 | 论文章节大纲制定 | 查某章定位 / 某 contribution 应该放在哪章 |
| Writer | `paper-section-writer` | Phase 3 | 每节 100-200 字简介 | 给某小节产出/重写摘要 |
| Writer | `paper-paragraph-skeleton` | Phase 4 | 每段一句主题句 + 类型标签 | 给某节产出/重排段落骨架 |
| Writer | `paper-paragraph-expander` | Phase 5 | 把主题句扩为完整段落（按章串行） | 扩写某章 / 重写某段 |
| Writer | `paper-figure-designer` | Phase 6 | 图表示意稿（ASCII + 可选代码） | 设计某张图/表 |
| Writer | `paper-polisher` | Phase 7 | 语言润色 + 章节衔接 + 整体定位（不动事实、不动格式细节） | 对某章做局部语言/段间桥接 / 句式多样化 |
| Writer | `paper-thesis-formalizer` | Phase 8 | 中文工科本科论文格式规范化（章号阿拉伯化 / 中英无空格 / 字母正斜体 / 引用句式 / 引导句模板 / 双语图表 / 主要符号表 / 残留英文术语清除） | 对全文做格式终审与机械化整改 |
| Reviewer | `paper-reviewer-tech` | 评审（多 Phase） | 技术正确性、文献真实性（含 check-citations） | 让它对某产出做技术核验 / 验证某条引用是否真实 |
| Reviewer | `paper-reviewer-writing` | 评审（多 Phase） | 写作流畅、术语一致、章节衔接 | 让它对某产出做写作审查 |
| Reviewer | `paper-reviewer-novelty` | 评审（多 Phase） | 新颖性、与文献定位、假想审稿人挑战 | 让它对某产出做新颖性审查 / 评估某 claim 强度 |

---

## 分支请求协议（Side-quest Protocol）

任何子智能体在执行任务时，如果发现**需要其他智能体提供更多信息**才能完成或更好完成任务，可在返回给主Agent 的文本中追加一段"分支请求"，请主Agent 启动目标智能体提供信息。

### 何时使用分支请求

合理的场景：
- ✅ Writer 写正文时，发现某文献细节模糊 → 请 `paper-literature-scout` 补查
- ✅ Writer 写实现细节时，对某 RTL 模块状态不确定 → 请 `paper-project-analyst` 复核
- ✅ Writer 在两个写法间犹豫，想要快速反馈 → 请 `paper-reviewer-{tech|writing|novelty}` 给一个 mini-review
- ✅ Reviewer 看到一处事实可疑但无法直接核验 → 请 `paper-project-analyst` 帮忙 Grep 验证

不合理的场景：
- ❌ 把本属于自己职责范围的活转嫁给别人（"我懒得查代码，请 project-analyst 查"——你就是 Writer，你应该自己查）
- ❌ 因为下一阶段还没跑就请下一阶段的智能体（违反 Phase 顺序）
- ❌ 链式分支（A 请 B，B 又请 C）——**分支不可嵌套**，目标智能体只能完成单次任务，不能再分支

### 分支请求格式（追加到任务输出末尾）

完成任务后，**先输出标准的"任务完成"格式**（让主Agent 知道你完成了），再追加分支请求。

```markdown
{标准的任务完成输出，如 "Phase 5 第 5 章扩写完成 / 段数 12 / [TBD]=2 / [CHECK]=4"}

## 分支请求（可选）

### 请求 1
- **目标智能体**: paper-literature-scout
- **需要的信息**: 补查 Tangram (MICRO'19) 论文的 Table 4 数据，本章 §5.2.p3 需要引用其单层 latency 数字
- **期望产出形式**: 在 literature.md 中追加该 Table 4 的关键数字行
- **恢复时我需要的输入**: literature.md 中新追加位置的 5-10 行摘录（路径或行号）
- **优先级**: 中（可不补，但能让 §5.2.p3 从 [CHECK] 变为可填）

### 请求 2
- **目标智能体**: paper-reviewer-tech
- **需要的信息**: 对本章 §5.4.p1-p3 三段做 mini-review，重点核验 RTL 引用是否对齐
- **期望产出形式**: 一份简短的 PASS/FAIL + 问题清单
- **恢复时我需要的输入**: 报告路径
- **优先级**: 低（不补也能继续，但会留 [CHECK]）
```

### 主Agent 收到分支请求后的处理

1. 主Agent 用 `Grep "^## 分支请求"` 检测请求
2. 对每个请求，**优先级 = 高/中** 的执行：
   - 启动目标智能体（一次性子任务，不计入主流水线 Phase 顺序）
   - 在 prompt 中说明：这是分支任务（side-quest），来自 `{源智能体}` 的请求，目标是 `{需要的信息}`
   - 等目标智能体完成后，记录产出路径
3. 然后 **resume 源智能体**，在 prompt 中提供分支产出路径，让源智能体修订自己的产出
4. 优先级 = 低 的可以**忽略**（在日志中记录"已忽略低优先级请求"），避免无限分支

### 单 Phase 分支预算

每个 Phase 内最多允许 **3 次分支调用**。超过则主Agent 拒绝并日志记录。
（这个预算包含修正循环中触发的分支。修正循环本身不算分支。）

### 分支不可嵌套

被分支调用启动的智能体（即"side-quest agent"）**禁止再发起分支请求**。如果它本身需要更多信息，在产出中标 `[CHECK: ...]` 让源智能体处理。

主智能体在启动 side-quest agent 时，会在 prompt 中明确告知 "你是被分支调用，不可再发分支请求"。

### 日志记录

主Agent 把每次分支调用记入 `main-log.md`：

```
- 260429 1620 Phase 5 第 5 章 触发分支：paper-paragraph-expander → paper-literature-scout（补 Tangram Table 4 数据）
- 260429 1635 分支完成：literature.md 已更新
- 260429 1635 Resume paper-paragraph-expander，传入分支产出路径
- 260429 1640 Phase 5 第 5 章 修订完成
```

---

## 名册引用方式（在每个子智能体提示词中）

每个子智能体提示词的固定位置（"工作流程"之前或末尾）都有以下区块：

```markdown
## 兄弟智能体名册与分支请求

完整名册见 `paper/agents/_AGENT_ROSTER.md`。

如果你在执行任务时发现需要其他智能体提供更多信息，按"分支请求协议"在任务输出末尾追加请求，主Agent 会调度对应智能体。

**禁止**：
- 把本属于你职责的工作转嫁出去
- 链式分支（你被分支调用时不可再发分支请求）
- 同一 Phase 累计 >3 次分支
```
