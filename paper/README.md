# FLUX_CNN 论文撰写多智能体协同方案

## 文件布局

```
paper/
├── 主智能体提示词-论文版.md    主 Agent 系统指令（对话触发时引用）
├── agents/
│   └── _AGENT_ROSTER.md        子智能体名册 + 分支请求协议
├── .env                         Elicit API key（gitignored，需手动填）
├── .env.example                 .env 模板
└── workspace/                   运行时由主 Agent 创建

.claude/agents/                  11 个 paper-* subagent 定义（Claude Code 自动加载）
```

## 流水线总览

```
Phase 0  文献调研          paper-literature-scout    评审：tech, novelty
Phase 1  项目工作梳理       paper-project-analyst    评审：tech, novelty
Phase 2  论文大纲           paper-outliner           评审：tech, writing, novelty
Phase 3  小节摘要           paper-section-writer     评审：writing
Phase 4  段落骨架           paper-paragraph-skeleton 评审：writing
Phase 5  正文扩充（按章串行） paper-paragraph-expander 评审：tech, writing
Phase 6  图表设计           paper-figure-designer    评审：tech
Phase 7  整体润色           paper-polisher           评审：writing, novelty
```

## 启动方式

对话触发。新会话时对主 Agent 说：

> 按 `paper/主智能体提示词-论文版.md` 开始论文撰写流水线。

主 Agent 会读取该文件、初始化 `paper/workspace/`、询问目标会议与文献调研方式，然后进入 Phase 0。

## 核心机制

1. **不确定标记**：`[TBD]` 待用户决定 / `[CHECK]` 待 FLUX_CNN 实现状态确认；禁止编造
2. **文献真实性**：`paper-literature-scout` 用 **Elicit API** + `check-citations` skill 双重校验
3. **三评审专家**：`tech` / `writing` / `novelty`，按 Phase 性质裁剪启用
4. **修正循环 ≤3 轮**：第 3 轮仍 FAIL 标 ⚠️ 妥协通过
5. **跨阶段分支调用（Side-quest）**：子智能体可请求主 Agent 启动其他智能体补信息；单 Phase ≤3 次、不可嵌套；详见 [`agents/_AGENT_ROSTER.md`](agents/_AGENT_ROSTER.md)

## 准备工作（首次使用）

```bash
cp paper/.env.example paper/.env
# 编辑 paper/.env，填入 ELICIT_API_KEY=elk_live_...（需 Elicit Pro 订阅）
```

`paper/` 整个目录已在 `.gitignore`，密钥不会进 git。
