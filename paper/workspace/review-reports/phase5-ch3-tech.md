# 技术评审报告 Phase 5 §3

## 第 1 次评审

### 判定

FAIL

### 通过项（抽样验证记录）

- **立项硬约束**：全章扫描"FPGA"、"可重构"、"FPGA 加速器"等词 0 命中。§3.1 定位为"面向端侧流式计算场景的卷积加速器 FLUX_CNN"，§3.2 提"端侧典型卷积层"、"挂接到 Zynq、定制 SoC 或其他 ARM/RISC-V 控制核为主的端侧平台"，全程未把 FLUX_CNN 框定为 FPGA 加速器。✅
- **关键参数与 CLAUDE.md 一致**：NUM_COL=NUM_PE=16 / DATA=8 / PSUM=32 / WRF=32 / ARF=32 / PARF=32 / IFB=8192 / WB=1024 / OFB=2048 全部命中且数值匹配。✅
- **5 模块清单**：line_buffer / mac_array / parf_accum / ofb_writer / wgt_buffer 完整，职能描述与 CLAUDE.md 一致。✅
- **DMA 子系统清单**：idma_ctrl / wdma_ctrl / odma_ctrl / mm2s_arb / axi_dm / axi_m_mux / axi_lite_csr 完整；额外列出 rdma_ctrl 与 dfe 二者已用 Glob 验证 `RTL/DMA/rdma_ctrl.sv`、`RTL/DMA/dfe.sv` 真实存在；rdma_ctrl 头注释「Residual / Bias DMA cmd controller」与 §3.3 「rdma_ctrl 负责拉取偏置与残差数据」一致。✅
- **核心命题（软硬协同分层调度）**：§3.4 三层调度（核内 cfg_regs 50 余寄存器 + 6 层嵌套 FSM cs→yout→tile→cins→round→pos / 核外宏观指令流 + Task Descriptor / 跨多核 W 切片）展开清晰。"AXI-Lite 写次数从 50 余次/层降到 4 次/层"与 `RTL/cfg_regs.sv` 头注释及 contributions 一致。✅
- **数据流声明**：WS（权重静止 + WB→WRF 双层）+ 激活值滑窗复用（行缓存环形缓冲）+ 输出通道广播（16 列 PE = 16 输出通道）三要素齐备，章末小结与正文 §3.5 自洽。✅
- **章节边界（不展开模块内部）**：未出现 PE 内部 counter / state machine 状态迁移图 / SRAM 端口冲突解析等 RTL 细节；未引入实测周期数、Fmax、利用率数字；提"N=1/2/4 已综合通过"是状态描述非性能数据，可接受。✅
- **禁词扫描**：narrative / prior art / wrapper / baseline / 谱系 / 哲学 / 兑现 / 公认 全部 0 命中（line 20 "关键词" 与 line 40+ 命中均在 §1 范围之外）。✅
- **元话语扫描**：「下文先在 / 本节铺垫 / 为后续 / 已论证 / 留到」等元话语在 §3 范围（line 206-270）内 0 命中。✅
- **抽样代码核验**：6 层循环命名 → `docs/slicing/hw-loops.md` 命中；rdma_ctrl 职责 → `RTL/DMA/rdma_ctrl.sv` 头注释一致；4 个启动寄存器（CTRL/DESC_LIST_BASE/DESC_COUNT/DMA_MODE）→ `RTL/cfg_regs.sv` 头注释明确列出。✅

### 失败项

| # | 严重度 | 位置 | 问题 | 期望修改方向 |
|---|--------|------|------|------------|
| 1 | 中等 | §3.4 倒数第 2 段（line 240）"另一个写口接 dfe，由描述符流写入逐层配置" | 与 RTL 不符。`RTL/cfg_regs.sv` 头注释明确写 "写口分两条: 1. csr_w_* (AXI-Lite, host 写) 2. **seq_w_* (sequencer CFG_WRITE descriptor)**"。dfe 只负责把描述符链表拉进 desc_fifo，**实际写 cfg_regs 的是 sequencer**（sequencer 从 fifo pop CFG_WRITE 描述符后再写 cfg_regs 的 seq_w_* 端口）。同时 §3.3 段 8 已把 dfe 职责定为"拉取描述符链表"，§3.4 这句又把 dfe 扩展到"写 cfg_regs"，存在内部不一致。 | 改为「另一个写口接 sequencer，由 sequencer 消费 dfe 拉取的 CFG_WRITE 描述符流写入逐层配置」或「另一个写口接描述符流（dfe 拉取 → sequencer 消化 → cfg_regs 写口），由描述符流写入逐层配置」。 |

### 修复后建议

修复上述 1 处后即可通过。其余整章（核心命题、参数、模块清单、章节边界、立项约束、数据流、禁词、元话语）全部强健，无需大改。

---

## 第 1 轮重测（2026-05-07）

### 判定
PASS

### 1 问题修复情况
- 中等 #1 cfg_regs 双写口：修

### 通过原因
line 240 当前表述「另一个写口接 sequencer，由 sequencer 消费 dfe 拉取的 CFG_WRITE 描述符流写入逐层配置（dfe 仅负责把描述符链表从 DDR 拉进描述符 FIFO，CFG_WRITE 类型的描述符再由 sequencer 从 FIFO 中取出并写入 cfg_regs 的描述符写口）」，与 `RTL/cfg_regs.sv` 头注释 `seq_w_* (sequencer CFG_WRITE descriptor)` 完全对齐：(1) 写口主体由错误的"dfe"改为正确的"sequencer"；(2) 显式补充 dfe→FIFO→sequencer→cfg_regs 的数据通路，消除了原文与 §3.3 段 8（dfe 职责限定为"拉取描述符链表"）的内部不一致；(3) 描述符 FIFO 的中间环节也补齐，技术链条完整可追溯。无新增编造，无新增可疑数字。
