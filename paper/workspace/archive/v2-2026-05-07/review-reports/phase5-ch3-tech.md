# 技术评审报告 Phase 5 §3

## 第 1 次评审

### 判定：PASS

---

### 评审范围

paper.md §3（行 183-245），25 段，6 小节 §3.1—§3.6。仅评灵魂层（设计决策、关键参数、差异化论证、模块清单、claim 强度、回归性）。

---

### 段落骨架对齐核验

| 小节 | 骨架要求 | 实际段数 | 行号 | 状态 |
|------|---------|---------|------|------|
| §3.1 引言 | 2 | 2 | 187, 189 | ✓ |
| §3.2 设计目标与约束 | 5 | 5 | 193, 195, 197, 199, 201 | ✓ |
| §3.3 总体架构方案 | 5 | 5 | 205, 207, 209, 211, 213 | ✓ |
| §3.4 数据流选择论证 | 7 | 7 | 217, 219, 221, 223, 225, 227, 229 | ✓ |
| §3.5 协同设计原则 | 5 | 5 | 233, 235, 237, 239, 241 | ✓ |
| §3.6 本章小结 | 1 | 1 | 245 | ✓ |
| **合计** | **25** | **25** | — | ✓ |

[CHECK] = 3（§3.4.3 三处 Alwani / Kang / Liu 措辞最终版）+ [TBD] = 1（§3.5 是否单独成节）与 Writer 自报一致。

---

### 关键参数与 CLAUDE.md 一致性核验

| 项 | CLAUDE.md | §3 实际 | 状态 |
|---|---|---|---|
| 阵列 | NUM_COL = NUM_PE = 16 | "固定 16 × 16 INT8 MAC 阵列"（§3.6）/ "16 × 16 这一量级"（§3.2.4）/ "OS + 列广播... 16 列广播"（§3.4.2） | ✓ |
| 寄存器堆 | WRF = ARF = PARF = 32 | §3.5 第二段明确列出 WRF=ARF=PARF=32 | ✓ |
| 数据位宽 | DATA = 8 / PSUM = 32 | §3.5 第二段明确列出 DATA=8 / PSUM=32；§3.2.4 INT8 量化、32-bit 累加 | ✓ |
| 缓冲容量 | IFB=8192 / WB=1024 / OFB=2048 | §3.5 第二段明确列出 | ✓ |
| 总线 | BUS_DATA_W=128 / CSR_DATA_W=32 | §3.5 第二段明确列出 | ✓ |
| 地址映射 | DDR `0x0000_0000—0x7FFF_FFFF` + Core[i] IFB `0x8000_0000 + i × 0x1000_0000` | §3.3 第四段完整一致复述 | ✓ |
| 平台 | XC7K325T-FFG900-2 | §3.2 第三段，资源数 LUT 203,800 / FF 407,600 / BRAM36 445 / DSP48E1 840（带 STATUS.md §1 来源注） | ✓ |
| 接口 | 1 路 AXI4 主 + 1 路 AXI-Lite 从 | §3.3 第一段 / §3.6 复述 | ✓ |

无一处与 CLAUDE.md 漂移。

---

### 5 模块 + DMA 子系统清单核验

| 类别 | CLAUDE.md 清单 | §3 实际 | 状态 |
|------|---------------|---------|------|
| 核流水 5 模块 | line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer | §3.3 第二段完整列出 5 个，并明确共享 cfg_regs | ✓ |
| DMA 控制器 | idma_ctrl / wdma_ctrl / odma_ctrl | §3.3 第三段完整列出 | ✓ |
| 仲裁/聚合 | mm2s_arb / axi_m_mux / axi_lite_csr | §3.3 第三段完整列出 | ✓ |
| Vivado IP | axi_dm | §3.3 第三段提及"Vivado axi_dm IP" | ✓ |
| MM2S 共享 | idma + wdma 共享 MM2S，odma 占 S2MM | §3.3 第三段措辞完全一致 | ✓ |
| 多核 fanout | multicore_top + axi_2to3 / axi_4to5 / axi_lite_1to4 | §3.3 第四段完整列出 | ✓ |

模块清单与项目结构 1:1 对应，无遗漏、无虚构。

---

### §3.4 三处差异化论证强度核验

#### (1) vs Alwani Fused-layer (MICRO'16) — §3.4.4
- 对比维度：缓冲粒度（"层数级" vs "行级"）
- prior art 数据有据：VGGNet-E 前 5 层 77 MB → 3.6 MB（来自 literature.md Alwani 条目）
- 本工作主张可证伪：strip_rows × W_IN 量级几行缓冲，单层硬件即可处理任意 H × W
- 措辞克制：未声称更优，仅"形成代差"
- **判定：扎实**，能经受审稿挑战。

#### (2) vs Kang AoCStream (FCCM'23 / Sensors'23) — §3.4.5
- 对比维度：片上存储与输入分辨率耦合度
- prior art 数据有据：所有特征图驻留片上 SRAM、line buffer 与图宽线性、每层专用 dataflow block
- 本工作主张可证伪：row-credit 反压解耦，VGA 480×640 单图 4.9 MB 仅约 10 KB ring
- 给出具体公式 `ofb_strip_rows_max = (OFB-1) // row_words`，公式可在 toolchain/hw_files.py 验证
- **判定：扎实**，差异化维度具体且可量化。

#### (3) vs Liu Full-Stack Streaming (TNNLS'21) — §3.4.6
- 对比维度：并行维度（层间 vs 层内 W 切片）
- prior art 数据有据：Intel Arria 10、conv/deconv/residual 独立模块、跨层 pipelined
- 本工作主张有数据支撑：ResNet-11 N=1/2/4 = 596K/450K/354K wall cycles
- 措辞克制：明确"两种路线在端侧资源约束下的取舍各有侧重"，避免 over-claim
- **判定：扎实**，对比公平。

三处差异化对比维度互不重叠（缓冲粒度 / BRAM-H 解耦 / 并行维度），覆盖审稿人最可能挑战的"为什么不是 X"问题。

---

### claim 强度核验（避免 over-claim）

抽样：
- §3.2.1 "不在抽象层面追求'通用最优'" — 主动收敛 ✓
- §3.4.3 末句 "把所有形状适配交给编译器" — 哲学论断，不是性能 claim ✓
- §3.4 末段（行 229）"这一选择并非声称比可重构互连或跨层融合'更优'，而是在端侧 FPGA 单器件、单 DDR 通道、有限 BRAM 容量的约束语境下做出的工程化最简组合" — **明确声明非比较性 claim** ✓
- §3.6 "本章诚实陈述：固定 16 × 16 阵列在 Cin < 16 与 stride ≥ 2 场景下确实存在 PE 利用率塌陷问题" — 主动暴露弱点 ✓
- 全文未出现 "novel" / "first" / "best" / "optimal" 等 over-claim 词 ✓

---

### 回归性核验（是否引入骨架没有的新引用 / 新数据）

抽样数字回溯：
- "VGGNet-E 前 5 层... 77 MB → 3.6 MB" → literature.md Alwani 条目（已确认）✓
- "VGA 480 × 640 的单图 4.9 MB 仅用约 10 KB ring" → contributions.md C1.3（已确认）✓
- "ResNet-11 N = 1/2/4 上 596K / 450K / 354K wall cycles" → contributions.md C4.4 + STATUS §2.8（已确认）✓
- 资源数字（LUT 203,800 / FF 407,600 / BRAM36 445 / DSP48E1 840）→ STATUS.md §1（已确认）✓

引用清单（Alwani / Kang / Liu / MAERI / Eyeriss-v2 / Tangram）全部出现于 literature.md，无新增。
未引入骨架未列的新事实或新数字。

---

### 抽样验证记录（≥3 处）

1. §3.3 第三段 "idma_ctrl 与 wdma_ctrl 共享 axi_dm 的 MM2S 通道（由 mm2s_arb 串行仲裁），odma_ctrl 独占 S2MM 通道" → 与 CLAUDE.md 项目总览第 3 段措辞 1:1 一致 ✓
2. §3.5 第二段参数清单 "NUM_COL = NUM_PE = 16、WRF = ARF = PARF = 32、DATA = 8、PSUM = 32、IFB = 8192、WB = 1024、OFB = 2048、BUS_DATA_W = 128、CSR_DATA_W = 32" → 与 CLAUDE.md 配置约定逐项匹配 ✓
3. §3.4.5 公式 `ofb_strip_rows_max = (OFB-1) // row_words` → 项目 toolchain 中确实是 cfg 派生 single source 的标准公式（无虚构）✓
4. §3.5 第三段 "Ky-fold 与 S2D 联合触发的核心决策函数 `s2d_eff()`" → 与 CLAUDE.md "PE 利用率优化"段中 --ky-fold / --s2d 描述一致 ✓
5. §3.3 第四段 multicore_top + axi_2to3 / axi_4to5 / axi_lite_1to4 → 与 STATUS.md §2 多核拓扑组件清单一致 ✓

---

### 总结

- 段落骨架：25 段全对齐 ✓
- 关键参数：8 项与 CLAUDE.md 100% 一致 ✓
- 模块清单：5 核流水模块 + 7 个 DMA/接口组件 + 多核 fanout 全部 1:1 ✓
- 三差异化论证：维度互不重叠，措辞克制，prior art 数据有据 ✓
- claim 强度：主动避免 over-claim，主动暴露弱点 ✓
- 回归性：未引入新事实 ✓
- [CHECK]=3 / [TBD]=1 标记位置合理 ✓

**判定 PASS。**
