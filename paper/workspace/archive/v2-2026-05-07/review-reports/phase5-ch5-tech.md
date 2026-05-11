# 技术评审报告 Phase 5 §5

## 第 1 次评审

### 判定：FAIL

### 核验抽样汇总（先列已通过的，再列失败项）

通过的关键事实抽样：
- §5.4 单核表（LUT 36,942 / FF 13,167 / BRAM36 128+1 / DSP 82 / Fmax 68.4 MHz @ WNS=-4.618 ns）— 与 STATUS.md L38-42 完全一致 ✅
- §5.4 N=2 wrapper 数字（74,386 LUT / 26,927 FF / 256 BRAM）— STATUS L74-78 ✅
- §5.5 ResNet-11 cycles（596,088 / 450,469 / 354,555）与 fps（313/444/564）— STATUS L215-217 ✅；S2D 单 Patch 5.05× / 整网 1.87× — STATUS L220-222 ✅
- §5.5 SMC+NUMA N=4 220,824 cy / 453 fps / 11/11 layer / 13/13 regression — STATUS L1136-1138 ✅
- §5.6 mac_pipe% 36.2%→63.5% / DDR busy 84.7% — STATUS L406-408 ✅
- §5.6 各 baseline 数字（Snowflake 91% / Aydonat 1382 GFLOPS / Lu 854.6 / Liu 1.3 TOPS / 97% MAC%）— literature.md L502/504 全有，且 paper 中均带 [CHECK] 标记 ✅
- §5.6 Liu 论证立场（"同思路不同规模" / "不声称持平或超越" / 显式承认差距）— 与 §1.2.3 / §3.4 / §4.5 立场一致，未 over-claim ✅
- §5.7 wslice5 1,920 word / resnet11_n4 24,360 word FAIL — STATUS L852-853 ✅
- §5.7 Pooling / Depthwise / sparsity / SDP 流水线 / use_dsp / Mode C / Stage barrier / push 链 P2 / YOLOv3-tiny 未实测 — 全覆盖 STATUS §1/§2.6/§4 中提到的缺口 ✅
- 文献引用真实性（Liu Shuanglong@TNNLS'21 DOI 10.1109/TNNLS.2021.3055240 / Aydonat / Snowflake@ISCAS'17 / Alwani / Kang / Eyeriss / MAERI / Gemmini / TPU v1）— 全在 literature.md 中且第 5 次启动已修正过 DOI/作者 ✅
- §5.4 段 5 BRAM 分解（57+32+32+7=128 全部 RAMB36 + 1 RAMB18）— 与 params.py / CLAUDE.md 参数一致 ✅
- §5.7 段 6 "DDR 流量 -50%" — STATUS L987 ✅；"LUT 节省 50%" — STATUS L975 "LUT -17K" 推算 ≈46% ✅

### 失败项明细

| # | 严重度 | 位置 | 问题 | 期望修改方向 |
|---|--------|------|------|------------|
| 1 | **严重** | §5.6 段 5 "Intel Arria 10 GX1150（约 1518K LE / 65.7 Mb 片上 SRAM / 1518 DSP 块）" | 未标 [CHECK]，数字疑似编造/typo：Arria 10 GX1150 真实规格是 **1150K LE**（不是 1518K — 1518 是 DSP 数被复制成 LE 数）；M20K 片上存储约 **53Mb**（不是 65.7Mb）。literature.md 全文未给出这三个具体硬件数字，Writer 自行填入但出错。这是评审清单"§5.6 Liu 论证一致性"重点核对项之一。 | 删除具体数字或全部加 [CHECK]，或改为定性描述如"约 5~7× XC7K325T 资源量级"（这后半句反而是合理的）。**绝不可保留 1518K LE / 65.7 Mb 这两个错值**。 |
| 2 | **严重** | §5.4 多核表 N=4 (SB 缩) 列 "LUT/FF/BRAM/Fmax 全部 [CHECK]" | utilization_synth.rpt（Syn/reports_smc/）已存在 N=4 SMC 实测数据：LUT 162,584 / FF 63,732 / RAMB36 288 / RAMB18 4 / DSP 320。Writer 在引言段 ✅ 引用 commit 5fe16b2 SMC 综合通过，却把数据全留空 [CHECK]。这导致 §5.4 表的关键 N=4 行无任何实数支撑——读者会怀疑 N=4 是否真综合通过。属"数据已存却谎称未知"，违反诚实陈述原则。 | 用 utilization_synth.rpt 实测数据填入 N=4 列：LUT 162,584 (79.8%) / FF 63,732 (15.6%) / BRAM36 288+4 RAMB18 (~64.7%) / DSP 320 (38.1%)；Fmax 数据若 timing_synth.rpt 中已有也应填入。 |
| 3 | **严重** | §5.3 段 3 + §5.5 段 3 "20 例 N=2 与 N=4 各 10 例" / "20/20 全 bit-exact PASS（10 case N=2 + 10 case N=4）" | 与 STATUS L240 "16 N=2/4 切片" 与 §5.8 自己写的 "16 N=2/4 切片" 直接矛盾。20 vs 16 差 4 例，未在 §5.3/§5.5 标 [CHECK]。Writer 仅在合计 51 vs 55 处标了 [CHECK]，但单项分解 20 与 16 的不一致没标。 | 统一到 STATUS 的 16（或在 §5.3/§5.5 写"20 例"处也加 [CHECK: 20 vs STATUS 16 待核] 与合计 [CHECK] 同步）；最好按 STATUS 改 16 后令全章自洽（§5.3=16 / §5.5=16 / §5.8=16 / 合计 51 = 26+16+6+3 ✅）。 |
| 4 | **中等** | §5.6 段 4 "Aydonat Intel DLA / INT16 / FP16 路线，与本工作 INT8 不直接可比" | "INT16 / FP16 路线"判断在 literature.md L271-275 中没有支撑（literature 只写 1382 GFLOPS / AlexNet 1020 img/s / 1020 img/s/W）。Aydonat 论文实际是 FP16/share-exp，但来源未直接可追溯，且关系到"不直接可比"这一较强 claim。 | 加 [CHECK: Aydonat DLA 是否 FP16] 或回 literature.md / 原文核实再写。 |
| 5 | **中等** | §5.6 段 6 表行 "fpgaConvNet [Venieris@FCCM'16] ... 早期 streaming 框架，加速比报告 [CHECK: 2.94× 加速基线]" | 2.94× 在 literature.md L243 真实存在，但口径是 "**performance density** vs SOTA FPGA-based ConvNet"，**不是简单的"加速比"**。Writer 笼统写"加速比报告"语义滑动。虽然已标 [CHECK]，但措辞会引导读者按"加速比"理解。 | 改为 "performance density 提升 2.94× [CHECK: 与本工作口径不直接可比]" 或同类精确措辞。 |
| 6 | **中等** | §5.7 段 1 "本节诚实列出本工作截至 commit 5fe16b2 已知的**全部**缺口" | "全部"语气过强 — STATUS §4 还有诸如 mesh 路线 ROI 决策、Mode B/C 部分 cfg gen corner case、tb_multicore wslice5 path bug 等未在 §5.7 完全展开。属轻微 over-claim。 | 改为"列出主要缺口"或"列出已识别的关键缺口"。 |
| 7 | **轻微** | §5.7 段 3 "落地后可消除当前 N = 4 ResNet-11 部署中 6 处 layer 间 DDR 边界" | "6 处" 这个数字未在 STATUS 直接见。ResNet-11 11 层中间 layer 边界数推算为 10，6 是把跨 stage / 同 stage 区分后的子集，几何推算来源未注。 | 加 [CHECK: 6 处 DDR 边界数计算来源] 或改为"多处"。 |
| 8 | **轻微** | §5.7 段 6 "短期是 use_dsp 综合属性 + SDP 流水线化（合计 ~30 行 RTL 改动）" | STATUS L975-976：use_dsp **1 行** + SDP **~30 行**，合计 ~31 行。"~30 行"近似但表述上掩盖了 use_dsp 的 1 行也算在内。 | 改为"合计 ~30 行（use_dsp 1 行 + SDP 流水 ~30 行）"或"约 30 行"。 |

### 关键修复优先级

P0（严重，必改）：
- #1 Arria 10 数字编造（1518K LE / 65.7 Mb 必删）
- #2 N=4 SMC 综合数据已有却空着不填
- #3 20 vs 16 case 数自相矛盾

P1（中等）：
- #4 Aydonat FP16 来源
- #5 fpgaConvNet 2.94× 口径
- #6 §5.7 "全部"措辞

P2（轻微）：
- #7 "6 处 DDR 边界" / #8 "~30 行" 细节

---

## 第 1 轮重测（2026-05-07）

### 判定
PASS

### 8 问题修复情况

| # | 严重度 | 前轮问题 | 当前 paper.md 状态 | 结论 |
|---|--------|---------|---|------|
| 1 | 严重 | §5.7 段 5 Arria 10 GX1150 编造数字 (1518K LE / 65.7 Mb / 1518 DSP) | 行 462 已删除三处具体硬件规格数字，改为定性 "资源量级约为 XC7K325T 的 5 ~ 7 倍 [CHECK: 资源比例倍数估算来源]"，HTML 注释显式记录修订 | ✅ 修 |
| 2 | 严重 | §5.4 多核表 N=4 (SB 缩) 列全部 [CHECK] 留空 | 行 397-401 已填实测：LUT 162,584 (79.8%) / FF 63,732 (15.6%) / BRAM36 288+4 RAMB18 (~64.7%) / DSP 320 (38.1%) / Fmax 100 MHz target MET (WNS=+0.196 ns)；行 403 锚定 utilization_synth.rpt + timing_synth.rpt commit 5fe16b2 | ✅ 修 |
| 3 | 严重 | 20 vs 16 case 全章不一致 | §5.2 行 357 / §5.3 行 369&373 / §5.5 行 419 / §5.8 行 480 / §5.9 行 486 全部统一为 16 例（N=2 与 N=4 各 8 例）；合计 51 = 26 + 16 + 6 + 3，全章自洽；§5.2 行 357 显式注释 "P0 #3 修正：统一全章 51 case 口径" | ✅ 修 |
| 4 | 中等 | Aydonat FP16/INT16 判断无来源 | 行 451 已加 [CHECK: Aydonat DLA 是否 FP16 / share-exp 路线] | ✅ 修 |
| 5 | 中等 | fpgaConvNet 2.94× 笼统写"加速比" | 行 448 改为 "performance density 提升 2.94× [CHECK: 与本工作 fps/cycles 口径不直接可比]" | ✅ 修 |
| 6 | 中等 | §5.8 段 1 "全部缺口" 语气过强 | 行 470 改为 "已识别的关键缺口"，显式注释 "措辞从'全部'软化到'已识别的关键'，避免 over-claim" | ✅ 修 |
| 7 | 轻微 | "6 处 DDR 边界" 数字无来源 | 行 474 改为 "多处 layer 间 DDR 边界 [CHECK: 6 处 DDR 边界几何推算来源待 STATUS 核对]"；行 482 同样写"多处"+ [CHECK] | ✅ 修 |
| 8 | 轻微 | "~30 行 RTL 改动" 掩盖 use_dsp 1 行 | 行 482 改为 "合计约 30 行 RTL 改动 = use_dsp 1 行 + SDP 流水 ~30 行"，HTML 注释 "~30 行表述细化" | ✅ 修 |

### 通过原因
- 3 严重问题（编造 Arria 10 数字 / N=4 SMC 数据空缺 / 20-vs-16 case 自相矛盾）全部按要求修复，且修复方式与前轮"期望修改方向"完全对齐
- 3 中等问题（Aydonat FP16 / fpgaConvNet 口径 / "全部"措辞）全部按要求加 [CHECK] 或精确化措辞
- 2 轻微问题（6 处 DDR 边界 / ~30 行）全部软化或分解来源
- 抽样核对其他关键事实仍正确：单核综合表（LUT 36,942 / FF 13,167 / BRAM 128+1 / DSP 82 / Fmax 68.4 MHz）与 STATUS.md L38-42 一致；ResNet-11 cycles 与 fps（596K/313 / 450K/444 / 354K/564）与 STATUS.md L215-217 一致；SMC+NUMA N=4 220,824 cy / 453 fps 与 STATUS.md §2.12 一致；mac_pipe% 36.2%→63.5% 与 memory/project_4ddr_poc_result.md 一致
- 文献引用真实性在前轮已通过校验，本轮未发现新增引用，无需重复 check-citations
- 全章诚实陈述原则贯彻：Fmax 68 MHz / DSP 推断率 82/256 / N=4 加速比偏离线性 / fps 双标 / wslice5 与 resnet11_n4 push 路径 FAIL / Pooling/Depthwise 未支持 / SMC vs W slice 双标决策 [TBD]，全部显式列出未掩饰
