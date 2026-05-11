# Tab.7: Performance Comparison with Prior Art

## 在 paper.md 中的引用位置
- **首次引入**：§7.6 Comparison with Prior Art, line 436
  - 原文：`Table 7 is the performance comparison: for each of TPU [Jouppi@ISCA'17], Eyeriss [Chen@ISSCC'16], Gemmini [Genc@DAC'21], Snowflake [Gokhale@ISCAS'17], Angel-Eye [Guo@TCAD'18], Aydonat [Aydonat@FPGA'17], Lu Winograd [Lu@FCCM'17], Liu Full-Stack [TNNLS'21], VTA [Moreau@ASPLOS'19], and fpgaConvNet [Venieris@TNNLS'19], we list dataset, device, Fmax, GOPS, MAC%, and end-to-end latency in a unified column layout.`
- **被引用次数**：1 次显式

## 论证作用
**论文 head-to-head 比较的主表**。10 个 prior art baseline 沿 Dataset / Device / Fmax / GOPS / MAC% / Latency 6 列对照 FLUX_CNN。这是 reviewer 最先扫的表——口径对齐至关重要。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| Method | 方法名 + 引用 | literature.md |
| Year/Venue | 发表会议/年份 | literature.md |
| Device | 硬件平台 | 各 baseline 原文 |
| Array | PE 阵列规模 | 同上 |
| Fmax (MHz) | 时钟频率 | 同上 |
| Peak GOPS | 峰值算力 | 同上 |
| Network MAC% | 整网 MAC 利用率 | 同上 |
| Workload | 测试网络 | 同上 |
| End-to-end Latency | 端到端时延 | 同上 |

## 表初稿

| Method | Year/Venue | Device | Array | Fmax (MHz) | Peak GOPS | Network MAC% | Workload | E2E Latency |
|--------|-----------|--------|-------|-----------|-----------|---------------|----------|-------------|
| **FLUX_CNN (this work)** | 2026 | XC7K325T (FPGA) | 16×16 INT8 | **68.4** measured / 100 target | **51.2 @ target / 35 @ Fmax** | **86.6%** | ResNet-18-style 11-layer | **5.95 ms @ target / 8.69 ms @ Fmax** |
| TPU v1 [Jouppi@ISCA'17] | ISCA'17 | 28nm ASIC | 256×256 INT8 | 700 | 92,000 (92 TOPS) | [CHECK] | datacenter mix | — |
| Eyeriss [Chen@ISSCC'16, JSSC'17] | ISSCC'16 / JSSC'17 | 65nm ASIC | 14×12 (168 PE) INT16 | 200 | [CHECK: 33.6 GOPS] | [CHECK] | AlexNet | [CHECK] |
| Gemmini [Genc@DAC'21] | DAC'21 | TSMC 16nm / Intel 22FFL ASIC | 16×16 INT8 (default) | [CHECK] | [CHECK] | [CHECK] | [CHECK] | [CHECK: 106.1 GOPS/W reported] |
| Snowflake [Gokhale@ISCAS'17] | ISCAS'17 | Zynq XC7Z045 | line-buffer vector | 100 | **128** (peak) | **91%** (avg) | AlexNet / GoogLeNet / ResNet-50 | [CHECK] |
| Angel-Eye [Guo@TCAD'18] | TCAD'18 | Zynq XC7Z045 / XC7Z020 | (custom INT8 ISA) | [CHECK] | [CHECK: ~6× contemporary FPGAs] | [CHECK] | VGG | [CHECK] |
| Aydonat DLA [Aydonat@FPGA'17] | FPGA'17 | Arria 10 | 1-D Winograd streaming | [CHECK] | **1382 GFLOPS** (FP16) | [CHECK] | AlexNet | [CHECK] |
| Lu Winograd [Lu@FCCM'17] | FCCM'17 | ZCU102 | (Winograd) | [CHECK] | **854.6 GOPS** | [CHECK] | AlexNet | [CHECK] |
| Liu Full-Stack [Liu@TNNLS'21] | TNNLS'21 | Arria 10 GX1150 (~1.15M ALMs) | (streaming + residual fusion) | [CHECK] | **>1300 GOPS** | **97%** | ResNet / DenseNet (multiple) | [CHECK] |
| VTA [Moreau@ASPLOS'19] | ASPLOS'19 / IEEE Micro'19 | Zynq XC7Z020/045 | 256 PE INT8 (default) | 100 | [CHECK] | [CHECK] | (TVM-targeted) | [CHECK] |
| fpgaConvNet [Venieris@TNNLS'19] | TNNLS'19 | Zynq | (HLS, per-layer modules) | [CHECK] | [CHECK] | [CHECK] | (varied) | [CHECK: 2.94× perf-density gain reported] |

## 数据来源

| 数据点 | 来源 | 状态 |
|--------|------|------|
| FLUX_CNN 全部数字 | contributions.md §4 / STATUS.md | ✅ measured |
| TPU v1 92 TOPS | literature.md (Jouppi@ISCA'17) | ✅ |
| Eyeriss array 14×12 / 65nm | literature.md (Chen@ISSCC'16) | ✅ array; ✅ device; [CHECK: 整网 MAC% / latency 数字 — contributions.md §8.1 #10] |
| Gemmini 16×16 default + 106.1 GOPS/W | literature.md (Genc@DAC'21) | ✅ array; [CHECK: Fmax/GOPS/latency] |
| Snowflake 128 GOPS / 91% MAC% | literature.md (Gokhale@ISCAS'17) | ✅ peak; ✅ MAC%; [CHECK: workload-specific latency] |
| Angel-Eye 6× contemporary | literature.md (Guo@TCAD'18) | ✅ relative; [CHECK: absolute Fmax/GOPS] |
| Aydonat 1382 GFLOPS | contributions.md §5.1 表 / literature.md | ✅ |
| Lu 854.6 GOPS | 同上 | ✅ |
| Liu 97% MAC% / >1.3 TOP/s | contributions.md §5.1 / paper.md §7.6 line 440 | ✅ |
| VTA 256 PE @ 100 MHz | contributions.md §5.1 / literature.md | ✅ |
| fpgaConvNet 2.94× perf-density | literature.md (Venieris@TNNLS'19) | ✅ relative; [CHECK: absolute] |

## 强制说明（建议作为 caption / 脚注）

> ¹ FLUX_CNN row reports two operating points: **(a) 100 MHz design target** (used in cycle-accurate simulation) and **(b) 68.4 MHz post-synthesis Fmax** measured on XC7K325T-FFG900-2 OOC mode (commit b158cab). Throughput and latency at both points are listed throughout this paper.
>
> ² **MAC% definitions differ across baselines**. Snowflake's 91% is "average computational efficiency" (denominator includes inter-layer transitions but excludes off-chip data transfer); Liu's 97% is end-to-end including residual paths; FLUX_CNN's 86.6% includes IDMA/ODMA channel switching, descriptor fetch, and final ODMA drain. Where definitions are not directly comparable, we mark the cell with [†].
>
> ³ Devices span ASIC (TPU/Eyeriss/Gemmini), Zynq (Snowflake/Angel-Eye/VTA/fpgaConvNet), 7-series Kintex (FLUX_CNN), and Arria 10 (Aydonat/Lu/Liu). **Same-device-class peers to FLUX_CNN are Snowflake and Angel-Eye** (XC7Z045 ↔ XC7K325T, both mid-range 7-series ~28nm).

## 与正文的一致性检查

- [x] §7.6 line 436 列出的 10 个 baseline + FLUX_CNN 共 11 行 — 表全
- [x] §7.6 line 436 列定义 "dataset, device, Fmax, GOPS, MAC%, end-to-end latency" — 表列对齐
- [x] §7.6 line 440 "86.6% (FLUX_CNN, XC7K325T) versus 97% (Liu, Arria 10 GX1150)" — 两行 MAC% 对齐
- [x] §7.6 line 442 "86.6% versus Snowflake's 91%" + denominator difference 警告 — 表脚注 ² 显式说明
- [x] §7.6 line 444 "Angel-Eye + Gemmini same-7-series + INT8 territory" — 同器件 peer 标记

## 不确定项

- [CHECK: contributions.md §8.1 #10] 各 baseline 整网 MAC% (Snowflake 91% / Liu 97% 已知；Angel-Eye / Aydonat / Lu / VTA 未知) — reviewer 阶段查原文
- [CHECK: contributions.md §8.1 #11] 同器件 (XC7K325T 量级) 其他工作 Fmax 对照 — reviewer 阶段查原文
- [CHECK: contributions.md §8.1 #13] 同器件 Angel-Eye / DPU / VTA 资源占用对照 — reviewer 阶段查原文
- [CHECK: paper.md line 442] Snowflake 91% 计算效率口径精确文字定义 — 读 ISCAS'17 §V Evaluation
- [CHECK: paper.md line 440] Arria 10 GX1150 ALM/LUT 数字 — reviewer 阶段确认
- [TBD: 是否单列 "Workload normalized GOPS"] 让数字更可比；但每 baseline 跑不同网络，难直接归一
- [TBD: 是否拆 ASIC vs FPGA 两子表] 1 张表 11 行混平台读起来勉强，拆开会让 narrative 更清晰但占版面更多

## 备注

**reviewer 阶段会逐行查证原文**。建议 polisher 阶段先用 [CHECK] 占位，camera-ready 前由 literature-scout 补全 → 在投稿前再做一次单独的"Tab.7 数据交叉验证"会议。
