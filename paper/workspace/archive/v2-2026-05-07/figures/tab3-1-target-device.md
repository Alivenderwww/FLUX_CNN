# 表 3.1 目标器件 XC7K325T 资源容量与本工作占用预算
# Table 3.1 Resource capacity of XC7K325T and resource budget of this work

## 在论文中的角色
- 首次引入：§3.2 段 "目标平台为 Xilinx Kintex-7 系列 XC7K325T-FFG900-2 单器件..." [依赖: Tab.3.1]
- 论证作用：让读者一眼看到 XC7K325T 单器件可用资源 + 本工作单核 / N=2 / N=4 占用预算 + 占比，作为后续 §5.4 详细综合表的引子。

## 列定义
| 列名 | 含义 | 数据来源 |
|------|------|---------|
| 资源 | LUT / FF / BRAM36 / DSP48E1 | XC7K325T datasheet |
| 器件容量 | XC7K325T-FFG900-2 总量 | STATUS.md §1 综合表 |
| 单核占用 | commit 5fe16b2 单核综合数 | STATUS.md §1 |
| N=2 占用 | N=2 多核 | STATUS.md §2 |
| N=4 (SMC) 占用 | N=4 SMC + NUMA 主线 | STATUS.md §2 |
| 单核占比 | 单核 / 容量 % | 计算 |

## 行定义
- LUT
- FF
- BRAM36 (+ RAMB18 单独列)
- DSP48E1
- 目标 Fmax / 实测 Fmax

## 表初稿

| 资源 | 器件容量 | 单核占用 | N=2 占用 | N=4 (SMC) 占用 | 单核占比 |
|------|---------|---------|---------|---------------|---------|
| LUT | 203,800 | 36,942 | 74,386 | 162,584 | 18.1% |
| FF | 407,600 | 13,167 | 26,927 | 63,732 | 3.2% |
| BRAM36 | 445 | 128 (+1 RAMB18) | 256 | 288 (+4 RAMB18, SB 缩 8192→2048) | 28.8% |
| DSP48E1 | 840 | 82 | 164 | 320 | 9.8% |
| 目标 Fmax | — | 100 MHz | 100 MHz | 100 MHz | — |
| 实测 Fmax | — | 68.4 MHz (WNS = -4.618 ns) | 68 MHz | **100 MHz MET (WNS = +0.196 ns)** | — |

注：N=4 (SMC) 数字来自 `Syn/reports_smc/utilization_synth.rpt` 与 `timing_synth.rpt`（commit 5fe16b2，design = `multicore_top_smc`），N=4 综合通过依赖将 Shortcut Bank 容量从 8192 缩小到 2048。N=4 SMC 主线 timing MET 100 MHz target，与单核 / N=2 的 68 MHz 不同——这是 SMC 路线分布式 SRAM 改造后 critical path 不再经过单核 SDP 后处理组合链所致。

## 与正文一致性检查
- [x] 单核 36,942 LUT / 128 BRAM36 / 82 DSP / Fmax 68.4 MHz — 与 §5.4 / STATUS.md 一致
- [x] N=4 SMC 100 MHz MET — 与 §5.4 一致
- [x] BRAM 缩 8192→2048 SB — 与 §5.4 一致

## 不确定项
无。
