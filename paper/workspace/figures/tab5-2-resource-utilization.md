# 表 5.2 不同核数配置下的 FPGA 资源占用
# Table 5.2 FPGA Resource Utilization across Multi-core Configurations

## 在论文中的角色

- 首次引入位置：§5.4.1（资源占用）
- 被哪些段引用：§5.4.1 论述（line 532 前后），§5.4.1 后续段落讨论 BRAM 主导约束 / DSP 推断问题。
- 论证作用：将单核与多核（*N*=1/2/4）SMC 配置在 XC7K325T-FFG900-2 上综合得到的 LUT / FF / BRAM36 / DSP48E1 资源占用列出，并对照器件容量给出 *N*=4 占比，支撑后续讨论：(1) 单核 BRAM 是主要瓶颈；(2) DSP 实际用量远低于理论 256，原因在于 mac_pe 描述风格阻碍了 Vivado 自动推断 DSP；(3) *N*=4 配置下通过缩减 Shortcut Bank 容量缓解 BRAM 上限。
- 立项硬约束补注：FLUX_CNN 最终目标为 ASIC，本表 FPGA 数据用于功能验证平台的资源刻画，不作为最终性能指标。

## 列定义

| 列名 | 含义 | 单位 | 数据来源 |
|------|------|------|----------|
| 资源 | 资源类别（LUT / FF / BRAM36 / DSP48E1） | - | Vivado 综合 |
| *N*=1 | 单核配置实测 | 个 | Syn/reports_single/utilization_synth.rpt |
| *N*=2 | 双核 SMC 配置实测 | 个 | Syn/reports_smc_n2/utilization_synth.rpt |
| *N*=4 | 四核 SMC 配置实测 | 个 | Syn/reports_smc/utilization_synth.rpt |
| 器件容量 | XC7K325T-FFG900-2 提供的总量 | 个 | Xilinx 7-series 器件手册 |
| *N*=4 占比 | *N*=4 用量 / 器件容量 | % | 由前两列折算 |

## 行定义

| 行 | 数据来源 | 置信度 |
|----|----------|--------|
| LUT | Vivado utilization_synth.rpt | *N*=1/2 已实测；*N*=4 待综合复核 |
| FF | Vivado utilization_synth.rpt | *N*=1/2 已实测；*N*=4 待综合复核 |
| BRAM36 | Vivado utilization_synth.rpt | *N*=1/2 已实测；*N*=4 待综合复核 |
| DSP48E1 | Vivado utilization_synth.rpt | *N*=1/2 已实测；*N*=4 待综合复核 |

## 表初稿（Markdown 表）

| 资源 | *N*=1 | *N*=2 | *N*=4 | 器件容量 | *N*=4 占比 |
|---|---|---|---|---|---|
| LUT | 36,942 | 74,386 | [CHECK: N=4 综合 LUT 实测，待查 Syn/reports_smc/utilization_synth.rpt] | 203,800 | [CHECK] |
| FF | 13,167 | 26,927 | [CHECK: N=4 综合 FF 实测] | 407,600 | [CHECK] |
| BRAM36 | 128 + 1 RAMB18 | 256 | [CHECK: N=4 综合 BRAM 实测，shortcut_bank 已缩 8192→2048] | 445 | [CHECK] |
| DSP48E1 | 82 | 164 | [CHECK: N=4 综合 DSP 实测] | 840 | [CHECK] |

注：
- *N*=1 BRAM36 列 "128 + 1 RAMB18" 表示 128 个完整 BRAM36 加 1 个 RAMB18（半块），与综合报告一致。
- *N*=4 配置下 Shortcut Bank 已从 8192 word 缩减至 2048 word（commit `5fe16b2`）以缓解 BRAM 上限。
- DSP 实际用量远低于阵列规模 256：原因在于 mac_pe 中乘法器描述风格阻碍 Vivado 自动推断（见 §5.4.1 正文）。

## 不确定项

- [CHECK] *N*=4 配置下 LUT / FF / BRAM36 / DSP48E1 实测值，待 Syn/reports_smc/utilization_synth.rpt 综合完成后回填。
- [CHECK] *N*=4 各资源对器件容量的占比，待四个实测值确定后折算。
- [TBD] 是否要附加一行 "Shortcut Bank 缩容前 *N*=4 BRAM 估算值" 以体现缩容动作的必要性？当前正文已表述，未必入表。

## image 生成提示词

无（表格直接以 markdown 形式嵌入论文）。
