# 综合

> 综合不是当前的主要关注点。这份文档给流程和参数，不维护具体时序/面积数字。

## 工具链与目标器件

- 工具：Xilinx Vivado
- 目标器件：`xcku060-ffva1156-2-e`（UltraScale+ FPGA）
- 综合模式：OOC（Out-of-Context），不含 I/O pad
- 时钟约束：10 ns（100 MHz）

## 运行

```bash
cd Syn
vivado -mode tcl -source run_syn.tcl -log syn_run.log -nojournal
```

报告输出到 `Syn/reports/`。

## 综合时映射

`sram_model` 映射到 RAMB36：IFB（128-bit × 8192）和 OFB（128-bit × 8192）各占若干 BRAM；WB（2048-bit × 8192）BRAM 占用最大。

`std_rf` 映射到 distributed RAM（PE 内的 32×8 WRF 体量小，综合工具可能选 LUTRAM）。

## 加新 RTL 后

每加一个 .sv 都要把它加进 `Syn/run_syn.tcl` 的 `read_verilog` 列表，否则综合会因找不到模块定义而失败。同步修改 `sim/<tb>/run.tcl` 的编译列表。
