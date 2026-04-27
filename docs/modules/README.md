# 模块文档

每个 RTL 模块的时序与数据流说明，对照源码 `RTL/*.sv`。

## 顶层

- [core_top](core_top.md) — 加速器顶层，连接 cfg、控制、SRAM、core 数据通路、DMA、AXI 桥

## 控制 / 配置

- [cfg_regs](cfg_regs.md) — 中心配置寄存器组
- [sequencer](sequencer.md) — descriptor → strip 控制器
- [desc_fifo](desc_fifo.md) — descriptor 同步 FIFO
- [axi_lite_csr](axi_lite_csr.md) — AXI-Lite slave → 寄存器读写桥

## Core 数据通路

- [line_buffer](line_buffer.md) — IFB → mac_array 激活供给
- [wgt_buffer](wgt_buffer.md) — WB → mac_array 权重 + bias 供给
- [mac_array](mac_array.md) — 16×16 PE 阵列 + 握手
  - [mac_col](mac_col.md) — 单列封装（16 PE + 加法树）
  - [mac_pe](mac_pe.md) — 单 PE
- [parf_accum](parf_accum.md) — PSUM 累加器外壳
  - [parf_col](parf_col.md) — 单列 PSUM 存储
- [ofb_writer](ofb_writer.md) — SDP + OFB 写
  - [sdp](sdp.md) — int32→int8 量化（mult/shift/relu/clip）

## DMA / AXI

- [dfe](dfe.md) — descriptor fetch engine
- `idma_dm` / `wdma_dm` / `odma_dm` — 通过 Xilinx AXI DataMover IP (mm2s/s2mm)
  搬运的 DMA wrapper. cfg 接口与原 idma/wdma/odma 一致, 只是 AXI4 master
  转交给 axi_dm IP. 详见 RTL 文件头注释
- `mm2s_arb` — idma_dm + wdma_dm 共享 axi_dm.MM2S 通道时的串行仲裁器
- [axi_m_mux](axi_m_mux.md) — 4 → 1 AXI master 聚合 (现 M[0]=axi_dm.mm2s,
  M[1]=axi_dm.s2mm, M[2]=unused, M[3]=DFE)

## Storage primitives

- [sram_model](sram_model.md) — 行为级 SRAM
- [std_rf](std_rf.md) — 标准寄存器堆（PE 内 WRF）

## 阅读建议

控制流：从 `core_top` 入手，看 `sequencer` 如何按 descriptor 分配 start pulse；时钟内每拍数据流向看 `mac_array → parf_accum → ofb_writer` 这条主线。

Streaming 行级反压：`idma ↔ line_buffer` 用 `rows_consumed / rows_available`，`ofb_writer ↔ odma` 用 `row_done_pulse / rows_drained`。

Fold / S2D 全在 `gen_isa_test.py` 编译期完成，HW 完全无感（Cout 小的情况硬件不复用，对应 PE 列空转）。`parf_accum` 内部把 PARF 拆成 16 个独立 `parf_col`（每列单独 SRAM），但所有列共享 wr_addr/we。
