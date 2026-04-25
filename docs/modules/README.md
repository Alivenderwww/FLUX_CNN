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
- [psum_reshape](psum_reshape.md) — Kx-fold cout 归约
- [ofb_writer](ofb_writer.md) — SDP + OFB 写
  - [sdp](sdp.md) — int32→int8 量化（mult/shift/relu/clip）

## DMA / AXI

- [dfe](dfe.md) — descriptor fetch engine
- [idma](idma.md) — DDR → IFB streaming DMA
- [wdma](wdma.md) — DDR → WB DMA（128→2048 打包）
- [odma](odma.md) — OFB → DDR streaming DMA
- [axi_m_mux](axi_m_mux.md) — 4 → 1 AXI master 聚合

## Storage primitives

- [sram_model](sram_model.md) — 行为级 SRAM
- [std_rf](std_rf.md) — 标准寄存器堆（PE 内 WRF）

## 阅读建议

控制流：从 `core_top` 入手，看 `sequencer` 如何按 descriptor 分配 start pulse；时钟内每拍数据流向看 `mac_array → parf_accum → psum_reshape → ofb_writer` 这条主线。

Streaming 行级反压：`idma ↔ line_buffer` 用 `rows_consumed / rows_available`，`ofb_writer ↔ odma` 用 `row_done_pulse / rows_drained`。

Fold 涉及的模块：Ky-fold 在 `gen_isa_test.py` 编译期完成，HW 不感知；Kx-fold 影响 `line_buffer`（iss_pos 扩展）、`wgt_buffer`（x_cnt 扩展）、`parf_accum`（每列 wr_addr 偏移 + we mask）、`psum_reshape`（drain 时 cout 归约）。
