# 综合

> **注意**：综合不是本项目当前的主要关注点。这份文档只给出流程和粗略预期，不保证综合数字最新。时序/面积/功耗的精确评估留给未来工程化阶段。

## 工具链与目标器件

- **工具**：Xilinx Vivado
- **目标器件**：`xcku060-ffva1156-2-e` (UltraScale+ FPGA)
- **综合模式**：OOC (Out-of-Context) —— 不含 I/O pad
- **时钟约束**：10 ns（100 MHz）

## 运行

```bash
cd Syn
vivado -mode tcl -source run_syn.tcl -log syn_run.log -nojournal
```

报告输出到 `Syn/reports/`。

## 预期关键路径

1. **MAC 列加法树**（16 个 16-bit 乘积 → 32-bit）—— 当前全组合 4 级加法；在 100 MHz 不成为瓶颈，拉更高频需要插 1-2 级流水
2. **握手穿透链** —— `parf_accum.psum_in_ready → mac_array.can_advance → act_ready/wgt_ready → line_buffer/wgt_buffer`；全组合路径经过 2-3 个 AND 门
3. **地址加法** —— line_buffer 的 `ptr_kx_base + iss_pos × stride`（20-bit + 5-bit）；stride ≤ 2 时是 shift/wire，组合逻辑短
4. **SRAM 读** —— 硬件固定 1 拍

`sram_model` 综合时映射到 RAMB36，占据大部分 RAMB 资源。

## 频率提升参考

| 阶段 | 预期时钟 | 改动 |
|------|---------|------|
| 当前 | 100 MHz | baseline（握手架构） |
| + 加法树单级流水 | ~200-250 MHz | `mac_col` 在 `prod_out` 后面加 1 级加法树流水寄存器，相应扩 mac_array 的 pipe valid 追踪到 3 级 |
| + SDP 流水化 | ~400+ MHz | SDP 内部拆 shift / relu / clip，调整 ofb_writer 写地址延迟 |

更细节见 `docs/roadmap.md` Phase 3。
