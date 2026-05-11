# 实验 12: VD100 OOC 综合首次 PASS (VE2302)

**日期**: 2026-05-08
**目标板**: ALINX VD100 (Versal AI Edge VE2302, xcve2302-sfva784-1LP-e-s)
**状态**: ✅ multicore_top_vd100 N=3 OOC 综合 PASS

## 关键数据

| 指标 | VD100 (Versal VE2302) | K325T 主线 (xc7k325t) | Δ |
|---|---:|---:|---:|
| LUT | **128,375** (85%) | 99,061 (49%) | +29K (跨架构差异) |
| DSP | **144** (31%) | 832 (99%) | -688 (没用 SIMD) |
| RAMB36 | 81 (54%, 2.9 Mb) | 288 (65%, 10.4 Mb) | -207 (URAM 替代) |
| **URAM** | **64** (67%) ⭐ | 0 | +64 (Versal 独有) |
| **Fmax** | **182.3 MHz** | 143.8 MHz | **+27%** ⭐ |

## URAM 自动映射 (Versal 独有特性)

VE2302 比 K325T 多了 96 个 UltraRAM block (每个 288 KB), 总 27 Mb. Vivado 2023.2 综合时
**自动把大 SRAM 模块映射到 URAM**:

| RTL 模块 | 大小 | K325T 映射 | VD100 映射 |
|---|---|---|---|
| IFB (line_buffer) | 8K × 128 bit = 1 MB | RAMB36 × ~28 | **URAM × ~3** |
| OFB (ofb_writer) | 2K × 128 bit = 256 KB | RAMB36 × ~8 | URAM × 1 |
| WB (wgt_buffer) | 1K × 256 bit = 256 KB | RAMB36 × ~8 | URAM × 1 |

3 ConvCore × ~5 URAM = 15-20 URAM 主要部分 + 64 URAM (含其他小映射) 都有富余。

**RTL 不动**: `sram_model.sv` 没 URAM 特定 attribute, Vivado 综合根据资源约束自动选最优映射 (URAM 比 BRAM 便宜在大尺寸下)。

## Fmax 提升解读

K325T 7-series (28nm) → VE2302 7nm (3 代工艺):
- 关键路径 LUT 延迟降 30-40%
- DSP cascade 延迟降
- 即使没改 RTL, Fmax 自然提升 +27%

## 资源紧 (LUT 85%) — 加 PS BD 后

OOC 综合不含 PS Versal CIPS / NoC / DDRMC / clk_wizard, 这些 Vitis BD 后会加进来:
- versal_cips: ~0 PL LUT (Hard IP, 不占 PL 资源)
- axi_noc + DDRMC: 很少 (~5K LUT, NoC 是 Hard, axi 接口胶水部分占 PL)
- clk_wizard + util_ds_buf: ~0.5K LUT
- proc_sys_reset: <1K LUT

**估计完整 BD 后 LUT ≈ 135K, 占用 90%**。够装但布线吃力。

## 工具链

```
Vivado 2023.2 (D:\Xilinx\Vivado\2023.2)
  ↓
xcve2302-sfva784-1LP-e-s (Versal AI Edge VE2302)
  ↓
Versal device support 已装 (data/parts/xilinx/versal/ 完整)
  ↓
axi_datamover IP retarget for VE2302 (read_ip + 自动 retarget)
```

## RTL 改动 (vd100-demo 分支独有)

- `RTL/Versal/multicore_top_vd100.sv` (345 行, vlog 0 errors)
- `Syn/run_syn_vd100.tcl` (OOC 综合脚本)
- `Syn/check_vd100.tcl` (smoke test)
- `Syn/cnn_vd100_ooc.xdc` (auto-generated 100 MHz constraint)
- `Syn/reports_vd100/utilization_synth.rpt` + `timing_synth.rpt`

main 分支 RTL **完全不动** (core_top.sv / mac_array.sv / 所有内核模块共享)。

## 下一步

| 步骤 | 工作量 |
|---|---|
| ALINX 02_pl_rw_ddr BD 拼接 tcl 改造 (加 multicore_top_vd100 + 3 S_AXI + GP + IRQ) | 1 周 |
| Vivado 板级 P&R + bitstream | 0.5 周 |
| Petalinux 工程定制 (ALINX 提供模板) + UIO 设备树 | 1 周 |
| PS A72 host runtime C 编译部署 (host/vd100_ps/resnet11_server.c) | 0.5 周 |
| 上位机 Windows Python client 测试 (host/vd100_pc/resnet11_client.py 已写) | 0.5 天 |
| 板上 bring-up + ResNet11 demo 跑通 | 1 周 |
| 视频录制 + 文档 | 0.5 周 |
| **总计** | **4-5 周** |

## 复现

```bash
# 1. 装 Vivado 2023.2 + Versal 设备支持
# 2. 跑 OOC 综合 (10-15 分钟)
cd C:\_Project\FLUX_CNN
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source Syn\run_syn_vd100.tcl -nojournal -log Syn\syn_vd100.log
# 3. 看报告
cat Syn/reports_vd100/utilization_synth.rpt
cat Syn/reports_vd100/timing_synth.rpt
```

## Sources

- ALINX VD100 用户手册: https://vd100-20232-v101.readthedocs.io/zh-cn/latest/
- ALINX 02_pl_rw_ddr 工程模板: `C:\_Project\Bishe\VD100\VD100_9_16\demo\course_s1\02_pl_rw_ddr`
- Versal AI Edge selection guide: AMD product brief
