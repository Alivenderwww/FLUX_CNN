# VD100 BD Full Flow 实测结果

## Step 1: Vivado 综合 + 实现 (full_flow.tcl + fix_xdc_relaunch.tcl)

### Routed Utilization (post-impl, design_1_wrapper top)

| Resource | Used | Available | Util% |
|---|---:|---:|---:|
| **LUT** | **117,384** | 150,272 | **78.1%** |
| - Logic LUTs | 108,846 | – | – |
| - LUTRAMs | 7,966 | 75,136 | 10.6% |
| - SRLs | 572 | – | – |
| FF | 41,571 | 300,544 | 13.8% |
| **RAMB36** | **81** | 145 | **55.9%** |
| **URAM** | **64** | 96 | **66.7%** ⭐ |
| **DSP** | **144** | 464 | **31.0%** |

### 子模块 LUT 分布

| Module | LUT | URAM | DSP | 说明 |
|---|---:|---:|---:|---|
| `u_mc_vd100` (我们 PL 加速器) | 116,826 | 64 | 144 | 99.5% 全是它 |
| `smartconnect_0` (PS GP → CSR) | 543 | 0 | 0 | 0.5% glue |
| `proc_sys_reset_0` | 16 | 0 | 0 | 复位同步 |
| `axi_noc_0` (Hard IP) | 0 | 0 | 0 | 不占 PL |
| `versal_cips_0` (Hard IP) | 0 | 0 | 0 | 不占 PL |
| `clk_wizard_0` | 0 | 0 | 0 | 占 BUFG/MMCM |

### Timing
- Clock period: 10.0 ns (100 MHz target)
- **WNS: +1.629 ns** (timing MET 大富余 16.3%)
- **Fmax: 119.5 MHz**

### 跟 OOC 综合对比

| 阶段 | LUT | DSP | URAM | Fmax |
|---|---:|---:|---:|---:|
| OOC synth (multicore_top_vd100_bd) | 128,375 | 144 | 64 | 182 MHz |
| **Routed (含 BD 完整 PS+NoC)** | **117,384** ⬇11K | 144 | 64 | **119.5 MHz** |

routed LUT **少 11K** 是 P&R 优化 (packed cell 更紧凑). Fmax 比 OOC 低 (含 BD framework 完整 timing path), 但**100 MHz target 富余 19.5%**, 实际可拉到 ~119 MHz 跑.

### Bitstream + .xsa
- routed_dcp: ✅ `design_1_wrapper_routed.dcp` 已生成
- **bitstream (.pdi)**: ✅ `design_1_wrapper.pdi` (5.9 MB)
- **.xsa**: ✅ `vd100_resnet11.xsa` (2.5 MB, 含 bitstream)

**License 解决**: 
- 之前 .lic 里 Enterprise Eval 30 day evaluation 优先级覆盖 built-in Standard,
  Vivado 选 Eval license → write_device_image disabled
- 修复: 编辑 .lic 注释/移除 Enterprise Eval entries, 让 Vivado 退到 built-in
  Standard (含 VE2302, 永久, 能 write bitstream)
- 验证: 不设 XILINXD_LICENSE_FILE=空 也能 write_device_image, license 干净

---

## Step 2: Vitis app build (待 .xsa)

(license 解决, .xsa 出来后填)

---

## Step 3: 板上 bring-up (待 bitstream + .xsa)

(完成后填)
