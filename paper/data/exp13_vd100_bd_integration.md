# 实验 13: VD100 BD 集成 (Vivado 板级工程脚手架)

**日期**: 2026-05-09
**状态**: BD wrapper RTL + create_project tcl 完成, 跑 Vivado 集成测试中

## 跟 exp12 (OOC 综合) 区别

| 维度 | exp12 (OOC) | exp13 (BD 板级) |
|---|---|---|
| 范围 | 仅 multicore_top_vd100 RTL | + Versal CIPS + axi_noc + DDRMC + clk_wizard + smartconnect |
| 资源数据 | OOC LUT/DSP/URAM | 加 PS BD 后的完整 routed 数据 |
| 用途 | RTL 验证可装下 VE2302 | 板级真实占用 + 准备 bitstream |
| 工程文件 | Syn/run_syn_vd100.tcl | Syn/vd100_bd/* (4 tcl + RTL wrapper) |

## RTL 改动 (BD 友好 wrapper)

新增 `RTL/Versal/multicore_top_vd100_bd.sv` (~390 行):
- 把 multicore_top_vd100 的 packed AXI 端口拆成 3 套独立 m00_axi/m01_axi/m02_axi
  + 1 个 csr_axil (AXI-Lite slave) + 1 个 irq_done (3-bit IRQ)
- 每个 AXI 端口加 Vivado `(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 ..." *)`
  attribute, BD `connect_bd_intf_net` 自动识别
- 内部纯 wire/assign 转换 packed ↔ 独立信号, 综合后 flatten 掉, 无额外资源

## BD 工程结构

```
Vivado design_1 (板级 BD)
├── versal_cips_0           (PS A72 + R5F + LPD + FPD)
│   └── M_AXI_FPD           → smartconnect → multicore_top_vd100_bd.csr_axil
│   └── pl_ps_irq[2:0]      ← xlconcat ← multicore_top_vd100_bd.irq_done
│   └── pl0_ref_clk          → clk_wizard input
│   └── pl0_resetn           → proc_sys_reset
├── clk_wizard_0            (200 MHz IBUFDS → 100 MHz axi_clk)
├── util_ds_buf_0           (sys 差分时钟 IBUFDS)
├── proc_sys_reset_0        (axi_clk reset 同步)
├── smartconnect_0          (PS GP → CSR aperture)
├── xlconcat_irq + 3 xlslice (3-bit irq_done → PS pl_ps_irq)
├── axi_noc_0               (3 SI 128-bit + 1 DDRMC, DDR4-3200)
└── multicore_top_vd100_bd  ⭐ 我们的 PL 加速器
    ├── 3 ConvCore (LUT-MAC, NUM_CORES=3)
    └── 1to3 csr demux

外部 IO:
- sys (200 MHz diff clk input)
- DDR4 interface (板载 DDR4 64-bit)
- (PS 网口 / USB / SD 通过 CIPS 自动连)
```

## Address Map

| Master | Address | Size | Target |
|---|---|---|---|
| PL 3 master (m00/01/02_axi) | 0x0000_0000 - 0x7FFF_FFFF | 2 GB | DDR4 low |
| PS GP (M_AXI_FPD) | 0xA000_0000 - 0xA000_3FFF | 16 KB | multicore_top_vd100_bd.csr_axil |

CSR aperture 内 4 KB / core × 3 core, addr[13:12] 选 core (跟 RTL 1to3 demux 一致).

## 集成进度

| 步骤 | 状态 |
|---|---|
| BD wrapper RTL (X_INTERFACE_INFO) | ✅ vlog 编译过 |
| pl_config.tcl 改用 wrapper | ✅ |
| create_project.tcl 加 wrapper + ALINX xdc | ✅ |
| **OOC 综合 multicore_top_vd100_bd 验证** | ✅ **PASS** |
| Vivado BD create_project + validate | ⚠️ tcl 自动化跑到 axi_noc 端口名时报错, 需 GUI 手动 finalize |
| 板级综合 + P&R + bitstream | ⏳ 待 BD GUI 调通后 |

## OOC 综合 multicore_top_vd100_bd 实测

跟原 multicore_top_vd100 完全一致 (wrapper 综合 flatten 后无额外开销):

| 指标 | multicore_top_vd100 | multicore_top_vd100_bd | 差异 |
|---|---:|---:|---:|
| Total LUT | 128,375 | 128,375 | 0 (一致) |
| wrapper 自身 LUT | – | 60 | wrapper 只是 X_INTERFACE assign |
| inner module LUT | – | 128,315 | 跟原 module 一致 |
| WNS @ 10 ns | +4.514 ns | +4.514 ns | 0 |
| **Fmax** | **182.3 MHz** | **182.3 MHz** | 0 |
| DSP / RAMB36 / URAM | 144 / 81 / 64 | 144 / 81 / 64 | 0 |

**wrapper 60 LUT 仅 0.05% (60/128,375)**, 完全 transparent. X_INTERFACE_INFO attribute
是 Vivado metadata, 不引入逻辑.

## BD create_project tcl 自动化失败原因

```
ERROR: [Common 17-55] 'set_property' expects at least one object.
```

发生在 axi_noc 端口名设置时:
```tcl
set_property CONFIG.CONNECTIONS [...] [get_bd_intf_pins /axi_noc_0/S00_AXI]
```

Vivado axi_noc IP 的 SI 端口实际命名跟 IP 版本/配置相关 (有版本是 `S00_AXI`,
有版本是 `S00_INI`), tcl 写死端口名容易 mismatch. ALINX/Xilinx 推荐用
GUI 拖拽 + tcl 微调, 不能 100% script 自动化.

## 最终路径 (board 实物到位后)

1. 在 Vivado 2023.2 GUI 打开 `Syn/vd100_bd/output/vd100_resnet11.xpr`
2. 手动检查 axi_noc.SI 端口实际名, 修 pl_config.tcl 或 GUI 直接连 (5-10 min)
3. validate_bd_design + 跑 P&R + write_bitstream (~30-60 min)
4. 拿 routed LUT/DSP/Fmax (含 PS BD 完整数据)
5. 出 .xsa 给 Vitis (PS host runtime 编译)
6. Petalinux 镜像 + UIO 设备树 + boot SD 卡
7. 板上 bring-up: 上电 → ssh root → 跑 server → PC client 验证

## 复现

```cmd
:: 1. OOC 验证 BD wrapper (~10-15 min)
cd C:\_Project\FLUX_CNN
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source Syn\run_syn_vd100.tcl -nojournal -log Syn\syn_vd100_bd.log

:: 2. 创建完整 BD 工程 (会失败在 axi_noc, 但工程文件能产生)
cd C:\_Project\FLUX_CNN\Syn\vd100_bd
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source create_project.tcl -nojournal -log create_bd.log

:: 3. GUI finalize (推荐)
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" output\vd100_resnet11.xpr
:: GUI 内: 检查 BD, 修 axi_noc 端口名, validate_bd_design
::         launch_runs synth_1 -jobs 8; wait_on_run synth_1
::         launch_runs impl_1 -to_step write_bitstream -jobs 8; wait_on_run impl_1
```

## 复现

```cmd
:: 1. OOC 验证 BD wrapper (~10-15 min)
cd C:\_Project\FLUX_CNN
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source Syn\run_syn_vd100.tcl -nojournal -log Syn\syn_vd100_bd.log

:: 2. 创建完整 BD 工程
cd C:\_Project\FLUX_CNN\Syn\vd100_bd
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source create_project.tcl -nojournal -log create_bd.log

:: 3. 板级 P&R + bitstream (在 GUI 里 launch_runs)
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" output\vd100_resnet11.xpr
:: GUI 内: launch_runs synth_1 -jobs 8; wait_on_run synth_1
::         launch_runs impl_1 -to_step write_bitstream -jobs 8; wait_on_run impl_1
```
