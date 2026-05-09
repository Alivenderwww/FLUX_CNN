# VD100 BD 在 Vivado GUI 里手动搭建 (避开 tcl 自动化坑)

之前 `create_project.tcl` 自动连线时撞 axi_noc 端口名 corner case 失败。
**最稳路径**: tcl 只创建 IP, GUI 拖拽连线。

## 你现在的状态

打开 `output/vd100_resnet11.xpr` 后看到 BD 空白 — 因为之前 tcl 跑到中途报错没保存。

---

## Step 1: 在 GUI Tcl Console 里跑创建脚本

GUI 顶部菜单 **Window → Tcl Console** 打开 console。

确认你在 BD 上 (左边 Sources 双击 `design_1.bd`, canvas 应该出现, 即使空白也算 open)。

如果没 open BD, 在 Tcl Console 跑:
```tcl
open_bd_design [get_files design_1.bd]
```

然后跑创建 IP 脚本 (路径相对工程根, `pwd` 看下):
```tcl
source ../build_bd_ips_only.tcl
```

预期输出:
```
  + versal_cips_0 created
  + clk_wizard_0 created
  + util_ds_buf_0 created
  + proc_sys_reset_0 created
  + axi_noc_0 created (3 SI 128-bit + 1 DDRMC)
  + smartconnect_0 created
  + xlconcat_irq created
  + irq_s0 / irq_s1 / irq_s2 created
  + u_mc_vd100 (multicore_top_vd100_bd) added as BD module
  + sys port / DDR4 port
============================================================
 BD IP-only build done.
```

如果 `u_mc_vd100` 失败 ("无法 add multicore_top_vd100_bd"), 先跑:
```tcl
add_files -norecurse C:/_Project/FLUX_CNN/RTL/Versal/multicore_top_vd100.sv
add_files -norecurse C:/_Project/FLUX_CNN/RTL/Versal/multicore_top_vd100_bd.sv
update_compile_order
```
然后再 `source ../build_bd_ips_only.tcl`。

**Canvas 上应该看到 10 个 IP block + 2 个 interface port (sys/DDR4)**, 都没连线。

---

## Step 2: GUI 手动拖线连接 (4 组连线)

按以下顺序拖, 鼠标点端口 → 拖到目标端口 → 松开。出现绿色虚线就 OK。

### 2.1 时钟路径 (200 MHz → 100 MHz)

| 源 | 目标 |
|---|---|
| `sys` (BD 顶层 port) | `util_ds_buf_0/CLK_IN_D` (拖 interface 整体, 不是单 pin) |
| `util_ds_buf_0/IBUF_OUT` | `clk_wizard_0/clk_in1` |
| `util_ds_buf_0/IBUF_OUT` | `axi_noc_0/sys_clk0` |
| `clk_wizard_0/clk_out1` | `axi_noc_0/aclk0` |
| `clk_wizard_0/clk_out1` | `axi_noc_0/aclk1` |
| `clk_wizard_0/clk_out1` | `axi_noc_0/aclk2` |
| `clk_wizard_0/clk_out1` | `proc_sys_reset_0/slowest_sync_clk` |
| `clk_wizard_0/clk_out1` | `smartconnect_0/aclk` |
| `clk_wizard_0/clk_out1` | `u_mc_vd100/clk` |
| `clk_wizard_0/clk_out1` | `versal_cips_0/m_axi_fpd_aclk` |

### 2.2 复位路径

| 源 | 目标 |
|---|---|
| `versal_cips_0/pl0_resetn` | `proc_sys_reset_0/ext_reset_in` |
| `proc_sys_reset_0/peripheral_aresetn` | `u_mc_vd100/rst_n` |
| `proc_sys_reset_0/peripheral_aresetn` | `smartconnect_0/aresetn` |

### 2.3 AXI 数据路径

| 源 | 目标 |
|---|---|
| `versal_cips_0/M_AXI_FPD` | `smartconnect_0/S00_AXI` (整个 AXI interface, 拖箭头那种) |
| `smartconnect_0/M00_AXI` | `u_mc_vd100/csr_axil` |
| `u_mc_vd100/m00_axi` | `axi_noc_0/S00_AXI` |
| `u_mc_vd100/m01_axi` | `axi_noc_0/S01_AXI` |
| `u_mc_vd100/m02_axi` | `axi_noc_0/S02_AXI` |
| `axi_noc_0/CH0_DDR4_0` | `DDR4` (BD 顶层 port) |

### 2.4 IRQ 路径

| 源 | 目标 |
|---|---|
| `u_mc_vd100/irq_done` (3-bit vector) | `irq_s0/Din` |
| `u_mc_vd100/irq_done` | `irq_s1/Din` |
| `u_mc_vd100/irq_done` | `irq_s2/Din` |
| `irq_s0/Dout` | `xlconcat_irq/In0` |
| `irq_s1/Dout` | `xlconcat_irq/In1` |
| `irq_s2/Dout` | `xlconcat_irq/In2` |
| `xlconcat_irq/dout` | `versal_cips_0/pl_ps_irq0` (需 Step 3 先启用) |

---

## Step 3: 启用 versal_cips_0 的 PL→PS IRQ

双击 `versal_cips_0` 进配置:

1. 左侧 Page Navigator → "I/O Configuration"
2. 左下角 search 框搜 "Interrupts"
3. 展开 "Interrupts" → 勾 "PL to PS Interrupt 0" (`pl_ps_irq0`)
4. 配置 `pl_ps_irq0` 宽度: 设 `[0:0]` 或 `[2:0]`
5. OK 关闭, BD 上 `versal_cips_0` 应该出 `pl_ps_irq0[0:0]` 端口
6. 然后回到 BD 把 `xlconcat_irq/dout` 连到这个端口

---

## Step 4: Address Editor 检查/分配地址

GUI 顶部菜单 **Window → Address Editor** (或 BD 工具栏的 "Address Editor" tab)

应该看到两组:

### `versal_cips_0/M_AXI_FPD` master:
- `u_mc_vd100/csr_axil/Reg`: **手动 Assign** → Offset Address `0xA0000000`, Range `16K`

### `u_mc_vd100/m00_axi` / `m01_axi` / `m02_axi` master (3 个):
- `axi_noc_0/S00/01/02_AXI/C0_DDR_LOW0`: **手动 Assign** → Offset `0x00000000`, Range `2G`

如果某行有 ⚠️ 红色 unassigned, 右键 "Assign Address"。

---

## Step 5: Validate Design

按 **F6** (或菜单 Tools → Validate Design)。

预期: 弹窗 "Validation Successful" + 0 errors。

可能 warning (不 fatal):
- `clk_wizard_0` 输入时钟没绑 board file → 加 board file 或忽略
- `axi_noc_0` PHY 配置 → 跟 ALINX VD100 实际 DDR4 SODIMM 配置可能要调

---

## Step 6: 综合 + 实现 + bitstream

GUI Tcl Console 跑:
```tcl
make_wrapper -files [get_files design_1.bd] -top -import
update_compile_order -fileset sources_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
```

等 30-60 min。

---

## Step 7: Export hardware (.xsa) 给 Vitis

GUI 菜单 **File → Export → Export Hardware...**
- 勾 "Include bitstream"
- 路径: `C:\_Project\FLUX_CNN\Syn\vd100_bd\output\vd100_resnet11.xsa`

完成后回 [BRINGUP.md Step 2](../host/BRINGUP.md) 做 Vitis baremetal app。

---

## 故障排查

| 症状 | 原因 | 修 |
|---|---|---|
| Tcl Console `source ../build_bd_ips_only.tcl` 报 "no such file" | 路径不对 | `pwd` 看, 用绝对路径: `source C:/_Project/FLUX_CNN/Syn/vd100_bd/build_bd_ips_only.tcl` |
| `multicore_top_vd100_bd` 不在 module 列表 | RTL 没 add 进工程 | Step 1 备注里的 add_files 命令 |
| 拖线时弹 "data width mismatch" | smartconnect 端口宽度跟 csr_axil 不一致 | 双击 smartconnect, M00_AXI 配 ADDR_WIDTH=14 / DATA_WIDTH=32 / PROTOCOL=AXI4LITE |
| axi_noc 端口 S00_AXI 不存在 (只有 S00_INI) | IP 版本差异 | 双击 axi_noc_0 → "Inputs" 页, 设 NUM_SI=3 + 每个 SI 的 "Connected To" 设 MC, "Configuration Options" → DATA_WIDTH 128 |
| Validate 报 "associated reset not found" | clk_wizard.clk_out1 ASSOCIATED_RESET 没设 | 双击 clk_wizard, 找 clk_out1 输出配置 → ASSOCIATED_RESET = peripheral_aresetn |
| Synth 失败 axi_noc DDR4 PHY config | board file 没装 | 装 ALINX VD100 board files (或者 Tools → Edit Device Properties 手动配 DDR4 pin) |

---

## 板级 xdc IO 约束

ALINX 02_pl_rw_ddr 提供的 xdc 已经 copy 进工程 (constrs_1)。
DDR4 + sys clk pin 应该已经绑好。

如果 implementation 报 unconstrained pin, 检查 `Syn/vd100_bd/output/vd100_resnet11.srcs/constrs_1/new/`
有没有 `ddr4.xdc` + `system.xdc`。
