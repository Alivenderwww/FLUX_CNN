# VD100 Block Design 工程

VD100 板级 demo 的 Vivado BD 工程脚手架, 基于 ALINX 02_pl_rw_ddr 模板改造。

## 文件

| 文件 | 用途 |
|---|---|
| `project_info.tcl` | Part / 工程名 / BD 名等参数 |
| `ps_config.tcl` | Versal CIPS (PS A72) 配置 (启用 M_AXI_FPD GP master) |
| `pl_config.tcl` | PL 端 BD 拼接 (axi_noc + DDRMC + clk_wizard + smartconnect + IRQ + multicore_top_vd100) |
| `create_project.tcl` | 主入口, 跑这个生成完整 Vivado 工程 |

## 用法

```
cd C:\_Project\FLUX_CNN\Syn\vd100_bd
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source create_project.tcl -nojournal -log create.log
```

工程输出到 `vd100_bd/output/vd100_resnet11.xpr`。

## 已知 TODO (BD 完整调通)

`create_project.tcl` 跑到 axi_noc 端口名设置时报错 `set_property expects at least one object`,
原因是 **axi_noc IP 的 SI 端口名跟 IP 版本相关** (有些版本是 `S00_AXI`, 有些是 `S00_INI`),
需要在 Vivado GUI 里看实际端口名再修 tcl. 这是 BD 自动化的常见 corner case,
ALINX/Xilinx 推荐用 GUI 拖拽 + tcl 微调, 不能 100% script 自动化.

当前脚本跑 create_project.tcl 后需要手动调通几件事:

1. **multicore_top_vd100 AXI interface 识别**:
   - RTL 端口是 packed vector (NUM_CORES*BUS_DATA_W bit 等), Vivado BD `create_bd_cell -reference` 不会自动识别为 3 个独立 AXI interface
   - 解决: 写 wrapper `multicore_top_vd100_bd.sv` 把每核 AXI 单独 expose 端口 + 加 `(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi_0 AWID" *)` 等 attribute. 工作量约 200 行 wrapper.
   - 或者: BD 用 net 级别连线 (connect_bd_net 逐信号), 不用 connect_bd_intf_net

2. **PS pl_ps_irq 接 xlconcat 输出**:
   - ps_config.tcl 加 `PS_NUM_FABRIC_INTERRUPTS_INPUT {3}` 启用 PL → PS irq[2:0]
   - 然后 connect_bd_net xlconcat_irq.dout → versal_cips.pl_ps_irq0

3. **板级 IO 约束 xdc**:
   - DDR4 pinout: 跟 ALINX 02_pl_rw_ddr/src/constraints/ddr4.xdc 一致 (拷过来)
   - sys_clk pin: 板载 200 MHz 差分时钟引脚
   - reset 按键: 用 PS pl0_resetn (不需要外部 reset)

4. **Petalinux 设备树 UIO**:
   - `<axi_lite_csr_aperture>` 0xA000_0000 - 0xA000_3FFF 标记为 `compatible = "generic-uio"` 给 Linux UIO driver
   - PS DDR4 用 reserved-memory 区分 Linux 内核可用区 + 加速器 IO 区

5. **axi_dm IP cross-device retarget**:
   - K325T 的 axi_dm.xci 在 VE2302 上是 LOCKED (different part), Vivado 跑 read_ip 时报 warning 但能 retarget
   - 实测 OOC 综合 PASS (Syn/reports_vd100/utilization_synth.rpt 显示 axi_dm × 3 实例化成功)
   - 板级 P&R 时 likely 一样能用. 如果有 IP_FLOW 错误, 重新 generate axi_dm IP for VE2302 (跑 gen_axi_datamover.tcl 改 PART)

## 下一步预估时间

| 步骤 | 工作量 | 说明 |
|---|---|---|
| 写 multicore_top_vd100_bd.sv wrapper (3 AXI interface 单独 expose) | 1-2 天 | 200 行 wrapper, X_INTERFACE_INFO attribute |
| pl_config.tcl 改用 wrapper + 测试 BD validate | 0.5 天 | 跑 validate_bd_design 看 connection 全 OK |
| 板级 P&R + bitstream | 0.5 天 (Vivado 跑) + 0.5 天 timing 调 | 需要看 routed WNS 看 timing 是否 met |
| Petalinux 镜像定制 + boot SD | 1 周 | UIO + reserved-memory + GEM 跟 host runtime 配合 |
| 板上 bring-up + ResNet11 demo | 1 周 | 上电 + ssh root + 跑 server + PC client 验证 |

**总计**: 4-5 周到 demo (不算 debug 反复). 当前 paper 角度 OOC 综合数据 (128K LUT / 182 MHz) 已经够.
