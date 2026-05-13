# VD100 BD-level Sim Infrastructure 状态 (2026-05-13)

## 一句话现状

ModelSim + Vivado xsim 两条 BD sim 路径 infrastructure 全部 setup 完成 (compile_simlib +
RTL compile + testbench + lib mapping 都 work), 但 cips_vip 在两条路径上都没正确 boot
PL_CLK/pl_rstn — NoC NMU BFM 在 ~20 ns 检测无 valid 时序触发 `$finish` 停 sim,
ConvCore 没机会跑.

## 已完成 infrastructure (commit f0fc665 + 之前)

### Vivado simlib (versal family)
- `Syn/compile_simlib_versal.tcl`: 编 Versal 所有 IP sim lib
- 关键 lib 都 OK: secureip (含 SIP_XRAM), axi_noc NMU/NSU/XBR, noc_mc_ddr4_phy,
  axi_vip_v1_1_16, versal_cips_ps_vip_v1_0_8, cpm5_v1_0_15, smartconnect_v1_0,
  proc_sys_reset_v5_0_14, lib_pkg_v1_0_3, axi_apb_bridge_v3_0_19, xlconcat_v2_1_5,
  xlslice_v1_0_3, xilinx_vip, unisim/unisims_ver, unimacro/unimacro_ver, xpm
- compile_simlib 大概 90% 完成 (剩 s-z 段 simprim/srio 等不影响 BD sim)

### BD sim export
- `Syn/vd100_bd/bd_sim_export.tcl`: export_simulation -simulator modelsim 成功
  生成 compile.do + simulate.do + design_1.sh
- `Syn/vd100_bd/setup_sim_fileset.tcl`: 把 ConvCore RTL add 到 sim_1 fileset
- BD sim source 含: cips_vip + axi_noc (NMU/NSU/XBR/DDR4 phy responder) + smartconnect_pl/0
  + axi_dm IP sim model

### ConvCore RTL compile
- `sim/tb_bd_full/design_1/modelsim/add_convcore_rtl.do`: 编 ConvCore RTL +
  multicore_top_vd100 + multicore_top_vd100_bd + multicore_top_vd100_bd_v + design_1_wrapper
  + tb_bd_top 到 xil_defaultlib
- 全部 0 errors compile

### Testbench
- `sim/tb_bd_full/tb_bd_top.sv`: 实例化 design_1_wrapper, drive sys_clk 200 MHz diff,
  DDR4 ports dangling (axi_noc 内置 DDR responder), hier-ref print sequencer +
  dispatcher state
- 正确 hier path: `tb_bd_top.dut.design_1_i.u_mc_vd100.inst.u_inner.u_inner.gen_core[0].u_core`

## 阻塞点: NoC NMU BFM $finish

ModelSim 跑 BD sim 时:
- t=0 ps: cips_vip + axi_noc + 多个 SIP_XRAM 初始化 (Time:0 ps print 大量)
- t=20801 ps: `nmu_axi_monitor.sv:3355` 触发 `$finish` 停 sim
- 期间无 PL_CLK0/pl_rstn 释放, ConvCore 一直 reset

加密源:
- `noc_nmu_v1_0_vl_rfs.sv:3355` (Xilinx 加密) — NoC NMU sim model 实际 trigger
- `nmu_axi_monitor.sv` (Xilinx 加密 BFM)

试过的绕开:
- `-onfinish stop`: 让 sim 不 quit 但也不再 advance (effective stop)
- `-suppress xxx`: 不知 finish 是 warning 还是 error
- `disable` hier-ref: 加密路径 disable 不 work

## Vivado xsim 路径也失败

`Syn/vd100_bd/xsim_launch.tcl` 试 `launch_simulation -batch` 调 Vivado 自家 xsim:
- xsim compile.bat 生成 OK (设 PATH 含 Vivado bin 后)
- xvlog/xvhdl analyze 步骤 INFO 行很多但 log 文件空 (0 byte)
- Vivado batch launch_simulation 报 `ERROR: [Common 17-180] Spawn failed: Broken pipe`
- 推测是 license / batch mode 进程 spawn 限制

## 真根因 hypothesis

cips_vip Vivado xsim 内置 boot sequence 用 `.cdo` 配置文件 (bd_70da_pspmc_0_0_*.cdo) +
私有 SystemVerilog DPI/PLI 调用. ModelSim 不知道怎么 load .cdo / 调 cips 私有 API.

正常 Vivado BD sim 流程 (Vivado IDE 内 Run Behavioral Simulation):
1. Vivado xsim 自动配 sim runtime, load cdo
2. cips_vip boot, 拉 PL_CLK0
3. PL reset 释放
4. NoC NMU 看到有效 traffic, $finish 不触发
5. testbench 通过 cips VIP API 调 write_data 写 csr_axil
6. ConvCore 跑 layer

但 batch mode + 命令行 ModelSim/xsim 不能完整复制 Vivado IDE 行为.

## 下次 session 建议

### 方案 A: 手动在 Vivado IDE 跑 BD sim (推荐)

1. 打开 Vivado: `D:\Xilinx\Vivado\2023.2\bin\vivado.bat output/vd100_resnet11.xpr`
2. Tools → Settings → Simulation: 选 simulator = Vivado Simulator (xsim default)
3. Project Manager → Add Sources → Add or create simulation sources → tb_bd_top.sv
4. Set sim_1 top = tb_bd_top
5. Flow Navigator → Run Simulation → Run Behavioral Simulation
6. GUI 内观察 cips_vip boot + axi_noc init
7. tb_bd_top 内补 cips VIP write_data API 调用替代 a72 deploy_smc_case
8. Add waveform, sim 跑到 232 row OFM 停, 看 axi_dm s2mm AW/W/B 信号

### 方案 B: Vivado xsim CLI 调试

Vivado batch launch_simulation 的 broken pipe 问题:
- 试 `-mode tcl` 而非 `-mode batch` (interactive)
- 检查 Windows process limit
- 看 license server 是否有 xsim license

### 方案 C: 接受 sim 不通, 走 ILA 实测

Board layer 0 stuck deterministic (OFM 永远 166KB / 232 row), 适合 ILA 抓 stuck 瞬间.
之前删 ila_dbg 释放 LUT, 可以单独抓 ConvCore[0] axi_dm.s2mm AW/W/B 通道:
- 删现有 dbg port 释放 ~2K LUT 给 ila_dbg
- ila_dbg DEPTH=1024 + 8 probes
- xsct 调 hw_ila API readback capture data

## 复用资产

当前 BD sim infrastructure 99% setup 完成, NoC NMU $finish 是唯一 blocker. 任何用 Vivado 2023.2.1
+ VE2302 + Versal axi_noc + cips_vip 的项目都能复用这套 setup:
- compile_simlib_versal.tcl (一次性 lib 编译)
- bd_sim_export.tcl (BD sim source 导出)
- tb_bd_top.sv (testbench 框架)
- add_convcore_rtl.do (ConvCore RTL 编译模板)

后续 BD bug 调试不用从零开始.
