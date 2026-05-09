# VD100 板上 bring-up 操作手册

**目标**: VD100 板 (VE2302) 跑 ResNet11 demo, PC 千兆网发图 → 板上推理 → 取结果

## 两条路径选择

| 路径 | 时间 | 复杂度 | 推荐 |
|---|---|---|---|
| **A. Baremetal (lwIP)** | **1-2 天** | 中 | ⭐ 推荐 demo |
| B. Petalinux (Linux socket) | 3-5 天 | 高 | 完整方案, 后续可扩 |

**先走 A 拿到 demo**, 跑通后想升级再走 B。下面默认 A 路径。

---

## 步骤 1: Vivado 2023.2 打开 BD 工程, 调通 axi_noc 端口

**估时**: 30 min - 1 h

```cmd
cd C:\_Project\FLUX_CNN\Syn\vd100_bd
"D:\Xilinx\Vivado\2023.2\bin\vivado.bat" output\vd100_resnet11.xpr
```

GUI 操作:

### 1.1 看 BD 当前状态
- 左侧 Sources → 双击 `design_1.bd` 打开 Block Design
- 应该能看到: versal_cips_0, util_ds_buf_0, clk_wizard_0, proc_sys_reset_0, smartconnect_0, axi_noc_0, xlconcat_irq, irq_s0/1/2

### 1.2 加入 multicore_top_vd100_bd 当 BD module
- 右键 BD 空白处 → "Add Module..." → 选 `multicore_top_vd100_bd` → OK
- 应该出现 `multicore_top_vd100_bd_0` 实例, 端口已识别成 `csr_axil` / `m00_axi` / `m01_axi` / `m02_axi` / `irq_done` interface

### 1.3 连线 (拖拽 GUI)
| 源 | 目标 |
|---|---|
| `clk_wizard_0/clk_out1` | `multicore_top_vd100_bd_0/clk` |
| `proc_sys_reset_0/peripheral_aresetn` | `multicore_top_vd100_bd_0/rst_n` |
| `smartconnect_0/M00_AXI` | `multicore_top_vd100_bd_0/csr_axil` (interface) |
| `multicore_top_vd100_bd_0/m00_axi` | `axi_noc_0/S00_AXI` (interface) |
| `multicore_top_vd100_bd_0/m01_axi` | `axi_noc_0/S01_AXI` (interface) |
| `multicore_top_vd100_bd_0/m02_axi` | `axi_noc_0/S02_AXI` (interface) |
| `multicore_top_vd100_bd_0/irq_done` (3-bit) | 拆 3 个 xlslice → `xlconcat_irq/In0/In1/In2` |
| `xlconcat_irq/dout` | `versal_cips_0/pl_ps_irq0` (需 ps_config 启用 PL→PS IRQ) |

### 1.4 启用 PS PL→PS IRQ (双击 versal_cips_0)
- Page Navigator → "PS PMC" → "I/O Configuration"
- 找到 "Interrupts" → 勾 "PL to PS Interrupt 0"
- OK 关闭, BD 上 versal_cips_0 应该出 `pl_ps_irq0[0:0]` 端口

### 1.5 Address Editor 检查
- BD 顶部菜单 "Window" → "Address Editor"
- 检查:
  - PL m00/01/02_axi 都有 `axi_noc_0/S00/01/02_AXI/C0_DDR_LOW0` segment, base=0x0000_0000, size=2GB
  - PS GP M_AXI_FPD 有 `multicore_top_vd100_bd_0/csr_axil/Reg`, base=0xA000_0000, size=16K
- 缺哪个右键 "Assign Address"

### 1.6 Validate Design
- BD 上 F6 (或菜单 Tools → Validate Design)
- 报错: 修. 应该 0 errors / 少量 warnings

### 1.7 生成 BD wrapper + 综合 + bitstream
GUI 顶部 Tcl Console:
```tcl
make_wrapper -files [get_files design_1.bd] -top -import
update_compile_order -fileset sources_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
```
等 30-60 min。

### 1.8 Export hardware (.xsa) 给 Vitis
GUI 菜单 File → Export → Export Hardware...
- 勾 "Include bitstream"
- 路径填: `C:\_Project\FLUX_CNN\Syn\vd100_bd\output\vd100_resnet11.xsa`

**完成后**: 拿到 `vd100_resnet11.bit` + `vd100_resnet11.xsa`, routed 资源/Fmax 数据也有了。

---

## 步骤 2: Vitis 2023.2 创建 baremetal app

**估时**: 2-3 h

```cmd
cd C:\_Project\FLUX_CNN\host\vd100_ps_baremetal
"D:\Xilinx\Vitis\2023.2\bin\vitis.bat"
```

### 2.1 Vitis 启动 → File → New Application Project
- Platform: 选 "Create new platform from hardware (XSA)" → 选刚 export 的 `vd100_resnet11.xsa`
- Application name: `resnet11_server`
- Processor: `psv_cortexa72_0` (PS A72 第一个核)
- Domain: standalone (baremetal, 不要 linux)
- Template: "lwIP Echo Server" (有现成 lwIP TCP 模板, 我们改造它)

### 2.2 改造 echoserver.c → resnet11_server
- 在 src/ 下找 `echo.c`, 复制 host/vd100_ps_baremetal/resnet11_main.c (我准备好的, 见下文)
- 修 main.c 改 LISTEN_PORT 5000 + recv_callback 处理 INFER 协议

### 2.3 Build + Run
- 右键工程 → Build Project (~3-5 min)
- 板子接 USB (JTAG + UART), 上电
- 右键 → Run As → "Launch Hardware (Single Application Debug)"
- Vitis 自动: 下 bit → 烧 baremetal elf → JTAG 启动 → console 打 "Listen on 5000"

---

## 步骤 3: 板上 bring-up + PC 端验证

**估时**: 1-2 h (含 debug)

### 3.1 板子物理连接
| VD100 接口 | 接什么 |
|---|---|
| 12V DC 电源 | 上电 |
| USB JTAG | PC USB (Vivado/Vitis 调试 + UART log) |
| RJ45 ETH0 (PS 端) | 跟 PC 直连或同一交换机 |
| 板载 LED | 看 PS 心跳 / done IRQ 闪 |

### 3.2 PC IP 配置
- 板子 lwIP 默认 static IP 192.168.1.10
- PC 网卡设静态 192.168.1.20 / 255.255.255.0 (改控制面板 / netsh)
- ping 192.168.1.10 通 → 网络 OK

### 3.3 跑 PC client
```cmd
cd C:\_Project\FLUX_CNN\host\vd100_pc
python resnet11_client.py ^
    --image C:\_Project\FLUX_CNN\toolchain\models\images\resnet11_test\gradient.png ^
    --vd100-ip 192.168.1.10 ^
    --port 5000
```

期望输出:
```
[1/3] 加载图片: gradient.png
  IFM 大小: 2025.0 KB, 用时 ~200 ms
[2/3] 连接 VD100: 192.168.1.10:5000
  连接成功
[3/3] 推理 × 1
  Top-5 logit (random weights, 仅 demo):
    #1  class[X]  = +127
    ...
HW cycles 平均: ~230000 cy   (跟 sim 一致就 OK!)
端到端 RTT 平均: ~50 ms      (含网络往返)
```

---

## 故障排查

| 现象 | 原因 | 修 |
|---|---|---|
| Vivado BD validate 失败 "axi_noc port unconnected" | 端口名 mismatch | GUI 双击 axi_noc 看实际端口名, 改 connect_bd_intf_net |
| Vivado synth 报 "module multicore_top_vd100_bd not found" | RTL 没加进 fileset | Add Source → 选 `RTL/Versal/multicore_top_vd100_bd.sv` + 所有依赖 |
| Vivado timing 不达 100 MHz | LUT 85% 紧 | Tools → Timing → Edit Timing Constraints, 降 clk 到 80 MHz; 或者 retiming + register balancing |
| Vitis lwIP 模板找不到 | Vitis Standard 版没装 lwIP | 装 Vitis Embedded Workflow 模块 |
| ping 不通 | PHY 延时 / IP 配置 | 看 Vitis console UART 是否打 "DHCP/Static IP done" |
| client 连不上 5000 | 防火墙 | Windows 防火墙允许 python.exe 入站 |
| HW cy 跟 sim 差很多 | DDR4 latency 比 sim 大 | 正常, 板上有 DDR ~50-100 cy 额外, sim cy 仅参考 |

---

## 板级数据填回 paper

bringup 跑通后, 数据填到 `paper/data/exp14_vd100_board_demo.md` (我会准备模板):
- routed LUT/DSP/Fmax (Vivado P&R 报告)
- 板上实测 cy (跟 sim 对比)
- 端到端 RTT (PC ↔ board)
- demo 视频/截图

---

## 路径 B (Petalinux) 简述

如果 baremetal 跑通后想升级 Linux:

1. 装 Petalinux 2023.2 SDK (Linux 主机, ~30GB)
2. `petalinux-create -t project --template versal -n vd100_resnet11`
3. `petalinux-config --get-hw-description=path/to/vd100_resnet11.xsa`
4. 改 device-tree 加 UIO + reserved-memory
5. petalinux-build (~30-60 min)
6. petalinux-package --boot 出 BOOT.BIN + image.ub
7. 烧 SD 卡 (SDFormatter + dd image)
8. 板子 boot 模式拨 SD, 上电
9. SCP `host/vd100_ps/resnet11_server.c` 编译后传上去 → ./run

Petalinux 的好处: 更方便部署 (多客户端 / 后台 service / file IO), 调试方便 (gdb / ssh).
缺点: 学习曲线 + Linux 主机要求.

---

**当前下一步**: 跑步骤 1.1 在 Vivado 打开工程, 检查 BD. 我可以指导每一步。
