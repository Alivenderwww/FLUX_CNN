# VD100 ResNet11 Demo 端到端 Bring-up 进度

日期: 2026-05-09  
**当前板状态**: 烧 ALINX 01_led 例程 (低负载占位), 等下次 demo 调试.

## 当前目录文件 (已整理)

```
host/vd100_ps_baremetal/
├── PROGRESS.md                       <-- 本文件
├── resnet11_main.c                   <-- A72 lwIP TCP server source (single source of truth)
├── create_vitis_app.tcl              <-- Vitis 平台 + app 创建脚本 (用 workspace2/)
├── program_pdi_via_vivado.tcl        <-- Vivado batch 烧 PDI (替代 xsct device program)
├── vd100_with_elf.bif                <-- bootgen BIF, image_id=0x1c000000 嵌 a72 ELF
├── vd100_with_elf.pdi                <-- 已生成的 PDI (含我们 PL bitstream + a72 ELF)
├── run_demo_full.ps1                 <-- 端到端 wrapper (烧 PDI + 提示 GUI)
├── run_xsct.ps1                      <-- xsct wrapper (Vivado hw_server + connect -url)
├── jtag_run_elf_only.tcl             <-- xsct 烧 PDI + dow ELF + run (legacy 备用)
├── jtag_check_a72.tcl                <-- 诊断: A72 PC + 寄存器
├── jtag_check_gem.tcl                <-- 诊断: GEM0 link state
├── jtag_probe_mem.tcl                <-- 诊断: PMC/A72 mwr OCM/XRAM/DDR
├── jtag_stop_a72.tcl                 <-- 诊断: stop A72 + dump regs
└── workspace2/                       <-- Vitis platform/app build 输出
```

## 已自动化的步骤

| 步骤 | 脚本 | 时间 | 备注 |
|---|---|---|---|
| 1. Vivado 综合 (BD + bitstream + .xsa) | `Syn/vd100_bd/apply_config.tcl` | 1-1.5 h | 含 PSU FPD-CCI-NOC + IPI ch0..6 + axi_noc 9 SI (3 PL + 6 PS NMU) |
| 2. Vitis BSP/app build | `host/vd100_ps_baremetal/create_vitis_app.tcl` | 5-10 min | workspace2/ 全新, mmu_tbl 跟新 .xsa 一致 |
| 3. bootgen 嵌 A72 ELF 到 PDI | `host/vd100_ps_baremetal/vd100_with_elf.bif` + bootgen | <10 s | image_id=0x1c000000 (PM_SUBSYS_DEFAULT) |
| 4. Vivado backend 烧 PDI | `host/vd100_ps_baremetal/program_pdi_via_vivado.tcl` | ~30 s | 自动 sideband reset, 重烧无需 power-cycle |
| **All-in-one** | `host/vd100_ps_baremetal/run_demo_full.ps1` | ~1 min (烧+启动) | 调 Step 4, PLM 自动 load A72 ELF |

## 已修复的关键 bug

1. **xsct hw_server 看不到 cable**:
   - 根因: Vitis 路径下 hw_server 不加载 Digilent JTAG plugin
   - 修复: [run_xsct.ps1](run_xsct.ps1) wrapper 显式启 Vivado 路径下 hw_server.bat + xsct connect -url
2. **xsct device program 重烧 PDI hang**:
   - 根因: PMC 已 boot 状态下 lightweight reset 不够触发 POR
   - 修复: 改用 Vivado tcl `program_hw_devices` (跟 HW Manager GUI 同 backend), 无需 power-cycle
3. **A72 写 DDR 0x40000000 firewall 阻 (DAP transaction error)**:
   - 根因: A72 通过 DAP 调试通道写 DDR, PMC default subsystem firewall 拒绝
   - 修复: bootgen 嵌 ELF 进 PDI, PLM 是 root 自动 load, 不轴 firewall
4. **PLM Major 0x33C / Minor 0x07F4 = XPM_INVALID_SUBSYSID**:
   - 根因: a72 ELF image_id 用了 0x1c000002, 不在 Versal 标准 subsystem ID list
   - 修复: 改用 `0x1c000000` (PM_SUBSYS_DEFAULT, ALINX 06_ps_hello 同样用法)
5. **A72 boot SError (PC=0x200 vector handler)**:
   - 根因 1: BD 的 axi_noc 缺 6 个 PS NMU port (FPD_CCI×4 + LPD + PMC)
   - 根因 2: BSP 是基于旧 .xsa build 的, mmu_tbl 跟新 NoC routing 不一致
   - 根因 3: ELF link 在 0x40000000 不在 default subsystem 默认可访问范围
   - 修复: apply_config.tcl 加 PS NMU + assign DDR addr; lscript link 到 axi_noc_0_C3_DDR_LOW0 ORIGIN=0x0 (Vitis 自动从新 .xsa 取); ELF entry=0x0 跟 ALINX 一致
6. **Xil_Assert spin 死锁 (PC=Xil_Assert+0x40)**:
   - 修复: main.c 加 `Xil_AssertWait = 0` 让 assertion 不 hang

## A72 当前状态

烧 vd100_with_elf.pdi 后:
- ✅ DONE bit HIGH (PDI 烧成功)
- ✅ A72 进入 ELF entry (PC 不再卡 vector handler 0x200)
- ✅ A72 跑 lwIP 主循环 (stop 时 PC=`sys_arch_unprotect+8` lwIP 内部函数)
- ❌ ping 169.254.111.10 不通, ARP 无响应

## 剩余工作 (需用户物理检查)

**已确诊: 软件 100% OK, 卡在物理 RJ45 link.**

ALINX 文档 [course_s1/Documents/05_ps_hello_CN.rst](C:/_Project/Bishe/VD100/VD100_9_16/demo/course_s1/Documents/05_ps_hello_CN.rst) §7.1.1
说板 PHY 是 **JL2121** (国产), Vitis 默认 lwIP 不识别, ALINX 提供修改版 lwIP. 我手动 patch
BSP `xemacpsif_physpeed.c` 强制 1000Mbps 跳过 PHY-specific auto-neg.

但即使 patch 后, GEM0 Net Status bit 0 (PCS link state) 仍 0:

| 实验 | 烧后 GEM0 Net Status | bit 0 (link) |
|---|---|---|
| 我们 vd100_with_elf.pdi (auto-neg) | 0x00000106 | 0 (DOWN) |
| 我们 vd100_with_elf.pdi (强制 1000Mbps patch) | 0x00000006 | 0 (DOWN) |
| ALINX 06_ps_hello boot.pdi (出货 demo) | 0x00000006 | 0 (DOWN) |

ALINX 出货 demo 也是 link DOWN. **bit 1 + bit 2 = 1 说明 MDIO bus 工作正常 (BSP 能读 PHY 寄存器, PHY 在板上),
但 PCS RGMII 物理层 link state = 0**. 唯一可能是 RJ45 网线物理连接 (PC 网卡 ↔ 板 RJ45 真有电气信号).

通过 [jtag_check_gem.tcl](jtag_check_gem.tcl) dump GEM0 寄存器:

```
GEM0 Net Status @ 0xFF0C0008 = 0x00000106
  bit 0 (PCS link state) = 0  ← link DOWN
  bit 2 (PHY mgmt idle) = 1   (MDIO 工作)
GEM0 Tx Status = 0  (没发过包, ARP 没出去)
GEM0 Rx Status = 0x3 (Rx Q init OK, 但 link 没起没收到包)
```

GEM controller 自己 init OK, 但 PHY 层 link 没建立. 

**用户回来后操作流程**:

### Step 0: 接 RJ45 网线 (核心)

板上有 1 个 RJ45 网口 (GEM0). 接 PC 网卡的以太网3 (用户已配 169.254.111.133).

接好后**板上 RJ45 link LED 必须亮** (一般绿色).

### Step 1: 重烧 PDI (确保板 boot 起来 lwIP server)

```powershell
& 'C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\run_demo_full.ps1'
```

### Step 2: 验证 link + ARP

跑 GEM 状态 dump 确认 link UP (Net Status bit 0 = 1):
```powershell
& 'C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\run_xsct.ps1' `
  'C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\jtag_check_gem.tcl'
```

### Step 3: ping + GUI demo

```powershell
ping 169.254.111.10
& 'C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe' `
  'C:\_Project\FLUX_CNN\host\vd100_pc\resnet11_gui.py'
# GUI: IP=169.254.111.10 Port=5000, 选图 + 连接 + 推理
```

### 物理排查清单

1. **检查 RJ45 link LED 是否亮**:
   - 板上 RJ45 网口本体应该有 link LED (一般 yellow/green): **link 亮 = OK**, **link 灭 = 物理层断**
   - 如果 link LED 灭: 重新插紧 RJ45 网线两头 / 换条网线试 / 换 PC 网卡口试
   - 注意: ALINX VD100 板的 PHY 是 RTL8211F (千兆), 千兆模式可能跟某些老网卡协商失败, 强制百兆试
2. **PC 端 ipconfig 看以太网3**: 应该显示 "已连接", media type 1Gbps 或 100Mbps. 如果是 "媒体已断开" → 板那头没起
3. **link 起来后 ping**: 重跑 `ping 169.254.111.10`. 应该秒回
4. **如果板 LED 亮但 ping 还不通** (less likely): BSP 的 xemacps PHY 检测可能不识别 RTL8211F.
   - 看 `workspace2/vd100_platform/.../bsp/.../libsrc/lwip213_v1_1/src/contrib/ports/xilinx/netif/xemacpsif_physpeed.c`
   - 或者 ALINX 提供的 BSP patch (一般有 RTL8211 detection patch)

## 当前关键文件

| 文件 | 用途 |
|---|---|
| [run_demo_full.ps1](run_demo_full.ps1) | 端到端 PowerShell wrapper, 一键烧 PDI |
| [run_xsct.ps1](run_xsct.ps1) | xsct wrapper, 解决 hw_server cable 问题 |
| [program_pdi_via_vivado.tcl](program_pdi_via_vivado.tcl) | Vivado backend 烧 PDI (失败时自动 dump PROGRAM.LOG) |
| [vd100_with_elf.bif](vd100_with_elf.bif) | bootgen BIF, 嵌 A72 ELF 进 PDI |
| [resnet11_main.c](resnet11_main.c) | A72 baremetal lwIP server (single source) |
| [vd100_pc/resnet11_gui.py](../vd100_pc/resnet11_gui.py) | PC tkinter GUI (推理客户端) |

## 重启 demo flow (重 build 完整)

```powershell
# Step 1: 综合 (1-1.5h, 改 BD config 时跑)
& 'D:\Xilinx\Vivado\2023.2\bin\vivado.bat' -mode batch `
  -source C:\_Project\FLUX_CNN\Syn\vd100_bd\apply_config.tcl

# Step 2: Vitis build (5-10 min, 改 a72 ELF 时跑)
& 'D:\Xilinx\Vitis\2023.2\bin\xsct.bat' `
  C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\create_vitis_app.tcl

# Step 3: bootgen + 烧 (~1 min)
cd C:\_Project\FLUX_CNN\host\vd100_ps_baremetal
& 'D:\Xilinx\Vitis\2023.2\bin\bootgen.bat' -arch versal `
  -image vd100_with_elf.bif -o vd100_with_elf.pdi -w on
& '.\run_demo_full.ps1'
```
