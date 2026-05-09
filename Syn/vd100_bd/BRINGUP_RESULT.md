# VD100 BD Full Flow 实测结果

(等 Vivado batch 跑完 / Vitis 部署后填数据)

## Step 1: Vivado synth + impl + bitstream (full_flow.tcl)

### Post-Synth Utilization
**Resource | Used | Available | %**
- LUT       | TBD | 150,272 | TBD
- FF        | TBD | 300,544 | TBD
- DSP       | TBD | 464     | TBD
- BRAM      | TBD | 5.4 Mb  | TBD
- URAM      | TBD | 96      | TBD

### Post-Impl (Routed) — 板级真实数据
**Resource | Used | Available | %**
- LUT       | TBD | 150,272 | TBD
- FF        | TBD | 300,544 | TBD
- DSP       | TBD | 464     | TBD
- BRAM      | TBD | 5.4 Mb  | TBD
- URAM      | TBD | 96      | TBD
- IO        | TBD | 238     | TBD (含 DDR4 + sys_clk pin)

### Timing
- Clock period target: 10.0 ns (100 MHz)
- WNS: TBD ns
- Fmax: TBD MHz
- TNS: TBD ns

### Bitstream
- Path: `Syn/vd100_bd/output/vd100_resnet11.runs/impl_1/design_1_wrapper.bit`
- 大小: TBD MB

### XSA (Vitis 用)
- Path: `Syn/vd100_bd/output/vd100_resnet11.xsa`
- 大小: TBD MB

---

## Step 2: Vitis app build

### Platform
- Created from `vd100_resnet11.xsa`
- Versal AI Edge VE2302
- Domain: standalone_psv_cortexa72_0
- BSP libs: lwip213

### Application
- Template: lwIP Echo Server
- Modified: src/echo.c → resnet11_main.c
- ELF: `host/vd100_ps_baremetal/workspace/resnet11_app/Debug/resnet11_app.elf`

---

## Step 3: 板上 bring-up

### 板物理连接
- [ ] 12V DC 电源
- [ ] USB JTAG (PC USB)
- [ ] RJ45 ETH0 (PS) → PC 直连或同一 LAN
- [ ] 上电

### PC 网络配置
- 静态 IP: 192.168.1.20 / 255.255.255.0
- ping 192.168.1.10 → 通?

### Vitis Run
- [ ] 右键 resnet11_app → Run As → Launch Hardware (Single App Debug)
- [ ] 自动: 下 bit → 烧 elf → 启动 → console 打 "Listen on 5000"

### PC client 验证
```cmd
python C:\_Project\FLUX_CNN\host\vd100_pc\resnet11_client.py ^
    --image C:\_Project\FLUX_CNN\toolchain\models\images\resnet11_test\gradient.png ^
    --vd100-ip 192.168.1.10 ^
    --port 5000
```

### 实测性能
- HW cycles 实测: TBD cy
- HW 时间 @100MHz: TBD ms
- 端到端 RTT: TBD ms
- 吞吐: TBD FPS

---

## 故障排查 log
(bring-up 时遇到问题 + 解决记录)

(待填)
