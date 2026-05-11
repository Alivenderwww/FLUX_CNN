# 实验 14: VD100 板上 ResNet11 demo (待板级 bring-up 后填数据)

**日期**: 2026-05-09 (开始)
**板**: ALINX VD100 (Versal AI Edge VE2302)
**工具链**: Vivado 2023.2 + Vitis 2023.2 (Baremetal lwIP)
**状态**: bringup 进行中

## 板级配置

| 项 | 配置 |
|---|---|
| Part | xcve2302-sfva784-1LP-e-s |
| PS clock | A72 1.5 GHz |
| PL clock | 100 MHz (clk_wizard 200 MHz → 100 MHz) |
| DDR4 | 4 GB, 3200 Mbps (NoC DDRMC) |
| Host 接口 | PS GEM ETH0 千兆 (RJ45) |
| Boot 模式 | JTAG (Vitis Run) for demo, SD/QSPI for production |

## Vivado P&R 数据 (待填)

| 指标 | OOC (synth) | **Routed (BD 完整)** |
|---|---:|---:|
| Total LUT | 128,375 | TBD |
| Total FF | 41,374 | TBD |
| RAMB36 | 81 | TBD |
| URAM | 64 | TBD |
| DSP | 144 | TBD |
| WNS @ 100 MHz | +4.514 ns | TBD |
| **Fmax** | 182.3 MHz | TBD |
| 含 PS BD overhead 估 | – | +5K LUT (NoC 配置) |

## 板上实测性能 (待填)

| 指标 | sim 估 | **板上实测** |
|---|---:|---:|
| 单图推理 PL cy | ~230,000 | TBD |
| HW 时间 @ 100 MHz | 2.3 ms | TBD |
| **PC ↔ board RTT** | – | TBD ms (含千兆网+PS 处理) |
| **演示吞吐** | – | TBD FPS |

## bringup 验证 checklist

- [ ] Vivado GUI 打开 BD 工程, 修 axi_noc 端口
- [ ] BD validate 0 errors
- [ ] 综合 + 实现 + bitstream 生成
- [ ] Export .xsa (含 bitstream)
- [ ] Vitis 创建 baremetal app (lwIP echo template)
- [ ] resnet11_main.c 替换 echo.c
- [ ] 板子 USB JTAG 接 PC
- [ ] 板子千兆网接 PC (静态 IP 192.168.1.10/20)
- [ ] Vitis Run baremetal → console 打 "Listen on 5000"
- [ ] PC client 连接成功
- [ ] 推理返回 OFM (522 byte)
- [ ] HW cy 跟 sim 大致一致 (±20%, 因为 DDR4 真实 latency)
- [ ] 多图连续推理稳定 (10+ 张不崩)

## 故障排查 log (bringup 时记录)

(待填)

## 视频/截图

(待填: bringup 录屏 + 板子 demo 视频)

## 论文写作 talking points

bringup 跑通后, paper 加这一段:
1. 实测 vs sim cy 对比 (验证 RTL ↔ 板级一致性)
2. 端到端 RTT 拆解 (网络 / PS handshake / PL 计算)
3. 板上 routed 实际资源占用 (跟 OOC 对比)
4. demo 视频/截图作为 supplementary material
