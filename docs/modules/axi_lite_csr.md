# axi_lite_csr

AXI4-Lite Slave → 通用寄存器读写端口的桥接。把 AXI-Lite 的 5 通道（AW / W / B / AR / R）翻译成简单的 `reg_w_en/addr/data/strb` 写脉冲 + `reg_r_addr/data` 组合读端口，喂给后端 `cfg_regs`。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W` | 地址位宽 | 12 |
| `DATA_W` | 数据位宽 | 32 |

## 接口

### AXI-Lite Slave

5 个标准通道：AW（写地址）、W（写数据）、B（写响应）、AR（读地址）、R（读数据）。LEN=0，无 burst，每次握手单拍。

### 后端寄存器 bank 接口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `reg_w_en` | out | 写脉冲（一拍） |
| `reg_w_addr / reg_w_data / reg_w_strb` | out | 写地址 / 数据 / strobe |
| `reg_r_addr` | out | 读地址（组合） |
| `reg_r_data` | in | 读数据（来自 cfg_regs 的组合 mux） |

## 写通道状态机

```
WS_IDLE → WS_ISSUE → WS_RESP → (BREADY) → WS_IDLE
```

WS_IDLE：等 AW 和 W 各自握手到位。AW 和 W 可分先后到达：
- `AWREADY = (ws == WS_IDLE) && !aw_got`
- `WREADY  = (ws == WS_IDLE) && !w_got`
- 接到一个就 latch 一个并 set `aw_got/w_got=1`，避免重复接收
- 当 `aw_got || evt_aw_hs && w_got || evt_w_hs` 都成立 → 进 WS_ISSUE

WS_ISSUE：当拍 `reg_w_en=1`，`reg_w_addr=aw_addr`、`reg_w_data=w_data`、`reg_w_strb=w_strb` 输出给 cfg_regs。下一拍进 WS_RESP。

WS_RESP：拉 `BVALID=1`，等 BREADY 握手成功后回 WS_IDLE。

事件信号：
```
evt_aw_hs      = WS_IDLE && AWVALID && AWREADY
evt_w_hs       = WS_IDLE && WVALID  && WREADY
evt_issue      = (ws == WS_ISSUE)
evt_both_ready = WS_IDLE && (aw_got || evt_aw_hs) && (w_got || evt_w_hs)
evt_b_hs       = WS_RESP && BVALID && BREADY
```

`aw_got / w_got` 是控制路径（决定 handshake 是否再 accept），复位归 0；`evt_issue` 时清零（一次写动作完成）。`aw_addr / w_data / w_strb` 是数据路径 latch，无复位。

## 读通道状态机

```
RS_IDLE → (AR 握手) → RS_DATA → (R 握手) → RS_IDLE
```

RS_IDLE：`ARREADY=1`，等 ARVALID 握手。`reg_r_addr` 直通 `ARADDR`（让 cfg_regs 当拍组合译码出 `reg_r_data`）。AR 握手当拍 latch `ar_addr <= ARADDR`，跳 RS_DATA。

RS_DATA：拉 `RVALID=1`，`RDATA = reg_r_data`，`reg_r_addr` 切到 latched `ar_addr`（保持 cfg_regs 的读输出稳定）。等 RREADY 握手成功后回 RS_IDLE。

注意 RS_IDLE 下 `reg_r_addr=ARADDR`，cfg_regs 的组合读对 ARADDR 直通，这样 AR 握手当拍 reg_r_data 已经是有效值；RS_DATA 拉 RVALID 时 RDATA 立即可用。

## 数据通路与控制路径

- 数据路径无复位：aw_addr、w_data、w_strb、ar_addr
- 控制路径同步复位：ws、rs、aw_got、w_got

## 在 core_top 中的位置

实例 `u_csr_bridge`：
- AXI-Lite slave 端：core_top 的对外 AXI-Lite 接口（接 host / TB 的 AXI-Lite master）
- 后端：`u_cfg.reg_*` 端口
