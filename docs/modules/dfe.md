# dfe（Descriptor Fetch Engine）

从 DDR 拉 N 条 256-bit descriptor 写入片内 desc_fifo。AXI4 read master 通道；一次性发整个 burst 的 AR，R 通道按 fifo_full 反压 r_ready。每条 descriptor = 32 byte = 2 个 128-bit AXI beat（little-endian：第一 beat 是低 16 byte，第二 beat 是高 16 byte）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W` | AXI 地址宽 | 32 |
| `DATA_W` | AXI 数据宽 | 128 |
| `M_ID` | AR/R ID 位宽 | 2 |
| `FIFO_W` | desc_fifo 项位宽 | 256 |

## 接口

### Control

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start / done / busy` | in/out/out | 启停握手 |
| `desc_list_base` | in | DDR 中 descriptor list 起始地址 |
| `desc_count` | in | descriptor 条数（≤128，单次 AR burst 最多 256 beat） |

### AXI4 M（仅 AR / R 通道，AW/W/B 在 core_top tie 0）

标准 AXI4 read master 端口。

### desc_fifo 写端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `fifo_wdata[256] / fifo_we / fifo_full` | out/out/in | 写一条 descriptor |

## 状态机

```
S_IDLE → S_AR → S_R → S_DONE → (start 重入)
```

- S_IDLE：等 start
- S_AR：拉 ARVALID，等 M_ARREADY 后跳 S_R
- S_R：接收 R 通道 burst，每 2 拍组装一条 descriptor 写 FIFO。最后一拍 (M_RLAST) → S_DONE
- S_DONE：r_done sticky，等下一 case 的 start

## AXI AR

```
M_ARID    = 0
M_ARADDR  = r_base                   (= desc_list_base, start 拍 latch)
M_ARLEN   = r_beats_total - 1        (= 2 × desc_count - 1)
M_ARBURST = INCR (2'b01)
M_ARVALID = (state == S_AR)
```

一次性发完整个 burst 的 AR，等 ARREADY 即过。

## AXI R 通道 + 双 beat 拼接

```
M_RREADY  = (state == S_R) && (r_phase == 0 || !fifo_full)
r_fire    = M_RVALID && M_RREADY

phase=0 (低半): 无条件 ready, latch 当拍 M_RDATA 到 r_low_latch
phase=1 (高半): ready 受 fifo_full 反压, fire 时 fifo_we=1 写 {M_RDATA, r_low_latch}
```

`r_phase` 在 r_fire 当拍 toggle。phase=0 时 fifo_we=0 不写 FIFO（只 latch 半条），phase=1 时把 latched 低半 + 当拍高半拼成 256-bit 一起写。

## 反压

phase=0 永远 r_ready=1（FIFO 没用，单纯 latch 寄存器）。phase=1 时 r_ready = !fifo_full：FIFO 满时拉 r_ready=0 反压 AXI 来源。AR 一次发完，R 通道靠 ready 节流。

## 寄存器

数据路径（start 时初始化，无复位）：
- `r_base`：AXI 起始地址
- `r_count`：descriptor 条数
- `r_beats_total = desc_count × 2`
- `r_beats_rcvd`：累计接收 beat 数（实际只在仿真观察用，FSM 用 M_RLAST 判完）
- `r_phase`：0 / 1
- `r_low_latch`：phase=0 latch 的低 128 bit

控制路径（同步复位）：
- `state`、`r_done`

`done = r_done && !start`：sticky done，多 case 重入时 start 当拍立即清 0。

## 在 core_top 中的位置

实例 `u_dfe`：
- AXI4 M[3]（mux 的最后一路，AW/W/B tie 0，只用 AR/R）
- start 来自 `u_cfg.start_dfe_pulse`
- desc_list_base / desc_count 来自 cfg_regs
- FIFO 写端口 → `u_desc_fifo`
- done → `u_cfg.dfe_done`（host 通过 STATUS 读）
