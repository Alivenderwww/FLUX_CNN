# cfg_regs

中心配置寄存器组 + DMA 描述符寄存器。通过 AXI-Lite 解码后的端口接受外部读写，向 core 内部各模块直驱组合输出。所有 cfg 都是当拍写当拍可读，下一拍下游模块看到新值。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W` | AXI-Lite 地址位宽 | 12 |
| `DATA_W` | AXI-Lite 数据位宽 | 32 |
| `CORE_ADDR_W` | core 内部 SRAM 地址宽 | 20 |

## 接口

### AXI-Lite decoded 端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `reg_w_en / reg_w_addr / reg_w_data / reg_w_strb` | in | 写端口；strb 当前不解析（按 32-bit 全字写） |
| `reg_r_addr / reg_r_data` | in / out | 读端口（组合输出，跟 raddr 同拍变化） |

### Status 输入

来自 core 内部的状态汇总：`core_done / *_busy / *_done / dfe_busy / dfe_done / layer_busy / layer_done`，由 `cfg_regs` 拼成 STATUS word（地址 0x004）供 host 读取。

### CTRL 输出（pulse）

| 信号 | 含义 |
| --- | --- |
| `start_dfe_pulse` | host 写 CTRL[4]=1 当拍发一拍，触发 DFE 拉 descriptor |
| `start_layer_pulse` | host 写 CTRL[5]=1 当拍发一拍，触发 Sequencer 启动 layer |

### cfg 输出

输出接到 line_buffer / wgt_buffer / parf_accum / psum_reshape / ofb_writer / sequencer / DMA 等下游。每个寄存器单独走 always_ff 写入 + assign 直通输出。

## 寄存器映射

按 4 字节对齐。地址段：

| 段 | 地址 | 含义 |
| --- | --- | --- |
| `0x000` CTRL | bit[4]=start_dfe，bit[5]=start_layer | 写 1 当拍触发 |
| `0x004` STATUS (RO) | bit[0..11] 拼接各模块 busy/done | 见 always_comb 中的 `status_word` |
| `0x100..0x13C` | layer 几何参数 | H_OUT/W_OUT/W_IN/K/KY/STRIDE/CIN_SLICES/COUT_SLICES/TILE_W/NUM_TILES/LAST_VALID_W/TOTAL_WRF/KK/ROUNDS_PER_CINS/ROUND_LEN_LAST |
| `0x13C..0x15F` | core 物理 base / step | IFB_BASE/WB_BASE/OFB_BASE/IFB_ROW_STEP/WB_COUT_STEP/TILE_IN_STEP |
| `0x160..0x164` | SDP（旧） | SDP_SHIFT, SDP_RELU_EN |
| `0x168..0x17C` | streaming / ring / DMA mode | H_IN_TOTAL/IFB_STRIP_ROWS/OFB_STRIP_ROWS/DDR_IFM_ROW_STRIDE/DDR_OFM_ROW_STRIDE/DMA_MODE |
| `0x180..0x184` | descriptor list | DESC_LIST_BASE, DESC_COUNT |
| `0x188..0x198` | SDP（新） | SDP_MULT/ZP_OUT/CLIP_MIN/CLIP_MAX/ROUND_EN |
| `0x1A0..0x1B4` | NHWC 派生 | IFB_RING_WORDS/OFB_ROW_WORDS/OFB_RING_WORDS/IFB_ISS_STEP/IFB_KY_STEP/TILE_PIX_STEP |
| `0x1B8` | ARF_REUSE_EN | bit[0]=1 时 stride=1&&K>1 启用滑窗复用 |
| `0x1BC..0x1C4` | Kx-fold | FOLD_COUT_ORIG/FOLD_COUT_GROUPS/FOLD_COL_SHIFT |
| `0x200..0x224` | DMA 参数 | IDMA_SRC_BASE/BYTE_LEN, WDMA_*, ODMA_* |

## 写入逻辑

所有 cfg 寄存器写在同一个 `always_ff @(posedge clk)` 里，按 `reg_w_addr` 做 case 分发：

```
if (reg_w_en) case (reg_w_addr)
    ADDR_X: r_x <= reg_w_data[..];
    ...
    default: ;
endcase
```

未命中地址时所有寄存器隐式保持。`reg_w_strb` 暂不解析（写按 32-bit 全字粒度）。

## start pulse 生成

```
start_dfe_pulse   = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[4]
start_layer_pulse = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[5]
```

只在写 CTRL 当拍发一拍。CTRL 寄存器本身不存储（写完即消，bit[4]/bit[5] 只触发 pulse）。

## DMA_MODE 复位

`r_dma_mode_ctrl` 是控制路径，复位归 0（虽然当前硬件恒 streaming 不读这个寄存器，但保留以兼容历史 cfg 文件不报错）。

## 数据通路与控制路径

- 数据路径无复位：所有几何 cfg / SDP cfg / DMA base / step / fold 参数
- 控制路径同步复位：`r_dma_mode_ctrl`

数据路径无复位的逻辑前提：cfg 在 `start_*_pulse` 之前必须先被 host 写入有效值（在 layer 启动前完成 cfg 装载），上电 X 不会被下游模块在启动前观测到。

## 读取 mux

`reg_r_data` 在 `always_comb` 里按 `reg_r_addr` 分发。每个寄存器读出时按位宽 padding 高位 0（unsigned）或符号扩展（signed 字段如 `sdp_zp_out / sdp_clip_min / sdp_clip_max`）。

`STATUS` 读出由 `status_word` 组合拼接：

```
status_word = {20'd0,
               layer_done, layer_busy, dfe_done, dfe_busy,
               odma_done,  wdma_done,  idma_done,
               odma_busy,  wdma_busy,  idma_busy,
               layer_busy, core_done}
```

## 在 core_top 中的位置

实例 `u_cfg`：
- 上游：`u_csr_bridge`（`axi_lite_csr` 桥接的 AXI-Lite slave 端口）解码后的 `reg_w_*` / `reg_r_*` 信号
- 下游：cfg 总线分散到所有 core 内部模块（line_buffer / wgt_buffer / parf_accum / psum_reshape / ofb_writer / sequencer / dfe / 三个 DMA）
- start pulse → sequencer 内部 FSM 触发
