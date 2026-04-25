# core_top

加速器顶层。把 cfg_regs / sequencer / desc_fifo / 三块 SRAM / 5 个数据通路模块 / 4 个 DMA / axi_m_mux / axi_lite_csr 桥联起来。对外暴露 1 路 AXI4-Lite Slave（host 配置）+ 1 路 AXI4 Master（DMA 出核访问 DDR）+ TB 后门 SRAM 口。

## 模块层次

```
core_top
├── u_csr_bridge         (axi_lite_csr)        AXI-Lite slave 桥
├── u_cfg                (cfg_regs)            中心寄存器组
├── u_desc_fifo          (desc_fifo)           descriptor FIFO 32×256
├── u_sequencer          (sequencer)           layer/strip 控制器
├── u_ifb / u_wb / u_ofb (sram_model ×3)        3 块 SRAM
├── u_line_buffer        (line_buffer)         act 供给
├── u_wgt_buffer         (wgt_buffer)          weight + bias 供给
├── u_mac_array          (mac_array)           16×16 PE
├── u_parf_accum         (parf_accum)          PSUM 累加
├── u_psum_reshape       (psum_reshape)        Kx-fold cout 归约
├── u_ofb_writer         (ofb_writer)          SDP + OFB 写
├── u_dfe                (dfe)                 DDR → desc_fifo
├── u_idma / u_wdma / u_odma (DMA ×3)
└── u_axi_mux            (axi_m_mux)           4→1 AXI master 聚合
```

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `NUM_COL / NUM_PE` | PE 阵列 | 16 / 16 |
| `DATA_WIDTH / PSUM_WIDTH` | 数据/psum 位宽 | 8 / 32 |
| `WRF_DEPTH / ARF_DEPTH / PARF_DEPTH` | 各 RF 深度 | 32 / 32 / 32 |
| `SRAM_DEPTH` | IFB / WB / OFB SRAM 深度 | 8192 |
| `CSR_ADDR_W / CSR_DATA_W` | AXI-Lite 参数 | 12 / 32 |
| `BUS_ADDR_W / BUS_DATA_W` | AXI4 M 参数 | 32 / 128 |
| `AXI_M_ID / AXI_M_WIDTH` | per-DMA ID 宽 / log2(masters) | 2 / 2 |
| `DMA_LEN_W` | DMA byte_len 宽 | 24 |

派生量：`ADDR_W=20`（core 内部 SRAM 地址）、`IFB_WIDTH=128`、`WB_WIDTH=2048`、`OFB_WIDTH=128`、`AW=$clog2(SRAM_DEPTH)=13`。

## 接口

### AXI-Lite Slave（host 配置通道）

标准 5 通道（AW/W/B/AR/R），位宽 12/32。host 通过这个接口写 cfg_regs、读 STATUS、触发 DFE 和 layer。

### AXI4 Master（DMA 出核）

由 axi_m_mux 聚合 IDMA / WDMA / ODMA / DFE 四个 master 后的统一对外接口。位宽 32/128，ID 宽 4（M_ID=2 + M_WIDTH=2）。

### TB 后门 SRAM 端口

| 信号 | 方向 | 用途 |
| --- | --- | --- |
| `ifb_we_ext / ifb_waddr_ext / ifb_wdata_ext` | in | TB `$readmemh` 风格直写 IFB（DMA 路径不接入时） |
| `wb_we_ext / wb_waddr_ext / wb_wdata_ext` | in | 同上写 WB |
| `ofb_re_ext / ofb_raddr_ext / ofb_rdata_ext` | in/in/out | 直读 OFB |

Mux 规则：DMA 优先，DMA 不发时让 ext 走。`ifb_we_mux = idma_ifb_we | ifb_we_ext`，addr/wdata 按 idma_ifb_we 选择。

### Debug

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `psum_out_vec[NUM_COL×32]` | out | mac_array 实时 psum（debug） |
| `done` | out | 顶层完成 = ofb_writer.done |

## 顶层数据通路

```
host AXI-Lite ─→ csr_bridge ─→ cfg ─┬→ line_buffer ─act─→ mac_array ─psum─→ parf ─→ reshape ─→ ofb_writer ─→ OFB SRAM
                                    │   ↑                    ↑              ↑
                                    │   │ act/wgt/psum        │ old_psum     │
                                    │   wgt_buffer ─wgt──────┘              │
                                    │   ↑          (bias)──────────────────┤
                                    │   │                                    │
                                    │   WB SRAM ←─ WDMA ←──┐                │
                                    │                       │                │
                                    │   IFB SRAM ←─ IDMA ←──┤  axi_m_mux ←──┘
                                    │                       │       ↑
                                    │   desc_fifo ←─ DFE ←──┘       │
                                    └→ sequencer (按 descriptor 分配启停 + strip cfg)
                                                                    │
                                                            外部 AXI4 M (DDR)
```

## 启停时序

1. host 写 cfg_regs（layer 几何 / DMA base / SDP 参数 / fold 参数 / descriptor list base/count）
2. host 写 CTRL[4]=1 → `start_dfe_pulse` → `u_dfe` 启动，从 DDR 拉所有 descriptor 到 `u_desc_fifo`
3. host 等 STATUS.dfe_done=1
4. host 写 CTRL[5]=1 → `start_layer_pulse` → `u_sequencer` S_IDLE→S_FETCH，开始 pop descriptor
5. 每条 CONV descriptor：
   - is_first：sequencer 走 S_PRELOAD → 发 `start_wdma_pulse`，等 `wdma_done` → S_DISPATCH
   - 否则：直接 S_DISPATCH
6. S_DISPATCH 当拍发 `start_core_pulse / start_idma_pulse / start_odma_pulse`，is_first 时还发 `start_wgt_pulse`
7. core 流水自驱跑完 strip：line_buffer、wgt_buffer、mac_array、parf_accum、ofb_writer 用 valid-ready 同步推进；IDMA / ODMA 与 LB / OW 用 row credit 反压
8. core_strip_done && idma_strip_done && odma_strip_done 全 1 后 sequencer 回 S_FETCH 取下一条
9. END descriptor → S_END → r_layer_done=1，host 看到 STATUS.layer_done=1

## SRAM 端口 mux

```
ifb_we_mux    = idma_ifb_we | ifb_we_ext              IFB 写: IDMA 优先
ifb_waddr_mux = idma_ifb_we ? idma_ifb_waddr : ifb_waddr_ext
ifb_wdata_mux = 同上

wb_we_mux     = wdma_wb_we | wb_we_ext                 WB 写: WDMA 优先
wb_waddr_mux  = ...

ofb_re_mux    = odma_ofb_re | ofb_re_ext               OFB 读: ODMA 优先
ofb_raddr_mux = ...
ofb_rdata_shared → 同时给 odma_ofb_rdata 和 ofb_rdata_ext
```

DMA 和 ext 同拍触发是不允许的（TB 契约）；不会同时高。

## 地址叠加（per-strip DMA）

```
eff_idma_src_base = cfg.idma_src_base + seq.strip_ifb_ddr_offset
eff_odma_dst_base = cfg.odma_dst_base + seq.strip_ofb_ddr_offset
eff_idma_byte_len = seq.strip_ifb_byte_len
eff_odma_byte_len = seq.strip_ofb_byte_len
```

## IFB pad offset 预扣

```
pad_offset    = pad_top × cfg_ifb_ky_step + pad_left × cfg_cin_slices
eff_ifb_base  = cfg_ifb_base + cfg_ifb_ring_words - pad_offset
```

`eff_ifb_base` 喂给 `u_line_buffer.cfg_ifb_base`。让 line_buffer 按虚拟坐标累加 ptr_*_base 时，物理 SRAM 地址在 (ky=pad_top, kx=pad_left) 那拍自然落到真实图像 (0,0)。`+ cfg_ifb_ring_words` 是为了让减法落在 [0, 2×ring) 内，line_buffer 内部 wrap_addr 单次 mod 即可。

## DMA AXI master 索引

| index | master | 端口情况 |
| ---: | --- | --- |
| M[0] | IDMA | 完整 5 通道（AW/W/B 实际 tie 0，因为只读） |
| M[1] | WDMA | 完整 5 通道（同上） |
| M[2] | ODMA | 完整 5 通道（AR/R tie 0，只写） |
| M[3] | DFE | AR/R 通道（AW/W/B 在 core_top tie 0） |

axi_m_mux 仲裁规则：低 index 优先（IDMA > WDMA > ODMA > DFE）。

## 状态汇总

`u_cfg` 通过 STATUS 寄存器（地址 0x004）汇总：
- core_done / layer_busy / layer_done
- idma_busy/done / wdma_busy/done / odma_busy/done
- dfe_busy / dfe_done

供 host 轮询。
