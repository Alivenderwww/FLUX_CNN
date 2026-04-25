# odma（Output DMA）

OFB SRAM → DDR 的 write DMA。AXI4 write master，按 yout 行 streaming 把 OFB ring 里 ofb_writer 已写完的 OFM 行搬出到 DDR。

数据通路：OFB 内是 NHWC 布局（每 yout 写 W_OUT × cout_slices 个 word，cs 段连续），ODMA 按 (yout, x, cs) 顺序读 OFB 拼回 DDR 的"每 cs 段一行"layout（DDR 布局假设是连续的 yout × cs × x，每 yout 行的字节步长 = `cfg_ddr_ofm_row_stride`）。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W / DATA_W / M_ID` | AXI 参数 | 32 / 128 / 2 |
| `SRAM_ADDR_W` | OFB 地址宽 | 13 |
| `LEN_W` | byte_len 宽（保留字段） | 24 |

## 接口

### Control / Descriptor

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start / done / busy` | in/out/out | |
| `dst_base` | in | DDR 起始地址 |
| `byte_len` | in | 保留字段（streaming 不用） |

### Streaming cfg

| 信号 | 含义 |
| --- | --- |
| `cfg_h_out_total` | OFM 总行数 |
| `cfg_w_out` | 每行像素数 |
| `cfg_cout_slices` | 每像素 cs 段数 |
| `cfg_ofb_row_words` | 每 yout 在 OFB 占的 word 数 = W_OUT × cout_slices |
| `cfg_ddr_ofm_row_stride` | DDR 跨 yout 字节步长 |
| `cfg_ofb_ring_words` | OFB ring wrap 模数 |
| `row_done_pulse` | 来自 ofb_writer，每完成 1 行 OFM 发一拍 |
| `rows_drained` | 输出，给 ofb_writer 反压 |

### AXI4 M

只用 AW / W / B（写通道），AR / R 全 tie 0。

### OFB SRAM 读端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `ofb_re / ofb_raddr / ofb_rdata[128]` | out/out/in | OFB 读 |

## 状态机

```
S_IDLE → start → S_AW_WAIT
S_AW_WAIT → has_work → S_AW
S_AW      → aw_fire → S_PREFETCH (1 拍) → S_W
S_W       → w_last_fire → S_B
S_B       → b_fire:
              all_done       → S_DONE
              row_complete   → S_AW_WAIT
              else           → S_AW (同 yout 内多 burst)
```

- S_AW_WAIT：等 `streaming_has_work`（攒了一行未 drain）后进 S_AW
- S_AW：发 AWVALID
- S_PREFETCH：1 拍 ofb_re 预取（让 ofb_rdata 在 S_W 第一拍就可用）
- S_W：每 w_fire 推一个 beat，最后一拍 M_WLAST
- S_B：等 BVALID 完成本 burst

每个 yout 拆成 ceil(cfg_ofb_row_words / 256) 个 burst，行末 b_fire 推进 row 计数和 cur_addr。

## AXI 通道

```
M_AWADDR  = cur_addr
M_AWLEN   = beats_to_issue - 1     (max 256)
M_AWBURST = INCR
M_AWVALID = (state == S_AW)

M_WDATA  = ofb_rdata
M_WSTRB  = '1
M_WVALID = (state == S_W)
M_WLAST  = (state == S_W) && (burst_sent == burst_beats - 1)

M_BREADY = (state == S_B)
```

## OFB 读地址生成（NHWC gather）

```
rd_ptr = yout_base + cs_offset + x_rd_cnt
       (cs_offset = cs_rd_cnt × cfg_w_out)
```

按 `for x: for cs: w_fire` 的顺序扫描：cs_rd_cnt 内层每 w_fire +1（到 cout_slices-1 wrap），x_rd_cnt 在 cs_rd_is_last 拍 +1。完成一个 yout（x_rd_is_last && cs_rd_is_last）时 yout_base 推进到下一行（在 OFB ring 里 wrap）。

ofb_re 在 S_PREFETCH 和 S_W 拉高（保持 ofb_rdata 流入 W 通道）。

## 行级反压

```
streaming_has_work       = (r_rows_produced != r_rows_drained)
r_rows_produced          += 1   on row_done_pulse  (来自 ofb_writer)
r_rows_drained           += 1   on streaming_row_complete_b
streaming_row_complete_b = b_fire && (row_beats_left == 0)
```

ofb_writer 看 `rows_drained`（= r_rows_drained）做 ring 反压：写到 strip_rows 行未被 ODMA 搬走时停。

## DDR 地址推进

```
cur_addr: start 时 = dst_base
          aw_fire 时 += beats_to_issue × 16  (本 burst 内推进)
          row_complete 时 = row_base_addr + cfg_ddr_ofm_row_stride

row_base_addr: start 时 = dst_base
               row_complete 时 += cfg_ddr_ofm_row_stride
```

cur_addr 在 yout 内跨 burst 沿 word 步长走，行末跳到下一 yout 的 base。

## 数据通路与控制路径

- 数据路径无复位：cur_addr、row_base_addr、row_beats_left、rd_ptr、x_rd_cnt、cs_rd_cnt、yout_base、cs_offset、burst_beats、burst_sent
- 控制路径同步复位：state、r_rows_produced、r_rows_drained、r_done

## 在 core_top 中的位置

实例 `u_odma`：
- AXI4 M[2]
- start 来自 `u_sequencer.start_odma_pulse`（每 strip）
- dst_base 来自 cfg + sequencer.strip_ofb_ddr_offset 叠加
- OFB SRAM 读端口与 axi_lite_csr 的预填读路径在 core_top 用 mux 合并
- row_done_pulse / rows_drained 与 `u_ofb_writer` 双向交互
