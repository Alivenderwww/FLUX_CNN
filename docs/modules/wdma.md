# wdma（Weight DMA）

DDR → WB SRAM 的 read DMA，含 128→2048 打包。AXI4 read master，每 16 个 128-bit beat 拼成 1 个 2048-bit WB SRAM 字写入。

WB 字内 128-bit 子字布局（与 gen_isa_test.py 写出的 wb.txt 字节序一致）：

```
bits [127:0]      = beat 0   (col 0, PE 0..15)
bits [255:128]    = beat 1   (col 1)
...
bits [2047:1920]  = beat 15  (col 15)
```

beat 0 在 LSB，beat 15 在 MSB。打包用 2048-bit 移位寄存器：每 r_fire 右移 128 bit、新 beat 进 MSB。第 16 beat 当拍组合 `{M_RDATA, word_buffer[2047:128]}` 直接写 WB SRAM，省一拍。

## 参数

| 参数 | 含义 | 默认 |
| --- | --- | ---: |
| `ADDR_W / DATA_W / M_ID` | AXI 参数 | 32 / 128 / 2 |
| `WB_DATA_W` | WB 字宽 | 2048 |
| `SRAM_ADDR_W` | WB 地址宽 | 13 |
| `LEN_W` | byte_len 宽 | 24 |

## 接口

### Control / Descriptor

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `start / done / busy` | in/out/out | |
| `src_base` | in | DDR 起始地址（4KB 对齐） |
| `byte_len` | in | 字节长度，必须是 256 字节（= WB_DATA_W/8）的整数倍 |

### AXI4 M

只读（AR / R），写通道全 tie 0。

### WB SRAM 写端口

| 信号 | 方向 | 含义 |
| --- | --- | --- |
| `wb_we / wb_waddr / wb_wdata[2048]` | out | WB 写 |

## 状态机

```
S_IDLE → S_AR → S_R → (more: S_AR；else: S_DONE) → (start 重入)
```

- S_IDLE：等 start
- S_AR：发 ARVALID。ar_fire 后 → S_R
- S_R：接收一个 burst（最多 256 beat = 16 个 WB 字）。M_RLAST 拍判 beats_remaining：还有数据就回 S_AR 发下一个 burst，否则 → S_DONE
- S_DONE：r_done sticky

## AXI 通道

```
M_ARADDR  = cur_addr
M_ARLEN   = beats_to_issue - 1            (max 256)
M_ARBURST = INCR
M_ARVALID = (state == S_AR)

M_RREADY  = (state == S_R)
```

`beats_to_issue = min(beats_remaining, 256)`：剩余不足 256 时一次发完，超过则分批。

## 打包逻辑

```
wdata_assembled = {M_RDATA, word_buffer[WB_DATA_W-1:DATA_W]}
                  // 当前 beat 顶进 MSB, 旧 buffer 下移 128 bit

complete_word = r_fire && (beat_in_word == 15)

word_buffer ← wdata_assembled  on r_fire        (移位)
beat_in_word ← 0  on (start || complete_word); +1  on r_fire
wb_wr_ptr ← +1   on complete_word
wb_we = complete_word
wb_wdata = wdata_assembled
```

## 跨 burst 一致性

AR 跨 burst 时（前一 burst 的 R 已完，下一 burst AR 还没 fire）r_fire=0，`beat_in_word / word_buffer` 自然保持。新 burst 第一 r_fire 接着上一次的 phase 继续。

## 数据路径与控制路径

- 数据路径无复位：cur_addr、beats_remaining、word_buffer、beat_in_word、wb_wr_ptr
- 控制路径同步复位：state、r_done

## 在 core_top 中的位置

实例 `u_wdma`：
- AXI4 M[1]
- start 来自 `u_sequencer.start_wdma_pulse`（仅 is_first，整 layer 一次）
- src_base / byte_len 来自 `u_cfg.wdma_*`
- WB SRAM 写端口与 axi_lite_csr 预填路径在 core_top 用 mux 合并
- done → sequencer 的 PRELOAD / S_END 等待条件
