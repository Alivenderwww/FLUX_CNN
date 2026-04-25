# 编译器侧切片处理

`gen_isa_test.py` 调 `hw_files.derive_layer_cfg` 算出切片数和派生 step；`hw_files.write_ifb / write_wb / write_expected_ofm` 把 IFM / 权重 / OFM 按切片顺序写到 `.txt` 文件，TB 通过 AXI 后门或 IDMA / WDMA 灌进 SRAM。

## 切片数派生

`derive_layer_cfg` 顶部的两行：

```python
cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
```

`HW_PE = HW_COL = 16` 是物理阵列尺寸。Cin 不被 16 整除时末片实际有效通道数 `local_cin = min(16, Cin - cins × 16)`，写文件时只写有效通道，剩下的 slot 在 word 里保持 0。

## IFM 字节布局

`write_ifb` 按 NHWC 顺序逐 word 写出，外层 y、中间层 x、内层 cin slice：

```python
for y in [0, H_IN):
    for x in [0, W_IN):
        for cins in [0, cin_slices):
            local_cin = min(16, NUM_CIN - cins × 16)
            word = pack(ifm[y, x, cins×16 : cins×16 + local_cin], 8 bit each)
            写 word (128 bit, 低位放 cin_local=0)
```

每行总字数 = `W_IN × cin_slices`。IFB SRAM 容量 = `H_IN × W_IN × cin_slices`（整图装得下时）或 `strip_rows × W_IN × cin_slices`（streaming）。

DDR 端布局相同。每 16 个 cin 通道占 1 个 128 bit word，cin_slice 间相邻；cin_slice 内部 cin_local=0..15 在 word 内由低到高。这种"通道在最内层"的安排让一个 word 一次进 PE 阵列的 16 行，正好填满 cin 维度。

## 权重字节布局

`write_wb` 输出顺序：bias 前缀 + (cs, cins, ky, kx) 嵌套权重。

bias 前缀，每 cs 一个 word（2048 bit）：低 16×32 bit 放 16 个 int32 bias，cout_local=0..15 由低到高。

```python
for cs in [0, cout_slices):
    word = pack(bias[cs×16 : cs×16+local_cout], 32 bit each)
    写 word (低位)
```

权重区，按 (cs, cins, ky, kx) 嵌套，每 (cs, cins, ky, kx) 一个 word：

```python
for cs in [0, cout_slices):                     # 外层 cs
    for cins in [0, cin_slices):                 # 内层 cins
        for ky in [0, KY):
            for kx in [0, K):
                word = 0
                for cout_local in [0, local_cout):
                    cout = cs × 16 + cout_local
                    cv = pack(W[ky, kx, cout, cins×16 : cins×16+local_cin], 8 bit)
                    word |= cv << (cout_local × 16 × 8)   # 每 cout_local 占 128 bit
                写 word
```

word 内布局：`bits [128×c+127 : 128×c]` 放 `cout = cs × 16 + c` 这一列的 16 个 cin 权重（cin_local=0..15 由低到高 8 bit 一片）。

总字数：`cout_slices` 个 bias word + `cout_slices × cin_slices × K × KY` 个权重 word。

加载顺序设计成 (cs 外, cins 内) 是为了让 `wgt_buffer` 在每个 (cs, yout, tile) 段处理完之后，能从同一个 cs 区域里挑下一组 cins 权重，cs 切换时跨 `WB_COUT_STEP = K² × cin_slices` 字。

## OFM 字节布局

`write_expected_ofm` 同样按 NHWC：

```python
for yout in [0, H_OUT):
    for px in [0, W_OUT):
        for cs in [0, cout_slices):
            local_cout = min(16, NUM_COUT - cs × 16)
            word = pack(ofm[yout, px, cs×16 : cs×16 + local_cout], 8 bit each)
            写 word
```

每行 `W_OUT × cout_slices` 个 word。每个像素先写 cs=0 的 16 cout，再 cs=1，依次。

硬件 `ofb_writer` 写 OFB SRAM 的顺序也是这样（NHWC + cs 内层），DDR 端 ODMA 用 (yout, x, cs) gather 把 OFB 数据搬出。

## 派生的 step 和 base

`derive_layer_cfg` 算出几个供硬件累加器用的步长：

| cfg 字段 | 公式 | 含义 |
| --- | --- | --- |
| `IFB_ROW_STEP` | `stride × W_IN × cin_slices` | 跨一个输出 yout 的 IFB word 步长 |
| `IFB_KY_STEP` | `W_IN × cin_slices` | kernel ky 维跨一行 IFB 的 word 步长 |
| `IFB_ISS_STEP` | `stride × cin_slices` | iss_pos 内部跨一个像素的 word 步长 |
| `TILE_IN_STEP` | `TILE_W × stride × cin_slices` | 跨一个 tile 的 IFB word 步长 |
| `WB_COUT_STEP` | `K × KY × cin_slices` | 跨一个 cs 的权重 word 步长 |
| `OFB_ROW_WORDS` | `W_OUT × cout_slices` | 跨一个输出 yout 的 OFB word 步长 |

这些都把 `cin_slices` 或 `cout_slices` 算进去，硬件累加器看不到切片细节，只看到 step 大小。

## 容量校验

`derive_layer_cfg` 在派生过程中检查三块 SRAM 容量：

```python
ifb_words = H_IN × W_IN × cin_slices              # 整图装得下时所需
ofb_words = H_OUT × W_OUT × cout_slices
wb_words  = K × KY × cin_slices × cout_slices + cout_slices  # 含 bias 前缀
```

IFB 不够装整图时切 strip（按行环走 streaming），但每行至少 `K+1` 行，否则 raise。OFB 不够装整图时按 8 行（或更小）切 strip。WB 不够装整层权重时直接 raise——当前没做权重 streaming（chunked load 是 WRF 内部时间复用，不是 SRAM 复用）。

## 编译器到 cfg 的传递

```python
cfg = derive_layer_cfg(...)               # dict 形式
cfg_dict = cfg_to_dict(cfg, ...)          # 加 _META_* 字段
write_config(out_dir, cfg_dict)            # 输出 config.txt
```

TB 在 `tb_core_dma.sv::load_config` 里读 config.txt，按 key 调 `axi_lite_write(ADDR_XXX, val)` 把每个字段写进 cfg_regs 寄存器组。`CIN_SLICES` 和 `COUT_SLICES` 是其中两个字段，跟其他 cfg 一起在 layer 启动前一次性灌好。

## 与 fold / S2D 的叠加

切片数和 fold / S2D 是相互独立的派生：

- 用 `--ky-fold` 时编译器先计算 `cin_fake = groups_y × Cin`，再用这个虚拟 Cin 算 `cin_slices = ⌈cin_fake / 16⌉`（一般正好等于 1）
- 用 `--s2d` 时编译器先用 `Cin_new = stride² × Cin` 当作新的 Cin，再算 `cin_slices = ⌈Cin_new / 16⌉`（可能 > 1）
- 用 `--kx-fold` 时编译器算 `cout_fake = groups_x × Cout`，对应的虚拟 cout 还是落在 16 列里，`cout_slices` 不变（fold 后 cout_fake = 16）

简言之，fold / S2D 是把"小通道"补成大通道；如果补完仍然 > 16，剩下的部分由切片机制承担。
