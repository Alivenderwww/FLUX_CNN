# 切片相关的配置寄存器和描述符

切片信息通过两条路径传到硬件：
1. **layer 级 cfg 寄存器**——每层一次性写入，整层处理过程中保持
2. **strip 级 descriptor**——每 strip 一条，由 sequencer 按需更新 cfg 子集

## layer 级 cfg 字段

直接承载切片数：

| 寄存器 | 位宽 | 字段 |
| --- | ---: | --- |
| `CIN_SLICES` (0x114) | 6 | `⌈Cin / 16⌉` |
| `COUT_SLICES` (0x118) | 6 | `⌈Cout / 16⌉` |

切片数最多 64，覆盖 Cin/Cout ≤ 1024（实际项目里 Cin/Cout ≤ 512 已足够）。

派生 step 字段（编译器算好后写入，硬件不做乘法）：

| 寄存器 | 位宽 | 公式 | 消费者 |
| --- | ---: | --- | --- |
| `IFB_ROW_STEP` (0x14C) | 20 | `stride × W_IN × cin_slices` | line_buffer 跨 yout 累加 |
| `IFB_KY_STEP` (0x1B0) | 20 | `W_IN × cin_slices` | line_buffer 跨 ky 累加 |
| `IFB_ISS_STEP` (0x1AC) | 20 | `stride × cin_slices` | line_buffer 跨 iss_pos 累加（reuse_en=0 模式） |
| `TILE_IN_STEP` (0x15C) | 20 | `TILE_W × stride × cin_slices` | line_buffer 跨 tile 累加 |
| `WB_COUT_STEP` (0x154) | 20 | `K × KY × cin_slices` | wgt_buffer 跨 cs 累加 |
| `IFB_RING_WORDS` (0x1A0) | 20 | `strip_rows × W_IN × cin_slices` | line_buffer / IDMA ring wrap 模数 |
| `OFB_ROW_WORDS` (0x1A4) | 20 | `W_OUT × cout_slices` | ODMA 跨 yout 步长 |
| `OFB_RING_WORDS` (0x1A8) | 20 | `strip_rows × W_OUT × cout_slices` | ofb_writer / ODMA ring wrap 模数 |

辅助字段：

| 寄存器 | 含义 |
| --- | --- |
| `KK` (0x130) | `K × KY`（fold 后是 `kxper × kyper`），parf_accum 计 fill 拍数用 |
| `TOTAL_WRF` (0x128) | `KK × cin_slices`，wgt_buffer cold load 总写 WRF 次数 |
| `ROUNDS_PER_CINS` (0x134) | `⌈KK / 32⌉`，K² 超 WRF 容量时按 round 切片 |
| `ROUND_LEN_LAST` (0x138) | 末 round 长度 |

## strip 级 descriptor 字段

每条 conv descriptor（256 bit）携带 strip 内容。切片相关字段：

| 字段 | 位宽 | 含义 |
| --- | ---: | --- |
| `n_yout_strip` | 16 | 这条 strip 处理几行 yout |
| `strip_y_start` | 16 | strip 在整图的 y 起点 |
| `ifb_ddr_offset` | 20 | 这条 strip 的 IFB DDR 偏移（相对 layer 的 `IDMA_SRC_BASE`） |
| `ifb_byte_len` | 24 | 字节长度 |
| `ofb_ddr_offset` | 20 | OFB DDR 偏移（相对 `ODMA_DST_BASE`） |
| `ofb_byte_len` | 24 | 字节长度 |

整图按 strip 切（H 方向）时，每条 strip 的 `ifb_byte_len` = `(strip_rows + 边界 K-1 行) × W_IN × cin_slices × 16 byte`。`cin_slices` 已在 layer cfg 里算进 IFB DDR layout，descriptor 的 byte_len 是按 layer 的 cin_slices 算好的整 strip 长度。

`cs` 切片**不分 strip**——一条 strip 内部的 `cout_slices` 段都靠硬件 cs_cnt 自己跑完。所以一条 strip 的 OFM 输出占 `n_yout × W_OUT × cout_slices × 16 byte`，descriptor 的 `ofb_byte_len` 就是这个值。

## 加载流程

host 写 layer cfg 一次：

```
axi_lite_write(ADDR_CIN_SLICES,   2)                 # Cin=32 → 2 片
axi_lite_write(ADDR_COUT_SLICES,  4)                 # Cout=64 → 4 片
axi_lite_write(ADDR_TOTAL_WRF,    18)                # 9 × 2 = 18
axi_lite_write(ADDR_KK,           9)                 # K=3 → 9
axi_lite_write(ADDR_IFB_ROW_STEP, 2 × 34 × 2)        # 136
axi_lite_write(ADDR_IFB_KY_STEP,  34 × 2)            # 68
axi_lite_write(ADDR_TILE_IN_STEP, TILE_W × 2 × 2)
axi_lite_write(ADDR_WB_COUT_STEP, 9 × 2)             # 18
axi_lite_write(ADDR_OFB_ROW_WORDS, 17 × 4)           # 68
... (其他 cfg)
axi_lite_write(ADDR_DESC_LIST_BASE, descriptor 起点)
axi_lite_write(ADDR_DESC_COUNT,     n_strips + 1)
```

然后写 `CTRL[4]=1` 触发 DFE 拉 descriptor，写 `CTRL[5]=1` 启动 layer。sequencer pop 第一条 descriptor 后给 line_buffer / ofb_writer 发 strip-level 的 `n_yout / pad / y_start / ddr_offset` 等参数（不是写回 cfg_regs，而是直接走专用线送达模块）。

`CIN_SLICES / COUT_SLICES` 在整层处理过程中保持不变，每条 strip 都用同一组切片数。

## 与 fold / S2D 字段的关系

启用 fold 或 S2D 时，编译器会用虚拟 cin / cout 重新算 `cin_slices / cout_slices`：

| 启用项 | 影响 |
| --- | --- |
| `--ky-fold` | 用 `cin_fake = groups_y × Cin` 重派 → 一般 `cin_slices = 1` |
| `--kx-fold` | 用 `cout_fake = groups_x × Cout = 16` 重派 → `cout_slices = 1` |
| `--s2d` | 用 `Cin_new = stride² × Cin` 重派 → `cin_slices` 可能 > 1 |

Kx-fold 配的 `FOLD_COUT_ORIG / FOLD_COUT_GROUPS / FOLD_COL_SHIFT` 是独立 cfg 字段（0x1BC..0x1C4），和切片机制不冲突。`KK` 在 fold 时用 `kxper × kyper`，`K` 和 `KY` 用虚拟 K（fold 后维度），`COUT_SLICES` 重新派生（一般是 1）。

## 软件 source of truth

所有派生计算只在 `toolchain/hw_files.py::derive_layer_cfg` 一处做。新加 cfg 字段时：

1. `derive_layer_cfg` 算出值，存到 `cfg` dict
2. `cfg_to_dict` 把 dict key 写进 config.txt 的 key=value 行
3. `tb_core_dma.sv::load_config` 加一个 `case ("KEY")` 分支调 `axi_lite_write(ADDR_KEY, val)`
4. `cfg_regs.sv` 声明 `r_key` 寄存器 + 输出端口
5. `core_top.sv` 把 `cfg.key` 连到消费它的模块

切片体系下大部分字段已经齐了；后续给硬件加新优化（比如 stride 滑窗复用扩展）时按这个流程加新 cfg。
