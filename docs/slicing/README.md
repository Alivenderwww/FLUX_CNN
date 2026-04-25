# 通道切片（Cin / Cout > 16）

PE 阵列物理尺寸是 16 行 × 16 列。当卷积层的 Cin > 16 或 Cout > 16 时，编译器把通道维**切成多片**，硬件通过两层循环计数器（`cins_cnt` 和 `cs_cnt`）按片串行处理。整个机制对软件透明——`gen_isa_test.py` 派生切片数后写进 cfg 寄存器，硬件按 cfg 自驱跑。

## 切片维度定义

| 名称 | 含义 | 计算 |
| --- | --- | --- |
| `cin_slices` | 输入通道切片数 | `⌈Cin / 16⌉` |
| `cout_slices` | 输出通道切片数 | `⌈Cout / 16⌉` |

每片包含至多 16 个通道；末片不足 16 时由 `local_cin / local_cout` 在编译期补齐到 16 个 word slot（多出来的 slot 写零）。

## 三层视角

| 角度 | 关注 | 文档 |
| --- | --- | --- |
| 编译器侧 | 怎么算切片数、怎么布权重 / IFM / OFM | [compiler.md](compiler.md) |
| 描述符 / 配置寄存器 | 哪些 cfg 字段承载切片信息、descriptor 怎么寻址各片 | [descriptor-cfg.md](descriptor-cfg.md) |
| 硬件循环 | 5 个核心模块的计数器嵌套和数据流 | [hw-loops.md](hw-loops.md) |

## 切片粒度的逻辑作用

切片让物理 16×16 的阵列处理任意大小的通道：

- **cin 切片**：每 (yout, cs, tile) 累加 cin_slices 次到同一 PARF 位置；cins=last 时该 tile 的输出累加完整，进入 drain。简言之，cin 切片是**时间维度**的累加。
- **cout 切片**：每 (yout, cs) 处理 16 个 cout 通道；不同 cs 在时间上**串行**展开。每完成一个 (yout, cs) 段，OFB 写出 W_OUT 个 16-cout word，一行内 cout_slices 段相邻。

输出通道和输入通道的切片角色不同：cin 切片靠 PARF 累加合并贡献，cout 切片靠 OFB 行内段 NHWC 拼接。

## 一个具体例子

ResNet-18 风格 L4.B1.C1 层：`K=3, stride=2, Cin=32, Cout=64, H_in=60, W_in=34, pad=1`。

派生：

```
cin_slices  = ⌈32 / 16⌉ = 2
cout_slices = ⌈64 / 16⌉ = 4
H_out       = (60 + 2 - 3) / 2 + 1 = 30
W_out       = (34 + 2 - 3) / 2 + 1 = 17
```

整层在硬件里跑的循环嵌套（从最外到最内）：

```
for yout in [0, 30):
  for cs in [0, 4):                 # 4 个 cout 切片串行
    for tile in [0, num_tiles):     # tile 划分 W_OUT
      for cins in [0, 2):            # 2 个 cin 切片累加进 PARF
        for ky in [0, 3):
          for kx in [0, 3):
            for iss_pos in [0, cur_valid_w):
              MAC fire
      drain PARF → SDP → OFB        # 每 (yout, cs, tile) drain 一次
```

总 MAC 次数 = `H_out × W_out × K² × Cin × Cout = 30 × 17 × 9 × 32 × 64 = 9,400,320`，由 16×16=256 个 PE 在 `30 × 4 × num_tiles × 2 × 9 × cur_valid_w` 拍内累加完成。

## 容量约束

切片不能让 IFB / WB / OFB 任一块 SRAM 撑爆。`derive_layer_cfg` 会做边界校验：

- IFB：每行 `W_IN × cin_slices` word，至少需要 `K+1` 行容纳一次卷积窗口
- WB：`K² × cin_slices × cout_slices + cout_slices` word（含 bias 前缀），整层一次性 load
- OFB：每行 `W_OUT × cout_slices` word，至少 2 行用于 streaming row credit

超容量会 raise；H 方向不够装时切 strip（行环），W 方向有 `tile_w` 做 X 切片。

## 相关 cfg 字段

直接承载切片数：`CIN_SLICES (6 bit)` / `COUT_SLICES (6 bit)`。

派生 step：`IFB_ROW_STEP = stride × W_IN × cin_slices`、`WB_COUT_STEP = K² × cin_slices`、`TILE_IN_STEP = TILE_W × stride × cin_slices`、`IFB_KY_STEP = W_IN × cin_slices`、`IFB_ISS_STEP = stride × cin_slices`、`OFB_ROW_WORDS = W_OUT × cout_slices`。

详见 [descriptor-cfg.md](descriptor-cfg.md)。
