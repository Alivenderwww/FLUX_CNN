# PyTorch 多层编译

把 `nn.Module`（当前支持 Conv2d + ReLU 串联链）端到端编译到硬件：逐层量化 → 规划 DDR 布局 → 生成 per-layer 硬件文件 → TB 驱动硬件逐层跑 → 终层比对金标准。硬件零改动，所有跨层 reset 已有支持。

## 文件位置

```
toolchain/
├── compile_layer.py        # 单层 Conv2d → 硬件文件
└── models/
    ├── compile_model.py    # Module 扫链 + calibration + DDR 规划
    ├── run_model.py        # 顶层入口: 跑 vsim + argmax 比对
    ├── train_mnist.py      # MNIST all-conv 训练脚本
    ├── dump_mnist_png.py   # MNIST → PNG (供 run_model 输入)
    ├── zoo.py              # 模型定义集合
    ├── ckpts/              # 训好的 .pt 权重
    └── images/             # 测试输入

sim/tb_core_dma/
├── tb_core_dma.sv          # preload_ddr 接受 per-layer base
└── cases/<model_name>_plan/model_plan.json   # 调试追溯用
```

## 量化与 scale 传递

`_calibrate_activations(model, x)` 用 PyTorch forward hook 采样：
- `acts[0..L-1]` = 每层 Conv2d 前的输入
- `acts[L]` = 整 model 输出
- `scales[k] = max|acts[k]| / 127`（per-tensor symmetric）

layer k 的量化参数：`s_x = scales[k]`、`s_y = scales[k+1]`，mult/shift 由 `derive_mult_shift(s_x · s_w / s_y)` 推出。

## 跨层 bit-exact

`compile_and_emit_conv2d` 接受 `ifm_int_override`：layer k+1 的整数输入 = layer k 软件模拟（`hw_files.compute_expected_ofm`）的输出，不再用 PyTorch float forward 的 `quantize_symmetric(acts[k+1])`，避免 off-by-1 舍入差异。这样硬件 RTL 输出和软件参考端到端 bit-exact。

## DDR layout

`_plan_ddr()` 把 16 MB DDR 分三段，4 KB 对齐：

| 区域 | 起点 | 止点 | 用途 |
| --- | ---: | ---: | --- |
| FM | 0x00_0000 | 0x80_0000 | feature map 共享链：layer k OFB base = layer k+1 IFB base |
| WB | 0x80_0000 | 0xF0_0000 | 每层独立权重区，按 cfg.wb_words × 256B 推进 |
| DESC | 0xF0_0000 | 0x100_0000 | 每层独立 descriptor 区（每层 32 KB） |

OFB 与 IFB 同 layout（NHWC `(y, x, channel_slice)`），中间层硬件输出直接当下层输入读，无需转置/搬移。

## config.txt 多层 META 字段

```
_META_DDR_IFB_BASE     IFB byte 地址
_META_DDR_WB_BASE      WB byte 地址
_META_DDR_OFB_BASE     OFB byte 地址（写 ODMA_DST + 终层比对）
_META_DDR_DESC_BASE    descriptor list byte 地址
_META_SKIP_IFB_PRELOAD 非首层=1 (TB 跳过 ifb.txt 预填，IFB 由上层 OFB 提供)
_META_SKIP_OFB_CLEAR   非终层=1 (TB 不清 OFB, 不读 expected_ofm.txt 不比对)
```

字段缺失时 TB fallback 到 hard-coded localparam，单层回归不受影响。

## TB 主循环（每 case 一层）

每层流程在 `tb_core_dma.sv` 的 case loop 里：

1. `load_config(case_dir)`：读 config.txt，解析 META + AXI-Lite 写所有 cfg 字段
2. `preload_ddr()`：首层用 ifb.txt 填 DDR[IFB_BASE]；wb.txt + desc_list.hex 每层写各自 base；终层清 OFB 区，中间层跳过
3. AXI-Lite 写 `IDMA_SRC_BASE / WDMA_SRC_BASE / ODMA_DST_BASE / DESC_LIST_BASE` = 本层 META
4. `CTRL[4]=1` → DFE 拉 descriptor，等 `STATUS.dfe_done=1`
5. `CTRL[5]=1` → 启动 layer，等 `STATUS.layer_done=1`
6. 终层：DDR[OFB_BASE..] vs expected_ofm.txt 比对；中间层跳过

## 使用

```bash
cd toolchain

# 训 MNIST all-conv (CPU ~2 min, 输出 models/ckpts/mnist_allconv.pt)
python models/train_mnist.py

# MNIST → PNG (供 run_model 输入)
python models/dump_mnist_png.py -c 20

# 端到端跑模型: 编译 + vsim + 软件 argmax 比对
python models/run_model.py --model mnist_allconv \
    --image-dir models/images/mnist_test --limit 10
```

## MNIST all-conv 模型

不用 Pool，stride=2 conv 下采样；末层 K=7 pad=0 当 FC：

```
Conv(1→16,  K=3, pad=1, s=1) + ReLU    28×28 → 28×28
Conv(16→16, K=3, pad=1, s=2) + ReLU    28×28 → 14×14
Conv(16→16, K=3, pad=1, s=1) + ReLU    14×14 → 14×14
Conv(16→16, K=3, pad=1, s=2) + ReLU    14×14 →  7×7
Conv(16→10, K=7, pad=0, s=1)            7×7  →  1×1   (logits, 无 ReLU)
```

末层 `relu_en=0, clip=[-128,127]` 输出 signed int8 logits。argmax 和 PyTorch float forward 一致。

## 范围 / 限制

支持：
- `Conv2d + ReLU` 任意深度串联；Sequential / ModuleList / 嵌套 Module 都 walk
- 任意 Cin / Cout（streaming 行 ring 已支持）
- 任意 H × W（多 strip 切片）
- streaming 模式（默认）

不支持：
- `BatchNorm`（需要量化前 fuse 进 Conv2d 权重）
- `MaxPool / AvgPool`（无 pool 硬件）
- Residual / skip connection（需要 SDP 残差通路）
- `Linear`（需要全连接算子，可用 1×1 conv 等价代替）
- non-symmetric 量化 / per-channel quant

DDR 尺寸：FM 链总和 ≤ 8 MB。

## 添加新模型

在 `models/zoo.py` 的 `MODELS` dict 加一个 `_build_*` 函数返回 `(model, calib_x, n_layers)`，然后 `python models/run_model.py --model <name>` 即可。
