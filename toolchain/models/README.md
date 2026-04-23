# `toolchain/models/` — 真实模型工程

不同于上层 case 回归（目的是 RTL 参数空间覆盖，遍历 K/stride/Cin/Cout/pad 等），
本目录面向**端到端工程验证**：拿训练好的 PyTorch 模型 + 真实图像，跑一遍硬件
仿真看结果对不对。

## 文件

- `compile_model.py` — `nn.Module` → 多层硬件编译流水。扫 Conv2d + ReLU 链、
  calibration、DDR layout、逐层派发到 `../compile_layer.py`。
- `zoo.py` — 模型定义集合（`MODELS[name] = builder`）。新增模型改这里。
- `train_mnist.py` — MNIST all-conv 训练脚本（CPU ~2 min，8 epoch）。
- `run_model.py` — 主 runner，类似 `run_regression.py` 但按"模型 × 输入图"跑。
- `dump_mnist_png.py` — torchvision MNIST binary → PNG（方便 `--image-dir` 输入）。

## 目录

- `ckpts/` — 训练后 `*.pt`（gitignored）
- `data/` — torchvision 下载的 MNIST raw（gitignored）
- `images/mnist_test/` — 输入 PNG，filename `NNNN_labelX.png`（label 直接解析为 ground truth）

## 快速上手

```bash
cd toolchain

# 一次性准备
.venv/Scripts/python.exe models/train_mnist.py           # 训, ~2min CPU
.venv/Scripts/python.exe models/dump_mnist_png.py -c 20  # 20 张 PNG

# 跑推理
.venv/Scripts/python.exe models/run_model.py --model mnist_allconv --image-dir models/images/mnist_test --limit 10
```

报告写到 `models/model_report.txt`。

## 当前支持

- 模型结构：Conv2d + ReLU 串联（Sequential / 嵌套 Module 都 walk）
- 量化：per-tensor symmetric int8；max-abs scale calibration
- DDR layout：FM 共享链（layer k OFB = layer k+1 IFB, 按 input size 推进避开覆盖）
- 硬件约束：streaming 模式, Cin/Cout 各 ≤ 16×slices, K ∈ {3,5,7}, stride ∈ {1,2}

## 已注册模型

- `mnist2` — 2 层 Conv smoke (48×48, Cin=1→8→16)
- `mnist_allconv` — 5 层 all-conv MNIST 分类器 (28×28→1×1×10, 无 Pool 用 stride=2 下采样, 末层 K=7 pad=0 当 FC 等价)
