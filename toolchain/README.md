# FLUX_CNN Toolchain

两条主要工作流：
- **Case 回归**（`run_regression.py`）：遍历卷积参数空间 (K/stride/Cin/Cout/pad/stream) 的随机数据测试，验证 RTL 覆盖度。
- **真实模型**（`models/run_model.py`）：端到端跑 PyTorch 训练好的 CNN 模型，逐图像验证硬件结果。

## 目录结构

```
toolchain/
├── .venv/                   # Python 虚拟环境 (gitignored, py -3.13 -m venv .venv)
├── requirements.txt         # torch, torchvision, numpy
├── README.md
│
├── compile_layer.py         # 单层 Conv2d → 硬件数据 (两条流水共用)
├── hw_files.py              # DDR 文件 I/O + cfg 派生 + SDP 软件模拟 (共用)
│
├── gen_isa_test.py          # 随机数据单层生成 (case 回归)
├── run_regression.py        # case 回归驱动 (35 case, batch+stream)
│
└── models/                  # 真实模型工程
    ├── compile_model.py     # nn.Module → 多层硬件编译, ifm chain, DDR 规划
    ├── zoo.py               # 模型定义 (mnist2 / mnist_allconv / ...)
    ├── train_mnist.py       # MNIST all-conv 训练 (CPU ~2 min, 8 epoch)
    ├── run_model.py         # 模型 runner: 选模型/图像源, 输出 txt 报告
    ├── dump_mnist_png.py    # MNIST test binary → PNG 到 images/mnist_test/
    ├── ckpts/               # 训练后 *.pt (gitignored)
    ├── data/                # torchvision MNIST raw (gitignored)
    └── images/mnist_test/   # 输入 PNG (filename `NNNN_labelX.png`)
```

## 首次安装

必须用标准 Windows Python（非 MSYS2 MinGW）。

```bash
cd toolchain
py -3.13 -m venv .venv
source .venv/Scripts/activate         # Git Bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## Case 回归（RTL 参数空间覆盖）

```bash
cd toolchain
python run_regression.py                   # 35 case (batch+stream+padding+VGA)
python run_regression.py --only stream     # 只 stream
python run_regression.py --case C16C16     # name 含 "C16C16" 的 case
python run_regression.py --label "my-note"
```

输出：`../sim/tb_core_dma/regression_report.txt`

## 真实模型工程验证

### 1. 训练（一次即可）

```bash
cd toolchain
.venv/Scripts/python.exe models/train_mnist.py   # ~2 min CPU, test_acc ≈ 98.9%
# → models/ckpts/mnist_allconv.pt
```

### 2. Dump 测试图

```bash
.venv/Scripts/python.exe models/dump_mnist_png.py --count 20
# → models/images/mnist_test/0000_label7.png, 0001_label2.png, ...
```

### 3. 跑推理 + 验证

```bash
# 单张图
.venv/Scripts/python.exe models/run_model.py --model mnist_allconv \
    --image models/images/mnist_test/0000_label7.png

# 文件夹批量 (默认用 models/images/mnist_test/)
.venv/Scripts/python.exe models/run_model.py --model mnist_allconv \
    --image-dir models/images/mnist_test --limit 10

# 从 torchvision test set 直接取若干张
.venv/Scripts/python.exe models/run_model.py --model mnist_allconv \
    --torchvision-idx 0 1 2 3 4

# 自定义输出报告路径
.venv/Scripts/python.exe models/run_model.py --model mnist_allconv \
    --image-dir my/images/ --out my_report.txt
```

报告：`models/model_report.txt`（格式见 `models/run_model.py` 文档）
- 每张图三方对比：True label / PyTorch float argmax / Hardware int8 argmax
- vsim 各层 bit-exact 通过率
- 三方一致率 + PyTorch/硬件 accuracy

### 新增模型

1. 在 `models/zoo.py` 加 `_build_<name>`，返回 `(model, calib_tensor, n_layers)`
2. 注册到 `MODELS[<name>]`
3. 在 `models/run_model.py` 命令行用 `--model <name>`

限制：当前只支持 Conv2d + ReLU 串联链；Pool / Linear / BN / Residual 待 roadmap。

## 输出路径

两种流水都写到 `../sim/tb_core_dma/cases/caseNN/`，vsim 在该目录跑。
