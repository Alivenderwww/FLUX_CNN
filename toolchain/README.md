# FLUX_CNN Toolchain

Python 脚本：随机测试生成器、回归驱动、PyTorch 算子编译器。

## 目录结构

```
toolchain/
├── .venv/                 # Python 虚拟环境（git 忽略）
├── requirements.txt       # torch, numpy, torchvision
├── gen_isa_test.py        # 随机数据生成（用于 RTL 回归）
├── run_regression.py      # 回归驱动（batch + streaming + padding case）
├── compile_layer.py       # (F-1c, TODO) PyTorch 单层 Conv2d → 硬件数据
├── models/                # (F-6, TODO) 训练好的 PyTorch state_dict
└── README.md
```

## 首次安装

```bash
cd toolchain
python -m venv .venv
# Windows (Git Bash)
source .venv/Scripts/activate
# Windows (cmd)
.venv\Scripts\activate
# Linux/Mac
source .venv/bin/activate

pip install -r requirements.txt
```

## 日常使用

所有脚本默认把生成文件写到 `../sim/tb_core_dma/`。仿真在该目录下跑 `vsim -c -do run.tcl`。

### 单个测试用例

```bash
# 在 toolchain/ 下
python gen_isa_test.py --k 3 --h_in 48 --w_in 48 --num_cin 8 --num_cout 8 --pad 1

# 跑仿真（切到 sim 目录）
cd ../sim/tb_core_dma && vsim -c -do run.tcl
```

### 回归

```bash
python run_regression.py                    # 全部 case (batch + stream)
python run_regression.py --only batch       # 仅 batch
python run_regression.py --only stream      # 仅 stream
python run_regression.py --label "my-note"  # 报告加 label
```

报告写入 `../sim/tb_core_dma/regression_report.txt`。

## 输出目录可配置

```bash
python gen_isa_test.py --out-dir /tmp/cnntest ...
```
