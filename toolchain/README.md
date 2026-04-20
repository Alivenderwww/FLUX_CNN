# FLUX_CNN Toolchain

Python 脚本：随机测试生成器、回归驱动、PyTorch 算子编译器。

## 目录结构

```
toolchain/
├── .venv/                 # Python 虚拟环境（git 忽略，用 py -3.13 -m venv .venv 创建）
├── requirements.txt       # torch, numpy
├── hw_files.py            # 硬件文件 I/O + cfg 派生 + SDP 软件模拟（共享层）
├── gen_isa_test.py        # 随机数据生成（RTL 回归用）
├── run_regression.py      # 回归驱动（batch + streaming + padding case）
├── compile_layer.py       # PyTorch 单层 Conv2d → 硬件数据 (F-1c)
├── models/                # (F-6, TODO) 训练好的 PyTorch state_dict
└── README.md
```

## 模块关系

```
gen_isa_test.py --随机 ifm/weight--┐
                                   ├─→ hw_files 写 ifb.txt/wb.txt/expected_ofm.txt/config.txt/desc_list.hex/sim_params.f
compile_layer.py --PyTorch 量化---┘
```

## 首次安装

**重要**：必须用标准 Windows Python（不是 MSYS2 MinGW Python），否则 PyPI 上没 wheel。

```bash
cd toolchain
# 用 py launcher 找 Windows Python (避免 MSYS2 Python)
py -3.13 -m venv .venv
# 激活（Git Bash）
source .venv/Scripts/activate
# 或 cmd
.venv\Scripts\activate

# 使用镜像源（可选，国内更快）
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

已验证版本：Python 3.13 + torch 2.11 + numpy 2.4。

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
