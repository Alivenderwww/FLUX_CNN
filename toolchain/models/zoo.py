"""
zoo.py — PyTorch 模型定义集合

单独维护：新增模型只在这里改；compile_model 从这里 import。

每个 _build_* 返回 (nn.Module, calibration_tensor, n_layers)：
  model              — 待编译的模块，暂支持 Conv2d + ReLU 串联
  calibration_tensor — 一个 batch 的输入样本，用于 per-tensor max-abs scale 估算
                       (compile_model 也用它决定每层 H_IN/W_IN)
  n_layers           — Conv2d 个数

calib 可被 run_model.py 替换成真实测试图像 (保持 scale calibration 不变, 只换 data path)。
"""

import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_TOOLCHAIN_DIR = os.path.dirname(_SCRIPT_DIR)
if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)
import compile_layer     # noqa: E402

CKPTS_DIR = os.path.join(_SCRIPT_DIR, "ckpts")


# ---------------------------------------------------------------------------
# Smoke / 基线
# ---------------------------------------------------------------------------
def _build_mnist2():
    """2 层 Conv smoke (48x48, Cin=1→8→16)。"""
    torch = compile_layer._require_torch()
    torch.manual_seed(42)
    model = torch.nn.Sequential(
        torch.nn.Conv2d(1,  8, 3, padding=1, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(8, 16, 3, padding=1, bias=True), torch.nn.ReLU(),
    )
    x = torch.randn(1, 1, 48, 48)
    return model, x, 2


# ---------------------------------------------------------------------------
# All-Conv MNIST：无 Pool，用 stride=2 conv 下采样，末层 K=7 conv 等价 FC
# 权重由 toolchain/train_mnist.py 训练并保存到 models/mnist_allconv.pt
# ---------------------------------------------------------------------------
def _build_mnist_allconv():
    torch = compile_layer._require_torch()
    ckpt_path = os.path.join(CKPTS_DIR, "mnist_allconv.pt")
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(
            f"checkpoint not found: {ckpt_path}\n"
            f"先跑训练: .venv/Scripts/python.exe train_mnist.py")
    ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)

    model = torch.nn.Sequential(
        torch.nn.Conv2d(1,  16, 3, padding=1, stride=1, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16, 16, 3, padding=1, stride=2, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16, 16, 3, padding=1, stride=1, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16, 16, 3, padding=1, stride=2, bias=True), torch.nn.ReLU(),
        torch.nn.Conv2d(16, 10, 7, padding=0, stride=1, bias=True),  # 无 ReLU
    )
    # AllConvMNIST 的 state_dict key 以 'net.' 前缀保存；这里直接是 Sequential，剥前缀
    sd = {k.replace("net.", "", 1): v for k, v in ckpt['state_dict'].items()}
    model.load_state_dict(sd)
    model.eval()
    # calib input：取训练时保存的第 0 张 test image (28x28)
    calib = ckpt['calib_image']   # (1,1,28,28)
    return model, calib, 5


# ---------------------------------------------------------------------------
# 注册表
# ---------------------------------------------------------------------------
MODELS = {
    'mnist2'        : _build_mnist2,
    'mnist_allconv' : _build_mnist_allconv,
}


def build_model(name):
    if name not in MODELS:
        raise ValueError(f"unknown model '{name}'; known: {list(MODELS.keys())}")
    return MODELS[name]()
