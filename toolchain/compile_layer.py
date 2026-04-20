"""
compile_layer.py — 单层 Conv2d (PyTorch) → 硬件数据 + cfg + descriptor

用法：
    python compile_layer.py [--out-dir DIR]
    (作为 library 被 compile_network.py 调用)

输入：
    - layer : torch.nn.Conv2d (含 weight + optional bias)
    - x_float : torch.Tensor [1, Cin, H, W]
    - s_x : float  (input activation scale, per-tensor symmetric)
    - s_y : float  (output activation scale)

输出（写到 out_dir，和 gen_isa_test.py 格式一致，可直接供 tb_core_dma 跑）：
    - ifb.txt  : int8 输入 (NHWC, 每 word 16 cin)
    - wb.txt   : [bias prefix | weight]
    - expected_ofm.txt : int8 输出 (NHWC, 每 word 16 cout)
    - config.txt : 所有 cfg 字段
    - desc_list.hex : descriptor
    - sim_params.f  : TB plusargs

量化方案：per-tensor symmetric int8
    - weight scale: s_w = max|w| / 127
    - bias scale:   s_b = s_x * s_w (int32 域)
    - output scale: s_y (用户给定或估算)
    - mult/shift:   M = s_x * s_w / s_y ≈ mult / 2^shift  (int32 mult, shift≈24~31)
"""

import os
import math
import argparse

# torch 延迟导入（让 --help 等能在 torch 缺失时跑）
def _require_torch():
    try:
        import torch
        return torch
    except ImportError as e:
        raise SystemExit(
            "ERROR: torch not installed. 在 toolchain/.venv 里先 `pip install torch numpy`.")

# gen_isa_test 里已有的 cfg derive / file writer 逻辑先复用
# (后续若需要再把公共部分抽到 _hwfile.py)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))


# ---------------------------------------------------------------------------
# 量化工具
# ---------------------------------------------------------------------------
def derive_mult_shift(scale_ratio, shift=24):
    """M = scale_ratio ≈ mult / 2^shift, 求定点 mult。"""
    mult = round(scale_ratio * (1 << shift))
    # 防止 mult 超 int32（如果太大就减 shift）
    while mult >= (1 << 31):
        shift -= 1
        mult = round(scale_ratio * (1 << shift))
    return mult, shift


def quantize_symmetric(tensor, scale, qmin=-128, qmax=127):
    """Per-tensor symmetric int8: q = clip(round(x / s), qmin, qmax)."""
    import torch
    q = torch.round(tensor / scale).clamp(qmin, qmax).to(torch.int32)
    return q


def compute_weight_scale(weight_tensor):
    """Per-tensor symmetric: s_w = max|w| / 127."""
    w_max = weight_tensor.abs().max().item()
    if w_max < 1e-12:
        return 1.0   # all-zero weights, scale doesn't matter
    return w_max / 127.0


def estimate_activation_scale(tensor):
    """粗略估计 activation scale：s = max|x| / 127 (对 calibration 数据)。"""
    x_max = tensor.abs().max().item()
    return max(x_max / 127.0, 1e-8)


# ---------------------------------------------------------------------------
# 主 API：单层 Conv2d 编译
# ---------------------------------------------------------------------------
def compile_conv2d(conv, x_float, s_x, s_y, out_dir=DEFAULT_OUT_DIR, streaming=False):
    """
    编译单层 Conv2d。conv 可以含 bias。
    返回 dict 包含量化参数 + 中间产物（方便 verify）。
    """
    torch = _require_torch()

    # --- 提取 layer 参数 ---
    K       = conv.kernel_size[0]
    assert conv.kernel_size[0] == conv.kernel_size[1], "只支持方形 kernel"
    stride  = conv.stride[0]
    pad     = conv.padding[0]
    Cin     = conv.in_channels
    Cout    = conv.out_channels
    H_IN    = x_float.shape[2]
    W_IN    = x_float.shape[3]
    H_OUT   = (H_IN + 2 * pad - K) // stride + 1
    W_OUT   = (W_IN + 2 * pad - K) // stride + 1

    assert conv.stride[0] == conv.stride[1] and conv.padding[0] == conv.padding[1]

    # --- 量化 ---
    w_float = conv.weight.data.clone()
    b_float = conv.bias.data.clone() if conv.bias is not None else torch.zeros(Cout)

    s_w     = compute_weight_scale(w_float)
    s_bias  = s_x * s_w
    scale_ratio = s_bias / s_y
    mult, shift = derive_mult_shift(scale_ratio)

    # 量化：x → int8, w → int8, bias → int32
    x_q = quantize_symmetric(x_float, s_x)
    w_q = quantize_symmetric(w_float, s_w)
    b_q = torch.round(b_float / s_bias).to(torch.int32)   # int32，不 clamp

    # --- Golden 计算（硬件流水模拟）---
    # 硬件：acc = bias_q + Σ(x_q * w_q); y = clip((acc*mult+round)>>shift + zp, clip_min, clip_max)
    # 这里用 PyTorch 做 float conv 再量化，作为参考；硬件用 int 乘加走量化流水
    with torch.no_grad():
        y_float = conv(x_float)
    y_expected_q = quantize_symmetric(y_float, s_y)  # 供对比参考（硬件应输出近似值）

    return {
        'K': K, 'stride': stride, 'pad': pad,
        'Cin': Cin, 'Cout': Cout,
        'H_IN': H_IN, 'W_IN': W_IN, 'H_OUT': H_OUT, 'W_OUT': W_OUT,
        's_x': s_x, 's_w': s_w, 's_bias': s_bias, 's_y': s_y,
        'mult': mult, 'shift': shift,
        'x_q': x_q, 'w_q': w_q, 'b_q': b_q, 'y_q': y_expected_q,
        'streaming': streaming,
        'out_dir': out_dir,
    }


# ---------------------------------------------------------------------------
# CLI (主要是 smoke test，真实场景由 compile_network.py 驱动)
# ---------------------------------------------------------------------------
def _smoke():
    """简单 sanity: 建一个 Conv2d(1→6, K=5, pad=2)，随机输入，编译。"""
    torch = _require_torch()
    torch.manual_seed(42)
    conv = torch.nn.Conv2d(1, 6, kernel_size=5, padding=2, bias=True)
    x = torch.randn(1, 1, 28, 28)   # MNIST Conv1 风格
    s_x = estimate_activation_scale(x)
    with torch.no_grad():
        y = conv(x)
    s_y = estimate_activation_scale(y)
    info = compile_conv2d(conv, x, s_x, s_y)
    print(f"Smoke compile OK: K={info['K']} Cin={info['Cin']} Cout={info['Cout']} "
          f"H_IN={info['H_IN']} mult={info['mult']} shift={info['shift']}")
    print(f"  s_x={info['s_x']:.5g} s_w={info['s_w']:.5g} s_y={info['s_y']:.5g}")
    print(f"  x_q range: [{info['x_q'].min().item()}, {info['x_q'].max().item()}]")
    print(f"  w_q range: [{info['w_q'].min().item()}, {info['w_q'].max().item()}]")
    print(f"  b_q range: [{info['b_q'].min().item()}, {info['b_q'].max().item()}]")
    print(f"  y_q range: [{info['y_q'].min().item()}, {info['y_q'].max().item()}]")


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--smoke', action='store_true', help='smoke test')
    p.add_argument('--out-dir', default=DEFAULT_OUT_DIR)
    args = p.parse_args()
    if args.smoke:
        _smoke()
    else:
        print("compile_layer.py: use --smoke for self-check, or import as library.")
