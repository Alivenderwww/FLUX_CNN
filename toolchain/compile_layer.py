"""
compile_layer.py — 单层 Conv2d (PyTorch) → 硬件数据 + cfg + descriptor

基于 hw_files 共享层写文件。支持 per-tensor symmetric int8 量化：
  - weight:   s_w = max|w|/127,  w_q = round(w/s_w).clip(-128,127) : int8
  - bias:     s_b = s_x·s_w,     b_q = round(b/s_b)                : int32
  - input:    s_x (user),        x_q = round(x/s_x).clip(-128,127) : int8
  - output:   s_y (user),        y_q = round(y/s_y).clip(-128,127) : int8
  - rescale:  M = s_x·s_w/s_y ≈ mult / 2^shift (硬件定点 SDP)

硬件模拟等价性：
  acc_i32 = bias_q + Σ(x_q · w_q)
  y_i8    = clip((acc_i32·mult + 2^(shift-1)) >> shift + zp_out, clip_min, clip_max)
等于 PyTorch 量化流程 y_q = round(y_float / s_y).clip(...) 到量化噪声内。

用法：
    # 作为 library：
    from compile_layer import compile_and_emit_conv2d
    compile_and_emit_conv2d(conv, x_float, s_x, s_y, out_dir='../sim/tb_core_dma')

    # CLI smoke：
    python compile_layer.py --smoke
"""

import os
import argparse

import hw_files


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))


def _require_torch():
    try:
        import torch
        return torch
    except ImportError:
        raise SystemExit(
            "ERROR: torch not installed. 在 toolchain/.venv 里先 `pip install torch numpy`.")


# ---------------------------------------------------------------------------
# 量化工具
# ---------------------------------------------------------------------------
def derive_mult_shift(scale_ratio, shift=24):
    """M = scale_ratio ≈ mult / 2^shift, 求定点 mult (int32)。"""
    mult = round(scale_ratio * (1 << shift))
    while mult >= (1 << 31):
        shift -= 1
        mult = round(scale_ratio * (1 << shift))
    while mult > 0 and mult < (1 << 24) and shift < 30:
        # 让 mult 占用更多位得到更好 rounding 精度
        shift += 1
        mult = round(scale_ratio * (1 << shift))
    return mult, shift


def quantize_symmetric(tensor, scale, qmin=-128, qmax=127):
    torch = _require_torch()
    q = torch.round(tensor / scale).clamp(qmin, qmax).to(torch.int32)
    return q


def compute_weight_scale(weight_tensor):
    w_max = weight_tensor.abs().max().item()
    return max(w_max / 127.0, 1e-12)


def estimate_activation_scale(tensor):
    """基于 max|x| 做对称量化 (calibration 粗略版)；实际可做 percentile。"""
    x_max = tensor.abs().max().item()
    return max(x_max / 127.0, 1e-8)


# ---------------------------------------------------------------------------
# PyTorch tensor → Python list (hw_files 消费的是 int list)
# ---------------------------------------------------------------------------
def _tensor_to_list3(t_NCHW):
    """(1, C, H, W) int → [H][W][C] Python list"""
    _, C, H, W = t_NCHW.shape
    out = [[[0] * C for _ in range(W)] for _ in range(H)]
    vals = t_NCHW[0].tolist()   # [C][H][W]
    for y in range(H):
        for x in range(W):
            for c in range(C):
                out[y][x][c] = int(vals[c][y][x])
    return out


def _tensor_to_list4_weight(w_OIHW):
    """(Cout, Cin, K, K) int → [K][K][Cout][Cin] Python list"""
    Cout, Cin, K, _ = w_OIHW.shape
    out = [[[[0] * Cin for _ in range(Cout)]
            for _ in range(K)] for _ in range(K)]
    vals = w_OIHW.tolist()
    for ky in range(K):
        for kx in range(K):
            for co in range(Cout):
                for ci in range(Cin):
                    out[ky][kx][co][ci] = int(vals[co][ci][ky][kx])
    return out


def _tensor_to_list_bias(b_tensor):
    return [int(v) for v in b_tensor.tolist()]


# ---------------------------------------------------------------------------
# 主编译 API
# ---------------------------------------------------------------------------
def compile_and_emit_conv2d(
    conv, x_float, s_x, s_y,
    out_dir=DEFAULT_OUT_DIR, streaming=False,
    seed_label="conv",
):
    """
    单层 Conv2d 编译：量化 weight/bias/input，派生 mult/shift，写硬件文件。
    返回 dict 含量化参数 + 硬件期望输出 (供调用方核对)。
    """
    torch = _require_torch()

    # --- 提取参数 ---
    assert conv.kernel_size[0] == conv.kernel_size[1], "只支持方形 kernel"
    assert conv.stride[0] == conv.stride[1], "只支持方形 stride"
    assert conv.padding[0] == conv.padding[1], "只支持对称 pad"
    K       = conv.kernel_size[0]
    stride  = conv.stride[0]
    pad     = conv.padding[0]
    Cin     = conv.in_channels
    Cout    = conv.out_channels
    H_IN    = x_float.shape[2]
    W_IN    = x_float.shape[3]

    # --- 量化 ---
    w_float = conv.weight.data.clone()
    b_float = conv.bias.data.clone() if conv.bias is not None else torch.zeros(Cout)

    s_w     = compute_weight_scale(w_float)
    s_bias  = s_x * s_w
    scale_ratio = s_bias / s_y
    sdp_mult, sdp_shift = derive_mult_shift(scale_ratio)

    x_q = quantize_symmetric(x_float, s_x)
    w_q = quantize_symmetric(w_float, s_w)
    b_q = torch.round(b_float / s_bias).to(torch.int32)   # int32 in acc 域

    # --- 转 Python list 给 hw_files ---
    ifm_arr  = _tensor_to_list3(x_q)
    w_arr    = _tensor_to_list4_weight(w_q)
    bias_arr = _tensor_to_list_bias(b_q)

    # --- SDP cfg (int8 symmetric)：relu_en=1, zp_out=0, clip=[0,127] (ReLU+int8) ---
    sdp_zp_out   = 0
    sdp_clip_min = 0           # ReLU post-clip: 输出限 [0, 127] (symmetric int8 + ReLU)
    sdp_clip_max = 127
    sdp_round_en = 1
    sdp_relu_en  = 1

    # --- 派生 cfg ---
    cfg = hw_files.derive_layer_cfg(
        H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=Cin, NUM_COUT=Cout,
        stride=stride, pad_top=pad, pad_left=pad,
        streaming=streaming)
    H_OUT, W_OUT = cfg['H_OUT'], cfg['W_OUT']

    # --- 硬件期望输出 (用 hw_files.sdp_sim 保证和 RTL 一致) ---
    ofm_arr, _H, _W = hw_files.compute_expected_ofm(
        H_IN, W_IN, K, Cin, Cout, stride, pad, pad,
        ifm_arr, w_arr, bias_arr,
        sdp_mult=sdp_mult, sdp_shift=sdp_shift, sdp_zp_out=sdp_zp_out,
        sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
        sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en)
    assert _H == H_OUT and _W == W_OUT

    # --- 写所有文件 ---
    os.makedirs(out_dir, exist_ok=True)
    hw_files.write_ifb(out_dir, ifm_arr, H_IN, W_IN, Cin, HW_PE=16)
    hw_files.write_wb(out_dir, w_arr, bias_arr=bias_arr,
                      K=K, NUM_CIN=Cin, NUM_COUT=Cout, HW_PE=16, HW_COL=16)
    hw_files.write_expected_ofm(out_dir, ofm_arr, H_OUT, W_OUT, Cout, HW_COL=16)

    cfg_dict = hw_files.cfg_to_dict(
        cfg, shift_amt=sdp_shift,        # legacy SDP_SHIFT 字段用新的 shift
        sdp_mult=sdp_mult, sdp_zp_out=sdp_zp_out,
        sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
        sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en)
    hw_files.write_config(out_dir, cfg_dict)

    n_desc, n_strips, strip_rows_eff = hw_files.write_descriptors(
        out_dir, H_IN, W_IN, H_OUT, W_OUT, cfg['cin_slices'], cfg['cout_slices'],
        pad_top=pad, pad_bot=pad, pad_left=pad, pad_right=pad,
        strip_rows=0, streaming=streaming)

    hw_files.write_sim_params(
        out_dir, H_OUT=H_OUT, W_OUT=W_OUT, cout_slices=cfg['cout_slices'],
        ifb_words=cfg['ifb_words'], wb_words=cfg['wb_words'],
        desc_count=n_desc, sram_depth=cfg['sram_depth'],
        num_cin=Cin, num_cout=Cout)

    # --- 诊断输出 ---
    print(f"[compile_layer] {seed_label}: Conv{Cin}→{Cout} K={K} stride={stride} pad={pad} "
          f"{H_IN}x{W_IN} → {H_OUT}x{W_OUT}")
    print(f"  scales:  s_x={s_x:.5g}  s_w={s_w:.5g}  s_y={s_y:.5g}")
    print(f"  mult/shift: {sdp_mult} / {sdp_shift}  (M≈{sdp_mult/(1<<sdp_shift):.5g}, target={scale_ratio:.5g})")
    print(f"  ranges:  x_q [{x_q.min().item()},{x_q.max().item()}]  "
          f"w_q [{w_q.min().item()},{w_q.max().item()}]  "
          f"b_q [{b_q.min().item()},{b_q.max().item()}]")

    return {
        'K': K, 'stride': stride, 'pad': pad,
        'Cin': Cin, 'Cout': Cout, 'H_IN': H_IN, 'W_IN': W_IN,
        'H_OUT': H_OUT, 'W_OUT': W_OUT,
        's_x': s_x, 's_w': s_w, 's_bias': s_bias, 's_y': s_y,
        'sdp_mult': sdp_mult, 'sdp_shift': sdp_shift,
        'ofm_expected_q': ofm_arr,     # list[H][W][Cout]，已经是 clip 后 8-bit
    }


# ---------------------------------------------------------------------------
# CLI smoke
# ---------------------------------------------------------------------------
def _smoke(out_dir=DEFAULT_OUT_DIR):
    torch = _require_torch()
    torch.manual_seed(42)
    # MNIST Conv1 风格：1→8 K=3 pad=1 (same-conv)，输入 48x48 对齐硬件回归
    conv = torch.nn.Conv2d(1, 8, kernel_size=3, padding=1, bias=True)
    x = torch.randn(1, 1, 48, 48)
    s_x = estimate_activation_scale(x)
    with torch.no_grad():
        y = conv(x)
    s_y = estimate_activation_scale(y)
    info = compile_and_emit_conv2d(conv, x, s_x, s_y, out_dir=out_dir, seed_label="smoke")
    print(f"  Smoke 完成。文件已写到 {out_dir}，vsim -c -do run.tcl 验证。")


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--smoke', action='store_true', help='smoke test')
    p.add_argument('--out-dir', default=DEFAULT_OUT_DIR)
    args = p.parse_args()
    if args.smoke:
        _smoke(out_dir=args.out_dir)
    else:
        print("compile_layer.py: use --smoke, or import compile_and_emit_conv2d.")
