"""pytorch_to_smc.py — PyTorch nn.Module → scheduler.Layer + 真权重 (SMC chain 用)

跟 frontend/pytorch_to_dsl.py 的区别:
  - pytorch_to_dsl: PyTorch → tb_core_dma 单层 case (sim only, 不上板)
  - pytorch_to_smc: PyTorch → scheduler.Layer list + 真权重/bias dict
                    供 run_multicore_chain.py 走 SMC chain 路径

复用 pytorch_to_dsl 的 conv chain 提取 + activation calibration; 量化 SDP 参数
跟 backend.data_emit.derive_mult_shift / compute_weight_scale / quantize_symmetric
一致 (RTL bit-exact).

API:
    layers, weights_per_layer, biases_per_layer = pytorch_to_smc_chain(
        model, x_calib_tensor)

    layers              : list[scheduler.Layer]   网络结构 + SDP 量化参数
    weights_per_layer   : list[w_arr_4d]          每层 [K][K][NUM_COUT][NUM_CIN] int8
    biases_per_layer    : list[bias_arr_1d]       每层 [NUM_COUT] int32

供 run_multicore_chain.py --pt 入口调用.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_TOOLCHAIN_DIR = os.path.dirname(_THIS_DIR)
if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)

from frontend.pytorch_to_dsl import _extract_conv_chain, _calibrate_activations  # noqa: E402
from backend import data_emit                                                     # noqa: E402
from midend import scheduler                                                      # noqa: E402


def _quantize_one_conv(conv, s_x, s_y, relu_en, layer_name, torch):
    """单层 conv: 真权重/bias int8 量化 + 算 SDP 参数. 返回 (Layer, w_arr_4d, bias_arr_1d)."""
    # 网络结构
    assert conv.kernel_size[0] == conv.kernel_size[1], f"{layer_name}: 只支持方形 kernel"
    assert conv.stride[0] == conv.stride[1], f"{layer_name}: 只支持方形 stride"
    assert conv.padding[0] == conv.padding[1], f"{layer_name}: 只支持对称 pad"
    K = conv.kernel_size[0]
    stride = conv.stride[0]
    pad = conv.padding[0]
    Cin = conv.in_channels
    Cout = conv.out_channels

    # 权重 + bias 量化
    w_float = conv.weight.data.clone()
    b_float = (conv.bias.data.clone() if conv.bias is not None
                else torch.zeros(Cout))
    s_w = data_emit.compute_weight_scale(w_float)
    s_bias = s_x * s_w
    scale_ratio = s_bias / s_y if s_y > 0 else 0.0
    sdp_mult, sdp_shift = data_emit.derive_mult_shift(scale_ratio)
    w_q = data_emit.quantize_symmetric(w_float, s_w)
    b_q = torch.round(b_float / s_bias).to(torch.int32) if s_bias > 0 else \
          torch.zeros(Cout, dtype=torch.int32)

    # 转 Python list (跟 gen_isa_test 期望格式一致)
    w_arr = data_emit._tensor_to_list4_weight(w_q)
    bias_arr = data_emit._tensor_to_list_bias(b_q)

    # 构造 Layer DSL (网络结构 + SDP 量化参数)
    # relu_en=1: post-ReLU → clip [0, 127] 非负 int8 作下层输入
    # relu_en=0: 无 ReLU    → clip [-128, 127] (输出 logits, 如分类末层)
    layer = scheduler.Layer(
        name=layer_name,
        k=K, c_in=Cin, c_out=Cout,
        h_in=0, w_in=0,    # caller 填 (从 calib activation shape)
        stride=stride, pad=pad,
        sdp_shift=sdp_shift,
        sdp_mult=sdp_mult,
        sdp_zp_out=0,
        sdp_clip_min=(0 if relu_en else -128),
        sdp_clip_max=127,
        sdp_round_en=1,
        sdp_relu_en=(1 if relu_en else 0),
    )
    return layer, w_arr, bias_arr


def pytorch_to_smc_chain(model, x_calib):
    """PyTorch model + calib 输入 → (layers, weights_list, biases_list).

    layers          : list[scheduler.Layer]      h_in/w_in 自动按 calib activation 填
    weights_list    : list[w_arr_4d]             每层量化好的 [K][K][Cout][Cin] int8
    biases_list     : list[bias_arr_1d]          每层 [Cout] int32

    供 run_multicore_chain.run_chain_data_gen 用 wb_arr_in/bias_arr_in 透传.
    """
    torch = data_emit._require_torch()
    # 1. 提取 conv chain + relu 标志
    conv_chain = _extract_conv_chain(model)   # [(conv, relu_en), ...]
    # 2. calibrate → 每层输入 activation + scales
    _, scales, acts = _calibrate_activations(model, x_calib)
    L = len(conv_chain)
    assert len(scales) == L + 1, f"scales={len(scales)} vs convs={L}+1"
    assert len(acts) == L + 1

    layers = []
    weights = []
    biases = []
    for k, (conv, relu_en) in enumerate(conv_chain):
        s_x = scales[k]       # 本层输入 scale
        s_y = scales[k + 1]   # 本层输出 scale
        layer, w_arr, bias_arr = _quantize_one_conv(
            conv, s_x, s_y, relu_en, f"L{k}", torch)
        # 填 H_IN/W_IN
        act = acts[k]   # 本层输入 tensor (1, Cin, H, W)
        layer.h_in = int(act.shape[2])
        layer.w_in = int(act.shape[3])
        layers.append(layer)
        weights.append(w_arr)
        biases.append(bias_arr)

    return layers, weights, biases, scales[0]   # 第 0 个 = layer 0 input scale (s_x)


def load_calib_input(model_name, calib_dir, limit=10):
    """从 calib 目录读 PNG, 拼成 1×Cin×H×W tensor.

    简单实现: 用 MNIST normalize (mean=0.1307 std=0.3081). 后续要支持
    其他 input pipeline 时, 走 manifest 里的 preprocess 字段.
    """
    import glob
    from PIL import Image
    torch = data_emit._require_torch()
    from torchvision import transforms

    files = sorted(glob.glob(os.path.join(calib_dir, '*.png')))
    if limit > 0:
        files = files[:limit]
    if not files:
        raise ValueError(f"calib dir {calib_dir} 没找到 .png 文件")
    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    tensors = []
    for f in files:
        img = Image.open(f).convert('L')
        tensors.append(tfm(img))
    x = torch.stack(tensors, dim=0)   # (B, 1, H, W)
    return x
