"""quantize.py — 独立量化 stage (Phase E).

把 PyTorch .pt model + calibration tensor 解耦量化, 输出 .quant.yaml + weights.npz.
quant.yaml 可被后续 compile.py 直接消费, 不需要再次跑 calibration.

Output schema:
  quant.yaml (or .json):
    schema_version: "1.0"
    model_name: mnist_allconv
    input_scale:  float       # 整网入口激活 scale (s_in)
    output_scale: float       # 最末层激活 scale (s_out)
    layers:
      - idx: 0
        name: conv0
        k: 3
        c_in: 1
        c_out: 16
        stride: 1
        pad: 1
        has_relu: true
        s_x: 0.01234            # layer 输入激活 scale
        s_y: 0.0567             # layer 输出激活 scale
        s_w: 0.00345            # 权重 scale
        s_b: 4.26e-5            # bias scale = s_x * s_w (derived)
        weight_npy: weights/L0_weight.npy   # int8 (cout, cin, k, k)
        bias_npy:   weights/L0_bias.npy     # int32 (cout,)
      - ...

Usage:
    python -m toolchain.frontend.quantize \\
        --model models/ckpts/mnist_allconv.pt \\
        --calib-image images/mnist_test/0000_label7.png \\
        --out models/mnist_allconv.quant.yaml
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, asdict
from typing import List, Optional

_FRONTEND_DIR = os.path.dirname(os.path.abspath(__file__))
_TOOLCHAIN_DIR = os.path.dirname(_FRONTEND_DIR)
if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)
_MODELS_DIR = os.path.join(_TOOLCHAIN_DIR, 'models')
if _MODELS_DIR not in sys.path:
    sys.path.insert(0, _MODELS_DIR)


SCHEMA_VERSION = "1.0"


@dataclass
class QuantLayer:
    idx:        int
    name:       str
    k:          int
    c_in:       int
    c_out:      int
    stride:     int
    pad:        int
    has_relu:   bool
    s_x:        float
    s_y:        float
    s_w:        float
    s_b:        float            # = s_x * s_w
    weight_npy: str = ''         # 相对 yaml 文件路径
    bias_npy:   str = ''


@dataclass
class QuantizedModel:
    schema_version: str
    model_name:     str
    input_scale:    float
    output_scale:   float
    layers:         List[QuantLayer]

    def to_dict(self) -> dict:
        return {
            'schema_version': self.schema_version,
            'model_name': self.model_name,
            'input_scale': self.input_scale,
            'output_scale': self.output_scale,
            'layers': [asdict(l) for l in self.layers],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "QuantizedModel":
        return cls(
            schema_version=d.get('schema_version', SCHEMA_VERSION),
            model_name=d.get('model_name', ''),
            input_scale=float(d['input_scale']),
            output_scale=float(d['output_scale']),
            layers=[QuantLayer(**l) for l in d.get('layers', [])],
        )


# =============================================================================
# Calibrate + Quantize
# =============================================================================
def calibrate_and_quantize(model_pt: str, calib_tensor=None, calib_image_path: Optional[str] = None,
                           model_builder=None, model_name: str = '') -> tuple:
    """从 .pt 加载 model + calib data, 跑 calibration, 返回 (QuantizedModel, weight_dict).

    weight_dict[layer_name] = {'weight': int8 np array, 'bias': int32 np array}

    Args:
        model_pt: .pt 文件 (state_dict).
        calib_tensor: torch.Tensor 形状 (1, C, H, W); 优先于 calib_image_path.
        calib_image_path: PIL 可读图; 自动跟 model input shape 匹配.
        model_builder: 用来构造空 nn.Module 的 callable, 默认从 zoo 取 (按 model_name).
        model_name: zoo 里注册的 model 名 (e.g. 'mnist_allconv').
    """
    from backend import data_emit as compile_layer
    import models.compile_model as cm
    torch = compile_layer._require_torch()

    # 加载 model
    if model_builder is None:
        from models import zoo
        if model_name not in zoo.MODELS:
            raise ValueError(f"Model name '{model_name}' 不在 zoo.MODELS: {list(zoo.MODELS)}")
        model, default_calib, _ = zoo.MODELS[model_name]()
    else:
        model, default_calib, _ = model_builder()
    # 加载 state_dict 到 model
    sd = torch.load(model_pt, map_location='cpu')
    if isinstance(sd, dict) and 'state_dict' in sd:
        sd = sd['state_dict']
    # 自动剥前缀 (zoo 有的模型 state_dict key 含 'net.' 前缀)
    try:
        model.load_state_dict(sd, strict=True)
    except RuntimeError:
        sd2 = {k.replace('net.', ''): v for k, v in sd.items()}
        model.load_state_dict(sd2, strict=False)
    model.eval()

    # 选 calib 数据
    if calib_tensor is None:
        if calib_image_path:
            from PIL import Image
            import numpy as np
            img = Image.open(calib_image_path).convert('L' if default_calib.shape[1] == 1 else 'RGB')
            img = img.resize((default_calib.shape[3], default_calib.shape[2]))
            arr = np.asarray(img, dtype='float32') / 255.0
            if arr.ndim == 2:
                arr = arr[None, :, :]
            else:
                arr = arr.transpose(2, 0, 1)
            calib_tensor = torch.from_numpy(arr).unsqueeze(0)
        else:
            calib_tensor = default_calib

    # 跑 calibration
    conv_chain, scales, _acts = cm._calibrate_activations(model, calib_tensor)

    # 量化每层 weights + bias
    weight_dict = {}
    qlayers = []
    for k, (conv, has_relu) in enumerate(conv_chain):
        s_x = scales[k]
        s_y = scales[k + 1]
        # weight scale
        w = conv.weight.data
        s_w = float(w.abs().max().item() / 127.0) if w.abs().max() > 0 else 1.0
        # quantize
        w_q = compile_layer.quantize_symmetric(w, s_w).to(torch.int8).cpu().numpy()
        if conv.bias is not None:
            s_b = s_x * s_w
            b_q = compile_layer.quantize_symmetric(
                conv.bias.data, s_b, qmin=-(1 << 31), qmax=(1 << 31) - 1
            ).to(torch.int32).cpu().numpy()
        else:
            s_b = 0.0
            import numpy as np
            b_q = np.zeros(conv.out_channels, dtype='int32')

        cout, cin, kh, kw = w_q.shape
        name = f'conv{k}'
        weight_dict[name] = {'weight': w_q, 'bias': b_q}
        qlayers.append(QuantLayer(
            idx=k, name=name, k=int(kh), c_in=int(cin), c_out=int(cout),
            stride=int(conv.stride[0]), pad=int(conv.padding[0]),
            has_relu=bool(has_relu),
            s_x=float(s_x), s_y=float(s_y), s_w=float(s_w), s_b=float(s_b),
        ))

    qmodel = QuantizedModel(
        schema_version=SCHEMA_VERSION,
        model_name=model_name,
        input_scale=float(scales[0]),
        output_scale=float(scales[-1]),
        layers=qlayers,
    )
    return qmodel, weight_dict


# =============================================================================
# Save / Load
# =============================================================================
def save_quantized(qmodel: QuantizedModel, weight_dict: dict, out_path: str):
    """写 quant.yaml + weights/L?_weight.npy / L?_bias.npy."""
    import numpy as np
    out_dir = os.path.dirname(os.path.abspath(out_path))
    weights_dir = os.path.join(out_dir, 'weights')
    os.makedirs(weights_dir, exist_ok=True)

    # 写 npy 引用
    for l in qmodel.layers:
        wd = weight_dict[l.name]
        w_path = os.path.join(weights_dir, f'L{l.idx}_{l.name}_weight.npy')
        b_path = os.path.join(weights_dir, f'L{l.idx}_{l.name}_bias.npy')
        np.save(w_path, wd['weight'])
        np.save(b_path, wd['bias'])
        l.weight_npy = os.path.relpath(w_path, out_dir).replace('\\', '/')
        l.bias_npy   = os.path.relpath(b_path, out_dir).replace('\\', '/')

    payload = qmodel.to_dict()
    if out_path.endswith('.json'):
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
    else:
        try:
            import yaml
            with open(out_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
        except ImportError:
            # fallback JSON
            json_path = os.path.splitext(out_path)[0] + '.json'
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            print(f"[WARN] PyYAML 没装, 用 JSON 输出: {json_path}")
            out_path = json_path

    return out_path


def load_quantized(path: str) -> tuple:
    """加载 quant.yaml/.json → (QuantizedModel, weight_dict)."""
    import numpy as np
    in_dir = os.path.dirname(os.path.abspath(path))
    if path.endswith('.json'):
        with open(path, 'r', encoding='utf-8') as f:
            d = json.load(f)
    else:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            d = yaml.safe_load(f)
    qmodel = QuantizedModel.from_dict(d)
    weight_dict = {}
    for l in qmodel.layers:
        w_path = os.path.join(in_dir, l.weight_npy)
        b_path = os.path.join(in_dir, l.bias_npy)
        weight_dict[l.name] = {
            'weight': np.load(w_path),
            'bias':   np.load(b_path),
        }
    return qmodel, weight_dict


# =============================================================================
# CLI
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description="独立量化 stage: model.pt + calib → quant.yaml")
    ap.add_argument('--model', required=True, help='zoo 名 (e.g. mnist_allconv) 或 .pt 路径')
    ap.add_argument('--ckpt', default=None, help='.pt state_dict 路径; --model 是 zoo 名时必填')
    ap.add_argument('--calib-image', default=None, help='calib 图像路径 (PNG, 默认 zoo 自带 calib_tensor)')
    ap.add_argument('--out', required=True, help='quant.yaml 输出路径')
    args = ap.parse_args()

    # 处理 model 参数: 如果是文件路径用 ckpt, 否则当 zoo 名
    if os.path.isfile(args.model):
        ckpt = args.model
        # 从 ckpt 推 model name (按文件名)
        model_name = os.path.splitext(os.path.basename(ckpt))[0]
    else:
        model_name = args.model
        ckpt = args.ckpt
        if ckpt is None:
            # zoo 默认 ckpt 路径
            ckpt_dir = os.path.join(_MODELS_DIR, 'ckpts')
            for ext in ('.pt', '.pth'):
                p = os.path.join(ckpt_dir, model_name + ext)
                if os.path.exists(p):
                    ckpt = p
                    break
            if ckpt is None:
                ap.error(f"找不到 ckpt: 试过 {ckpt_dir}/{model_name}.pt[.pth], 用 --ckpt 显式指定")
    print(f"[quantize] model={model_name} ckpt={ckpt} calib_image={args.calib_image}")

    qmodel, weights = calibrate_and_quantize(
        model_pt=ckpt,
        calib_image_path=args.calib_image,
        model_name=model_name,
    )
    out_path = save_quantized(qmodel, weights, args.out)
    print(f"[quantize] 写出: {out_path}")
    print(f"  - {len(qmodel.layers)} layer")
    print(f"  - input_scale  = {qmodel.input_scale:.6f}")
    print(f"  - output_scale = {qmodel.output_scale:.6f}")
    print(f"  - weights/ dir 含 {2 * len(qmodel.layers)} npy 文件")


if __name__ == '__main__':
    main()
