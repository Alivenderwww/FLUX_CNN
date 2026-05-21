"""[Phase C] frontend.dsl_loader — Chain DSL YAML 加载/保存.

Phase C: 包装 scheduler.Layer + 双向序列化 (Python list ↔ YAML).
未来: YAML schema 校验, 量化 scale 字段, weights .npy 引用.

Usage:
    from toolchain.frontend.dsl_loader import load_chain_dsl, dump_chain_dsl
    layers = load_chain_dsl('chains/resnet11.yaml')
    dump_chain_dsl(layers, 'chains/out.yaml')
"""
import os as _os
import sys as _sys
from dataclasses import asdict
from typing import List

_TOOLCHAIN_DIR = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _TOOLCHAIN_DIR not in _sys.path:
    _sys.path.insert(0, _TOOLCHAIN_DIR)

from midend.scheduler import Layer


SCHEMA_VERSION = "1.0"


def dump_chain_dsl(layers: List[Layer], path: str, model_name: str = ""):
    """List[Layer] → YAML. 失败时尝试用 stdlib (无 yaml dep 时降级 JSON)."""
    payload = {
        'schema_version': SCHEMA_VERSION,
        'model_name': model_name,
        'layers': [_layer_to_dict(l) for l in layers],
    }
    try:
        import yaml
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
    except ImportError:
        # fallback JSON (大部分场景 Python 自带 json)
        import json
        # 把 yaml 路径改成 .json
        json_path = _os.path.splitext(path)[0] + '.json'
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2)
        print(f"[WARN] PyYAML 没装, 改用 JSON: {json_path}")
        path = json_path
    return path


def load_chain_dsl(path: str) -> List[Layer]:
    """YAML / JSON → List[Layer]. 自动按后缀识别格式."""
    if path.endswith('.json'):
        import json
        with open(path, 'r', encoding='utf-8') as f:
            payload = json.load(f)
    else:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            payload = yaml.safe_load(f)
    layers = []
    for d in payload.get('layers', []):
        layers.append(_dict_to_layer(d))
    return layers


def _layer_to_dict(l: Layer) -> dict:
    """Layer → plain dict (跳过默认值减少 YAML 大小)."""
    full = asdict(l)
    defaults = Layer(name='_', k=0, c_in=0, c_out=0, h_in=0, w_in=0)
    out = {}
    for k, v in full.items():
        if k in {'name', 'k', 'c_in', 'c_out', 'h_in', 'w_in'}:
            out[k] = v
        elif v != getattr(defaults, k, None):
            out[k] = v
    return out


def _dict_to_layer(d: dict) -> Layer:
    """dict → Layer (用 Layer 的 dataclass 字段名一对一)."""
    return Layer(**d)


if __name__ == '__main__':
    # smoke test: round-trip
    layers = [
        Layer(name='L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, pad=1),
        Layer(name='L1', k=3, c_in=16, c_out=16, h_in=32, w_in=32, pad=1, input_src='L0'),
    ]
    import tempfile
    fd = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    fd.close()
    path = fd.name
    out = dump_chain_dsl(layers, path, model_name='smoke_test')
    layers2 = load_chain_dsl(out)
    assert len(layers2) == 2
    assert layers2[0].name == 'L0'
    assert layers2[1].input_src == 'L0'
    print(f"Round-trip OK: {out}")
    _os.unlink(out)
