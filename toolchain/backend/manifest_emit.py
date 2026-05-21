"""[Phase C] backend.manifest_emit — case manifest.json 写出.

Phase B 已经在 run_multicore_chain._emit_manifest_json 实现了核心逻辑.
此处暴露 Manifest 类 + emit helper, 让新代码可以独立调用 (Phase D SMC allocator 用).
"""
import os as _os
import sys as _sys
_TOOLCHAIN_DIR = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _TOOLCHAIN_DIR not in _sys.path:
    _sys.path.insert(0, _TOOLCHAIN_DIR)
from ir.manifest import (  # noqa: F401
    Manifest, LayerManifest, LayerSmcLayout, SCHEMA_VERSION,
)


def write_case_manifest(case_dir: str, manifest: Manifest) -> str:
    """写 case_dir/manifest.json. 返回路径."""
    path = _os.path.join(case_dir, 'manifest.json')
    manifest.write_json(path)
    return path
