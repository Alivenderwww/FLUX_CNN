"""manifest.py — case/ 目录元数据 (后端 IR).

Phase B: 替代 multicore_meta.txt 的 KV 文本格式. 老格式仍生成 (sim 兼容),
manifest.json 是 host 端首选 source.

Schema:
{
  "schema_version": "1.0",
  "toolchain_commit": "<git sha>",
  "hardware_cfg_source": "vd100.json",
  "n_cores": 1,
  "n_layers": 1,
  "ddr_global_base": "0x10000000",
  "layers": [
    {
      "idx": 0, "name": "L0",
      "k": 3, "c_in": 16, "c_out": 16, "h_in": 32, "w_in": 75,
      "stride": 1, "pad": 1,
      "h_out": 32, "w_out": 75,
      "smc_layout": {
        "core_0": {
          "ifb_base": "0x10D00000", "ofm_base": "0x10F00000",
          "wb_base":  "0x10800000", "rdma_base": "0x10900000",
          "desc_base": "0x10A00000", "desc_count": 63,
          "idma_cmd_base": "0x10B00000", "odma_cmd_base": "0x10C00000"
        }
      },
      "hw_cfg": {
        "IFB_STRIP_ROWS": 6, "IFB_RING_WORDS": 450, "KK": 9
      },
      "data_files": {
        "ifb":  "chain_data/layer00/ifb.txt",
        "wb":   "chain_data/layer00/wb.txt",
        "rdma": "chain_data/layer00/rdma_data.txt",
        "expected_ofm": "chain_data/layer00/expected_ofm.txt"
      }
    }
  ],
  "deploy": {
    "desc_list_base_per_core": ["0x10A00000"],
    "desc_count_per_core": [63],
    "final_ofm_base": "0x10F00000"
  }
}
"""
from __future__ import annotations

import json
import os
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional


SCHEMA_VERSION = "1.0"


def _git_sha(cwd: Optional[str] = None) -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=cwd, stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


def _hex(v) -> str:
    if isinstance(v, str):
        return v
    return f"0x{v:08X}"


@dataclass
class LayerSmcLayout:
    """Per-(layer, core) SMC 地址分配."""
    ifb_base:       int = 0
    ofm_base:       int = 0
    wb_base:        int = 0
    rdma_base:      int = 0
    desc_base:      int = 0
    desc_count:     int = 0
    idma_cmd_base:  int = 0
    odma_cmd_base:  int = 0

    def to_dict(self) -> dict:
        return {
            'ifb_base':      _hex(self.ifb_base),
            'ofm_base':      _hex(self.ofm_base),
            'wb_base':       _hex(self.wb_base),
            'rdma_base':     _hex(self.rdma_base),
            'desc_base':     _hex(self.desc_base),
            'desc_count':    int(self.desc_count),
            'idma_cmd_base': _hex(self.idma_cmd_base),
            'odma_cmd_base': _hex(self.odma_cmd_base),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "LayerSmcLayout":
        def p(k, default=0):
            v = d.get(k, default)
            return int(v, 0) if isinstance(v, str) else int(v)
        return cls(
            ifb_base=p('ifb_base'), ofm_base=p('ofm_base'),
            wb_base=p('wb_base'), rdma_base=p('rdma_base'),
            desc_base=p('desc_base'), desc_count=p('desc_count'),
            idma_cmd_base=p('idma_cmd_base'), odma_cmd_base=p('odma_cmd_base'),
        )


@dataclass
class LayerManifest:
    idx:        int
    name:       str
    k:          int
    c_in:       int
    c_out:      int
    h_in:       int
    w_in:       int
    stride:     int
    pad:        int
    h_out:      int
    w_out:      int
    smc_layout: Dict[int, LayerSmcLayout] = field(default_factory=dict)   # core_id -> layout
    hw_cfg:     dict = field(default_factory=dict)                       # derive 后硬件 cfg
    data_files: dict = field(default_factory=dict)                       # 数据文件相对路径
    extra:      dict = field(default_factory=dict)                       # 跨层 / mode-specific 字段

    def to_dict(self) -> dict:
        return {
            'idx': self.idx, 'name': self.name,
            'k': self.k, 'c_in': self.c_in, 'c_out': self.c_out,
            'h_in': self.h_in, 'w_in': self.w_in,
            'stride': self.stride, 'pad': self.pad,
            'h_out': self.h_out, 'w_out': self.w_out,
            'smc_layout': {f'core_{c}': v.to_dict() for c, v in self.smc_layout.items()},
            'hw_cfg': self.hw_cfg,
            'data_files': self.data_files,
            **({'extra': self.extra} if self.extra else {}),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "LayerManifest":
        smc = {}
        for k, v in d.get('smc_layout', {}).items():
            cid = int(k.split('_')[1]) if k.startswith('core_') else int(k)
            smc[cid] = LayerSmcLayout.from_dict(v)
        return cls(
            idx=int(d['idx']), name=d['name'],
            k=int(d['k']), c_in=int(d['c_in']), c_out=int(d['c_out']),
            h_in=int(d['h_in']), w_in=int(d['w_in']),
            stride=int(d['stride']), pad=int(d['pad']),
            h_out=int(d['h_out']), w_out=int(d['w_out']),
            smc_layout=smc, hw_cfg=d.get('hw_cfg', {}),
            data_files=d.get('data_files', {}),
            extra=d.get('extra', {}),
        )


@dataclass
class Manifest:
    n_cores:           int
    n_layers:          int
    ddr_global_base:   int
    hardware_cfg_source: str = ""
    layers:            List[LayerManifest] = field(default_factory=list)
    # Deploy 端用 (host RPC 直接读这些, 不用扫 layers)
    desc_list_base_per_core: List[int] = field(default_factory=list)
    desc_count_per_core:     List[int] = field(default_factory=list)
    final_ofm_base:    int = 0
    # Metadata
    schema_version:    str = SCHEMA_VERSION
    toolchain_commit:  str = ""

    def to_dict(self) -> dict:
        return {
            'schema_version': self.schema_version,
            'toolchain_commit': self.toolchain_commit,
            'hardware_cfg_source': self.hardware_cfg_source,
            'n_cores': self.n_cores,
            'n_layers': self.n_layers,
            'ddr_global_base': _hex(self.ddr_global_base),
            'layers': [l.to_dict() for l in self.layers],
            'deploy': {
                'desc_list_base_per_core': [_hex(v) for v in self.desc_list_base_per_core],
                'desc_count_per_core': list(self.desc_count_per_core),
                'final_ofm_base': _hex(self.final_ofm_base),
            },
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Manifest":
        def p(v):
            return int(v, 0) if isinstance(v, str) else int(v)
        dep = d.get('deploy', {})
        return cls(
            n_cores=int(d['n_cores']), n_layers=int(d['n_layers']),
            ddr_global_base=p(d['ddr_global_base']),
            hardware_cfg_source=d.get('hardware_cfg_source', ''),
            layers=[LayerManifest.from_dict(x) for x in d.get('layers', [])],
            desc_list_base_per_core=[p(v) for v in dep.get('desc_list_base_per_core', [])],
            desc_count_per_core=[int(v) for v in dep.get('desc_count_per_core', [])],
            final_ofm_base=p(dep.get('final_ofm_base', 0)),
            schema_version=d.get('schema_version', SCHEMA_VERSION),
            toolchain_commit=d.get('toolchain_commit', ''),
        )

    def write_json(self, path: str, indent: int = 2):
        if not self.toolchain_commit:
            self.toolchain_commit = _git_sha(os.path.dirname(path))
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=indent, ensure_ascii=False)

    @classmethod
    def load_json(cls, path: str) -> "Manifest":
        with open(path, 'r', encoding='utf-8') as f:
            return cls.from_dict(json.load(f))


if __name__ == '__main__':
    # Smoke test: round-trip
    m = Manifest(
        n_cores=1, n_layers=1, ddr_global_base=0x10000000,
        hardware_cfg_source='vd100.json',
        layers=[LayerManifest(
            idx=0, name='L0', k=3, c_in=16, c_out=16, h_in=32, w_in=75,
            stride=1, pad=1, h_out=32, w_out=75,
            smc_layout={0: LayerSmcLayout(
                ifb_base=0x10D00000, ofm_base=0x10F00000,
                wb_base=0x10800000, rdma_base=0x10900000,
                desc_base=0x10A00000, desc_count=63,
                idma_cmd_base=0x10B00000, odma_cmd_base=0x10C00000)},
            hw_cfg={'IFB_STRIP_ROWS': 6, 'IFB_RING_WORDS': 450, 'KK': 9},
            data_files={'ifb': 'chain_data/layer00/ifb.txt'},
        )],
        desc_list_base_per_core=[0x10A00000],
        desc_count_per_core=[63],
        final_ofm_base=0x10F00000,
    )
    import tempfile, os
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        path = f.name
    m.write_json(path)
    m2 = Manifest.load_json(path)
    assert m2.layers[0].smc_layout[0].ifb_base == 0x10D00000
    assert m2.desc_list_base_per_core == [0x10A00000]
    print(f"Round-trip OK: {path}")
    print(json.dumps(m2.to_dict(), indent=2)[:600])
    os.unlink(path)
