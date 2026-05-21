"""hardware_cfg.py — Hardware JSON loader.

加载 hardware/*.json → HardwareConfig dataclass. 编译链所有阶段共用.

Schema 见 vd100.json (含 _comment 字段).

Usage:
    from toolchain.hardware.hardware_cfg import HardwareConfig
    cfg = HardwareConfig.load('toolchain/hardware/vd100.json')
    print(cfg.arch.num_pe, cfg.deployment.regions.input_base)
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict


def _parse_int(v):
    """支持 int / "0x..." / "0b..." / "123" 三种格式."""
    if isinstance(v, int):
        return v
    if isinstance(v, str):
        return int(v, 0)
    raise TypeError(f"无法解析为 int: {v!r}")


@dataclass
class ArchCfg:
    num_pe:           int
    num_col:          int
    data_width:       int
    psum_width:       int
    wrf_depth:        int
    arf_depth:        int
    parf_depth:       int
    ifb_depth:        int
    wb_depth:         int
    ofb_depth:        int
    shortcut_depth:   int
    bus_addr_width:   int
    bus_data_width:   int
    axi_m_id:         int
    axi_m_width:      int
    dma_len_width:    int
    csr_addr_width:   int
    csr_data_width:   int

    @property
    def core_bus_id(self) -> int:
        return self.axi_m_id + self.axi_m_width

    @property
    def ifb_word_bytes(self) -> int:
        return self.num_pe * self.data_width // 8

    @property
    def wb_word_bytes(self) -> int:
        return self.num_col * self.num_pe * self.data_width // 8

    @property
    def ofb_word_bytes(self) -> int:
        return self.num_col * self.data_width // 8

    @classmethod
    def from_dict(cls, d: dict) -> "ArchCfg":
        d = {k: v for k, v in d.items() if not k.startswith('_')}
        return cls(**d)


@dataclass
class Regions:
    ifm_ofm_base:           int
    layer_data_offset:      int
    wb_base:                int
    layer_wb_offset:        int
    rdma_base:              int
    layer_rdma_offset:      int
    desc_base:              int
    layer_desc_offset:      int
    idma_cmd_base:          int
    layer_idma_cmd_offset:  int
    odma_cmd_base:          int
    layer_odma_cmd_offset:  int
    input_base:             int
    layer_input_offset:     int
    final_ofm_base:         int

    @classmethod
    def from_dict(cls, d: dict) -> "Regions":
        d = {k: _parse_int(v) for k, v in d.items() if not k.startswith('_')}
        return cls(**d)


@dataclass
class DeploymentCfg:
    target:           str
    num_cores:        int
    ddr_base:         int
    ddr_size_mb:      int
    smc_mem_stride:   int
    regions:          Regions
    csr_axil:         Dict[int, int] = field(default_factory=dict)   # core_id -> base addr

    @classmethod
    def from_dict(cls, d: dict) -> "DeploymentCfg":
        regions = Regions.from_dict(d['regions'])
        csr_axil = {}
        for k, v in d.get('csr_axil', {}).items():
            if k.startswith('_'):
                continue
            if k.startswith('core_'):
                core_id = int(k.split('_')[1])
            else:
                core_id = int(k)
            csr_axil[core_id] = _parse_int(v)
        return cls(
            target=d['target'],
            num_cores=int(d['num_cores']),
            ddr_base=_parse_int(d['ddr_base']),
            ddr_size_mb=int(d['ddr_size_mb']),
            smc_mem_stride=_parse_int(d['smc_mem_stride']),
            regions=regions,
            csr_axil=csr_axil,
        )

    def smc_addr(self, mem_id: int, region_offset: int) -> int:
        """global byte addr = ddr_base + mem_id * smc_mem_stride + region_offset."""
        return self.ddr_base + mem_id * self.smc_mem_stride + region_offset


@dataclass
class HardwareConfig:
    arch:        ArchCfg
    deployment:  DeploymentCfg
    _source_path: str = ""

    @classmethod
    def load(cls, json_path: str) -> "HardwareConfig":
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        arch = ArchCfg.from_dict(data['arch'])
        deployment = DeploymentCfg.from_dict(data['deployment'])
        return cls(arch=arch, deployment=deployment, _source_path=os.path.abspath(json_path))


if __name__ == '__main__':
    import sys
    p = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'vd100.json')
    cfg = HardwareConfig.load(p)
    print(f"Loaded: {p}")
    print(f"  arch.num_pe = {cfg.arch.num_pe}")
    print(f"  arch.ifb_depth = {cfg.arch.ifb_depth}")
    print(f"  arch.core_bus_id (derived) = {cfg.arch.core_bus_id}")
    print(f"  deployment.target = {cfg.deployment.target}")
    print(f"  deployment.num_cores = {cfg.deployment.num_cores}")
    print(f"  deployment.ddr_base = 0x{cfg.deployment.ddr_base:08x}")
    print(f"  deployment.regions.input_base = 0x{cfg.deployment.regions.input_base:08x}")
    print(f"  deployment.regions.final_ofm_base = 0x{cfg.deployment.regions.final_ofm_base:08x}")
    print(f"  deployment.csr_axil[0] = 0x{cfg.deployment.csr_axil[0]:08x}")
