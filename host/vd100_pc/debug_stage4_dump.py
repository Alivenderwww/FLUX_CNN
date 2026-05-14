#!/usr/bin/env python3
"""Dump board OFM vs expected, side-by-side, identify mismatch pattern."""
import sys, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
from test_stage4_bitexact import build_case, BRAM_OFM, chunked_load, chunked_read, BRAM_BASE, DESC_OFF, ISG_OFF, OSG_OFF, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_ISG, BRAM_OSG, CTRL_START_DFE, CTRL_START_LAYER
import time

rpc = Vd100Rpc('169.254.111.10'); rpc.connect()

# K=1 H=W=8 Cin=Cout=16
desc, n_desc, cfg, ifm, wb, rdma, expected, isg, osg = build_case(
    8, 8, 1, 16, 16, 1, 0, r'C:/_Project/FLUX_CNN/sim/tb_stage4_case')

# Load BRAM
chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
chunked_load(rpc, BRAM_ISG, isg)
chunked_load(rpc, BRAM_OSG, osg)
chunked_load(rpc, BRAM_IFM, ifm)
chunked_load(rpc, BRAM_WB,  wb)
chunked_load(rpc, BRAM_RDMA, rdma)

# Clear OFM
chunked_load(rpc, BRAM_OFM, b'\x00' * len(expected))

# Run
rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
rpc.poke_csr(0, 0x184, n_desc)
rpc.poke_csr(0, 0x000, CTRL_START_DFE); time.sleep(0.1)
rpc.poke_csr(0, 0x000, CTRL_START_LAYER); time.sleep(0.3)

ofm = chunked_read(rpc, BRAM_OFM, len(expected))

# Compare line by line (16 byte / line)
print(f'OFM 比对 (16 byte/word, {len(expected)//16} word):')
print(f'{"word":>4}  {"got (board)":<48}  {"expected":<48}  err')
for i in range(len(expected) // 16):
    g = ofm[i*16:(i+1)*16]
    e = expected[i*16:(i+1)*16]
    n_diff = sum(1 for a, b in zip(g, e) if a != b)
    print(f'  {i:>3}  {g.hex():<48}  {e.hex():<48}  {n_diff}')

rpc.close()
