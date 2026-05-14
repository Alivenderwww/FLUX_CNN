#!/usr/bin/env python3
"""Dump H=33 W=25 board OFM 每 row 详细错位模式. 找 yout 31+ 错的规律."""
import sys, os, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import BRAM_BASE, DESC_OFF, BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM, CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET, chunked_load, chunked_read

case_dir = r'C:/_Project/FLUX_CNN/sim/tb_h33_dump'
H, W = 33, 25

rpc = Vd100Rpc('169.254.111.10'); rpc.connect()

# 软 reset
rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET); time.sleep(0.01)

ret = gen_isa_test.generate_random(out_dir=case_dir, ifm_arr_in=None, seed=42, shift_amt=0, streaming=False,
    H_IN=H, W_IN=W, K=3, NUM_CIN=16, NUM_COUT=16, TILE_W=25, stride=1, pad_top=1, pad_left=1)
desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
    case_dir, K=3, H_IN=H, W_IN=W, NUM_CIN=16, NUM_COUT=16, stride=1, pad=1)

# Load + run
chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
chunked_load(rpc, BRAM_ISG, isg); chunked_load(rpc, BRAM_OSG, osg)
chunked_load(rpc, BRAM_IFM, ifm); chunked_load(rpc, BRAM_WB, wb); chunked_load(rpc, BRAM_RDMA, rdma)
chunked_load(rpc, BRAM_OFM, b'\xAA' * len(exp))
rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF); rpc.poke_csr(0, 0x184, n_desc)
rpc.poke_csr(0, 0x000, CTRL_START_DFE); time.sleep(0.05)
rpc.poke_csr(0, 0x000, CTRL_START_LAYER); time.sleep(0.3)

ofm = chunked_read(rpc, BRAM_OFM, len(exp))

# row by row 对比
row_bytes = W * 16   # 25 * 16 = 400 byte/row
print(f'H_OUT={H} W_OUT={W}: total OFM = {len(exp)} byte, {len(exp)//row_bytes} row × {row_bytes} byte/row')
print()
for r in range(H):
    g = ofm[r*row_bytes:(r+1)*row_bytes]
    e = exp[r*row_bytes:(r+1)*row_bytes]
    n_diff = sum(1 for a, b in zip(g, e) if a != b)
    aa_cnt = sum(1 for b in g if b == 0xAA)
    if r in [0, 30, 31, 32] or n_diff > 0:
        print(f'row {r:2d}: diff={n_diff:3d}/400, 0xAA={aa_cnt}')
        if r in [31, 32]:
            # 详细对比每 word
            for w_idx in range(W):
                gw = g[w_idx*16:(w_idx+1)*16]
                ew = e[w_idx*16:(w_idx+1)*16]
                if gw != ew:
                    print(f'  word {w_idx:2d}: got={gw.hex()}')
                    print(f'             exp={ew.hex()}')
                    if w_idx >= 3: break  # 只看前几个

# 看 board got 的 yout 31 是不是 yout 30 / yout 29 的 copy
print()
print('=== check if yout 31 = yout N for some N ===')
g31 = ofm[31*row_bytes:(31+1)*row_bytes]
for src in range(33):
    e_src = exp[src*row_bytes:(src+1)*row_bytes]
    match = sum(1 for a, b in zip(g31, e_src) if a == b)
    if match > 300:  # 大量重叠
        print(f'  yout 31 board got vs exp yout {src}: {match}/400 byte match')

rpc.close()
