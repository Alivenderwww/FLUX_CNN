#!/usr/bin/env python3
"""H=33 实时 trace dispatcher state, 看 cmd 31 → cmd 32 转换路径."""
import sys, time, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import (BRAM_BASE, DESC_OFF, BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM,
                                    CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET, chunked_load, chunked_read)

ODMA_STATES = {0:'IDLE', 1:'WAIT', 2:'FETCH_ISS', 3:'FETCH_DAT', 5:'CMD', 6:'PREFETCH', 7:'TX', 8:'STS', 9:'DONE'}
IDMA_STATES = {0:'IDLE', 1:'F_ISS', 2:'F_DAT', 4:'RING_WAIT', 5:'ISSUE', 6:'DATA', 9:'DONE'}
SEQ_STATES = {0:'IDLE', 1:'FETCH', 2:'PRELOAD', 3:'DISPATCH', 4:'WAIT', 5:'BARRIER', 6:'END'}

case_dir = r'C:/_Project/FLUX_CNN/sim/tb_h33_trace'
H, W = 33, 25

rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET); time.sleep(0.01)

ret = gen_isa_test.generate_random(out_dir=case_dir, ifm_arr_in=None, seed=42, shift_amt=0, streaming=False,
    H_IN=H, W_IN=W, K=3, NUM_CIN=16, NUM_COUT=16, TILE_W=25, stride=1, pad_top=1, pad_left=1)
desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
    case_dir, K=3, H_IN=H, W_IN=W, NUM_CIN=16, NUM_COUT=16, stride=1, pad=1)

chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
chunked_load(rpc, BRAM_ISG, isg); chunked_load(rpc, BRAM_OSG, osg)
chunked_load(rpc, BRAM_IFM, ifm); chunked_load(rpc, BRAM_WB, wb); chunked_load(rpc, BRAM_RDMA, rdma)
chunked_load(rpc, BRAM_OFM, b'\xAA' * len(exp))
rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF); rpc.poke_csr(0, 0x184, n_desc)

rpc.poke_csr(0, 0x000, CTRL_START_DFE); time.sleep(0.05)
rpc.poke_csr(0, 0x000, CTRL_START_LAYER)

# 持续 peek SEQ_DBG 每 100us 采样一次, 看 state 转换
print(f'{"t (us)":>6}  STATUS    SEQ_DBG   seq      idma_sg     odma_sg')
print('-' * 70)
last_sd = -1
t0 = time.perf_counter()
samples = []
for i in range(10000):  # 1s 总 monitor
    st = rpc.peek_csr(0, 0x004); sd = rpc.peek_csr(0, 0x008)
    if sd != last_sd:
        t_us = (time.perf_counter() - t0) * 1e6
        seq_s = sd & 0xF
        idma_s = (sd >> 12) & 0xF
        odma_s = (sd >> 8) & 0xF
        samples.append((t_us, st, sd, seq_s, idma_s, odma_s))
        last_sd = sd
    if (st >> 11) & 1:
        break

print(f'Total {len(samples)} unique state samples in {(time.perf_counter()-t0)*1000:.0f}ms')
print()
for t_us, st, sd, seq, idma, odma in samples:
    print(f'{t_us:6.0f}  0x{st:08x}  0x{sd:08x}   {SEQ_STATES.get(seq, seq):8s} idma={IDMA_STATES.get(idma, idma):10s} odma={ODMA_STATES.get(odma, odma)}')

rpc.close()
