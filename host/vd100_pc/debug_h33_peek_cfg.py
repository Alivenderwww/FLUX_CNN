#!/usr/bin/env python3
"""H=33 board peek cfg_regs 实际写入的 ODMA_CMD_COUNT, 验证不是 cfg 位宽截断."""
import sys, time, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import (BRAM_BASE, DESC_OFF, BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM,
                                    CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET, chunked_load, chunked_read)

case_dir = r'C:/_Project/FLUX_CNN/sim/tb_h33_peek'
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

# sequencer 在 start_layer 时才消费 CFG_WRITE desc 写 cfg_regs.
# 所以要 start_layer 后 peek (layer run 跑完, cfg_regs 已被 CFG_WRITE 写满).
rpc.poke_csr(0, 0x000, CTRL_START_DFE); time.sleep(0.05)
rpc.poke_csr(0, 0x000, CTRL_START_LAYER)
for _ in range(500):
    st = rpc.peek_csr(0, 0x004)
    if (st >> 11) & 1: break
    time.sleep(0.01)
print(f'layer_done STATUS=0x{st:08x}')

# Peek 关键 cfg_regs
print('=== board cfg_regs peek after layer ===')
print(f'  H_OUT (0x100)            = {rpc.peek_csr(0, 0x100)} (期望 33)')
print(f'  W_OUT (0x104)            = {rpc.peek_csr(0, 0x104)} (期望 25)')
print(f'  H_IN_TOTAL (0x168)       = {rpc.peek_csr(0, 0x168)} (期望 33)')
print(f'  IFB_STRIP_ROWS (0x16C)   = {rpc.peek_csr(0, 0x16C)} (期望 33)')
print(f'  OFB_STRIP_ROWS (0x170)   = {rpc.peek_csr(0, 0x170)} (期望 33)')
print(f'  IFB_RING_WORDS (0x1A0)   = {rpc.peek_csr(0, 0x1A0)} (期望 825)')
print(f'  OFB_RING_WORDS (0x1A8)   = {rpc.peek_csr(0, 0x1A8)} (期望 825)')
print(f'  OFB_ROW_WORDS (0x1A4)    = {rpc.peek_csr(0, 0x1A4)} (期望 25)')
print(f'  IDMA_CMD_COUNT (0x1DC)   = {rpc.peek_csr(0, 0x1DC)} (期望 33)')
print(f'  IDMA_CMDS_PER_ROW (0x1E0)= {rpc.peek_csr(0, 0x1E0)} (期望 1)')
print(f'  ODMA_CMD_COUNT (0x1E8)   = {rpc.peek_csr(0, 0x1E8)} (期望 33)')
print(f'  ODMA_CMDS_PER_ROW (0x1EC)= {rpc.peek_csr(0, 0x1EC)} (期望 1)')
print(f'  ODMA_CMD_LIST_PTR (0x1E4)= 0x{rpc.peek_csr(0, 0x1E4):08X}')

rpc.close()
