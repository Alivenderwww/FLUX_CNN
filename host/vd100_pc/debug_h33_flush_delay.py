#!/usr/bin/env python3
"""H=33 hypothesis: axi_dm s2mm flush 延迟. layer_done 后等几 ms 再读 OFM."""
import sys, os, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import BRAM_BASE, DESC_OFF, BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM, CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET, chunked_load, chunked_read

case_dir = r'C:/_Project/FLUX_CNN/sim/tb_h33_flush'
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

# wait layer_done
for _ in range(500):
    st = rpc.peek_csr(0, 0x004)
    if (st >> 11) & 1: break
    time.sleep(0.01)
print(f'layer_done after {(_*10)} ms, STATUS=0x{st:08x}')

# multiple delays
for delay_ms in [0, 5, 20, 100, 500, 1000]:
    time.sleep(delay_ms / 1000)
    ofm = chunked_read(rpc, BRAM_OFM, len(exp))
    diff = sum(1 for a, b in zip(ofm, exp) if a != b)
    # 看 last row 是不是 0xAA pattern
    last_row = ofm[(H-1)*W*16:(H)*W*16]
    aa_in_last = sum(1 for b in last_row if b == 0xAA)
    print(f'  delay {delay_ms:>4}ms: diff={diff}/{len(exp)}, last row 0xAA={aa_in_last}/{W*16}')

rpc.close()
