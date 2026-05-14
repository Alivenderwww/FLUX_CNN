#!/usr/bin/env python3
"""Debug streaming mode stuck — H=40 case 跑时持续 monitor STATUS/SEQ_DBG 看 progress.

策略: 启动 layer 后每 5ms peek 一次 STATUS + SEQ_DBG, 记录 state 变化轨迹,
看 stuck 是在哪一步停下来.
"""
import sys, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import (BRAM_BASE, DESC_OFF, BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA,
                                    BRAM_OFM, CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET,
                                    chunked_load, chunked_read)

def decode_status(st):
    return f'core_b={(st>>1)&1} idma_b={(st>>2)&1} odma_b={(st>>4)&1} ' \
           f'idma_d={(st>>5)&1} wdma_d={(st>>6)&1} odma_d={(st>>7)&1} ' \
           f'dfe_d={(st>>9)&1} layer_b={(st>>10)&1} layer_d={(st>>11)&1}'

def decode_seq(sd):
    seq = sd & 0xF; fifo = (sd>>4) & 0xF
    odma_sg = (sd>>8) & 0xF; idma_sg = (sd>>12) & 0xF
    seq_names = ['IDLE','FETCH','PRELOAD','DISPATCH','WAIT','BARRIER','END']
    idma_names = ['IDLE','FETCH_ISS','FETCH_DAT','?','RING_WAIT','ISSUE','DATA','?','?','DONE']
    odma_names = ['IDLE','WAIT','FETCH_ISS','FETCH_DAT','?','CMD','PREFETCH','TX','STS','DONE']
    return f'seq={seq_names[seq] if seq<len(seq_names) else seq} fifo={fifo} ' \
           f'idma_sg={idma_names[idma_sg] if idma_sg<len(idma_names) else idma_sg} ' \
           f'odma_sg={odma_names[odma_sg] if odma_sg<len(odma_names) else odma_sg}'


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== streaming debug: H=40 case monitor ===')

    case_dir = r'C:/_Project/FLUX_CNN/sim/tb_streaming_dbg'
    ret = gen_isa_test.generate_random(
        out_dir=case_dir, ifm_arr_in=None, seed=42,
        shift_amt=0, streaming=False,
        H_IN=40, W_IN=40, K=3, NUM_CIN=16, NUM_COUT=16,
        TILE_W=32, stride=1, pad_top=1, pad_left=1)

    desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
        case_dir, K=3, H_IN=40, W_IN=40, NUM_CIN=16, NUM_COUT=16, stride=1, pad=1)

    print(f'cfg: ifb_strip={cfg["ifb_strip"]} (H_IN=40) ifb_ring={cfg["ifb_ring_words"]} '
          f'ofb_strip={cfg["ofb_strip"]} ofb_ring={cfg["ofb_ring_words"]}')
    print(f'desc count={n_desc} OFM size={len(exp)} byte')

    rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET); time.sleep(0.01)
    chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
    chunked_load(rpc, BRAM_ISG, isg)
    chunked_load(rpc, BRAM_OSG, osg)
    chunked_load(rpc, BRAM_IFM, ifm)
    chunked_load(rpc, BRAM_WB, wb)
    chunked_load(rpc, BRAM_RDMA, rdma)
    chunked_load(rpc, BRAM_OFM, b'\xAA' * len(exp))
    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)

    rpc.poke_csr(0, 0x000, CTRL_START_DFE)
    time.sleep(0.05)
    rpc.poke_csr(0, 0x000, CTRL_START_LAYER)

    print()
    print(f'{"t (ms)":>8}  STATUS    SEQ_DBG    state')
    print('-' * 90)
    last_sd = None
    t0 = time.perf_counter()
    for i in range(100):  # 1s 总 monitor
        time.sleep(0.005)
        st = rpc.peek_csr(0, 0x004); sd = rpc.peek_csr(0, 0x008)
        if sd != last_sd:
            t_ms = (time.perf_counter() - t0) * 1000
            print(f'{t_ms:8.1f}  0x{st:08x}  0x{sd:08x}  {decode_seq(sd)}')
            last_sd = sd
        if (st >> 11) & 1:
            print(f'   layer_done! @{(time.perf_counter()-t0)*1000:.1f}ms')
            break

    # final
    st = rpc.peek_csr(0, 0x004); sd = rpc.peek_csr(0, 0x008)
    print()
    print(f'final STATUS=0x{st:08x}  {decode_status(st)}')
    print(f'final SEQ_DBG=0x{sd:08x}  {decode_seq(sd)}')

    rpc.close()

if __name__ == '__main__':
    main()
