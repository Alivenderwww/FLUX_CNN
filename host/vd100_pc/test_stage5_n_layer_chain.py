#!/usr/bin/env python3
"""Stage 5 N-layer chain bit-exact: 跑 N 层 random conv, 每层用上层 OFM 作 IFM.

每层独立 board run, layer 之间 host 端把 Python ofm_arr 作为下层 ifm_arr_in
(等价于 board L_n OFM bit-exact = Python L_n OFM, 用 Python 端继续算 L_{n+1}).

目标: 验证 board 多层连续启停 + 软 reset 隔离 + cfg 切换在 N≥5 层下都稳定.

case: K=3 H=W=8 Cin=Cout=16 pad=1 (整图 fit 范围内)
"""
import sys, os, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import hw_files
import gen_isa_test
from test_stage5_chain_bitexact import (
    gen_case_in_dir, setup_case_for_board, run_board_layer
)


def main():
    N_LAYERS = 11
    K, H, W, CIN, COUT, STRIDE, PAD = 3, 8, 8, 16, 16, 1, 1

    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print(f'=== Stage 5 N-layer chain: N={N_LAYERS} layers ===')

    prev_ofm_arr = None  # L_0 用 random IFM, L_n>0 用 L_{n-1} OFM
    for layer_idx in range(N_LAYERS):
        print()
        print(f'[L{layer_idx}] generate + board')
        case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_stage5_n_l{layer_idx}'
        # 不同 layer 用不同 seed (避免完全一样的 weight)
        ret = gen_isa_test.generate_random(
            out_dir=case_dir,
            ifm_arr_in=prev_ofm_arr,
            seed=42 + layer_idx,
            shift_amt=0, streaming=False,
            H_IN=H, W_IN=W, K=K, NUM_CIN=CIN, NUM_COUT=COUT,
            TILE_W=32, stride=STRIDE, pad_top=PAD, pad_left=PAD)

        desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
            case_dir, K=K, H_IN=H, W_IN=W, NUM_CIN=CIN, NUM_COUT=COUT,
            stride=STRIDE, pad=PAD)

        ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
        if not ok:
            print(f'  [L{layer_idx}] FAIL: {err}')
            return 1
        print(f'  [L{layer_idx}] PASS  OFM {len(exp)} byte bit-exact')

        prev_ofm_arr = ret['ofm_arr']

    print()
    print(f'[OK] Stage 5 N-layer chain: {N_LAYERS}/{N_LAYERS} layer 全 bit-exact PASS')
    rpc.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
