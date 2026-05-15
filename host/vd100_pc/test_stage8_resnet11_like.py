#!/usr/bin/env python3
"""Stage 8: 真 ResNet11 风格 11-layer chain on board.

模拟 ResNet11 完整 11 层架构 (各层不同 shape, 含 residual fusion),
全 H≤32 (用现有 256KB BRAM + H>32 fix 后跑得了).

architecture:
  L0  Conv K=3 32×32 Cin=3   Cout=16 s=1 pad=1   (input expansion)
  L1  Conv K=3 32×32 Cin=16  Cout=16 s=2 pad=1   (ds1, H=32→16)
  L2  Conv K=3 16×16 Cin=16  Cout=16 s=1 pad=1   (resid block 1.1)
  L3  Conv K=3 16×16 Cin=16  Cout=16 s=1 pad=1 + shortcut(L1) (resid 1.2)
  L4  Conv K=3 16×16 Cin=16  Cout=32 s=2 pad=1   (ds2, H=16→8)
  L5  Conv K=3 8×8   Cin=32  Cout=32 s=1 pad=1   (resid 2.1)
  L6  Conv K=3 8×8   Cin=32  Cout=32 s=1 pad=1 + shortcut(L4) (resid 2.2)
  L7  Conv K=3 8×8   Cin=32  Cout=64 s=2 pad=1   (ds3, H=8→4)
  L8  Conv K=3 4×4   Cin=64  Cout=64 s=1 pad=1   (resid 3.1)
  L9  Conv K=3 4×4   Cin=64  Cout=64 s=1 pad=1 + shortcut(L7) (resid 3.2)
  L10 Conv K=4 4×4   Cin=64  Cout=10 s=1 pad=0   (FC, H=4→1)

residual layers (L3/L6/L9) shortcut = 上 2 层 OFM (跟当前层 OFM 同 shape).
"""
import sys, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board, run_board_layer


# (name, K, H, W, Cin, Cout, stride, pad, shortcut_from)
LAYERS = [
    ('L0_input',  3, 32, 32, 3,  16, 1, 1, None),
    ('L1_ds1',    3, 32, 32, 16, 16, 2, 1, None),
    ('L2_r1.1',   3, 16, 16, 16, 16, 1, 1, None),
    ('L3_r1.2',   3, 16, 16, 16, 16, 1, 1, 'L1'),  # shortcut = L1 OFM
    ('L4_ds2',    3, 16, 16, 16, 32, 2, 1, None),
    ('L5_r2.1',   3, 8,  8,  32, 32, 1, 1, None),
    ('L6_r2.2',   3, 8,  8,  32, 32, 1, 1, 'L4'),
    ('L7_ds3',    3, 8,  8,  32, 64, 2, 1, None),
    ('L8_r3.1',   3, 4,  4,  64, 64, 1, 1, None),
    ('L9_r3.2',   3, 4,  4,  64, 64, 1, 1, 'L7'),
    ('L10_fc',    4, 4,  4,  64, 10, 1, 0, None),
]


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== Stage 8: ResNet11-like 11-layer chain (mixed shape + residual) ===')

    ofm_cache = {}  # name → ofm_arr, 用于 shortcut
    prev_ofm = None
    pass_cnt = 0

    for idx, (name, K, H, W, Cin, Cout, stride, pad, sc_from) in enumerate(LAYERS):
        print()
        print(f'[L{idx} {name}] K={K} H={H}x{W} Cin={Cin}→Cout={Cout} stride={stride} pad={pad}'
              f' shortcut={sc_from or "—"}')

        case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_resnet11/{name}'
        kwargs = dict(
            out_dir=case_dir, ifm_arr_in=prev_ofm,
            seed=42 + idx, shift_amt=0, streaming=False,
            H_IN=H, W_IN=W, K=K, NUM_CIN=Cin, NUM_COUT=Cout,
            TILE_W=32, stride=stride, pad_top=pad, pad_left=pad)
        residual_en = 0
        sc_mult = 0; sc_shift = 0
        if sc_from is not None:
            kwargs['shortcut_arr_in'] = ofm_cache[sc_from]
            residual_en = 1
            sc_mult = 1
        ret = gen_isa_test.generate_random(**kwargs)

        desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
            case_dir, K=K, H_IN=H, W_IN=W, NUM_CIN=Cin, NUM_COUT=Cout,
            stride=stride, pad=pad,
            residual_en=residual_en, shortcut_mult=sc_mult, shortcut_shift=sc_shift)

        ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
        if not ok:
            if board_ofm:
                diff = sum(1 for a, b in zip(board_ofm, exp) if a != b)
                print(f'  [{name}] FAIL mismatch {diff}/{len(exp)}')
            else:
                print(f'  [{name}] FAIL {err}')
            break
        print(f'  [{name}] PASS  OFM {len(exp)} byte bit-exact')
        pass_cnt += 1
        ofm_cache[name.split('_')[0]] = ret['ofm_arr']
        prev_ofm = ret['ofm_arr']

    print()
    print(f'=== Summary: {pass_cnt}/{len(LAYERS)} layer PASS ===')
    rpc.close()
    return 0 if pass_cnt == len(LAYERS) else 1


if __name__ == '__main__':
    sys.exit(main())
