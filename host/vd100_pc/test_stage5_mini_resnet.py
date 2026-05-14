#!/usr/bin/env python3
"""Stage 5+ Mini ResNet — mixed-shape 多层 chain, 真 ResNet 风格.

各层 K/H/W/Cin/Cout/stride 都不同, 模拟真 ResNet 各层 shape 变化:
  L0: Conv K=3  H=W=16  Cin=8  Cout=16          (input expansion)
  L1: Conv K=1  H=W=16  Cin=16 Cout=16          (pointwise)
  L2: Conv K=3  H=W=16  Cin=16 Cout=16          (residual block 1.1)
  L3: Conv K=3  H=W=16  Cin=16 Cout=32          (channel doubling)
  L4: Conv K=1  H=W=16  Cin=32 Cout=32          (pointwise)
  L5: Conv K=3  H=W=16  Cin=32 Cout=32          (residual block 2.1)
  L6: Conv K=3  H=W=8   Cin=32 Cout=32 stride=2 (downsample)
  L7: Conv K=1  H=W=8   Cin=32 Cout=64          (channel up)
  L8: Conv K=3  H=W=8   Cin=64 Cout=64          (residual block 3.1)

L6 stride=2: H_in=16 → H_out=8, 真 ResNet downsample 风格.

验证目标:
  - 各层不同 cfg 完整 chain bit-exact (软 reset 隔离正常)
  - 跨 channel slice (cin/cout cs=2/4) 正常
  - stride=2 downsample 正常
  - 9 层 mixed-shape 整网 bit-exact
"""
import sys, os, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board, run_board_layer


# Mini ResNet layer 定义
LAYERS = [
    # name,         K, H_in, W_in, Cin, Cout, stride, pad
    ('L0_expand',   3, 16,   16,   8,   16,   1,      1),
    ('L1_point1',   1, 16,   16,   16,  16,   1,      0),
    ('L2_resid1.1', 3, 16,   16,   16,  16,   1,      1),
    ('L3_chan_x2',  3, 16,   16,   16,  32,   1,      1),
    ('L4_point2',   1, 16,   16,   32,  32,   1,      0),
    ('L5_resid2.1', 3, 16,   16,   32,  32,   1,      1),
    ('L6_ds_str2',  3, 16,   16,   32,  32,   2,      1),
    ('L7_chan_x2',  1, 8,    8,    32,  64,   1,      0),
    ('L8_resid3.1', 3, 8,    8,    64,  64,   1,      1),
]


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== Stage 5+ Mini ResNet (mixed-shape 9-layer chain) ===')

    prev_ofm = None
    pass_count = 0
    for idx, (name, K, Hin, Win, Cin, Cout, stride, pad) in enumerate(LAYERS):
        print()
        print(f'[L{idx} {name}] K={K} H={Hin}x{Win} Cin={Cin}→Cout={Cout} stride={stride} pad={pad}')

        # 检查 prev_ofm dim 跟当前 layer Cin / Hin/Win 一致
        if prev_ofm is not None:
            # prev_ofm 是 [H_OUT_prev][W_OUT_prev][NUM_COUT_prev] int 列表
            h_prev = len(prev_ofm); w_prev = len(prev_ofm[0]); c_prev = len(prev_ofm[0][0])
            if h_prev != Hin or w_prev != Win or c_prev != Cin:
                print(f'  [WARN] prev_ofm shape {h_prev}x{w_prev}x{c_prev} != expected {Hin}x{Win}x{Cin}')
                print(f'  [WARN] using random IFM (chain breaks)')
                prev_ofm = None

        case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_mini_resnet/{name}'
        ret = gen_isa_test.generate_random(
            out_dir=case_dir, ifm_arr_in=prev_ofm,
            seed=42 + idx, shift_amt=0, streaming=False,
            H_IN=Hin, W_IN=Win, K=K, NUM_CIN=Cin, NUM_COUT=Cout,
            TILE_W=32, stride=stride, pad_top=pad, pad_left=pad)

        desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
            case_dir, K=K, H_IN=Hin, W_IN=Win, NUM_CIN=Cin, NUM_COUT=Cout,
            stride=stride, pad=pad)

        ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
        if not ok:
            if board_ofm:
                diff = sum(1 for a, b in zip(board_ofm, exp) if a != b)
                print(f'  [{name}] FAIL: mismatch {diff}/{len(exp)} byte')
            else:
                print(f'  [{name}] FAIL: {err}')
            break
        print(f'  [{name}] PASS  OFM {len(exp)} byte bit-exact')
        pass_count += 1
        prev_ofm = ret['ofm_arr']

    print()
    print(f'=== Summary: {pass_count}/{len(LAYERS)} layers PASS ===')
    rpc.close()
    return 0 if pass_count == len(LAYERS) else 1


if __name__ == '__main__':
    sys.exit(main())
