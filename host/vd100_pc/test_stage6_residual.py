#!/usr/bin/env python3
"""Stage 6 Residual fusion — SDP shortcut add 路径 board bit-exact 验证.

真 ResNet 残差融合: y = ReLU(quantize(conv(x) + shortcut * scale))
- conv 结果经 SDP (shift / clip / ReLU) 量化
- shortcut tensor 加到 SDP 路径上 (bias_rf + Shortcut Bank)
- 整段在 ofb_writer SDP 链内完成

测试:
  case 1: K=3 H=W=8 Cin=Cout=16, residual=True, random shortcut
  case 2: K=3 H=W=16 Cin=Cout=32 (cs=2), residual=True
  case 3: K=1 H=W=8 Cin=Cout=32, residual=True
"""
import sys, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board, run_board_layer


CASES = [
    # name, K, H, W, Cin, Cout, stride, pad
    ('R1_K3_H8',    3, 8,  8,  16, 16, 1, 1),
    ('R2_K3_H16_C32', 3, 16, 16, 32, 32, 1, 1),
    ('R3_K1_H8_C32',  1, 8,  8,  32, 32, 1, 0),
    ('R4_K3_H16_C16', 3, 16, 16, 16, 16, 1, 1),
]


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== Stage 6 Residual Fusion (SDP shortcut path) ===')

    pass_cnt = 0
    for idx, (name, K, H, W, Cin, Cout, stride, pad) in enumerate(CASES):
        print()
        print(f'[{name}] K={K} H={H}x{W} Cin={Cin} Cout={Cout} stride={stride} pad={pad} residual=ON')

        case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_residual/{name}'
        ret = gen_isa_test.generate_random(
            out_dir=case_dir, ifm_arr_in=None,
            seed=42 + idx, shift_amt=0, streaming=False, residual=True,
            H_IN=H, W_IN=W, K=K, NUM_CIN=Cin, NUM_COUT=Cout,
            TILE_W=32, stride=stride, pad_top=pad, pad_left=pad)

        desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
            case_dir, K=K, H_IN=H, W_IN=W, NUM_CIN=Cin, NUM_COUT=Cout,
            stride=stride, pad=pad,
            residual_en=1, shortcut_mult=1, shortcut_shift=0)

        ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
        if ok:
            print(f'  [{name}] PASS bit-exact')
            pass_cnt += 1
        else:
            if board_ofm:
                diff = sum(1 for a, b in zip(board_ofm, exp) if a != b)
                print(f'  [{name}] FAIL: mismatch {diff}/{len(exp)} byte')
            else:
                print(f'  [{name}] FAIL: {err}')

    print()
    print(f'=== Summary: {pass_cnt}/{len(CASES)} residual case PASS ===')
    rpc.close()
    return 0 if pass_cnt == len(CASES) else 1


if __name__ == '__main__':
    sys.exit(main())
