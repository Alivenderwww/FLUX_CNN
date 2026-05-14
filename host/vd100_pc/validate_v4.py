#!/usr/bin/env python3
"""v4 PDI 验证: axi_dm 在软 reset 范围 + 64 拍 stretched.

测试矩阵:
  1. v3 baseline 重测 (Stage 4 corner case + 11-layer chain) 确认 fix 没 break
  2. 多次跑 + 软 reset 之间, 验证 axi_dm 状态干净不累积
  3. H=33/H=40/H=64 重测 看 fix 是否解决 H>32 mismatch
"""
import sys, time, subprocess, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
from test_stage5_chain_bitexact import setup_case_for_board, run_board_layer
import gen_isa_test

CTRL_SOFT_RESET = 0x80

def run_case(rpc, label, K, H, W, Cin, Cout, stride, pad, seed=42):
    case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_v4_{label}'
    ret = gen_isa_test.generate_random(out_dir=case_dir, ifm_arr_in=None, seed=seed,
        shift_amt=0, streaming=False,
        H_IN=H, W_IN=W, K=K, NUM_CIN=Cin, NUM_COUT=Cout, TILE_W=32,
        stride=stride, pad_top=pad, pad_left=pad)
    desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
        case_dir, K=K, H_IN=H, W_IN=W, NUM_CIN=Cin, NUM_COUT=Cout, stride=stride, pad=pad)
    ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
    if ok:
        print(f'  [{label}] PASS')
        return True
    if board_ofm:
        diff = sum(1 for a,b in zip(board_ofm, exp) if a!=b)
        print(f'  [{label}] FAIL: mismatch {diff}/{len(exp)} byte')
    else:
        print(f'  [{label}] FAIL: {err}')
    return False


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== v4 PDI 验证 ===')

    # Test 1: 重测 v3 corner case 全 PASS (确认 axi_dm reset 改动没 break)
    print()
    print('[Test 1] v3 baseline retest (整图 fit H<=32)')
    cases = [
        ('K=1_H1', 1, 1, 1, 16, 16, 1, 0),
        ('K=3_H8', 3, 8, 8, 16, 16, 1, 1),
        ('K=3_H16', 3, 16, 16, 16, 16, 1, 1),
        ('K=3_H32', 3, 32, 32, 16, 16, 1, 1),
        ('K=5_H8', 5, 8, 8, 16, 16, 1, 2),
        ('K=3_H8_stride2', 3, 8, 8, 16, 16, 2, 1),  # 注意 H_OUT=4
    ]
    n_pass1 = 0
    for cinfo in cases:
        ok = run_case(rpc, *cinfo)
        if ok: n_pass1 += 1
    print(f'  Test 1: {n_pass1}/{len(cases)} PASS')

    # Test 2: 多次跑 + 软 reset 之间 (验证 axi_dm 不累积 state)
    print()
    print('[Test 2] 多 run + 软 reset 之间, axi_dm 状态干净')
    n_pass2 = 0
    for i in range(10):
        ok = run_case(rpc, f'run{i}', 3, 8, 8, 16, 16, 1, 1, seed=42+i)
        if ok: n_pass2 += 1
    print(f'  Test 2: {n_pass2}/10 PASS')

    # Test 3: H>32 重测 (axi_dm reset fix 之后)
    print()
    print('[Test 3] H>32 case (v4 fix 后)')
    h_cases = [
        ('H=33_W25', 3, 33, 25, 16, 16, 1, 1),
        ('H=40_W25', 3, 40, 25, 16, 16, 1, 1),
        ('H=64_W8', 3, 64, 8, 16, 16, 1, 1),
    ]
    n_pass3 = 0
    for cinfo in h_cases:
        ok = run_case(rpc, *cinfo)
        if ok: n_pass3 += 1
    print(f'  Test 3: {n_pass3}/{len(h_cases)} PASS')

    print()
    print(f'=== Summary: T1 {n_pass1}/{len(cases)} | T2 {n_pass2}/10 | T3 {n_pass3}/{len(h_cases)} ===')
    rpc.close()
    return 0 if (n_pass1 == len(cases) and n_pass2 == 10) else 1


if __name__ == '__main__':
    sys.exit(main())
