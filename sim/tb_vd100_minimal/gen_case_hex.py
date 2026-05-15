#!/usr/bin/env python3
"""Generate BRAM initialization hex for tb_vd100_minimal — 严格复刻 board byte stream.

跑跟 board test_stage4_bitexact 完全一致的 setup_case_for_board (host 端 build cfg
+ desc list + SG cmd list bytes), 然后 dump 整个 BRAM (256KB) 为 $readmemh 128-bit
hex file (16384 word).

用法:
  python gen_case_hex.py --k 3 --h 200 --w 16 --cin 16 --cout 16 --stride 1 --pad 1
  # 输出 sim/tb_vd100_minimal/{bram_init.hex, expected_ofm.hex, params.f}
"""
import sys, os, argparse, struct
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board
from test_stage4_bitexact import (DESC_OFF, ISG_OFF, OSG_OFF, IFM_OFF, WB_OFF, RDMA_OFF, OFM_OFF, BRAM_BASE,
                                    BRAM_ISG, BRAM_OSG, BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM)

ap = argparse.ArgumentParser()
ap.add_argument('--k', type=int, required=True)
ap.add_argument('--h', type=int, required=True)
ap.add_argument('--w', type=int, required=True)
ap.add_argument('--cin', type=int, default=16)
ap.add_argument('--cout', type=int, default=16)
ap.add_argument('--stride', type=int, default=1)
ap.add_argument('--pad', type=int, default=1)
ap.add_argument('--out', default=r'C:/_Project/FLUX_CNN/sim/tb_vd100_minimal')
args = ap.parse_args()

case_dir = os.path.join(args.out, f'case_h{args.h}_w{args.w}_k{args.k}')
os.makedirs(case_dir, exist_ok=True)

# 1. 跑 toolchain 生成 ifb/wb/rdma/expected
gen_isa_test.generate_random(out_dir=case_dir, seed=42, shift_amt=0, streaming=False,
    H_IN=args.h, W_IN=args.w, K=args.k, NUM_CIN=args.cin, NUM_COUT=args.cout,
    TILE_W=32, stride=args.stride, pad_top=args.pad, pad_left=args.pad)

# 2. 跑 host setup_case_for_board 生成 desc + SG cmd 跟 board 完全一致
desc, n_desc, cfg, ifm, wb, rdma, expected, isg, osg = setup_case_for_board(
    case_dir, K=args.k, H_IN=args.h, W_IN=args.w, NUM_CIN=args.cin, NUM_COUT=args.cout,
    stride=args.stride, pad=args.pad)

# 3. 拼 256KB BRAM byte stream (跟 board BRAM layout 一致)
BRAM_SIZE = 256 * 1024
mem = bytearray([0] * BRAM_SIZE)
mem[DESC_OFF: DESC_OFF + len(desc)]  = desc
mem[ISG_OFF : ISG_OFF + len(isg)]    = isg
mem[OSG_OFF : OSG_OFF + len(osg)]    = osg
mem[IFM_OFF : IFM_OFF + len(ifm)]    = ifm
mem[WB_OFF  : WB_OFF + len(wb)]      = wb
mem[RDMA_OFF: RDMA_OFF + len(rdma)]  = rdma
# OFM region 留 0 (board pre-clear)

# 4. dump 128-bit/line hex (16384 word)
hex_path = os.path.join(args.out, 'bram_init.hex')
with open(hex_path, 'w') as f:
    for i in range(BRAM_SIZE // 16):
        word_bytes = mem[i*16:(i+1)*16]
        # little-endian byte → big-endian 128-bit hex (高 byte 在左)
        val = int.from_bytes(word_bytes, 'little')
        f.write(f'{val:032x}\n')

# 5. dump expected OFM hex (用于 sim 比对)
exp_path = os.path.join(args.out, 'expected_ofm.hex')
with open(exp_path, 'w') as f:
    pad_len = ((len(expected) + 15) // 16) * 16
    exp_padded = expected + b'\x00' * (pad_len - len(expected))
    for i in range(pad_len // 16):
        val = int.from_bytes(exp_padded[i*16:(i+1)*16], 'little')
        f.write(f'{val:032x}\n')

# 6. dump params (sim 用)
params_path = os.path.join(args.out, 'params.f')
n_ofm_words = (len(expected) + 15) // 16
n_desc_per_layer = n_desc
desc_list_base = BRAM_BASE + DESC_OFF
ofm_base = BRAM_BASE + OFM_OFF
with open(params_path, 'w') as f:
    # SV +define+ value 必须是 SV literal — 用 decimal 避免 0x 前缀
    f.write(f'+define+DESC_LIST_BASE={desc_list_base}\n')
    f.write(f'+define+DESC_COUNT={n_desc_per_layer}\n')
    f.write(f'+define+OFM_BASE={ofm_base}\n')
    f.write(f'+define+OFM_BYTES={len(expected)}\n')
    f.write(f'+define+OFM_WORDS={n_ofm_words}\n')

print(f'[GEN] BRAM init: {hex_path} (16384 word)')
print(f'[GEN] Expected OFM: {exp_path} ({n_ofm_words} word, {len(expected)} byte)')
print(f'[GEN] params: {params_path}')
print(f'      DESC_LIST_BASE=0x{desc_list_base:x} COUNT={n_desc_per_layer}')
print(f'      OFM_BASE=0x{ofm_base:x} BYTES={len(expected)}')
