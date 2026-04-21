"""
gen_isa_test.py — 随机数据生成器（给 RTL 回归用）

职责：生成随机 ifm + weight + 软件模拟出 expected_ofm，再调 hw_files 写文件。
所有文件 I/O 与 cfg 派生逻辑共用 hw_files 模块（和 compile_layer.py 共享）。

用法：
  python gen_isa_test.py --k 3 --h_in 48 --w_in 48 --num_cin 8 --num_cout 8 --pad 1
  # 输出默认 ../sim/tb_core_dma/
  python gen_isa_test.py ... --out-dir /tmp/foo
"""
import argparse
import os
import random
import sys

import hw_files


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))


def parse_args():
    p = argparse.ArgumentParser(description='Generate random test data for tb_core_dma.')
    p.add_argument('--h_in',     type=int, default=48)
    p.add_argument('--w_in',     type=int, default=48)
    p.add_argument('--k',        type=int, default=3)
    p.add_argument('--stride',   type=int, default=1)
    p.add_argument('--num_cin',  type=int, default=8)
    p.add_argument('--num_cout', type=int, default=8)
    p.add_argument('--hw_pe',    type=int, default=16)
    p.add_argument('--hw_col',   type=int, default=16)
    p.add_argument('--tile_w',   type=int, default=32)
    p.add_argument('--seed',     type=int, default=42)
    p.add_argument('--shift',    type=int, default=0)
    # J-1: 硬件统一为 streaming 数据路径；flag 保留仅为兼容，默认 True
    p.add_argument('--streaming', action='store_true', default=True,
                   help='(deprecated, always streaming) emit streaming-ring cfg')
    p.add_argument('--pad',      type=int, default=0, help='symmetric pad')
    p.add_argument('--pad_top',  type=int, default=None)
    p.add_argument('--pad_left', type=int, default=None)
    p.add_argument('--strip_rows', type=int, default=0,
                   help='0 = single strip (default); >0 = multi-strip')
    p.add_argument('--out-dir',  default=DEFAULT_OUT_DIR,
                   help='output directory (default: ../sim/tb_core_dma/)')
    p.add_argument('--case-name', default="",
                   help='case name to embed in config.txt (F-2 multi-case)')
    return p.parse_args()


def generate_random(
    H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
    shift_amt=0, stride=1, HW_PE=16, HW_COL=16,
    streaming=False, pad_top=0, pad_left=0, strip_rows=0,
    out_dir=DEFAULT_OUT_DIR, case_name="",
):
    """随机 ifm + weight + 兼容旧 SDP (mult=1, shift=N, clip[0,255], ReLU) 的测试生成。"""
    os.makedirs(out_dir, exist_ok=True)
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")

    # ---- 派生 cfg ----
    try:
        cfg = hw_files.derive_layer_cfg(
            H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
            stride=stride, pad_top=pad_top, pad_left=pad_left,
            TILE_W=TILE_W, HW_PE=HW_PE, HW_COL=HW_COL,
            streaming=streaming)
    except ValueError as e:
        sys.exit(f"ERROR: {e}")

    H_OUT, W_OUT = cfg['H_OUT'], cfg['W_OUT']
    cin_slices, cout_slices = cfg['cin_slices'], cfg['cout_slices']

    PYTHON_SRAM_LIMIT = 524288
    for name, sz in [('IFB', cfg['ifb_words']), ('WB', cfg['wb_words']), ('OFB', cfg['ofb_words'])]:
        if sz > PYTHON_SRAM_LIMIT:
            sys.exit(f"ERROR: {name} overflow! size={sz} > {PYTHON_SRAM_LIMIT}")

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, stride={stride}, "
          f"Cin={NUM_CIN} ({cin_slices} slice(s)), Cout={NUM_COUT} ({cout_slices} slice(s)), "
          f"TILE_W={cfg['TILE_W']}")
    print(f"Output : H_OUT={H_OUT}, W_OUT={W_OUT}, num_tiles={cfg['num_tiles']}, "
          f"last_valid_w={cfg['last_valid_w']}")
    mode = 'packed' if cfg['wrf_packed'] else 'chunked'
    print(f"Weight : total_wrf={cfg['total_wrf']}, mode={mode}, "
          f"rounds/cins={cfg['rounds_per_cins']}, round_len_last={cfg['round_len_last']}")
    print(f"SRAM   : IFB={cfg['ifb_words']}, WB={cfg['wb_words']}, OFB={cfg['ofb_words']} -> "
          f"SRAM_DEPTH={cfg['sram_depth']}")

    # ---- 生成 random ifm_arr: [H_IN][W_IN][NUM_CIN] int (0..7, small) ----
    ifm_arr = [[[0] * NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
    for cins in range(cin_slices):
        local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
        for y in range(H_IN):
            for x in range(W_IN):
                for cin_local in range(local_cin):
                    cin = cins * HW_PE + cin_local
                    ifm_arr[y][x][cin] = random.randint(0, 7)

    # ---- 生成 random w_arr: [K][K][NUM_COUT][NUM_CIN] int (-3..3, small) ----
    w_arr = [[[[0] * NUM_CIN for _ in range(NUM_COUT)]
              for _ in range(K)] for _ in range(K)]
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for cins in range(cin_slices):
            local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
            for ky in range(K):
                for kx in range(K):
                    for lc in range(local_cout):
                        cout = cs * HW_COL + lc
                        for cin_local in range(local_cin):
                            cin = cins * HW_PE + cin_local
                            w_arr[ky][kx][cout][cin] = random.randint(-3, 3)

    # ---- 兼容配置 (F-1a): mult=1, zp=0, clip=[0,255], round=0, relu=1 → 等价老 shift+ReLU+clip[0,255]
    sdp_mult, sdp_zp_out, sdp_clip_min, sdp_clip_max = 1, 0, 0, 255
    sdp_round_en, sdp_relu_en = 0, 1

    # ---- 模拟硬件算 expected_ofm（bias=None）----
    ofm_arr, _H, _W = hw_files.compute_expected_ofm(
        H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride, pad_top, pad_left,
        ifm_arr, w_arr, bias_arr=None,
        sdp_mult=sdp_mult, sdp_shift=shift_amt, sdp_zp_out=sdp_zp_out,
        sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
        sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en)
    assert _H == H_OUT and _W == W_OUT

    # ---- 写文件 ----
    hw_files.write_ifb(out_dir, ifm_arr, H_IN, W_IN, NUM_CIN, HW_PE)
    hw_files.write_wb(out_dir, w_arr, bias_arr=None,
                      K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
                      HW_PE=HW_PE, HW_COL=HW_COL)
    hw_files.write_expected_ofm(out_dir, ofm_arr, H_OUT, W_OUT, NUM_COUT, HW_COL)

    cfg_dict = hw_files.cfg_to_dict(cfg, shift_amt=shift_amt,
                                     sdp_mult=sdp_mult, sdp_zp_out=sdp_zp_out,
                                     sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
                                     sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en,
                                     case_name=case_name)
    hw_files.write_config(out_dir, cfg_dict)

    n_desc, n_strips, strip_rows_eff = hw_files.write_descriptors(
        out_dir, H_IN, W_IN, H_OUT, W_OUT, cin_slices, cout_slices,
        pad_top=pad_top, pad_bot=pad_top, pad_left=pad_left, pad_right=pad_left,
        strip_rows=strip_rows, streaming=streaming)
    print(f"Desc   : n_strips={n_strips} (+1 END), total={n_desc} descriptors, "
          f"strip_rows={strip_rows_eff}")

    hw_files.write_sim_params(
        out_dir, H_OUT=H_OUT, W_OUT=W_OUT, cout_slices=cout_slices,
        ifb_words=cfg['ifb_words'], wb_words=cfg['wb_words'],
        desc_count=n_desc, sram_depth=cfg['sram_depth'],
        num_cin=NUM_CIN, num_cout=NUM_COUT)

    print(f"Files  : ifb.txt  wb.txt  expected_ofm.txt  config.txt  sim_params.f  desc_list.hex")


if __name__ == '__main__':
    args = parse_args()
    pad_t = args.pad_top  if args.pad_top  is not None else args.pad
    pad_l = args.pad_left if args.pad_left is not None else args.pad
    generate_random(
        H_IN=args.h_in, W_IN=args.w_in, K=args.k,
        NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
        TILE_W=args.tile_w, seed=args.seed,
        shift_amt=args.shift, stride=args.stride,
        HW_PE=args.hw_pe, HW_COL=args.hw_col,
        streaming=args.streaming,
        pad_top=pad_t, pad_left=pad_l,
        strip_rows=args.strip_rows,
        out_dir=args.out_dir, case_name=args.case_name)
