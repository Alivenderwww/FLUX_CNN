"""
gen_isa_test.py -- CNN Accelerator Configuration Test Generator
(formerly ISA-compiler; rewritten for configuration-register FSM controller)

Usage:
  python gen_isa_test.py [--h_in H] [--w_in W] [--k K] [--stride S]
                         [--num_cin C] [--num_cout N] [--tile_w T]
                         [--shift SH] [--seed SD]

Outputs:
  - ifb.txt           : packed input feature map (same layout as before)
  - wb.txt            : packed weights (same layout as before)
  - expected_ofm.txt  : golden output for TB comparison
  - config.txt        : key=value register map consumed by TB (new)
  - sim_params.f      : plusargs for TB (H_OUT/W_OUT/COUT_SLICES)

Padding == 0 always (phase-1 FSM does not implement padding yet).
"""
import argparse
import math
import sys


WRF_DEPTH = 32  # Hardware: 5-bit wgt_rf_addr


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description='Generate test files for configuration-driven CNN core.')
    p.add_argument('--h_in',     type=int, default=68)
    p.add_argument('--w_in',     type=int, default=120)
    p.add_argument('--k',        type=int, default=3)
    p.add_argument('--stride',   type=int, default=1)
    p.add_argument('--num_cin',  type=int, default=8)
    p.add_argument('--num_cout', type=int, default=8)
    p.add_argument('--hw_pe',    type=int, default=16)
    p.add_argument('--hw_col',   type=int, default=16)
    p.add_argument('--tile_w',   type=int, default=32)
    p.add_argument('--seed',     type=int, default=42)
    p.add_argument('--shift',    type=int, default=0)
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
             shift_amt=0, stride=1, HW_PE=16, HW_COL=16):
    import random
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")

    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

    # No padding in phase 1 FSM
    H_OUT = (H_IN - K) // stride + 1
    W_OUT = (W_IN - K) // stride + 1
    if H_OUT <= 0 or W_OUT <= 0:
        sys.exit(f"ERROR: invalid output {H_OUT}x{W_OUT}")

    kk           = K * K
    total_wrf    = kk * cin_slices
    wrf_packed   = total_wrf <= WRF_DEPTH
    num_tiles    = (W_OUT + TILE_W - 1) // TILE_W
    last_valid_w = W_OUT - (num_tiles - 1) * TILE_W

    # Within one cin_slice, split kk weights into rounds of <= WRF_DEPTH each
    # (packed: always 1 round; chunked kk<=32: 1 round; chunked kk>32: multiple rounds)
    rounds_per_cins = (kk + WRF_DEPTH - 1) // WRF_DEPTH
    round_len_last  = kk - (rounds_per_cins - 1) * WRF_DEPTH

    # IFB slice stride + SRAM sizing
    IFB_CIN_STEP  = H_IN * W_IN
    IFB_ROW_STEP  = stride * W_IN
    WB_CIN_STEP   = kk
    WB_COUT_STEP  = kk * cin_slices
    OFB_COUT_STEP = H_OUT * W_OUT
    TILE_IN_STEP  = TILE_W * stride

    ifb_size = IFB_CIN_STEP * cin_slices
    wb_size  = kk * cout_slices * cin_slices
    ofb_size = OFB_COUT_STEP * cout_slices

    PYTHON_SRAM_LIMIT = 65536   # 64K words; raised to host ImageNet-scale single-slice FM
    for name, sz in [('IFB', ifb_size), ('WB', wb_size), ('OFB', ofb_size)]:
        if sz > PYTHON_SRAM_LIMIT:
            sys.exit(f"ERROR: {name} overflow! size={sz} > {PYTHON_SRAM_LIMIT}")

    hw_sram_depth = max(ifb_size, ofb_size, wb_size, 1024)
    hw_sram_depth = 1 << math.ceil(math.log2(hw_sram_depth))

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, stride={stride}, "
          f"Cin={NUM_CIN} ({cin_slices} slice(s)), Cout={NUM_COUT} ({cout_slices} slice(s)), "
          f"TILE_W={TILE_W}")
    print(f"Output : H_OUT={H_OUT}, W_OUT={W_OUT}, num_tiles={num_tiles}, "
          f"last_valid_w={last_valid_w}")
    mode = 'packed' if wrf_packed else 'chunked'
    print(f"Weight : total_wrf={total_wrf} (K*K*cin_slices), mode={mode}, "
          f"rounds/cins={rounds_per_cins}, round_len_last={round_len_last}")
    print(f"SRAM   : IFB={ifb_size}, WB={wb_size}, OFB={ofb_size} -> SRAM_DEPTH={hw_sram_depth}")

    # ------------------------------------------------------------------
    # 1. IFB data (same layout as ISA version)
    # ------------------------------------------------------------------
    ifb_hex_chars = HW_PE * 2
    ifm_arr = [[[0] * NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
    ifb_data = []
    for cins in range(cin_slices):
        local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
        for y in range(H_IN):
            for x in range(W_IN):
                val = 0
                for cin_local in range(local_cin):
                    cin = cins * HW_PE + cin_local
                    v = random.randint(0, 7)
                    ifm_arr[y][x][cin] = v
                    val |= (v & 0xFF) << (cin_local * 8)
                ifb_data.append(f"{val:0{ifb_hex_chars}X}")
    with open('ifb.txt', 'w') as f:
        f.writelines(d + '\n' for d in ifb_data)

    # ------------------------------------------------------------------
    # 2. WB data (same layout as ISA version)
    #    WB addr = (cs * cin_slices + cins) * K*K + ky*K + kx
    # ------------------------------------------------------------------
    wb_hex_chars = HW_COL * HW_PE * 2
    w_arr = [[[[0] * NUM_CIN for _ in range(NUM_COUT)]
              for _ in range(K)] for _ in range(K)]
    wb_data = []
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for cins in range(cin_slices):
            local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
            for ky in range(K):
                for kx in range(K):
                    val = 0
                    for lc in range(local_cout):
                        cout = cs * HW_COL + lc
                        cv = 0
                        for cin_local in range(local_cin):
                            cin = cins * HW_PE + cin_local
                            v = random.randint(-3, 3)
                            w_arr[ky][kx][cout][cin] = v
                            cv |= (v & 0xFF) << (cin_local * 8)
                        val |= cv << (lc * HW_PE * 8)
                    wb_data.append(f"{val:0{wb_hex_chars}X}")
    with open('wb.txt', 'w') as f:
        f.writelines(d + '\n' for d in wb_data)

    # ------------------------------------------------------------------
    # 3. Expected OFM (golden, same as ISA version)
    # ------------------------------------------------------------------
    expected_ofm = []
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for yout in range(H_OUT):
            for px in range(W_OUT):
                pixel_val = 0
                for lc in range(local_cout):
                    cout = cs * HW_COL + lc
                    psum = 0
                    for ky in range(K):
                        for kx in range(K):
                            for cin in range(NUM_CIN):
                                iy = yout * stride + ky
                                ix = px   * stride + kx
                                psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                    act = max(0, min(255, psum >> shift_amt))
                    pixel_val |= (act & 0xFF) << (lc * 8)
                expected_ofm.append(f"{pixel_val:0{HW_COL * 2}X}")
    with open('expected_ofm.txt', 'w') as f:
        f.writelines(d + '\n' for d in expected_ofm)

    # ------------------------------------------------------------------
    # 4. config.txt -- register map for TB to poke into core_ctrl
    #    One "key = value" per line (decimal).
    # ------------------------------------------------------------------
    cfg = {
        'H_OUT'         : H_OUT,
        'W_OUT'         : W_OUT,
        'W_IN'          : W_IN,
        'K'             : K,
        'STRIDE'        : stride,
        'CIN_SLICES'    : cin_slices,
        'COUT_SLICES'   : cout_slices,
        'TILE_W'        : TILE_W,
        'TOTAL_WRF'     : total_wrf,
        'WRF_PACKED'    : 1 if wrf_packed else 0,
        'KK'            : kk,
        'ROUNDS_PER_CINS': rounds_per_cins,
        'ROUND_LEN_LAST' : round_len_last,
        'IFB_BASE'      : 0,
        'WB_BASE'       : 0,
        'OFB_BASE'      : 0,
        'IFB_CIN_STEP'  : IFB_CIN_STEP,
        'IFB_ROW_STEP'  : IFB_ROW_STEP,
        'WB_CIN_STEP'   : WB_CIN_STEP,
        'WB_COUT_STEP'  : WB_COUT_STEP,
        'OFB_COUT_STEP' : OFB_COUT_STEP,
        'NUM_TILES'     : num_tiles,
        'LAST_VALID_W'  : last_valid_w,
        'TILE_IN_STEP'  : TILE_IN_STEP,
        'SDP_SHIFT'     : shift_amt,
        'SDP_RELU_EN'   : 1,
    }
    with open('config.txt', 'w') as f:
        for k, v in cfg.items():
            f.write(f"{k} = {v}\n")

    # sim_params.f
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n+COUT_SLICES={cout_slices}\n"
                f"-gSRAM_DEPTH={hw_sram_depth}\n")

    print(f"Files  : ifb.txt  wb.txt  expected_ofm.txt  config.txt  sim_params.f")


if __name__ == '__main__':
    args = parse_args()
    generate(H_IN=args.h_in, W_IN=args.w_in, K=args.k,
             NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
             TILE_W=args.tile_w, seed=args.seed,
             shift_amt=args.shift, stride=args.stride,
             HW_PE=args.hw_pe, HW_COL=args.hw_col)
