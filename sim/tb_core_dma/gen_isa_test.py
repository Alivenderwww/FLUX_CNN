"""
gen_isa_test.py -- CNN Accelerator Configuration Test Generator

Usage:
  python gen_isa_test.py [--h_in H] [--w_in W] [--k K] [--stride S]
                         [--num_cin C] [--num_cout N] [--tile_w T]
                         [--shift SH] [--seed SD] [--pad P]
                         [--strip_rows N] [--streaming]

Outputs:
  - ifb.txt           : packed input feature map (same layout as before)
  - wb.txt            : packed weights (same layout as before)
  - expected_ofm.txt  : golden output for TB comparison
  - config.txt        : key=value register map consumed by TB
  - sim_params.f      : plusargs for TB
  - desc_list.hex     : Phase C-4 descriptor list (256-bit/desc = 2× 128-bit AXI beat)
                        little-endian: word[3..0] in lower beat, word[7..4] in upper
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
    p.add_argument('--streaming', action='store_true',
                   help='emit streaming-ring cfg (requires cin_slices=1 and cout_slices=1)')
    p.add_argument('--ifb_strip', type=int, default=8, help='IFB ring strip_rows')
    p.add_argument('--ofb_strip', type=int, default=8, help='OFB ring strip_rows')
    # Phase C-2 padding
    p.add_argument('--pad',      type=int, default=0, help='symmetric pad (top=left=pad)')
    p.add_argument('--pad_top',  type=int, default=None)
    p.add_argument('--pad_left', type=int, default=None)
    # Phase C-4 descriptor
    p.add_argument('--strip_rows', type=int, default=0,
                   help='Phase C-4: 0 = single strip (legacy) / >0 = multi-strip descriptor chain')
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
             shift_amt=0, stride=1, HW_PE=16, HW_COL=16,
             streaming=False, ifb_strip=8, ofb_strip=8,
             pad_top=0, pad_left=0, strip_rows=0):
    pad_bot, pad_right = pad_top, pad_left  # 对称 pad
    import random
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")

    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

    # Phase C-2: H/W_OUT 含 padding
    H_OUT = (H_IN + pad_top + pad_bot - K) // stride + 1
    W_OUT = (W_IN + pad_left + pad_right - K) // stride + 1
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

    PYTHON_SRAM_LIMIT = 524288   # 512K words，支持 480x640 单 slice
    for name, sz in [('IFB', ifb_size), ('WB', wb_size), ('OFB', ofb_size)]:
        if sz > PYTHON_SRAM_LIMIT:
            sys.exit(f"ERROR: {name} overflow! size={sz} > {PYTHON_SRAM_LIMIT}")

    if streaming:
        # streaming 下物理 SRAM 放 ring，不需要按整图尺寸扩大
        hw_sram_depth = 8192
    else:
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
                                iy = yout * stride + ky - pad_top
                                ix = px   * stride + kx - pad_left
                                if 0 <= iy < H_IN and 0 <= ix < W_IN:
                                    psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                                # pad: contributes 0
                    act = max(0, min(255, psum >> shift_amt))
                    pixel_val |= (act & 0xFF) << (lc * 8)
                expected_ofm.append(f"{pixel_val:0{HW_COL * 2}X}")
    with open('expected_ofm.txt', 'w') as f:
        f.writelines(d + '\n' for d in expected_ofm)

    # ------------------------------------------------------------------
    # 4. config.txt -- register map for TB to poke into core_ctrl
    #    One "key = value" per line (decimal).
    # ------------------------------------------------------------------
    # Streaming (v2) requires cin_slices=1 AND cout_slices=1
    if streaming and (cin_slices != 1 or cout_slices != 1):
        sys.exit(f"ERROR: --streaming requires cin_slices=1 && cout_slices=1 "
                 f"(got cin_slices={cin_slices}, cout_slices={cout_slices})")

    # 在 streaming 下，IFB_CIN_STEP / OFB_COUT_STEP 含义变为 ring wrap 模数
    ifb_cin_step_eff  = ifb_strip * W_IN  if streaming else IFB_CIN_STEP
    ofb_cout_step_eff = ofb_strip * W_OUT if streaming else OFB_COUT_STEP
    dma_mode_val      = 0x3 if streaming else 0x0

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
        'IFB_CIN_STEP'  : ifb_cin_step_eff,
        'IFB_ROW_STEP'  : IFB_ROW_STEP,
        'WB_CIN_STEP'   : WB_CIN_STEP,
        'WB_COUT_STEP'  : WB_COUT_STEP,
        'OFB_COUT_STEP' : ofb_cout_step_eff,
        'NUM_TILES'     : num_tiles,
        'LAST_VALID_W'  : last_valid_w,
        'TILE_IN_STEP'  : TILE_IN_STEP,
        'SDP_SHIFT'     : shift_amt,
        'SDP_RELU_EN'   : 1,
        # --- Phase C-2 padding (global CSR 临时路径) ---
        'PAD_TOP'            : pad_top,
        'PAD_LEFT'           : pad_left,
        # --- streaming v2 fields (batch mode 留 0 / 写入不影响) ---
        'H_IN_TOTAL'         : H_IN,     # Phase C-2: batch 下也需，供 line_buffer is_pad_bot_y 判定
        'IFB_STRIP_ROWS'     : ifb_strip if streaming else 0,
        'OFB_STRIP_ROWS'     : ofb_strip if streaming else 0,
        'DDR_IFM_ROW_STRIDE' : W_IN  * 16 if streaming else 0,
        'DDR_OFM_ROW_STRIDE' : W_OUT * 16 if streaming else 0,
        'DMA_MODE'           : dma_mode_val,
    }
    with open('config.txt', 'w') as f:
        for k, v in cfg.items():
            f.write(f"{k} = {v}\n")

    # ------------------------------------------------------------------
    # 5. Phase C-4: descriptor list (desc_list.hex)
    #
    # 格式：每条 descriptor 256 bit = 2 × 128-bit AXI beat，每 beat 一行
    # (32 hex chars)。beat0 (低 128 bit) 在先，beat1 (高 128 bit) 在后。
    #
    # Descriptor 字段布局 (见 docs/descriptor-sequencer.md §2.1)：
    #   word 0 [3:0]    type            (0=NOP 1=CONV 2=BARRIER F=END)
    #   word 0 [7:4]    flags           {rsvd, streaming_en, is_last, is_first}
    #   word 0 [11:8]   pad_top
    #   word 0 [15:12]  pad_bot
    #   word 0 [19:16]  pad_left
    #   word 0 [23:20]  pad_right
    #   word 1 [15:0]   strip_y_start
    #   word 1 [31:16]  n_yout_strip
    #   word 2 [19:0]   ifb_ddr_offset (bytes)
    #   word 3 [23:0]   ifb_byte_len
    #   word 4 [19:0]   ofb_ddr_offset (bytes)
    #   word 5 [23:0]   ofb_byte_len
    #   word 6..7       rsvd
    #
    # Single strip (strip_rows=0 or >=H_OUT): 1 条 CONV + 1 条 END，等价 legacy
    # Multi strip: ceil(H_OUT/strip_rows) 条 CONV + 1 条 END
    # ------------------------------------------------------------------
    def pack_desc(type_, flags, pad_t, pad_b, pad_l, pad_r,
                  strip_y_start, n_yout, ifb_off, ifb_len, ofb_off, ofb_len):
        w0 = ((type_ & 0xF)      << 0 ) | \
             ((flags & 0xF)      << 4 ) | \
             ((pad_t & 0xF)      << 8 ) | \
             ((pad_b & 0xF)      << 12) | \
             ((pad_l & 0xF)      << 16) | \
             ((pad_r & 0xF)      << 20)
        w1 = (strip_y_start & 0xFFFF) | ((n_yout & 0xFFFF) << 16)
        w2 = ifb_off & 0xFFFFF
        w3 = ifb_len & 0xFFFFFF
        w4 = ofb_off & 0xFFFFF
        w5 = ofb_len & 0xFFFFFF
        w6 = 0
        w7 = 0
        desc256 = (w0)       | (w1 << 32)  | (w2 << 64)  | (w3 << 96) | \
                  (w4 << 128)| (w5 << 160) | (w6 << 192) | (w7 << 224)
        beat0 = desc256 & ((1 << 128) - 1)
        beat1 = (desc256 >> 128) & ((1 << 128) - 1)
        return beat0, beat1

    TYPE_NOP, TYPE_CONV, TYPE_BARRIER, TYPE_END = 0x0, 0x1, 0x2, 0xF
    FLAG_IS_FIRST     = 1 << 0
    FLAG_IS_LAST      = 1 << 1
    FLAG_STREAMING_EN = 1 << 2

    strip_rows_eff = strip_rows if (strip_rows > 0 and strip_rows < H_OUT) else H_OUT
    n_strips = (H_OUT + strip_rows_eff - 1) // strip_rows_eff

    # Byte sizes per row (AXI 128-bit word = 16 byte)
    AXI_BYTES = 16
    ifb_bytes_per_row = W_IN  * AXI_BYTES * cin_slices
    ofb_bytes_per_row = W_OUT * AXI_BYTES * cout_slices
    ifb_total_bytes   = H_IN  * ifb_bytes_per_row
    ofb_total_bytes   = H_OUT * ofb_bytes_per_row

    descs = []
    for i in range(n_strips):
        strip_y_start = i * strip_rows_eff
        n_yout        = min(strip_rows_eff, H_OUT - strip_y_start)
        is_first      = (i == 0)
        is_last       = (i == n_strips - 1)

        # Per-strip pad：边缘 strip 保留全局 pad，内部 strip 归零
        local_pad_top   = pad_top   if is_first else 0
        local_pad_bot   = pad_bot   if is_last  else 0
        local_pad_left  = pad_left              # 水平 pad 所有 strip 共享
        local_pad_right = pad_right

        # IFB DDR offset：strip 对应的 IFM 起始行
        #   strip 第一个 yout 访问的最小 src_y = strip_y_start*stride - pad_top
        #   取 max(0) 去掉 pad 部分（DDR 里无 pad 数据）
        ifb_y_start = max(0, strip_y_start * stride - pad_top)
        ifb_y_end   = min(H_IN, (strip_y_start + n_yout - 1) * stride + K - pad_top)
        ifb_rows    = max(0, ifb_y_end - ifb_y_start)
        ifb_ddr_offset = ifb_y_start * ifb_bytes_per_row
        ifb_byte_len   = ifb_rows    * ifb_bytes_per_row

        # OFB DDR offset：strip 产出的 OFM 起始行
        ofb_ddr_offset = strip_y_start * ofb_bytes_per_row
        ofb_byte_len   = n_yout        * ofb_bytes_per_row

        flags = 0
        if is_first: flags |= FLAG_IS_FIRST
        if is_last:  flags |= FLAG_IS_LAST
        if streaming: flags |= FLAG_STREAMING_EN

        descs.append(pack_desc(
            TYPE_CONV, flags,
            local_pad_top, local_pad_bot, local_pad_left, local_pad_right,
            strip_y_start, n_yout,
            ifb_ddr_offset, ifb_byte_len,
            ofb_ddr_offset, ofb_byte_len))

    # END descriptor
    descs.append(pack_desc(TYPE_END, 0, 0, 0, 0, 0, H_OUT, 0, 0, 0, 0, 0))

    with open('desc_list.hex', 'w') as f:
        for (beat0, beat1) in descs:
            f.write(f"{beat0:032X}\n")
            f.write(f"{beat1:032X}\n")

    # 补充 config.txt 的 DESC_COUNT（TB 写入 0x184 CSR，DFE 用）
    with open('config.txt', 'a') as f:
        f.write(f"DESC_COUNT = {len(descs)}\n")
        f.write(f"DESC_LIST_BASE = 0\n")   # DDR 起始地址（TB 侧决定实际物理 base）

    print(f"Desc   : n_strips={n_strips} (+1 END), total={len(descs)} descriptors, "
          f"strip_rows={strip_rows_eff}")

    # sim_params.f — 供 TB 通过 plusargs 获取尺寸（避免硬编码默认值）
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n+COUT_SLICES={cout_slices}\n"
                f"+IFB_WORDS={ifb_size}\n+WB_WORDS={wb_size}\n"
                f"+DESC_COUNT={len(descs)}\n"
                f"-gSRAM_DEPTH={hw_sram_depth}\n")

    print(f"Files  : ifb.txt  wb.txt  expected_ofm.txt  config.txt  sim_params.f  desc_list.hex")


if __name__ == '__main__':
    args = parse_args()
    pad_t = args.pad_top  if args.pad_top  is not None else args.pad
    pad_l = args.pad_left if args.pad_left is not None else args.pad
    generate(H_IN=args.h_in, W_IN=args.w_in, K=args.k,
             NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
             TILE_W=args.tile_w, seed=args.seed,
             shift_amt=args.shift, stride=args.stride,
             HW_PE=args.hw_pe, HW_COL=args.hw_col,
             streaming=args.streaming,
             ifb_strip=args.ifb_strip, ofb_strip=args.ofb_strip,
             pad_top=pad_t, pad_left=pad_l,
             strip_rows=args.strip_rows)
