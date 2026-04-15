"""
gen_isa_test.py -- CNN Accelerator ISA Test Generator

Usage:
  python gen_isa_test.py [--h_in H] [--w_in W] [--k K] [--stride S]
                         [--num_cin C] [--num_cout N] [--tile_w T] [--seed SD]
                         [--shift SH]

  H_OUT = (H_IN - K) // stride + 1  (padding=0)
  W_OUT = (W_IN - K) // stride + 1

Weight scheduling:
  WRF_DEPTH = 32 (hardware fixed, 5-bit wgt_rf_addr).
  Total WRF slots needed = K*K * cin_slices.
  - If total <= 32: "packed" mode -- load all cin_slices' weights BEFORE the yout
    loop (true weight-stationary). WRF slot for (cins, k_pos) = cins*K² + k_pos.
  - If total > 32:  "chunked" mode -- reload per cin_slice inside the loop body.
    wgt_addr restarts at 0 for each cin_slice's LD_WGT.
    Within each cin_slice, if K² > 32 the existing round mechanism applies.

Cin slicing (NUM_CIN > HW_PE):
  IFB layout: cin_slice 0 occupies IFB[0..H_IN*W_IN-1],
              cin_slice 1 occupies IFB[H_IN*W_IN..2*H_IN*W_IN-1], etc.
  The yout loop uses r0 as the cin_slice-0 row base. Before each cin_slice's
  LDnMAC block, an ALU instruction advances r0 by IFB_SLICE_STRIDE (=H_IN*W_IN).
  After the last cin_slice, r0 is restored to the cin_slice-0 base so the loop
  tail (r0 += stride*W_IN) advances correctly.

  PARF accumulation: clr_parf=1 ONLY on the very first LDnMAC across all cin_slices
  for a given tile; all subsequent LDnMAC (including those for later cin_slices)
  keep clr_parf=0. ST_OFM is emitted ONCE after all cin_slices complete.

Cout slicing (NUM_COUT > HW_COL):
  The outermost loop iterates over cout_slices; each slice drives its own
  LD_WGT + yout loop and writes to a separate OFB region.
  Cout is guaranteed to be fully generated (all channels contiguous per pixel)
  so that downstream layer-fusion reads a complete activation before moving on.

Stride handling:
  stride=1: LDnMAC(ld_len=TILE_W) for kx=0, LDnMAC(ld_len=1) sliding-window
            for kx=1..K-1 (ARF reuse).
  stride>1: ALL kx do full reloads.
"""
import argparse
import math
import sys

# ---------------------------------------------------------------------------
# Opcode constants
# ---------------------------------------------------------------------------
OP_NOP     = 0
OP_LD_WGT  = 1
OP_LD_ARF  = 2
OP_LD_PARF = 3
OP_ST_OFM  = 4
OP_MAC_RUN = 5
OP_LD1MAC  = 6   # (Legacy) ld_len=1 special case, encoded as OP_LDnMAC in hardware
OP_LDnMAC  = 7   # Unified Load-n-and-MAC: ld_len pixels loaded, mac_len cycles MAC
                 # length[15:11] = ld_len-1  (0..31 → load 1..32 pixels)
                 # length[10:0]  = mac_len   (MAC active cycles, typically TILE_W)
OP_FINISH  = 15
OP_LI      = 8   # Load Immediate:     scalar_rf[rd] <- imm16
OP_ALU     = 9   # Scalar ALU:         rd <- rs1 +/- rs2  or  rd <- rs1 +/- imm16
OP_JMP     = 10  # Unconditional Jump: PC <- target_pc
OP_BNZ     = 11  # Branch if Not Zero: if rs!=0: rs--, PC<-target_pc
OP_LD_SDP  = 12  # Load SDP params:    SDP.shift_amt <- ld_arf_addr[4:0]

WRF_DEPTH = 32   # Hardware fixed: 5-bit wgt_rf_addr field (max address = 31)

# ---------------------------------------------------------------------------
# Instruction packing helpers
# ---------------------------------------------------------------------------
def pack_inst(opcode, clr_parf=0, sdp_en=0, arf_addr=0, wgt_addr=0,
              parf_addr=0, sram_addr=0, length=0, stride=0, ld_arf_addr=0):
    inst  = (opcode      & 0xF)    << 60
    inst |= (clr_parf    & 0x1)    << 59
    inst |= (sdp_en      & 0x1)    << 58
    inst |= (arf_addr    & 0x1F)   << 53
    inst |= (wgt_addr    & 0x1F)   << 48
    inst |= (parf_addr   & 0x1F)   << 43
    inst |= (sram_addr   & 0x7FFF) << 28
    inst |= (length      & 0xFFFF) << 12
    # [11:8] reserved = 0
    inst |= (stride      & 0x7)    << 5
    inst |= (ld_arf_addr & 0x1F)
    return f"{inst:016X}"

def pack_ldnmac(clr_parf=0, wgt_addr=0, parf_addr=0, arf_addr=0,
                sram_addr=0, ld_len=1, mac_len=32, stride=0, ld_arf_addr=0):
    """Pack a unified LDnMAC instruction (opcode=7).

    ld_len  : number of pixels to load from IFB into ARF (1..32)
    mac_len : number of MAC active cycles (typically TILE_W=32)
    ld_arf_addr : ARF write base address for loaded pixels
    arf_addr    : ARF read base address for MAC
    stride  : IFB read stride (0=1, 1=1, 2..7=2..7)

    length field encoding: length[15:11] = ld_len-1,  length[10:0] = mac_len
    """
    assert 1 <= ld_len <= 32, f"ld_len={ld_len} out of range [1..32]"
    assert 0 <= mac_len <= 2047, f"mac_len={mac_len} out of range [0..2047]"
    encoded_length = ((ld_len - 1) << 11) | (mac_len & 0x7FF)
    return pack_inst(OP_LDnMAC, clr_parf=clr_parf, arf_addr=arf_addr,
                     wgt_addr=wgt_addr, parf_addr=parf_addr,
                     sram_addr=sram_addr, length=encoded_length,
                     stride=stride, ld_arf_addr=ld_arf_addr)

def pack_li(rd, imm):
    return pack_inst(OP_LI, arf_addr=rd, length=imm & 0xFFFF)

def pack_alu(rd, rs1, op='+', imm=None, rs2=None):
    sub = 1 if op == '-' else 0
    if imm is not None:
        return pack_inst(OP_ALU, clr_parf=sub, sdp_en=1,
                         arf_addr=rd, wgt_addr=rs1, length=imm & 0xFFFF)
    return pack_inst(OP_ALU, clr_parf=sub, sdp_en=0,
                     arf_addr=rd, wgt_addr=rs1, parf_addr=rs2)

def pack_jmp(target_pc):
    return pack_inst(OP_JMP, sram_addr=target_pc)

def pack_bnz(rs, target_pc):
    return pack_inst(OP_BNZ, arf_addr=rs, sram_addr=target_pc)

def pack_ld_sdp(shift_amt):
    """OP_LD_SDP: SDP.shift_amt <- shift_amt[4:0]  (ld_arf_addr field)"""
    return pack_inst(OP_LD_SDP, ld_arf_addr=shift_amt & 0x1F)

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description='Generate ISA test files for CNN accelerator.')
    p.add_argument('--h_in',     type=int, default=123, help='Input height  (default 123)')
    p.add_argument('--w_in',     type=int, default=45,  help='Input width   (default 45)')
    p.add_argument('--k',        type=int, default=3,   help='Kernel size KxK (default 3)')
    p.add_argument('--stride',   type=int, default=1,   help='Convolution stride (default 1, max 7)')
    p.add_argument('--num_cin',  type=int, default=16,  help='Input channels  (default 16)')
    p.add_argument('--num_cout', type=int, default=16,  help='Output channels (default 16)')
    p.add_argument('--hw_pe',    type=int, default=16,  help='Hardware PE count per column (default 16)')
    p.add_argument('--hw_col',   type=int, default=16,  help='Hardware column count (default 16)')
    p.add_argument('--tile_w',   type=int, default=32,  help='ARF tile width  (default 32)')
    p.add_argument('--seed',     type=int, default=42,  help='Random seed     (default 42)')
    p.add_argument('--shift',    type=int, default=0,
                   help='SDP dequant right-shift amount 0..31 (default 0 = no shift)')
    return p.parse_args()

# ---------------------------------------------------------------------------
# Helper: emit one LDnMAC instruction
# Encapsulates the stride=1 sliding-window vs stride>1 full-reload logic.
# ---------------------------------------------------------------------------
def _emit_ldnmac(instructions, clr, wgt_addr, ky, kx, x_tile_in, valid_w,
                 stride, W_IN, TILE_W):
    """Append one LDnMAC instruction.

    For stride=1 and full tiles: kx=0 loads TILE_W pixels, kx>0 slides by 1.
    For stride>1 or partial tiles: always full-reload.
    """
    if stride == 1:
        if valid_w == TILE_W:
            if kx == 0:
                # Full tile load, ARF base = 0
                instructions.append(pack_ldnmac(
                    clr_parf=clr,
                    wgt_addr=wgt_addr, parf_addr=0, arf_addr=0,
                    sram_addr=ky * W_IN + x_tile_in,
                    ld_len=TILE_W, mac_len=TILE_W,
                    stride=0, ld_arf_addr=0))
            else:
                # Sliding window: load 1 new pixel at ARF[kx-1], MAC reads from ARF[kx]
                instructions.append(pack_ldnmac(
                    clr_parf=clr,
                    wgt_addr=wgt_addr, parf_addr=0, arf_addr=kx,
                    sram_addr=ky * W_IN + x_tile_in + TILE_W + kx - 1,
                    ld_len=1, mac_len=TILE_W,
                    stride=0, ld_arf_addr=kx - 1))
        else:
            # Partial tile: full reload, no sliding window
            instructions.append(pack_ldnmac(
                clr_parf=clr,
                wgt_addr=wgt_addr, parf_addr=0, arf_addr=0,
                sram_addr=ky * W_IN + x_tile_in + kx,
                ld_len=valid_w, mac_len=valid_w,
                stride=0, ld_arf_addr=0))
    else:
        # stride > 1: always full reload
        ld_len = valid_w if valid_w < TILE_W else TILE_W
        instructions.append(pack_ldnmac(
            clr_parf=clr,
            wgt_addr=wgt_addr, parf_addr=0, arf_addr=0,
            sram_addr=ky * W_IN + x_tile_in + kx,
            ld_len=ld_len, mac_len=ld_len,
            stride=stride, ld_arf_addr=0))

# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
             shift_amt=0, stride=1, HW_PE=16, HW_COL=16):
    import random
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")

    # Slice counts
    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

    # Output dimensions (padding=0)
    H_OUT = (H_IN - K) // stride + 1
    W_OUT = (W_IN - K) // stride + 1

    if H_OUT <= 0 or W_OUT <= 0:
        sys.exit(f"ERROR: K={K}x{K} stride={stride} too large for {H_IN}x{W_IN} "
                 f"(output {H_OUT}x{W_OUT} is invalid).")

    # ------------------------------------------------------------------
    # SRAM sizing
    # IFB: cin_slices regions, each H_IN*W_IN words
    # WB:  cout_slices * cin_slices * K*K words
    # OFB: cout_slices * H_OUT*W_OUT words
    # Hardware uses a single SRAM_DEPTH for all three; take the max and
    # round up to the next power-of-2 (min 8192) for clean addressing.
    # ------------------------------------------------------------------
    IFB_SLICE_STRIDE = H_IN * W_IN     # IFB words per cin_slice region
    ifb_size = IFB_SLICE_STRIDE * cin_slices
    wb_size  = K * K * cout_slices * cin_slices
    ofb_size = H_OUT * W_OUT * cout_slices

    PYTHON_SRAM_LIMIT = 8192 * 64
    if ifb_size > PYTHON_SRAM_LIMIT:
        sys.exit(f"ERROR: IFB overflow! ifb_size={ifb_size} > {PYTHON_SRAM_LIMIT}. "
                 f"Reduce H_IN/W_IN or NUM_CIN.")
    if ofb_size > PYTHON_SRAM_LIMIT:
        sys.exit(f"ERROR: OFB overflow! ofb_size={ofb_size} > {PYTHON_SRAM_LIMIT}.")

    hw_sram_depth = max(ifb_size, ofb_size, wb_size, 8192)
    hw_sram_depth = 1 << math.ceil(math.log2(hw_sram_depth))

    # ------------------------------------------------------------------
    # Weight scheduling: packed vs chunked
    #
    # packed  (K²*cin_slices ≤ WRF_DEPTH):
    #   Single LD_WGT before the yout loop loads all cin_slices at once.
    #   WRF slot for (cins, k_pos_local) = cins * K² + k_pos_local.
    #
    # chunked (K²*cin_slices > WRF_DEPTH):
    #   LD_WGT per cin_slice inside the loop body.
    #   wgt_addr restarts at 0 for each cin_slice.
    #   If K² > WRF_DEPTH, K²-positions are further split into rounds.
    # ------------------------------------------------------------------
    all_positions    = [(ky, kx) for ky in range(K) for kx in range(K)]
    total_wrf_needed = K * K * cin_slices
    wrf_packed       = total_wrf_needed <= WRF_DEPTH

    # Chunked K²-rounds (each ≤ WRF_DEPTH positions)
    chunk_rounds = [all_positions[i:i + WRF_DEPTH]
                    for i in range(0, len(all_positions), WRF_DEPTH)]

    num_tiles = (W_OUT + TILE_W - 1) // TILE_W

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, stride={stride}, "
          f"Cin={NUM_CIN} ({cin_slices} slice(s)), Cout={NUM_COUT} ({cout_slices} slice(s)), "
          f"TILE_W={TILE_W}")
    print(f"Output : H_OUT={H_OUT}, W_OUT={W_OUT}, "
          f"TOTAL_OFM={H_OUT * W_OUT}, tiles/row={num_tiles}")
    if wrf_packed:
        print(f"Weight : K*K*cin_slices={total_wrf_needed} <= WRF_DEPTH={WRF_DEPTH} "
              f"-> packed (weight-stationary, single LD_WGT per cout_slice before loop)")
    else:
        print(f"Weight : K*K*cin_slices={total_wrf_needed} > WRF_DEPTH={WRF_DEPTH} "
              f"-> chunked ({len(chunk_rounds)} K*K-round(s) x {cin_slices} cin_slice(s) "
              f"per tile, LD_WGT inside loop body)")
    print(f"HW SRAM: IFB={ifb_size}, WB={wb_size}, OFB={ofb_size} → SRAM_DEPTH={hw_sram_depth}")

    # ------------------------------------------------------------------
    # 1. IFB data
    #    Layout: [cin_slice 0: H_IN*W_IN words] [cin_slice 1: H_IN*W_IN words] ...
    #    Each IFB word = HW_PE channels × 8-bit.
    #    Only local_cin channels carry valid data; higher channels are zero-padded.
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
    # 2. WB data
    #    Layout: for each (cout_slice cs, cin_slice cins): K*K consecutive words.
    #    WB address = (cs * cin_slices + cins) * K * K + ky * K + kx
    #    Each WB word = HW_COL × HW_PE × 8-bit.
    #    Only (local_cout columns × local_cin channels) carry valid weights.
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
    # 3. Instructions
    #
    # Register convention (per cout_slice yout loop):
    #   r0 = IFB row base for cin_slice-0 of current yout row
    #        (stepped +IFB_SLICE_STRIDE / -(cin_slices-1)*IFB_SLICE_STRIDE
    #         inside tile body to switch between cin_slices)
    #   r1 = OFB base for this cout_slice (offset = cs * H_OUT * W_OUT)
    #   r2 = yout loop counter (H_OUT-1 down to 0, decremented by BNZ)
    # ------------------------------------------------------------------
    instructions = []

    # SDP config (once per layer)
    instructions.append(pack_ld_sdp(shift_amt))

    for cs in range(cout_slices):
        # WB base for all (cin_slices × K²) weights of this cout_slice
        wb_base_cs = cs * cin_slices * K * K

        # ---- PACKED: hoist all cin_slices' weights before yout loop ----
        if wrf_packed:
            # WB[wb_base_cs .. wb_base_cs + total_wrf_needed - 1] are contiguous
            instructions.append(
                pack_inst(OP_LD_WGT, wgt_addr=0,
                          sram_addr=wb_base_cs, length=total_wrf_needed))

        # Scalar register init
        instructions.append(pack_li(rd=0, imm=0))                         # r0 = cin0 IFB row base
        instructions.append(pack_li(rd=1, imm=cs * H_OUT * W_OUT))        # r1 = OFB base
        instructions.append(pack_li(rd=2, imm=H_OUT - 1))                 # r2 = yout counter

        LOOP_START = len(instructions)

        for x_tile_out in range(0, W_OUT, TILE_W):
            valid_w   = min(TILE_W, W_OUT - x_tile_out)
            x_tile_in = x_tile_out * stride
            first_mac = True   # clr_parf=1 only on the very first LDnMAC in this tile

            for cins in range(cin_slices):

                if wrf_packed:
                    # ---- PACKED: wrf_offset = cins * K² ----
                    wrf_offset = cins * K * K
                    for local_idx, (ky, kx) in enumerate(all_positions):
                        clr = 1 if first_mac else 0
                        first_mac = False
                        _emit_ldnmac(instructions, clr,
                                     wrf_offset + local_idx,
                                     ky, kx, x_tile_in, valid_w,
                                     stride, W_IN, TILE_W)

                else:
                    # ---- CHUNKED: reload WRF per cin_slice ----
                    wb_base_cins = wb_base_cs + cins * K * K
                    for round_idx, round_pos in enumerate(chunk_rounds):
                        round_start = wb_base_cins + round_idx * WRF_DEPTH
                        instructions.append(
                            pack_inst(OP_LD_WGT, wgt_addr=0,
                                      sram_addr=round_start,
                                      length=len(round_pos)))
                        for local_idx, (ky, kx) in enumerate(round_pos):
                            clr = 1 if first_mac else 0
                            first_mac = False
                            _emit_ldnmac(instructions, clr,
                                         local_idx,   # wgt_addr restarts at 0 per round
                                         ky, kx, x_tile_in, valid_w,
                                         stride, W_IN, TILE_W)

                # ---- Advance r0 to next cin_slice's IFB region ----
                if cins < cin_slices - 1:
                    instructions.append(
                        pack_alu(rd=0, rs1=0, op='+', imm=IFB_SLICE_STRIDE))

            # ---- Restore r0 to cin_slice-0 base (after last cin_slice) ----
            if cin_slices > 1:
                instructions.append(
                    pack_alu(rd=0, rs1=0, op='-',
                             imm=(cin_slices - 1) * IFB_SLICE_STRIDE))

            # ---- ST_OFM: emit ONCE per tile after all cin_slices ----
            instructions.append(pack_inst(
                OP_ST_OFM, sdp_en=1, parf_addr=0,
                sram_addr=x_tile_out, length=valid_w))

        # Loop tail: advance row bases, decrement counter
        instructions.append(pack_alu(rd=0, rs1=0, op='+', imm=stride * W_IN))
        instructions.append(pack_alu(rd=1, rs1=1, op='+', imm=W_OUT))
        instructions.append(pack_bnz(rs=2, target_pc=LOOP_START))

    instructions.append(pack_inst(OP_FINISH))

    with open('inst.txt', 'w') as f:
        f.writelines(inst + '\n' for inst in instructions)

    # --- sim_params.f for testbench plusargs ---
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n+COUT_SLICES={cout_slices}\n"
                f"-gSRAM_DEPTH={hw_sram_depth}\n")

    # ------------------------------------------------------------------
    # 4. Expected OFM (golden reference)
    #    Full convolution accumulating across ALL cin channels (sum over slices).
    #    Layout: cout_slice 0 region [0..H*W-1], cout_slice 1 [H*W..], ...
    #    Each entry = HW_COL × 8-bit word.
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
                            for cin in range(NUM_CIN):   # all channels, all slices
                                iy = yout * stride + ky
                                ix = px   * stride + kx
                                psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                    act = max(0, min(255, psum >> shift_amt))
                    pixel_val |= (act & 0xFF) << (lc * 8)
                expected_ofm.append(f"{pixel_val:0{HW_COL * 2}X}")
    with open('expected_ofm.txt', 'w') as f:
        f.writelines(d + '\n' for d in expected_ofm)

    # Summary
    n = len(instructions)
    print(f"Instructions : {n} total  ({cout_slices} cout_slice(s) × {cin_slices} cin_slice(s), "
          f"runs {H_OUT} yout rows per slice combo)")
    print(f"Files written: ifb.txt  wb.txt  inst.txt  expected_ofm.txt  sim_params.f")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    args = parse_args()
    generate(H_IN=args.h_in, W_IN=args.w_in, K=args.k,
             NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
             TILE_W=args.tile_w, seed=args.seed,
             shift_amt=args.shift, stride=args.stride,
             HW_PE=args.hw_pe, HW_COL=args.hw_col)
