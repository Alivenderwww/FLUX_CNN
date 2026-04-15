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
  K*K positions are split into rounds of WRF_DEPTH each.
  - K*K <= 32 (e.g. 1x1, 3x3, 5x5): single round, LD_WGT runs ONCE before the
    yout loop (true weight-stationary).
  - K*K >  32 (e.g. 6x6, 7x7):      multiple rounds, LD_WGT runs inside the
    loop body at each round boundary (weight-reload / weight-streaming).
  ARF state carries over between rounds: the sliding-window ARF is not reset
  by LD_WGT, so mid-row round transitions work correctly.

Stride handling:
  stride=1: LDnMAC(ld_len=TILE_W) for kx=0, LDnMAC(ld_len=1) sliding-window for kx=1..K-1.
            Sliding window works because only kx=0 does a full ARF reload; kx=1..K-1 each
            load just 1 new pixel into ARF while the MAC reads the rest from existing ARF state.
  stride>1: ALL kx do full reloads (ld_len=TILE_W for full tiles, ld_len=valid_w for partial).
            ARF sliding window is NOT applied for stride>1 because:
              - kx < stride each issue a full ld_len=TILE_W reload, all writing to ARF[0..TILE_W-1].
              - The second full reload (kx=1) overwrites kx=0's data, so kx=2's attempt to slide
                on kx=0 would read stale ARF content.
              - Additionally, kx//stride aliases consecutive kx values to the same slide_count
                (e.g. kx=2 and kx=3 both give slide_count=1 for stride=2).
            Future optimisation: interleave kx chains (0,2,4,.. then 1,3,5,..) with separate
            ARF regions to re-enable stride>1 sliding, but this requires ARF depth ≥ 2*TILE_W.
"""
import argparse
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
    p.add_argument('--tile_w',   type=int, default=32,  help='ARF tile width  (default 32)')
    p.add_argument('--seed',     type=int, default=42,  help='Random seed     (default 42)')
    p.add_argument('--shift',    type=int, default=0,
                   help='SDP dequant right-shift amount 0..31 (default 0 = no shift)')
    return p.parse_args()

# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed, shift_amt=0, stride=1):
    import random
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")

    # H_OUT / W_OUT for arbitrary stride (padding=0)
    H_OUT = (H_IN - K) // stride + 1
    W_OUT = (W_IN - K) // stride + 1

    if H_OUT <= 0 or W_OUT <= 0:
        sys.exit(f"ERROR: K={K}x{K} stride={stride} too large for {H_IN}x{W_IN} "
                 f"(output {H_OUT}x{W_OUT} is invalid).")

    # SRAM capacity check (must match SRAM_DEPTH in testbench / core_top)
    SRAM_DEPTH = 8192*64
    ifb_size = H_IN * W_IN
    ofb_size = H_OUT * W_OUT
    if ifb_size > SRAM_DEPTH:
        sys.exit(f"ERROR: IFB overflow! H_IN*W_IN = {ifb_size} > SRAM_DEPTH = {SRAM_DEPTH}. "
                 f"Max H_IN for W_IN={W_IN}: {SRAM_DEPTH // W_IN}")
    if ofb_size > SRAM_DEPTH:
        sys.exit(f"ERROR: OFB overflow! H_OUT*W_OUT = {ofb_size} > SRAM_DEPTH = {SRAM_DEPTH}. "
                 f"Max H_OUT for W_OUT={W_OUT}: {SRAM_DEPTH // W_OUT}")

    # ------------------------------------------------------------------
    # Split K*K kernel positions into WRF_DEPTH-sized rounds
    # ------------------------------------------------------------------
    all_positions = [(ky, kx) for ky in range(K) for kx in range(K)]
    rounds = [all_positions[i:i+WRF_DEPTH]
              for i in range(0, len(all_positions), WRF_DEPTH)]
    num_rounds = len(rounds)
    num_tiles  = (W_OUT + TILE_W - 1) // TILE_W

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, stride={stride}, "
          f"Cin={NUM_CIN}, Cout={NUM_COUT}, TILE_W={TILE_W}")
    print(f"Output : H_OUT={H_OUT}, W_OUT={W_OUT}, "
          f"TOTAL_OFM={H_OUT*W_OUT}, tiles/row={num_tiles}")
    print(f"Weight : K*K={K*K} positions -> {num_rounds} WRF round(s) "
          f"of up to {WRF_DEPTH} each "
          + ("(weight-stationary)" if num_rounds == 1
             else f"(weight-reload every yout x {num_tiles} tile(s))"))

    # ------------------------------------------------------------------
    # 1. IFB data
    # ------------------------------------------------------------------
    # 1. IFB data  (H_IN x W_IN pixels, each NUM_CIN channels x 8-bit)
    # ------------------------------------------------------------------
    ifb_hex_chars = NUM_CIN * 2  # each channel = 8-bit = 2 hex chars
    ifm_arr = [[[0]*NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
    ifb_data = []
    for y in range(H_IN):
        for x in range(W_IN):
            val = 0
            for cin in range(NUM_CIN):
                v = random.randint(0, 7)
                ifm_arr[y][x][cin] = v
                val |= (v & 0xFF) << (cin * 8)
            ifb_data.append(f"{val:0{ifb_hex_chars}X}")
    with open('ifb.txt', 'w') as f:
        f.writelines(d + '\n' for d in ifb_data)

    # ------------------------------------------------------------------
    # 2. WB data  (K*K weight positions stored flat)
    #    Each word = NUM_COUT columns x NUM_CIN PEs x 8-bit = NUM_COUT*NUM_CIN*8 bits
    # ------------------------------------------------------------------
    wb_hex_chars = NUM_COUT * NUM_CIN * 2  # total hex chars per WB word
    w_arr = [[[[0]*NUM_CIN for _ in range(NUM_COUT)] for _ in range(K)] for _ in range(K)]
    wb_data = []
    for ky in range(K):
        for kx in range(K):
            val = 0
            for cout in range(NUM_COUT):
                cv = 0
                for cin in range(NUM_CIN):
                    v = random.randint(-3, 3)
                    w_arr[ky][kx][cout][cin] = v
                    cv |= (v & 0xFF) << (cin * 8)
                val |= cv << (cout * NUM_CIN * 8)
            wb_data.append(f"{val:0{wb_hex_chars}X}")
    with open('wb.txt', 'w') as f:
        f.writelines(d + '\n' for d in wb_data)

    # ------------------------------------------------------------------
    # 3. Instructions (loop-compressed, multi-round weight scheduling)
    # ------------------------------------------------------------------
    # Register convention:
    #   r0 = IFB row base (stride=1: r0+=W_IN per row; stride>1: r0+=stride*W_IN per row)
    #   r1 = OFB base addr offset (r1 += W_OUT per row)
    #   r2 = yout loop counter
    # sram_addr in MAC/ST instructions is RELATIVE to r0/r1:
    #   stride=1 : actual_ifb = r0 + ky*W_IN + x_tile_out [+ cnt]
    #   stride>1 : actual_ifb = r0 + ky*W_IN + x_tile_in  [+ cnt*stride]
    #              where x_tile_in = x_tile_out * stride
    instructions = []

    # --- SDP config (once per layer) ---
    instructions.append(pack_ld_sdp(shift_amt))

    # --- Single-round: LD_WGT ONCE before the loop (true weight-stationary) ---
    if num_rounds == 1:
        instructions.append(
            pack_inst(OP_LD_WGT, wgt_addr=0, sram_addr=0, length=K * K))

    # --- Scalar register initialisation ---
    instructions.append(pack_li(rd=0, imm=0))           # r0 = 0
    instructions.append(pack_li(rd=1, imm=0))           # r1 = 0
    instructions.append(pack_li(rd=2, imm=H_OUT - 1))  # r2 = yout counter

    # --- Loop body start ---
    LOOP_START = len(instructions)

    for x_tile_out in range(0, W_OUT, TILE_W):          # unrolled over output tiles
        valid_w     = min(TILE_W, W_OUT - x_tile_out)
        x_tile_in   = x_tile_out * stride               # corresponding input column
        first_mac   = True   # clr_parf=1 only for the first MAC of each x_tile

        for round_idx, round_pos in enumerate(rounds):
            round_global_start = round_idx * WRF_DEPTH

            if num_rounds > 1:
                instructions.append(
                    pack_inst(OP_LD_WGT, wgt_addr=0,
                              sram_addr=round_global_start,
                              length=len(round_pos)))

            for local_idx, (ky, kx) in enumerate(round_pos):
                clr       = 1 if first_mac else 0
                first_mac = False

                if stride == 1:
                    # --------------------------------------------------
                    # stride=1: sliding-window optimization.
                    # Full tile (valid_w == TILE_W):
                    #   kx=0       → LDnMAC(ld_len=TILE_W): load all pixels fresh
                    #   kx=1..K-1  → LDnMAC(ld_len=1):      slide window by 1
                    # Partial tile (valid_w < TILE_W, last tile only):
                    #   All kx     → LDnMAC(ld_len=valid_w): full reload each tap,
                    #                no sliding (sliding would read beyond valid ARF range)
                    # --------------------------------------------------
                    if valid_w == TILE_W:
                        if kx == 0:
                            rel = ky * W_IN + x_tile_in
                            instructions.append(pack_ldnmac(
                                clr_parf=clr,
                                wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                                sram_addr=rel,
                                ld_len=TILE_W, mac_len=TILE_W,
                                stride=0, ld_arf_addr=0))
                        else:
                            # Sliding window: load 1 new pixel into ARF[kx-1],
                            # MAC reads ARF[kx .. kx+TILE_W-1]
                            rel = ky * W_IN + x_tile_in + TILE_W + kx - 1
                            instructions.append(pack_ldnmac(
                                clr_parf=clr,
                                wgt_addr=local_idx, parf_addr=0, arf_addr=kx,
                                sram_addr=rel,
                                ld_len=1, mac_len=TILE_W,
                                stride=0, ld_arf_addr=kx - 1))
                    else:
                        # Partial tile: reload valid_w pixels every tap, no sliding
                        rel = ky * W_IN + x_tile_in + kx
                        instructions.append(pack_ldnmac(
                            clr_parf=clr,
                            wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                            sram_addr=rel,
                            ld_len=valid_w, mac_len=valid_w,
                            stride=0, ld_arf_addr=0))
                else:
                    # --------------------------------------------------
                    # stride>1: full reload for ALL kx, both partial and full tiles.
                    #
                    # ARF sliding window is NOT used for stride>1 because:
                    #   - kx < stride each issue a full TILE_W reload into ARF[0..TILE_W-1].
                    #   - With stride>1 the second full reload (kx=1) overwrites kx=0,
                    #     so kx=2 cannot slide on kx=0 — the ARF precondition is violated.
                    #   - Additionally kx//stride aliases consecutive kx (e.g. kx=2 and kx=3
                    #     both give slide_count=1 for stride=2), creating a second aliasing bug.
                    #
                    # Partial tile (valid_w < TILE_W): use mac_len=valid_w to avoid waste.
                    # Full tile   (valid_w == TILE_W): use mac_len=TILE_W.
                    # --------------------------------------------------
                    if valid_w < TILE_W:
                        # Partial tile: reload valid_w pixels, MAC only valid_w cycles
                        rel = ky * W_IN + x_tile_in + kx
                        instructions.append(pack_ldnmac(
                            clr_parf=clr,
                            wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                            sram_addr=rel,
                            ld_len=valid_w, mac_len=valid_w,
                            stride=stride, ld_arf_addr=0))
                    else:
                        # Full tile: full reload with hardware IFB stride step
                        rel = ky * W_IN + x_tile_in + kx
                        instructions.append(pack_ldnmac(
                            clr_parf=clr,
                            wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                            sram_addr=rel,
                            ld_len=TILE_W, mac_len=TILE_W,
                            stride=stride, ld_arf_addr=0))

        # ST_OFM: write valid_w pixels to OFB (addr relative to r1)
        instructions.append(pack_inst(
            OP_ST_OFM, sdp_en=1, parf_addr=0,
            sram_addr=x_tile_out, length=valid_w))

    # --- Loop tail ---
    # r0 advances by stride*W_IN per output row (input row step = stride rows)
    instructions.append(pack_alu(rd=0, rs1=0, op='+', imm=stride * W_IN))
    instructions.append(pack_alu(rd=1, rs1=1, op='+', imm=W_OUT))
    instructions.append(pack_bnz(rs=2, target_pc=LOOP_START))

    instructions.append(pack_inst(OP_FINISH))

    with open('inst.txt', 'w') as f:
        f.writelines(inst + '\n' for inst in instructions)

    # --- sim_params.f for testbench plusargs and parameters ---
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n-gSRAM_DEPTH={SRAM_DEPTH}\n")

    # ------------------------------------------------------------------
    # 4. Expected OFM (golden reference, computed in Python)
    # ------------------------------------------------------------------
    expected_ofm = []
    for yout in range(H_OUT):
        for px in range(W_OUT):
            pixel_val = 0
            for cout in range(NUM_COUT):
                psum = 0
                for ky in range(K):
                    for kx in range(K):
                        for cin in range(NUM_CIN):
                            # input pixel: top-left = (yout*stride, px*stride)
                            iy = yout * stride + ky
                            ix = px   * stride + kx
                            psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                act = max(0, min(255, psum >> shift_amt))
                pixel_val |= (act & 0xFF) << (cout * 8)
            expected_ofm.append(f"{pixel_val:0{NUM_COUT * 2}X}")
    with open('expected_ofm.txt', 'w') as f:
        f.writelines(d + '\n' for d in expected_ofm)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    n = len(instructions)
    print(f"Instructions : {n} total  (LOOP_START=PC {LOOP_START}, "
          f"runs {H_OUT} times)")
    print(f"Files written: ifb.txt  wb.txt  inst.txt  expected_ofm.txt  sim_params.f")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    args = parse_args()
    generate(H_IN=args.h_in, W_IN=args.w_in, K=args.k,
             NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
             TILE_W=args.tile_w, seed=args.seed,
             shift_amt=args.shift, stride=args.stride)
