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
    p.add_argument('--hw_pe',    type=int, default=16,  help='Hardware PE count per column (default 16)')
    p.add_argument('--hw_col',   type=int, default=16,  help='Hardware column count (default 16)')
    p.add_argument('--tile_w',   type=int, default=32,  help='ARF tile width  (default 32)')
    p.add_argument('--seed',     type=int, default=42,  help='Random seed     (default 42)')
    p.add_argument('--shift',    type=int, default=0,
                   help='SDP dequant right-shift amount 0..31 (default 0 = no shift)')
    return p.parse_args()

# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
             shift_amt=0, stride=1, HW_PE=16, HW_COL=16):
    import random
    random.seed(seed)

    if NUM_CIN > HW_PE:
        sys.exit(f"ERROR: NUM_CIN={NUM_CIN} > HW_PE={HW_PE}. Multi-cin-slice not yet supported.")

    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

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
    ofb_size = H_OUT * W_OUT * cout_slices   # each cout_slice occupies its own OFB region
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
          f"TOTAL_OFM={H_OUT*W_OUT}, tiles/row={num_tiles}, cout_slices={cout_slices}")
    print(f"Weight : K*K={K*K} positions -> {num_rounds} WRF round(s) "
          f"of up to {WRF_DEPTH} each "
          + ("(weight-stationary)" if num_rounds == 1
             else f"(weight-reload every yout x {num_tiles} tile(s))"))

    # ------------------------------------------------------------------
    # 1. IFB data  (H_IN x W_IN pixels, packed to HW_PE channels x 8-bit)
    #    Only NUM_CIN channels carry valid data; channels NUM_CIN..HW_PE-1 = 0
    # ------------------------------------------------------------------
    ifb_hex_chars = HW_PE * 2  # hardware IFB word width in hex chars
    ifm_arr = [[[0]*NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
    ifb_data = []
    for y in range(H_IN):
        for x in range(W_IN):
            val = 0
            for cin in range(NUM_CIN):
                v = random.randint(0, 7)
                ifm_arr[y][x][cin] = v
                val |= (v & 0xFF) << (cin * 8)
            # channels NUM_CIN..HW_PE-1 are implicitly 0 in val
            ifb_data.append(f"{val:0{ifb_hex_chars}X}")
    with open('ifb.txt', 'w') as f:
        f.writelines(d + '\n' for d in ifb_data)

    # ------------------------------------------------------------------
    # 2. WB data  (K*K weight positions, stored per cout_slice)
    #    WB layout: cout_slice 0 occupies WB[0..K²-1], cout_slice 1 at WB[K²..2K²-1], etc.
    #    Each WB word = HW_COL x HW_PE x 8-bit (hardware width)
    #    Within each cout_slice, only the first `local_cout` columns carry valid weights.
    # ------------------------------------------------------------------
    wb_hex_chars = HW_COL * HW_PE * 2
    w_arr = [[[[0]*NUM_CIN for _ in range(NUM_COUT)] for _ in range(K)] for _ in range(K)]
    wb_data = []
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for ky in range(K):
            for kx in range(K):
                val = 0
                for lc in range(local_cout):
                    cout = cs * HW_COL + lc
                    cv = 0
                    for cin in range(NUM_CIN):
                        v = random.randint(-3, 3)
                        w_arr[ky][kx][cout][cin] = v
                        cv |= (v & 0xFF) << (cin * 8)
                    val |= cv << (lc * HW_PE * 8)
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

    # --- Cout slice loop (each slice has its own LD_WGT + yout loop) ---
    for cs in range(cout_slices):
        wb_base = cs * K * K  # WB 地址: cout_slice cs 的权重从 WB[cs*K²] 开始

        # --- LD_WGT for this cout_slice ---
        # Single-round: load all K² weights at once
        # Multi-round: LD_WGT inside loop body (handled below)
        if num_rounds == 1:
            instructions.append(
                pack_inst(OP_LD_WGT, wgt_addr=0, sram_addr=wb_base, length=K * K))

        # --- Scalar register initialisation ---
        instructions.append(pack_li(rd=0, imm=0))                          # r0 = IFB base
        instructions.append(pack_li(rd=1, imm=cs * H_OUT * W_OUT))         # r1 = OFB base for this cout_slice
        instructions.append(pack_li(rd=2, imm=H_OUT - 1))                  # r2 = yout counter

        # --- Loop body start ---
        LOOP_START = len(instructions)

        for x_tile_out in range(0, W_OUT, TILE_W):
            valid_w     = min(TILE_W, W_OUT - x_tile_out)
            x_tile_in   = x_tile_out * stride
            first_mac   = True

            for round_idx, round_pos in enumerate(rounds):
                round_global_start = wb_base + round_idx * WRF_DEPTH

                if num_rounds > 1:
                    instructions.append(
                        pack_inst(OP_LD_WGT, wgt_addr=0,
                                  sram_addr=round_global_start,
                                  length=len(round_pos)))

                for local_idx, (ky, kx) in enumerate(round_pos):
                    clr       = 1 if first_mac else 0
                    first_mac = False

                    if stride == 1:
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
                                rel = ky * W_IN + x_tile_in + TILE_W + kx - 1
                                instructions.append(pack_ldnmac(
                                    clr_parf=clr,
                                    wgt_addr=local_idx, parf_addr=0, arf_addr=kx,
                                    sram_addr=rel,
                                    ld_len=1, mac_len=TILE_W,
                                    stride=0, ld_arf_addr=kx - 1))
                        else:
                            rel = ky * W_IN + x_tile_in + kx
                            instructions.append(pack_ldnmac(
                                clr_parf=clr,
                                wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                                sram_addr=rel,
                                ld_len=valid_w, mac_len=valid_w,
                                stride=0, ld_arf_addr=0))
                    else:
                        if valid_w < TILE_W:
                            rel = ky * W_IN + x_tile_in + kx
                            instructions.append(pack_ldnmac(
                                clr_parf=clr,
                                wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                                sram_addr=rel,
                                ld_len=valid_w, mac_len=valid_w,
                                stride=stride, ld_arf_addr=0))
                        else:
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
        instructions.append(pack_alu(rd=0, rs1=0, op='+', imm=stride * W_IN))
        instructions.append(pack_alu(rd=1, rs1=1, op='+', imm=W_OUT))
        instructions.append(pack_bnz(rs=2, target_pc=LOOP_START))

    instructions.append(pack_inst(OP_FINISH))

    with open('inst.txt', 'w') as f:
        f.writelines(inst + '\n' for inst in instructions)

    # --- sim_params.f for testbench plusargs ---
    total_ofm = H_OUT * W_OUT * cout_slices
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n+COUT_SLICES={cout_slices}\n"
                f"-gSRAM_DEPTH={SRAM_DEPTH}\n")

    # ------------------------------------------------------------------
    # 4. Expected OFM (golden reference)
    #    Layout: cout_slice 0 region [0..H*W-1], cout_slice 1 [H*W..2*H*W-1], ...
    #    Each entry = HW_COL × 8-bit (only local_cout channels valid, rest 0)
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
    # Summary
    # ------------------------------------------------------------------
    n = len(instructions)
    print(f"Instructions : {n} total  ({cout_slices} cout_slice(s), "
          f"runs {H_OUT} yout per slice)")
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
