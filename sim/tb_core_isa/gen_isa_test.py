"""
gen_isa_test.py -- CNN Accelerator ISA Test Generator

Usage:
  python gen_isa_test.py [--h_in H] [--w_in W] [--k K]
                         [--num_cin C] [--num_cout N] [--tile_w T] [--seed S]

  H_OUT = H_IN - K + 1  (padding=0, stride=1)
  W_OUT = W_IN - K + 1

Weight scheduling:
  WRF_DEPTH = 32 (hardware fixed, 5-bit wgt_rf_addr).
  K*K positions are split into rounds of WRF_DEPTH each.
  - K*K <= 32 (e.g. 1x1, 3x3, 5x5): single round, LD_WGT runs ONCE before the
    yout loop (true weight-stationary).
  - K*K >  32 (e.g. 6x6, 7x7):      multiple rounds, LD_WGT runs inside the
    loop body at each round boundary (weight-reload / weight-streaming).
  ARF state carries over between rounds: the sliding-window ARF is not reset
  by LD_WGT, so mid-row round transitions work correctly.
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
OP_LD1MAC  = 6   # Load 1 pixel -> ARF[ld_arf_addr] AND run MAC simultaneously
OP_LD32MAC = 7   # Load TILE_W pixels -> ARF AND run MAC simultaneously (length+1 cycles)
OP_FINISH  = 15
OP_LI      = 8   # Load Immediate:     scalar_rf[rd] <- imm16
OP_ALU     = 9   # Scalar ALU:         rd <- rs1 +/- rs2  or  rd <- rs1 +/- imm16
OP_JMP     = 10  # Unconditional Jump: PC <- target_pc
OP_BNZ     = 11  # Branch if Not Zero: if rs!=0: rs--, PC<-target_pc

WRF_DEPTH = 32   # Hardware fixed: 5-bit wgt_rf_addr field (max address = 31)

# ---------------------------------------------------------------------------
# Instruction packing helpers
# ---------------------------------------------------------------------------
def pack_inst(opcode, clr_parf=0, sdp_en=0, arf_addr=0, wgt_addr=0,
              parf_addr=0, sram_addr=0, length=0, ld_arf_addr=0):
    inst  = (opcode      & 0xF)    << 60
    inst |= (clr_parf    & 0x1)    << 59
    inst |= (sdp_en      & 0x1)    << 58
    inst |= (arf_addr    & 0x1F)   << 53
    inst |= (wgt_addr    & 0x1F)   << 48
    inst |= (parf_addr   & 0x1F)   << 43
    inst |= (sram_addr   & 0x7FFF) << 28
    inst |= (length      & 0xFFFF) << 12
    inst |= (ld_arf_addr & 0x1F)
    return f"{inst:016X}"

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

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description='Generate ISA test files for CNN accelerator.')
    p.add_argument('--h_in',     type=int, default=123, help='Input height  (default 123)')
    p.add_argument('--w_in',     type=int, default=45,  help='Input width   (default 45)')
    p.add_argument('--k',        type=int, default=3,   help='Kernel size KxK (default 3)')
    p.add_argument('--num_cin',  type=int, default=8,   help='Input channels  (default 8)')
    p.add_argument('--num_cout', type=int, default=8,   help='Output channels (default 8)')
    p.add_argument('--tile_w',   type=int, default=32,  help='ARF tile width  (default 32)')
    p.add_argument('--seed',     type=int, default=42,  help='Random seed     (default 42)')
    return p.parse_args()

# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------
def generate(H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed):
    import random
    random.seed(seed)

    H_OUT = H_IN - K + 1
    W_OUT = W_IN - K + 1

    if H_OUT <= 0 or W_OUT <= 0:
        sys.exit(f"ERROR: K={K}x{K} too large for {H_IN}x{W_IN} "
                 f"(output {H_OUT}x{W_OUT} is invalid).")

    # ------------------------------------------------------------------
    # Split K*K kernel positions into WRF_DEPTH-sized rounds
    # ------------------------------------------------------------------
    all_positions = [(ky, kx) for ky in range(K) for kx in range(K)]
    rounds = [all_positions[i:i+WRF_DEPTH]
              for i in range(0, len(all_positions), WRF_DEPTH)]
    num_rounds = len(rounds)
    num_tiles  = (W_OUT + TILE_W - 1) // TILE_W

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, "
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
    ifm_arr = [[[0]*NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
    ifb_data = []
    for y in range(H_IN):
        for x in range(W_IN):
            val = 0
            for cin in range(NUM_CIN):
                v = random.randint(0, 7)
                ifm_arr[y][x][cin] = v
                val |= (v & 0xFF) << (cin * 8)
            ifb_data.append(f"{val:016X}")
    with open('ifb.txt', 'w') as f:
        f.writelines(d + '\n' for d in ifb_data)

    # ------------------------------------------------------------------
    # 2. WB data  (K*K weight positions stored flat)
    # ------------------------------------------------------------------
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
                val |= cv << (cout * 64)
            wb_data.append(f"{val:0128X}")
    with open('wb.txt', 'w') as f:
        f.writelines(d + '\n' for d in wb_data)

    # ------------------------------------------------------------------
    # 3. Instructions (loop-compressed, multi-round weight scheduling)
    # ------------------------------------------------------------------
    # Register convention:
    #   r0 = IFB base addr offset  (HW adds to every ifb_raddr)
    #   r1 = OFB base addr offset  (HW adds to every ofb_waddr)
    #   r2 = yout loop counter
    # sram_addr in MAC/ST instructions is RELATIVE to base register:
    #   actual_ifb = r0 + inst.sram_addr [+ cnt]
    #   actual_ofb = r1 + inst.sram_addr [+ cnt]
    instructions = []

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

    for x_tile in range(0, W_OUT, TILE_W):              # unrolled (few tiles)
        valid_w   = min(TILE_W, W_OUT - x_tile)
        first_mac = True   # clr_parf=1 only for the first MAC of each x_tile

        for round_idx, round_pos in enumerate(rounds):
            round_global_start = round_idx * WRF_DEPTH

            # Multi-round: reload WRF from WB at each round boundary.
            # LD_WGT uses absolute WB address (not affected by r0/r1).
            if num_rounds > 1:
                instructions.append(
                    pack_inst(OP_LD_WGT, wgt_addr=0,
                              sram_addr=round_global_start,
                              length=len(round_pos)))

            for local_idx, (ky, kx) in enumerate(round_pos):
                clr       = 1 if first_mac else 0
                first_mac = False

                if kx == 0:
                    # LD32MAC: load TILE_W pixels from this IFB row into ARF,
                    # MAC runs in parallel starting from cnt=1.
                    # relative sram_addr = ky*W_IN + x_tile  (added to r0 by HW)
                    rel = ky * W_IN + x_tile
                    instructions.append(pack_inst(
                        OP_LD32MAC, clr_parf=clr,
                        wgt_addr=local_idx, parf_addr=0, arf_addr=0,
                        sram_addr=rel, length=TILE_W))
                else:
                    # LD1MAC: load 1 new pixel into ARF[kx-1], MAC reads
                    # ARF[kx .. kx+TILE_W-1] (sliding window, no hazard).
                    # relative sram_addr = ky*W_IN + x_tile + TILE_W + kx - 1
                    rel = ky * W_IN + x_tile + TILE_W + kx - 1
                    instructions.append(pack_inst(
                        OP_LD1MAC, clr_parf=clr,
                        wgt_addr=local_idx, parf_addr=0, arf_addr=kx,
                        sram_addr=rel, length=TILE_W,
                        ld_arf_addr=kx - 1))

        # ST_OFM: write valid_w pixels to OFB (addr relative to r1)
        instructions.append(pack_inst(
            OP_ST_OFM, sdp_en=1, parf_addr=0,
            sram_addr=x_tile, length=valid_w))

    # --- Loop tail ---
    instructions.append(pack_alu(rd=0, rs1=0, op='+', imm=W_IN))   # r0 += W_IN
    instructions.append(pack_alu(rd=1, rs1=1, op='+', imm=W_OUT))  # r1 += W_OUT
    instructions.append(pack_bnz(rs=2, target_pc=LOOP_START))

    instructions.append(pack_inst(OP_FINISH))

    with open('inst.txt', 'w') as f:
        f.writelines(inst + '\n' for inst in instructions)

    # --- sim_params.f for testbench plusargs ---
    with open('sim_params.f', 'w') as f:
        f.write(f"+H_OUT={H_OUT}\n+W_OUT={W_OUT}\n")

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
                            psum += ifm_arr[yout+ky][px+kx][cin] * w_arr[ky][kx][cout][cin]
                act = max(0, min(255, psum))   # ReLU + 8-bit saturation
                pixel_val |= (act & 0xFF) << (cout * 8)
            expected_ofm.append(f"{pixel_val:016X}")
    with open('expected_ofm.txt', 'w') as f:
        f.writelines(d + '\n' for d in expected_ofm)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    n = len(instructions)
    body = n - (1 if num_rounds == 1 else 0) - 3 - 3 - 1  # rough body count
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
             TILE_W=args.tile_w, seed=args.seed)
