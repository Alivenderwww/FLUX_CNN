import os
import random

OP_NOP = 0
OP_LD_WGT = 1
OP_LD_ARF = 2
OP_LD_PARF = 3
OP_ST_OFM = 4
OP_MAC_RUN = 5
OP_LD1MAC = 6   # Load 1 pixel to ARF[ld_arf_addr] AND run MAC simultaneously
OP_LD32MAC = 7  # Load N pixels from IFB into ARF AND run MAC simultaneously (length+1 cycles)
OP_FINISH = 15

def pack_inst(opcode, clr_parf=0, sdp_en=0, arf_addr=0, wgt_addr=0, parf_addr=0, sram_addr=0, length=0, ld_arf_addr=0):
    inst = 0
    inst |= (opcode    & 0xF)     << 60
    inst |= (clr_parf  & 0x1)     << 59
    inst |= (sdp_en    & 0x1)     << 58
    inst |= (arf_addr  & 0x1F)    << 53
    inst |= (wgt_addr  & 0x1F)    << 48
    inst |= (parf_addr & 0x1F)    << 43
    inst |= (sram_addr & 0x7FFF)  << 28
    inst |= (length    & 0xFFFF)  << 12
    inst |= (ld_arf_addr & 0x1F)        # stride[4:0]：OP_LD1MAC 的 ARF 写入地址
    return f"{inst:016X}"

# 1. Configuration
H_IN = 123
W_IN = 45
H_OUT = 121
W_OUT = 43
NUM_CIN = 8
NUM_COUT = 8
TILE_W = 32

# Random seed
random.seed(42)

# 2. Generate IFB Data (H_IN rows * W_IN pixels)
ifm_arr = [[[0 for _ in range(NUM_CIN)] for _ in range(W_IN)] for _ in range(H_IN)]
ifb_data = []

for y in range(H_IN):
    for x in range(W_IN):
        val = 0
        for cin in range(NUM_CIN):
            v = random.randint(0, 7) # Limit IFM to [0, 7]
            ifm_arr[y][x][cin] = v
            val |= (v & 0xFF) << (cin * 8)
        ifb_data.append(f"{val:016X}")

with open('ifb.txt', 'w') as f:
    for data in ifb_data:
        f.write(f"{data}\n")

# 3. Generate WB Data (9 lines for 3x3 kernel)
w_arr = [[[[0 for _ in range(NUM_CIN)] for _ in range(NUM_COUT)] for _ in range(3)] for _ in range(3)]
wb_data = []

for ky in range(3):
    for kx in range(3):
        val = 0
        for cout in range(NUM_COUT):
            cout_val = 0
            for cin in range(NUM_CIN):
                v = random.randint(-3, 3) # Limit W to [-3, 3]
                w_arr[ky][kx][cout][cin] = v
                cout_val |= (v & 0xFF) << (cin * 8)
            val |= (cout_val << (cout * 64))
        wb_data.append(f"{val:0128X}")

with open('wb.txt', 'w') as f:
    for data in wb_data:
        f.write(f"{data}\n")

# 4. Generate Instructions
instructions = []

# Load Weights (9 items for 3x3 kernel)
instructions.append(pack_inst(OP_LD_WGT, wgt_addr=0, sram_addr=0, length=9))

for yout in range(H_OUT):
    # Iterate over the width in chunks of TILE_W (32)
    for x_tile in range(0, W_OUT, TILE_W):
        valid_w = min(TILE_W, W_OUT - x_tile)
        
        for ky in range(3):
            yin = yout + ky
            xin = x_tile

            for kx in range(3):
                clr = 1 if (ky == 0 and kx == 0) else 0
                if kx == 0:
                    # kx=0: Load 32 pixels from IFB into ARF AND run MAC simultaneously.
                    # State runs length+1 = 33 cycles: cnt=0 prefetch, cnt=1..32 overlap.
                    instructions.append(pack_inst(OP_LD32MAC, clr_parf=clr, wgt_addr=ky*3+kx,
                                                  parf_addr=0, arf_addr=0,
                                                  sram_addr=yin*W_IN + xin, length=TILE_W))
                else:
                    # kx>0: load 1 new pixel to ARF[kx-1] WHILE running MAC on ARF[kx..kx+31]
                    # ARF write addr (kx-1) != MAC read range (kx..kx+31) → no hazard
                    new_pixel_sram = yin*W_IN + xin + 32 + kx - 1
                    instructions.append(pack_inst(OP_LD1MAC, clr_parf=clr, wgt_addr=ky*3+kx,
                                                  parf_addr=0, arf_addr=kx,
                                                  sram_addr=new_pixel_sram, length=TILE_W,
                                                  ld_arf_addr=kx-1))
        
        # Store OFM for this tile, ONLY storing the valid_w pixels to avoid overwriting next tile!
        instructions.append(pack_inst(OP_ST_OFM, sdp_en=1, parf_addr=0, sram_addr=yout*W_OUT + x_tile, length=valid_w))

# Finish Execution
instructions.append(pack_inst(OP_FINISH))

with open('inst.txt', 'w') as f:
    for inst in instructions:
        f.write(f"{inst}\n")

# 5. Compute Expected OFM
expected_ofm = []

for yout in range(H_OUT):
    for px in range(W_OUT):
        pixel_val = 0
        for cout in range(NUM_COUT):
            psum = 0
            for ky in range(3):
                for kx in range(3):
                    for cin in range(NUM_CIN):
                        psum += ifm_arr[yout+ky][px+kx][cin] * w_arr[ky][kx][cout][cin]
            
            # SDP: ReLU + Truncation
            if psum < 0:
                act = 0
            elif psum > 255:
                act = 255
            else:
                act = psum & 0xFF
                
            pixel_val |= (act << (cout * 8))
        expected_ofm.append(f"{pixel_val:016X}")

with open('expected_ofm.txt', 'w') as f:
    for data in expected_ofm:
        f.write(f"{data}\n")

print(f"Generated Assembly Code ({len(instructions)} ops) and Test Data (123x45) Successfully!")