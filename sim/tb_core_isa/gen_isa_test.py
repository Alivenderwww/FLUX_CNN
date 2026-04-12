import os
import random

OP_NOP = 0
OP_LD_WGT = 1
OP_LD_ARF = 2
OP_LD_PARF = 3
OP_ST_OFM = 4
OP_MAC_RUN = 5
OP_FINISH = 15

def pack_inst(opcode, shift_arf=0, clr_parf=0, sdp_en=0, wgt_rf_addr=0, parf_addr=0, sram_addr=0, length=0, stride=0):
    inst = 0
    inst |= (opcode & 0xF) << 60
    inst |= (shift_arf & 0x1) << 59
    inst |= (clr_parf & 0x1) << 58
    inst |= (sdp_en & 0x1) << 57
    inst |= (wgt_rf_addr & 0x1F) << 52
    inst |= (parf_addr & 0x1F) << 47
    inst |= (sram_addr & 0x7FFF) << 32
    inst |= (length & 0xFFFF) << 16
    inst |= (stride & 0xFFFF)
    return f"{inst:016X}"

# 1. Generate Instructions
instructions = []

# Load Weights (9 items for 3x3 kernel, from wb sram addr 0 to wgt_rf_addr 0)
instructions.append(pack_inst(OP_LD_WGT, wgt_rf_addr=0, sram_addr=0, length=9))

# For ky=0..2
for ky in range(3):
    # Load 32 pixels for row ky to ARF
    # IFB address: ky * 34 (34 pixels total in one padded row)
    instructions.append(pack_inst(OP_LD_ARF, sram_addr=ky*34, length=32))
    
    # kx = 0
    # Clear PARF only on very first calculation (ky=0, kx=0)
    clr = 1 if ky == 0 else 0
    instructions.append(pack_inst(OP_MAC_RUN, shift_arf=0, clr_parf=clr, wgt_rf_addr=ky*3+0, parf_addr=0, length=32))
    
    # kx = 1
    # Shift ARF, load next pixel from IFB (ky*34 + 32)
    instructions.append(pack_inst(OP_MAC_RUN, shift_arf=1, clr_parf=0, wgt_rf_addr=ky*3+1, parf_addr=0, sram_addr=ky*34+32, length=32))
    
    # kx = 2
    # Shift ARF, load next pixel from IFB (ky*34 + 33)
    instructions.append(pack_inst(OP_MAC_RUN, shift_arf=1, clr_parf=0, wgt_rf_addr=ky*3+2, parf_addr=0, sram_addr=ky*34+33, length=32))

# Finish Execution
instructions.append(pack_inst(OP_FINISH))

with open('inst.txt', 'w') as f:
    for inst in instructions:
        f.write(f"{inst}\n")

# Random seed for reproducibility (optional but good)
random.seed(42)

# 2. Generate IFB Data (3 rows * 34 pixels = 102 lines)
# Matrix format: IFM[ky][px][cin]
ifm_arr = [[[0 for _ in range(8)] for _ in range(34)] for _ in range(3)]
ifb_data = []

for ky in range(3):
    for px in range(34):
        val = 0
        for cin in range(8):
            v = random.randint(-128, 127)
            ifm_arr[ky][px][cin] = v
            # To format signed 8-bit correctly in hex:
            val |= (v & 0xFF) << (cin * 8)
        ifb_data.append(f"{val:016X}")

with open('ifb.txt', 'w') as f:
    for data in ifb_data:
        f.write(f"{data}\n")

# 3. Generate WB Data (9 lines for 3x3 kernel)
# Matrix format: W[ky][kx][cout][cin]
w_arr = [[[[0 for _ in range(8)] for _ in range(8)] for _ in range(3)] for _ in range(3)]
wb_data = []

for ky in range(3):
    for kx in range(3):
        val = 0
        for cout in range(8):
            cout_val = 0
            for cin in range(8):
                v = random.randint(-128, 127)
                w_arr[ky][kx][cout][cin] = v
                cout_val |= (v & 0xFF) << (cin * 8)
            val |= (cout_val << (cout * 64))
        wb_data.append(f"{val:0128X}")

with open('wb.txt', 'w') as f:
    for data in wb_data:
        f.write(f"{data}\n")

# 4. Compute Expected Psum (32 pixels, 8 cout channels)
# Output should match the way hardware iterates over it.
with open('expected_psum.txt', 'w') as f:
    for px in range(32):
        for cout in range(8):
            psum = 0
            for ky in range(3):
                for kx in range(3):
                    for cin in range(8):
                        psum += ifm_arr[ky][px+kx][cin] * w_arr[ky][kx][cout][cin]
            # write as hex (two's complement 32-bit)
            f.write(f"{(psum & 0xFFFFFFFF):08X}\n")

print("Generated Assembly Code and Test Data Successfully!")