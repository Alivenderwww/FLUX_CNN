#!/usr/bin/env python3
"""Stage 2: 最小 conv layer 触发完整数据流 (1x1x1x1 conv, H_OUT=1 W_OUT=1).

目标: 验证 IDMA → mac (bypass) → parf → ofb_writer → ODMA 整条数据流, ODMA 真写 BRAM.
不验 OFM bit-exact (mac garbage 输出).

BRAM layout (BRAM_BASE=0xA4100000, 64KB):
  0x0000  desc list  (~50 CFG + CONV + END, ~1.6KB)
  0x0800  IDMA SG cmd[0]  (32 byte)
  0x0820  ODMA SG cmd[0]  (32 byte)
  0x1000  IFM data        (16 byte = 1 word)
  0x2000  Weight data     (256 byte = 1 word)
  0x2100  RDMA bias data  (64 byte = 4 word)
  0x3000  OFM area        (16 byte, host pre-clear, 期望 ODMA 写入)
"""
import sys, struct, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
from vd100_rpc import Vd100Rpc

BRAM_BASE = 0xA4100000
DESC_OFF  = 0x0000
ISG_OFF   = 0x0800
OSG_OFF   = 0x0820
IFM_OFF   = 0x1000
WB_OFF    = 0x2000
RDMA_OFF  = 0x2100
OFM_OFF   = 0x3000

BRAM_IFM  = BRAM_BASE + IFM_OFF
BRAM_WB   = BRAM_BASE + WB_OFF
BRAM_RDMA = BRAM_BASE + RDMA_OFF
BRAM_OFM  = BRAM_BASE + OFM_OFF
BRAM_ISG  = BRAM_BASE + ISG_OFF
BRAM_OSG  = BRAM_BASE + OSG_OFF

def make_cfg(addr, data):
    w0 = 0x3 | ((addr & 0xFFF) << 4)
    return struct.pack('<IIII', w0, data, 0, 0) + b'\x00' * 16

def make_conv(is_first=1, is_last=1, pad_top=0, pad_bot=0, pad_left=0, pad_right=0,
              strip_y_start=0, n_yout_strip=1, ifb_ddr_off=0, ifb_byte_len=16,
              ofb_ddr_off=0, ofb_byte_len=16):
    """CONV desc 32 byte:
       word0: type[3:0]=1, is_first[4], is_last[5], FLAG_STREAMING[6]=1, pad_top[11:8], pad_bot[15:12], pad_left[19:16], pad_right[23:20]
       word1: strip_y_start[47:32]=word1[15:0], n_yout_strip[63:48]=word1[31:16]
       word2: ifb_ddr_offset[19:0], ifb_byte_len[..23:96]
       word3: ofb_ddr_offset[19:0], ofb_byte_len[..23:160]"""
    w0 = 0x1 | (is_first << 4) | (is_last << 5) | (1 << 6)  # FLAG_STREAMING_EN
    w0 |= ((pad_top & 0xF) << 8) | ((pad_bot & 0xF) << 12) | ((pad_left & 0xF) << 16) | ((pad_right & 0xF) << 20)
    w1 = (strip_y_start & 0xFFFF) | ((n_yout_strip & 0xFFFF) << 16)
    w2 = ifb_ddr_off & 0xFFFFF
    w3 = (ifb_byte_len & 0xFFFFFF) << 0   # 注意 ifb_byte_len 在 word3 [23:0]? 看 sequencer.sv 解析
    # 实际看 sequencer.sv:
    #   hd_ifb_ddr_offset = fifo_rd_data[83:64]  // word 2 [19:0]
    #   hd_ifb_byte_len   = fifo_rd_data[119:96] // word 3 [23:0]
    #   hd_ofb_ddr_offset = fifo_rd_data[147:128]// word 4 [19:0]
    #   hd_ofb_byte_len   = fifo_rd_data[183:160]// word 5 [23:0]
    # → desc 共 8 word (32 byte): word0/1/2/3 一共 16 byte, word4/5/6/7 一共 16 byte
    w4 = ofb_ddr_off & 0xFFFFF
    w5 = ofb_byte_len & 0xFFFFFF
    w_ifb_off = ifb_ddr_off & 0xFFFFF
    w_ifb_len = ifb_byte_len & 0xFFFFFF
    # word 0..3 + word 4..7 (各 32 byte 总), pack 8 个 word
    return struct.pack('<IIIIIIII', w0, w1, w_ifb_off, w_ifb_len, w4, w5, 0, 0)

def make_end():
    return struct.pack('<IIII', 0xF, 0, 0, 0) + b'\x00' * 16

def make_sg_idma(src_addr, btt, sram_off=0, last_cmd=1):
    """IDMA SG cmd 32 byte:
       word0: src_addr (32 bit)
       word1: btt[22:0] + last_cmd[23]
       word2: sram_offset[12:0]
       word3: reserved"""
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', src_addr, w1, sram_off & 0x1FFF, 0) + b'\x00' * 16

def make_sg_odma(dst_addr, btt, ofb_w_start=0, last_cmd=1):
    """ODMA SG cmd 32 byte:
       word0: dst_addr
       word1: btt[22:0] + last_cmd[23]
       word2: ofb_w_start[15:0]
       word3: reserved"""
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', dst_addr, w1, ofb_w_start & 0xFFFF, 0) + b'\x00' * 16

def main():
    rpc = Vd100Rpc('169.254.111.10', 5000); rpc.connect()
    print('=== Stage 2: 最小 conv layer (1x1x1x1, mac bypass) ===')

    # 1. 构造 desc list: 各种 CFG_WRITE + CONV + END
    descs = []
    # Layer cfg (所有 cfg_regs 字段, 最小有效值)
    cfg_pairs = [
        (0x100, 1),     # H_OUT
        (0x104, 1),     # W_OUT
        (0x108, 1),     # W_IN
        (0x10C, 1),     # K
        (0x12C, 1),     # KY
        (0x110, 1),     # STRIDE
        (0x1F0, 1),     # STRIDE_H  (verify FLUX_ADDR_STRIDE_H, fallback if unmapped)
        (0x114, 1),     # CIN_SLICES
        (0x118, 1),     # COUT_SLICES
        (0x11C, 1),     # TILE_W
        (0x128, 1),     # TOTAL_WRF
        (0x130, 1),     # KK
        (0x134, 1),     # ROUNDS_PER_CINS
        (0x138, 1),     # ROUND_LEN_LAST
        (0x13C, 0),     # IFB_BASE
        (0x140, 0),     # WB_BASE
        (0x144, 0),     # OFB_BASE
        (0x14C, 1),     # IFB_ROW_STEP
        (0x154, 1),     # WB_COUT_STEP
        (0x15C, 1),     # TILE_IN_STEP
        (0x120, 1),     # NUM_TILES
        (0x124, 1),     # LAST_VALID_W
        (0x160, 0),     # SDP_SHIFT
        (0x164, 0),     # SDP_RELU_EN
        (0x168, 1),     # H_IN_TOTAL
        (0x16C, 1),     # IFB_STRIP_ROWS
        (0x170, 1),     # OFB_STRIP_ROWS
        (0x174, 16),    # DDR_IFM_ROW_STRIDE
        (0x178, 16),    # DDR_OFM_ROW_STRIDE
        (0x1A0, 1),     # IFB_RING_WORDS
        (0x1A4, 1),     # OFB_ROW_WORDS
        (0x1A8, 1),     # OFB_RING_WORDS
        (0x1AC, 1),     # IFB_ISS_STEP
        (0x1B0, 1),     # IFB_KY_STEP
        (0x1B4, 1),     # TILE_PIX_STEP
        (0x1B8, 0),     # ARF_REUSE_EN
        (0x1BC, 0),     # RESIDUAL_EN
        (0x1C8, 0),     # BIAS_BASE
        (0x188, 1),     # SDP_MULT
        (0x18C, 0),     # SDP_ZP_OUT
        (0x190, 0xFFFFFF80),  # SDP_CLIP_MIN (-128 signed)
        (0x194, 0x7F),  # SDP_CLIP_MAX (127)
        (0x198, 0),     # SDP_ROUND_EN
        # DMA bases + len
        (0x210, BRAM_WB),       # WDMA_SRC_BASE
        (0x214, 256),           # WDMA_BYTE_LEN (1 WB word = 256 byte)
        (0x230, BRAM_RDMA),     # RDMA_SRC_BASE
        (0x234, 64),            # RDMA_BYTE_LEN (4 RDMA word = 64 byte)
        # SG cmd lists
        (0x1D8, BRAM_ISG),      # IDMA_CMD_LIST_PTR
        (0x1DC, 1),             # IDMA_CMD_COUNT
        (0x1E0, 1),             # IDMA_CMDS_PER_ROW
        (0x1E4, BRAM_OSG),      # ODMA_CMD_LIST_PTR
        (0x1E8, 1),             # ODMA_CMD_COUNT
        (0x1EC, 1),             # ODMA_CMDS_PER_ROW
        # SKIP_IDMA off (single core, 不跳 IDMA)
        (0x1CC, 0),             # SKIP_IDMA
    ]
    for addr, val in cfg_pairs:
        descs.append(make_cfg(addr, val))
    # CONV (is_first=1, is_last=1, 1 row OFM, 16 byte IFM+OFM)
    descs.append(make_conv(is_first=1, is_last=1, n_yout_strip=1,
                           ifb_ddr_off=0, ifb_byte_len=16,
                           ofb_ddr_off=0, ofb_byte_len=16))
    descs.append(make_end())
    desc_list = b''.join(descs)
    n_desc = len(descs)
    print(f'[1] desc list: {n_desc} desc ({len(cfg_pairs)} CFG + CONV + END), {len(desc_list)} byte')

    # 2. 构造 SG cmd lists
    idma_sg = make_sg_idma(BRAM_IFM, btt=16, sram_off=0, last_cmd=1)
    odma_sg = make_sg_odma(BRAM_OFM, btt=16, ofb_w_start=0, last_cmd=1)
    print(f'[2] IDMA SG: src=0x{BRAM_IFM:08x} btt=16 last=1')
    print(f'    ODMA SG: dst=0x{BRAM_OFM:08x} btt=16 last=1')

    # 3. 数据 (任意值, mac bypass garbage)
    ifm_data = bytes([0x11] * 16)
    wb_data  = bytes([0x22] * 256)
    rdma_data = bytes([0x33] * 64)

    # 4. 清 OFM 区, 写 0x00 (host pre-clear, 看 ODMA 是否写)
    print('[3] write all data to BRAM (chunked)')
    def chunked_load(rpc, addr, data, chunk=1024):
        for off in range(0, len(data), chunk):
            rpc.load_ddr(addr + off, data[off:off+chunk])
    rpc.load_ddr(BRAM_OFM, b'\x00' * 16)
    chunked_load(rpc, BRAM_BASE + DESC_OFF, desc_list)
    rpc.load_ddr(BRAM_ISG, idma_sg)
    rpc.load_ddr(BRAM_OSG, odma_sg)
    rpc.load_ddr(BRAM_IFM, ifm_data)
    rpc.load_ddr(BRAM_WB, wb_data)
    rpc.load_ddr(BRAM_RDMA, rdma_data)

    # 5. 配 dfe
    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)
    print(f'[4] DESC_LIST_BASE=0x{rpc.peek_csr(0, 0x180):08x} DESC_COUNT={rpc.peek_csr(0, 0x184)}')

    # OFM pre-state
    ofm_before = rpc.read_ddr(BRAM_OFM, 16)
    print(f'[5] OFM before: {ofm_before.hex()}')

    # 6. start_dfe + start_layer
    rpc.poke_csr(0, 0x000, 1 << 4); time.sleep(0.3)
    st = rpc.peek_csr(0, 0x004); sd = rpc.peek_csr(0, 0x008)
    print(f'[6a] start_dfe: STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x} dfe_done={(st>>9)&1}')

    rpc.poke_csr(0, 0x000, 1 << 5)
    print('[6b] start_layer + poll 5s')
    for t in range(50):
        time.sleep(0.1)
        st = rpc.peek_csr(0, 0x004)
        if st & (1 << 11):
            print(f'  layer_done @ t={t*0.1:.1f}s STATUS=0x{st:08x}')
            break

    # final state
    st = rpc.peek_csr(0, 0x004); sd = rpc.peek_csr(0, 0x008)
    seq = sd & 0xF; fifo = (sd >> 4) & 0xF
    odma_st = (sd >> 8) & 0xF; idma_st = (sd >> 12) & 0xF
    print(f'[7] final: STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x} seq={seq} fifo={fifo} oSG={odma_st} iSG={idma_st}')
    print(f'   layer_busy={(st>>10)&1} layer_done={(st>>11)&1}')
    print(f'   dfe_done={(st>>9)&1} odma_done={(st>>7)&1} wdma_done={(st>>6)&1} idma_done={(st>>5)&1}')
    print(f'   idma_busy={(st>>2)&1} wdma_busy={(st>>3)&1} odma_busy={(st>>4)&1}')

    # 8. OFM 区读回, 看 ODMA 是否写
    ofm_after = rpc.read_ddr(BRAM_OFM, 16)
    print(f'[8] OFM after:  {ofm_after.hex()}')
    written = (ofm_after != ofm_before)
    if written:
        print(f'    [OK] ODMA 真写了 BRAM (数据 garbage 但有写)')
    else:
        print(f'    [NO] ODMA 没写 BRAM (整条数据流没跑通)')

if __name__ == '__main__':
    main()
