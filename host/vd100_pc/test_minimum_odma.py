#!/usr/bin/env python3
"""
Minimum bring-up test: 4 个 DMA 各 1 cmd × 16 byte (axi_dm + axi_noc 基础路径).

不重综合 — 用现有 v32 PDI. PC 写 minimum desc list + 各 DMA 数据, trigger layer.

如果 layer_done=1 + DDR 写入正确 → axi_dm + axi_noc 基础 work, 多 cmd 触发 vendor 边界
如果 stuck → axi_dm + axi_noc 单 cmd 都卡, hardware bug 100% 锁定
"""

import os, sys, struct, time
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

ADDR_CTRL               = 0x000   # bit[4]=start_dfe, bit[5]=start_layer
ADDR_STATUS             = 0x004
ADDR_SEQ_DBG            = 0x008
ADDR_IDMA_CMD_LIST_PTR  = 0x1D8
ADDR_IDMA_CMD_COUNT     = 0x1DC
ADDR_IDMA_CMDS_PER_ROW  = 0x1E0
ADDR_ODMA_CMD_LIST_PTR  = 0x1E4
ADDR_ODMA_CMD_COUNT     = 0x1E8
ADDR_ODMA_CMDS_PER_ROW  = 0x1EC
ADDR_IDMA_BYTE_LEN      = 0x204
ADDR_WDMA_BYTE_LEN      = 0x214
ADDR_ODMA_BYTE_LEN      = 0x224
ADDR_RDMA_BYTE_LEN      = 0x234
ADDR_WDMA_SRC_BASE      = 0x210
ADDR_RDMA_SRC_BASE      = 0x230
ADDR_DESC_LIST_BASE     = 0x180
ADDR_DESC_COUNT         = 0x184
# desc type
TYPE_CONV = 0x1
TYPE_CFG  = 0x3
TYPE_END  = 0xF

def make_cfg_desc(cfg_addr, cfg_data):
    """32 byte desc: word0 = {16'd0, cfg_addr[11:4], TYPE_CFG} actually [15:4]=addr, [3:0]=type, word1=cfg_data, word2=word3=0"""
    word0 = (TYPE_CFG & 0xF) | ((cfg_addr & 0xFFF) << 4)
    return struct.pack('<IIII', word0, cfg_data, 0, 0) + b'\x00' * 16

def make_conv_desc():
    word0 = TYPE_CONV & 0xF
    return struct.pack('<IIII', word0, 0, 0, 0) + b'\x00' * 16

def make_end_desc():
    word0 = TYPE_END & 0xF
    return struct.pack('<IIII', word0, 0, 0, 0) + b'\x00' * 16

# DDR addresses (各 DMA 数据 + cmd list)
DESC_BASE = 0x10A00000   # dfe desc list
IDMA_CMD_BASE = 0x10B00000   # IDMA SG cmd list
ODMA_CMD_BASE = 0x10C00000   # ODMA SG cmd list
ODMA_DST_BASE = 0x10D00000   # ODMA OFM 目标
IDMA_SRC_BASE = 0x10E00000   # IDMA IFM 源 (16 byte fake data)
WDMA_SRC_BASE = 0x10F00000   # WDMA weight 源
RDMA_SRC_BASE = 0x11000000   # RDMA bias 源

def make_sg_cmd(addr, btt, sram_offset=0, last_cmd=True):
    """32 byte SG cmd: word0=addr, word1=btt|last, word2=sram_offset, word3=rsvd, +16 zero"""
    word1 = (btt & 0x7FFFFF) | ((1 << 23) if last_cmd else 0)
    return struct.pack('<IIII', addr, word1, sram_offset, 0) + b'\x00' * 16

def main():
    print("=== Minimum bring-up: 4 DMA × 1 cmd × 16 byte ===")
    rpc = Vd100Rpc('169.254.111.10', 5000)
    rpc.connect()

    print("[1/8] Clear OFM target 0x10D00000 (verify ODMA wrote)")
    rpc.load_ddr(ODMA_DST_BASE, b'\x00' * 256)

    print("[2/8] Preload data (16 byte each) to IDMA/WDMA/RDMA src")
    rpc.load_ddr(IDMA_SRC_BASE, bytes(range(16)))  # 0x00..0x0F
    rpc.load_ddr(WDMA_SRC_BASE, b'\xAA' * 16)
    rpc.load_ddr(RDMA_SRC_BASE, b'\xBB' * 16)

    print("[3/8] Preload IDMA SG cmd[0] + ODMA SG cmd[0]")
    idma_sg = make_sg_cmd(IDMA_SRC_BASE, btt=16, sram_offset=0)
    odma_sg = make_sg_cmd(ODMA_DST_BASE, btt=16, sram_offset=0)
    rpc.load_ddr(IDMA_CMD_BASE, idma_sg)
    rpc.load_ddr(ODMA_CMD_BASE, odma_sg)

    print("[4a/8] Build minimum desc list (CFG_WRITE × N + CONV + END)")
    desc_list = b''
    cfg_pairs = [
        (ADDR_IDMA_CMD_LIST_PTR, IDMA_CMD_BASE),
        (ADDR_IDMA_CMD_COUNT,    1),
        (ADDR_IDMA_CMDS_PER_ROW, 1),
        (ADDR_ODMA_CMD_LIST_PTR, ODMA_CMD_BASE),
        (ADDR_ODMA_CMD_COUNT,    1),
        (ADDR_ODMA_CMDS_PER_ROW, 1),
        (ADDR_IDMA_BYTE_LEN,     16),
        (ADDR_WDMA_BYTE_LEN,     16),
        (ADDR_WDMA_SRC_BASE,     WDMA_SRC_BASE),
        (ADDR_RDMA_BYTE_LEN,     16),
        (ADDR_RDMA_SRC_BASE,     RDMA_SRC_BASE),
        (ADDR_ODMA_BYTE_LEN,     16),
        # layer cfg (minimum 1 row × 1 col)
        (0x100, 1), (0x104, 1), (0x108, 1), (0x10C, 1), (0x110, 1), (0x114, 1), (0x118, 1),
    ]
    for addr, val in cfg_pairs:
        desc_list += make_cfg_desc(addr, val)
    desc_list += make_conv_desc()
    desc_list += make_end_desc()
    n_desc = len(cfg_pairs) + 2
    print(f"  desc count: {n_desc}, size: {len(desc_list)} byte")
    rpc.load_ddr(DESC_BASE, desc_list)

    print("[4b/8] POKE dfe desc list base + count")
    rpc.poke_csr(0, ADDR_DESC_LIST_BASE, DESC_BASE)
    rpc.poke_csr(0, ADDR_DESC_COUNT, n_desc)

    print("[5/8] start_dfe (CTRL bit[4])")
    rpc.poke_csr(0, ADDR_CTRL, 1 << 4)
    # wait dfe_done (status bit 9)
    for t in range(50):
        time.sleep(0.05)
        status = rpc.peek_csr(0, ADDR_STATUS)
        if status & (1 << 9):
            print(f"  dfe_done @ t={t*0.05:.2f}s STATUS=0x{status:08x}")
            break
    else:
        print(f"  ⚠️ dfe stuck STATUS=0x{status:08x}")

    print("[5b/8] POKE verify after dfe pulled desc")
    val = rpc.peek_csr(0, ADDR_ODMA_CMD_COUNT)
    print(f"  ODMA_CMD_COUNT = {val} (expect 1)")
    val = rpc.peek_csr(0, ADDR_IDMA_CMD_COUNT)
    print(f"  IDMA_CMD_COUNT = {val} (expect 1)")

    print("[6/8] start_layer (CTRL bit[5])")
    rpc.poke_csr(0, ADDR_CTRL, 1 << 5)

    print("[7/8] Poll STATUS 10s")
    layer_done = False
    for t in range(100):
        time.sleep(0.1)
        status = rpc.peek_csr(0, ADDR_STATUS)
        seq_dbg = rpc.peek_csr(0, ADDR_SEQ_DBG)
        if t % 5 == 0 or status & (1 << 11):
            seq_state = seq_dbg & 0xF
            fifo_cnt = (seq_dbg >> 4) & 0xF
            isg_st = (seq_dbg >> 8) & 0xF
            osg_st = (seq_dbg >> 12) & 0xF
            print(f"  t={t*0.1:.1f}s STATUS=0x{status:08x} seq={seq_state} fifo={fifo_cnt} iSG={isg_st} oSG={osg_st}")
        if status & (1 << 11):
            print(f"  ✅ LAYER_DONE at t={t*0.1:.1f}s")
            layer_done = True
            break

    print("\n[8/8] DDR readback at 0x10D00000")
    data = rpc.read_ddr(ODMA_DST_BASE, 32)
    print(f"  first 32 byte: {data.hex()}")
    nz = sum(1 for b in data[:16] if b != 0)
    print(f"  first 16 byte nonzero: {nz}/16")

    print("\n=== 结论 ===")
    if layer_done and nz > 0:
        print("[PASS] Minimum 4-DMA test PASS")
        print("  -> axi_dm + axi_noc basic OK, multi-cmd triggers vendor edge")
    elif not layer_done:
        print("[FAIL] Minimum 4-DMA test STUCK")
        print("  -> single cmd stuck, axi_dm + axi_noc hardware deadlock locked")
    else:
        print("[WARN] layer_done but DDR not written - inconsistent state")

if __name__ == '__main__':
    main()
