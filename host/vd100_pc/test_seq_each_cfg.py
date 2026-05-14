#!/usr/bin/env python3
"""每个 CFG_WRITE 写不同寄存器, 看哪些生效."""
import sys, struct, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
from vd100_rpc import Vd100Rpc

def make_cfg(addr, data):
    w0 = (0x3) | ((addr & 0xFFF) << 4)
    return struct.pack('<IIII', w0, data, 0, 0) + b'\x00' * 16
def make_conv():
    return struct.pack('<IIII', 0x1, 0, 0, 0) + b'\x00' * 16
def make_end():
    return struct.pack('<IIII', 0xF, 0, 0, 0) + b'\x00' * 16

# 测试集: 一系列不同 register, 每个写 magic value
# 都是 seq_w writable + 有 readback 的寄存器
TESTS = [
    # (addr, mask, name)
    (0x100, 0xFFFF,    'H_OUT'),
    (0x104, 0xFFFF,    'W_OUT'),
    (0x108, 0xFFFF,    'W_IN'),
    (0x10C, 0xF,       'K'),
    (0x110, 0x7,       'STRIDE'),
    (0x114, 0x3F,      'CIN_SLICES'),
    (0x118, 0x3F,      'COUT_SLICES'),
    (0x11C, 0x3F,      'TILE_W'),
    (0x120, 0xFF,      'NUM_TILES'),
    (0x124, 0x3F,      'LAST_VALID_W'),
    (0x128, 0x3FF,     'TOTAL_WRF'),
    (0x130, 0x3FF,     'KK'),
    (0x134, 0x7,       'ROUNDS_PER_CINS'),
    (0x138, 0x3F,      'ROUND_LEN_LAST'),
    (0x13C, 0xFFFFF,   'IFB_BASE'),
    (0x140, 0xFFFFF,   'WB_BASE'),
    (0x144, 0xFFFFF,   'OFB_BASE'),
    (0x14C, 0xFFFFF,   'IFB_ROW_STEP'),
    (0x154, 0xFFFFF,   'WB_COUT_STEP'),
    (0x15C, 0xFFFFF,   'TILE_IN_STEP'),
    (0x160, 0x3F,      'SDP_SHIFT'),
    (0x168, 0xFFFF,    'H_IN_TOTAL'),
    (0x16C, 0xFF,      'IFB_STRIP_ROWS'),
    (0x170, 0x3F,      'OFB_STRIP_ROWS'),
    (0x174, 0xFFFFF,   'DDR_IFM_ROW_STR'),
    (0x178, 0xFFFFF,   'DDR_OFM_ROW_STR'),
    (0x1A0, 0xFFFFF,   'IFB_RING_WORDS'),
    (0x1A4, 0xFFFFF,   'OFB_ROW_WORDS'),
    (0x1A8, 0xFFFFF,   'OFB_RING_WORDS'),
    (0x1AC, 0xFFFFF,   'IFB_ISS_STEP'),
    (0x1B0, 0xFFFFF,   'IFB_KY_STEP'),
    (0x1B4, 0xFFFF,    'TILE_PIX_STEP'),
    (0x200, 0xFFFFFFFF,'IDMA_SRC_BASE'),
    (0x204, 0xFFFFFF,  'IDMA_BYTE_LEN'),
    (0x210, 0xFFFFFFFF,'WDMA_SRC_BASE'),
    (0x214, 0xFFFFFF,  'WDMA_BYTE_LEN'),
    (0x220, 0xFFFFFFFF,'ODMA_DST_BASE'),
    (0x224, 0xFFFFFF,  'ODMA_BYTE_LEN'),
    (0x230, 0xFFFFFFFF,'RDMA_SRC_BASE'),
    (0x234, 0xFFFFFF,  'RDMA_BYTE_LEN'),
    (0x1D8, 0xFFFFFFFF,'IDMA_CMDLIST'),
    (0x1DC, 0xFFFF,    'IDMA_CMD_COUNT'),
    (0x1E0, 0xFF,      'IDMA_CMDS_PER_ROW'),
    (0x1E4, 0xFFFFFFFF,'ODMA_CMDLIST'),
    (0x1E8, 0xFFFF,    'ODMA_CMD_COUNT'),
    (0x1EC, 0xFF,      'ODMA_CMDS_PER_ROW'),
]
# 共 46 个寄存器, 加 CONV+END = 48 desc
# 每个写不同 magic value (addr ^ 0xCAFE0000), readback 验证

descs = []
for i, (addr, mask, name) in enumerate(TESTS):
    magic = 0xCAFE0000 | (i & 0xFFFF)   # 每个不同
    descs.append(make_cfg(addr, magic))
descs.append(make_conv())
descs.append(make_end())
desc_list = b''.join(descs)
n_total = len(descs)
print(f'共 {n_total} desc ({len(TESTS)} CFG + CONV + END)')

DESC = 0x10A00000
rpc = Vd100Rpc('169.254.111.10', 5000); rpc.connect()
rpc.load_ddr(DESC, b'\x00' * (n_total * 32 + 64))
rpc.load_ddr(DESC, desc_list)
rpc.poke_csr(0, 0x180, DESC)
rpc.poke_csr(0, 0x184, n_total)
rpc.poke_csr(0, 0x000, 1 << 4)
time.sleep(0.2)
print(f'  start_dfe: STATUS=0x{rpc.peek_csr(0, 0x004):08x} SEQ_DBG=0x{rpc.peek_csr(0, 0x008):08x}')
rpc.poke_csr(0, 0x000, 1 << 5)
time.sleep(0.5)
print(f'  start_layer: STATUS=0x{rpc.peek_csr(0, 0x004):08x} SEQ_DBG=0x{rpc.peek_csr(0, 0x008):08x}')

print('=== readback (注意: 写入值经过 mask 后 readback) ===')
ok_count = 0
fail_count = 0
for i, (addr, mask, name) in enumerate(TESTS):
    magic = 0xCAFE0000 | (i & 0xFFFF)
    expected = magic & mask
    v = rpc.peek_csr(0, addr) & mask
    ok = (v == expected)
    if ok: ok_count += 1
    else: fail_count += 1
    tag = 'OK' if ok else 'FAIL'
    print(f'  [{i:>2}] {name:<20} addr=0x{addr:03x} expect=0x{expected:08x} got=0x{v:08x} {tag}')

print(f'\nSUMMARY: {ok_count} OK / {fail_count} FAIL out of {len(TESTS)}')
