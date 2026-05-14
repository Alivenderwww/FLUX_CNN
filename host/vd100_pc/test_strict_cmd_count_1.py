#!/usr/bin/env python3
"""
严格版 cmd_count=1 测试: 先 deploy + 清 OFM 区, 再跑, 排除 preload 残留数据干扰.

目的: 验证 BD m00_axi fix 后 ODMA 是否真写 DDR (vs preload 残留).
"""
import os, sys, struct, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

HOST = '169.254.111.10'; PORT = 5000

ADDR_CTRL          = 0x000
ADDR_STATUS        = 0x004
ADDR_SEQ_DBG       = 0x008
ADDR_ODMA_CMD_COUNT= 0x1E8
ADDR_IDMA_CMD_COUNT= 0x1DC
ADDR_IDMA_CMDLIST  = 0x1D8
ADDR_ODMA_CMDLIST  = 0x1E4

CORE_BASES    = [0x10A00000, 0x11A00000, 0x12A00000]
ODMA_SG_BASES = [0x10C00000, 0x11C00000, 0x12C00000]
IDMA_SG_BASES = [0x10B00000, 0x11B00000, 0x12B00000]
OFM_BASES     = [0x10D00000, 0x11D00000, 0x12D00000]   # ODMA 写到这
N_DESCS = 63

def patch_desc(rpc, desc_base, target_addr, new_val):
    data = rpc.read_ddr(desc_base, N_DESCS * 32)
    for i in range(N_DESCS):
        off = i * 32
        w0 = struct.unpack('<I', data[off:off+4])[0]
        if (w0 & 0xF) == 0x3 and ((w0 >> 4) & 0xFFF) == target_addr:
            rpc.load_ddr(desc_base + off + 4, struct.pack('<I', new_val))
            return i
    return -1

def main():
    print('=== Strict cmd_count=1 test ===')
    case_dir = r"C:/_Project/FLUX_CNN/sim/tb_smc/cases/vd100_resnet11_n3"
    print('[1/7] deploy_smc_case --skip-run (load 数据)')
    subprocess.run([
        r"C:/_Project/FLUX_CNN/toolchain/.venv/Scripts/python.exe",
        r"C:/_Project/FLUX_CNN/host/vd100_pc/deploy_smc_case.py",
        "--case-dir", case_dir, "--vd100-ip", HOST, "--port", str(PORT), "--skip-run"
    ], check=False, timeout=120, capture_output=True)

    rpc = Vd100Rpc(HOST, PORT); rpc.connect()

    print('[2/7] 清 OFM 区 (3 core × 1MB at 0x10D00000/0x11D00000/0x12D00000)')
    for base in OFM_BASES:
        rpc.load_ddr(base, b'\x00' * (1024 * 1024))
        nz = sum(1 for b in rpc.read_ddr(base, 4096) if b != 0)
        print(f'  cleared {base:#x}: nonzero={nz}/4096 (must be 0)')

    print('[3/7] Patch desc IDMA/ODMA_CMD_COUNT=1 + SG cmd[0] last_cmd')
    for c, base in enumerate(CORE_BASES):
        i1 = patch_desc(rpc, base, 0x1DC, 1)
        i2 = patch_desc(rpc, base, 0x1E8, 1)
        print(f'  c{c} patched desc[{i1}]=IDMA_CMD_COUNT, desc[{i2}]=ODMA_CMD_COUNT')
    for c in range(3):
        for tag, base in [('ODMA', ODMA_SG_BASES[c]), ('IDMA', IDMA_SG_BASES[c])]:
            cmd0 = rpc.read_ddr(base, 32)
            w1 = struct.unpack('<I', cmd0[4:8])[0]
            rpc.load_ddr(base + 4, struct.pack('<I', w1 | (1 << 23)))

    print('[4/7] start_dfe')
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 4)
    time.sleep(0.5)
    for c in range(3):
        st = rpc.peek_csr(c, ADDR_STATUS)
        print(f'  c{c} STATUS after dfe = 0x{st:08x}')

    print('[5/7] start_layer + poll')
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 5)
    for t in range(50):
        time.sleep(0.1)
        st0 = rpc.peek_csr(0, ADDR_STATUS)
        if st0 & (1 << 11): break
    st0 = rpc.peek_csr(0, ADDR_STATUS)
    sd0 = rpc.peek_csr(0, ADDR_SEQ_DBG)
    print(f'  final c0 STATUS=0x{st0:08x} SEQ_DBG=0x{sd0:08x} t={t*0.1:.1f}s')

    print('[6/7] PEEK cfg_regs CMD_COUNT/CMDLIST 看 CFG_WRITE 是否生效')
    for c in range(3):
        cc_odma = rpc.peek_csr(c, ADDR_ODMA_CMD_COUNT)
        cc_idma = rpc.peek_csr(c, ADDR_IDMA_CMD_COUNT)
        ptr_odma = rpc.peek_csr(c, ADDR_ODMA_CMDLIST)
        ptr_idma = rpc.peek_csr(c, ADDR_IDMA_CMDLIST)
        print(f'  c{c} ODMA_CMD_COUNT={cc_odma} ODMA_CMDLIST=0x{ptr_odma:08x} IDMA_CMD_COUNT={cc_idma} IDMA_CMDLIST=0x{ptr_idma:08x}')

    print('[7/7] OFM 区写入验证 (3 core × 4KB)')
    for c, base in enumerate(OFM_BASES):
        data = rpc.read_ddr(base, 4096)
        nz = sum(1 for b in data if b != 0)
        print(f'  c{c} OFM @ {base:#x}: nonzero={nz}/4096, head={data[:32].hex()}')

if __name__ == '__main__':
    main()
