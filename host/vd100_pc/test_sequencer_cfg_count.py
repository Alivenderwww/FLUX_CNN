#!/usr/bin/env python3
"""二分查找 sequencer 在多少 CFG_WRITE desc 之后开始 fail."""
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

# 用 H_OUT 0x100 寄存器, 每次 PEEK 后写入新 magic, 看 CFG_WRITE 是否生效
TEST_REG = 0x100
TEST_MASK = 0xFFFF

def run_test(rpc, n_cfg):
    """构造 n_cfg CFG_WRITE + 1 CONV + END, 第一个 CFG 写 H_OUT=0xABCD, 看 PEEK."""
    MAGIC = 0xABCD
    # 第一个 CFG 写 magic, 后续 n_cfg-1 个写 dummy (addr=0x180 desc_list_base, csr_w only,
    # seq_w 写不进, 验证侧)
    descs = [make_cfg(TEST_REG, MAGIC)]
    # 后面填充 n_cfg-1 个 CFG_WRITE 到无效地址 (e.g., 0x000, 不在 case 列表)
    for i in range(1, n_cfg):
        # 写到 H_OUT (0x100) 让真覆盖, 但每次都同样 magic 验证 idempotent
        descs.append(make_cfg(TEST_REG, MAGIC))
    descs.append(make_conv())
    descs.append(make_end())
    desc_list = b''.join(descs)
    n_total = len(descs)

    # 清 H_OUT 寄存器 (没法直接写, 但通过 sequencer 写 0)
    DESC = 0x10A00000
    rpc.load_ddr(DESC, b'\x00' * (n_total * 32 + 64))
    rpc.load_ddr(DESC, desc_list)
    rpc.poke_csr(0, 0x180, DESC)
    rpc.poke_csr(0, 0x184, n_total)
    rpc.poke_csr(0, 0x000, 1 << 4)
    time.sleep(0.2)
    rpc.poke_csr(0, 0x000, 1 << 5)
    time.sleep(0.5)
    st = rpc.peek_csr(0, 0x004)
    sd = rpc.peek_csr(0, 0x008)
    v = rpc.peek_csr(0, TEST_REG) & TEST_MASK
    ok = (v == MAGIC)
    return ok, st, sd, v

def main():
    rpc = Vd100Rpc('169.254.111.10', 5000); rpc.connect()
    print(f'初始 STATUS=0x{rpc.peek_csr(0, 0x004):08x}')
    # 注意: 这测试不能多次跑 (sequencer 进 S_END 后 sticky), 需要每次重烧 PDI
    # 这里跑 N=10 单次测试. 如果 PASS, 用户手动重烧 + 改 N 测下一个值.
    import os
    N = int(os.environ.get('N_CFG', '10'))
    print(f'=== Test N_CFG={N} ===')
    ok, st, sd, v = run_test(rpc, N)
    seq = sd & 0xF; fifo = (sd >> 4) & 0xF
    odma_st = (sd >> 8) & 0xF; idma_st = (sd >> 12) & 0xF
    print(f'  STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x} (seq={seq} fifo={fifo} oSG={odma_st} iSG={idma_st})')
    print(f'  H_OUT peek=0x{v:04x} (expect 0xABCD) {"PASS" if ok else "FAIL"}')

if __name__ == '__main__':
    main()
