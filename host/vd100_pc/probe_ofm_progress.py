"""扫 OFM DST 看 ConvCore 写到哪一行就卡了; 同时 poll SEQ_DBG 看 state 是否真 stuck."""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
from vd100_rpc import Vd100Rpc

# layer 0 OFM 写到 0x10d80000 (c0). 扫 2 KB 看末尾在哪
OFM_BASE = 0x10d80000
SCAN = 1024

STATE_NAMES = {0: 'S_IDLE', 1: 'S_FETCH', 2: 'S_PRELOAD', 3: 'S_DISPATCH', 4: 'S_WAIT', 5: 'S_BARRIER', 6: 'S_END'}

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    data = rpc.read_ddr(OFM_BASE, SCAN)
    # 找最后一个非零 byte
    last_nonzero = -1
    for i, b in enumerate(data):
        if b != 0:
            last_nonzero = i
    nonzero_count = sum(1 for b in data if b != 0)
    print(f'OFM DST 0x{OFM_BASE:08x} scan {SCAN}B: last_nonzero=0x{last_nonzero:x} (#{last_nonzero}), total_nonzero={nonzero_count}')

    # 看几个 cluster
    for off in [0, 256, 512, 1024, 2048, 4096, 6144]:
        if off + 64 > SCAN: break
        chunk = data[off:off+64]
        nz = sum(1 for b in chunk if b != 0)
        first = chunk[:16].hex()
        print(f'  [0x{OFM_BASE+off:08x}] nz={nz}/64  first16: {first}')

    print()
    print('=== poll SEQ_DBG 10 sec 看 state/fifo 是否变化 ===')
    prev = [None]*3
    for t in range(50):
        time.sleep(0.2)
        line = f't={t*0.2:.1f}s'
        for c in range(3):
            v = rpc.peek_csr(c, 0x008)
            s = v & 0xF
            fifo = (v >> 4) & 0xF
            arv = (v >> 8) & 0xF
            rv = (v >> 12) & 0xF
            stamp = (s, fifo, arv, rv)
            if stamp != prev[c]:
                line += f'  c{c}={STATE_NAMES.get(s,"?")}/fifo{fifo:x}/arv{arv:b}/rv{rv:b} *'
                prev[c] = stamp
            else:
                line += f'  c{c}=same'
        if '*' in line:
            print(line)
