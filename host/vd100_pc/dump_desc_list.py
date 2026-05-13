"""dump_desc_list.py — 读 c0 desc list 头几个 desc, 看 dfe 应该拉的 DDR 内容是什么."""
import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
from vd100_rpc import Vd100Rpc

CORES = [(0, 0x10a00000), (1, 0x11a00000), (2, 0x12a00000)]
START = 45
N = 18                   # desc[45..62]

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    for core_id, base in CORES:
        data = rpc.read_ddr(base + START * 32, N * 32)
        print(f'=== c{core_id} desc[{START}..{START+N-1}] @ 0x{base + START*32:08x} ===')
        for i in range(N):
            idx = START + i
            off = i * 32
            d = data[off:off + 32]
            word0 = int.from_bytes(d[0:4], 'little')
            word1 = int.from_bytes(d[4:8], 'little')
            type_id = word0 & 0xF
            type_name = {0: 'NOP', 1: 'CONV', 2: 'BARRIER', 3: 'CFG_WRITE', 0xF: 'END'}.get(type_id, f'UNK({type_id:x})')
            print(f'  desc[{idx:2d}] type=0x{type_id:x} ({type_name:9s})  w0=0x{word0:08x} w1=0x{word1:08x}')
