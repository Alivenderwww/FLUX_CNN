"""PEEK 详细 STATUS bit + SEQ_DBG 各 core, 看 v26 是不是真跟 v25 一样 stuck."""
import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
from vd100_rpc import Vd100Rpc

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    for c in range(3):
        s = rpc.peek_csr(c, 0x004)
        d = rpc.peek_csr(c, 0x008)
        print(f'-- core {c} --')
        print(f'  STATUS=0x{s:08x}')
        print(f'    layer_done(11)={(s>>11)&1} layer_busy(10)={(s>>10)&1}')
        print(f'    dfe_done(9)={(s>>9)&1} dfe_busy(8)={(s>>8)&1}')
        print(f'    odma_done(7)={(s>>7)&1} wdma_done(6)={(s>>6)&1} idma_done(5)={(s>>5)&1}')
        print(f'    odma_busy(4)={(s>>4)&1} wdma_busy(3)={(s>>3)&1} idma_busy(2)={(s>>2)&1}')
        print(f'  SEQ_DBG=0x{d:08x}')
        print(f'    seq_state={d&0xF}  fifo_count[3:0]={(d>>4)&0xF}  arvalid={(d>>8)&0xF:b}  rvalid={(d>>12)&0xF:b}')
