"""probe_cfg_after_dfe.py — Phase 1 (start_dfe) 后 PEEK 各 cfg_regs 值, 看 CFG_WRITE 真有没写到."""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'

REGS_TO_PEEK = [
    (0x100, 'H_OUT'), (0x104, 'W_OUT'), (0x108, 'W_IN'), (0x10C, 'K'),
    (0x110, 'STRIDE'), (0x114, 'CIN_SLICES'), (0x118, 'COUT_SLICES'),
    (0x1D8, 'IDMA_CMD_LIST_PTR'),
    (0x1DC, 'IDMA_CMD_COUNT'),
    (0x1E0, 'IDMA_CMDS_PER_ROW'),
    (0x1E4, 'ODMA_CMD_LIST_PTR'),
    (0x1E8, 'ODMA_CMD_COUNT'),
    (0x1EC, 'ODMA_CMDS_PER_ROW'),
    (0x210, 'WDMA_SRC_BASE'), (0x214, 'WDMA_BYTE_LEN'),
    (0x230, 'RDMA_SRC_BASE'), (0x234, 'RDMA_BYTE_LEN'),
]

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    print('=== Preload (skip if already done; if board reset, run probe_seq_dbg first) ===')

    print()
    print('=== PEEK cfg_regs (phase 0 → 应全 0/garbage) ===')
    for c in range(3):
        print(f'-- core {c} --')
        for addr, name in REGS_TO_PEEK:
            v = rpc.peek_csr(c, addr)
            print(f'  0x{addr:03x} {name:20s} = 0x{v:08x} ({v})')
