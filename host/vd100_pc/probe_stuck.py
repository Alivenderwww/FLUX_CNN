"""集成: reset 后跑 layer 0, stuck 时 scan OFM 看写到哪一行 + poll SEQ_DBG 看 state change."""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'
STATE_NAMES = {0: 'S_IDLE', 1: 'S_FETCH', 2: 'S_PRELOAD', 3: 'S_DISPATCH', 4: 'S_WAIT', 5: 'S_BARRIER', 6: 'S_END'}

OFM_BASE = 0x10d80000

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()

    # Clear OFM region (write 0 to detect new writes)
    print('=== Clear OFM region 256KB to 0x00 ===')
    for off in range(0, 256*1024, 32768):
        rpc.load_ddr(OFM_BASE + off, b'\x00' * 32768)

    print('=== Preload ===')
    meta = d.parse_meta(CASE)
    gb = d.derive_smc_global_base(meta)
    n_layers = meta['NUM_LAYERS']
    for l in range(n_layers):
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root:
            d.load_ifb_smc(rpc, CASE, meta, l, gb)
        d.load_wb_smc(rpc, CASE, meta, l, gb)
        d.load_rdma_smc(rpc, CASE, meta, l, gb)
        d.load_desc_smc(rpc, CASE, meta, l)
        d.load_sg_cmd_smc(rpc, CASE, meta, l)
    print('Preload done')

    cfgs = d.build_layer_cfgs(meta)
    cfg0 = cfgs[0]

    # phase 1 start_dfe
    print('=== Phase 1: start_dfe ===')
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x180, cfg0['desc_list_base'][c])
            rpc.poke_csr(c, 0x184, cfg0['desc_count'][c])
            rpc.poke_csr(c, 0x000, 0x10)
    for t in range(30):
        if all(rpc.peek_csr(c, 0x004) & 0x200 for c in range(3)): break
        time.sleep(0.05)
    print(f'  dfe done {t}')

    # phase 2 start_layer
    print('=== Phase 2: start_layer ===')
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x000, 0x20)

    # Poll SEQ_DBG every 100ms for 3s, only print changes
    print('--- poll SEQ_DBG (only state changes) ---')
    prev = [None]*3
    for t in range(30):
        time.sleep(0.1)
        changed = False
        snap = []
        for c in range(3):
            v = rpc.peek_csr(c, 0x008)
            s = v & 0xF
            fifo = (v >> 4) & 0xF
            arv = (v >> 8) & 0xF
            rv = (v >> 12) & 0xF
            stamp = (s, fifo)
            snap.append((STATE_NAMES.get(s,"?"), fifo, arv, rv))
            if stamp != prev[c]:
                changed = True
                prev[c] = stamp
        if changed:
            print(f't={t*0.1:.1f}s  ' + '  '.join(f'c{i}={n}/f{f:x}/ar{a:b}/rv{r:b}' for i,(n,f,a,r) in enumerate(snap)))
        if all(rpc.peek_csr(c, 0x004) & 0x001 for c in range(3)):
            print('=== ALL LAYER_DONE === ')
            break

    # scan OFM extent (4KB chunks)
    print()
    print('=== OFM scan extent (4KB chunks) ===')
    last_global = -1
    nz_global = 0
    for off in range(0, 256*1024, 4096):
        try:
            data = rpc.read_ddr(OFM_BASE + off, 4096)
        except Exception as e:
            print(f'  read_ddr @+0x{off:05x} FAIL: {e}')
            break
        nz = sum(1 for b in data if b != 0)
        last = -1
        for i in range(len(data)-1, -1, -1):
            if data[i] != 0: last = i; break
        if nz > 0:
            last_global = off + max(last, 0)
            nz_global += nz
        elif last_global >= 0 and off > last_global + 8192:
            break  # 8KB 都没新 nonzero, 停扫
    print(f'TOTAL nonzero={nz_global}, last_nonzero @ 0x{OFM_BASE + last_global:08x} (+0x{last_global:x})')
