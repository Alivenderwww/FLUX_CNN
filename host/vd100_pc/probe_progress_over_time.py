"""持续跑 layer 0, 每秒 scan OFM size + STATUS, 看是真死锁还是慢进."""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'
OFM_BASE = 0x10d80000
STATE_NAMES = {0: 'IDLE', 1: 'FETCH', 2: 'PRELOAD', 3: 'DISPATCH', 4: 'WAIT', 5: 'BARRIER', 6: 'END'}

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    # clear 256 KB
    for off in range(0, 256*1024, 32768):
        rpc.load_ddr(OFM_BASE + off, b'\x00' * 32768)
    print('OFM cleared')

    meta = d.parse_meta(CASE)
    gb = d.derive_smc_global_base(meta)
    for l in range(meta['NUM_LAYERS']):
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root: d.load_ifb_smc(rpc, CASE, meta, l, gb)
        d.load_wb_smc(rpc, CASE, meta, l, gb)
        d.load_rdma_smc(rpc, CASE, meta, l, gb)
        d.load_desc_smc(rpc, CASE, meta, l)
        d.load_sg_cmd_smc(rpc, CASE, meta, l)

    cfgs = d.build_layer_cfgs(meta)
    cfg0 = cfgs[0]

    # phase 1
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x180, cfg0['desc_list_base'][c])
            rpc.poke_csr(c, 0x184, cfg0['desc_count'][c])
            rpc.poke_csr(c, 0x000, 0x10)
    for t in range(30):
        if all(rpc.peek_csr(c, 0x004) & 0x200 for c in range(3)): break
        time.sleep(0.05)

    # phase 2
    t_start = time.time()
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x000, 0x20)

    print('=== running layer 0, scan OFM size every 1s for 10s ===')
    for t in range(10):
        time.sleep(1.0)
        # scan OFM (16 KB chunks, 200 KB total)
        last_nonzero = -1
        nz_total = 0
        for off in range(0, 200*1024, 16384):
            try:
                data = rpc.read_ddr(OFM_BASE + off, 16384)
            except Exception:
                break
            for i in range(len(data)-1, -1, -1):
                if data[i] != 0:
                    last_nonzero = off + i
                    break
            nz_total += sum(1 for b in data if b != 0)
        # state
        states = []
        for c in range(3):
            v = rpc.peek_csr(c, 0x008)
            s = STATE_NAMES.get(v & 0xF, '?')
            states.append(f'c{c}={s}')
        all_done = all(rpc.peek_csr(c, 0x004) & 0x001 for c in range(3))
        print(f't={t+1}s  nz={nz_total}  last@+0x{last_nonzero:x} ({last_nonzero}B)  states=[{",".join(states)}]  all_done={all_done}')
        if all_done:
            print(f'=== layer 0 done in {time.time()-t_start:.2f}s ===')
            break
