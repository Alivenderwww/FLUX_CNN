"""probe_seq_dbg.py — 跑 layer 0, poll ADDR_SEQ_DBG (0x008) 看 sequencer 卡哪个 state."""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'
ADDR_SEQ_DBG = 0x008

STATE_NAMES = {0: 'S_IDLE', 1: 'S_FETCH', 2: 'S_PRELOAD', 3: 'S_DISPATCH', 4: 'S_WAIT', 5: 'S_BARRIER', 6: 'S_END'}

def decode(v):
    return {
        'seq_state': v & 0xF,
        'state_name': STATE_NAMES.get(v & 0xF, f'?{v&0xF}'),
        'fifo_count_lo4': (v >> 4) & 0xF,
        'master_arvalid': (v >> 8) & 0xF,
        'master_rvalid': (v >> 12) & 0xF,
    }

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    print('=== Preload all layers ===')
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

    print()
    print('=== Phase 0: SEQ_DBG before any start ===')
    for c in range(3):
        v = rpc.peek_csr(c, ADDR_SEQ_DBG)
        s = rpc.peek_csr(c, 0x004)
        d_ = decode(v)
        print(f'  c{c} STATUS=0x{s:08x}  SEQ_DBG=0x{v:08x}  state={d_["state_name"]} fifo_lo4={d_["fifo_count_lo4"]} arvalid={d_["master_arvalid"]:b} rvalid={d_["master_rvalid"]:b}')

    print()
    print('=== Phase 1: start_dfe ===')
    cfgs = d.build_layer_cfgs(meta)
    cfg0 = cfgs[0]
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x180, cfg0['desc_list_base'][c])
            rpc.poke_csr(c, 0x184, cfg0['desc_count'][c])
            rpc.poke_csr(c, 0x000, 0x10)
    # 等 dfe_done
    for t in range(30):
        all_done = True
        for c in range(3):
            s = rpc.peek_csr(c, 0x004)
            if not (s & 0x200): all_done = False
        if all_done: break
        time.sleep(0.05)
    for c in range(3):
        v = rpc.peek_csr(c, ADDR_SEQ_DBG)
        s = rpc.peek_csr(c, 0x004)
        d_ = decode(v)
        print(f'  c{c} STATUS=0x{s:08x}  SEQ_DBG=0x{v:08x}  state={d_["state_name"]} fifo_lo4={d_["fifo_count_lo4"]} arvalid={d_["master_arvalid"]:b} rvalid={d_["master_rvalid"]:b}')

    print()
    print('=== Phase 2: start_layer + poll SEQ_DBG ===')
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            rpc.poke_csr(c, 0x000, 0x20)
    for t in range(20):
        time.sleep(0.1)
        line = f't={t*0.1:.1f}s'
        for c in range(3):
            v = rpc.peek_csr(c, ADDR_SEQ_DBG)
            s = rpc.peek_csr(c, 0x004)
            d_ = decode(v)
            line += f'  c{c}={d_["state_name"]}/fifo{d_["fifo_count_lo4"]:x}/arv{d_["master_arvalid"]:b}/rv{d_["master_rvalid"]:b} S{s&0xfff:03x}'
        print(line)
        # all done?
        if all(rpc.peek_csr(c, 0x004) & 0x001 for c in range(3)):
            print('=== ALL LAYER_DONE STICKY === DONE')
            break
