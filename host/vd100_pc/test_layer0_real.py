"""
test_layer0_real.py - 真实 ResNet11 layer 0 (Patch) 在板上跑 + PC 端 poll done

跳过 a72 RUN_LAYERS (避免 lwIP hot-poll timeout). PC 直接 POKE 三核 boot regs +
start_dfe + 自己 poll STATUS, 看 ConvCore 是否真完成 Patch layer.
"""
import os, sys, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    print('=== Preload all layers ===')
    meta = d.parse_meta(CASE)
    gb = d.derive_smc_global_base(meta)
    n_layers = meta['NUM_LAYERS']
    t0 = time.time()
    for l in range(n_layers):
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root:
            d.load_ifb_smc(rpc, CASE, meta, l, gb)
        d.load_wb_smc(rpc, CASE, meta, l, gb)
        d.load_rdma_smc(rpc, CASE, meta, l, gb)
        d.load_desc_smc(rpc, CASE, meta, l)
        d.load_sg_cmd_smc(rpc, CASE, meta, l)
    print(f'Preload done ({time.time()-t0:.1f}s)')

    print('=== Run layer 0: phase 1 — start_dfe (3 cores) ===')
    cfgs = d.build_layer_cfgs(meta)
    cfg0 = cfgs[0]
    active = []
    for c in range(3):
        if cfg0['desc_count'][c] > 0:
            base = cfg0['desc_list_base'][c]
            cnt  = cfg0['desc_count'][c]
            rpc.poke_csr(c, 0x180, base)
            rpc.poke_csr(c, 0x184, cnt)
            rpc.poke_csr(c, 0x000, 0x10)
            print(f'  core {c}: start_dfe desc_base=0x{base:08x} count={cnt}')
            active.append(c)

    print('--- poll dfe_done (STATUS[9]=1) ---')
    t_dfe = time.time()
    for i in range(120):
        time.sleep(0.02)
        statuses = [rpc.peek_csr(c, 0x004) for c in active]
        dfe_done = sum(1 for s in statuses if (s >> 9) & 0x1)
        if i % 5 == 0 or dfe_done == len(active):
            line = ' '.join(f'c{c}=0x{s:08x}' for c, s in zip(active, statuses))
            print(f'  t={(i+1)*0.02:.2f}s {line} dfe_done={dfe_done}/{len(active)}')
        if dfe_done == len(active):
            print(f'  -- dfe done in {(time.time()-t_dfe)*1000:.1f}ms --')
            break
    else:
        print('  *** dfe_done TIMEOUT, abort ***')
        sys.exit(1)

    print('=== Run layer 0: phase 2 — start_layer (CTRL[5]=1) ===')
    for c in active:
        rpc.poke_csr(c, 0x000, 0x20)
        print(f'  core {c}: start_layer pulse')

    print('--- poll layer_done sticky (STATUS[0]=1) ---')
    t1 = time.time()
    for i in range(300):
        time.sleep(0.05)
        statuses = [rpc.peek_csr(c, 0x004) for c in active]
        done = sum(1 for s in statuses if s & 0x1)
        if i % 4 == 0 or done >= 1:
            line = ' '.join(f'c{c}=0x{s:08x}' for c, s in zip(active, statuses))
            print(f'  t={(i+1)*0.05:.2f}s {line}  done={done}/{len(active)}')
        if done == len(active):
            print(f'\n  *** ALL CORES LAYER 0 DONE in {(time.time()-t1)*1000:.1f}ms ***')
            break
    else:
        print(f'\n  *** TIMEOUT after 15s ***')
