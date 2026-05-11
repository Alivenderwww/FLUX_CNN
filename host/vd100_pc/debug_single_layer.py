"""
debug_single_layer.py — 不走 RUN_LAYERS, 手动 POKE CSR + 长 poll, 看 ConvCore 行为
"""
import sys, time
sys.path.insert(0, '.')
import deploy_smc_case as d
from vd100_rpc import Vd100Rpc

CASE = '../../sim/tb_smc/cases/vd100_resnet11_n3'

def explain_status(v):
    return (f"done={v&1} layer_busy={(v>>1)&1} "
            f"idma_busy={(v>>2)&1} wdma_busy={(v>>3)&1} odma_busy={(v>>4)&1} "
            f"idma_done={(v>>5)&1} wdma_done={(v>>6)&1} odma_done={(v>>7)&1} "
            f"dfe_busy={(v>>8)&1} dfe_done={(v>>9)&1}")

with Vd100Rpc('169.254.111.10') as rpc:
    print('ping:', hex(rpc.ping()))
    meta = d.parse_meta(CASE)
    gb = d.derive_smc_global_base(meta)
    print(f'SMC_GLOBAL_BASE=0x{gb:08x}')

    # 1. preload 完整 layer 0 数据 (3 核全 LOAD)
    print('preload layer 0 ...')
    d.load_ifb_smc(rpc, CASE, meta, 0, gb)
    d.load_wb_smc(rpc, CASE, meta, 0, gb)
    d.load_rdma_smc(rpc, CASE, meta, 0, gb)
    d.load_desc_smc(rpc, CASE, meta, 0)
    d.load_sg_cmd_smc(rpc, CASE, meta, 0)

    # 2. 写 boot regs core 0
    print('write boot regs core 0 (DESC_LIST_BASE + COUNT)...')
    rpc.poke_csr(0, 0x180, 0x10A00000)
    rpc.poke_csr(0, 0x184, 63)
    print('STATUS pre-start:', hex(rpc.peek_csr(0, 0x004)))

    # 3. 写 start_dfe
    print('start_dfe (CTRL=0x10) on core 0 only ...')
    rpc.poke_csr(0, 0x000, 0x10)

    # 4. 慢 poll 5s
    last_status = None
    for i in range(50):
        time.sleep(0.1)
        try:
            v = rpc.peek_csr(0, 0x004)
        except Exception as e:
            print(f'  t={(i+1)*0.1:.1f}s PEEK FAILED: {e}')
            break
        if v != last_status:
            print(f'  t={(i+1)*0.1:.1f}s STATUS=0x{v:08x}  {explain_status(v)}')
            last_status = v
        if v & 0x1:
            print('  DONE!')
            break
    print('exit poll loop')
