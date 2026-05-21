"""diag_n3_hang.py — 系统化收集 N=3 150MHz 新 PDI 的 hang 诊断数据.

跑 3 种 case (N=1 mnist / N=2 resnet11 / N=3 resnet11), 每种 case:
  1. RUN_LAYERS 前: peek 3 个 core CSR (清干净状态)
  2. RUN_LAYERS (会 timeout) 后: peek 3 个 core CSR
  3. 解码 STATUS / SEQ_DBG

对比"卡的 core"跟"完成的 core"状态差异, 定位 hang 位置.
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import deploy_smc_case as dsc
from vd100_rpc import Vd100Rpc


def decode_status(s):
    bits = {
        0:  'core_done_sticky', 1:  'layer_busy',
        2:  'idma_busy', 3:  'wdma_busy', 4:  'odma_busy',
        5:  'idma_done', 6:  'wdma_done', 7:  'odma_done',
        8:  'dfe_busy',  9:  'dfe_done',
        10: 'layer_busy_dup', 11: 'layer_done',
    }
    return [n for b, n in bits.items() if s & (1 << b)]


def decode_seq_dbg(d):
    return {
        'seq_state':      d & 0xF,
        'fifo_count':     (d >> 4) & 0xF,
        'master_arvalid': (d >> 8) & 0xF,
        'master_rvalid':  (d >> 12) & 0xF,
    }


def peek_all_cores(rpc, label):
    print(f"\n--- {label} ---")
    for c in range(3):
        s = rpc.peek_csr(c, 0x004)
        d = rpc.peek_csr(c, 0x008)
        b = rpc.peek_csr(c, 0x180)
        cnt = rpc.peek_csr(c, 0x184)
        sd = decode_seq_dbg(d)
        print(f"  core {c}: STATUS=0x{s:04x} {decode_status(s)}")
        print(f"          DESC_BASE=0x{b:08x} COUNT={cnt}  "
              f"seq_state={sd['seq_state']} fifo={sd['fifo_count']} "
              f"AR={sd['master_arvalid']:04b} RV={sd['master_rvalid']:04b}")


def run_case(rpc, case_dir, label):
    print(f"\n========== {label}: {case_dir} ==========")
    meta = dsc.parse_meta(case_dir)
    n_layers = meta['NUM_LAYERS']
    gb = dsc.derive_smc_global_base(meta)

    peek_all_cores(rpc, "before deploy")

    # deploy
    t0 = time.time()
    for l in range(n_layers):
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root:
            dsc.load_ifb_smc(rpc, case_dir, meta, l, gb)
        dsc.load_wb_smc(rpc, case_dir, meta, l, gb)
        dsc.load_rdma_smc(rpc, case_dir, meta, l, gb)
        dsc.load_desc_smc(rpc, case_dir, meta, l)
        dsc.load_sg_cmd_smc(rpc, case_dir, meta, l)
    print(f"  deploy took {time.time()-t0:.1f}s")

    peek_all_cores(rpc, "after deploy (before RUN)")

    # RUN_LAYERS (会 timeout)
    cfgs = dsc.build_layer_cfgs(meta)
    try:
        cy = rpc.run_layers(cfgs)
        print(f"\n  *** UNEXPECTED PASS: cycles={cy} ***")
    except Exception as e:
        print(f"\n  RUN_LAYERS RpcError: {e}")

    peek_all_cores(rpc, "after RUN_LAYERS timeout")


def main():
    rpc = Vd100Rpc('169.254.111.10', 5000, timeout=90.0)
    rpc.connect()
    rpc.ping()

    # case 1: N=1 MNIST
    run_case(rpc, './models/compiled/mnist_allconv_n1', "N=1 MNIST")

    # case 2: N=2 ResNet11
    run_case(rpc, './models/compiled/resnet11_n2', "N=2 ResNet11")

    # case 3: N=3 ResNet11
    run_case(rpc, './models/compiled/resnet11_n3', "N=3 ResNet11")

    rpc.close()


if __name__ == '__main__':
    main()
