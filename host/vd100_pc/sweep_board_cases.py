"""sweep_board_cases.py — 板上多 demo case 批量测, 不动 RTL/PDI"""
import argparse, os, sys, subprocess, time
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

TOOLCHAIN = r'C:\_Project\FLUX_CNN\toolchain'
CASES_DIR = r'C:\_Project\FLUX_CNN\sim\tb_smc\cases'
PYTHON = r'C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe'
DEPLOY = r'C:\_Project\FLUX_CNN\host\vd100_pc\deploy_smc_case.py'

# demo_name, expected (H, W, C_out) for OFM compare
DEMOS = [
    # baseline (no wrap)
    ('wslice1',         32, 32, 16, 'H=32 W=32 K=3 (no wrap, baseline)'),
    # === wrap 程度 sweep: 同 W=32 不同 H ===
    ('sw_h64_w32',      64, 32, 16, 'H=64 W=32 K=3 (wrap 2x)'),
    ('sw_h96_w32',      96, 32, 16, 'H=96 W=32 K=3 (wrap 3x)'),
    ('sw_h128_w32',    128, 32, 16, 'H=128 W=32 K=3 (wrap 4x)'),
    # === stride=2 ===
    ('sw_h32_w32_s2',   32, 32, 16, 'H=32 W=32 K=3 s=2 (stride 2 no wrap)'),
    ('sw_h64_w32_s2',   64, 32, 16, 'H=64 W=32 K=3 s=2 (stride 2 wrap)'),
    ('sw_h64_w75_s2',   64, 75, 16, 'H=64 W=75 K=3 s=2'),
    # === W=75 sweep (重复确认) ===
    ('h32_ref',         32, 75, 16, 'H=32 W=75 K=3 ref'),
]


N_CORES = int(os.environ.get('FLUX_N_CORES', '1'))


def gen_case(demo, case_name):
    """toolchain run_multicore_chain --smc --demo X --n_cores N (N 由 FLUX_N_CORES 控制)."""
    cmd = [PYTHON, os.path.join(TOOLCHAIN, 'run_multicore_chain.py'),
           '--smc', '--demo', demo, '--case_name', case_name, '--n_cores', str(N_CORES)]
    env = os.environ.copy()
    env['FLUX_SMC_GLOBAL_BASE'] = '0x10000000'
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180,
                       env=env, cwd=TOOLCHAIN)
    if r.returncode != 0:
        return False, r.stderr[-300:] + r.stdout[-300:]
    return True, ''


def deploy_and_check(rpc, case_dir, demo_name, H, W, C):
    """deploy + read OFM + bit-compare. 返回 (status, ok_rows, first_bad_r, info)"""
    cmd = [PYTHON, DEPLOY, '--case-dir', case_dir, '--vd100-ip', '169.254.111.10']
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    last = r.stdout[-400:] if r.stdout else r.stderr[-400:]
    # 检查 RUN_LAYERS 完成
    if 'hw cycles=' not in r.stdout:
        return ('DEPLOY_FAIL', 0, -1, last)

    # 提取 hw cycles
    cy = '?'
    for ln in r.stdout.split('\n'):
        if 'hw cycles=' in ln:
            cy = ln.split('hw cycles=')[1].split()[0].rstrip(',')
            break

    # 读 expected
    exp_path = os.path.join(case_dir, 'chain_data', 'layer00', 'expected_ofm.txt')
    if not os.path.exists(exp_path):
        return ('NO_EXPECTED', 0, -1, f'cycles={cy}')

    # 取实际 H_OUT (k_size/stride 等改变 OFM size)
    meta_path = os.path.join(case_dir, 'multicore_meta.txt')
    h_out, w_out, c_out = H, W, C
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            for ln in f:
                ln = ln.strip()
                if ln.startswith('FINAL_H_OUT'):    h_out = int(ln.split('=')[1].strip())
                elif ln.startswith('FINAL_W_OUT'):  w_out = int(ln.split('=')[1].strip())
                elif ln.startswith('FINAL_C_OUT'):  c_out = int(ln.split('=')[1].strip())

    WB = 16; cout_slices = (c_out + 15) // 16
    n_bytes = h_out * w_out * cout_slices * WB
    if n_bytes > 4 * 1024 * 1024:
        return ('TOO_BIG', 0, -1, f'OFM {n_bytes} byte')

    # 直接复用 deploy_smc_case 的 read_ofm_smc, 支持 N=2 W slice stitch
    try:
        import deploy_smc_case
        meta = deploy_smc_case.parse_meta(case_dir)
        global_base = meta.get('_DDR_GLOBAL_BASE', 0x10000000)
        # 重建 global_base from manifest if available
        if '_DESC_LIST_BASE_PER_CORE' in meta and meta['_DESC_LIST_BASE_PER_CORE']:
            db0 = meta['_DESC_LIST_BASE_PER_CORE'][0]
            global_base = db0 - deploy_smc_case.SMC_DESC_BASE
        last_l = meta['NUM_LAYERS'] - 1
        got = bytearray(deploy_smc_case.read_ofm_smc(rpc, meta, last_l, global_base))
    except Exception as e:
        return ('READ_ERR', 0, -1, str(e)[:200])

    with open(exp_path) as f:
        exp = bytearray()
        for ln in f:
            ln = ln.strip()
            if ln: exp.extend(bytes.fromhex(ln)[::-1])
    # truncate to actual length
    L = min(len(got), len(exp))
    got = got[:L]; exp = exp[:L]

    # row-by-row
    first_bad = -1; n_ok = 0; tot_diff = 0
    for r_ in range(h_out):
        n_diff = 0
        for x in range(w_out):
            for cc in range(c_out):
                idx = (r_*w_out + x)*WB + cc
                if idx < L and got[idx] != exp[idx]:
                    n_diff += 1
        tot_diff += n_diff
        if n_diff == 0:
            n_ok += 1
        elif first_bad < 0:
            first_bad = r_
    status = 'PASS' if first_bad < 0 else 'FAIL'
    pct = 100*tot_diff / (h_out*w_out*c_out) if h_out*w_out*c_out > 0 else 0
    info = f'cy={cy} mismatch={pct:.1f}% H_out={h_out} W_out={w_out} C_out={c_out}'
    return (status, n_ok, first_bad, info)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vd100-ip', default='169.254.111.10')
    ap.add_argument('--skip-gen', action='store_true', help='跳过生成, 用 existing case dir')
    args = ap.parse_args()

    print(f'Connecting board {args.vd100_ip}...')
    with Vd100Rpc(args.vd100_ip, 5000, timeout=20.0) as rpc:
        rpc.ping()
        print('  OK\n')

        results = []
        for demo, H, W, C, desc in DEMOS:
            case_name = f'sweep_{demo}_n{N_CORES}'
            case_dir = os.path.join(CASES_DIR, case_name)
            print(f'[{demo}] {desc}')

            if not args.skip_gen:
                print(f'  gen case...')
                ok, err = gen_case(demo, case_name)
                if not ok:
                    print(f'    GEN_FAIL: {err}')
                    results.append((demo, 'GEN_FAIL', 0, -1, err[:100]))
                    continue

            status, ok_rows, first_bad, info = deploy_and_check(rpc, case_dir, demo, H, W, C)
            results.append((demo, status, ok_rows, first_bad, info))
            print(f'    -> {status}  ok_rows={ok_rows} first_bad_r={first_bad}  {info}')

        # 总结
        print(f'\n=== SUMMARY ===')
        print(f'{"demo":20s} {"status":12s} {"ok_rows":>10s} {"first_bad":>10s}  info')
        for demo, status, ok_rows, first_bad, info in results:
            print(f'{demo:20s} {status:12s} {ok_rows:>10d} {first_bad:>10d}  {info}')


if __name__ == '__main__':
    main()
