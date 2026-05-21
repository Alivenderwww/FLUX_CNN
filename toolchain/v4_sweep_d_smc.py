"""
v4_sweep_d_smc.py  --  v4 paper §5.5.5 实验 D (N=4 SMC 部分) 单层 W 扫频

跑 4 个 W (16, 32, 64, 128) 单层 K=3 stride=1 case, N=4 SMC.
每 W 一次 vsim run (因为 tb_smc 单 chain), 解析 wall cycles + per-core util.
"""
import datetime
import os
import re
import shutil
import subprocess
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.dirname(_THIS_DIR))

from midend import scheduler                # noqa: E402
import run_multicore_chain as RMC  # noqa: E402

SIM_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "sim", "tb_smc"))


def run_one_smc(w, n_cores=4):
    """跑一次 SMC 单层 case (W=W H=W K=3 c=16). 返回 dict 含 wall_cy + per-core 数据."""
    case_name = f"v4d_n{n_cores}_w{w:03d}"
    out_root = os.path.join(SIM_DIR, "cases", case_name)
    if os.path.exists(out_root):
        shutil.rmtree(out_root)
    os.makedirs(out_root, exist_ok=True)

    layers = [
        scheduler.Layer(f'L0_w{w}', k=3, c_in=16, c_out=16,
                        h_in=w, w_in=w, stride=1, pad=1, sdp_shift=2),
    ]

    print(f"\n=== Generating SMC case: N={n_cores} W={w} ===")
    RMC.run_multicore_chain(layers, n_cores, out_root, smc=True, input_image=None)

    # active_case.txt 让 tb_smc/run.tcl 知道跑哪个 case
    active = os.path.join(SIM_DIR, "active_case.txt")
    with open(active, 'w') as f:
        f.write(f"cases/{case_name}\n")

    # vsim 跑
    print(f"  vsim batch run ...")
    r = subprocess.run("vsim -c -do run.tcl", shell=True,
                       capture_output=True, text=True, cwd=SIM_DIR)
    out = r.stdout + r.stderr
    log_path = os.path.join(SIM_DIR, f"v4d_w{w:03d}_vsim.log")
    with open(log_path, "w", encoding="utf-8", errors="replace") as f:
        f.write(out)

    # parse 输出
    res = {'w': w, 'n_cores': n_cores, 'log': log_path,
           'pass': 'RESULT: PASS' in out,
           'wall_cy': None, 'per_core': []}
    m = re.search(r"Wall:\s+(\d+)\s+cycles", out)
    if m:
        res['wall_cy'] = int(m.group(1))
    # per-core: "    C%0d L%0d cy=%0d fire=%0d util=%.1f%% act_st=... act_id=... wgt_st=... wgt_id=... psm_st=... psm_id=... acc_id=..."
    for cm in re.finditer(
            r"C(\d+)\s+L(\d+)\s+cy=(\d+)\s+fire=(\d+)\s+util=([\d.]+)%"
            r"\s+act_st=(\d+)\s+act_id=(\d+)\s+wgt_st=(\d+)\s+wgt_id=(\d+)"
            r"\s+psm_st=(\d+)\s+psm_id=(\d+)\s+acc_id=(\d+)", out):
        res['per_core'].append({
            'core': int(cm.group(1)), 'layer': int(cm.group(2)),
            'cy':   int(cm.group(3)), 'fire': int(cm.group(4)),
            'util': float(cm.group(5)),
            'act_st': int(cm.group(6)), 'act_id': int(cm.group(7)),
            'wgt_st': int(cm.group(8)), 'wgt_id': int(cm.group(9)),
            'psm_st': int(cm.group(10)), 'psm_id': int(cm.group(11)),
            'acc_id': int(cm.group(12)),
        })
    print(f"  PASS={res['pass']}  wall_cy={res['wall_cy']}  cores logged: {len(res['per_core'])}")
    return res


def main():
    print("=" * 70)
    print("  FLUX CNN v4 §5.5.5 实验 D — N=4 SMC W 扫频")
    print("=" * 70)

    results = []
    for w in [16, 32, 64, 128]:
        res = run_one_smc(w, n_cores=4)
        results.append(res)

    # 汇总打印
    print("\n" + "=" * 70)
    print("  Summary")
    print("=" * 70)
    print(f"  {'W':>4}  {'N':>2}  {'PASS':<6}  {'wall_cy':>10}  per-core util")
    for r in results:
        utils = ', '.join(f"C{p['core']}={p['util']:.1f}%" for p in r['per_core'])
        print(f"  {r['w']:>4}  {r['n_cores']:>2}  {str(r['pass']):<6}  "
              f"{r['wall_cy'] or 0:>10}  {utils}")

    # 写汇总 .json (给汇总 md 脚本用)
    import json
    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_json = os.path.join(SIM_DIR, f"v4_sweep_d_smc_{ts}.json")
    with open(out_json, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults: {out_json}")


if __name__ == "__main__":
    main()
