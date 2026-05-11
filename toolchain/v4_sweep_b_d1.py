"""
v4_sweep_b_d1.py  --  v4 paper §5.5.5 实验 B + 实验 D (N=1) 单核扫频

实验 B (单核基线): K=1 stride={1, 2, 3, 4}, H=W=64, Cin=Cout=16 = 4 case
  纯 stride 维度对 wall cycles / IDMA cmd 数的影响. H-step 对照引 exp6 数据.

实验 D N=1 基线: K=3 stride=1, Cin=Cout=16, H=W ∈ {16, 32, 64, 128} = 4 case
  单核单层 wall cycles 给 N=4 SMC 对照算加速比.

合计 8 case 单 vsim batch 跑完.
"""
import datetime
import os
import shutil
import subprocess
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

import run_regression as RR  # noqa: E402
import gen_isa_test            # noqa: E402


def build_v4_cases():
    cases = []

    # ---- 实验 B 基线: K x stride 扫频 (ResNet 经典 4 组) ----
    # H=W=64, Cin=Cout=16
    # K=1 stride=3/4 单核 sim 不支持 (line_buffer K<stride 路径未实现) — 跳过
    b_specs = [
        # (k, stride, pad)
        (1, 1, 0),    # K=1 stride=1 (1x1 conv, 计算密集型)
        (1, 2, 0),    # K=1 stride=2 (ResNet ds layer)
        (3, 1, 1),    # K=3 stride=1 (ResNet 主 conv)
        (3, 2, 1),    # K=3 stride=2 (ResNet ds layer 主路径)
    ]
    for k, s, pad in b_specs:
        cases.append({
            'name'        : f'B.k{k}s{s}',
            'input_src'   : '',
            'shortcut_src': '',
            'c_in': 16, 'c_out': 16, 'k': k,
            'h_in': 64, 'w_in': 64,
            'stride': s, 'pad': pad,
            'sdp_mult': 1, 'sdp_shift': 4, 'sdp_zp_out': 0,
            'sdp_clip_min': 0, 'sdp_clip_max': 127,
            'sdp_round_en': 1, 'sdp_relu_en': 1,
        })

    # ---- 实验 D N=1 基线: 不同 W 单核 wall cycles ----
    # K=3 stride=1 pad=1, Cin=Cout=16, 方阵 H=W
    for w in [16, 32, 64, 128]:
        cases.append({
            'name'        : f'D.n1_w{w:03d}',
            'input_src'   : '',
            'shortcut_src': '',
            'c_in': 16, 'c_out': 16, 'k': 3,
            'h_in': w, 'w_in': w,
            'stride': 1, 'pad': 1,
            'sdp_mult': 1, 'sdp_shift': 4, 'sdp_zp_out': 0,
            'sdp_clip_min': 0, 'sdp_clip_max': 127,
            'sdp_round_en': 1, 'sdp_relu_en': 1,
        })

    return cases


def gen_per_case(cases, plan):
    case_meta = {}
    from run_regression import SIM_DIR
    for i, c in enumerate(cases):
        case_dir = os.path.join(SIM_DIR, "cases", f"case{i:02d}")
        os.makedirs(case_dir, exist_ok=True)
        bases = plan[i]
        try:
            ret = gen_isa_test.generate_random(
                H_IN=c['h_in'], W_IN=c['w_in'], K=c['k'],
                NUM_CIN=c['c_in'], NUM_COUT=c['c_out'],
                TILE_W=32, seed=42 + i,
                shift_amt=c['sdp_shift'], stride=c['stride'],
                HW_PE=16, HW_COL=16, streaming=True,
                pad_top=c['pad'], pad_left=c['pad'],
                strip_rows=0, out_dir=case_dir,
                case_name=f"{c['name'].replace(' ', '_')}|conv",
                ky_fold=False, s2d=False, residual=False,
                ifm_arr_in=None, shortcut_arr_in=None,
                sdp_mult=c['sdp_mult'], sdp_zp_out=c['sdp_zp_out'],
                sdp_clip_min=c['sdp_clip_min'], sdp_clip_max=c['sdp_clip_max'],
                sdp_round_en=c['sdp_round_en'], sdp_relu_en=c['sdp_relu_en'],
                shortcut_mult=0, shortcut_shift=0,
                ddr_ifb_base=bases['ifb_base'], ddr_wb_base=bases['wb_base'],
                ddr_ofb_base=bases['ofb_base'], ddr_desc_base=bases['desc_base'],
                ddr_rdma_base=bases['rdma_base'],
                skip_ifb_preload=False, skip_ofb_clear=False,
                ifb_sram_words_override=8192, ofb_sram_words_override=2048,
            )
        except (ValueError, AssertionError, SystemExit) as e:
            print(f"  case {i} ({c['name']}) gen ERROR: {e}")
            sys.exit(1)
        case_meta[i] = {
            'case_dir': case_dir, 'opt_tag': '-',
            'h_out': ret['H_OUT'], 'w_out': ret['W_OUT'],
        }
        print(f"  [{i+1}/{len(cases)}] {c['name']:<22} → "
              f"{ret['H_OUT']}x{ret['W_OUT']}x{c['c_out']:<4}  "
              f"ifb={bases['ifb_base']:#08x}")
    return case_meta


def main():
    print("=" * 70)
    print("  FLUX CNN v4 §5.5.5 sweep  (实验 B baseline + 实验 D N=1)")
    print("=" * 70)

    cases = build_v4_cases()
    n = len(cases)
    print(f"\n[Step 0] 校验 {n} cases ...")
    RR.validate_chain(cases)

    print(f"\n[Step 1] 分配 DDR ...")
    plan = RR.plan_chain_ddr(cases)

    print(f"\n[Step 2] 生成 {n} case 数据 ...")
    case_meta = gen_per_case(cases, plan)
    case_opts = {i: case_meta[i]['opt_tag'] for i in case_meta}

    src = os.path.join(RR.SIM_DIR, "cases", "case00", "sim_params.f")
    dst = os.path.join(RR.SIM_DIR, "sim_params.f")
    shutil.copy(src, dst)

    timeout_ns = sum(RR.estimate_case_timeout_ns(c) for c in cases) + 100_000_000
    with open(dst, "a", encoding="utf-8") as f:
        f.write(f"+N_CASES={n}\n")
        f.write(f"+TIMEOUT_NS={timeout_ns}\n")
    print(f"  watchdog timeout = {timeout_ns:,} ns")

    print(f"\n[Step 3] vsim batch run (N={n}) ...")
    r = subprocess.run("vsim -c -do run.tcl", shell=True,
                       capture_output=True, text=True, cwd=RR.SIM_DIR)
    out = r.stdout + r.stderr
    log_path = os.path.join(RR.SIM_DIR, "v4_sweep_b_d1_vsim.log")
    with open(log_path, "w", encoding="utf-8", errors="replace") as f:
        f.write(out)

    print(f"\n[Step 4] parse log ...")
    results = RR.parse_sim_log(out)
    print(f"  parsed {len(results)} CASE_RESULT (expected {n})")

    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    RR.OUTPUT_FILE = os.path.join(RR.SIM_DIR, f"v4_sweep_b_d1_report_{ts}.txt")
    all_pass = RR.write_report(results, case_opts=case_opts, label="(v4 §5.5.5 B+D-N1)")
    print(f"\n报告: {RR.OUTPUT_FILE}")
    print(f"日志: {log_path}")
    print(f"结论: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
