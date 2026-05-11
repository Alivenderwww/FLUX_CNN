"""
v4_sweep_ac.py  --  v4 paper §5.5.5 实验 A + C 扫频 (单核 tb_core_dma)

实验 A: Cin ∈ {1, 2, 4, 8, 16, 32, 64} × {fold off, fold on} = 14 case
实验 C: K   ∈ {1, 3, 5, 7} (stride=1) + {K=8 stride=4 +s2d} = 5 case

合计 19 case 单 vsim batch 跑完, 输出 v4_sweep_ac_<ts>.txt 报告.

复用 run_regression.py 的内部函数 (CASES list 替换). chain 内全 root case (互相独立).

用法:
  cd toolchain
  ../toolchain/.venv/Scripts/python.exe v4_sweep_ac.py
"""
import datetime
import os
import shutil
import subprocess
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

import run_regression as RR  # noqa: E402


def build_v4_cases():
    """返回 [(case_dict, opt_tag, want_ky, want_s2d)] 列表.

    case_dict 跟 run_regression.CASES item 同结构 (root case, input_src='', shortcut_src='').
    want_ky / want_s2d 是 per-case override (run_regression 的 ky_fold_global / s2d_global
    是统一开关, 我们这里要细粒度: 实验 A 需要每个 Cin 跑两次, 一次 fold off 一次 fold on).
    """
    cases = []

    # ---- 实验 A: Cin × Ky-fold ----
    # K=3, stride=1, pad=1, H=W=32, Cout=16
    for cin in [1, 2, 4, 8, 16, 32, 64]:
        for fold_on in [False, True]:
            tag = "ky" if fold_on else "off"
            cases.append((
                {
                    'name'        : f'A.cin{cin:02d}_{tag}',
                    'input_src'   : '',
                    'shortcut_src': '',
                    'c_in': cin, 'c_out': 16, 'k': 3,
                    'h_in': 32, 'w_in': 32,
                    'stride': 1, 'pad': 1,
                    'sdp_mult': 1, 'sdp_shift': 4, 'sdp_zp_out': 0,
                    'sdp_clip_min': 0, 'sdp_clip_max': 127,
                    'sdp_round_en': 1, 'sdp_relu_en': 1,
                },
                'ky' if fold_on else '-',
                fold_on,    # want_ky
                False,      # want_s2d
            ))

    # ---- 实验 C: K 扫频 ----
    # K=1/3/5/7 stride=1; K=8 stride=4 +s2d
    for k in [1, 3, 5, 7]:
        pad = (k - 1) // 2 if k > 1 else 0
        cases.append((
            {
                'name'        : f'C.k{k}',
                'input_src'   : '',
                'shortcut_src': '',
                'c_in': 16, 'c_out': 16, 'k': k,
                'h_in': 32, 'w_in': 32,
                'stride': 1, 'pad': pad,
                'sdp_mult': 1, 'sdp_shift': 4, 'sdp_zp_out': 0,
                'sdp_clip_min': 0, 'sdp_clip_max': 127,
                'sdp_round_en': 1, 'sdp_relu_en': 1,
            },
            '-',
            False, False,
        ))
    # K=8 stride=4 + s2d (Patch 等价对照点)
    cases.append((
        {
            'name'        : 'C.k8_s2d',
            'input_src'   : '',
            'shortcut_src': '',
            'c_in': 4, 'c_out': 16, 'k': 8,
            'h_in': 32, 'w_in': 32,
            'stride': 4, 'pad': 0,
            'sdp_mult': 1, 'sdp_shift': 4, 'sdp_zp_out': 0,
            'sdp_clip_min': 0, 'sdp_clip_max': 127,
            'sdp_round_en': 1, 'sdp_relu_en': 1,
        },
        's2d',
        False, True,
    ))

    return cases


def gen_per_case(cases_with_opt, plan):
    """跟 run_regression.gen_chained_cases 同功能但支持 per-case ky/s2d override.

    cases_with_opt: list[(case_dict, opt_tag, want_ky, want_s2d)]
    """
    import gen_isa_test
    from run_regression import SIM_DIR

    case_meta = {}
    for i, (c, opt_tag, want_ky, want_s2d) in enumerate(cases_with_opt):
        case_dir = os.path.join(SIM_DIR, "cases", f"case{i:02d}")
        os.makedirs(case_dir, exist_ok=True)

        bases = plan[i]
        try:
            ret = gen_isa_test.generate_random(
                H_IN=c['h_in'], W_IN=c['w_in'], K=c['k'],
                NUM_CIN=c['c_in'], NUM_COUT=c['c_out'],
                TILE_W=32, seed=42 + i,
                shift_amt=c['sdp_shift'], stride=c['stride'],
                HW_PE=16, HW_COL=16,
                streaming=True,
                pad_top=c['pad'], pad_left=c['pad'],
                strip_rows=0,
                out_dir=case_dir,
                case_name=f"{c['name'].replace(' ', '_')}|conv",
                ky_fold=want_ky, s2d=want_s2d,
                residual=False,
                ifm_arr_in=None, shortcut_arr_in=None,
                sdp_mult     =c['sdp_mult'],
                sdp_zp_out   =c['sdp_zp_out'],
                sdp_clip_min =c['sdp_clip_min'],
                sdp_clip_max =c['sdp_clip_max'],
                sdp_round_en =c['sdp_round_en'],
                sdp_relu_en  =c['sdp_relu_en'],
                shortcut_mult =0, shortcut_shift=0,
                ddr_ifb_base =bases['ifb_base'],
                ddr_wb_base  =bases['wb_base'],
                ddr_ofb_base =bases['ofb_base'],
                ddr_desc_base=bases['desc_base'],
                ddr_rdma_base=bases['rdma_base'],
                skip_ifb_preload=False,
                skip_ofb_clear  =False,
                ifb_sram_words_override=8192,
                ofb_sram_words_override=2048,
            )
        except (ValueError, AssertionError, SystemExit) as e:
            print(f"  case {i} ({c['name']}) gen ERROR: {e}")
            sys.exit(1)
        case_meta[i] = {
            'case_dir': case_dir,
            'opt_tag' : opt_tag,
            'h_out'   : ret['H_OUT'],
            'w_out'   : ret['W_OUT'],
        }
        print(f"  [{i+1}/{len(cases_with_opt)}] {c['name']:<20} → "
              f"{ret['H_OUT']}x{ret['W_OUT']}x{c['c_out']:<4} [{opt_tag}]  "
              f"ifb={bases['ifb_base']:#08x}")
    return case_meta


def main():
    print("=" * 70)
    print("  FLUX CNN v4 §5.5.5 sweep  (实验 A + 实验 C)")
    print("=" * 70)

    cases_with_opt = build_v4_cases()
    cases = [t[0] for t in cases_with_opt]
    n = len(cases)

    print(f"\n[Step 0] 校验 {n} cases ...")
    RR.validate_chain(cases)
    print(f"  chain ok")

    print(f"\n[Step 1] 分配 DDR ...")
    plan = RR.plan_chain_ddr(cases)

    print(f"\n[Step 2] 生成 {n} case 数据 ...")
    case_meta = gen_per_case(cases_with_opt, plan)
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
    log_path = os.path.join(RR.SIM_DIR, "v4_sweep_ac_vsim.log")
    with open(log_path, "w", encoding="utf-8", errors="replace") as f:
        f.write(out)

    print(f"\n[Step 4] parse log ...")
    results = RR.parse_sim_log(out)
    print(f"  parsed {len(results)} CASE_RESULT (expected {n})")

    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    RR.OUTPUT_FILE = os.path.join(RR.SIM_DIR, f"v4_sweep_ac_report_{ts}.txt")
    all_pass = RR.write_report(results, case_opts=case_opts, label="(v4 §5.5.5 A+C)")

    print(f"\n报告: {RR.OUTPUT_FILE}")
    print(f"日志: {log_path}")
    print(f"结论: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
