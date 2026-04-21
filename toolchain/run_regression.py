"""
run_regression.py -- FLUX CNN 统一回归测试脚本 (batch + streaming)

用法：
    python run_regression.py [--label LABEL] [--out FILE]
                             [--only batch|stream|all]   (默认 all)

输出：
    regression_report.txt  单份报告含两种模式所有用例，附握手 / SRAM / ARF / PARF
    perf 指标。
"""

import subprocess
import re
import os
import sys
import shutil
import datetime
import argparse

# Path: 脚本在 toolchain/，仿真在 sim/tb_core_dma/
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR     = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))
GEN_SCRIPT  = os.path.join(_SCRIPT_DIR, "gen_isa_test.py")

# 优先用 venv 里的 python（如果存在），否则当前解释器
_VENV_PY_WIN = os.path.join(_SCRIPT_DIR, ".venv", "Scripts", "python.exe")
_VENV_PY_NIX = os.path.join(_SCRIPT_DIR, ".venv", "bin", "python")
if   os.path.exists(_VENV_PY_WIN): PY = _VENV_PY_WIN
elif os.path.exists(_VENV_PY_NIX): PY = _VENV_PY_NIX
else:                              PY = sys.executable

# Report 输出到 sim dir（和生成的 .txt 放一起）
OUTPUT_FILE = os.path.join(SIM_DIR, "regression_report.txt")

# ---------------------------------------------------------------------------
# 测试用例定义
#   (name, mode, c_in, c_out, k, h_in, w_in, stride, shift, pad)
#   mode: 'batch' 一次性搬完后核跑；'stream' row-ring 并行
#   pad : symmetric pad (top=bot=left=right)；0 = no pad
# ---------------------------------------------------------------------------
CASES = [
    # --- Batch 模式 (基础 48x48) ---------------------------------------
    ("K=3 C4C4   48x48 s=1",    "batch",  4,  4, 3, 48, 48, 1, 0, 0),
    ("K=3 C4C8   48x48 s=1",    "batch",  4,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C4   48x48 s=1",    "batch",  8,  4, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C8   48x48 s=1",    "batch",  8,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C32  48x48 s=1",    "batch",  8, 32, 3, 48, 48, 1, 0, 0),
    ("K=3 C16C16 48x48 s=1",    "batch", 16, 16, 3, 48, 48, 1, 0, 0),
    ("K=3 C32C8  48x48 s=1",    "batch", 32,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C32C16 48x48 s=1",    "batch", 32, 16, 3, 48, 48, 1, 0, 0),
    ("K=3 C32C32 48x48 s=1",    "batch", 32, 32, 3, 48, 48, 1, 0, 0),
    # ("K=3 C64C16 48x48 s=1",    "batch", 64, 16, 3, 48, 48, 1, 0, 0),   # IFB 9216 > SRAM 8192，只能 stream
    ("K=7 C8C8   48x48 s=1",    "batch",  8,  8, 7, 48, 48, 1, 2, 0),
    ("K=7 C16C16 48x48 s=1",    "batch", 16, 16, 7, 48, 48, 1, 2, 0),
    ("K=5 C16C16 48x48 s=2",    "batch", 16, 16, 5, 48, 48, 2, 1, 0),
    ("K=5 C32C16 48x48 s=2",    "batch", 32, 16, 5, 48, 48, 2, 1, 0),
    # --- Batch 模式 (padding: same-conv 保持尺寸) ----------------------
    ("K=3 C8C8   48x48 s=1 p1", "batch",  8,  8, 3, 48, 48, 1, 0, 1),
    ("K=3 C16C16 48x48 s=1 p1", "batch", 16, 16, 3, 48, 48, 1, 0, 1),
    ("K=5 C16C16 48x48 s=1 p2", "batch", 16, 16, 5, 48, 48, 1, 1, 2),
    ("K=7 C16C16 48x48 s=1 p3", "batch", 16, 16, 7, 48, 48, 1, 2, 3),

    # --- Streaming 模式（基础 48x48） -----------------------------------
    ("K=3 C4C4   48x48 s=1",    "stream",  4,  4, 3, 48, 48, 1, 0, 0),
    ("K=3 C4C8   48x48 s=1",    "stream",  4,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C4   48x48 s=1",    "stream",  8,  4, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C8   48x48 s=1",    "stream",  8,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C16C16 48x48 s=1",    "stream", 16, 16, 3, 48, 48, 1, 0, 0),
    ("K=7 C8C8   48x48 s=1",    "stream",  8,  8, 7, 48, 48, 1, 2, 0),
    ("K=7 C16C16 48x48 s=1",    "stream", 16, 16, 7, 48, 48, 1, 2, 0),
    ("K=5 C16C16 48x48 s=2",    "stream", 16, 16, 5, 48, 48, 2, 1, 0),
    # --- Streaming 模式 (padding) ---------------------------------------
    ("K=3 C8C8   48x48 s=1 p1", "stream",  8,  8, 3, 48, 48, 1, 0, 1),
    ("K=3 C16C16 48x48 s=1 p1", "stream", 16, 16, 3, 48, 48, 1, 0, 1),
    ("K=5 C16C16 48x48 s=1 p2", "stream", 16, 16, 5, 48, 48, 1, 1, 2),
    ("K=7 C16C16 48x48 s=1 p3", "stream", 16, 16, 7, 48, 48, 1, 2, 3),
    # --- Streaming 模式 (多 slice) --------------------------------------
    ("K=3 C32C8  48x48 s=1",    "stream", 32,  8, 3, 48, 48, 1, 0, 0),
    ("K=3 C8C32  48x48 s=1",    "stream",  8, 32, 3, 48, 48, 1, 0, 0),
    ("K=3 C32C32 48x48 s=1",    "stream", 32, 32, 3, 48, 48, 1, 0, 0),
    ("K=3 C64C16 48x48 s=1",    "stream", 64, 16, 3, 48, 48, 1, 0, 0),
    ("K=5 C32C16 48x48 s=2",    "stream", 32, 16, 5, 48, 48, 2, 1, 0),
    # --- Streaming 模式 (大图 VGA，确认流式可扩展) -----------------------
    ("K=3 C3C16  478x638 s=1",  "stream",  3, 16, 3, 480, 640, 1, 2, 0),
]


# ---------------------------------------------------------------------------
# Phase G 多层模型回归：("<model_name>", "model", <n_layers>)
# 模型定义在 compile_model.py 的 MODELS dict；这里只声明名称+层数供 case 展开
# ---------------------------------------------------------------------------
MODEL_CASES = [
    ("mnist2",         "model", 2),
    ("mnist_allconv",  "model", 5),
]
CASES += MODEL_CASES


def flatten_cases(cases):
    """把 model case 展成 per-layer sub-entries，返回 flat list of dict。"""
    flat = []
    for c in cases:
        if len(c) == 3 and c[1] == "model":
            model_name, _, n_layers = c
            for k in range(n_layers):
                flat.append({
                    'kind'      : 'model_layer',
                    'model_name': model_name,
                    'n_layers'  : n_layers,
                    'layer_idx' : k,
                    'name'      : f"{model_name}/L{k}",
                    'mode'      : 'model',
                })
        else:
            flat.append({
                'kind': 'single',
                'args': c,
                'name': c[0],
                'mode': c[1],
            })
    return flat


# ---------------------------------------------------------------------------
# 生成单个 case 数据到 cases/caseNN/
# ---------------------------------------------------------------------------
def gen_case_files(case_idx, name, mode, c_in, c_out, k, h_in, w_in, stride, shift, pad):
    case_dir = os.path.join(SIM_DIR, "cases", f"case{case_idx:02d}")
    os.makedirs(case_dir, exist_ok=True)
    streaming_flag = "--streaming" if mode == "stream" else ""
    gen_cmd = (f"\"{PY}\" \"{GEN_SCRIPT}\" {streaming_flag} "
               f"--num_cin {c_in} --num_cout {c_out} --k {k} "
               f"--h_in {h_in} --w_in {w_in} --stride {stride} "
               f"--shift {shift} --pad {pad} "
               f"--out-dir \"{case_dir}\" "
               # case-name 里不能有空格（TB $sscanf %s 会在空格停）；用 _ 替代
               f"--case-name \"{name.replace(' ', '_')}|{mode}\"")
    r = subprocess.run(gen_cmd, shell=True, capture_output=True, text=True)
    if r.returncode != 0:
        return None, r.stderr.strip() or r.stdout.strip()
    h_out = w_out = 0
    for line in r.stdout.splitlines():
        m = re.search(r'H_OUT=(\d+),\s*W_OUT=(\d+)', line)
        if m:
            h_out, w_out = int(m.group(1)), int(m.group(2))
    return {"name": name, "mode": mode, "h_out": h_out, "w_out": w_out,
            "case_dir": case_dir}, None


def gen_model_cases(start_idx, model_name, n_layers):
    """subprocess 调 compile_model（用 venv python，主进程无需 torch）。
    解析 plan_dir/model_plan.json 返回每层 meta."""
    plan_dir = os.path.join(SIM_DIR, "cases", f"{model_name}_plan")
    os.makedirs(plan_dir, exist_ok=True)
    cmd = (f'"{PY}" "{os.path.join(_SCRIPT_DIR, "compile_model.py")}" '
           f'--model {model_name} --sim-dir "{SIM_DIR}" '
           f'--start-idx {start_idx} --out-dir "{plan_dir}"')
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip() or r.stdout.strip())
    import json
    with open(os.path.join(plan_dir, 'model_plan.json')) as f:
        plan = json.load(f)
    if plan['n_layers'] != n_layers:
        raise ValueError(f"model {model_name} plan has {plan['n_layers']} layers ≠ {n_layers}")
    return [
        {"name": f"{model_name}/L{layer['idx']}", "mode": "model",
         "h_out": layer['H_OUT'], "w_out": layer['W_OUT'],
         "case_dir": os.path.join(SIM_DIR, "cases", f"case{start_idx + layer['idx']:02d}")}
        for layer in plan['layers']
    ]


# ---------------------------------------------------------------------------
# 解析单次 vsim 日志里的所有 CASE_RESULT 行
#   "CASE_RESULT N PASS cycles=X mac_fire=Y mac_util=Z% arf_w=.. ..."
# ---------------------------------------------------------------------------
CASE_RE = re.compile(
    r"CASE_RESULT\s+(\d+)\s+(PASS|FAIL)\s+cycles=(\d+)"
    r"(?:\s+mac_fire=(\d+))?"
    r"(?:\s+mac_util=([\d.]+)%)?"
    r"(?:\s+arf_w=(\d+))?"
    r"(?:\s+arf_r=(\d+))?"
    r"(?:\s+parf_f=(\d+))?"
    r"(?:\s+parf_d=(\d+))?"
    r"(?:\s+ifb_r=(\d+))?"
    r"(?:\s+wb_r=(\d+))?"
    r"(?:\s+ofb_w=(\d+))?"
    r"(?:\s+mismatches=(\d+))?"
    r"\s+name=([^\n]+)")


def parse_sim_log(text):
    """返回 list of dict（按 case index 排序）"""
    results = {}
    for line in text.splitlines():
        m = CASE_RE.search(line)
        if not m:
            continue
        idx = int(m.group(1))
        results[idx] = {
            "idx":        idx,
            "passed":     m.group(2) == "PASS",
            "cycles":     int(m.group(3)),
            "mac_fire":   int(m.group(4) or 0),
            "mac_util":   float(m.group(5) or 0.0),
            "arf_w":      int(m.group(6) or 0),
            "arf_r":      int(m.group(7) or 0),
            "parf_f":     int(m.group(8) or 0),
            "parf_d":     int(m.group(9) or 0),
            "ifb_r":      int(m.group(10) or 0),
            "wb_r":       int(m.group(11) or 0),
            "ofb_w":      int(m.group(12) or 0),
            "name":       m.group(14).strip(),
            "mismatches": int(m.group(13) or 0),
        }
    return [results[i] for i in sorted(results.keys())]


# ---------------------------------------------------------------------------
# 报告生成
# ---------------------------------------------------------------------------
def write_report(results, label=""):
    lines = []
    def L(s=""): lines.append(s)

    SEP = "=" * 140
    SUB = "-" * 140
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Regression (batch + streaming)  {label}")
    L(f"  Generated: {now}")
    L(SEP)
    L()

    # ---- Summary 表 ----
    hdr = (f"  {'Case':<27} {'Mode':^6} {'Res':^5} {'Cycles':>10} {'MAC%':>6}  "
           f"{'IFB_R':>8} {'WB_R':>6} {'OFB_W':>7}  "
           f"{'ARF_W':>8} {'ARF_R':>8} {'Ratio':>6}  "
           f"{'PARF_F':>8} {'PARF_D':>7}")
    L(hdr)
    L(SUB)
    all_pass = True
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        if not r["passed"]:
            all_pass = False
        # name 格式 "K=3 C8C8 48x48 s=1|batch" —— 拆 mode
        if "|" in r["name"]:
            cname, mode = r["name"].rsplit("|", 1)
        else:
            cname, mode = r["name"], "?"
        ratio = (r['arf_r'] / r['arf_w']) if r['arf_w'] > 0 else 0.0
        row = (f"  {cname:<27} {mode:^6} {status:^5} {r['cycles']:>10,} "
               f"{r['mac_util']:>5.1f}%  "
               f"{r['ifb_r']:>8,} {'-':>6} {r['ofb_w']:>7,}  "
               f"{r['arf_w']:>8,} {r['arf_r']:>8,} {ratio:>5.2f}x  "
               f"{r['parf_f']:>8,} {r['parf_d']:>7,}")
        L(row)
    L(SUB)
    L("  MAC%: 硬件利用率 = min(Cin,16)×min(Cout,16)/(16×16)；")
    L("  ARF_W=ARF写(=IFB读扣除 pad)；ARF_R=ARF读到 MAC；Ratio=ARF_R/ARF_W (>1 表示 kx 滑窗复用)")
    L("  IFB_R/OFB_W=SRAM 访问次数；PARF_F/PARF_D=psum 累加写/读")
    L()

    L(SEP)
    L(f"  Overall: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}  ({len(results)} cases)")
    L(SEP)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return all_pass


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def main():
    global OUTPUT_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="", help="Report label")
    parser.add_argument("--out",   default=OUTPUT_FILE, help="Output file")
    parser.add_argument("--only",  default="all",
                        choices=["batch", "stream", "model", "all"],
                        help="Filter cases: batch/stream/model/all (default: all)")
    parser.add_argument("--case",  default=None,
                        help="只跑 name 包含此子串的 case（大小写敏感）。例: --case C16C10")
    args = parser.parse_args()
    OUTPUT_FILE = args.out

    cases = [c for c in CASES if args.only == "all" or c[1] == args.only]
    if args.case is not None:
        cases = [c for c in cases if args.case in c[0]]
        if not cases:
            print(f"  no case name contains '{args.case}'; aborted.")
            sys.exit(1)
    flat = flatten_cases(cases)
    n_cases = len(flat)

    print("=" * 60)
    print(f"  FLUX CNN Regression  {args.label}")
    print(f"  Mode filter: {args.only}  ({n_cases} cases, including model sub-layers)")
    print("=" * 60)

    # Step 1: 按 flat 列表生成 case 数据到 cases/caseNN/
    #   single case: 一个 gen_case_files 调用
    #   model case : 展开为多个 sub-case，compile_model 一次性产出全部层
    print(f"\n[Step 1] 生成 {n_cases} 个 case 数据 ...")
    i = 0
    while i < n_cases:
        item = flat[i]
        if item['kind'] == 'single':
            gen_info, err = gen_case_files(i, *item['args'])
            if err:
                print(f"  case {i} ({item['name']}) gen ERROR: {err}")
                sys.exit(1)
            print(f"  [{i+1}/{n_cases}] {item['name']}  → H_OUT={gen_info['h_out']} W_OUT={gen_info['w_out']}")
            i += 1
        else:  # model_layer
            assert item['layer_idx'] == 0, "flatten 保证 model 第一子层 layer_idx=0"
            L = item['n_layers']
            try:
                infos = gen_model_cases(i, item['model_name'], L)
            except Exception as e:
                print(f"  model {item['model_name']} gen ERROR: {e}")
                sys.exit(1)
            for k, info in enumerate(infos):
                print(f"  [{i+k+1}/{n_cases}] {info['name']}  → H_OUT={info['h_out']} W_OUT={info['w_out']}")
            i += L

    # 把 case0 的 sim_params.f 复制到 SIM_DIR/ 给 vsim 启动参数用（-gSRAM_DEPTH，-sva 等）
    # 再 append +N_CASES 给 TB plusarg 用
    src = os.path.join(SIM_DIR, "cases", "case00", "sim_params.f")
    dst = os.path.join(SIM_DIR, "sim_params.f")
    shutil.copy(src, dst)
    with open(dst, "a", encoding="utf-8") as f:
        f.write(f"+N_CASES={n_cases}\n")

    # Step 2: 单次 vsim 跑全部 case
    print(f"\n[Step 2] 启动单次 vsim (N_CASES={n_cases})，核心复用多次 start/done ...")
    r_sim = subprocess.run("vsim -c -do run.tcl", shell=True,
                           capture_output=True, text=True, cwd=SIM_DIR)
    out = r_sim.stdout + r_sim.stderr
    if r_sim.returncode != 0 and "ALL " not in out:
        print("  vsim failed to finish normally.")
        # 仍尝试 parse

    # 把 sim 输出也 dump 一份到 sim 目录，方便 debug
    with open(os.path.join(SIM_DIR, "vsim_all.log"), "w", encoding="utf-8", errors="replace") as f:
        f.write(out)

    # Step 3: parse 日志
    print(f"\n[Step 3] 解析 vsim 日志 ...")
    results = parse_sim_log(out)
    if len(results) != n_cases:
        print(f"  WARNING: expected {n_cases} CASE_RESULT, got {len(results)}")
        missing = set(range(n_cases)) - set(r['idx'] for r in results)
        if missing:
            print(f"  missing case idx: {sorted(missing)}")

    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        print(f"  [{r['idx']:>2}] {status}  cycles={r['cycles']:>9,}  "
              f"MAC={r['mac_util']:>5.1f}%  {r['name']}")

    all_pass = write_report(results, label=args.label)

    print(f"\n报告已写入: {OUTPUT_FILE}")
    print(f"总体结论  : {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
