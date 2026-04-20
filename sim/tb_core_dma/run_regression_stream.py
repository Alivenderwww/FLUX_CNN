"""
run_regression_stream.py -- FLUX CNN streaming-mode 回归测试脚本

v2 streaming 限制：cin_slices=1 && cout_slices=1（即 Cin <= 16 && Cout <= 16）。
"""

import subprocess
import re
import sys
import datetime

OUTPUT_FILE = "regression_stream_report.txt"

# v2 streaming-capable 用例 (Cin<=16 且 Cout<=16)
CASES = [
    # --- 小图 smoke 测试 ---
    ("K=3 C4C4   66x118 s=1",    4,    4, 3,   68,  120,      1,     0),
    ("K=3 C4C8   66x118 s=1",    4,    8, 3,   68,  120,      1,     0),
    ("K=3 C8C4   66x118 s=1",    8,    4, 3,   68,  120,      1,     0),
    ("K=3 C8C8   66x118 s=1",    8,    8, 3,   68,  120,      1,     0),
    ("K=3 C16C16 66x118 s=1",   16,   16, 3,   68,  120,      1,     0),
    ("K=7 C8C8   62x114 s=1",    8,    8, 7,   68,  120,      1,     2),
    ("K=7 C16C16 62x114 s=1",   16,   16, 7,   68,  120,      1,     2),
    ("K=5 C16C16 30x58  s=2",   16,   16, 5,   64,  120,      2,     1),
    # --- 大图 (v2 streaming 主要目标) ---
    ("K=3 C3C8   126x126 s=1",   3,    8, 3,  128,  128,      1,     1),
    ("K=3 C3C16  238x318 s=1",   3,   16, 3,  240,  320,      1,     1),
    ("K=3 C3C16  478x638 s=1",   3,   16, 3,  480,  640,      1,     2),
    # --- Phase D: 任意 Cin/Cout 多切片 ---
    ("K=3 C32C8  66x118 s=1",   32,    8, 3,   68,  120,      1,     0),  # cin=2 cout=1
    ("K=3 C8C32  66x118 s=1",    8,   32, 3,   68,  120,      1,     0),  # cin=1 cout=2
    ("K=3 C32C32 66x118 s=1",   32,   32, 3,   68,  120,      1,     0),  # cin=2 cout=2
    ("K=3 C64C16 66x118 s=1",   64,   16, 3,   68,  120,      1,     0),  # cin=4 cout=1
    ("K=5 C32C16 30x58  s=2",   32,   16, 5,   64,  120,      2,     1),  # cin=2 stride=2
]


def extract_int(text, keyword):
    for line in text.splitlines():
        if keyword in line:
            nums = re.findall(r'\d+', line)
            if nums:
                return int(nums[-1])
    return 0


def extract_float(text, keyword):
    for line in text.splitlines():
        if keyword in line:
            m = re.search(r'[\d]+\.[\d]+', line)
            if m:
                return float(m.group())
    return 0.0


def run_case(name, c_in, c_out, k, h_in, w_in, stride, shift):
    gen_cmd = (f"python gen_isa_test.py --streaming "
               f"--num_cin {c_in} --num_cout {c_out} --k {k} --h_in {h_in} --w_in {w_in} "
               f"--stride {stride} --shift {shift}")
    r_gen = subprocess.run(gen_cmd, shell=True, capture_output=True, text=True)
    if r_gen.returncode != 0:
        return None, r_gen.stderr.strip()

    r_sim = subprocess.run("vsim -c -do run.tcl",
                           shell=True, capture_output=True, text=True)
    out = r_sim.stdout + r_sim.stderr

    passed     = "0 mismatches" in out or "PASSED" in out
    cycles     = extract_int(out, "Core Active Cycles")
    mac_util   = extract_float(out, "MAC Utilization")

    h_out = w_out = 0
    for line in r_gen.stdout.splitlines():
        m = re.search(r'H_OUT=(\d+),\s*W_OUT=(\d+)', line)
        if m:
            h_out, w_out = int(m.group(1)), int(m.group(2))

    return {
        "name": name, "passed": passed,
        "k": k, "h_in": h_in, "w_in": w_in, "stride": stride,
        "h_out": h_out, "w_out": w_out,
        "cycles": cycles, "mac_util": mac_util,
    }, None


def write_report(results, label=""):
    lines = []
    def L(s=""): lines.append(s)

    SEP = "=" * 90
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Streaming Regression  {label}")
    L(f"  Generated: {now}")
    L(SEP)
    L()
    L(f"  {'Case':<24} {'Result':^6}  {'Output':^10}  {'Cycles':>10}  {'MAC%':>6}")
    L("  " + "-" * 88)

    all_pass = True
    for r in results:
        if r is None:
            continue
        status = "[PASS]" if r["passed"] else "[FAIL]"
        if not r["passed"]:
            all_pass = False
        out_sz = f"{r['h_out']}x{r['w_out']}"
        L(f"  {r['name']:<24} {status:^6}  {out_sz:^10}  {r['cycles']:>10,}  "
          f"{r['mac_util']:>5.2f}%")
    L("  " + "-" * 88)
    L(f"  Overall: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    L()
    L(SEP)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return all_pass


def main():
    import argparse
    global OUTPUT_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="", help="Report label")
    parser.add_argument("--out", default=OUTPUT_FILE, help="Output file")
    args = parser.parse_args()
    OUTPUT_FILE = args.out

    print("=" * 60)
    print(f"  FLUX CNN Streaming Regression  {args.label}")
    print("=" * 60)

    results = []
    for i, case in enumerate(CASES):
        name = case[0]
        print(f"\n[{i+1}/{len(CASES)}] {name} ...", end="", flush=True)
        result, err = run_case(*case)
        if err:
            print(f"  ERROR: {err}")
            results.append(None)
        else:
            status = "PASS" if result["passed"] else "FAIL"
            print(f"  {status}  cycles={result['cycles']:,}  "
                  f"MAC_util={result['mac_util']:.2f}%")
            results.append(result)

    valid = [r for r in results if r is not None]
    all_pass = write_report(valid, label=args.label)

    print(f"\n报告已写入: {OUTPUT_FILE}")
    print(f"总体结论  : {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
