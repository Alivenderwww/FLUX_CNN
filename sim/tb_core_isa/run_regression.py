"""
run_regression.py -- FLUX CNN 加速器回归测试脚本

用法：
    python run_regression.py

输出：
    - 终端实时进度
    - regression_report.txt（原始指标数据表）
"""

import subprocess
import re
import sys
import datetime

OUTPUT_FILE = "regression_report.txt"

# ---------------------------------------------------------------------------
# 标准测试用例定义
# (名称, k, h_in, w_in, stride, shift)
# ---------------------------------------------------------------------------
CASES = [
    # --- 功能验证用例 ---
    ("K=3  10x10  s=1",   3,   10,   10,   1,  0),
    ("K=3  20x20  s=2",   3,   20,   20,   2,  0),
    ("K=3  20x20  s=3",   3,   20,   20,   3,  1),
    ("K=7  60x45  s=1",   7,   60,   45,   1,  2),
    ("K=7  30x30  s=2",   7,   30,   30,   2,  2),
    ("K=1 123x45  s=1",   1,  123,   45,   1,  0),
    # --- 性能对比用例 ---
    ("K=7 100x40  s=1",   7,  100,   40,   1,  0),
    ("K=7 100x40  s=2",   7,  100,   40,   2,  0),
    ("K=7 100x80  s=1",   7,  100,   80,   1,  0),
    ("K=7 100x80  s=2",   7,  100,   80,   2,  0),
    ("K=7  68x120 s=1",   7,   68,  120,   1,  0),
    ("K=7  68x120 s=2",   7,   68,  120,   2,  0),
]

# ---------------------------------------------------------------------------
# 辅助：解析仿真输出
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# 运行单个用例
# ---------------------------------------------------------------------------
def run_case(name, k, h_in, w_in, stride, shift):
    gen_cmd = (f"python gen_isa_test.py "
               f"--k {k} --h_in {h_in} --w_in {w_in} "
               f"--stride {stride} --shift {shift}")
    r_gen = subprocess.run(gen_cmd, shell=True, capture_output=True, text=True)
    if r_gen.returncode != 0:
        return None, r_gen.stderr.strip()

    r_sim = subprocess.run("vsim -c -do tb_core_isa.tcl",
                           shell=True, capture_output=True, text=True)
    out = r_sim.stdout + r_sim.stderr

    passed     = "0 mismatches" in out
    cycles     = extract_int(out,   "Total Active Cycles")
    mac_util   = extract_float(out, "MAC Utilization Rate")
    arf_writes = extract_int(out,   "ARF  Writes (SRAM Load)")
    arf_reads  = extract_int(out,   "ARF  Reads  (To MAC)")
    arf_ratio  = (arf_reads / arf_writes) if arf_writes > 0 else 0.0

    h_out = w_out = inst_cnt = 0
    for line in r_gen.stdout.splitlines():
        m = re.search(r'H_OUT=(\d+),\s*W_OUT=(\d+)', line)
        if m:
            h_out, w_out = int(m.group(1)), int(m.group(2))
        m2 = re.search(r'Instructions\s*:\s*(\d+)', line)
        if m2:
            inst_cnt = int(m2.group(1))

    return {
        "name": name, "passed": passed,
        "k": k, "h_in": h_in, "w_in": w_in, "stride": stride,
        "h_out": h_out, "w_out": w_out, "inst_cnt": inst_cnt,
        "cycles": cycles, "mac_util": mac_util,
        "arf_writes": arf_writes, "arf_reads": arf_reads,
        "arf_ratio": arf_ratio,
    }, None

# ---------------------------------------------------------------------------
# 报告写入
# ---------------------------------------------------------------------------
def write_report(results, label=""):
    lines = []
    def L(s=""): lines.append(s)

    W   = 110
    SEP = "=" * W

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Accelerator -- Regression Report  {label}")
    L(f"  Generated: {now}")
    L(SEP)
    L()

    hdr = (f"  {'Case':<22} {'Result':^6}  {'Output':^9}  {'Cycles':>9}  "
           f"{'ARF_W':>7}  {'ARF_R':>7}  {'Ratio':>6}  {'MAC%':>6}  {'Insts':>5}")
    L(hdr)
    L("  " + "-" * (W - 2))

    all_pass = True
    for r in results:
        if r is None:
            continue
        status = "[PASS]" if r["passed"] else "[FAIL]"
        if not r["passed"]:
            all_pass = False
        out_sz = f"{r['h_out']}x{r['w_out']}"
        row = (f"  {r['name']:<22} {status:^6}  {out_sz:^9}  {r['cycles']:>9,}  "
               f"{r['arf_writes']:>7,}  {r['arf_reads']:>7,}  {r['arf_ratio']:>5.2f}x  "
               f"{r['mac_util']:>5.2f}%  {r['inst_cnt']:>5}")
        L(row)

    L("  " + "-" * (W - 2))
    L(f"  ARF_W=ARF写次数(=IFB读)  ARF_R=ARF读到MAC  Ratio=ARF_R/ARF_W(>1表示滑窗复用)")
    L(f"  Overall: {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    L()
    L(SEP)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return all_pass

# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def main():
    import argparse
    global OUTPUT_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="", help="Report label (e.g. 'baseline' or 'pipeline')")
    parser.add_argument("--out", default=OUTPUT_FILE, help="Output file")
    args = parser.parse_args()

    OUTPUT_FILE = args.out

    print("=" * 60)
    print(f"  FLUX CNN Regression  {args.label}")
    print("=" * 60)

    results = []
    for i, (name, k, h_in, w_in, stride, shift) in enumerate(CASES):
        print(f"\n[{i+1}/{len(CASES)}] {name} ...", end="", flush=True)
        result, err = run_case(name, k, h_in, w_in, stride, shift)
        if err:
            print(f"  ERROR: {err}")
            results.append(None)
        else:
            status = "PASS" if result["passed"] else "FAIL"
            print(f"  {status}  cycles={result['cycles']:,}  "
                  f"ARF_ratio={result['arf_ratio']:.2f}x  "
                  f"MAC_util={result['mac_util']:.2f}%")
            results.append(result)

    valid = [r for r in results if r is not None]
    all_pass = write_report(valid, label=args.label)

    print(f"\n报告已写入: {OUTPUT_FILE}")
    print(f"总体结论  : {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)

if __name__ == "__main__":
    main()
