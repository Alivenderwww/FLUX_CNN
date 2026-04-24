"""
run_regression.py -- FLUX CNN 统一回归测试脚本 (batch + streaming)

用法：
    python run_regression.py [--label LABEL] [--out FILE]
                             [--only batch|stream|all]   (默认 all)

输出：
    regression_report.txt  单份报告含两种模式所有用例，附握手 / SRAM / ARF / PARF
    perf 指标。（txt文件在 sim/tb_core_dma/ 下）
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
    # J-1 起 batch/stream 数据路径统一（硬件单一 streaming engine），mode='conv'。
    # 整图装得下 SRAM → strip_rows=H_IN（ring 不 wrap，等价原 batch 串行 latency）；
    # 装不下 → strip 切小（ring-buffer 流式，IDMA/core/ODMA 并发，如 VGA）。
    #
    # ---- ResNet-18-like model (input 960x540) — 每一 conv 层的等效独立 case ----
    # Pool 层硬件不支持，跳过。FC 可视为 1x1 conv。streaming 已支持任意 Cin/Cout。

    # ---- Stem ----
    ("K=7 C4C8    960x540 s2 p3", "conv",   4,   8, 7, 960, 540, 2, 0, 3),  # Conv2d stem            -> 480x270
  # ("MaxPool2d   480x270 s2 p1", "pool",   8,   8, 3, 480, 270, 2, 0, 1),  # MaxPool2d              -> 240x135  [无 pool 硬件]

    # ---- Layer 1: 4 conv, C=8 ----
    ("K=3 C8C8    240x135 s1 p1", "conv",   8,   8, 3, 240, 135, 1, 0, 1),  # L1.B1.Conv1
    ("K=3 C8C8    240x135 s1 p1", "conv",   8,   8, 3, 240, 135, 1, 0, 1),  # L1.B1.Conv2
    ("K=3 C8C8    240x135 s1 p1", "conv",   8,   8, 3, 240, 135, 1, 0, 1),  # L1.B2.Conv1
    ("K=3 C8C8    240x135 s1 p1", "conv",   8,   8, 3, 240, 135, 1, 0, 1),  # L1.B2.Conv2

    # ---- Layer 2: Cin 8->16, Cout 16 ----
    ("K=3 C8C16   240x135 s2 p1", "conv",   8,  16, 3, 240, 135, 2, 0, 1),  # L2.B1.Conv1            -> 120x68
    ("K=3 C16C16  120x68  s1 p1", "conv",  16,  16, 3, 120,  68, 1, 0, 1),  # L2.B1.Conv2
    ("K=1 C8C16   240x135 s2 p0", "conv",   8,  16, 1, 240, 135, 2, 0, 0),  # L2.B1.Downsample (1x1) -> 120x68
    ("K=3 C16C16  120x68  s1 p1", "conv",  16,  16, 3, 120,  68, 1, 0, 1),  # L2.B2.Conv1
    ("K=3 C16C16  120x68  s1 p1", "conv",  16,  16, 3, 120,  68, 1, 0, 1),  # L2.B2.Conv2

    # ---- Layer 3: Cin 16->32, Cout 32 ----
    ("K=3 C16C32  120x68  s2 p1", "conv",  16,  32, 3, 120,  68, 2, 0, 1),  # L3.B1.Conv1            -> 60x34
    ("K=3 C32C32  60x34   s1 p1", "conv",  32,  32, 3,  60,  34, 1, 0, 1),  # L3.B1.Conv2
    ("K=1 C16C32  120x68  s2 p0", "conv",  16,  32, 1, 120,  68, 2, 0, 0),  # L3.B1.Downsample       -> 60x34
    ("K=3 C32C32  60x34   s1 p1", "conv",  32,  32, 3,  60,  34, 1, 0, 1),  # L3.B2.Conv1
    ("K=3 C32C32  60x34   s1 p1", "conv",  32,  32, 3,  60,  34, 1, 0, 1),  # L3.B2.Conv2

    # ---- Layer 4: Cin 32->64, Cout 64 ----
    ("K=3 C32C64  60x34   s2 p1", "conv",  32,  64, 3,  60,  34, 2, 0, 1),  # L4.B1.Conv1            -> 30x17
    ("K=3 C64C64  30x17   s1 p1", "conv",  64,  64, 3,  30,  17, 1, 0, 1),  # L4.B1.Conv2
    ("K=1 C32C64  60x34   s2 p0", "conv",  32,  64, 1,  60,  34, 2, 0, 0),  # L4.B1.Downsample       -> 30x17
    ("K=3 C64C64  30x17   s1 p1", "conv",  64,  64, 3,  30,  17, 1, 0, 1),  # L4.B2.Conv1
    ("K=3 C64C64  30x17   s1 p1", "conv",  64,  64, 3,  30,  17, 1, 0, 1),  # L4.B2.Conv2

    # ---- Head ----
  # ("AvgPool2d   4x4     s8 p0", "pool",  64,  64, 4,   4,   4, 8, 0, 0),  # AvgPool2d  -> 1x1      [无 pool 硬件]
    ("FC1 C256C512 1x1",          "conv", 256, 512, 1,   1,   1, 1, 0, 0),  # FC1 (256->512, 1x1 conv)
    ("FC2 C512C10  1x1",          "conv", 512,  10, 1,   1,   1, 1, 0, 0),  # FC2 (512->10,  1x1 conv)
]


# ---------------------------------------------------------------------------
# 生成单个 case 数据到 cases/caseNN/
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 单 case 超时估算 (ns) -- 用于 run 整份 regression 的 watchdog
#   校准：stem K=7 C4C8 960x540 实测 ~6.35M cycles；这里用 100 cycles/out_pix/tile
#   再乘 10x sim_time/cycle 系数 + 5x 安全余量，留足 DMA 阻塞等 corner case
# ---------------------------------------------------------------------------
def estimate_case_timeout_ns(c_in, c_out, k, h_in, w_in, stride, pad):
    h_out = max(1, (h_in + 2 * pad - k) // stride + 1)
    w_out = max(1, (w_in + 2 * pad - k) // stride + 1)
    cin_tiles  = max(1, (c_in  + 15) // 16)
    cout_tiles = max(1, (c_out + 15) // 16)
    cycles = h_out * w_out * cin_tiles * cout_tiles * 100 + 500_000  # 500K 启停 overhead
    return cycles * 10 * 5  # @10 ns clk, 5x safety margin


def gen_case_files(case_idx, name, mode, c_in, c_out, k, h_in, w_in, stride, shift, pad,
                   ky_fold=False, kx_fold=False):
    case_dir = os.path.join(SIM_DIR, "cases", f"case{case_idx:02d}")
    os.makedirs(case_dir, exist_ok=True)
    # J-1: gen_isa_test.py 的 --streaming 默认 True，无需显式传
    fold_flag = ""
    if ky_fold: fold_flag += " --ky-fold"
    if kx_fold: fold_flag += " --kx-fold"
    gen_cmd = (f"\"{PY}\" \"{GEN_SCRIPT}\" "
               f"--num_cin {c_in} --num_cout {c_out} --k {k} "
               f"--h_in {h_in} --w_in {w_in} --stride {stride} "
               f"--shift {shift} --pad {pad} {fold_flag} "
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
    # 所有计数列统一 11 位宽 (够放 9,999,999 = 9 字符带逗号, 再留 2 余量)
    hdr = (f"  {'Case':<27} {'Mode':^6} {'Res':^5} {'Cycles':>11} {'MAC%':>6}  "
           f"{'IFB_R':>11} {'WB_R':>9} {'OFB_W':>11}  "
           f"{'ARF_W':>11} {'ARF_R':>11} {'Ratio':>6}  "
           f"{'PARF_F':>11} {'PARF_D':>11}")
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
        row = (f"  {cname:<27} {mode:^6} {status:^5} {r['cycles']:>11,} "
               f"{r['mac_util']:>5.1f}%  "
               f"{r['ifb_r']:>11,} {'-':>9} {r['ofb_w']:>11,}  "
               f"{r['arf_w']:>11,} {r['arf_r']:>11,} {ratio:>5.2f}x  "
               f"{r['parf_f']:>11,} {r['parf_d']:>11,}")
        L(row)
    L(SUB)
    L("  MAC%: useful_mac_ops / peak_mac_ops; useful = H_out×W_out×K²×Cin×Cout (pre-fold原始 conv);")
    L("        peak = cycles×NUM_COL×NUM_PE. fold pad + Kx tail partial 都会扣掉利用率。")
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
                        choices=["all"],
                        help="(legacy option, J-1 起只有 conv 一种) default: all")
    parser.add_argument("--case",  default=None,
                        help="只跑 name 包含此子串的 case（大小写敏感）。例: --case C16C10")
    parser.add_argument("--timeout-ns", type=int, default=0,
                        help="vsim watchdog 超时 (ns)；0=按每 case 尺寸自动估算")
    parser.add_argument("--fold", action="store_true",
                        help="对所有 K>1 且 Cin<16 的 case 启用 Ky fold")
    parser.add_argument("--kx-fold", action="store_true",
                        help="对所有 K>1 且 Cout<16 的 case 启用 Kx fold (可与 --fold 叠加)")
    args = parser.parse_args()
    OUTPUT_FILE = args.out

    cases = [c for c in CASES if args.only == "all" or c[1] == args.only]
    if args.case is not None:
        cases = [c for c in cases if args.case in c[0]]
        if not cases:
            print(f"  no case name contains '{args.case}'; aborted.")
            sys.exit(1)
    n_cases = len(cases)

    print("=" * 60)
    print(f"  FLUX CNN Case Regression  {args.label}")
    print(f"  Mode filter: {args.only}  ({n_cases} cases)")
    print("=" * 60)

    # Step 1: 生成每 case 数据到 cases/caseNN/
    print(f"\n[Step 1] 生成 {n_cases} 个 case 数据 ...")
    for i, case in enumerate(cases):
        # case 里 c_in 是第 3 个字段 (idx 2), k 是第 5 个 (idx 4)
        _, _, c_in_v, _, k_v, _, _, _, _, _ = case
        # case 里 c_out 是第 4 个字段 (idx 3)
        _, _, _, c_out_v, _, _, _, _, _, _ = case
        # fold 条件: K>1 且 Cin<16 (Ky); K>1 且 Cout<16 (Kx)
        use_ky = args.fold    and (k_v > 1) and (c_in_v  < 16)
        use_kx = args.kx_fold and (k_v > 1) and (c_out_v < 16)
        gen_info, err = gen_case_files(i, *case, ky_fold=use_ky, kx_fold=use_kx)
        if err:
            print(f"  case {i} ({case[0]}) gen ERROR: {err}")
            sys.exit(1)
        fold_mark = ""
        if use_ky: fold_mark += "[ky]"
        if use_kx: fold_mark += "[kx]"
        print(f"  [{i+1}/{n_cases}] {case[0]}  → H_OUT={gen_info['h_out']} W_OUT={gen_info['w_out']} {fold_mark}")

    # 把 case0 的 sim_params.f 复制到 SIM_DIR/ 给 vsim 启动参数用（-gSRAM_DEPTH，-sva 等）
    # 再 append +N_CASES / +TIMEOUT_NS 给 TB plusarg 用
    src = os.path.join(SIM_DIR, "cases", "case00", "sim_params.f")
    dst = os.path.join(SIM_DIR, "sim_params.f")
    shutil.copy(src, dst)

    if args.timeout_ns > 0:
        timeout_ns = args.timeout_ns
        t_src = "user --timeout-ns"
    else:
        # 按 case 总尺寸估算：对所有 case 的预算求和 + 100 ms 基础 overhead
        est_total = sum(estimate_case_timeout_ns(c[2], c[3], c[4], c[5], c[6], c[7], c[9])
                        for c in cases)
        timeout_ns = est_total + 100_000_000  # +100 ms 设施 overhead
        t_src = "auto-est"

    with open(dst, "a", encoding="utf-8") as f:
        f.write(f"+N_CASES={n_cases}\n")
        f.write(f"+TIMEOUT_NS={timeout_ns}\n")

    print(f"  watchdog timeout = {timeout_ns:,} ns "
          f"(~{timeout_ns // 10:,} cycles @10 ns)  [{t_src}]")

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
