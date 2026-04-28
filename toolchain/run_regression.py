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

# Report 输出到 sim dir（和生成的 .txt 放一起）。
# 默认文件名带时间戳: regression_report_YYYY-MM-DD_HH-MM-SS.txt
# 在 main() 里 lazy 生成（取脚本启动当时的时间），可用 --out 覆盖.
OUTPUT_FILE = None

# ---------------------------------------------------------------------------
# 测试用例定义
#   (name, mode, c_in, c_out, k, h_in, w_in, stride, shift, pad)
#   mode: 'batch' 一次性搬完后核跑；'stream' row-ring 并行
#   pad : symmetric pad (top=bot=left=right)；0 = no pad
# ---------------------------------------------------------------------------
CASES_OLD = [
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

CASES = [
    # 网络结构（输入 960×540×4）:
    #   Patch embed (4×4, s=4, 4→16)         → 240×135×16    [33.2M MACs]
    #   Layer 1 (16→16, s=2)                 → 120×68×16     [39.7M MACs]
    #   Layer 2 (16→32, s=2)                 → 60×34×32      [29.2M MACs]
    #   Layer 3 (32→64, s=2)                 → 30×17×64      [29.2M MACs]
    #   AvgPool                              → 2×2×64        [无 MAC]
    #   FC merged (256→522)                  → 512+10 split  [0.13M MACs]
    #
    # 每个 Layer 内部:
    #   Block1.Conv1     K=3, s=2, Cout=Cin*2 (Layer 1 例外: Cout=Cin)
    #   Block1.Conv2     K=3, s=1, Cout=Cin   (Cin = Conv1 的输出通道)
    #   Block2.DownSample K=1, s=2, Cout=Cin*2 (Layer 1 例外)
    # 软件层将 Block1.Conv2 输出与 Block2.DownSample 输出相加 (硬件不支持 SDP 残差通路, 各 conv 独立跑).
    #
    # AvgPool 没硬件, 跳过. FC 视为 1×1 conv. 注: stride=4 的 Patch embed 必须配合 `--s2d`
    # 跑 (line_buffer 原生只支持 stride 1/2).

    # ---- Patch embed: 4×4 stride=4, Cin=4 → Cout=16 ----
    ("Patch K=4 C4C16  960x540 s4 p0",      "conv",   4,  16, 4, 960, 540, 4, 0, 0),  # → 240×135

    # ---- Layer 1 (16→16, 不翻倍) ----
    ("L1.B1.C1 K=3 C16C16 240x135 s2 p1",   "conv",  16,  16, 3, 240, 135, 2, 0, 1),  # → 120×68
    ("L1.B1.C2 K=3 C16C16 120x68  s1 p1",   "conv",  16,  16, 3, 120,  68, 1, 0, 1),
    ("L1.B2.ds K=1 C16C16 240x135 s2 p0",   "conv",  16,  16, 1, 240, 135, 2, 0, 0),

    # ---- Layer 2 (16→32, 翻倍) ----
    ("L2.B1.C1 K=3 C16C32 120x68  s2 p1",   "conv",  16,  32, 3, 120,  68, 2, 0, 1),  # → 60×34
    ("L2.B1.C2 K=3 C32C32 60x34   s1 p1",   "conv",  32,  32, 3,  60,  34, 1, 0, 1),
    ("L2.B2.ds K=1 C16C32 120x68  s2 p0",   "conv",  16,  32, 1, 120,  68, 2, 0, 0),

    # ---- Layer 3 (32→64, 翻倍) ----
    ("L3.B1.C1 K=3 C32C64 60x34   s2 p1",   "conv",  32,  64, 3,  60,  34, 2, 0, 1),  # → 30×17
    ("L3.B1.C2 K=3 C64C64 30x17   s1 p1",   "conv",  64,  64, 3,  30,  17, 1, 0, 1),
    ("L3.B2.ds K=1 C32C64 60x34   s2 p0",   "conv",  32,  64, 1,  60,  34, 2, 0, 0),

    # ---- FC merged: AvgPool 后 2×2×64 flatten=256 维, 输出 522 (= 512+10 拆分) ----
    ("FC C256C522 1x1",                     "conv", 256, 522, 1,   1,   1, 1, 0, 0),
]

# R.2 残差测试用例: 用 --residual 开关启用. 每个 layer 加随机 INT8 shortcut, 经 SDP
# 后的 fusion 路径 (mult=1, shift=0 直加) 验证残差数据通路 + bit-exact 数值匹配.
# 选几个有代表性的 shape:
#   - 小 K=3 conv: 验证基本残差路径
#   - K=1 stride=2 ds: 模拟 ResNet block 的 downsample 路径
#   - 大 K=3 stride=1: 验证多 strip / ring wrap 场景
RESIDUAL_CASES = [
    ("Residual K=3 C16C16 30x30 s1 p1",   "conv", 16, 16, 3, 30, 30, 1, 0, 1),
    ("Residual K=1 C16C16 30x30 s1 p0",   "conv", 16, 16, 1, 30, 30, 1, 0, 0),
    ("Residual K=3 C16C32 60x34 s1 p1",   "conv", 16, 32, 3, 60, 34, 1, 0, 1),
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
                   ky_fold=False, s2d=False, residual=False):
    case_dir = os.path.join(SIM_DIR, "cases", f"case{case_idx:02d}")
    os.makedirs(case_dir, exist_ok=True)
    # J-1: gen_isa_test.py 的 --streaming 默认 True，无需显式传
    fold_flag = ""
    if ky_fold:   fold_flag += " --ky-fold"
    if s2d:       fold_flag += " --s2d"
    if residual:  fold_flag += " --residual"
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

# CASE_PROFILE 行: 4 个 V/R 接口 × 3 状态 = 12 个原始 handshake 计数
PROFILE_RE = re.compile(
    r"CASE_PROFILE\s+(\d+)\s+cycles=(\d+)"
    r"\s+act_fire=(\d+)\s+act_stall=(\d+)\s+act_idle=(\d+)"
    r"\s+wgt_fire=(\d+)\s+wgt_stall=(\d+)\s+wgt_idle=(\d+)"
    r"\s+psum_fire=(\d+)\s+psum_stall=(\d+)\s+psum_idle=(\d+)"
    r"\s+acc_fire=(\d+)\s+acc_stall=(\d+)\s+acc_idle=(\d+)"
    r"\s+name=([^\n]+)")


def parse_sim_log(text):
    """返回 list of dict（按 case index 排序）"""
    results = {}
    for line in text.splitlines():
        m = CASE_RE.search(line)
        if m:
            idx = int(m.group(1))
            results.setdefault(idx, {})
            results[idx].update({
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
            })
            continue
        mp = PROFILE_RE.search(line)
        if mp:
            idx = int(mp.group(1))
            results.setdefault(idx, {})
            results[idx].update({
                "act_fire":   int(mp.group(3)),  "act_stall":  int(mp.group(4)),  "act_idle":   int(mp.group(5)),
                "wgt_fire":   int(mp.group(6)),  "wgt_stall":  int(mp.group(7)),  "wgt_idle":   int(mp.group(8)),
                "psum_fire":  int(mp.group(9)),  "psum_stall": int(mp.group(10)), "psum_idle":  int(mp.group(11)),
                "acc_fire":   int(mp.group(12)), "acc_stall":  int(mp.group(13)), "acc_idle":   int(mp.group(14)),
            })
    return [results[i] for i in sorted(results.keys())]


# ---------------------------------------------------------------------------
# 报告生成
# ---------------------------------------------------------------------------
def write_report(results, case_opts=None, label=""):
    """
    case_opts: dict[idx -> str], 描述编译器对该层做的优化标记 ("-"/"s2d"/"ky"/"s2d+ky"/...).
               None 时一律显示 "-".
    """
    lines = []
    def L(s=""): lines.append(s)

    # name 格式 "K=3 C8C8 48x48 s=1|conv" —— 拆掉历史遗留的 mode 后缀, 只留 case 名
    def split_name(r):
        if "|" in r["name"]:
            return r["name"].rsplit("|", 1)[0]
        return r["name"]

    def opt_of(r):
        if case_opts is None:
            return "-"
        return case_opts.get(r["idx"], "-")

    # 动态 case 列宽：取所有 case name 的最长长度 (修 case 名超过固定 27 时, 后续
    # 列被推右导致 Mode/Res/Cycles 列错位的问题)
    case_w = max([len(split_name(r)) for r in results] + [len("Case")])
    opt_w  = max([len(opt_of(r))     for r in results] + [len("Opt"), 6])

    # 聚合统计 (用于 Cyc% / Wst% / TOTAL 行)
    total_cycles  = sum(r['cycles']  for r in results)
    # waste = (1 - mac_util) × cycles, 单位 = cycle-equiv (×256 在分子分母都出现, 抵消)
    case_waste    = {r['idx']: (1.0 - r['mac_util']/100.0) * r['cycles'] for r in results}
    total_waste   = sum(case_waste.values())
    avg_mac_pct   = 100.0 * (total_cycles - total_waste) / max(total_cycles, 1)
    sum_ifb_r     = sum(r['ifb_r']   for r in results)
    sum_ofb_w     = sum(r['ofb_w']   for r in results)
    sum_arf_w     = sum(r['arf_w']   for r in results)
    sum_arf_r     = sum(r['arf_r']   for r in results)
    sum_parf_f    = sum(r['parf_f']  for r in results)
    sum_parf_d    = sum(r['parf_d']  for r in results)
    avg_ratio     = (sum_arf_r / sum_arf_w) if sum_arf_w > 0 else 0.0

    # ---- Summary 表 ----
    # 所有计数列统一 11 位宽 (够放 9,999,999 = 9 字符带逗号, 再留 2 余量)
    # Opt 列: 编译器对该层做的特殊处理标记 (- / s2d / ky / s2d+ky / ...). 单一 conv mode
    # 已经退化, 这一列改为显示编译器优化, 比 'conv' 重复值更有信息量.
    hdr = (f"  {'Case':<{case_w}} {'Opt':^{opt_w}} {'Res':^5} "
           f"{'Cycles':>11} {'Cyc%':>6} {'MAC%':>6} {'Wst%':>6}  "
           f"{'IFB_R':>11} {'WB_R':>9} {'OFB_W':>11}  "
           f"{'ARF_W':>11} {'ARF_R':>11} {'Ratio':>6}  "
           f"{'PARF_F':>11} {'PARF_D':>11}")

    # SEP/SUB 宽度根据 header 实际长度算 (避免短了导致表格切到一半)
    bar_w = max(len(hdr), 140)
    SEP = "=" * bar_w
    SUB = "-" * bar_w

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Regression (batch + streaming)  {label}")
    L(f"  Generated: {now}")
    L(SEP)
    L()
    L(hdr)
    L(SUB)
    all_pass = True
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        if not r["passed"]:
            all_pass = False
        cname   = split_name(r)
        opt     = opt_of(r)
        ratio   = (r['arf_r'] / r['arf_w']) if r['arf_w'] > 0 else 0.0
        cyc_pct = 100.0 * r['cycles'] / max(total_cycles, 1)
        wst_pct = (100.0 * case_waste[r['idx']] / total_waste) if total_waste > 0 else 0.0
        row = (f"  {cname:<{case_w}} {opt:^{opt_w}} {status:^5} "
               f"{r['cycles']:>11,} {cyc_pct:>5.1f}% {r['mac_util']:>5.1f}% {wst_pct:>5.1f}%  "
               f"{r['ifb_r']:>11,} {'-':>9} {r['ofb_w']:>11,}  "
               f"{r['arf_w']:>11,} {r['arf_r']:>11,} {ratio:>5.2f}x  "
               f"{r['parf_f']:>11,} {r['parf_d']:>11,}")
        L(row)
    L(SUB)
    # ---- TOTAL 总结行 (cycle-weighted MAC%, 各 SRAM 访问总和) ----
    total_row = (f"  {'TOTAL':<{case_w}} {'':<{opt_w}} {'':<5} "
                 f"{total_cycles:>11,} {'100.0%':>6} {avg_mac_pct:>5.1f}% {'100.0%':>6}  "
                 f"{sum_ifb_r:>11,} {'-':>9} {sum_ofb_w:>11,}  "
                 f"{sum_arf_w:>11,} {sum_arf_r:>11,} {avg_ratio:>5.2f}x  "
                 f"{sum_parf_f:>11,} {sum_parf_d:>11,}")
    L(total_row)
    L(SUB)
    L("  Opt: 编译器对该层做的特殊处理. -=baseline; s2d=Space-to-Depth (stride>=2 抽样折到 cin);")
    L("       ky=Ky-fold (Cin<16 时 ky 折到 cin, 输入 y 偏移复制); 多个用 + 拼.")
    L("  MAC%: useful_mac_ops / peak_mac_ops; useful = H_out×W_out×K²×Cin×Cout (pre-fold原始 conv);")
    L("        peak = cycles×NUM_COL×NUM_PE. Cout<16 (PE 列空转) + Ky-fold pad 都会扣利用率。")
    L("  Cyc%: 该 case cycles 占总 cycles 比例。 Wst%: 该 case MAC 浪费占总浪费比例 (定位瓶颈层)")
    L("  TOTAL.MAC% = cycle-weighted (Σ useful / Σ peak); TOTAL.Ratio = Σ ARF_R / Σ ARF_W")
    L("  ARF_W=ARF写(=IFB读扣除 pad)；ARF_R=ARF读到 MAC；Ratio=ARF_R/ARF_W (>1 表示 kx 滑窗复用)")
    L("  IFB_R/OFB_W=SRAM 访问次数；PARF_F/PARF_D=psum 累加写/读")
    L()

    # ---- Handshake PROFILE 表 ----
    # 4 个 V/R 接口 × {fire, stall, idle} 原始计数, 百分比相对 case cycles
    # 解读:
    #   fire  = V&R 都 1 (握手成功)
    #   stall = V=1 R=0 (上游有数据但下游不收, 下游慢)
    #   idle  = V=0 R=1 (下游能收但上游没数据, 上游慢)
    if any('act_fire' in r for r in results):
        L(SEP)
        L("  Handshake PROFILE — 各 V/R 接口 {fire / stall / idle} 占总 cycle %")
        L(SEP)
        L()
        hdr = (f"  {'Case':<{case_w}} {'Cycles':>11}  "
               f"{'act F/S/I':>22}  "
               f"{'wgt F/S/I':>22}  "
               f"{'psum F/S/I':>22}  "
               f"{'acc F/S/I':>22}")
        L(hdr)
        L(SUB)
        for r in results:
            if 'act_fire' not in r:
                continue
            cname = split_name(r)
            cy = max(r['cycles'], 1)
            def fsi(prefix):
                f = 100.0 * r[f'{prefix}_fire']  / cy
                s = 100.0 * r[f'{prefix}_stall'] / cy
                i = 100.0 * r[f'{prefix}_idle']  / cy
                return f"{f:>5.1f}/{s:>5.1f}/{i:>5.1f}"
            row = (f"  {cname:<{case_w}} {r['cycles']:>11,}  "
                   f"{fsi('act'):>22}  "
                   f"{fsi('wgt'):>22}  "
                   f"{fsi('psum'):>22}  "
                   f"{fsi('acc'):>22}")
            L(row)
        L(SUB)
        L("  接口含义:")
        L("    act  = line_buffer  → mac_array      (输入激活)")
        L("    wgt  = wgt_buffer   → mac_array      (权重)")
        L("    psum = mac_array    → parf_accum     (psum 写入)")
        L("    acc  = parf_accum   → sdp/ofb_writer (psum 读出送 SDP)")
        L("  解读:")
        L("    fire 高 → 接口在该 % 时间正常握手 (理想就是 100% fire)")
        L("    stall 高 → 该接口被下游卡: 检查 stall 接口的下一级")
        L("    idle 高 → 该接口在等上游: 检查 idle 接口的上一级")
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
    parser.add_argument("--out",   default=None,
                        help="Output file (默认: regression_report_<timestamp>.txt)")
    parser.add_argument("--only",  default="all",
                        choices=["all"],
                        help="(legacy option, J-1 起只有 conv 一种) default: all")
    parser.add_argument("--case",  default=None,
                        help="只跑 name 包含此子串的 case（大小写敏感）。例: --case C16C10")
    parser.add_argument("--timeout-ns", type=int, default=0,
                        help="vsim watchdog 超时 (ns)；0=按每 case 尺寸自动估算")
    parser.add_argument("--fold", action="store_true",
                        help="对所有 K>1 且 Cin<16 的 case 启用 Ky fold")
    parser.add_argument("--s2d", action="store_true",
                        help="对所有 stride>=2 且 K>=stride 的 case 启用 Space-to-Depth (可与 fold 叠加)")
    parser.add_argument("--residual", action="store_true",
                        help="R.2: 跑 RESIDUAL_CASES (启用 SDP 残差 fusion 路径), 不跑常规 CASES")
    args = parser.parse_args()
    if args.out is not None:
        OUTPUT_FILE = args.out
    else:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        OUTPUT_FILE = os.path.join(SIM_DIR, f"regression_report_{ts}.txt")

    case_pool = RESIDUAL_CASES if args.residual else CASES
    cases = [c for c in case_pool if args.only == "all" or c[1] == args.only]
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
    case_opts = {}   # idx -> opt 字符串 (供 report Opt 列展示)
    for i, case in enumerate(cases):
        # case 里 c_in 是第 3 个字段, k 是第 5 个, stride 是第 8 个
        _, _, c_in_v, _, k_v, _, _, stride_v, _, _ = case
        # 转换条件:
        #   S2D : stride>=2 AND K>=stride AND Cin<PE_H (=16)
        #     Cin>=16 时 PE 行已经填满, S2D 反而引入内核 pad 浪费 (K%stride!=0 时);
        #     只在 Cin<16 时启用, 让 stride² × Cin 把 PE 行填上去, 是真正的优化场景.
        #   Ky-fold: K>1 AND Cin<16 (S2D 后 Cin' 通常 ≥16, Ky-fold 自动跳过)
        use_s2d = args.s2d and (stride_v >= 2) and (k_v >= stride_v) and (c_in_v < 16)
        # S2D 后 Cin' = stride²·Cin, 重新判断 fold 触发条件
        c_in_eff = (stride_v * stride_v * c_in_v) if use_s2d else c_in_v
        k_eff    = ((k_v + stride_v - 1) // stride_v) if use_s2d else k_v
        use_ky = args.fold and (k_eff > 1) and (c_in_eff < 16)
        gen_info, err = gen_case_files(i, *case,
                                       ky_fold=use_ky, s2d=use_s2d,
                                       residual=args.residual)
        if err:
            print(f"  case {i} ({case[0]}) gen ERROR: {err}")
            sys.exit(1)
        opt_tags = []
        if use_s2d:       opt_tags.append("s2d")
        if use_ky:        opt_tags.append("ky")
        if args.residual: opt_tags.append("res")
        case_opts[i] = "+".join(opt_tags) if opt_tags else "-"
        fold_mark = f"[{case_opts[i]}]" if opt_tags else ""
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

    all_pass = write_report(results, case_opts=case_opts, label=args.label)

    print(f"\n报告已写入: {OUTPUT_FILE}")
    print(f"总体结论  : {'All PASS [OK]' if all_pass else 'Some FAILED [!!]'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
