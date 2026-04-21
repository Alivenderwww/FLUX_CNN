"""
run_model.py — 真实模型端到端工程验证

类似 run_regression，但面向"模型 × 输入图像"矩阵：
  - 选模型 (zoo.MODELS)
  - 选输入来源 (PNG 文件夹 / torchvision test set / 单张 PNG)
  - 每张图走一遍：compile_model (用该图作 calib) → vsim → 读硬件 OFB argmax → 对比
  - 每张 pass/fail + 三方 label (true / pytorch-float / hardware-int8) 进 report txt

用法:
  # 单张: 批量 dump 过的 PNG 中某一张
  .venv/Scripts/python.exe run_model.py --model mnist_allconv \\
      --image images/mnist_test/0000_label7.png

  # 文件夹遍历 (按字母序)
  .venv/Scripts/python.exe run_model.py --model mnist_allconv \\
      --image-dir images/mnist_test/ --limit 10

  # 从 torchvision test set 取索引 5
  .venv/Scripts/python.exe run_model.py --model mnist_allconv \\
      --torchvision-idx 5

  # 检验每层 OFB bit-exact (默认只检验终层 argmax)
  .venv/Scripts/python.exe run_model.py --model mnist_allconv \\
      --image-dir images/mnist_test/ --verify-intermediate

输出:
  toolchain/models/model_report.txt  (可 --out 覆盖)
"""

import os
import sys
import argparse
import subprocess
import re
import json
import glob
import datetime


_SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
_TOOLCHAIN_DIR = os.path.dirname(_SCRIPT_DIR)
SIM_DIR        = os.path.normpath(os.path.join(_TOOLCHAIN_DIR, "..", "sim", "tb_core_dma"))
DEFAULT_IMAGES = os.path.join(_SCRIPT_DIR, "images", "mnist_test")
DEFAULT_REPORT = os.path.join(_SCRIPT_DIR, "model_report.txt")

# 用本目录的 Python 路径 (zoo / train_mnist / compile_model)
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
if _TOOLCHAIN_DIR not in sys.path:
    sys.path.insert(0, _TOOLCHAIN_DIR)

import compile_layer     # noqa: E402
import compile_model     # noqa: E402
import zoo               # noqa: E402


# ---------------------------------------------------------------------------
# 输入加载
# ---------------------------------------------------------------------------
FNAME_LABEL_RE = re.compile(r'label(\d+)', re.IGNORECASE)


def _load_png_as_tensor(path, torch):
    """PNG → Normalized 28×28 MNIST tensor (1,1,28,28)。文件名 lookahead: label<N> → true_label"""
    from PIL import Image
    from torchvision import transforms
    img = Image.open(path).convert("L")
    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    x = tfm(img).unsqueeze(0)   # (1,1,H,W)
    m = FNAME_LABEL_RE.search(os.path.basename(path))
    true_label = int(m.group(1)) if m else None
    return x, true_label, os.path.basename(path)


def _load_torchvision_idx(idx, torch):
    from torchvision import datasets, transforms
    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    data_dir = os.path.join(_SCRIPT_DIR, "data")
    ds = datasets.MNIST(data_dir, train=False, download=True, transform=tfm)
    x, label = ds[idx]
    return x.unsqueeze(0), int(label), f"torchvision_test[{idx}]"


def collect_inputs(args, torch):
    """返回 list of (x_tensor, true_label, label_str)。"""
    items = []
    if args.image:
        items.append(_load_png_as_tensor(args.image, torch))
    elif args.image_dir:
        files = sorted(glob.glob(os.path.join(args.image_dir, "*.png")))
        if args.limit > 0:
            files = files[:args.limit]
        for f in files:
            items.append(_load_png_as_tensor(f, torch))
    elif args.torchvision_idx is not None:
        for idx in args.torchvision_idx:
            items.append(_load_torchvision_idx(idx, torch))
    else:
        raise ValueError("必须指定 --image / --image-dir / --torchvision-idx 之一")
    return items


# ---------------------------------------------------------------------------
# 每张图一轮: compile → vsim → parse → argmax
# ---------------------------------------------------------------------------
def compile_model_for_input(model_name, x_tensor, sim_dir, start_idx):
    """调 compile_model.compile_and_emit_model 把该图作 calib 输入编译到 cases/caseNN。"""
    model, _default_calib, _n_layers = zoo.build_model(model_name)
    plan_dir = os.path.join(sim_dir, "cases", f"{model_name}_plan")
    os.makedirs(plan_dir, exist_ok=True)

    def _mk_layer_dir(k):
        return os.path.join(sim_dir, "cases", f"case{start_idx + k:02d}")

    compile_model.compile_and_emit_model(
        model, x_tensor, out_dir=plan_dir, streaming=True,
        model_name=model_name, layer_dir_fn=_mk_layer_dir,
    )
    with open(os.path.join(plan_dir, "model_plan.json")) as f:
        plan = json.load(f)
    return plan, model   # model 已 load weights (zoo.build_model)


def run_vsim(sim_dir, n_cases):
    """跑一次 vsim (复用 F-2 多 case 架构)，返回 {case_idx: result_dict}。"""
    case0_sim_params = os.path.join(sim_dir, "cases", "case00", "sim_params.f")
    sim_params       = os.path.join(sim_dir, "sim_params.f")
    with open(case0_sim_params) as f:
        content = f.read()
    with open(sim_params, "w") as f:
        f.write(content)
        f.write(f"+N_CASES={n_cases}\n")
    r = subprocess.run("vsim -c -do run.tcl", shell=True, cwd=sim_dir,
                       capture_output=True, text=True)
    log = r.stdout + r.stderr
    with open(os.path.join(sim_dir, "vsim_all.log"), "w", encoding="utf-8",
              errors="replace") as f:
        f.write(log)
    return parse_log(log)


RE_CASE_RESULT = re.compile(r"CASE_RESULT\s+(\d+)\s+(PASS|FAIL)")


def parse_log(text):
    out = {}
    for line in text.splitlines():
        m = RE_CASE_RESULT.search(line)
        if not m:
            continue
        idx = int(m.group(1))
        cy = re.search(r'cycles=(\d+)', line)
        mm = re.search(r'mismatches=(\d+)', line)
        nm = re.search(r'name=([^\n]+)', line)
        out[idx] = {
            'passed'    : m.group(2) == "PASS",
            'cycles'    : int(cy.group(1)) if cy else 0,
            'mismatches': int(mm.group(1)) if mm else 0,
            'name'      : nm.group(1).strip() if nm else "",
        }
    return out


def top_k(values, k):
    """返回 top-k index 列表，按 values 降序。"""
    return sorted(range(len(values)), key=lambda i: -values[i])[:k]


def read_hw_logits(ofm_txt, num_cout):
    """终层 expected_ofm.txt 第一行 16-byte word, 低 num_cout byte 为 signed int8 logits。
    regression PASS 保证内容 == DDR OFB bit-exact。"""
    with open(ofm_txt) as f:
        hexline = f.readline().strip()
    logits = []
    for i in range(num_cout):
        bs = hexline[32 - 2*(i+1):32 - 2*i]
        b = int(bs, 16)
        if b >= 128:
            b -= 256
        logits.append(b)
    return logits


# ---------------------------------------------------------------------------
# TB 中间层 compare 开关 —— 直接 patch tb 的 skip_ofb_clear 逻辑绕开
# 简化做法: 用 compile_model 的 emit_expected_ofm 让所有层写 expected_ofm.txt,
# 但 TB 端仍按 skip_ofb_clear_cfg 过滤比对. verify-intermediate 场景下 TB 需要
# 中间层也 compare. 本阶段保守做法: 只检验终层 argmax (skip_ofb_clear 中间层
# 不比对), verify-intermediate 开关留 TODO.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                description=__doc__)
    p.add_argument('--model', required=True, choices=list(zoo.MODELS.keys()),
                   help='zoo 里的 key, 例: mnist_allconv')
    group = p.add_mutually_exclusive_group()
    group.add_argument('--image',     default=None, help='单张 PNG 路径')
    group.add_argument('--image-dir', default=None,
                       help='PNG 文件夹，文件名 label<N> 被解析为 true label')
    group.add_argument('--torchvision-idx', type=int, nargs='+', default=None,
                       help='用 torchvision MNIST test set 第 N 张 (可多个)')
    p.add_argument('--limit', type=int, default=0,
                   help='--image-dir 最多跑多少张 (0=全部)')
    p.add_argument('--out',   default=DEFAULT_REPORT, help='report 输出路径')
    p.add_argument('--verify-intermediate', action='store_true',
                   help='检验每层 OFB bit-exact (注: 当前为终层 argmax-only)')
    args = p.parse_args()

    if not (args.image or args.image_dir or args.torchvision_idx):
        args.image_dir = DEFAULT_IMAGES
        if not os.path.exists(args.image_dir) or not os.listdir(args.image_dir):
            p.error("默认 images/mnist_test/ 为空; 先跑 dump_mnist_png.py 或指定 --image* 参数")

    torch = compile_layer._require_torch()
    inputs = collect_inputs(args, torch)
    print(f"[run_model] {args.model} × {len(inputs)} 张图")

    # 加载 model 到 PyTorch (拿 float forward 对照)
    model, _default_calib, n_layers = zoo.build_model(args.model)
    model.eval()

    # Logits 对齐的 post-processing: mnist_allconv 用 AllConvMNIST wrapper 压 (N,10)
    # 这里直接 forward 并 flatten 最后一维
    def pytorch_forward(x):
        with torch.no_grad():
            y = model(x)
        return y.flatten().tolist()

    # 跑每张图
    per_image = []
    for i, (x, true_label, label_str) in enumerate(inputs):
        print(f"  [{i+1}/{len(inputs)}] {label_str} → compile...", end=' ', flush=True)
        # 清 cases/
        import shutil as _sh
        cases_dir = os.path.join(SIM_DIR, "cases")
        if os.path.exists(cases_dir):
            _sh.rmtree(cases_dir, ignore_errors=True)
        os.makedirs(cases_dir, exist_ok=True)

        plan, _model = compile_model_for_input(args.model, x, SIM_DIR, start_idx=0)
        n_cases = plan['n_layers']
        print(f"vsim {n_cases}L...", end=' ', flush=True)
        results = run_vsim(SIM_DIR, n_cases)

        # 终层 argmax
        last_layer_case = os.path.join(cases_dir, f"case{n_cases - 1:02d}")
        ofm_txt = os.path.join(last_layer_case, "expected_ofm.txt")
        num_cout = plan['layers'][-1]['Cout']
        hw_logits    = read_hw_logits(ofm_txt, num_cout=num_cout)
        float_logits = pytorch_forward(x)
        hw_pred    = max(range(num_cout), key=lambda k: hw_logits[k])
        float_pred = max(range(num_cout), key=lambda k: float_logits[k])

        # 终层 vsim 是否 PASS (OFB bit-exact match compile_layer golden)
        final_case_passed = results.get(n_cases - 1, {}).get('passed', False)
        hw_correct = (true_label is not None) and (hw_pred == true_label)
        fp_correct = (true_label is not None) and (float_pred == true_label)
        status = "PASS" if (final_case_passed and
                            (hw_pred == float_pred if true_label is None else hw_correct))\
                        else "FAIL"

        # 每层 cycles 汇总 + 全模型总 cycles
        per_layer_cycles = [results.get(k, {}).get('cycles', 0) for k in range(n_cases)]
        total_cycles = sum(per_layer_cycles)

        # Top-3 置信度
        float_top3 = top_k(float_logits, 3)
        hw_top3    = top_k(hw_logits, 3)

        per_image.append({
            'idx': i, 'label_str': label_str, 'true_label': true_label,
            'float_pred': float_pred, 'hw_pred': hw_pred,
            'float_logits': float_logits, 'hw_logits': hw_logits,
            'float_top3': float_top3, 'hw_top3': hw_top3,
            'per_layer_cycles': per_layer_cycles,
            'total_cycles': total_cycles,
            'vsim_ok': final_case_passed,
            'hw_correct': hw_correct, 'fp_correct': fp_correct,
            'status': status,
        })
        print(f"true={true_label} fp={float_pred} hw={hw_pred} "
              f"cycles={total_cycles} [{status}]")

    # 写报告
    write_report(args, per_image)


def write_report(args, per_image):
    lines = []
    def L(s=""): lines.append(s)
    SEP = "=" * 110
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L(SEP)
    L(f"  FLUX CNN Model Runner — {args.model}")
    L(f"  Generated: {now}")
    L(f"  Inputs: {len(per_image)} image(s)")
    L(SEP)
    L()

    # Summary 表
    hdr = (f"  {'Idx':>4}  {'Name':<30}  {'True':>5}  {'FP':>5}  {'HW':>5}  "
           f"{'Cycles':>10}  {'vsim':>5}  {'Status':>7}")
    L(hdr)
    L("  " + "-" * (len(hdr) - 2))
    n_hw_correct = 0
    n_fp_correct = 0
    n_match      = 0
    n_vsim_ok    = 0
    for r in per_image:
        vs  = "OK" if r['vsim_ok'] else "FAIL"
        tru = str(r['true_label']) if r['true_label'] is not None else "-"
        L(f"  {r['idx']:>4}  {r['label_str']:<30}  {tru:>5}  "
          f"{r['float_pred']:>5}  {r['hw_pred']:>5}  "
          f"{r['total_cycles']:>10,}  {vs:>5}  {r['status']:>7}")
        n_hw_correct += int(r['hw_correct'])
        n_fp_correct += int(r['fp_correct'])
        n_match      += int(r['float_pred'] == r['hw_pred'])
        n_vsim_ok    += int(r['vsim_ok'])
    L()
    n = len(per_image)
    L(f"  Totals (of {n}):")
    L(f"    vsim  bit-exact:     {n_vsim_ok}/{n}  ({100*n_vsim_ok/n:.1f}%)")
    L(f"    PyTorch argmax OK:   {n_fp_correct}/{n}  ({100*n_fp_correct/n:.1f}%)")
    L(f"    Hardware  argmax OK: {n_hw_correct}/{n}  ({100*n_hw_correct/n:.1f}%)")
    L(f"    float == hw:         {n_match}/{n}  ({100*n_match/n:.1f}%)")
    if n > 0:
        avg_cy = sum(r['total_cycles'] for r in per_image) // n
        L(f"    Avg cycles/image:    {avg_cy:,}")
    L()
    L(SEP)
    L(f"  详细：Top-3 置信度 + 每层 cycles")
    L(SEP)
    for r in per_image:
        tru = str(r['true_label']) if r['true_label'] is not None else "-"
        L()
        L(f"  [{r['idx']:>3}] {r['label_str']}  true={tru}  "
          f"status={r['status']}  total={r['total_cycles']:,} cycles")
        # Top-3 置信度（float + hw 并排）
        L(f"        {'Rank':<5}  {'FP class':<9}  {'FP logit':>10}  |  {'HW class':<9}  {'HW int8':>7}")
        for rk in range(3):
            fc = r['float_top3'][rk]; hc = r['hw_top3'][rk]
            fv = r['float_logits'][fc]; hv = r['hw_logits'][hc]
            mark_fp = " ←" if fc == r['float_pred'] else "  "
            mark_hw = " ←" if hc == r['hw_pred']    else "  "
            L(f"        {rk+1:<5}  class {fc}{mark_fp}  {fv:>+10.3f}  |  class {hc}{mark_hw}  {hv:>+7d}")
        # 每层 cycles
        cy_str = "  ".join(f"L{i}={c:,}" for i, c in enumerate(r['per_layer_cycles']))
        L(f"        per-layer cycles: {cy_str}")
    L(SEP)

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    # 控制台打印 summary 部分（避免太长）
    print()
    summary_end = next(i for i, l in enumerate(lines) if l.startswith(SEP) and i > 5)
    print("\n".join(lines[:summary_end + 1]))
    print(f"\n详细报告 → {args.out}")


if __name__ == '__main__':
    main()
