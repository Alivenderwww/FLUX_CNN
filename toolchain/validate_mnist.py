"""
validate_mnist.py — 从硬件仿真输出 argmax + 对比 PyTorch float forward

流程：
  1. 加载 train_mnist.py 保存的 ckpt: state_dict + calib_image + calib_label
  2. 读硬件仿真产出 (mnist_allconv 的 L4 expected_ofm.txt, 因 regression PASS 已
     bit-exact == DDR OFB)
  3. 解析 1x1x10 signed int8 logits
  4. 同图 PyTorch float forward → softmax argmax
  5. 打印并比对

前置：先 `python run_regression.py --case mnist_allconv` 保证 regression PASS，
      此时 cases/case04/expected_ofm.txt 是硬件 bit-exact 产出。
"""

import os
import argparse
import json

import compile_layer
import train_mnist


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR     = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))


def read_hw_logits(ofm_txt_path, num_cout=10):
    """硬件 OFB 的 expected_ofm.txt 里第 1 行 = 16 byte hex, 低 num_cout byte 为 signed int8 logits。"""
    with open(ofm_txt_path) as f:
        hexline = f.readline().strip()
    if len(hexline) != 32:
        raise ValueError(f"unexpected hex length {len(hexline)}, want 32 (16 bytes)")
    logits = []
    for i in range(num_cout):
        byte_str = hexline[32 - 2*(i+1):32 - 2*i]
        b = int(byte_str, 16)
        if b >= 128:
            b -= 256      # signed int8
        logits.append(b)
    return logits


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', default='mnist_allconv',
                   help='用哪个 model (对应 model_zoo 里的 key)')
    p.add_argument('--ofm-txt', default=None,
                   help='硬件最终层 expected_ofm.txt 路径。默认 cases/case04/expected_ofm.txt '
                        '(mnist_allconv 的 L4)')
    p.add_argument('--ckpt', default=None,
                   help='PyTorch ckpt 路径 (train_mnist.py 保存)')
    args = p.parse_args()

    torch = compile_layer._require_torch()

    ckpt_path = args.ckpt or os.path.join(_SCRIPT_DIR, "models", "mnist_allconv.pt")
    if args.ofm_txt:
        ofm_txt = args.ofm_txt
    else:
        # 自动扫 cases/caseNN 找末层 case_dir (case_name 含 "_L<N-1>|model" 的最大 N)
        import glob
        cases_root = os.path.join(SIM_DIR, "cases")
        ofm_txt = None
        last_idx = -1
        for d in sorted(glob.glob(os.path.join(cases_root, "case*"))):
            cfg = os.path.join(d, "config.txt")
            if not os.path.exists(cfg):
                continue
            with open(cfg) as f:
                name_line = next((l for l in f if l.startswith("_META_CASE_NAME")), "")
            if f"{args.model}_L" not in name_line or "|model" not in name_line:
                continue
            # 提取 L 后的数字
            tail = name_line.split(f"{args.model}_L", 1)[1]
            try:
                idx = int(tail.split("|", 1)[0])
            except ValueError:
                continue
            if idx > last_idx:
                last_idx = idx
                ofm_txt = os.path.join(d, "expected_ofm.txt")
        if ofm_txt is None:
            raise SystemExit(f"ERROR: 扫 {cases_root}/case*/config.txt 找不到 {args.model} 的层。"
                             f" 先跑 run_regression.py --case {args.model}。")
        print(f"[auto] 终层 OFM = {ofm_txt} (case_name {args.model}_L{last_idx})")

    if not os.path.exists(ofm_txt):
        raise SystemExit(
            f"ERROR: 找不到 {ofm_txt}\n"
            f"先跑: python run_regression.py --case {args.model}")
    if not os.path.exists(ckpt_path):
        raise SystemExit(f"ERROR: 找不到 ckpt {ckpt_path}")

    # 1. 硬件输出 logits
    hw_logits = read_hw_logits(ofm_txt, num_cout=10)
    hw_pred   = max(range(len(hw_logits)), key=lambda i: hw_logits[i])

    # 2. PyTorch float forward (用训练时存的 calib image)
    ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)
    model = train_mnist.AllConvMNIST()
    model.load_state_dict(ckpt['state_dict'])
    model.eval()
    x = ckpt['calib_image']        # (1,1,28,28)
    true_label = ckpt['calib_label']

    with torch.no_grad():
        logits_float = model(x).flatten().tolist()
    float_pred = max(range(len(logits_float)), key=lambda i: logits_float[i])

    # 3. 打印
    print("=" * 60)
    print(f"  MNIST {args.model} — 硬件 vs PyTorch vs true label")
    print("=" * 60)
    print(f"  True label            : {true_label}")
    print(f"  PyTorch float argmax  : {float_pred}")
    print(f"  Hardware int8 argmax  : {hw_pred}")
    print()
    print("  Class  |  FP logit  |  HW int8 logit")
    print("  -------+------------+---------------")
    for i in range(10):
        marker_fp = " ←" if i == float_pred else ""
        marker_hw = " ←" if i == hw_pred else ""
        marker_gt = " *"  if i == true_label else ""
        print(f"    {i}    | {logits_float[i]:>+9.3f}{marker_fp:2s} | {hw_logits[i]:>+6d}{marker_hw:2s}{marker_gt:2s}")
    print()

    hw_correct   = (hw_pred   == true_label)
    fp_correct   = (float_pred == true_label)
    match_fp_hw  = (hw_pred   == float_pred)

    status = ("OK" if (hw_correct and match_fp_hw) else "INFO")
    print(f"  [{status}]  true={true_label}  "
          f"float_pred={float_pred}{' (correct)' if fp_correct else ' (WRONG)'}  "
          f"hw_pred={hw_pred}{' (correct)' if hw_correct else ' (WRONG)'}  "
          f"{'float==hw' if match_fp_hw else 'float!=hw (量化误差)'}")
    print("=" * 60)

    # 退出码：只要硬件预测正确 (不要求 == float) 就算成功
    return 0 if hw_correct else 1


if __name__ == '__main__':
    raise SystemExit(main())
