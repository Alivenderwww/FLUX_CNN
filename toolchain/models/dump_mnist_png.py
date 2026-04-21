"""
dump_mnist_png.py — 把 torchvision MNIST test set dump 成 PNG 供 run_model 用

filename 约定: `NNNN_labelX.png` (4 位 0 padded index + true label)
  例: 0000_label7.png, 0001_label2.png, ...

用法:
    .venv/Scripts/python.exe dump_mnist_png.py              # 前 64 张
    .venv/Scripts/python.exe dump_mnist_png.py --count 10000  # 全部 test set
    .venv/Scripts/python.exe dump_mnist_png.py --out-dir my/folder --count 20

run_model.py 用 --image-dir 指向 images/mnist_test/ 就能批量跑。
"""

import os
import argparse


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR    = os.path.join(_SCRIPT_DIR, "data")
DEFAULT_OUT = os.path.join(_SCRIPT_DIR, "images", "mnist_test")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--count', type=int, default=64,
                   help='dump 前 N 张 (default 64; MNIST test 共 10000)')
    p.add_argument('--out-dir', default=DEFAULT_OUT)
    p.add_argument('--data-dir', default=DATA_DIR,
                   help='torchvision MNIST 存储根目录')
    args = p.parse_args()

    from torchvision import datasets
    from PIL import Image

    os.makedirs(args.out_dir, exist_ok=True)
    # Normalize 不做 (只存 raw uint8 图, 预处理留给 run_model)
    ds = datasets.MNIST(args.data_dir, train=False, download=True)
    n = min(args.count, len(ds))
    for i in range(n):
        img, label = ds[i]          # PIL Image (28x28 grayscale), int label
        if not isinstance(img, Image.Image):
            img = Image.fromarray(img)
        fname = f"{i:04d}_label{label}.png"
        img.save(os.path.join(args.out_dir, fname))
    print(f"dumped {n} PNG → {args.out_dir}")


if __name__ == '__main__':
    main()
