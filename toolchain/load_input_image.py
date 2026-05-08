"""
load_input_image.py — 真图 → ResNet11 layer 0 IFM 转换工具

功能:
  PNG/JPG → resize 到 (H_in, W_in) → 转 RGB → pad 到 cin 通道 → INT8 量化
  → [H_in][W_in][cin] int8 array (跟 gen_isa_test.generate_random ifm_arr_in 兼容)

ResNet11 当前 layer 0 cin=4 (RGB + 1 alpha/zero pad), 因为 16 PE 行整除 (cin_slice=4
配合 patch s2d K=4 → cin_eff=64). RGB 真图 pad 1 通道 (alpha=255 或 zero) 让 cin=4.

INT8 量化: per-tensor symmetric, scale = max(abs(pixel)) / 127, 输入 uint8 0..255 先
减 128 → signed -128..127, 然后乘 scale (此处 scale=1 因为 uint8 已经是 INT8 范围).

用法:
    from load_input_image import load_image_as_ifm
    ifm_arr = load_image_as_ifm('test.png', H_in=960, W_in=540, cin=4)
    # ifm_arr: list[H_in] of list[W_in] of list[cin] int  (跟 gen_isa_test 兼容)
"""
import os


def load_image_as_ifm(image_path, H_in, W_in, cin=4, pad_value=0, normalize='center'):
    """加载真图转 IFM array.

    Args:
        image_path: PNG/JPG 路径
        H_in, W_in: 目标图像尺寸 (resize 到这个)
        cin: 输入通道数 (RGB 3, ResNet11 cin=4 时 pad 1 通道)
        pad_value: cin > 3 时 pad 通道的填充值 (0 = 黑色, 255 = 白色, 128 = 中灰)
        normalize: 'center' = uint8 [0,255] - 128 → signed [-128,127]
                   'raw'    = uint8 [0,255] (无符号, 但 INT8 视图 [128,127] 跨界)

    Returns:
        list[H_in] × list[W_in] × list[cin] int8 (-128..127)
    """
    try:
        from PIL import Image
    except ImportError:
        raise ImportError("需要 PIL (pip install Pillow)")

    img = Image.open(image_path).convert('RGB')   # 强制 3 通道
    img = img.resize((W_in, H_in), Image.BILINEAR)
    pixels = list(img.getdata())                  # [H*W] tuples (R,G,B)

    # 重排 [H][W][3]
    rgb = [[pixels[r * W_in + c] for c in range(W_in)] for r in range(H_in)]

    # Pad 到 cin 通道
    if cin == 3:
        ifm_uint8 = rgb
    elif cin > 3:
        ifm_uint8 = [
            [list(rgb[r][c]) + [pad_value] * (cin - 3) for c in range(W_in)]
            for r in range(H_in)
        ]
    else:
        # cin < 3 (e.g. grayscale): 取 R 通道或者 luminance
        ifm_uint8 = [
            [[(rgb[r][c][0] * 299 + rgb[r][c][1] * 587 + rgb[r][c][2] * 114) // 1000]
             for c in range(W_in)]
            for r in range(H_in)
        ]
        if cin > 1:
            ifm_uint8 = [
                [ifm_uint8[r][c] + [pad_value] * (cin - 1) for c in range(W_in)]
                for r in range(H_in)
            ]

    # 量化 uint8 → int8
    if normalize == 'center':
        # uint8 [0,255] - 128 → int8 [-128,127]
        ifm_int8 = [
            [[int(v) - 128 for v in ifm_uint8[r][c]] for c in range(W_in)]
            for r in range(H_in)
        ]
    else:
        # raw: uint8 当 int8 看 (不推荐, [128,255] 会变 [-128,-1])
        ifm_int8 = [
            [[int(v) if v < 128 else int(v) - 256 for v in ifm_uint8[r][c]] for c in range(W_in)]
            for r in range(H_in)
        ]

    return ifm_int8


def gen_synthetic_test_image(out_path, W=540, H=960, pattern='gradient'):
    """生成合成测试图 (不依赖外部图源, 适合 sim demo).

    Args:
        out_path: 输出 PNG 路径
        W, H: 图像尺寸
        pattern: 'gradient' (彩色梯度), 'checkerboard', 'circles'
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        raise ImportError("需要 PIL (pip install Pillow)")

    img = Image.new('RGB', (W, H))
    pixels = img.load()

    if pattern == 'gradient':
        for r in range(H):
            for c in range(W):
                pixels[c, r] = (
                    int(255 * c / W),                  # R: 横向
                    int(255 * r / H),                  # G: 纵向
                    int(255 * (1 - (c + r) / (W + H))),# B: 反对角
                )
    elif pattern == 'checkerboard':
        block = 32
        for r in range(H):
            for c in range(W):
                v = ((r // block) + (c // block)) % 2 * 255
                pixels[c, r] = (v, v, v)
    elif pattern == 'circles':
        cx, cy = W // 2, H // 2
        for r in range(H):
            for c in range(W):
                d = ((c - cx) ** 2 + (r - cy) ** 2) ** 0.5
                v = int(255 * abs((d / 50) % 2 - 1))
                pixels[c, r] = (v, (v + 80) % 255, (v + 160) % 255)
    else:
        raise ValueError(f"unknown pattern: {pattern}")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    img.save(out_path)
    print(f"saved {pattern} {W}x{H} → {out_path}")


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--gen', choices=['gradient', 'checkerboard', 'circles', 'all'],
                   help='生成合成测试图')
    p.add_argument('--out-dir', default='models/images/resnet11_test',
                   help='输出目录')
    p.add_argument('--W', type=int, default=540)
    p.add_argument('--H', type=int, default=960)
    args = p.parse_args()

    if args.gen:
        patterns = ['gradient', 'checkerboard', 'circles'] if args.gen == 'all' else [args.gen]
        for pat in patterns:
            gen_synthetic_test_image(
                os.path.join(args.out_dir, f'{pat}.png'),
                W=args.W, H=args.H, pattern=pat
            )
