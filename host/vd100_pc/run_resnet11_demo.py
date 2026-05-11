"""
run_resnet11_demo.py — VD100 板 ResNet11 N=3 真硬件端到端 demo

板子永久烧的是 bitstream + vd100_rpc_server.c (无业务逻辑). PC 通过本脚本:
  1. 读 sim/tb_smc/cases/vd100_resnet11_n3/ (toolchain 编译输出)
  2. PIL 加载图片 → 540×960×4 INT8 → 数据 layout 重排 (s2d) → 240×135×64 NHWC
  3. RPC LOAD_DDR 上传所有 SMC 数据 (wb / desc / idma_cmd / odma_cmd / rdma) — 一次
  4. RPC LOAD_DDR 上传图片 IFM 到 SMC INPUT 区 (按 W slice 散布)
  5. RPC RUN_LAYERS 触发 a72 跑 11 层
  6. RPC READ_DDR 读 522-d OFM logit, top-5 显示
  7. 跟 sim expected_ofm.txt 对比 (random weights demo, 验证硬件等价性)

用户视角: 板烧 1 次, 任意图片任意次推理, 模型从来不烧到 PDI.

依赖: pip install Pillow numpy

先决条件:
  1. 板子烧好 PDI (含 bitstream + vd100_rpc_server.c ELF)
  2. 网线接通, 板 IP 169.254.111.10
  3. test_ddr_loopback.py 通过 (PS↔DDR 通路 OK)
  4. case 已生成: cd toolchain && FLUX_SMC_GLOBAL_BASE=0x10000000 \\
       python run_multicore_chain.py --smc --demo resnet11 --n_cores 3 \\
       --case_name vd100_resnet11_n3
"""
import argparse
import os
import sys
import time

import deploy_smc_case
import vd100_rpc
from vd100_rpc import Vd100Rpc


# ResNet11 layer 0 (Patch) 原始维度 (跟 toolchain run_multicore_chain.py 'resnet11' demo 一致)
ORIG_H, ORIG_W, ORIG_CIN = 960, 540, 4
PATCH_K, PATCH_STRIDE = 4, 4   # Patch conv: K=4 stride=4 pad=0
# s2d 后等效 (硬件视角): H_s2d × W_s2d × Cin_new
H_S2D = ORIG_H // PATCH_STRIDE      # 240
W_S2D = ORIG_W // PATCH_STRIDE      # 135
CIN_S2D = PATCH_STRIDE * PATCH_STRIDE * ORIG_CIN   # 64

DEFAULT_CASE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'sim', 'tb_smc', 'cases', 'vd100_resnet11_n3'
)


def load_image_int8(image_path: str) -> bytearray:
    """PNG/JPG → resize 到 (ORIG_H, ORIG_W) → cin=4 INT8 byte stream (NHWC).
    跟 host/vd100_pc/resnet11_client.py 同口径: uint8 - 128 居中, alpha=0.
    返回 ORIG_H × ORIG_W × ORIG_CIN bytes (= 2,073,600 byte, ~2 MB).
    """
    try:
        from PIL import Image
    except ImportError:
        sys.exit("ERROR: 需要 Pillow. 装: pip install Pillow")

    img = Image.open(image_path).convert('RGB')
    img = img.resize((ORIG_W, ORIG_H), Image.BILINEAR)
    pixels = list(img.getdata())   # [H*W] tuples (R, G, B)

    buf = bytearray(ORIG_H * ORIG_W * ORIG_CIN)
    idx = 0
    for r in range(ORIG_H):
        for c in range(ORIG_W):
            R, G, B = pixels[r * ORIG_W + c]
            # uint8 - 128 → int8, 存 little-endian byte (& 0xFF 防符号扩展)
            buf[idx]   = (R - 128) & 0xFF
            buf[idx+1] = (G - 128) & 0xFF
            buf[idx+2] = (B - 128) & 0xFF
            buf[idx+3] = 0   # alpha pad
            idx += 4
    return buf


def s2d_rearrange(ifm_orig: bytes, h_in: int, w_in: int, cin: int,
                  stride: int) -> bytes:
    """对 NHWC int8 byte stream 做 space-to-depth 重排 (无 padding, K=stride=4).
    跟 toolchain/hw_files.s2d_input 数学等价 (pad_top=0 pad_left=0):
      ifm_s2d[Y, X, p*cin+c] = ifm_orig[Y*stride + a(p), X*stride + b(p), c]
      其中 p = a*stride + b, a,b ∈ [0, stride)
    返回 H_s2d × W_s2d × (stride²·cin) bytes.

    数据量不变, 只重排 (Patch K=stride pad=0 时无 zero pad).
    """
    H_s2d = h_in // stride
    W_s2d = w_in // stride
    cin_new = stride * stride * cin
    out = bytearray(H_s2d * W_s2d * cin_new)
    for Y in range(H_s2d):
        for X in range(W_s2d):
            dst_base = (Y * W_s2d + X) * cin_new
            for a in range(stride):
                for b in range(stride):
                    p = a * stride + b
                    y_src = Y * stride + a
                    x_src = X * stride + b
                    src_base = (y_src * w_in + x_src) * cin
                    out[dst_base + p * cin : dst_base + p * cin + cin] = \
                        ifm_orig[src_base : src_base + cin]
    return bytes(out)


def parse_logit_top_n(ofm_bytes: bytes, n: int = 5) -> list:
    """522-d INT8 logit → top-N (random weights, 仅演示数据通路 bit-exact)."""
    logits = [int.from_bytes([b], byteorder='little', signed=True) for b in ofm_bytes]
    indexed = list(enumerate(logits))
    indexed.sort(key=lambda x: -x[1])
    return indexed[:n]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', default=None,
                   help='真图 PNG/JPG (resize 到 540×960, s2d 后送板). 不指定用 sim 默认 IFB.')
    p.add_argument('--case-dir', default=DEFAULT_CASE,
                   help='vd100_resnet11_n3 case 目录')
    p.add_argument('--vd100-ip', default='169.254.111.10')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--repeat', type=int, default=1, help='跑 N 次 inference (复用 LOAD)')
    p.add_argument('--no-verify', action='store_true', help='跳过 expected_ofm 比对')
    args = p.parse_args()

    if not os.path.isdir(args.case_dir):
        sys.exit(f"ERROR: case dir not found: {args.case_dir}\n"
                 f"  生成: cd toolchain && FLUX_SMC_GLOBAL_BASE=0x10000000 "
                 f"python run_multicore_chain.py --smc --demo resnet11 --n_cores 3 "
                 f"--case_name vd100_resnet11_n3")

    override_ifm = None
    if args.image:
        print(f"[0/4] Load image: {args.image}")
        t0 = time.time()
        orig = load_image_int8(args.image)
        print(f"  resize → {ORIG_H}×{ORIG_W}×{ORIG_CIN} = {len(orig)/1024:.1f} KB, "
              f"用时 {(time.time()-t0)*1000:.0f} ms")
        t0 = time.time()
        s2d = s2d_rearrange(orig, ORIG_H, ORIG_W, ORIG_CIN, PATCH_STRIDE)
        print(f"  s2d → {H_S2D}×{W_S2D}×{CIN_S2D} = {len(s2d)/1024:.1f} KB, "
              f"用时 {(time.time()-t0)*1000:.0f} ms")
        override_ifm = s2d
        # disable verify when using real image (sim expected_ofm 是基于 sim IFB random 数据)
        if not args.no_verify:
            print("  [info] 真图模式: 自动 --no-verify (expected_ofm 是 sim random IFB 算的)")
            args.no_verify = True

    print(f"\nConnect VD100 @ {args.vd100_ip}:{args.port}")
    with Vd100Rpc(args.vd100_ip, args.port) as rpc:
        # 1-3: deploy + run + (optional) verify
        stats = deploy_smc_case.deploy_and_run(
            rpc, args.case_dir, override_ifm=override_ifm,
            skip_verify=args.no_verify,
        )

        # 4. Top-5 logit
        ofm = stats['ofm']
        if len(ofm) >= 522:
            top5 = parse_logit_top_n(ofm[:522], 5)
            print("\n=== ResNet11 OFM top-5 logit ===")
            for rank, (idx, val) in enumerate(top5, 1):
                print(f"  #{rank}  class[{idx:3d}] = {val:+4d}")
            if args.image:
                print("  (random weights demo: 类别本身无意义, 验证的是数据通路 bit-exact)")

        # 重复推理 (复用已 LOAD 数据测吞吐)
        if args.repeat > 1:
            print(f"\n=== Repeat × {args.repeat - 1} (复用 LOAD, 只跑 RUN_LAYERS) ===")
            meta = deploy_smc_case.parse_meta(args.case_dir)
            cfgs = deploy_smc_case.build_layer_cfgs(meta)
            cy_list = []
            wall_list = []
            for i in range(args.repeat - 1):
                t0 = time.time()
                cy = rpc.run_layers(cfgs)
                t_rt = time.time() - t0
                cy_list.append(cy)
                wall_list.append(t_rt)
                print(f"  run #{i+1}: cy={cy} ({cy/100e6*1000:.2f} ms HW), "
                      f"wall {t_rt*1000:.1f} ms")
            if cy_list:
                avg_cy = sum(cy_list) / len(cy_list)
                avg_wall = sum(wall_list) / len(wall_list)
                print(f"\n  平均 HW {avg_cy/100e6*1000:.2f} ms ({100e6/avg_cy:.1f} FPS)")
                print(f"  平均 wall {avg_wall*1000:.1f} ms ({1.0/avg_wall:.1f} FPS, 含 RPC turnaround)")

    return 0 if stats.get('mm', 0) == 0 or args.no_verify else 1


if __name__ == '__main__':
    sys.exit(main())
