#!/usr/bin/env python3
"""Stage 7 Multi-Image Batch Inference — N 张图片连续跑 Mini ResNet chain.

模拟真 ResNet 端侧推理: 多张图片连续推理, 验证:
  - 跨 image 软 reset 隔离 (image 间没数据污染)
  - throughput (image / sec)
  - 多 image 多 layer 复合 stress 下稳定性

设计: 5 张图片 × 5 层 chain = 25 次 layer run.
"""
import sys, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import gen_isa_test
from test_stage5_chain_bitexact import setup_case_for_board, run_board_layer

# 5-layer mini ResNet chain (各层 shape 不同)
LAYERS = [
    ('L0', 3, 16, 16, 8,  16, 1, 1),
    ('L1', 1, 16, 16, 16, 16, 1, 0),
    ('L2', 3, 16, 16, 16, 32, 1, 1),
    ('L3', 3, 16, 16, 32, 32, 2, 1),   # downsample to H=8
    ('L4', 3, 8,  8,  32, 32, 1, 1),
]

N_IMAGES = 5


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print(f'=== Stage 7 Multi-Image Batch Inference: {N_IMAGES} images × {len(LAYERS)} layers ===')

    img_total_t = 0
    pass_imgs = 0
    for img_idx in range(N_IMAGES):
        print(f'\n--- Image {img_idx} ---')
        prev_ofm = None
        all_layers_ok = True
        t_img_start = time.perf_counter()
        for l_idx, (name, K, H, W, Cin, Cout, stride, pad) in enumerate(LAYERS):
            case_dir = rf'C:/_Project/FLUX_CNN/sim/tb_batch_img{img_idx}/L{l_idx}'
            # 每 image 用不同 seed (模拟不同图片), 但 weight 用同一 seed (模拟同一模型)
            ifm_seed = img_idx * 1000 + l_idx  # image-specific IFM
            w_seed = l_idx                     # layer-specific weight (cross-image 一致)

            # 实际上 gen_isa_test seed 同时控制 IFM 和 W. 我们 chain mode 用 ifm_arr_in 替代
            # IFM, 所以 weight 可以保持 layer-specific seed.
            ret = gen_isa_test.generate_random(
                out_dir=case_dir, ifm_arr_in=prev_ofm,
                seed=w_seed if prev_ofm is not None else ifm_seed,
                shift_amt=0, streaming=False,
                H_IN=H, W_IN=W, K=K, NUM_CIN=Cin, NUM_COUT=Cout,
                TILE_W=32, stride=stride, pad_top=pad, pad_left=pad)

            desc, n_desc, cfg, ifm, wb, rdma, exp, isg, osg = setup_case_for_board(
                case_dir, K=K, H_IN=H, W_IN=W, NUM_CIN=Cin, NUM_COUT=Cout,
                stride=stride, pad=pad)

            ok, board_ofm, err = run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, exp, isg, osg)
            if not ok:
                all_layers_ok = False
                if board_ofm:
                    diff = sum(1 for a, b in zip(board_ofm, exp) if a != b)
                    print(f'  [Img {img_idx} L{l_idx}] FAIL mismatch {diff}/{len(exp)}')
                else:
                    print(f'  [Img {img_idx} L{l_idx}] FAIL {err}')
                break
            prev_ofm = ret['ofm_arr']
        t_img = (time.perf_counter() - t_img_start)
        img_total_t += t_img
        if all_layers_ok:
            pass_imgs += 1
            print(f'  Image {img_idx} all 5 layers PASS  total {t_img*1000:.1f} ms')

    avg_t = img_total_t / N_IMAGES
    print(f'\n=== Summary ===')
    print(f'  {pass_imgs}/{N_IMAGES} images all-layer PASS')
    print(f'  Total time {img_total_t*1000:.1f} ms, avg {avg_t*1000:.1f} ms/image, '
          f'~{1/avg_t:.1f} img/sec (含 host overhead)')

    rpc.close()
    return 0 if pass_imgs == N_IMAGES else 1


if __name__ == '__main__':
    sys.exit(main())
