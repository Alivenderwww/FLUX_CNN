"""stress_test_resnet11.py — ResNet11 N=2 压力测试

一次性 preload (WB / RDMA / desc / sg_cmd 不变), 循环 N 次重 LOAD IFM + run + verify.
模拟实际推理流: 模型参数 load 一次, 每张新 image 只更新 IFM 跑.

Usage:
    python stress_test_resnet11.py --case-dir ../../sim/tb_smc/cases/resnet11_n2 \
        --iters 100 --vd100-ip 169.254.111.10
    # 加 --per-layer 每 iter 校验全部 11 层 OFM (慢但能定位中间层错误)
    # 加 --per-layer --per-layer-every K  只在 K 倍数 iter 做 per-layer 校验

输出:
    - 每次 iter 的 cycles + PASS/FAIL
    - 总结: PASS 率, cycles min/max/mean/stddev
    - per-layer 模式: 每层 PASS 计数 + 首次失败 iter/layer/diff
"""
import argparse
import json
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

import vd100_rpc
from vd100_rpc import Vd100Rpc, NUM_CORES
import deploy_smc_case as dep


def _layer_ofm_offsets(case_dir: str, n_layers: int, global_base: int) -> list:
    """从 manifest.json 抠每层 OFM 在每个 mem 内的偏移 (相对 global_base+core*stride).

    fallback: 找不到 manifest 时用公式 (layer < n-1: IFM_OFM_BASE + L*LAYER_DATA_OFFSET,
              last: FINAL_OFM_BASE).
    """
    manifest = os.path.join(case_dir, 'manifest.json')
    offsets = []
    if os.path.exists(manifest):
        with open(manifest, 'r', encoding='utf-8') as f:
            m = json.load(f)
        for layer in m['layers']:
            # 取 core_0 ofm_base (W slice 各 core 同 offset 写各自 mem)
            ob = layer['smc_layout']['core_0']['ofm_base']
            ob_int = int(ob, 0) if isinstance(ob, str) else int(ob)
            offsets.append(ob_int - global_base)
        return offsets
    # fallback 公式
    for l in range(n_layers):
        if l == n_layers - 1:
            offsets.append(dep.SMC_FINAL_OFM_BASE)
        else:
            offsets.append(dep.SMC_IFM_OFM_BASE + l * dep.SMC_LAYER_DATA_OFFSET)
    return offsets


def stress_test(case_dir: str, iters: int, vd100_ip: str, verbose: bool = False,
                per_layer: bool = False, per_layer_every: int = 1):
    rpc = Vd100Rpc(vd100_ip, 5000, timeout=90.0)
    rpc.connect()

    meta = dep.parse_meta(case_dir)
    n_layers = meta['NUM_LAYERS']
    n_cores  = meta['NUM_CORES']
    global_base = dep.derive_smc_global_base(meta)
    print(f"== ResNet11 stress test: {case_dir} ==")
    print(f"  n_layers={n_layers} n_cores={n_cores} global_base=0x{global_base:08x}")
    print(f"  iters={iters} per_layer={per_layer} per_layer_every={per_layer_every}")

    rpc.ping()

    # ---- ONCE: 跟 deploy_smc_case 一致流程 (含 layer 0 IFB), 验证基线 OK ----
    print(f"[1/2] One-time preload (跟 deploy_smc_case 流程一致) ...")
    t0 = time.time()
    for l in range(n_layers):
        is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
        if is_root:
            dep.load_ifb_smc(rpc, case_dir, meta, l, global_base)
        dep.load_wb_smc  (rpc, case_dir, meta, l, global_base)
        dep.load_rdma_smc(rpc, case_dir, meta, l, global_base)
        dep.load_desc_smc(rpc, case_dir, meta, l)
        dep.load_sg_cmd_smc(rpc, case_dir, meta, l)
    print(f"  one-time preload done in {time.time()-t0:.1f}s")

    # ---- expected OFM (一次性 load 跟 compare 用) ----
    last_l = n_layers - 1
    layer_cfgs = dep.build_layer_cfgs(meta)
    # 全部 11 层的 expected OFM (per-layer 校验用; 不开 per-layer 时只用 last)
    expected_per_layer = [bytes(dep.load_expected_ofm(case_dir, meta, l))
                          for l in range(n_layers)]
    ofm_offsets = _layer_ofm_offsets(case_dir, n_layers, global_base)
    print(f"  per-layer OFM offsets (relative to mem[i] base):")
    for l in range(n_layers):
        print(f"    layer {l:2d}: offset=0x{ofm_offsets[l]:08x} "
              f"expected_len={len(expected_per_layer[l])} byte")

    # ---- LOOP: 每次只重新 LOAD IFM + run + verify ----
    print(f"[2/2] {iters} iter loop (重 LOAD IFM + run_layers + read OFM + verify) ...")
    pass_count = 0
    fail_count = 0
    cycles_list = []
    first_fail_iter = None
    first_fail_info = None
    # per-layer 统计: 每层 PASS/FAIL/SKIP 计数
    layer_pass = [0] * n_layers
    layer_fail = [0] * n_layers
    layer_first_fail = [None] * n_layers   # (iter, n_diff, first_diff_byte, got, exp)

    # 找所有 root layer IDX (loop 里都要重 LOAD, 否则 layer 0 W slice 大 IFM 会覆盖
    # 其他 root layer IFB 区 — toolchain SMC_LAYER_INPUT_OFFSET=512KB 给 layer 0 1MB+ 数据不够)
    root_layers = [l for l in range(n_layers)
                    if l == 0 or meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1]
    print(f"  root layers to reload each iter: {root_layers}")

    t_total = time.time()
    for i in range(iters):
        # 重 LOAD 所有 root IFB. 第 0 次跳过 (preload 阶段已 LOAD)
        if i > 0:
            for rl in root_layers:
                dep.load_ifb_smc(rpc, case_dir, meta, rl, global_base)

        # run
        try:
            cycles = rpc.run_layers(layer_cfgs)
        except Exception as e:
            print(f"  iter {i}: RUN_ERR {type(e).__name__}: {e}")
            fail_count += 1
            if first_fail_iter is None:
                first_fail_iter = i
                first_fail_info = f"RUN_ERR: {e}"
            continue

        cycles_list.append(cycles)

        # ---- read + verify ----
        do_per_layer = per_layer and (i % per_layer_every == 0)
        layers_to_check = list(range(n_layers)) if do_per_layer else [last_l]
        iter_fail_layers = []   # 本 iter 哪些层失败

        for l in layers_to_check:
            got = dep.read_ofm_smc(rpc, meta, l, global_base, ofm_offsets[l])
            exp = expected_per_layer[l]
            n_diff = sum(1 for a, b in zip(got, exp) if a != b)
            if n_diff == 0:
                layer_pass[l] += 1
            else:
                layer_fail[l] += 1
                iter_fail_layers.append((l, n_diff, len(got)))
                if layer_first_fail[l] is None:
                    fdb = next(idx for idx, (a, b) in enumerate(zip(got, exp)) if a != b)
                    layer_first_fail[l] = (i, n_diff, len(got), fdb, got[fdb], exp[fdb])

        # 本 iter PASS/FAIL 判定 (任一 checked 层错 = FAIL)
        if not iter_fail_layers:
            pass_count += 1
            tag = "PL-PASS" if do_per_layer else "PASS"
            if verbose or i < 3 or i == iters - 1 or do_per_layer:
                print(f"  iter {i:3d}: {tag} cy={cycles}")
        else:
            fail_count += 1
            tag = "PL-FAIL" if do_per_layer else "FAIL"
            msg_parts = [f"L{l}={d}/{tot}" for l, d, tot in iter_fail_layers]
            print(f"  iter {i:3d}: {tag} cy={cycles} mismatch=[{', '.join(msg_parts)}]")
            if first_fail_iter is None:
                first_fail_iter = i
                first_fail_info = f"layers={iter_fail_layers}"

    t_wall = time.time() - t_total
    print()
    print(f"=== Summary ===")
    print(f"  Total iters:    {iters}")
    print(f"  PASS:           {pass_count} ({100*pass_count/iters:.1f}%)")
    print(f"  FAIL:           {fail_count} ({100*fail_count/iters:.1f}%)")
    if cycles_list:
        cy_min = min(cycles_list); cy_max = max(cycles_list)
        cy_mean = sum(cycles_list) / len(cycles_list)
        cy_var = sum((c - cy_mean)**2 for c in cycles_list) / len(cycles_list)
        cy_std = cy_var**0.5
        print(f"  Cycles:         min={cy_min} max={cy_max} mean={cy_mean:.0f} std={cy_std:.1f}")
        print(f"  Cycles jitter:  ±{(cy_max-cy_min)/2:.0f} ({100*(cy_max-cy_min)/cy_mean:.3f}%)")
    print(f"  Wall time:      {t_wall:.1f}s ({iters/t_wall:.1f} iter/s)")
    if first_fail_iter is not None:
        print(f"  First fail @ iter {first_fail_iter}: {first_fail_info}")

    if per_layer:
        print()
        print(f"=== Per-layer breakdown ===")
        print(f"  (checked every {per_layer_every} iter, ~{iters//per_layer_every} samples)")
        for l in range(n_layers):
            chk = layer_pass[l] + layer_fail[l]
            if chk == 0:
                continue
            status = "OK" if layer_fail[l] == 0 else "BAD"
            print(f"  layer {l:2d}: {status} PASS={layer_pass[l]}/{chk} FAIL={layer_fail[l]}", end='')
            if layer_first_fail[l] is not None:
                it, nd, tot, fdb, g, e = layer_first_fail[l]
                print(f" first_fail @ iter {it} {nd}/{tot} byte (byte[{fdb}] got=0x{g:02x} exp=0x{e:02x})", end='')
            print()

    rpc.close()
    return pass_count, fail_count, cycles_list


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case-dir', required=True)
    ap.add_argument('--iters', type=int, default=100)
    ap.add_argument('--vd100-ip', default='169.254.111.10')
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--per-layer', action='store_true',
                    help='每 iter 校验全部层 OFM (慢但能定位中间层错)')
    ap.add_argument('--per-layer-every', type=int, default=1,
                    help='--per-layer 时每 K 个 iter 做一次 per-layer 校验')
    args = ap.parse_args()
    p, f, _ = stress_test(args.case_dir, args.iters, args.vd100_ip,
                          args.verbose, args.per_layer, args.per_layer_every)
    return 0 if f == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
