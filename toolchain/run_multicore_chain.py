"""
run_multicore_chain.py  --  M2.5 多核多层流水线 driver

输入: 一组 Layer (chain) + N_cores
输出: 给每核生成 case 目录 (含 desc_list.hex 多 layer 串接) + DDR layout
      + multicore_meta.txt 给 TB 用 (NUM_CORES, layers, golden ofb base)

数据流:
  1. scheduler.schedule(layers, n_cores) → stages
  2. scheduler.gen_per_core_plan(stages) → per_core: dict[core_id, list[LayerStep]]
  3. 跑整个 chain 一次 (single-core 模式) 拿每层 IFM/WB/OFM 数据 cache
  4. 分配 DDR layout: 每 layer 一个区 (中间 OFM = 下层 IFM)
  5. 给每核每层算 cfg_dict (基于 LayerStep.input_from / output_to / push_*_core)
  6. 用 build_layer_desc_segment 拼每核的多 layer desc list

跑法:
  python run_multicore_chain.py --case_name resnet11 --n_cores 2
"""

import os
import sys
import shutil
import argparse
import random

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.dirname(_THIS_DIR))    # 项目根, 取 params.py

import gen_isa_test
import hw_files
import scheduler
from params import core_ifb_axi_base, IFB_DEPTH, OFB_DEPTH


# ---------------------------------------------------------------------------
# DDR Layout
# ---------------------------------------------------------------------------
class DDRPlanner:
    """
    DDR layout 规划:
      Layer 0 input:    DDR_INPUT_BASE
      Layer i 中间 OFM (=下层 IFM): DDR_LAYER_OFM_BASE_i
      共享 WB 段:       DDR_WB_BASE (每 layer 一段)
      共享 RDMA 段:     DDR_RDMA_BASE (每 layer 一段)
      共享 desc 段:     DDR_DESC_BASE (每核一段)
      最终 OFM (整网输出): DDR_FINAL_OFM_BASE
    """
    def __init__(self):
        # 紧凑分区 (axi_slave_mem AIDX_W=25 → 有效地址 32 MB byte = 0x2_000_000)
        # 所有 region 放在 32 MB 内
        self.region = {
            'INPUT'      : 0x0000_0000,   # 0-1MB  (1MB)
            'OFM_LAYER'  : 0x0010_0000,   # 1-13MB (12MB / 12 layers max, 1MB each)
            'WB'         : 0x0100_0000,   # 16-20MB (4MB / 16 layers, 256KB each)
            'RDMA'       : 0x0140_0000,   # 20-21MB (1MB)
            'DESC'       : 0x0150_0000,   # 21-25MB (4MB / 16 cores, 256KB each)
            'FINAL_OFM'  : 0x0190_0000,   # 25-26MB (1MB)
        }
        self.ofm_offset  = 0x0010_0000   # 1 MB / layer
        self.wb_offset   = 0x0004_0000   # 256 KB / layer
        self.rdma_offset = 0x0001_0000   # 64 KB / layer
        self.desc_offset = 0x0004_0000   # 256 KB / core

    def layer_input_ddr(self, layer_idx):
        """layer i 的 IFM 来源 DDR base. layer 0 = INPUT, 否则 = layer i-1 的 OFM."""
        if layer_idx == 0:
            return self.region['INPUT']
        return self.region['OFM_LAYER'] + (layer_idx - 1) * self.ofm_offset

    def layer_output_ddr(self, layer_idx, n_layers):
        """layer i 的 OFM 写到 DDR base. last layer 写 FINAL_OFM, 否则 OFM_LAYER 区."""
        if layer_idx == n_layers - 1:
            return self.region['FINAL_OFM']
        return self.region['OFM_LAYER'] + layer_idx * self.ofm_offset

    def layer_wb_ddr(self, layer_idx):
        return self.region['WB'] + layer_idx * self.wb_offset

    def layer_rdma_ddr(self, layer_idx):
        return self.region['RDMA'] + layer_idx * self.rdma_offset

    def core_desc_ddr(self, core_id):
        return self.region['DESC'] + core_id * self.desc_offset


# ---------------------------------------------------------------------------
# 跑 chain 拿每层 IFM/WB/OFM 数据 (single-core 模式, 用作 DDR 数据)
# ---------------------------------------------------------------------------
def run_chain_data_gen(layers, out_dir, ddr_planner, seed_base=42):
    """
    给每层调 gen_isa_test.generate_random 跑一次, 收集每层数据.
    返回 list of dict, 每 dict 有 'ifb' / 'wb' / 'ofm' / 'expected_ofm' / 'cfg_dict' / 'rdma' 等.
    生成后还把 ifb/wb/expected_ofm 写到对应 DDR 区域文件 (TB 用 $readmemh 加载).
    """
    chain_data = []
    prev_ofm = None        # layer i-1 的 OFM, 给 layer i 当 IFM

    for i, layer in enumerate(layers):
        layer_dir = os.path.join(out_dir, f'layer{i:02d}')
        if os.path.exists(layer_dir):
            shutil.rmtree(layer_dir)
        os.makedirs(layer_dir, exist_ok=True)

        full_name = f"L{i}_{layer.name}|conv"
        ret = gen_isa_test.generate_random(
            H_IN=layer.h_in, W_IN=layer.w_in, K=layer.k,
            NUM_CIN=layer.c_in, NUM_COUT=layer.c_out,
            TILE_W=16, seed=seed_base + i,
            shift_amt=layer.sdp_shift, stride=layer.stride,
            HW_PE=16, HW_COL=16, streaming=True,
            pad_top=layer.pad, pad_left=layer.pad, strip_rows=0,
            out_dir=layer_dir, case_name=full_name,
            ky_fold=False, s2d=False, residual=False,
            ifm_arr_in=prev_ofm,
            sdp_mult=1, sdp_zp_out=0,
            sdp_clip_min=0, sdp_clip_max=255,
            sdp_round_en=0, sdp_relu_en=1,
            ddr_ifb_base =ddr_planner.layer_input_ddr(i),
            ddr_wb_base  =ddr_planner.layer_wb_ddr(i),
            ddr_ofb_base =ddr_planner.layer_output_ddr(i, len(layers)),
            ddr_rdma_base=ddr_planner.layer_rdma_ddr(i),
            skip_ifb_preload=False, skip_ofb_clear=False,
        )
        prev_ofm = ret['ofm_arr']
        chain_data.append({
            'layer_idx' : i,
            'layer_dir' : layer_dir,
            'h_out'     : ret['H_OUT'],
            'w_out'     : ret['W_OUT'],
            'ifb_words' : ret['ifb_words'],
            'wb_words'  : ret['wb_words'],
            'ofb_words' : ret['ofb_words'],
        })
        print(f"  Layer {i:>2} {layer.name:<14} {layer.h_in}x{layer.w_in}x{layer.c_in} -> "
              f"{ret['H_OUT']}x{ret['W_OUT']}x{layer.c_out}  ifb={ret['ifb_words']} wb={ret['wb_words']} ofb={ret['ofb_words']}")

    return chain_data


# ---------------------------------------------------------------------------
# 给单核单层算 cfg_dict (考虑 mode A/C, push/DDR, W 切片含 halo)
#
# Mode C.2 W 切片下的 cfg 计算 (computed redundancy halo, 每核多算 halo 列):
#   原 layer: H × W_full × c_in -> H_out × W_out_full × c_out
#   切到 N 核, halo = (K-1)/2, 每核负责 W_out 段:
#     core i 主要 W_out 段:  [i*W/N, (i+1)*W/N)
#     core i 含 halo W_in 段: [i*W/N - halo, (i+1)*W/N + halo) (clip 到 [0, W))
#   每核独立跑这个 sub-conv. DDR_*_ROW_STRIDE 仍是全图 W stride.
# ---------------------------------------------------------------------------
def compute_w_slice_geometry(layer, my_core, n_split):
    """返回 (w_in_local, w_out_local, w_in_start, w_out_start, pad_left_local, pad_right_local)."""
    # 期望 W_out 段
    w_out_full = layer.w_out
    w_out_per_core = w_out_full // n_split
    w_out_start = my_core * w_out_per_core
    w_out_end = (my_core + 1) * w_out_per_core if my_core < n_split - 1 else w_out_full
    w_out_local = w_out_end - w_out_start

    # W_in 段含 halo:
    # 反推: W_in 索引 = stride * W_out 索引 + (kx - pad_left)
    # 每核 W_in 起点 = stride * w_out_start - pad_left (整层 pad)
    # 每核 W_in 终点 = stride * (w_out_end - 1) + (K - pad_left - 1) + 1 = stride * w_out_end + K - pad_left - stride
    pad_left_full = layer.pad
    pad_right_full = layer.pad

    w_in_start_unbounded = layer.stride * w_out_start - pad_left_full
    w_in_end_unbounded   = layer.stride * (w_out_end - 1) + layer.k - pad_left_full

    # Clip 到 [0, layer.w_in), 计算实际 pad
    if w_in_start_unbounded < 0:
        pad_left_local = -w_in_start_unbounded
        w_in_start = 0
    else:
        pad_left_local = 0
        w_in_start = w_in_start_unbounded

    if w_in_end_unbounded > layer.w_in:
        pad_right_local = w_in_end_unbounded - layer.w_in
        w_in_end = layer.w_in
    else:
        pad_right_local = 0
        w_in_end = w_in_end_unbounded

    w_in_local = w_in_end - w_in_start

    return {
        'w_in_local'      : w_in_local,
        'w_out_local'     : w_out_local,
        'w_in_start'      : w_in_start,
        'w_out_start'     : w_out_start,
        'pad_left_local'  : pad_left_local,
        'pad_right_local' : pad_right_local,
        'w_in_full'       : layer.w_in,
        'w_out_full'      : layer.w_out,
    }


def build_step_cfg_dict(step, layers, ddr_planner, n_layers):
    """生成 step 对应 layer 的 cfg_dict (考虑 mode A / mode C 切片, push/DDR)."""
    layer = step.layer
    layer_idx = step.layer_idx

    # 决定本核实际跑的 sub-layer 维度
    if step.mode == 'C_w_slice':
        geom = compute_w_slice_geometry(layer, step.my_core, len(step.cores_all))
        my_w_in       = geom['w_in_local']
        my_w_out      = geom['w_out_local']
        my_pad_left   = geom['pad_left_local']
        my_pad_right_unused = geom['pad_right_local']  # gen_isa_test 用 pad_left 单值
        my_pad_top    = layer.pad
        # IDMA SRC 偏到自己 W 段
        ifb_w_start = geom['w_in_start']
        # ODMA DST 偏到自己 W_out 段
        ofb_w_start = geom['w_out_start']
    elif step.mode == 'C_cout_slice':
        # cout 切: 每核完整 W_in/W_out 但 cout 段
        my_w_in    = layer.w_in
        my_w_out   = layer.w_out
        my_pad_left = layer.pad
        my_pad_top  = layer.pad
        ifb_w_start = 0
        ofb_w_start = 0
        # cout segment 处理: TBD (P1 先做 W slice, cout slice 是 P2)
        raise NotImplementedError("Cout slice 实现 P2 再做")
    else:  # mode A
        my_w_in    = layer.w_in
        my_w_out   = layer.w_out
        my_pad_left = layer.pad
        my_pad_top  = layer.pad
        ifb_w_start = 0
        ofb_w_start = 0

    # 派生 sub-layer cfg.
    # W slice 下 derive 不支持 asymmetric pad, 我们用 (W_IN + halo*2) + symmetric_pad
    # 让 derive 算出 W_OUT 跟我们期望的 my_w_out 一致.
    # Trick: 通过 pad_left = halo 让 derive 把 halo 部分当 pad 算掉.
    # 但更简单: 直接 override cfg 字段.
    cfg = hw_files.derive_layer_cfg(
        H_IN=layer.h_in, W_IN=my_w_in, K=layer.k,
        NUM_CIN=layer.c_in, NUM_COUT=layer.c_out,
        stride=layer.stride, pad_top=my_pad_top, pad_left=my_pad_left,
        TILE_W=min(16, my_w_out), streaming=True,
    )
    # Override: W slice 下 my_w_out 来自 geometry 计算 (含 asymmetric pad)
    if step.mode == 'C_w_slice':
        cfg['W_OUT'] = my_w_out
        # pad_right 需要根据 geometry 设 (asymmetric)
        cfg['pad_right'] = my_pad_right_unused
        # 重新计算 W slice 下的 derived 字段
        cfg['num_tiles'] = (my_w_out + cfg['TILE_W'] - 1) // cfg['TILE_W']
        cfg['last_valid_w'] = my_w_out - (cfg['num_tiles'] - 1) * cfg['TILE_W']
        # ofb_words 重算
        cfg['ofb_words'] = layer.h_out * my_w_out * cfg['cout_slices']

    # IDMA SRC 跟 ODMA DST 的精确偏移
    # 注: cin_slices 用 layer 原值 (因为 cin 没切)
    cin_slices  = (layer.c_in  + 16 - 1) // 16
    cout_slices = (layer.c_out + 16 - 1) // 16

    if step.input_from == 'push':
        idma_src = core_ifb_axi_base(step.my_core)  # 自己核 IFB region (push 进来)
    else:
        idma_src = ddr_planner.layer_input_ddr(layer_idx) + ifb_w_start * cin_slices * 16

    if step.output_to == 'push':
        odma_dst = core_ifb_axi_base(step.push_to_core)
    else:
        odma_dst = ddr_planner.layer_output_ddr(layer_idx, n_layers) + ofb_w_start * cout_slices * 16

    # WB 全核共享 (W slice 下 weight 完全相同; cout slice 下需要切 cout 段, 这里 W slice 暂不处理)
    wb_base = ddr_planner.layer_wb_ddr(layer_idx)

    cfg_dict = hw_files.cfg_to_dict(
        cfg, shift_amt=layer.sdp_shift,
        sdp_mult=1, sdp_zp_out=0,
        sdp_clip_min=0, sdp_clip_max=255,
        sdp_round_en=0, sdp_relu_en=1,
        case_name=f"L{layer_idx}_{layer.name}_c{step.my_core}",
        ddr_ifb_base =idma_src,
        ddr_wb_base  =wb_base,
        ddr_ofb_base =odma_dst,
        ddr_rdma_base=ddr_planner.layer_rdma_ddr(layer_idx),
        skip_ifb_preload=False, skip_ofb_clear=False,
        skip_idma=step.skip_idma,
    )

    # W slice 下覆写 DDR_*_ROW_STRIDE 用全图 W (不是 my_w_in/out)
    if step.mode == 'C_w_slice':
        cfg_dict['DDR_IFM_ROW_STRIDE'] = layer.w_in  * cin_slices  * 16
        cfg_dict['DDR_OFM_ROW_STRIDE'] = layer.w_out * cout_slices * 16

    return cfg_dict, cfg


# ---------------------------------------------------------------------------
# 给每核生成 desc list
# ---------------------------------------------------------------------------
def gen_core_desc_list(core_id, steps, layers, ddr_planner, n_layers):
    """生成核的多 layer desc list (concat with BARRIER), 返回 list of (beat0, beat1)."""
    layer_segments = []
    for step in steps:
        cfg_dict, cfg = build_step_cfg_dict(step, layers, ddr_planner, n_layers)
        seg = hw_files.build_layer_desc_segment(
            cfg_dict,
            H_IN=cfg['H_IN'], W_IN=cfg['W_IN'],
            H_OUT=cfg['H_OUT'], W_OUT=cfg['W_OUT'],
            cin_slices=cfg['cin_slices'], cout_slices=cfg['cout_slices'],
            pad_top=cfg['pad_top'], pad_bot=cfg['pad_bot'],
            pad_left=cfg['pad_left'], pad_right=cfg['pad_right'],
            strip_rows=0,
        )
        layer_segments.append(seg)
    return layer_segments


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def run_multicore_chain(layers, n_cores, out_dir):
    """生成多核多层 chain 的 case 目录 + desc list."""

    # 1. Schedule
    print(f"\n=== Multicore chain: {len(layers)} layers, N_cores={n_cores} ===")
    stages = scheduler.schedule(layers, n_cores)
    per_core_plan = scheduler.gen_per_core_plan(stages, n_cores)
    scheduler.print_per_core_plan(per_core_plan)

    # 2. DDR layout
    ddr = DDRPlanner()

    # 3. Chain 数据生成 (single-core 模式跑一次)
    print("\n[Step 1] Generating chain data (single-core run)...")
    chain_data_dir = os.path.join(out_dir, 'chain_data')
    os.makedirs(chain_data_dir, exist_ok=True)
    chain_data = run_chain_data_gen(layers, chain_data_dir, ddr)

    # 4. 给每核生成 desc list
    print(f"\n[Step 2] Generating per-core desc lists for {n_cores} cores...")
    core_desc_counts = {}
    for core_id in range(n_cores):
        steps = per_core_plan[core_id]
        layer_segments = gen_core_desc_list(core_id, steps, layers, ddr, len(layers))
        core_dir = os.path.join(out_dir, f'core{core_id}')
        os.makedirs(core_dir, exist_ok=True)
        desc_path = os.path.join(core_dir, 'desc_list.hex')
        n_descs = hw_files.write_multilayer_desc_list(desc_path, layer_segments)
        core_desc_counts[core_id] = n_descs
        print(f"  Core {core_id}: {len(steps)} layers, {n_descs} descs (incl. {len(steps)-1} BARRIER + 1 END)")

    # 5. 写 multicore_meta.txt 给 TB 用
    meta_path = os.path.join(out_dir, 'multicore_meta.txt')
    with open(meta_path, 'w') as f:
        f.write(f"NUM_CORES = {n_cores}\n")
        f.write(f"NUM_LAYERS = {len(layers)}\n")
        f.write(f"DDR_INPUT_BASE = 0x{ddr.region['INPUT']:08x}\n")
        f.write(f"DDR_FINAL_OFM_BASE = 0x{ddr.region['FINAL_OFM']:08x}\n")
        for core_id in range(n_cores):
            steps = per_core_plan[core_id]
            f.write(f"CORE_{core_id}_DESC_BASE = 0x{ddr.core_desc_ddr(core_id):08x}\n")
            f.write(f"CORE_{core_id}_N_LAYERS = {len(steps)}\n")
            f.write(f"CORE_{core_id}_DESC_COUNT = {core_desc_counts[core_id]}\n")
        # final layer 信息 (TB 比对用)
        last_layer = layers[-1]
        f.write(f"FINAL_H_OUT = {last_layer.h_out}\n")
        f.write(f"FINAL_W_OUT = {last_layer.w_out}\n")
        f.write(f"FINAL_C_OUT = {last_layer.c_out}\n")
        # data file 路径
        for i, cd in enumerate(chain_data):
            f.write(f"LAYER_{i}_DIR = {cd['layer_dir']}\n")
            f.write(f"LAYER_{i}_IFB_WORDS = {cd['ifb_words']}\n")
            f.write(f"LAYER_{i}_WB_WORDS = {cd['wb_words']}\n")
            f.write(f"LAYER_{i}_OFB_WORDS = {cd['ofb_words']}\n")
            f.write(f"LAYER_{i}_DDR_IFB = 0x{ddr.layer_input_ddr(i):08x}\n")
            f.write(f"LAYER_{i}_DDR_WB = 0x{ddr.layer_wb_ddr(i):08x}\n")
            f.write(f"LAYER_{i}_DDR_OFB = 0x{ddr.layer_output_ddr(i, len(layers)):08x}\n")
            f.write(f"LAYER_{i}_DDR_RDMA = 0x{ddr.layer_rdma_ddr(i):08x}\n")
    print(f"\n  Meta written to {meta_path}")

    return chain_data, per_core_plan


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case_name', default='multicore_demo',
                        help='输出目录名 (在 sim/tb_multicore/cases 下)')
    parser.add_argument('--n_cores', type=int, default=2)
    parser.add_argument('--demo', choices=['wslice1', 'simple2', 'simple3', 'resnet11'], default='wslice1',
                        help='simple3: 3 层 chain demo / resnet11: 11 层 ResNet')
    args = parser.parse_args()

    if args.demo == 'wslice1':
        # 单层 W slice 验证 (N=2 切 W=32)
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'simple2':
        # 2 层 conv chain (P0 minimal), 每核一层
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L1', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'simple3':
        # 3 层 conv chain (会触发同核多 layer, DDR 模式下跨核同步问题, P0 不验证)
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L1', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L2', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'resnet11':
        from run_regression import CASES
        layers = scheduler.chain_to_layers(CASES[:11])

    out_root = os.path.join(_THIS_DIR, '..', 'sim', 'tb_multicore', 'cases', args.case_name)
    if os.path.exists(out_root):
        shutil.rmtree(out_root)
    os.makedirs(out_root, exist_ok=True)

    chain_data, per_core_plan = run_multicore_chain(layers, args.n_cores, out_root)

    print(f"\n=== Done. Output: {out_root} ===")
