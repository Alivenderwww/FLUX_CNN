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

    def core_layer_desc_ddr(self, core_id, layer_idx, per_layer_size=0x4000):
        """Per-(core, layer) desc list base. 16 KB / (core, layer) 默认, 256 KB / core 上限."""
        return self.region['DESC'] + core_id * self.desc_offset + layer_idx * per_layer_size


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
# Mode C.2 W 切片几何在 hw_files.compute_w_slice_geom + derive_w_slice_cfg 内.
# 本文件只负责: per-core ifb_w_start/ofb_w_start 偏移 → IDMA/ODMA DDR 基址.
# ---------------------------------------------------------------------------
def build_step_cfg_dict(step, layers, ddr_planner, n_layers):
    """生成 step 对应 layer 的 cfg_dict (考虑 mode A / mode C 切片, push/DDR)."""
    layer = step.layer
    layer_idx = step.layer_idx
    n_split = len(step.cores_all)

    cin_slices  = (layer.c_in  + 16 - 1) // 16
    cout_slices = (layer.c_out + 16 - 1) // 16

    # 决定本核实际跑的 sub-layer 维度 + per-core IDMA/ODMA 偏移
    if step.mode == 'C_w_slice':
        cfg = hw_files.derive_w_slice_cfg(
            H_IN=layer.h_in, W_full=layer.w_in, K=layer.k,
            NUM_CIN=layer.c_in, NUM_COUT=layer.c_out, stride=layer.stride,
            pad_top=layer.pad, pad_left_full=layer.pad,
            my_core=step.my_core, n_split=n_split,
            TILE_W=32, streaming=True,
        )
        ifb_w_start = cfg['_W_SLICE_W_IN_START']
        ofb_w_start = cfg['_W_SLICE_W_OUT_START']
    elif step.mode == 'C_cout_slice':
        # cout 切: 每核完整 W_in/W_out 但 cout 段 — P2 再做.
        raise NotImplementedError("Cout slice 实现 P2 再做")
    else:  # mode A
        cfg = hw_files.derive_layer_cfg(
            H_IN=layer.h_in, W_IN=layer.w_in, K=layer.k,
            NUM_CIN=layer.c_in, NUM_COUT=layer.c_out,
            stride=layer.stride, pad_top=layer.pad, pad_left=layer.pad,
            TILE_W=32, streaming=True,
        )
        ifb_w_start = 0
        ofb_w_start = 0

    # IDMA SRC 跟 ODMA DST 的精确偏移
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

    return cfg_dict, cfg


# ---------------------------------------------------------------------------
# 给每核生成 per-layer desc list (host stage barrier 模式)
#   - 每 (core, layer) 一个独立 desc list, 末尾 END
#   - host 一层一层串: 写 DESC_LIST_BASE → start_dfe → start_layer → wait done
#   - W slice 跨核数据依赖通过 host barrier 解决: 上层所有核都写完 DDR OFM
#     后, 才让下层 IDMA 启动. 比 sequencer BARRIER (只看本核) 更安全.
# ---------------------------------------------------------------------------
def gen_core_per_layer_desc_lists(core_id, steps, layers, ddr_planner, n_layers):
    """
    返回 dict: layer_idx → list of (beat0, beat1) descs (每个独立 desc list, 末尾 END).
    若该 (core, layer) 在 plan 里没分到任务, key 不在 dict 中.
    """
    out = {}
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
        out[step.layer_idx] = seg
    return out


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

    # 4. 给每核生成 per-layer desc lists (host stage barrier 模式)
    print(f"\n[Step 2] Generating per-(core,layer) desc lists for {n_cores} cores...")
    # core_layer_descs[core_id][layer_idx] = n_descs (0 表示该 core 该层没活)
    core_layer_descs = [[0] * len(layers) for _ in range(n_cores)]
    for core_id in range(n_cores):
        steps = per_core_plan[core_id]
        per_layer_segs = gen_core_per_layer_desc_lists(core_id, steps, layers, ddr, len(layers))
        core_dir = os.path.join(out_dir, f'core{core_id}')
        os.makedirs(core_dir, exist_ok=True)
        for layer_idx, seg in per_layer_segs.items():
            desc_path = os.path.join(core_dir, f'layer{layer_idx:02d}_desc_list.hex')
            n_descs = hw_files.write_multilayer_desc_list(desc_path, [seg])
            core_layer_descs[core_id][layer_idx] = n_descs
        active_layers = [l for l in range(len(layers)) if core_layer_descs[core_id][l] > 0]
        print(f"  Core {core_id}: layers={active_layers}, "
              f"descs/layer={[core_layer_descs[core_id][l] for l in active_layers]}")

    # 5. 写 multicore_meta.txt 给 TB 用
    meta_path = os.path.join(out_dir, 'multicore_meta.txt')
    with open(meta_path, 'w') as f:
        f.write(f"NUM_CORES = {n_cores}\n")
        f.write(f"NUM_LAYERS = {len(layers)}\n")
        f.write(f"DDR_INPUT_BASE = 0x{ddr.region['INPUT']:08x}\n")
        f.write(f"DDR_FINAL_OFM_BASE = 0x{ddr.region['FINAL_OFM']:08x}\n")
        # 每 (core, layer) desc base + count
        for core_id in range(n_cores):
            for layer_idx in range(len(layers)):
                n_descs = core_layer_descs[core_id][layer_idx]
                if n_descs == 0:
                    continue   # 该核该层没活, 跳过
                base = ddr.core_layer_desc_ddr(core_id, layer_idx)
                f.write(f"CORE_{core_id}_LAYER_{layer_idx}_DESC_BASE = 0x{base:08x}\n")
                f.write(f"CORE_{core_id}_LAYER_{layer_idx}_DESC_COUNT = {n_descs}\n")
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
    parser.add_argument('--demo',
                        choices=['wslice1', 'simple2', 'simple3', 'wslice4', 'wslice5', 'wslice_mixed', 'resnet11'],
                        default='wslice1',
                        help='wslice4/5: 4/5 层 conv chain (W slice 验证) | wslice_mixed: 多 K/stride')
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
    elif args.demo == 'wslice5':
        # 5 层等尺寸 conv chain (W slice 多层验证, 无 stride / 无 residual)
        layers = [
            scheduler.Layer(f'L{i}', k=3, c_in=16, c_out=16,
                            h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2)
            for i in range(5)
        ]
    elif args.demo == 'wslice4':
        layers = [
            scheduler.Layer(f'L{i}', k=3, c_in=16, c_out=16,
                            h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2)
            for i in range(4)
        ]
    elif args.demo == 'wslice_mixed':
        # 混合 K (3, 5) / stride 1, 多层 W slice 验证 (linear chain 不变 H/W)
        # 注: stride>1 + 中间层不一致会让 chain 输入尺寸不匹配, 这里全部 stride=1
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L1', k=5, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=2, sdp_shift=3),
            scheduler.Layer('L2', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L3', k=1, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=0, sdp_shift=2),
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
