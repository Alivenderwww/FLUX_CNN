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
        # 大分区 (axi_slave_mem DDR_DEPTH = 32M words × 16 byte/word = 512 MB byte 容量)
        # ResNet Patch 输入 960×540×4 ≈ 8.3 MB → INPUT region 必须 ≥ 16 MB.
        # OFM 区跟 RDMA 区也变大: residual 层 shortcut 数据 ≈ H_OUT*W_OUT*cout_slices*16 byte,
        # ResNet L1.B2.ds shortcut = 120*68*1*16 ≈ 130 KB, 64 KB region 不够; bump 到 1 MB / layer.
        self.region = {
            'INPUT'      : 0x0000_0000,   # 0-16MB  (16MB) — 大输入 (整网入口)
            'OFM_LAYER'  : 0x0100_0000,   # 16-48MB (32MB, 16 层 × 2MB)
            'WB'         : 0x0300_0000,   # 48-56MB (8MB, 16 层 × 512KB)
            'RDMA'       : 0x0380_0000,   # 56-72MB (16MB, 16 层 × 1MB / 含 residual shortcut)
            'DESC'       : 0x0480_0000,   # 72-80MB (8MB, 16 核 × 512KB)
            'FINAL_OFM'  : 0x0500_0000,   # 80-82MB (2MB)
            'ROOT_IFB'   : 0x0510_0000,   # 81MB+ (16MB / 16 根 layer × 1MB) — ResNet FC 等 root layer
        }
        self.ofm_offset  = 0x0020_0000   # 2 MB / layer
        self.wb_offset   = 0x0008_0000   # 512 KB / layer
        self.rdma_offset = 0x0010_0000   # 1 MB / layer (residual shortcut data 大)
        self.desc_offset = 0x0008_0000   # 512 KB / core
        self.root_ifb_offset = 0x0010_0000  # 1 MB / root layer

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

    def core_layer_desc_ddr(self, core_id, layer_idx, per_layer_size=0x8000):
        """Per-(core, layer) desc list base. 32 KB / (core, layer) 默认, 512 KB / core 上限."""
        return self.region['DESC'] + core_id * self.desc_offset + layer_idx * per_layer_size

    def root_ifb_ddr(self, root_idx):
        """Root layer (input_src='' 且非链式) 的 IFB region.
        i 个 root layer 顺序分配, 跟 OFM_LAYER 区互不重叠.
        """
        return self.region['ROOT_IFB'] + root_idx * self.root_ifb_offset

    def core_layer_rdma_ddr(self, core_id, layer_idx, n_cores=4):
        """Per-(core, layer) RDMA region base. residual + W slice 时每核 rdma_data 不同 (sliced shortcut).
        layout: layer_rdma_ddr(layer) + core_id * (rdma_offset / n_cores).
        """
        per_core = self.rdma_offset // max(n_cores, 1)
        return self.region['RDMA'] + layer_idx * self.rdma_offset + core_id * per_core


# ---------------------------------------------------------------------------
# 跑 chain 拿每层 IFM/WB/OFM 数据 (single-core 模式, 用作 DDR 数据)
# ---------------------------------------------------------------------------
def run_chain_data_gen(layers, out_dir, ddr_planner, seed_base=42):
    """
    给每层调 gen_isa_test.generate_random 跑一次, 收集每层数据.

    ResNet block 不是线性 chain — 用 layer.input_src / layer.shortcut_src 按名字
    在 ofm_dict 里查输入跟 shortcut. 维持 prev_ofm 兼容老 demo (input_src=='' 时
    fallback 为 prev_ofm).
    """
    chain_data = []
    prev_ofm = None              # 老 demo 兼容: input_src=='' 时用 prev (除了第一层)
    ofm_dict = {}                # layer.name → ofm_arr (ResNet 用)

    for i, layer in enumerate(layers):
        layer_dir = os.path.join(out_dir, f'layer{i:02d}')
        if os.path.exists(layer_dir):
            shutil.rmtree(layer_dir)
        os.makedirs(layer_dir, exist_ok=True)

        # ifm 来源:
        #   input_src 非空 → 按名字查 ofm_dict (ResNet block / 链式 case)
        #   input_src 空 → 当 root layer (gen 用 random ifm, 跟 run_regression _chain.root 语义一致)
        #   兼容老 demo (没设 input_src 的 simple3 等): i>0 时用 prev_ofm
        if layer.input_src:
            if layer.input_src not in ofm_dict:
                raise ValueError(f"layer {layer.name}: input_src='{layer.input_src}' "
                                 f"not in ofm_dict {list(ofm_dict.keys())}")
            ifm_arr_in = ofm_dict[layer.input_src]
        elif i == 0:
            ifm_arr_in = None
        elif layer.h_in != (chain_data[-1]['h_out'] if chain_data else 0) \
             or layer.w_in != (chain_data[-1]['w_out'] if chain_data else 0):
            # 维度不匹配 prev → 当 root, 用 random ifm (e.g. ResNet FC 接 GAP/Flatten)
            ifm_arr_in = None
        else:
            ifm_arr_in = prev_ofm

        # shortcut: shortcut_src 给定就在 ofm_dict 里查
        shortcut_arr_in = None
        residual = False
        if layer.shortcut_src:
            if layer.shortcut_src not in ofm_dict:
                raise ValueError(f"layer {layer.name}: shortcut_src='{layer.shortcut_src}' "
                                 f"not in ofm_dict {list(ofm_dict.keys())}")
            shortcut_arr_in = ofm_dict[layer.shortcut_src]
            residual = True

        full_name = f"L{i}_{layer.name}|conv"
        ret = gen_isa_test.generate_random(
            H_IN=layer.h_in, W_IN=layer.w_in, K=layer.k,
            NUM_CIN=layer.c_in, NUM_COUT=layer.c_out,
            TILE_W=16, seed=seed_base + i,
            shift_amt=layer.sdp_shift, stride=layer.stride,
            HW_PE=16, HW_COL=16, streaming=True,
            pad_top=layer.pad, pad_left=layer.pad, strip_rows=0,
            out_dir=layer_dir, case_name=full_name,
            ky_fold=False, s2d=layer.force_s2d, residual=residual,
            ifm_arr_in=ifm_arr_in,
            shortcut_arr_in=shortcut_arr_in,
            shortcut_mult=layer.shortcut_mult, shortcut_shift=layer.shortcut_shift,
            sdp_mult=layer.sdp_mult, sdp_zp_out=layer.sdp_zp_out,
            sdp_clip_min=layer.sdp_clip_min, sdp_clip_max=layer.sdp_clip_max,
            sdp_round_en=layer.sdp_round_en, sdp_relu_en=layer.sdp_relu_en,
            ddr_ifb_base =ddr_planner.layer_input_ddr(i),
            ddr_wb_base  =ddr_planner.layer_wb_ddr(i),
            ddr_ofb_base =ddr_planner.layer_output_ddr(i, len(layers)),
            ddr_rdma_base=ddr_planner.layer_rdma_ddr(i),
            skip_ifb_preload=False, skip_ofb_clear=False,
        )
        prev_ofm = ret['ofm_arr']
        ofm_dict[layer.name] = ret['ofm_arr']
        # ifb_is_root: 该 layer IFB 是 gen 用 random 现造的, TB 需要 preload (而不是依赖
        # 上一层 ODMA 写). 整网入口 (i==0) + ResNet FC 这种维度跟上层不匹配的 root layer 都是.
        ifb_is_root = (ifm_arr_in is None)
        chain_data.append({
            'layer_idx'      : i,
            'layer_dir'      : layer_dir,
            'h_out'          : ret['H_OUT'],
            'w_out'          : ret['W_OUT'],
            'ifb_words'      : ret['ifb_words'],
            'wb_words'       : ret['wb_words'],
            'ofb_words'      : ret['ofb_words'],
            'rdma_words'     : ret.get('rdma_words', layer.cout_slices * 4),
            # 保留 shortcut_arr 给 W slice + residual 用 (后处理切片)
            'shortcut_arr'   : shortcut_arr_in,
            'has_residual'   : residual,
            'ifb_is_root'    : ifb_is_root,
        })
        residual_tag = f' (+res from {layer.shortcut_src})' if residual else ''
        print(f"  Layer {i:>2} {layer.name:<14} {layer.h_in}x{layer.w_in}x{layer.c_in} -> "
              f"{ret['H_OUT']}x{ret['W_OUT']}x{layer.c_out}  ifb={ret['ifb_words']} "
              f"wb={ret['wb_words']} ofb={ret['ofb_words']}{residual_tag}")

    return chain_data


# ---------------------------------------------------------------------------
# 给单核单层算 cfg_dict (考虑 mode A/C, push/DDR, W 切片含 halo)
#
# Mode C.2 W 切片几何在 hw_files.compute_w_slice_geom + derive_w_slice_cfg 内.
# 本文件只负责: per-core ifb_w_start/ofb_w_start 偏移 → IDMA/ODMA DDR 基址.
# ---------------------------------------------------------------------------
def build_step_cfg_dict(step, layers, ddr_planner, n_layers,
                         name_to_idx=None, chain_data=None,
                         core_layer_rdma_words=None, n_cores_total=None,
                         root_ifb_idx=None):
    """生成 step 对应 layer 的 cfg_dict (考虑 mode A / mode C 切片, push/DDR).

    name_to_idx: layer.name → list index (用 layer.input_src / shortcut_src 查 DDR 区).
    chain_data:  list of dicts from run_chain_data_gen.
    core_layer_rdma_words: 二维数组, 非 0 表示该 (core, layer) 用 sliced rdma 文件.
    root_ifb_idx: layer_idx → root slot, 给 ifb_is_root 层用 ROOT_IFB 区 (避开上层 OFM).
    """
    layer = step.layer
    layer_idx = step.layer_idx
    n_split = len(step.cores_all)

    # S2D 等效维度: force_s2d 时用 s2d 后的 H/W/K/c_in/stride 算 cfg, 不变就用原值.
    # gen_isa_test 内部也做同 s2d 重排, 两边维度必须一致 → DDR 数据 ↔ cfg 才匹配.
    h_in_e, w_in_e, k_e, c_in_e, stride_e = layer.s2d_eff()
    pad_e = 0 if layer.force_s2d else layer.pad

    cin_slices  = (c_in_e      + 16 - 1) // 16
    cout_slices = (layer.c_out + 16 - 1) // 16

    # 决定本核实际跑的 sub-layer 维度 + per-core IDMA/ODMA 偏移.
    # step.my_core 是物理核 ID (e.g. 2), derive_w_slice_cfg 要的是切片 index (0..n_split-1).
    # 物理 → 切片 index: cores_all.index(my_core).
    if step.mode == 'C_w_slice':
        slice_idx = step.cores_all.index(step.my_core)
        cfg = hw_files.derive_w_slice_cfg(
            H_IN=h_in_e, W_full=w_in_e, K=k_e,
            NUM_CIN=c_in_e, NUM_COUT=layer.c_out, stride=stride_e,
            pad_top=pad_e, pad_left_full=pad_e,
            my_core=slice_idx, n_split=n_split,
            TILE_W=32, streaming=True,
        )
        ifb_w_start = cfg['_W_SLICE_W_IN_START']
        ofb_w_start = cfg['_W_SLICE_W_OUT_START']
    elif step.mode == 'C_cout_slice':
        # cout 切: 每核完整 W_in/W_out 但 cout 段 — P2 再做.
        raise NotImplementedError("Cout slice 实现 P2 再做")
    else:  # mode A
        cfg = hw_files.derive_layer_cfg(
            H_IN=h_in_e, W_IN=w_in_e, K=k_e,
            NUM_CIN=c_in_e, NUM_COUT=layer.c_out,
            stride=stride_e, pad_top=pad_e, pad_left=pad_e,
            TILE_W=32, streaming=True,
        )
        ifb_w_start = 0
        ofb_w_start = 0

    # IDMA SRC:
    #   push: 自己核 IFB region
    #   input_src 非空: 上游 layer OFM 区
    #   ifb_is_root (i>0): 独立 ROOT_IFB slot
    #   layer 0 / 线性链: layer_input_ddr(i)
    if step.input_from == 'push':
        idma_src = core_ifb_axi_base(step.my_core)  # 自己核 IFB region (push 进来)
    else:
        if layer.input_src and name_to_idx is not None and layer.input_src in name_to_idx:
            src_idx = name_to_idx[layer.input_src]
            idma_base = ddr_planner.layer_output_ddr(src_idx, n_layers)
        elif root_ifb_idx is not None and layer_idx in root_ifb_idx:
            idma_base = ddr_planner.root_ifb_ddr(root_ifb_idx[layer_idx])
        else:
            idma_base = ddr_planner.layer_input_ddr(layer_idx)
        idma_src = idma_base + ifb_w_start * cin_slices * 16

    if step.output_to == 'push':
        odma_dst = core_ifb_axi_base(step.push_to_core)
    else:
        odma_dst = ddr_planner.layer_output_ddr(layer_idx, n_layers) + ofb_w_start * cout_slices * 16

    # WB 全核共享 (W slice 下 weight 完全相同; cout slice 下需要切 cout 段, 这里 W slice 暂不处理)
    wb_base = ddr_planner.layer_wb_ddr(layer_idx)

    # rdma_words: 该 layer 的 rdma_data 行数 (含 bias + 可选 shortcut).
    rdma_words = None
    rdma_ddr   = ddr_planner.layer_rdma_ddr(layer_idx)
    if (core_layer_rdma_words is not None
        and core_layer_rdma_words[step.my_core][layer_idx] > 0):
        # W slice + residual: 用 per-core sliced rdma file
        rdma_words = core_layer_rdma_words[step.my_core][layer_idx]
        rdma_ddr   = ddr_planner.core_layer_rdma_ddr(step.my_core, layer_idx,
                                                      n_cores=n_cores_total or 4)
    elif chain_data is not None and layer_idx < len(chain_data):
        rdma_words = chain_data[layer_idx].get('rdma_words', None)

    cfg_dict = hw_files.cfg_to_dict(
        cfg, shift_amt=layer.sdp_shift,
        sdp_mult       =layer.sdp_mult,    sdp_zp_out =layer.sdp_zp_out,
        sdp_clip_min   =layer.sdp_clip_min, sdp_clip_max=layer.sdp_clip_max,
        sdp_round_en   =layer.sdp_round_en, sdp_relu_en =layer.sdp_relu_en,
        residual_en    =1 if layer.has_residual else 0,
        shortcut_mult  =layer.shortcut_mult,
        shortcut_shift =layer.shortcut_shift,
        rdma_words_total=rdma_words,
        case_name=f"L{layer_idx}_{layer.name}_c{step.my_core}",
        ddr_ifb_base =idma_src,
        ddr_wb_base  =wb_base,
        ddr_ofb_base =odma_dst,
        ddr_rdma_base=rdma_ddr,
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
def gen_core_per_layer_desc_lists(core_id, steps, layers, ddr_planner, n_layers,
                                    name_to_idx=None, chain_data=None,
                                    core_layer_rdma_words=None, n_cores_total=None,
                                    root_ifb_idx=None):
    """
    返回 dict: layer_idx → list of (beat0, beat1) descs (每个独立 desc list, 末尾 END).
    若该 (core, layer) 在 plan 里没分到任务, key 不在 dict 中.
    """
    out = {}
    for step in steps:
        cfg_dict, cfg = build_step_cfg_dict(step, layers, ddr_planner, n_layers,
                                              name_to_idx=name_to_idx, chain_data=chain_data,
                                              core_layer_rdma_words=core_layer_rdma_words,
                                              n_cores_total=n_cores_total,
                                              root_ifb_idx=root_ifb_idx)
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

    # name → layer index 索引 (ResNet 用 input_src/shortcut_src 找上游 layer 的 DDR 区)
    name_to_idx = {l.name: i for i, l in enumerate(layers)}

    # Root layer (i>0 但 ifb_is_root=True) 各自独占 ROOT_IFB 区, 避开上层 OFM 覆盖.
    # layer 0 (整网入口) 用 INPUT 区, 不归 ROOT_IFB.
    root_ifb_idx = {}   # layer_idx → root slot
    n_roots = 0
    for i, cd in enumerate(chain_data):
        if i > 0 and cd['ifb_is_root']:
            root_ifb_idx[i] = n_roots
            n_roots += 1

    # 后处理: residual + W slice 层, 生成 per-(core, layer) sliced rdma_data.txt.
    # core_layer_rdma_words[core_id][layer_idx] = 该核该层的 rdma word 数 (含 bias + sliced shortcut)
    # core_layer_rdma_words[c][l] = 0 表示该核该层不参与 (沿用 layer.rdma_words 默认即可)
    core_layer_rdma_words = [[0] * len(layers) for _ in range(n_cores)]
    for stage in stages:
        for a in stage.assignments:
            if a.mode != scheduler.Mode.C_W_SLICE:
                continue
            layer_idx = name_to_idx[a.layer.name]
            cd = chain_data[layer_idx]
            if not cd['has_residual']:
                continue
            # 该 layer W slice + residual: 每核切自己 W slice 的 shortcut 写独立 rdma 文件
            # 用 main path geom 算 W slice (确保 shortcut 段跟 OFM 段对齐).
            n_split = len(a.cores)
            for slice_idx, core_id in enumerate(a.cores):
                geom = hw_files.compute_w_slice_geom(
                    W_full=a.layer.w_in, K=a.layer.k, stride=a.layer.stride,
                    pad_left_full=a.layer.pad,
                    my_core=slice_idx, n_split=n_split,
                )
                w_slice_start = geom['w_out_start']
                w_slice_end   = w_slice_start + geom['my_w_out']
                fname = f'rdma_data_c{core_id}.txt'
                n_lines, _ = hw_files.write_rdma_data(
                    cd['layer_dir'], bias_arr=None,
                    shortcut_arr=cd['shortcut_arr'],
                    NUM_COUT=a.layer.c_out, HW_COL=16,
                    H_OUT=a.layer.h_out, W_OUT=a.layer.w_out,
                    out_filename=fname,
                    w_slice_start=w_slice_start, w_slice_end=w_slice_end,
                )
                core_layer_rdma_words[core_id][layer_idx] = n_lines

    # 4. 给每核生成 per-layer desc lists (host stage barrier 模式)
    print(f"\n[Step 2] Generating per-(core,layer) desc lists for {n_cores} cores...")
    # core_layer_descs[core_id][layer_idx] = n_descs (0 表示该 core 该层没活)
    core_layer_descs = [[0] * len(layers) for _ in range(n_cores)]
    for core_id in range(n_cores):
        steps = per_core_plan[core_id]
        per_layer_segs = gen_core_per_layer_desc_lists(core_id, steps, layers, ddr, len(layers),
                                                        name_to_idx=name_to_idx,
                                                        chain_data=chain_data,
                                                        core_layer_rdma_words=core_layer_rdma_words,
                                                        n_cores_total=n_cores,
                                                        root_ifb_idx=root_ifb_idx)
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
                # W slice + residual: per-core sliced rdma file (file 名 / DDR base / words)
                core_rdma_words = core_layer_rdma_words[core_id][layer_idx]
                if core_rdma_words > 0:
                    rdma_base = ddr.core_layer_rdma_ddr(core_id, layer_idx, n_cores=n_cores)
                    f.write(f"CORE_{core_id}_LAYER_{layer_idx}_RDMA_BASE = 0x{rdma_base:08x}\n")
                    f.write(f"CORE_{core_id}_LAYER_{layer_idx}_RDMA_WORDS = {core_rdma_words}\n")
        # final layer 信息 (TB 比对用)
        last_layer = layers[-1]
        f.write(f"FINAL_H_OUT = {last_layer.h_out}\n")
        f.write(f"FINAL_W_OUT = {last_layer.w_out}\n")
        f.write(f"FINAL_C_OUT = {last_layer.c_out}\n")
        # data file 路径
        for i, cd in enumerate(chain_data):
            layer = layers[i]
            f.write(f"LAYER_{i}_DIR = {cd['layer_dir']}\n")
            f.write(f"LAYER_{i}_IFB_WORDS = {cd['ifb_words']}\n")
            f.write(f"LAYER_{i}_WB_WORDS = {cd['wb_words']}\n")
            f.write(f"LAYER_{i}_OFB_WORDS = {cd['ofb_words']}\n")
            f.write(f"LAYER_{i}_RDMA_WORDS = {cd['rdma_words']}\n")  # bias + opt shortcut
            f.write(f"LAYER_{i}_PRELOAD_IFB = {1 if cd['ifb_is_root'] else 0}\n")
            # IDMA SRC:
            #   1. input_src 非空 → 上游 layer 的 OFM 区 (ResNet block 内链)
            #   2. ifb_is_root && i>0 → 独立 ROOT_IFB slot (避免被上层 OFM 覆盖)
            #   3. 其他 (layer 0 或线性 chain) → layer_input_ddr(i)
            if layer.input_src and layer.input_src in name_to_idx:
                idma_ddr = ddr.layer_output_ddr(name_to_idx[layer.input_src], len(layers))
            elif i in root_ifb_idx:
                idma_ddr = ddr.root_ifb_ddr(root_ifb_idx[i])
            else:
                idma_ddr = ddr.layer_input_ddr(i)
            f.write(f"LAYER_{i}_DDR_IFB = 0x{idma_ddr:08x}\n")
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
                        choices=['wslice1', 'simple2', 'simple3', 'wslice4', 'wslice5',
                                 'wslice_mixed', 'wslice_stride2', 'wslice_oddw',
                                 'wslice_smallw', 'wslice_k7', 'wslice_k1',
                                 'resnet_block1', 'resnet_residual_wslice', 'resnet11'],
                        default='wslice1',
                        help='wslice4/5: 4/5 层 chain | wslice_mixed: 多 K | '
                             'wslice_stride2: 含 ds 层 | wslice_oddw: W=33 奇 | '
                             'wslice_smallw: W=8 极小 | wslice_k7: K=7 大 halo | wslice_k1: K=1 无 halo')
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
    elif args.demo == 'wslice_stride2':
        # 含 stride=2 的多层 chain: layer 间 H/W 维度变化
        # L0 32×32 K=3 s=2 pad=1 → 16×16
        # L1 16×16 K=3 s=1 pad=1 → 16×16
        # L2 16×16 K=3 s=2 pad=1 → 8×8
        # L3 8×8   K=3 s=1 pad=1 → 8×8
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=32, stride=2, pad=1, sdp_shift=3),
            scheduler.Layer('L1', k=3, c_in=16, c_out=16, h_in=16, w_in=16, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L2', k=3, c_in=16, c_out=16, h_in=16, w_in=16, stride=2, pad=1, sdp_shift=3),
            scheduler.Layer('L3', k=3, c_in=16, c_out=16, h_in=8,  w_in=8,  stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'wslice_oddw':
        # W 不被 N 整除: W=33, 33/2=16余1, 33/4=8余1
        # 用 K=3 pad=1 让维度保持 (奇数 H 也 OK).
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=33, w_in=33, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L1', k=3, c_in=16, c_out=16, h_in=33, w_in=33, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L2', k=3, c_in=16, c_out=16, h_in=33, w_in=33, stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'wslice_smallw':
        # 极小 W: W=8 (N=4 时每核 W_OUT=2, 极端边界).
        layers = [
            scheduler.Layer('L0', k=3, c_in=16, c_out=16, h_in=32, w_in=8, stride=1, pad=1, sdp_shift=2),
            scheduler.Layer('L1', k=3, c_in=16, c_out=16, h_in=32, w_in=8, stride=1, pad=1, sdp_shift=2),
        ]
    elif args.demo == 'wslice_k7':
        # 大 K: K=7 N=2 → halo=3, 验证 大 halo 几何 + 不对称 pad
        layers = [
            scheduler.Layer('L0', k=7, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=3, sdp_shift=4),
            scheduler.Layer('L1', k=7, c_in=16, c_out=16, h_in=32, w_in=32, stride=1, pad=3, sdp_shift=4),
        ]
    elif args.demo == 'wslice_k1':
        # K=1 W slice (无 halo, 直接切): 强制 prefer_w_slice 跑
        # 注 scheduler 默认对 K=1 c=16 cycles 太小用 mode A; 这里用 c=64 让 cycles 大
        layers = [
            scheduler.Layer('L0', k=1, c_in=64, c_out=64, h_in=16, w_in=16, stride=1, pad=0, sdp_shift=4),
            scheduler.Layer('L1', k=1, c_in=64, c_out=64, h_in=16, w_in=16, stride=1, pad=0, sdp_shift=4),
        ]
    elif args.demo == 'resnet_residual_wslice':
        # Residual layer 自己 W slice: 用 K=3 (cycles 大) 让 scheduler 选 W slice.
        # L0 → L1 → L2 (+ residual from L0). 全部 K=3, 全部 32×32×16.
        # L2 是 K=3 + residual, scheduler 会切 N 核 — 测试 RDMA W slice.
        L0 = scheduler.Layer('B1.C1', k=3, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=1, sdp_shift=2)
        L1 = scheduler.Layer('B1.C2', k=3, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=1, sdp_shift=2, input_src='B1.C1')
        L2 = scheduler.Layer('B2.add', k=3, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=1, sdp_shift=2,
                             input_src='B1.C2',
                             shortcut_src='B1.C1',
                             has_residual=True,
                             shortcut_mult=1, shortcut_shift=0)
        layers = [L0, L1, L2]
    elif args.demo == 'resnet_block1':
        # 小 ResNet block: 3 conv 层 + residual. 32×32 入口 (避开 ResNet Patch 的 8M cycle).
        #   L0 (B1.C1, stride=1):    32×32×16 → 32×32×16
        #   L1 (B1.C2):              32×32×16 → 32×32×16
        #   L2 (B2.ds, K=1, +residual from L0):  32×32×16 → 32×32×16
        # 注: 不做下采样 (ds_stride=1) 让 chain 维度一致 + L2 的 shortcut 来源 = L0 input src 改成 'L0',
        # 用 L0 作 block 入口 (L1 在 main path), 跟 ResNet 风格 B2.ds shortcut = B1.C2 不同, 但够测 residual 数据流.
        # 真正 ResNet 跟 Patch 用 resnet11 demo, 这里只测 residual + 多层 chain 兼容.
        L0 = scheduler.Layer('B1.C1', k=3, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=1, sdp_shift=2, input_src='')
        L1 = scheduler.Layer('B1.C2', k=3, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=1, sdp_shift=2, input_src='B1.C1')
        L2 = scheduler.Layer('B2.ds', k=1, c_in=16, c_out=16, h_in=32, w_in=32,
                             stride=1, pad=0, sdp_shift=2,
                             input_src='B1.C2',          # main path: 接 C2 (跟真 ResNet 不同, 真 ResNet 接 block 入口)
                             shortcut_src='B1.C1',       # residual path: 用 C1 的 OFM
                             has_residual=True,
                             shortcut_mult=1, shortcut_shift=0)
        layers = [L0, L1, L2]
    elif args.demo == 'resnet11':
        from run_regression import CASES
        layers = scheduler.chain_to_layers(CASES[:11])

    out_root = os.path.join(_THIS_DIR, '..', 'sim', 'tb_multicore', 'cases', args.case_name)
    if os.path.exists(out_root):
        shutil.rmtree(out_root)
    os.makedirs(out_root, exist_ok=True)

    chain_data, per_core_plan = run_multicore_chain(layers, args.n_cores, out_root)

    print(f"\n=== Done. Output: {out_root} ===")
