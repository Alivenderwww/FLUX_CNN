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
import dataclasses
import random

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.dirname(_THIS_DIR))    # 项目根, 取 params.py

import gen_isa_test
import hw_files
import scheduler
import mesh_cmd
from params import core_ifb_axi_base, IFB_DEPTH, OFB_DEPTH


# ===========================================================================
# Phase 7 SMC + NUMA: 全局地址 layout
#   addr[25:24] = mem_id 路由 (跟 axi_crossbar_4to4_sim / SmartConnect IP 配置一致)
#   每 mem 16 MB, 4 mem 总 64 MB. 每 mem 内分区:
# ===========================================================================
SMC_MEM_STRIDE              = 0x0100_0000   # 16 MB / mem (= 1 << 24)
SMC_IFM_OFM_BASE            = 0x0000_0000   # 8 MB IFM/OFM (16 layer × 512 KB / layer)
SMC_LAYER_DATA_OFFSET       = 0x0008_0000   # 512 KB / (layer, mem)
SMC_WB_BASE                 = 0x0080_0000   # 1 MB WB (16 layer × 64 KB; 全核共享, broadcast 4 mem)
SMC_LAYER_WB_OFFSET         = 0x0001_0000   # 64 KB / (layer, mem)
SMC_RDMA_BASE               = 0x0090_0000   # 1 MB RDMA per-core
SMC_LAYER_RDMA_OFFSET       = 0x0001_0000
SMC_DESC_BASE               = 0x00A0_0000   # 1 MB desc list per-(core, layer)
SMC_LAYER_DESC_OFFSET       = 0x0001_0000   # 64 KB / (layer)
SMC_IDMA_CMD_BASE           = 0x00B0_0000   # 1 MB IDMA SG cmd list per-(core, layer)
SMC_LAYER_IDMA_CMD_OFFSET   = 0x0001_0000
SMC_ODMA_CMD_BASE           = 0x00C0_0000   # 1 MB ODMA SG cmd list per-(core, layer)
SMC_LAYER_ODMA_CMD_OFFSET   = 0x0001_0000
SMC_FINAL_OFM_BASE          = 0x00E0_0000   # 1 MB 最后一层 OFM (放 mem[0]) — 整图最终输出
SMC_INPUT_BASE              = 0x00D0_0000   # 1 MB layer 0 IFB (4 mem 各装本核 W 段)
SMC_LAYER_INPUT_OFFSET      = 0x0008_0000   # 512 KB / root layer slot (layer 0 占 0+, root i>0 占 (slot+1)*offset)
                                            # 必须 ≥ max layer 0 IFB seg 容量 (Patch s2d 240×33×4×16 ≈ 496 KB)


def smc_global_addr(mem_id: int, mem_offset: int) -> int:
    """全局地址 = (mem_id << 24) | mem_offset"""
    return (mem_id * SMC_MEM_STRIDE) + mem_offset


def smc_layer_data_addr(mem_id: int, layer_idx: int) -> int:
    """layer i 的 IFM/OFM 段在 mem[mem_id] 内紧凑 layout 起点 (全局地址)"""
    return smc_global_addr(mem_id, SMC_IFM_OFM_BASE + layer_idx * SMC_LAYER_DATA_OFFSET)


def smc_layer_wb_addr(mem_id: int, layer_idx: int) -> int:
    return smc_global_addr(mem_id, SMC_WB_BASE + layer_idx * SMC_LAYER_WB_OFFSET)


def smc_layer_rdma_addr(core_id: int, layer_idx: int) -> int:
    """RDMA per-core: ConvCore[c] 用 mem[c] 的 RDMA 区"""
    return smc_global_addr(core_id, SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET)


def smc_core_layer_desc_addr(core_id: int, layer_idx: int) -> int:
    return smc_global_addr(core_id, SMC_DESC_BASE + layer_idx * SMC_LAYER_DESC_OFFSET)


def smc_core_layer_idma_cmd_addr(core_id: int, layer_idx: int) -> int:
    return smc_global_addr(core_id, SMC_IDMA_CMD_BASE + layer_idx * SMC_LAYER_IDMA_CMD_OFFSET)


def smc_core_layer_odma_cmd_addr(core_id: int, layer_idx: int) -> int:
    return smc_global_addr(core_id, SMC_ODMA_CMD_BASE + layer_idx * SMC_LAYER_ODMA_CMD_OFFSET)


def smc_input_addr(mem_id: int) -> int:
    """layer 0 IFB 的 mem 内 base (本核 W 段)"""
    return smc_global_addr(mem_id, SMC_INPUT_BASE)


def smc_final_ofm_addr(mem_id: int) -> int:
    return smc_global_addr(mem_id, SMC_FINAL_OFM_BASE)


def cd_get(chain_data, idx, key, default):
    """取 chain_data[idx][key] 默认值 fallback (chain_data 元素是 dict)."""
    if 0 <= idx < len(chain_data):
        return chain_data[idx].get(key, default)
    return default


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
        # SMC + NUMA 架构: 4 mem 通过 axi_smc_4to4 IP 路由 (mem_id 由 addr[25:24] 解码).
        # 每 mem 16 MB, 全图 IFM/OFM/WB/desc 等按地址分区散布到 4 mem 内.
        # 大分区 (axi_slave_mem DEPTH = 32M words × 16 byte/word = 512 MB byte 容量)
        # ResNet Patch 输入 960×540×4 ≈ 8.3 MB → INPUT region 必须 ≥ 16 MB.
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
            # chain_data single-core 整图跑用大 SRAM (绕过 W slice cfg 容量限制)
            #   ResNet11 layer 0 W=135 cs=4 row_words=540, IFB 1024 不够装 K+1=2 strip
            #   single-core 仅生成 golden expected_ofm 不上硬件, 用 8192/2048 没影响
            ifb_sram_words_override=8192,
            ofb_sram_words_override=2048,
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
                         root_ifb_idx=None, smc=False,
                         smc_idma_cmd=None, smc_odma_cmd=None):
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

        # Round I: K=1 stride>1 (ds layer) 时 IDMA 只拉 stride 行, mac_array 当 H stride=1 跑.
        #   (W 维 mac_array 仍按原 stride 跳, 因为 axi_dm 不支持 W 维 strided burst.)
        #   暂限 cin_slices==1: cin_slices>1 触发 OFM mismatch (待 debug)
        if k_e == 1 and stride_e > 1 and cin_slices == 1:
            h_in_idma = (h_in_e + stride_e - 1) // stride_e   # 跟 SMC SG cmd 生成同步
            cfg['_STRIDE_H']  = 1
            cfg['_H_IN_IDMA'] = h_in_idma
            # ifb ring 大小不变 (按原 stride 算的 strip 已 K+1 行足够)
    elif step.mode == 'C_cout_slice':
        # cout 切: 每核读全图 IFM, 写自己 cout 段紧凑 layout. ifb/ofb_w_start=0
        # (W 维不切). cfg['cout_slices'] 是本核段的 cs 数 (整图 cs 的一段).
        slice_idx = step.cores_all.index(step.my_core)
        cfg = hw_files.derive_cout_slice_cfg(
            H_IN=h_in_e, W_IN=w_in_e, K=k_e,
            NUM_CIN=c_in_e, NUM_COUT_full=layer.c_out, stride=stride_e,
            pad_top=pad_e, pad_left=pad_e,
            my_core=slice_idx, n_split=n_split,
            TILE_W=32, streaming=True,
        )
        ifb_w_start = 0
        ofb_w_start = 0
        # 本核实际负责的 cout_slices (整图 cs 的一段, 用于 odma cmd list 生成)
        cout_slices = cfg['cout_slices']
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
    if smc:
        # SMC 模式: idma_src 不再单一 base, 由 SG cmd list 内每条 cmd 携带各自全局地址.
        # 这里给 idma_src=0 (ConvCore IDMA SG dispatcher 不读 IDMA_SRC_BASE 寄存器).
        idma_src = 0
    elif step.input_from == 'push':
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

    if smc:
        # SMC 模式: odma_dst 由 SG cmd list 携带, ConvCore ODMA SG dispatcher 不读
        # ODMA_DST_BASE 寄存器
        odma_dst = 0
    elif step.output_to == 'push':
        odma_dst = core_ifb_axi_base(step.push_to_core)
    else:
        odma_dst = ddr_planner.layer_output_ddr(layer_idx, n_layers) + ofb_w_start * cout_slices * 16

    # WB 全核共享 (W slice 下 weight 完全相同; cout slice 下需要切 cout 段, 这里 W slice 暂不处理)
    if smc:
        # SMC 模式: WB 在每 mem 内紧凑 layout, ConvCore[c] 拉自己核 mem 的 WB
        # (broadcast preload 4 mem 都装 WB 数据, 任一 mem 都能服务)
        wb_base = smc_layer_wb_addr(step.my_core, layer_idx)
    else:
        wb_base = ddr_planner.layer_wb_ddr(layer_idx)

    # rdma_words: 该 layer 的 rdma_data 行数 (含 bias + 可选 shortcut).
    rdma_words = None
    if smc:
        rdma_ddr = smc_layer_rdma_addr(step.my_core, layer_idx)
    else:
        rdma_ddr = ddr_planner.layer_rdma_ddr(layer_idx)
    if (core_layer_rdma_words is not None
        and core_layer_rdma_words[step.my_core][layer_idx] > 0):
        # W slice + residual: 用 per-core sliced rdma file
        rdma_words = core_layer_rdma_words[step.my_core][layer_idx]
        if not smc:
            rdma_ddr = ddr_planner.core_layer_rdma_ddr(step.my_core, layer_idx,
                                                        n_cores=n_cores_total or 4)
    elif chain_data is not None and layer_idx < len(chain_data):
        rdma_words = chain_data[layer_idx].get('rdma_words', None)

    # SMC 模式: 注入 SG cmd list ptr/count/cmds_per_row (cfg_to_dict 自动写入 cfg_dict
    # 让 build_layer_desc_segment 生成 CFG_WRITE descriptor)
    sg_kwargs = {}
    if smc and smc_idma_cmd is not None:
        sg_kwargs['idma_cmd_list_ptr'] = smc_idma_cmd['ptr']
        sg_kwargs['idma_cmd_count']    = smc_idma_cmd['count']
        sg_kwargs['idma_cmds_per_row'] = smc_idma_cmd['cmds_per_row']
    if smc and smc_odma_cmd is not None:
        sg_kwargs['odma_cmd_list_ptr'] = smc_odma_cmd['ptr']
        sg_kwargs['odma_cmd_count']    = smc_odma_cmd['count']
        sg_kwargs['odma_cmds_per_row'] = smc_odma_cmd['cmds_per_row']

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
        **sg_kwargs,
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
                                    root_ifb_idx=None, smc=False,
                                    smc_per_layer_cmds=None):
    """
    返回 dict: layer_idx → list of (beat0, beat1) descs (每个独立 desc list, 末尾 END).
    若该 (core, layer) 在 plan 里没分到任务, key 不在 dict 中.
    """
    out = {}
    for step in steps:
        # SMC 模式: 拿出预先算好的 IDMA + ODMA SG cmd list 信息 (per-layer ptr/count/cmds_per_row)
        smc_idma_cmd = None
        smc_odma_cmd = None
        if smc and smc_per_layer_cmds is not None:
            smc_idma_cmd = smc_per_layer_cmds.get(('idma', core_id, step.layer_idx))
            smc_odma_cmd = smc_per_layer_cmds.get(('odma', core_id, step.layer_idx))
        cfg_dict, cfg = build_step_cfg_dict(step, layers, ddr_planner, n_layers,
                                              name_to_idx=name_to_idx, chain_data=chain_data,
                                              core_layer_rdma_words=core_layer_rdma_words,
                                              n_cores_total=n_cores_total,
                                              root_ifb_idx=root_ifb_idx, smc=smc,
                                              smc_idma_cmd=smc_idma_cmd, smc_odma_cmd=smc_odma_cmd)
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
def run_multicore_chain(layers, n_cores, out_dir, smc=False):
    """生成多核多层 chain 的 case 目录 + desc list (SMC + NUMA 主线)."""

    # 1. Schedule
    print(f"\n=== Multicore chain: {len(layers)} layers, N_cores={n_cores}"
          f"{', SMC' if smc else ''} ===")
    # SMC 模式强制全 W slice (driver layout 假设每层都按整图 W 4 等分散布到 4 mem)
    force_multicore = True if smc else False
    stages = scheduler.schedule(layers, n_cores, force_multicore=force_multicore)
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

    # 3.5. SMC 模式: 生成 IDMA + ODMA SG cmd list per-(core, layer) (在 desc list 之前,
    #      desc list 内的 CFG_WRITE 需要嵌入 SG ptr/count/cmds_per_row).
    smc_per_layer_cmds = {}    # ('idma'|'odma', core_id, layer_idx) → {'ptr', 'count', 'cmds_per_row'}
    if smc:
        print(f"\n[SMC] Generating per-(core, layer) IDMA + ODMA SG cmd lists...")
        for core_id in range(n_cores):
            core_dir = os.path.join(out_dir, f'core{core_id}')
            os.makedirs(core_dir, exist_ok=True)
            for step in per_core_plan[core_id]:
                layer = step.layer
                layer_idx = step.layer_idx
                h_in_e, w_in_e, k_e, c_in_e, stride_e = layer.s2d_eff()
                pad_e = 0 if layer.force_s2d else layer.pad
                cin_slices  = (c_in_e + 16 - 1) // 16
                cout_slices = (layer.c_out + 16 - 1) // 16
                n_split = len(step.cores_all)

                if n_split == 1:
                    # mode A 单核 layer (FC W=1 等): 该核拉整图 IFM, 写整图 OFM, 全部
                    # 在该核自己 mem 内紧凑存储. SMC layout 兼容: 上层 mode A OFM 在 mem[c]
                    # 整图区, 下层如果是 W slice 各核拉自己 W 段从该 mem; 如果下层也 mode A
                    # 单核拉整图 OK.
                    # 简化: 我们用 1 个 "段" 装整图, 所有 4 个 mem base 都指向 mem[my_core] (即"段"
                    # 集中放在一个 mem), seg_w_starts_in=[0,...], seg_widths_in=[w_full,0,...] 不行,
                    # 改成单段表示整图.
                    w_full_eff = w_in_e
                    h_out_eff_a = (h_in_e + 2 * pad_e - k_e) // stride_e + 1
                    w_out_eff_a = (w_in_e + 2 * pad_e - k_e) // stride_e + 1

                    # mode A IDMA: 一行一条 cmd, 整图 W
                    idma_cmds = []
                    if layer_idx == 0:
                        ifb_mem_base = smc_global_addr(0, SMC_INPUT_BASE)
                    elif chain_data[layer_idx]['ifb_is_root']:
                        # root layer i>0 用独立 slot (避免覆盖 layer 0 IFB), 64 KB / root
                        root_slot = root_ifb_idx[layer_idx]
                        ifb_mem_base = smc_global_addr(0, SMC_INPUT_BASE + (root_slot + 1) * SMC_LAYER_INPUT_OFFSET)
                    else:
                        # 上层 OFM. 上层若是 mode A 在 mem[prev_my_core], 若是 W slice 散 4 mem.
                        # mode A 单核拉时, 我们假设上层是 mode A 在 mem[step.my_core] 整图存储.
                        # 若上层是 W slice 跨 mem, mode A 单核暂不支持 (复杂 stitch).
                        prev_step = None
                        for s in per_core_plan[step.my_core]:
                            if s.layer_idx == layer_idx - 1:
                                prev_step = s
                                break
                        if prev_step is None or len(prev_step.cores_all) != 1:
                            raise NotImplementedError(
                                f"SMC mode A layer {layer.name}: prev layer {layer_idx-1} 必须也是 mode A "
                                f"在同一 core, 当前 W slice → mode A stitch 暂不支持"
                            )
                        ifb_mem_base = smc_layer_data_addr(prev_step.my_core, layer_idx - 1)
                    sram_offset_a = 0
                    row_words_a = w_full_eff * cin_slices
                    for r in range(h_in_e):
                        idma_cmds.append(mesh_cmd.SgCmd(
                            src_addr    = ifb_mem_base + r * row_words_a * 16,
                            btt         = row_words_a * 16,
                            sram_offset = sram_offset_a,
                            last_cmd    = (r == h_in_e - 1),
                            name        = f"r{r}_modeA",
                        ))
                        sram_offset_a += row_words_a
                    idma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_idma_sg.hex')
                    mesh_cmd.write_sg_cmd_list(idma_cmd_path, idma_cmds, header_lines=[
                        f"core={core_id} layer={layer.name} (idx={layer_idx}) MODE_A_SINGLE",
                        f"H_in={h_in_e} W_in={w_in_e} cin_slices={cin_slices}",
                        f"ifb_mem_base=0x{ifb_mem_base:08x}",
                    ])
                    smc_per_layer_cmds[('idma', core_id, layer_idx)] = {
                        'ptr': smc_core_layer_idma_cmd_addr(core_id, layer_idx),
                        'count': len(idma_cmds), 'cmds_per_row': 1,
                        'file': os.path.relpath(idma_cmd_path, out_dir),
                    }

                    # mode A ODMA: 一行一条 cmd, 整图 W
                    odma_cmds = []
                    is_last_layer = (layer_idx == len(layers) - 1)
                    if is_last_layer:
                        ofm_mem_base = smc_global_addr(step.my_core, SMC_FINAL_OFM_BASE)
                    else:
                        ofm_mem_base = smc_layer_data_addr(step.my_core, layer_idx)
                    row_words_o = w_out_eff_a * cout_slices
                    for r in range(h_out_eff_a):
                        odma_cmds.append(mesh_cmd.OdmaSgCmd(
                            dst_addr    = ofm_mem_base + r * row_words_o * 16,
                            btt         = row_words_o * 16,
                            ofb_w_start = 0,
                            last_cmd    = (r == h_out_eff_a - 1),
                            name        = f"r{r}_modeA",
                        ))
                    odma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_odma_sg.hex')
                    mesh_cmd.write_odma_sg_cmd_list(odma_cmd_path, odma_cmds, header_lines=[
                        f"core={core_id} layer={layer.name} (idx={layer_idx}) MODE_A_SINGLE",
                        f"H_out={h_out_eff_a} W_out={w_out_eff_a} cout_slices={cout_slices}",
                        f"ofm_mem_base=0x{ofm_mem_base:08x}",
                        f"is_last_layer={is_last_layer}",
                    ])
                    smc_per_layer_cmds[('odma', core_id, layer_idx)] = {
                        'ptr': smc_core_layer_odma_cmd_addr(core_id, layer_idx),
                        'count': len(odma_cmds), 'cmds_per_row': 1,
                        'file': os.path.relpath(odma_cmd_path, out_dir),
                    }
                    continue   # mode A 处理完, 跳过 W slice 分支

                if n_split != n_cores:
                    raise NotImplementedError(
                        f"SMC requires every layer to use all {n_cores} cores in W slice (or 1 in mode A); "
                        f"got n_split={n_split} for layer {layer.name}"
                    )
                slice_idx = step.cores_all.index(step.my_core)

                if step.mode == 'C_cout_slice':
                    # ============================================================
                    # COUT slice (D 路径退化版): 每核拉相同整图 IFM, 写自己 cout 段
                    # 紧凑 layout 到自己 mem. 适用 H×W 小 + cout 大 (e.g., L10 FC).
                    # 要求: 上层 IFM 集中存放 (layer 是 ifb_is_root 或 layer_idx=0,
                    # 或上层是 mode A). 不支持上层 W slice → cout slice (复杂 stitch).
                    # ============================================================
                    h_out_eff_c = (h_in_e + 2 * pad_e - k_e) // stride_e + 1
                    w_out_eff_c = (w_in_e + 2 * pad_e - k_e) // stride_e + 1

                    # IFM 源 mem: 必须是集中存放
                    if layer_idx == 0:
                        ifb_mem_base = smc_global_addr(0, SMC_INPUT_BASE)
                    elif chain_data[layer_idx]['ifb_is_root']:
                        root_slot = root_ifb_idx[layer_idx]
                        ifb_mem_base = smc_global_addr(0, SMC_INPUT_BASE + (root_slot + 1) * SMC_LAYER_INPUT_OFFSET)
                    else:
                        # 上层是 mode A 在 mem[prev_my_core] 整图. 否则不支持.
                        prev_mode_a_core = None
                        for s in per_core_plan[0]:   # 任一核扫: mode A layer 在所有核 plan 里只 1 个 step
                            if s.layer_idx == layer_idx - 1 and len(s.cores_all) == 1:
                                prev_mode_a_core = s.my_core
                                break
                        if prev_mode_a_core is None:
                            raise NotImplementedError(
                                f"SMC cout slice layer {layer.name}: prev layer {layer_idx-1} 必须 mode A "
                                f"集中存放或 layer 是 root, 当前 W slice → cout slice 不支持"
                            )
                        ifb_mem_base = smc_layer_data_addr(prev_mode_a_core, layer_idx - 1)

                    # 本核 cout 段
                    seg_cout_starts, seg_widths_cout, seg_cs_per_core = \
                        mesh_cmd.compute_cout_segments(layer.c_out, n_split)
                    my_cs       = seg_cs_per_core[slice_idx]
                    my_cout_lo  = seg_cout_starts[slice_idx]
                    my_cout_hi  = my_cout_lo + seg_widths_cout[slice_idx]

                    # IDMA: 整图 IFM, 4 核 cmd 相同 (都从 ifb_mem_base 拉)
                    idma_cmds = mesh_cmd.gen_idma_sg_cmd_list_cout_slice(
                        h_in=h_in_e, w_in=w_in_e, cin_slices=cin_slices,
                        ifb_mem_base=ifb_mem_base,
                    )
                    idma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_idma_sg.hex')
                    mesh_cmd.write_sg_cmd_list(idma_cmd_path, idma_cmds, header_lines=[
                        f"core={core_id} layer={layer.name} (idx={layer_idx}) COUT_SLICE",
                        f"H_in={h_in_e} W_in={w_in_e} cin_slices={cin_slices}",
                        f"ifb_mem_base=0x{ifb_mem_base:08x}",
                        f"my_cout=[{my_cout_lo}:{my_cout_hi}) my_cs={my_cs}",
                    ])
                    smc_per_layer_cmds[('idma', core_id, layer_idx)] = {
                        'ptr': smc_core_layer_idma_cmd_addr(core_id, layer_idx),
                        'count': len(idma_cmds), 'cmds_per_row': 1,
                        'file': os.path.relpath(idma_cmd_path, out_dir),
                    }

                    # ODMA: 本核 cout 段紧凑写自己 mem
                    is_last_layer = (layer_idx == len(layers) - 1)
                    if is_last_layer:
                        ofm_mem_base = smc_global_addr(step.my_core, SMC_FINAL_OFM_BASE)
                    else:
                        ofm_mem_base = smc_layer_data_addr(step.my_core, layer_idx)
                    odma_cmds = mesh_cmd.gen_odma_sg_cmd_list_cout_slice(
                        h_out=h_out_eff_c, w_out=w_out_eff_c,
                        cout_slices_per_core=my_cs,
                        ofm_mem_base=ofm_mem_base,
                    )
                    odma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_odma_sg.hex')
                    mesh_cmd.write_odma_sg_cmd_list(odma_cmd_path, odma_cmds, header_lines=[
                        f"core={core_id} layer={layer.name} (idx={layer_idx}) COUT_SLICE",
                        f"H_out={h_out_eff_c} W_out={w_out_eff_c} my_cs={my_cs}",
                        f"ofm_mem_base=0x{ofm_mem_base:08x}",
                        f"my_cout=[{my_cout_lo}:{my_cout_hi}) is_last_layer={is_last_layer}",
                    ])
                    smc_per_layer_cmds[('odma', core_id, layer_idx)] = {
                        'ptr': smc_core_layer_odma_cmd_addr(core_id, layer_idx),
                        'count': len(odma_cmds), 'cmds_per_row': 1,
                        'file': os.path.relpath(odma_cmd_path, out_dir),
                    }
                    continue   # cout slice 处理完, 跳过 W slice 分支

                # 本核 W slice geom
                geom = hw_files.compute_w_slice_geom(
                    W_full=w_in_e, K=k_e, stride=stride_e,
                    pad_left_full=pad_e,
                    my_core=slice_idx, n_split=n_split,
                )
                w_in_lo = geom['w_in_start']
                w_in_hi = w_in_lo + geom['sub_W'] - 1
                w_out_lo = geom['w_out_start']
                w_out_hi = w_out_lo + geom['my_w_out'] - 1
                w_out_full = geom['w_out_full']

                # 算 cfg_ifb_ring_words 给 SG cmd sram_offset wrap 用
                _w_slice_cfg = hw_files.derive_w_slice_cfg(
                    H_IN=h_in_e, W_full=w_in_e, K=k_e,
                    NUM_CIN=c_in_e, NUM_COUT=layer.c_out, stride=stride_e,
                    pad_top=pad_e, pad_left_full=pad_e,
                    my_core=slice_idx, n_split=n_split,
                    TILE_W=32, streaming=True,
                )
                _ifb_ring_words_eff = _w_slice_cfg.get('ifb_ring_words', 0)

                # IFM 段散布 (整图 W 4 等分): seg_widths_in 跟 compute_w_slice_geom 对应
                seg_w_starts_in, seg_widths_in = mesh_cmd.compute_smc_w_segments(w_in_e, n_cores)
                # IFM 段在 mem[i] 内 byte base
                #   layer 0 IFB: SMC_INPUT_BASE
                #   layer >0   : 上层 OFM = 本层 IFM = SMC_LAYER_DATA_OFFSET * (layer_idx-1) 起?
                #                我们让 layer i 的 IFM 段 = layer i-1 的 OFM 段, layer i 的 OFM 段
                #                写到 mem 内 SMC_LAYER_DATA_OFFSET * layer_idx
                #                所以 layer i 的 IFM 段 base = layer_idx-1 的 OFM 段
                #   layer 0 是 root: 用 SMC_INPUT_BASE
                if layer_idx == 0:
                    seg_mem_bases_in = [smc_global_addr(i, SMC_INPUT_BASE) for i in range(n_cores)]
                elif chain_data[layer_idx]['ifb_is_root']:
                    root_slot = root_ifb_idx[layer_idx]
                    seg_mem_bases_in = [smc_global_addr(i, SMC_INPUT_BASE + (root_slot + 1) * SMC_LAYER_INPUT_OFFSET)
                                        for i in range(n_cores)]
                elif layer.input_src and layer.input_src in name_to_idx:
                    # ResNet 风格: layer 间非 linear chain, input 来自 input_src 指定 layer
                    src_idx = name_to_idx[layer.input_src]
                    seg_mem_bases_in = [smc_layer_data_addr(i, src_idx) for i in range(n_cores)]
                else:
                    seg_mem_bases_in = [smc_layer_data_addr(i, layer_idx - 1) for i in range(n_cores)]

                # Round I: K=1 stride>1 (ds layer) 时只拉 stride 行 IFM (W stride 仍 mac_array 跳).
                #   IDMA cmd 数 H 维减半, 数据量减半. cfg STRIDE_H=1 让 line_buffer 当 H stride=1 跑.
                #   暂限 cin_slices==1: cin_slices>1 (e.g. L9 cin=32) 触发 OFM mismatch (待 debug)
                ds_h_stride_compress = (k_e == 1 and stride_e > 1 and cin_slices == 1)
                if ds_h_stride_compress:
                    h_in_idma   = (h_in_e + stride_e - 1) // stride_e   # 实际拉的行数 (= H_out)
                    h_compress  = stride_e
                else:
                    h_in_idma   = h_in_e
                    h_compress  = 1

                idma_cmds = mesh_cmd.gen_idma_sg_cmd_list_w_slice(
                    target_core_id=core_id, h_in=h_in_idma, cin_slices=cin_slices,
                    w_in_lo=w_in_lo, w_in_hi=w_in_hi,
                    seg_w_starts=seg_w_starts_in,
                    seg_mem_bases=seg_mem_bases_in,
                    seg_widths=seg_widths_in,
                    ifb_ring_words=_ifb_ring_words_eff,
                    h_compress_stride=h_compress,
                )
                idma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_idma_sg.hex')
                idma_hdr = [
                    f"core={core_id} layer={layer.name} (idx={layer_idx})",
                    f"H_in={h_in_e} W_in={w_in_e} cin_slices={cin_slices}",
                    f"my W slice: w_in[{w_in_lo}:{w_in_hi+1}] sub_W={geom['sub_W']}",
                    f"seg_widths_in={seg_widths_in} seg_w_starts_in={seg_w_starts_in}",
                    f"H stride compress: h_in_idma={h_in_idma} h_compress={h_compress}",
                ]
                mesh_cmd.write_sg_cmd_list(idma_cmd_path, idma_cmds, header_lines=idma_hdr)
                # cmds_per_row = idma_cmds 总数 / h_in_idma
                idma_cmds_per_row = len(idma_cmds) // h_in_idma
                smc_per_layer_cmds[('idma', core_id, layer_idx)] = {
                    'ptr':           smc_core_layer_idma_cmd_addr(core_id, layer_idx),
                    'count':         len(idma_cmds),
                    'cmds_per_row':  idma_cmds_per_row,
                    'file':          os.path.relpath(idma_cmd_path, out_dir),
                }

                # OFM 段散布: layer i OFM 段 = layer i+1 IFM 段
                #   每核 OFM 段 = compute_w_slice_geom 返回的 (w_out_start, my_w_out)
                #   段 mem base = layer_idx 的 IFM/OFM region (mem 内 LAYER_DATA_OFFSET*layer_idx)
                seg_w_starts_out, seg_widths_out = mesh_cmd.compute_smc_w_segments(w_out_full, n_cores)
                is_last_layer = (layer_idx == len(layers) - 1)
                if is_last_layer:
                    seg_mem_bases_out = [smc_global_addr(i, SMC_FINAL_OFM_BASE) for i in range(n_cores)]
                else:
                    seg_mem_bases_out = [smc_layer_data_addr(i, layer_idx) for i in range(n_cores)]
                h_out_eff = (h_in_e + 2 * pad_e - k_e) // stride_e + 1
                odma_cmds = mesh_cmd.gen_odma_sg_cmd_list_w_slice(
                    h_out=h_out_eff,
                    cout_slices=cout_slices,
                    w_out_lo=w_out_lo, w_out_hi=w_out_hi,
                    seg_w_starts=seg_w_starts_out,
                    seg_mem_bases=seg_mem_bases_out,
                    seg_widths=seg_widths_out,
                    ofb_w_offset_in_core=0,
                )
                odma_cmd_path = os.path.join(core_dir, f'layer{layer_idx:02d}_odma_sg.hex')
                odma_hdr = [
                    f"core={core_id} layer={layer.name} (idx={layer_idx})",
                    f"H_out={h_out_eff} W_out_full={w_out_full} cout_slices={cout_slices}",
                    f"my OFM W [{w_out_lo}:{w_out_hi+1}] my_w_out={geom['my_w_out']}",
                    f"seg_widths_out={seg_widths_out} seg_w_starts_out={seg_w_starts_out}",
                    f"is_last_layer={is_last_layer}",
                ]
                mesh_cmd.write_odma_sg_cmd_list(odma_cmd_path, odma_cmds, header_lines=odma_hdr)
                odma_cmds_per_row = max(1, len(odma_cmds) // max(1, h_out_eff))
                smc_per_layer_cmds[('odma', core_id, layer_idx)] = {
                    'ptr':           smc_core_layer_odma_cmd_addr(core_id, layer_idx),
                    'count':         len(odma_cmds),
                    'cmds_per_row':  odma_cmds_per_row,
                    'file':          os.path.relpath(odma_cmd_path, out_dir),
                }
        # 打印 SG cmd 摘要
        for core_id in range(n_cores):
            for layer_idx in range(len(layers)):
                ic = smc_per_layer_cmds.get(('idma', core_id, layer_idx))
                oc = smc_per_layer_cmds.get(('odma', core_id, layer_idx))
                if ic is None: continue
                print(f"  Core {core_id} Layer {layer_idx}: "
                      f"IDMA cmds={ic['count']} (per_row={ic['cmds_per_row']}), "
                      f"ODMA cmds={oc['count']} (per_row={oc['cmds_per_row']})")

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
                                                        root_ifb_idx=root_ifb_idx,
                                                        smc=smc,
                                                        smc_per_layer_cmds=smc_per_layer_cmds)
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
                if smc:
                    base = smc_core_layer_desc_addr(core_id, layer_idx)
                else:
                    base = ddr.core_layer_desc_ddr(core_id, layer_idx)
                    base = ddr.apply_slot(base, core_id)   # 4-DDR mode: desc 也在 core 自己 slot
                f.write(f"CORE_{core_id}_LAYER_{layer_idx}_DESC_BASE = 0x{base:08x}\n")
                f.write(f"CORE_{core_id}_LAYER_{layer_idx}_DESC_COUNT = {n_descs}\n")
                # W slice + residual: per-core sliced rdma file (file 名 / DDR base / words)
                core_rdma_words = core_layer_rdma_words[core_id][layer_idx]
                if core_rdma_words > 0:
                    if smc:
                        rdma_base = smc_layer_rdma_addr(core_id, layer_idx)
                    else:
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
            # 每层维度 (TB 逐层 OFM 验证用)
            f.write(f"LAYER_{i}_H_OUT = {layer.h_out}\n")
            f.write(f"LAYER_{i}_W_OUT = {layer.w_out}\n")
            f.write(f"LAYER_{i}_C_OUT = {layer.c_out}\n")
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
            ofb_ddr = ddr.layer_output_ddr(i, len(layers))
            f.write(f"LAYER_{i}_DDR_IFB = 0x{idma_ddr:08x}\n")
            f.write(f"LAYER_{i}_DDR_WB = 0x{ddr.layer_wb_ddr(i):08x}\n")
            f.write(f"LAYER_{i}_DDR_OFB = 0x{ofb_ddr:08x}\n")
            f.write(f"LAYER_{i}_DDR_RDMA = 0x{ddr.layer_rdma_ddr(i):08x}\n")
    print(f"\n  Meta written to {meta_path}")

    # ---- SMC meta 写出 (给 TB 用) ----
    if smc:
        # 每 layer SMC mode + my_core
        #   mode 'A' = 单核独占, my_core = 该核 id
        #   mode 'W' = W slice 多核, my_core = -1
        #   mode 'C' = cout slice 多核, my_core = IFM 源 mem core (集中存放在哪)
        layer_smc_mode        = ['?'] * len(layers)
        layer_smc_my_core     = [-1]  * len(layers)
        layer_smc_root_slot   = [-1]  * len(layers)
        layer_smc_cout_starts = [None] * len(layers)   # cout slice 时 4 核段起点
        layer_smc_cout_widths = [None] * len(layers)   # cout slice 时 4 核段宽 (cout 通道数)
        for c in range(n_cores):
            for s in per_core_plan[c]:
                n_split = len(s.cores_all)
                if n_split == 1:
                    layer_smc_mode[s.layer_idx]    = 'A'
                    layer_smc_my_core[s.layer_idx] = s.my_core
                elif s.mode == 'C_cout_slice':
                    layer_smc_mode[s.layer_idx] = 'C'
                    if layer_smc_cout_starts[s.layer_idx] is None:
                        starts, widths, _cs = mesh_cmd.compute_cout_segments(
                            s.layer.c_out, n_split)
                        layer_smc_cout_starts[s.layer_idx] = starts
                        layer_smc_cout_widths[s.layer_idx] = widths
                    # IFM 源 mem core: layer 0 / root 都是 mem[0], 否则上层 mode A 的 my_core
                    if s.layer_idx == 0 or s.layer_idx in root_ifb_idx:
                        layer_smc_my_core[s.layer_idx] = 0
                    else:
                        for ps in per_core_plan[0]:
                            if ps.layer_idx == s.layer_idx - 1 and len(ps.cores_all) == 1:
                                layer_smc_my_core[s.layer_idx] = ps.my_core
                                break
                else:
                    layer_smc_mode[s.layer_idx] = 'W'
        for i, cd in enumerate(chain_data):
            if i in root_ifb_idx:
                layer_smc_root_slot[i] = root_ifb_idx[i]

        with open(meta_path, 'a') as f:
            f.write(f"\n; ---- SMC + NUMA layout ----\n")
            f.write(f"SMC = 1\n")
            f.write(f"SMC_N_MEM = {n_cores}\n")
            f.write(f"SMC_MEM_STRIDE = 0x{SMC_MEM_STRIDE:08x}\n")
            f.write(f"SMC_INPUT_BASE = 0x{SMC_INPUT_BASE:08x}\n")
            f.write(f"SMC_FINAL_OFM_BASE = 0x{SMC_FINAL_OFM_BASE:08x}\n")
            f.write(f"SMC_LAYER_DATA_OFFSET = 0x{SMC_LAYER_DATA_OFFSET:08x}\n")
            # 每 layer 维度 (含 s2d 等效维度) + mode
            for i, layer in enumerate(layers):
                h_in_e, w_in_e, k_e, c_in_e, stride_e = layer.s2d_eff()
                pad_e = 0 if layer.force_s2d else layer.pad
                cin_slices  = (c_in_e      + 16 - 1) // 16
                cout_slices = (layer.c_out + 16 - 1) // 16
                f.write(f"SMC_LAYER_{i}_H_IN  = {h_in_e}\n")
                f.write(f"SMC_LAYER_{i}_W_IN  = {w_in_e}\n")
                f.write(f"SMC_LAYER_{i}_C_IN_SLICES = {cin_slices}\n")
                f.write(f"SMC_LAYER_{i}_C_OUT_SLICES = {cout_slices}\n")
                f.write(f"SMC_LAYER_{i}_PAD = {pad_e}\n")
                f.write(f"SMC_LAYER_{i}_K   = {k_e}\n")
                f.write(f"SMC_LAYER_{i}_STRIDE = {stride_e}\n")
                f.write(f"SMC_LAYER_{i}_MODE = {layer_smc_mode[i]}\n")
                f.write(f"SMC_LAYER_{i}_MODE_A_CORE = {layer_smc_my_core[i]}\n")
                f.write(f"SMC_LAYER_{i}_ROOT_SLOT = {layer_smc_root_slot[i]}\n")
                f.write(f"SMC_LAYER_{i}_HAS_RESIDUAL = {1 if cd_get(chain_data, i, 'has_residual', 0) else 0}\n")
                if layer_smc_mode[i] == 'C':
                    # cout slice 时输出每核 cout 段 (TB 用来切 WB / 验证 OFM)
                    f.write(f"SMC_LAYER_{i}_COUT_SEG_STARTS = {' '.join(str(x) for x in layer_smc_cout_starts[i])}\n")
                    f.write(f"SMC_LAYER_{i}_COUT_SEG_WIDTHS = {' '.join(str(x) for x in layer_smc_cout_widths[i])}\n")
            for core_id in range(n_cores):
                for layer_idx in range(len(layers)):
                    ic = smc_per_layer_cmds.get(('idma', core_id, layer_idx))
                    oc = smc_per_layer_cmds.get(('odma', core_id, layer_idx))
                    if ic is None: continue
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_IDMA_CMD_BASE = 0x{ic['ptr']:08x}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_IDMA_CMD_COUNT = {ic['count']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_IDMA_CMD_FILE = {ic['file']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_ODMA_CMD_BASE = 0x{oc['ptr']:08x}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_ODMA_CMD_COUNT = {oc['count']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_ODMA_CMD_FILE = {oc['file']}\n")
            # Per-(core, layer) W slice geom (TB 用来 stitch OFM)
            #   - mode 'A' / 'C' 时不需要 W geom (TB 不读), 跳过避免 W=1 等畸形几何
            for core_id in range(n_cores):
                for step in per_core_plan[core_id]:
                    if step.mode != 'C_w_slice':
                        continue
                    layer = step.layer
                    layer_idx = step.layer_idx
                    h_in_e, w_in_e, k_e, c_in_e, stride_e = layer.s2d_eff()
                    pad_e = 0 if layer.force_s2d else layer.pad
                    n_split = len(step.cores_all)
                    slice_idx = step.cores_all.index(step.my_core)
                    geom = hw_files.compute_w_slice_geom(
                        W_full=w_in_e, K=k_e, stride=stride_e,
                        pad_left_full=pad_e,
                        my_core=slice_idx, n_split=n_split,
                    )
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_W_OUT_START = {geom['w_out_start']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_MY_W_OUT = {geom['my_w_out']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_SUB_W_IN = {geom['sub_W']}\n")
                    f.write(f"SMC_CORE_{core_id}_LAYER_{layer_idx}_W_IN_START = {geom['w_in_start']}\n")
            # Per-layer mem 散布 layout (TB preload IFB + check OFM 用, 替代手算切片公式)
            #   driver 算的 mem 段宽 + 起点, TB 直接读, 切片公式不再两份独立实现.
            for i, layer in enumerate(layers):
                h_in_e, w_in_e, k_e, c_in_e, stride_e = layer.s2d_eff()
                pad_e = 0 if layer.force_s2d else layer.pad
                w_out_full = (w_in_e + 2 * pad_e - k_e) // stride_e + 1
                ifm_starts, ifm_widths = mesh_cmd.compute_smc_w_segments(w_in_e,    n_cores)
                ofm_starts, ofm_widths = mesh_cmd.compute_smc_w_segments(w_out_full, n_cores)
                f.write(f"SMC_LAYER_{i}_IFM_SEG_WIDTHS = {' '.join(str(x) for x in ifm_widths)}\n")
                f.write(f"SMC_LAYER_{i}_IFM_SEG_STARTS = {' '.join(str(x) for x in ifm_starts)}\n")
                f.write(f"SMC_LAYER_{i}_OFM_SEG_WIDTHS = {' '.join(str(x) for x in ofm_widths)}\n")
                f.write(f"SMC_LAYER_{i}_OFM_SEG_STARTS = {' '.join(str(x) for x in ofm_starts)}\n")
        print(f"  SMC meta appended to {meta_path}")

    return chain_data, per_core_plan


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case_name', default='multicore_demo',
                        help='输出目录名 (在 sim/tb_multicore/cases 下)')
    parser.add_argument('--n_cores', type=int, default=2)
    parser.add_argument('--smc', action='store_true',
                        help='SMC + NUMA 模式 (Phase 7, 主线): 走 multicore_top_smc DUT (4 ConvCore + '
                             'axi_smc_4to4 IP + 4 mem). 强制全核 W slice + 每层 IFM/OFM W '
                             '4 等分散布到 4 mem (halo 物理只一份). 生成 IDMA + ODMA SG cmd list '
                             'per-(core, layer), 跨 mem 边界 axi_crossbar IP 自动路由.')
    parser.add_argument('--demo',
                        choices=['wslice1', 'simple2', 'simple3', 'wslice4', 'wslice5',
                                 'wslice_mixed', 'wslice_stride2', 'wslice_oddw',
                                 'wslice_smallw', 'wslice_k7', 'wslice_k1',
                                 'patch1', 'patch_small', 'patch_s2d', 'patch_s2d_resnet',
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
    elif args.demo == 'patch1':
        # ResNet11 layer 0 简化版: K=1 stride=1 c_in=64 c_out=16 (= s2d 后等效)
        # H=240 W=135 (整图 patch s2d 后维度), cin_slices=4 cout_slices=1
        layers = [
            scheduler.Layer('Patch1', k=1, c_in=64, c_out=16, h_in=240, w_in=135,
                            stride=1, pad=0, sdp_shift=4),
        ]
    elif args.demo == 'patch_s2d':
        # 完全模拟 ResNet11 Patch (K=4 stride=4 c=4) + force_s2d (auto by property), 单层验证
        layers = [
            scheduler.Layer('PatchS2D', k=4, c_in=4, c_out=16, h_in=960, w_in=540,
                            stride=4, pad=0, sdp_shift=4),
        ]
    elif args.demo == 'patch_s2d_resnet':
        # ResNet11 Patch with same SDP params (shift=5, clip[0,127], round_en=1, relu=1)
        layers = [
            scheduler.Layer('PatchSR', k=4, c_in=4, c_out=16, h_in=960, w_in=540,
                            stride=4, pad=0, sdp_shift=5,
                            sdp_clip_max=127, sdp_round_en=1),
        ]
    elif args.demo == 'patch_small':
        # 同上但小尺寸: H=32 W=32 验证 cs_in=4 W slice 是否 PASS
        layers = [
            scheduler.Layer('PSmall', k=1, c_in=64, c_out=16, h_in=32, w_in=32,
                            stride=1, pad=0, sdp_shift=4),
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

    # SMC 模式输出到独立目录 sim/tb_smc/cases/{case_name}
    if args.smc:
        out_root = os.path.join(_THIS_DIR, '..', 'sim', 'tb_smc', 'cases', args.case_name)
    else:
        out_root = os.path.join(_THIS_DIR, '..', 'sim', 'tb_multicore', 'cases', args.case_name)
    if os.path.exists(out_root):
        shutil.rmtree(out_root)
    os.makedirs(out_root, exist_ok=True)

    if args.smc and args.n_cores != 4:
        sys.exit("ERROR: --smc only supports n_cores=4 (4 SI / 4 MI crossbar fixed topology)")
    chain_data, per_core_plan = run_multicore_chain(layers, args.n_cores, out_root, smc=args.smc)

    print(f"\n=== Done. Output: {out_root} ===")
