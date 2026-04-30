"""
gen_cross_core_test.py  --  M2 跨核两层 chain 测试生成器

生成两层 conv chain, 拆成两份 case 目录:
  cases/<case_name>/core0/   layer 0 cfg + ifb.txt + wb.txt + desc_list.hex (producer)
  cases/<case_name>/core1/   layer 1 cfg + wb.txt + desc_list.hex + expected_ofm.txt (consumer)

Layer 0 (producer):
  - IDMA 从 DDR 拉 IFB
  - ODMA push OFM 到 0x90000000+ (Core 1 IFB region) -- 不写 DDR
  - DDR 中没有 layer 0 OFB 区
Layer 1 (consumer):
  - SKIP_IDMA=1: 不启 IDMA, IFB 等远端 push 进来
  - ODMA 写 OFM 到 DDR 给 host 比对

DDR layout (multicore TB 用 base_offset 0 在 Core 0 视角):
  0x0000_0000  Layer 0 IFB
  0x0080_0000  Layer 0 WB
  0x0090_0000  Layer 1 WB
  0x00A0_0000  Layer 1 OFB (golden 比对)
  0x00B0_0000  Layer 0 desc list
  0x00C0_0000  Layer 1 desc list

Core 1 视角 base_offset = 0 (Core 1 不需要独立 DDR 区, 所有 DDR 共享 Core 0 的).
Core 1 收到的 host AXI-Lite 写仅写 boot regs (DESC_LIST_BASE=0xC00000, DESC_COUNT, CTRL).
"""

import os
import sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import gen_isa_test
import hw_files


# ---------------------------------------------------------------------------
# Cross-core IFB region (跟 multicore_top.sv 全局地址映射对齐)
# ---------------------------------------------------------------------------
def core_ifb_axi_base(core_id):
    """0x80000000 + core_id * 0x10000000 -- multicore_top crossbar BASE_ADDR."""
    return 0x80000000 + core_id * 0x10000000


# ---------------------------------------------------------------------------
# DDR layout (双层 chain 的固定 region 划分)
# ---------------------------------------------------------------------------
DDR_L0_IFB_BASE   = 0x0000_0000
DDR_L0_WB_BASE    = 0x0080_0000
DDR_L1_WB_BASE    = 0x0090_0000
DDR_L1_OFB_BASE   = 0x00A0_0000
DDR_L0_DESC_BASE  = 0x00B0_0000
DDR_L1_DESC_BASE  = 0x00C0_0000
DDR_L0_RDMA_BASE  = 0x00D0_0000   # bias/shortcut data (避开 IFB/WB 区)
DDR_L1_RDMA_BASE  = 0x00E0_0000


def gen_two_layer_chain(
    case_name,
    out_root,
    # Layer 0 (Patch-like, K x K stride S, c_in_l0 → c_out_l0)
    h_in_l0, w_in_l0, k_l0, c_in_l0, c_out_l0, stride_l0=1, pad_l0=None,
    shift_l0=0, mult_l0=1, zp_l0=0, clip_min_l0=0, clip_max_l0=127,
    relu_l0=1, round_l0=1,
    # Layer 1 (后续 conv, c_out_l0 → c_out_l1)
    k_l1=3, c_out_l1=None, stride_l1=1, pad_l1=None,
    shift_l1=0, mult_l1=1, zp_l1=0, clip_min_l1=0, clip_max_l1=127,
    relu_l1=1, round_l1=1,
    # 平台 (tile_w=16 让两层都成 NUM_TILES>1 + 整 tile, 避开 partial-last-tile 的 corner case)
    seed=42, tile_w=16, hw_pe=16, hw_col=16,
    consumer_core_id=1,
):
    """生成两层 chain. 返回 dict 含两核的 case_dir + meta."""
    if pad_l0 is None: pad_l0 = (k_l0 - 1) // 2 if k_l0 > 1 else 0
    if pad_l1 is None: pad_l1 = (k_l1 - 1) // 2 if k_l1 > 1 else 0
    if c_out_l1 is None: c_out_l1 = c_out_l0

    case_root  = os.path.join(out_root, case_name)
    core0_dir  = os.path.join(case_root, 'core0')
    core1_dir  = os.path.join(case_root, 'core1')
    os.makedirs(core0_dir, exist_ok=True)
    os.makedirs(core1_dir, exist_ok=True)

    # -----------------------------------------------------------------------
    # Layer 0 (producer @ Core 0)
    #   - 从 DDR 拉 IFB (IDMA), 算完 push 给 Core 1 IFB (ODMA_DST_BASE = AXI base of Core 1 IFB)
    #   - 不写 DDR OFB (skip_ofb_clear=True 避免 TB 干扰)
    # -----------------------------------------------------------------------
    consumer_ifb_axi_base = core_ifb_axi_base(consumer_core_id)
    print(f"== gen layer 0 (producer @ Core 0): "
          f"{h_in_l0}x{w_in_l0}x{c_in_l0} → ?x?x{c_out_l0}, ODMA→0x{consumer_ifb_axi_base:08x}")

    ret_l0 = gen_isa_test.generate_random(
        H_IN=h_in_l0, W_IN=w_in_l0, K=k_l0, NUM_CIN=c_in_l0, NUM_COUT=c_out_l0,
        TILE_W=tile_w, seed=seed, shift_amt=shift_l0, stride=stride_l0,
        HW_PE=hw_pe, HW_COL=hw_col, streaming=True,
        pad_top=pad_l0, pad_left=pad_l0, strip_rows=0,
        out_dir=core0_dir, case_name=f"{case_name}.L0|conv",
        ky_fold=False, s2d=False, residual=False,
        sdp_mult=mult_l0, sdp_zp_out=zp_l0,
        sdp_clip_min=clip_min_l0, sdp_clip_max=clip_max_l0,
        sdp_round_en=round_l0, sdp_relu_en=relu_l0,
        ddr_ifb_base =DDR_L0_IFB_BASE,
        ddr_wb_base  =DDR_L0_WB_BASE,
        ddr_ofb_base =consumer_ifb_axi_base,   # KEY: 指 Core 1 IFB AXI region
        ddr_desc_base=DDR_L0_DESC_BASE,
        ddr_rdma_base=DDR_L0_RDMA_BASE,
        skip_ifb_preload=False,                # Core 0 IFB 由 DDR preload
        skip_ofb_clear  =True,                 # OFB 不在 DDR, TB 不要清 (会清 Core 1 IFB region)
        skip_idma       =False,                # Core 0 用本地 IDMA
    )
    h_out_l0, w_out_l0 = ret_l0['H_OUT'], ret_l0['W_OUT']
    ofm_l0 = ret_l0['ofm_arr']
    print(f"   Layer 0 done: ofm shape={h_out_l0}x{w_out_l0}x{c_out_l0}, "
          f"ifb={ret_l0['ifb_words']}w wb={ret_l0['wb_words']}w ofb={ret_l0['ofb_words']}w")

    # -----------------------------------------------------------------------
    # Layer 1 (consumer @ Core 1)
    #   - SKIP_IDMA=1: IFB 等 push 进来, 本地 IDMA 不启
    #   - ifm_arr_in = Layer 0 OFM (生成时用作 expected_ofm 计算的输入)
    #   - skip_ifb_preload=True: TB 不要 readmemh ifb.txt 到 Core 1 IFB region
    #   - ODMA 写 DDR (Core 1 视角 OFB_BASE = DDR_L1_OFB_BASE)
    # -----------------------------------------------------------------------
    print(f"== gen layer 1 (consumer @ Core {consumer_core_id}): "
          f"{h_out_l0}x{w_out_l0}x{c_out_l0} → ?x?x{c_out_l1}, SKIP_IDMA=1")

    ret_l1 = gen_isa_test.generate_random(
        H_IN=h_out_l0, W_IN=w_out_l0, K=k_l1, NUM_CIN=c_out_l0, NUM_COUT=c_out_l1,
        TILE_W=tile_w, seed=seed + 100, shift_amt=shift_l1, stride=stride_l1,
        HW_PE=hw_pe, HW_COL=hw_col, streaming=True,
        pad_top=pad_l1, pad_left=pad_l1, strip_rows=0,
        out_dir=core1_dir, case_name=f"{case_name}.L1|conv",
        ky_fold=False, s2d=False, residual=False,
        ifm_arr_in=ofm_l0,                     # KEY: 用 L0 OFM 作 IFM (golden 计算依据)
        sdp_mult=mult_l1, sdp_zp_out=zp_l1,
        sdp_clip_min=clip_min_l1, sdp_clip_max=clip_max_l1,
        sdp_round_en=round_l1, sdp_relu_en=relu_l1,
        ddr_ifb_base =0,                       # IDMA 不启, 任意值 OK
        ddr_wb_base  =DDR_L1_WB_BASE,
        ddr_ofb_base =DDR_L1_OFB_BASE,
        ddr_desc_base=DDR_L1_DESC_BASE,
        ddr_rdma_base=DDR_L1_RDMA_BASE,
        skip_ifb_preload=True,                 # KEY: IFB 不预加载, 等 push
        skip_ofb_clear  =False,                # OFB 在 DDR, TB 清 0
        skip_idma       =True,                 # KEY: cfg.SKIP_IDMA=1
    )
    h_out_l1, w_out_l1 = ret_l1['H_OUT'], ret_l1['W_OUT']
    print(f"   Layer 1 done: ofm shape={h_out_l1}x{w_out_l1}x{c_out_l1}, "
          f"ifb={ret_l1['ifb_words']}w wb={ret_l1['wb_words']}w ofb={ret_l1['ofb_words']}w")

    return {
        'case_root' : case_root,
        'core0_dir' : core0_dir,
        'core1_dir' : core1_dir,
        'l0_meta'   : {'h_in': h_in_l0, 'w_in': w_in_l0, 'c_in': c_in_l0, 'c_out': c_out_l0,
                       'h_out': h_out_l0, 'w_out': w_out_l0,
                       'ifb_words': ret_l0['ifb_words'], 'wb_words': ret_l0['wb_words'],
                       'ofb_words': ret_l0['ofb_words']},
        'l1_meta'   : {'h_in': h_out_l0, 'w_in': w_out_l0, 'c_in': c_out_l0, 'c_out': c_out_l1,
                       'h_out': h_out_l1, 'w_out': w_out_l1,
                       'ifb_words': ret_l1['ifb_words'], 'wb_words': ret_l1['wb_words'],
                       'ofb_words': ret_l1['ofb_words']},
    }


if __name__ == '__main__':
    # 默认生成一个小 case: 30x30x8 → conv3x3 → 30x30x16 → conv3x3 → 30x30x16
    out_root = os.path.join(SCRIPT_DIR, '..', 'sim', 'tb_multicore', 'cases')
    # cin=8 不 fold (验证之前怀疑的 bug 实际是 RDMA 地址冲突, 已修)
    info = gen_two_layer_chain(
        case_name='cross00',
        out_root=out_root,
        h_in_l0=30, w_in_l0=30,
        k_l0=3, c_in_l0=8, c_out_l0=16, stride_l0=1, shift_l0=5,
        clip_max_l0=255, round_l0=0,
        k_l1=3, c_out_l1=16, stride_l1=1, shift_l1=5,
        clip_max_l1=255, round_l1=0,
    )
    print()
    print("Generated cross-core 2-layer test:")
    print(f"  Core 0 dir: {info['core0_dir']}")
    print(f"  Core 1 dir: {info['core1_dir']}")
