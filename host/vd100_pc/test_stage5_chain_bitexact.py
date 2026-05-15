#!/usr/bin/env python3
"""Stage 5 (bit-exact): 真 2-layer chain — L1 expected 基于 L0 OFM 算.

直接 import hw_files (不走 subprocess), 用 generate_random 的 ifm_arr_in 参数
让 L1 用 L0 OFM 作 IFM, 计算正确 expected_ofm.

board 流程:
  1. 跑 L0 → check L0 OFM == expected_0
  2. 取 board L0 OFM bytes → reshape 回 ofm_arr [H][W][C]
  3. 用 L0 ofm_arr 作 L1 IFM, 调 generate_random 生成 L1 case + expected_1
  4. 跑 L1 → check L1 OFM == expected_1
"""
import sys, struct, time, os
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import hw_files
import gen_isa_test
from test_stage4_bitexact import (
    BRAM_BASE, DESC_OFF, ISG_OFF, OSG_OFF, BRAM_ISG, BRAM_OSG,
    compute_layout,
    CTRL_START_DFE, CTRL_START_LAYER, CTRL_SOFT_RESET,
    parse_hex_file, beat_pair_to_bytes, make_sg_idma, make_sg_odma,
    chunked_load, chunked_read
)


def gen_case_in_dir(case_dir, ifm_arr_in=None, **kwargs):
    """直接调 generate_random, 返回 ofm_arr + 其他 metadata."""
    os.makedirs(case_dir, exist_ok=True)
    return gen_isa_test.generate_random(
        out_dir=case_dir,
        ifm_arr_in=ifm_arr_in,
        seed=42, shift_amt=0, streaming=False,
        **kwargs)


def parse_ofm_bytes_to_arr(ofm_bytes, H_OUT, W_OUT, NUM_COUT, HW_COL=16):
    """expected_ofm.txt 格式: word = (yout, x, cs), 16 byte (16 cout) / word.
    bytes 顺序: word[0]=ofm[0][0][0..15] little-endian (LSB byte 在低地址).
    return [H_OUT][W_OUT][NUM_COUT] int (signed int8 -> 0..255)"""
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
    arr = [[[0] * NUM_COUT for _ in range(W_OUT)] for _ in range(H_OUT)]
    word_idx = 0
    for yout in range(H_OUT):
        for x in range(W_OUT):
            for cs in range(cout_slices):
                local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
                word_bytes = ofm_bytes[word_idx*16 : (word_idx+1)*16]
                for lc in range(local_cout):
                    cout = cs * HW_COL + lc
                    arr[yout][x][cout] = word_bytes[lc]
                word_idx += 1
    return arr


def setup_case_for_board(case_dir, K, H_IN, W_IN, NUM_CIN, NUM_COUT, stride, pad,
                          residual_en=0, shortcut_mult=0, shortcut_shift=0):
    """读 case_dir 下 ifb.txt/wb.txt/rdma_data.txt/expected_ofm.txt/desc_list.hex,
    构造 board load 需要的 bytes + SG cmd list + cfg."""
    # 历史 host force-streaming workaround 已删除: 真因是 ofb_strip_rows 6-bit
    # 截断 (commit 13a0797 fix 6→8 bit). 整图 fit case 现在 work.
    cfg = hw_files.derive_layer_cfg(
        H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
        stride=stride, pad_top=pad, pad_left=pad, TILE_W=32)

    H_IN_eff, H_OUT = cfg['H_IN'], cfg['H_OUT']

    # 读 case_dir 的 config.txt 拿到 rdma_words_total (residual case 含 shortcut 段)
    rdma_words_total = cfg['rdma_words']
    cfg_txt_path = os.path.join(case_dir, 'config.txt')
    if os.path.exists(cfg_txt_path):
        with open(cfg_txt_path) as f:
            for line in f:
                if line.startswith('_META_RDMA_WORDS'):
                    rdma_words_total = int(line.split('=')[1].strip())
                    break

    # 读 case_dir 数据先 (要算 region size)
    ifm = parse_hex_file(os.path.join(case_dir, 'ifb.txt'))
    wb_bytes = parse_hex_file(os.path.join(case_dir, 'wb.txt'))
    rdma = parse_hex_file(os.path.join(case_dir, 'rdma_data.txt'))
    expected = parse_hex_file(os.path.join(case_dir, 'expected_ofm.txt'))

    # v9 fix: 动态 BRAM layout 避免 region overlap (大 case IFM>44KB 被 WB 覆盖 bug)
    ifm_off, wb_off, rdma_off, ofm_off = compute_layout(
        len(ifm), len(wb_bytes), len(rdma), len(expected))
    BRAM_IFM  = BRAM_BASE + ifm_off
    BRAM_WB   = BRAM_BASE + wb_off
    BRAM_RDMA = BRAM_BASE + rdma_off
    BRAM_OFM  = BRAM_BASE + ofm_off

    # cfg_dict + desc list (重新构造, 不复用 gen_isa_test 写的 desc_list.hex
    # 因为 SG cmd cfg 我们要 board 路径专用)
    # residual case: shortcut_mult=1 shortcut_shift=0 默认 (cfg_to_dict default)
    cfg_dict = hw_files.cfg_to_dict(
        cfg, shift_amt=0, sdp_mult=1, sdp_zp_out=0,
        sdp_clip_min=0, sdp_clip_max=255, sdp_round_en=0, sdp_relu_en=1,
        ddr_ifb_base=BRAM_IFM, ddr_wb_base=BRAM_WB,
        ddr_ofb_base=BRAM_OFM, ddr_rdma_base=BRAM_RDMA,
        rdma_words_total=rdma_words_total,
        residual_en=residual_en, shortcut_mult=shortcut_mult, shortcut_shift=shortcut_shift,
        idma_cmd_list_ptr=BRAM_ISG, idma_cmd_count=H_IN_eff, idma_cmds_per_row=1,
        odma_cmd_list_ptr=BRAM_OSG, odma_cmd_count=H_OUT,   odma_cmds_per_row=1,
        skip_idma=0)

    seg = hw_files.build_layer_desc_segment(
        cfg_dict, H_IN=cfg['H_IN'], W_IN=cfg['W_IN'],
        H_OUT=cfg['H_OUT'], W_OUT=cfg['W_OUT'],
        cin_slices=cfg['cin_slices'], cout_slices=cfg['cout_slices'],
        pad_top=cfg['pad_top'], pad_bot=cfg['pad_bot'],
        pad_left=cfg['pad_left'], pad_right=cfg['pad_right'],
        strip_rows=0)
    seg.append(hw_files._pack_desc(hw_files.TYPE_END, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    desc_bytes = b''.join(beat_pair_to_bytes(b0, b1) for (b0, b1) in seg)

    # SG cmd (用 dynamic BRAM_IFM/OFM 算 src/dst addr)
    cs_in = cfg['cin_slices']; cs_out = cfg['cout_slices']
    row_bytes_ifm = cfg['W_IN'] * cs_in * 16
    row_bytes_ofm = cfg['W_OUT'] * cs_out * 16
    ifb_ring = cfg['ifb_ring_words']
    isg_list = []; sram_off_raw = 0
    for r in range(H_IN_eff):
        last = 1 if r == H_IN_eff - 1 else 0
        sram_off_eff = sram_off_raw % ifb_ring if ifb_ring > 0 else sram_off_raw
        isg_list.append(make_sg_idma(BRAM_IFM + r*row_bytes_ifm,
                                      btt=row_bytes_ifm, sram_off=sram_off_eff, last_cmd=last))
        sram_off_raw += cfg['W_IN'] * cs_in
    isg_blob = b''.join(isg_list)
    osg_list = []
    for r in range(H_OUT):
        last = 1 if r == H_OUT - 1 else 0
        osg_list.append(make_sg_odma(BRAM_OFM + r*row_bytes_ofm,
                                      btt=row_bytes_ofm, ofb_w_start=0, last_cmd=last))
    osg_blob = b''.join(osg_list)
    # v9: 返回额外 layout offsets 给调用方 load BRAM 用
    return desc_bytes, len(seg), cfg, ifm, wb_bytes, rdma, expected, isg_blob, osg_blob, \
           (BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM)


def run_board_layer(rpc, desc, n_desc, ifm, wb, rdma, expected, isg, osg):
    """跑一层 board, return (ok, ofm, err)."""
    rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET); time.sleep(0.005)
    chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
    chunked_load(rpc, BRAM_ISG, isg)
    chunked_load(rpc, BRAM_OSG, osg)
    chunked_load(rpc, BRAM_IFM, ifm)
    chunked_load(rpc, BRAM_WB, wb)
    chunked_load(rpc, BRAM_RDMA, rdma)
    chunked_load(rpc, BRAM_OFM, b'\xAA' * len(expected))

    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)
    rpc.poke_csr(0, 0x000, CTRL_START_DFE)
    for _ in range(50):
        if (rpc.peek_csr(0, 0x004) >> 9) & 1: break
        time.sleep(0.01)
    rpc.poke_csr(0, 0x000, CTRL_START_LAYER)
    for _ in range(500):
        if (rpc.peek_csr(0, 0x004) >> 11) & 1: break
        time.sleep(0.01)
    else:
        return False, b'', 'layer stuck'
    ofm = chunked_read(rpc, BRAM_OFM, len(expected))
    if ofm != expected:
        diff = sum(1 for a, b in zip(ofm, expected) if a != b)
        return False, ofm, f'mismatch {diff}/{len(expected)}'
    return True, ofm, None


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== Stage 5 chain bit-exact: 2 layer ===')

    # Layer 0: K=3 H=W=8 Cin=Cout=16 (random IFM)
    print()
    print('[L0] generate_random (random IFM, compute expected)')
    case_dir_l0 = r'C:/_Project/FLUX_CNN/sim/tb_stage5_chain_l0'
    ret0 = gen_case_in_dir(case_dir_l0, ifm_arr_in=None,
                            H_IN=8, W_IN=8, K=3, NUM_CIN=16, NUM_COUT=16,
                            TILE_W=32, stride=1, pad_top=1, pad_left=1)
    # ret0['ofm_arr'] = L0 OFM (Python int H_OUT × W_OUT × NUM_COUT)

    desc0, n_desc0, cfg0, ifm0, wb0, rdma0, exp0, isg0, osg0 = setup_case_for_board(
        case_dir_l0, K=3, H_IN=8, W_IN=8, NUM_CIN=16, NUM_COUT=16, stride=1, pad=1)
    ok0, board_ofm0, err0 = run_board_layer(rpc, desc0, n_desc0, ifm0, wb0, rdma0, exp0, isg0, osg0)
    if not ok0:
        print(f'  [L0] FAIL: {err0}'); return 1
    print(f'  [L0] PASS  OFM {len(exp0)} byte bit-exact (board vs Python expected)')

    # Layer 1: 用 L0 expected OFM (Python 端 ofm_arr) 作 IFM, compute expected_1
    # 这里用 Python 端的 ofm_arr (ret0['ofm_arr']), 不是 board 读回的 — 因为 board L0
    # 已经 bit-exact PASS 跟 Python 一致, 两者数值相同.
    print()
    print('[L1] generate_random (ifm_arr_in = L0 OFM, compute new expected)')
    case_dir_l1 = r'C:/_Project/FLUX_CNN/sim/tb_stage5_chain_l1'
    ret1 = gen_case_in_dir(case_dir_l1, ifm_arr_in=ret0['ofm_arr'],
                            H_IN=8, W_IN=8, K=3, NUM_CIN=16, NUM_COUT=16,
                            TILE_W=32, stride=1, pad_top=1, pad_left=1)

    desc1, n_desc1, cfg1, ifm1, wb1, rdma1, exp1, isg1, osg1 = setup_case_for_board(
        case_dir_l1, K=3, H_IN=8, W_IN=8, NUM_CIN=16, NUM_COUT=16, stride=1, pad=1)
    ok1, board_ofm1, err1 = run_board_layer(rpc, desc1, n_desc1, ifm1, wb1, rdma1, exp1, isg1, osg1)
    if not ok1:
        print(f'  [L1] FAIL: {err1}'); return 1
    print(f'  [L1] PASS  OFM {len(exp1)} byte bit-exact (board vs Python expected, IFM=L0 OFM)')

    print()
    print('[OK] Stage 5 真 chain bit-exact: 2 layer 全 bit-exact PASS')
    rpc.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
