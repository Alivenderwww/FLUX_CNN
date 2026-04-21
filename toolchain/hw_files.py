"""
hw_files.py — 硬件 DDR 文件 I/O 共享层

被 gen_isa_test.py (随机数据) 和 compile_layer.py (PyTorch 量化数据) 共用。
所有函数接受 pure Python 列表/dict，不依赖 numpy/torch。

输出文件 (仿真 tb_core_dma 需要)：
  - ifb.txt            : IFB DDR，NHWC，每行 1 word = NUM_PE×8 bit = 16 个 cin
  - wb.txt             : WB DDR，前缀 cout_slices 个 bias word (低 512bit 放 16 个 int32)
                         后接 weight (cs/cins/ky/kx 顺序) ，每行 1 word = 2048 bit
  - expected_ofm.txt   : 参考 OFM，NHWC (yout, x, cs)，每行 16 个 int8
  - config.txt         : k=v 逐行
  - desc_list.hex      : 每条 descriptor 2 × 128-bit AXI beat，每 beat 一行 32 hex chars
  - sim_params.f       : +KEY=val TB plusargs
"""

import os


def _out_path(out_dir, fname):
    return os.path.join(out_dir, fname)


# ---------------------------------------------------------------------------
# 工具：SDP 软件模拟（硬件等价）
# ---------------------------------------------------------------------------
def sdp_sim(psum_i32, mult, shift, zp_out, clip_min, clip_max, round_en, relu_en):
    """
    Per-tensor symmetric int8 SDP 硬件流水软件模拟。
    输入 psum_i32 是 int32 累加器值（已含 bias），输出 8-bit（raw byte 位模式）。
    """
    prod = psum_i32 * mult                    # int64
    if round_en and shift > 0:
        prod += 1 << (shift - 1)
    # Python 'signed right shift' for negative ints works as arithmetic
    if prod >= 0:
        q = prod >> shift
    else:
        q = -((-prod) >> shift) if (-prod) & ((1 << shift) - 1) == 0 else -(((-prod) >> shift) + 1)
    q_zp = q + zp_out
    if relu_en and q_zp < 0:
        q_zp = 0
    if q_zp < clip_min: q_zp = clip_min
    if q_zp > clip_max: q_zp = clip_max
    return q_zp & 0xFF


# ---------------------------------------------------------------------------
# IFB: NHWC (y, x, cin_slice) 顺序打包
# ---------------------------------------------------------------------------
def write_ifb(out_dir, ifm_arr, H_IN, W_IN, NUM_CIN, HW_PE):
    """
    ifm_arr: [H_IN][W_IN][NUM_CIN] int（可负，会 & 0xFF）
    每 word 存一个 (y,x,cin_slice) 段 = 16 个 cin 的 int8 (低 cin_local 在低位)
    一行 = W_IN × cin_slices words
    """
    cin_slices  = (NUM_CIN + HW_PE - 1) // HW_PE
    hex_chars   = HW_PE * 2
    lines = []
    for y in range(H_IN):
        for x in range(W_IN):
            for cins in range(cin_slices):
                local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
                val = 0
                for cin_local in range(local_cin):
                    cin = cins * HW_PE + cin_local
                    val |= (ifm_arr[y][x][cin] & 0xFF) << (cin_local * 8)
                lines.append(f"{val:0{hex_chars}X}")
    with open(_out_path(out_dir, 'ifb.txt'), 'w') as f:
        f.writelines(d + '\n' for d in lines)
    return len(lines)


# ---------------------------------------------------------------------------
# WB: bias prefix + weight (cs, cins, ky, kx) 顺序打包
# ---------------------------------------------------------------------------
def write_wb(out_dir, w_arr, bias_arr, K, NUM_CIN, NUM_COUT, HW_PE, HW_COL):
    """
    w_arr:    [K][K][NUM_COUT][NUM_CIN] int8
    bias_arr: [NUM_COUT] int32 或 None（None 等同全 0）
    WB 前 cout_slices 个 word 放 bias (每 word 低 NUM_COL × 32bit 放 16 个 int32)，
    其后 weight (cs, cins, ky, kx) 顺序，每个 (cs, cins, ky, kx) 1 word (2048bit)。
    """
    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
    wb_hex_chars = HW_COL * HW_PE * 2           # 2048 bit = 512 hex chars

    # bias prefix: 一 cs 一 word，低 16×32 bit 放 16 个 int32 bias
    bias_lines = []
    for cs in range(cout_slices):
        word = 0
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for lc in range(local_cout):
            cout = cs * HW_COL + lc
            bv = bias_arr[cout] if bias_arr is not None else 0
            word |= (bv & 0xFFFFFFFF) << (lc * 32)       # 低位先
        bias_lines.append(f"{word:0{wb_hex_chars}X}")

    # weight: cs × cins × ky × kx 顺序
    w_lines = []
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for cins in range(cin_slices):
            local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
            for ky in range(K):
                for kx in range(K):
                    word = 0
                    for lc in range(local_cout):
                        cout = cs * HW_COL + lc
                        cv = 0
                        for cin_local in range(local_cin):
                            cin = cins * HW_PE + cin_local
                            v = w_arr[ky][kx][cout][cin]
                            cv |= (v & 0xFF) << (cin_local * 8)
                        word |= cv << (lc * HW_PE * 8)
                    w_lines.append(f"{word:0{wb_hex_chars}X}")

    with open(_out_path(out_dir, 'wb.txt'), 'w') as f:
        f.writelines(d + '\n' for d in bias_lines)
        f.writelines(d + '\n' for d in w_lines)
    return len(bias_lines) + len(w_lines)


# ---------------------------------------------------------------------------
# Expected OFM: NHWC (yout, x, cs) 顺序
# ---------------------------------------------------------------------------
def write_expected_ofm(out_dir, ofm_arr, H_OUT, W_OUT, NUM_COUT, HW_COL):
    """
    ofm_arr: [H_OUT][W_OUT][NUM_COUT] int (0..255 for uint8 或 -128..127 for int8)
    每 word 存一个 (yout, x, cs) 段 = 16 个 cout 的 8-bit (& 0xFF)
    """
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
    hex_chars   = HW_COL * 2
    lines = []
    for yout in range(H_OUT):
        for px in range(W_OUT):
            for cs in range(cout_slices):
                local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
                word = 0
                for lc in range(local_cout):
                    cout = cs * HW_COL + lc
                    v = ofm_arr[yout][px][cout] & 0xFF
                    word |= v << (lc * 8)
                lines.append(f"{word:0{hex_chars}X}")
    with open(_out_path(out_dir, 'expected_ofm.txt'), 'w') as f:
        f.writelines(d + '\n' for d in lines)
    return len(lines)


# ---------------------------------------------------------------------------
# 计算 expected_ofm（硬件 int 流水软件模拟）
# ---------------------------------------------------------------------------
def compute_expected_ofm(
    H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride, pad_top, pad_left,
    ifm_arr, w_arr, bias_arr,
    sdp_mult, sdp_shift, sdp_zp_out, sdp_clip_min, sdp_clip_max,
    sdp_round_en, sdp_relu_en,
):
    """
    模拟硬件一轮 Conv + SDP 量化流水。返回 [H_OUT][W_OUT][NUM_COUT] 8-bit int。
    """
    H_OUT = (H_IN + 2 * pad_top - K) // stride + 1    # 假设对称 pad
    W_OUT = (W_IN + 2 * pad_left - K) // stride + 1
    ofm = [[[0] * NUM_COUT for _ in range(W_OUT)] for _ in range(H_OUT)]
    for yout in range(H_OUT):
        for px in range(W_OUT):
            for cout in range(NUM_COUT):
                psum = bias_arr[cout] if bias_arr is not None else 0
                for ky in range(K):
                    for kx in range(K):
                        iy = yout * stride + ky - pad_top
                        ix = px   * stride + kx - pad_left
                        if 0 <= iy < H_IN and 0 <= ix < W_IN:
                            for cin in range(NUM_CIN):
                                psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                        # pad: contributes 0
                ofm[yout][px][cout] = sdp_sim(
                    psum, sdp_mult, sdp_shift, sdp_zp_out,
                    sdp_clip_min, sdp_clip_max, sdp_round_en, sdp_relu_en)
    return ofm, H_OUT, W_OUT


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
def write_config(out_dir, cfg_dict):
    """k = v 每行，k 按 dict 插入顺序。_META_ 前缀的字段给 TB 用，不写 cfg_regs。"""
    with open(_out_path(out_dir, 'config.txt'), 'w') as f:
        for k, v in cfg_dict.items():
            f.write(f"{k} = {v}\n")


def append_config(out_dir, kv_dict):
    """append 行到 config.txt（descriptor 写完后补 DESC_COUNT / DESC_LIST_BASE）"""
    with open(_out_path(out_dir, 'config.txt'), 'a') as f:
        for k, v in kv_dict.items():
            f.write(f"{k} = {v}\n")


# ---------------------------------------------------------------------------
# Descriptor list
# ---------------------------------------------------------------------------
TYPE_NOP, TYPE_CONV, TYPE_BARRIER, TYPE_END = 0x0, 0x1, 0x2, 0xF
FLAG_IS_FIRST     = 1 << 0
FLAG_IS_LAST      = 1 << 1
FLAG_STREAMING_EN = 1 << 2


def _pack_desc(type_, flags, pad_t, pad_b, pad_l, pad_r,
               strip_y_start, n_yout, ifb_off, ifb_len, ofb_off, ofb_len):
    """256-bit descriptor → (beat0, beat1) 每 128-bit。"""
    w0 = ((type_ & 0xF)      << 0 ) | \
         ((flags & 0xF)      << 4 ) | \
         ((pad_t & 0xF)      << 8 ) | \
         ((pad_b & 0xF)      << 12) | \
         ((pad_l & 0xF)      << 16) | \
         ((pad_r & 0xF)      << 20)
    w1 = (strip_y_start & 0xFFFF) | ((n_yout & 0xFFFF) << 16)
    w2 = ifb_off & 0xFFFFF
    w3 = ifb_len & 0xFFFFFF
    w4 = ofb_off & 0xFFFFF
    w5 = ofb_len & 0xFFFFFF
    desc256 = (w0) | (w1 << 32) | (w2 << 64) | (w3 << 96) | (w4 << 128) | (w5 << 160)
    beat0 = desc256 & ((1 << 128) - 1)
    beat1 = (desc256 >> 128) & ((1 << 128) - 1)
    return beat0, beat1


def write_descriptors(
    out_dir, H_IN, W_IN, H_OUT, W_OUT, cin_slices, cout_slices,
    pad_top, pad_bot, pad_left, pad_right,
    strip_rows=0, streaming=False,
):
    """
    写 desc_list.hex + append DESC_COUNT/DESC_LIST_BASE 到 config.txt。
    返回 n_desc（含 END）。
    """
    strip_rows_eff = strip_rows if (strip_rows > 0 and strip_rows < H_OUT) else H_OUT
    n_strips = (H_OUT + strip_rows_eff - 1) // strip_rows_eff

    AXI_BYTES = 16
    ifb_bytes_per_row   = W_IN  * AXI_BYTES
    ifb_bytes_per_slice = H_IN  * ifb_bytes_per_row
    ifb_total_bytes     = ifb_bytes_per_slice * cin_slices
    ofb_bytes_per_row   = W_OUT * AXI_BYTES
    ofb_bytes_per_slice = H_OUT * ofb_bytes_per_row
    ofb_total_bytes     = ofb_bytes_per_slice * cout_slices

    descs = []
    for i in range(n_strips):
        strip_y_start = i * strip_rows_eff
        n_yout        = min(strip_rows_eff, H_OUT - strip_y_start)
        is_first      = (i == 0)
        is_last       = (i == n_strips - 1)
        local_pad_top = pad_top  if is_first else 0
        local_pad_bot = pad_bot  if is_last  else 0

        if n_strips == 1:
            ifb_off, ifb_len = 0, ifb_total_bytes
            ofb_off, ofb_len = 0, ofb_total_bytes
        else:
            ifb_off, ifb_len = 0, ifb_total_bytes
            ofb_off = strip_y_start * ofb_bytes_per_row
            ofb_len = n_yout * ofb_bytes_per_row * cout_slices

        flags = 0
        if is_first:  flags |= FLAG_IS_FIRST
        if is_last:   flags |= FLAG_IS_LAST
        if streaming: flags |= FLAG_STREAMING_EN

        descs.append(_pack_desc(
            TYPE_CONV, flags,
            local_pad_top, local_pad_bot, pad_left, pad_right,
            strip_y_start, n_yout,
            ifb_off, ifb_len, ofb_off, ofb_len))

    descs.append(_pack_desc(TYPE_END, 0, 0, 0, 0, 0, H_OUT, 0, 0, 0, 0, 0))

    with open(_out_path(out_dir, 'desc_list.hex'), 'w') as f:
        for (beat0, beat1) in descs:
            f.write(f"{beat0:032X}\n")
            f.write(f"{beat1:032X}\n")

    append_config(out_dir, {'DESC_COUNT': len(descs), 'DESC_LIST_BASE': 0})
    return len(descs), n_strips, strip_rows_eff


# ---------------------------------------------------------------------------
# sim_params.f
# ---------------------------------------------------------------------------
def write_sim_params(out_dir, H_OUT, W_OUT, cout_slices, ifb_words, wb_words,
                     desc_count, sram_depth, num_cin, num_cout):
    """只留 SRAM_DEPTH 作编译期 -g 参数；其它 meta 走 config.txt 供 TB 运行期读"""
    with open(_out_path(out_dir, 'sim_params.f'), 'w') as f:
        f.write(f"-gSRAM_DEPTH={sram_depth}\n")


# ---------------------------------------------------------------------------
# Derived cfg helpers (shared derive logic)
# ---------------------------------------------------------------------------
def derive_layer_cfg(H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride,
                     pad_top, pad_left, TILE_W=32, HW_PE=16, HW_COL=16,
                     WRF_DEPTH=32, ARF_DEPTH=32, streaming=False,
                     IFB_SRAM_WORDS=8192, OFB_SRAM_WORDS=8192):
    """
    派生 conv layer 的所有硬件 cfg 字段。返回 dict，含：
      派生 H/W/kk/slices/tile_w/num_tiles/last_valid_w/strip/pix_step/...
    调用方添加 bases/SDP/DMA_MODE 等后写 config.txt。
    """
    pad_bot, pad_right = pad_top, pad_left

    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

    H_OUT = (H_IN + pad_top + pad_bot - K) // stride + 1
    W_OUT = (W_IN + pad_left + pad_right - K) // stride + 1
    if H_OUT <= 0 or W_OUT <= 0:
        raise ValueError(f"invalid output {H_OUT}x{W_OUT}")

    kk          = K * K
    total_wrf   = kk * cin_slices
    wrf_packed  = total_wrf <= WRF_DEPTH

    # E-2 ARF reuse: cur_fill_len = cur_valid_w + K - 1 ≤ ARF_DEPTH → tile_w ≤ 33-K
    arf_reuse_en = (stride == 1 and K > 1)
    if arf_reuse_en:
        max_tile_w = ARF_DEPTH - K + 1
        if TILE_W > max_tile_w:
            TILE_W = max_tile_w

    num_tiles    = (W_OUT + TILE_W - 1) // TILE_W
    last_valid_w = W_OUT - (num_tiles - 1) * TILE_W

    rounds_per_cins = (kk + WRF_DEPTH - 1) // WRF_DEPTH
    round_len_last  = kk - (rounds_per_cins - 1) * WRF_DEPTH

    IFB_ROW_STEP  = stride * W_IN * cin_slices
    WB_COUT_STEP  = kk * cin_slices
    TILE_IN_STEP  = TILE_W * stride * cin_slices

    ifb_words = H_IN * W_IN * cin_slices
    wb_words  = kk * cout_slices * cin_slices + cout_slices   # +bias prefix
    ofb_words = H_OUT * W_OUT * cout_slices

    # Streaming strip 计算
    if streaming:
        ifb_row_words = W_IN * cin_slices
        ifb_strip_rows_max = IFB_SRAM_WORDS // ifb_row_words
        ifb_strip_rows_min = K + 1
        if ifb_strip_rows_max < ifb_strip_rows_min:
            raise ValueError(
                f"IFB SRAM 容量不足: W_IN*cin_slices={ifb_row_words}, "
                f"max rows={ifb_strip_rows_max} < K+1={ifb_strip_rows_min}")
        ifb_strip = min(ifb_strip_rows_min + 2, ifb_strip_rows_max, H_IN)

        ofb_row_words_calc = W_OUT * cout_slices
        ofb_strip_rows_max = OFB_SRAM_WORDS // ofb_row_words_calc
        ofb_strip_rows_min = 2
        if ofb_strip_rows_max < ofb_strip_rows_min:
            raise ValueError(
                f"OFB SRAM 容量不足: W_OUT*cout_slices={ofb_row_words_calc}, "
                f"max rows={ofb_strip_rows_max} < 2")
        ofb_strip = min(8, ofb_strip_rows_max, H_OUT)
        sram_depth = 8192
    else:
        ifb_strip = H_IN
        ofb_strip = H_OUT
        # 统一 8192 (与 streaming 一致；F-2 多 case 共享 -gSRAM_DEPTH，避免 case 间尺寸冲突)
        sram_depth = 8192

    # NHWC streaming 预算值
    if streaming:
        ifb_ring_words = ifb_strip * W_IN  * cin_slices
        ofb_ring_words = ofb_strip * W_OUT * cout_slices
    else:
        ifb_ring_words = H_IN  * W_IN  * cin_slices
        ofb_ring_words = H_OUT * W_OUT * cout_slices
    ofb_row_words = W_OUT * cout_slices
    dma_mode     = 0x3 if streaming else 0x0

    return {
        # 尺寸 / slicing
        'H_IN': H_IN, 'W_IN': W_IN, 'K': K,
        'NUM_CIN': NUM_CIN, 'NUM_COUT': NUM_COUT,
        'H_OUT': H_OUT, 'W_OUT': W_OUT,
        'cin_slices': cin_slices, 'cout_slices': cout_slices,
        'kk': kk, 'total_wrf': total_wrf, 'wrf_packed': wrf_packed,
        'rounds_per_cins': rounds_per_cins, 'round_len_last': round_len_last,
        'num_tiles': num_tiles, 'last_valid_w': last_valid_w,
        'TILE_W': TILE_W, 'stride': stride,
        'pad_top': pad_top, 'pad_bot': pad_bot,
        'pad_left': pad_left, 'pad_right': pad_right,
        'arf_reuse_en': arf_reuse_en,
        # 步长
        'IFB_ROW_STEP': IFB_ROW_STEP, 'WB_COUT_STEP': WB_COUT_STEP,
        'TILE_IN_STEP': TILE_IN_STEP,
        # 大小
        'ifb_words': ifb_words, 'wb_words': wb_words, 'ofb_words': ofb_words,
        'sram_depth': sram_depth,
        # Streaming
        'streaming': streaming, 'dma_mode': dma_mode,
        'ifb_strip': ifb_strip, 'ofb_strip': ofb_strip,
        'ifb_ring_words': ifb_ring_words, 'ofb_row_words': ofb_row_words,
        'ofb_ring_words': ofb_ring_words,
    }


def cfg_to_dict(cfg, shift_amt=0, sdp_mult=1, sdp_zp_out=0,
                sdp_clip_min=0, sdp_clip_max=255, sdp_round_en=0, sdp_relu_en=1,
                case_name=""):
    """
    把 derive_layer_cfg 的结果转成 config.txt 用的有序 dict。
    包含 SDP 量化参数（F-1a/F-1b 补齐）。
    """
    return {
        # --- META (TB 读取，不写 cfg_regs) ---
        # TB 从 config.txt 解析这些字段做仿真控制（OFB 对比循环 / 打印等）
        '_META_CASE_NAME'   : case_name,
        '_META_IFB_WORDS'   : cfg['ifb_words'],
        '_META_WB_WORDS'    : cfg['wb_words'],
        '_META_OFB_WORDS'   : cfg['ofb_words'],
        '_META_SRAM_DEPTH'  : cfg['sram_depth'],
        '_META_NUM_CIN'     : cfg['NUM_CIN'],
        '_META_NUM_COUT'    : cfg['NUM_COUT'],
        # --- cfg_regs 字段 ---
        'H_OUT'          : cfg['H_OUT'],
        'W_OUT'          : cfg['W_OUT'],
        'W_IN'           : cfg['W_IN'],
        'K'              : cfg['K'],
        'STRIDE'         : cfg['stride'],
        'CIN_SLICES'     : cfg['cin_slices'],
        'COUT_SLICES'    : cfg['cout_slices'],
        'TILE_W'         : cfg['TILE_W'],
        'TOTAL_WRF'      : cfg['total_wrf'],
        'WRF_PACKED'     : 1 if cfg['wrf_packed'] else 0,
        'KK'             : cfg['kk'],
        'ROUNDS_PER_CINS': cfg['rounds_per_cins'],
        'ROUND_LEN_LAST' : cfg['round_len_last'],
        'IFB_BASE'       : 0,
        'WB_BASE'        : cfg['cout_slices'],     # bias prefix 之后
        'OFB_BASE'       : 0,
        'IFB_ROW_STEP'   : cfg['IFB_ROW_STEP'],
        'WB_COUT_STEP'   : cfg['WB_COUT_STEP'],
        'NUM_TILES'      : cfg['num_tiles'],
        'LAST_VALID_W'   : cfg['last_valid_w'],
        'TILE_IN_STEP'   : cfg['TILE_IN_STEP'],
        'SDP_SHIFT'      : shift_amt,
        'SDP_RELU_EN'    : sdp_relu_en,
        'SDP_MULT'       : sdp_mult,
        'SDP_ZP_OUT'     : sdp_zp_out,
        'SDP_CLIP_MIN'   : sdp_clip_min,
        'SDP_CLIP_MAX'   : sdp_clip_max,
        'SDP_ROUND_EN'   : sdp_round_en,
        'PAD_TOP'        : cfg['pad_top'],
        'PAD_LEFT'       : cfg['pad_left'],
        'H_IN_TOTAL'     : cfg['H_IN'],
        'IFB_STRIP_ROWS' : cfg['ifb_strip'] if cfg['streaming'] else 0,
        'OFB_STRIP_ROWS' : cfg['ofb_strip'] if cfg['streaming'] else 0,
        'DDR_IFM_ROW_STRIDE' : cfg['W_IN']  * cfg['cin_slices']  * 16 if cfg['streaming'] else 0,
        'DDR_OFM_ROW_STRIDE' : cfg['W_OUT'] * cfg['cout_slices'] * 16 if cfg['streaming'] else 0,
        'DMA_MODE'       : cfg['dma_mode'],
        'IFB_RING_WORDS' : cfg['ifb_ring_words'],
        'OFB_ROW_WORDS'  : cfg['ofb_row_words'],
        'OFB_RING_WORDS' : cfg['ofb_ring_words'],
        'IFB_ISS_STEP'   : cfg['stride'] * cfg['cin_slices'],
        'IFB_KY_STEP'    : cfg['W_IN']   * cfg['cin_slices'],
        'TILE_PIX_STEP'  : cfg['TILE_W'] * cfg['stride'],
        'ARF_REUSE_EN'   : 1 if cfg['arf_reuse_en'] else 0,
    }
