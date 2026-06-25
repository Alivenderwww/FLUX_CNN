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
import sys

# 引入 params.py 作为参数 single source of truth
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_THIS_DIR))   # 项目根目录 (c:/_Project/FLUX_CNN)
from params import (
    NUM_PE as _PARAMS_NUM_PE,
    NUM_COL as _PARAMS_NUM_COL,
    IFB_DEPTH as _PARAMS_IFB_DEPTH,
    OFB_DEPTH as _PARAMS_OFB_DEPTH,
    CSR_ADDR_MAP as _PARAMS_CSR_ADDR_MAP,
    BOOT_REG_ADDRS as _PARAMS_BOOT_REG_ADDRS,
)


def _out_path(out_dir, fname):
    return os.path.join(out_dir, fname)


# ---------------------------------------------------------------------------
# 工具：SDP 软件模拟（硬件等价）
# ---------------------------------------------------------------------------
def _arith_rshift(val, shift):
    """Signed (arithmetic) right shift (Python 自带 // 已是 arithmetic, 但为清晰显式写)."""
    if shift <= 0:
        return val
    if val >= 0:
        return val >> shift
    return -((-val + (1 << shift) - 1) >> shift)


def sdp_sim(psum_i32, mult, shift, zp_out, clip_min, clip_max, round_en, relu_en,
            residual_en=False, shortcut_val=0, shortcut_mult=0, shortcut_shift=0):
    """
    Per-tensor symmetric int8 SDP 硬件流水软件模拟 (R.1 起含 bias add + residual).
      psum_i32 = parf 输出 (含 bias 已加, 因为本函数被 conv 累加 + bias 后调用)
      shortcut_val = 当前位置上一层 INT8 输出 (signed); residual_en=False 时不用
    """
    # 主路径: × mult, round, >> shift, + zp_out
    prod = psum_i32 * mult
    if round_en and shift > 0:
        prod += 1 << (shift - 1)
    q = _arith_rshift(prod, shift)
    q_zp = q + zp_out

    # R.1: shortcut rescale + residual add (residual_en=False 时归零)
    if residual_en:
        sc_prod    = shortcut_val * shortcut_mult           # signed INT24
        sc_shifted = _arith_rshift(sc_prod, shortcut_shift)
        summed     = q_zp + sc_shifted
    else:
        summed = q_zp

    if relu_en and summed < 0:
        summed = 0
    if summed < clip_min: summed = clip_min
    if summed > clip_max: summed = clip_max
    # 返回 *signed int8* (-128..127), 跟 RTL line_buffer 把 IFB byte 当 INT8 解释一致.
    # 写 expected_ofm.txt 仍 & 0xFF 得 byte. 关键: 链式 layer 间, 上层 OFM 当下层 IFM 时,
    # 必须用 signed 解释保持数值一致, 否则 clip_max>127 输出 byte 在 Python (无符号 200)
    # vs RTL (signed -56) 就会算出截然不同的 psum (chain Layer 2+ OFM mismatch root cause).
    v = summed & 0xFF
    return v - 256 if v >= 128 else v


# ---------------------------------------------------------------------------
# Space-to-Depth (S2D): stride>=2 时把 (kx%stride, ky%stride) 相位折到 cin 维
#   原 conv (K, Cin, stride>=2) → 等价 conv (K_new=ceil(K/stride), Cin_new=stride²·Cin, stride=1)
#   编译器侧纯数据重排 + 内核补零, HW 完全无感 (按普通 stride=1 conv 跑).
#   收益: 不需要输入复制 (vs Ky-fold), 多核 DDR 带宽 1× (vs Ky-fold 的 stride²× inflation).
#   代价: K 不被 stride 整除时, 不同相位 sub-kernel 形状不齐, 需补零到统一 K_new × K_new,
#         浪费比例 = (K_new² × stride² − K²) / (K_new² × stride²).
# ---------------------------------------------------------------------------
def compute_s2d_params(K, NUM_CIN, stride, HW_PE=16):
    """
    返回 (K_new, Cin_new, applicable, padding_waste).
    适用条件: stride >= 2 AND K >= stride.
    K_new       = ceil(K / stride)
    Cin_new     = stride² × NUM_CIN
    padding_waste = 内核 0-pad 比例 (K=stride 倍数时为 0).
    """
    if stride < 2 or K < stride:
        return K, NUM_CIN, False, 0.0
    K_new = (K + stride - 1) // stride
    Cin_new = stride * stride * NUM_CIN
    total_padded = K_new * K_new * stride * stride
    total_useful = K * K
    padding_waste = (total_padded - total_useful) / total_padded
    return K_new, Cin_new, True, padding_waste


def s2d_input(ifm_arr, H_IN, W_IN, NUM_CIN, pad_top, pad_left, stride):
    """
    对原始 ifm 做 pre-pad + S2D 重排.
    1. pre-pad 到 stride 整除尺寸 (含 pad_top/left 区, 必要时末尾再补 0)
    2. I_s2d[Y, X, p·Cin+c] = I_padded[Y·stride + a(p), X·stride + b(p), c]
       其中 p = a·stride + b, a,b ∈ [0, stride)
    返回 (ifm_s2d, H_s2d, W_s2d, Cin_new).
    新 conv 用 stride=1, pad_top=0, pad_left=0 (pad 已吸收进 pre-pad 输入).
    """
    H_pad = H_IN + 2 * pad_top
    W_pad = W_IN + 2 * pad_left
    H_pad_aln = ((H_pad + stride - 1) // stride) * stride
    W_pad_aln = ((W_pad + stride - 1) // stride) * stride
    Cin_new   = stride * stride * NUM_CIN
    H_s2d     = H_pad_aln // stride
    W_s2d     = W_pad_aln // stride

    # pre-padded full image (zeros for pad regions + stride 对齐补零)
    padded = [[[0] * NUM_CIN for _ in range(W_pad_aln)] for _ in range(H_pad_aln)]
    for y in range(H_IN):
        for x in range(W_IN):
            for c in range(NUM_CIN):
                padded[y + pad_top][x + pad_left][c] = ifm_arr[y][x][c]

    # S2D rearrange
    ifm_s2d = [[[0] * Cin_new for _ in range(W_s2d)] for _ in range(H_s2d)]
    for Y in range(H_s2d):
        for X in range(W_s2d):
            for a in range(stride):
                for b in range(stride):
                    p = a * stride + b
                    y_src = Y * stride + a
                    x_src = X * stride + b
                    for c in range(NUM_CIN):
                        ifm_s2d[Y][X][p * NUM_CIN + c] = padded[y_src][x_src][c]
    return ifm_s2d, H_s2d, W_s2d, Cin_new


def s2d_weights(w_arr, K, NUM_CIN, NUM_COUT, stride):
    """
    权重 S2D 重排 + 内核补零到 K_new × K_new.
    W'[ky', kx', co, p·Cin+c] = W[ky'·stride + a(p), kx'·stride + b(p), co, c]
        if ky'·stride + a < K AND kx'·stride + b < K, else 0 (kernel pad).
    返回 w_new [K_new][K_new][Cout][Cin_new].
    """
    K_new   = (K + stride - 1) // stride
    Cin_new = stride * stride * NUM_CIN
    w_new = [[[[0] * Cin_new for _ in range(NUM_COUT)]
              for _ in range(K_new)]
             for _ in range(K_new)]
    for ky_new in range(K_new):
        for kx_new in range(K_new):
            for a in range(stride):
                for b in range(stride):
                    p = a * stride + b
                    ky_orig = ky_new * stride + a
                    kx_orig = kx_new * stride + b
                    if ky_orig < K and kx_orig < K:
                        for co in range(NUM_COUT):
                            for c in range(NUM_CIN):
                                w_new[ky_new][kx_new][co][p * NUM_CIN + c] = \
                                    w_arr[ky_orig][kx_orig][co][c]
    return w_new


# ---------------------------------------------------------------------------
# Ky fold: Cin < PE_H 时把 Ky 维折到 cin_fake 维, 把 PE 阵列空闲的行填满
# ---------------------------------------------------------------------------
def compute_fold_params(K, NUM_CIN, HW_PE=16):
    """
    返回 (groups, ky_per_group, cin_fake, pad_ky).
    无 fold 时 groups=1, ky_per_group=K, cin_fake=NUM_CIN, pad_ky=0.

    条件: NUM_CIN < HW_PE 且 K > 1.
    groups = HW_PE // NUM_CIN (最大并行组数).
    ky_per_group = ceil(K / groups) (每组覆盖几个 Ky).
    pad_ky = groups * ky_per_group - K (末组 zero 填充数).
    """
    if NUM_CIN >= HW_PE or K <= 1:
        return 1, K, NUM_CIN, 0
    groups = HW_PE // NUM_CIN
    ky_per_group = (K + groups - 1) // groups
    pad_ky = groups * ky_per_group - K
    cin_fake = groups * NUM_CIN
    return groups, ky_per_group, cin_fake, pad_ky


def fold_weights(w_arr, K, NUM_CIN, NUM_COUT,
                 groups_y, ky_per_group):
    """
    Ky-fold 权重重排.
    w_arr: [K][K][NUM_COUT][NUM_CIN] 原卷积核 (KY=K 方形)
    返回: [ky_per_group][K][NUM_COUT][cin_fake]
        cin_fake = groups_y * NUM_CIN
        W'[ky_local, kx, co, g_y*NUM_CIN+c] = W[g_y*ky_per_group + ky_local, kx, co, c]
        (g_y*ky_per_group + ky_local 超出原 K 范围填 0)
    """
    cin_fake = groups_y * NUM_CIN
    w_virt = [[[[0 for _ in range(cin_fake)]
                for _ in range(NUM_COUT)]
               for _ in range(K)]
              for _ in range(ky_per_group)]
    for ky_local in range(ky_per_group):
        for kx in range(K):
            for co in range(NUM_COUT):
                for g_y in range(groups_y):
                    for c in range(NUM_CIN):
                        ky_orig = g_y * ky_per_group + ky_local
                        if ky_orig < K:
                            w_virt[ky_local][kx][co][g_y * NUM_CIN + c] = \
                                w_arr[ky_orig][kx][co][c]
    return w_virt


def fold_input(ifm_arr, H_IN, W_IN, NUM_CIN, pad_top,
               H_OUT, stride, groups, ky_per_group):
    """
    生成虚拟输入图像. 虚拟卷积 pad_top=0 (原 pad_top 吸收进 I_padded + 组 shift).

    虚拟 I[y, x, g*NUM_CIN + c] = I_padded[y + g*ky_per_group, x, c]
    其中 I_padded = 原 ifm 顶部 pad_top 行 0 + 底部 pad_top 行 0 (对称 pad).

    返回 (ifm_virt, H_IN_virt): [H_IN_virt][W_IN][cin_fake]
    H_IN_virt = (H_OUT - 1) * stride + ky_per_group
    """
    cin_fake = groups * NUM_CIN
    H_IN_virt = (H_OUT - 1) * stride + ky_per_group
    pad_bot = pad_top  # symmetric
    ifm_virt = [[[0 for _ in range(cin_fake)]
                 for _ in range(W_IN)]
                for _ in range(H_IN_virt)]
    for y_virt in range(H_IN_virt):
        for g in range(groups):
            # 源 y (在 padded 坐标系): y_virt + g*ky_per_group
            # 映射到原 ifm: y_orig = (y_virt + g*ky_per_group) - pad_top
            y_src_padded = y_virt + g * ky_per_group
            y_orig       = y_src_padded - pad_top
            if 0 <= y_orig < H_IN:
                for x in range(W_IN):
                    for c in range(NUM_CIN):
                        ifm_virt[y_virt][x][g * NUM_CIN + c] = ifm_arr[y_orig][x][c]
            # else: 超出原 ifm 范围 (上/下 pad 区), 保持 0
    return ifm_virt, H_IN_virt


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
# WB: weight (cs, cins, ky, kx) 顺序打包. R.1 起 bias 不再放 wb prefix,
#     由独立的 rdma_data.txt 装到 Shortcut Bank, 见 write_rdma_data().
# ---------------------------------------------------------------------------
def write_wb(out_dir, w_arr, K, NUM_CIN, NUM_COUT, HW_PE, HW_COL, KY=None):
    """
    w_arr: [KY][K][NUM_COUT][NUM_CIN] int8  (KY=None 时 KY=K, 方形核)
    weight (cs, cins, ky, kx) 顺序, 每 (cs, cins, ky, kx) 1 word (2048bit).
    """
    if KY is None:
        KY = K
    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
    wb_hex_chars = HW_COL * HW_PE * 2           # 2048 bit = 512 hex chars

    w_lines = []
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for cins in range(cin_slices):
            local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
            for ky in range(KY):
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
        f.writelines(d + '\n' for d in w_lines)
    return len(w_lines)


# ---------------------------------------------------------------------------
# RDMA data: bias section + (optional) shortcut section, 128-bit per line.
#   - bias_arr: [NUM_COUT] int32 或 None (None = 全 0)
#   - shortcut_arr: [H_OUT][W_OUT][NUM_COUT] int8 或 None (R.1 默认 None)
#
# bias 布局: 每 cs 4 个 128-bit 字, 每字打包 4 个 INT32 (低位先):
#   word0 = {b3, b2, b1, b0}, word1 = {b7, b6, b5, b4}, ...
# shortcut 布局: NHWC, 每 (yout, x, cs) 1 个 128-bit 字 = 16 × INT8 (低位先).
# ---------------------------------------------------------------------------
def write_rdma_data(out_dir, bias_arr, shortcut_arr,
                    NUM_COUT, HW_COL, H_OUT=0, W_OUT=0,
                    out_filename='rdma_data.txt',
                    w_slice_start=0, w_slice_end=None):
    """生成 rdma_data.txt. 返回 (n_lines, n_bias_lines).

    W slice (residual + 多核) 用: w_slice_start/end 限定 shortcut 写第 [start, end) 列.
    每核独立 rdma 文件, ofb_writer 的 SB 按 sub_W_OUT 寻址自动对齐.
    """
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL
    hex_chars   = HW_COL * 2                 # 128 bit = 32 hex chars
    if w_slice_end is None:
        w_slice_end = W_OUT

    lines = []

    # ---- bias section: cout_slices × 4 行 (128-bit) ----
    for cs in range(cout_slices):
        local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
        for sub in range(4):                 # 4 sub-words per cs (16 INT32 / 4)
            word = 0
            for k in range(4):               # 4 INT32 per sub-word
                lc = sub * 4 + k
                if lc < local_cout:
                    cout = cs * HW_COL + lc
                    bv = bias_arr[cout] if bias_arr is not None else 0
                    word |= (bv & 0xFFFFFFFF) << (k * 32)
            lines.append(f"{word:0{hex_chars}X}")

    n_bias_lines = len(lines)

    # ---- shortcut section: NHWC, 每 (yout, x, cs) 1 行 ----
    #   (yout, x ∈ [w_slice_start, w_slice_end), cs) — W slice 时只写本核段
    if shortcut_arr is not None:
        for yout in range(H_OUT):
            for x in range(w_slice_start, w_slice_end):
                for cs in range(cout_slices):
                    local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
                    word = 0
                    for lc in range(local_cout):
                        cout = cs * HW_COL + lc
                        v = shortcut_arr[yout][x][cout] & 0xFF
                        word |= v << (lc * 8)
                    lines.append(f"{word:0{hex_chars}X}")

    with open(_out_path(out_dir, out_filename), 'w') as f:
        f.writelines(d + '\n' for d in lines)
    return len(lines), n_bias_lines


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
    residual_en=False, shortcut_arr=None, shortcut_mult=0, shortcut_shift=0,
    KY=None, stride_h=None, stride_w=None,
):
    """
    模拟硬件一轮 Conv + SDP 量化流水. 返回 [H_OUT][W_OUT][NUM_COUT] 8-bit int.
    R.1: residual_en=True 时把 shortcut_arr[yout][px][cout] (INT8) 经 mult/shift
         rescale 后加到 SDP 量化结果上, 再 ReLU/clip.
    KY: ky 方向核高度, 默认 = K (方核). 异型 kernel KY≠K (跟 derive_layer_cfg 一致).
    stride_h/stride_w: 独立 H/W stride, 默认 = stride. AvgPool sh≠sw 时分开 (跟 derive_layer_cfg 一致).
    """
    if KY is None:
        KY = K
    if stride_h is None: stride_h = stride
    if stride_w is None: stride_w = stride
    H_OUT = (H_IN + 2 * pad_top - KY) // stride_h + 1    # ky 方向用 KY + H stride
    W_OUT = (W_IN + 2 * pad_left - K) // stride_w + 1    # kx 方向用 K + W stride (假设对称 pad)
    ofm = [[[0] * NUM_COUT for _ in range(W_OUT)] for _ in range(H_OUT)]
    for yout in range(H_OUT):
        for px in range(W_OUT):
            for cout in range(NUM_COUT):
                psum = bias_arr[cout] if bias_arr is not None else 0
                for ky in range(KY):
                    for kx in range(K):
                        iy = yout * stride_h + ky - pad_top
                        ix = px   * stride_w + kx - pad_left
                        if 0 <= iy < H_IN and 0 <= ix < W_IN:
                            for cin in range(NUM_CIN):
                                psum += ifm_arr[iy][ix][cin] * w_arr[ky][kx][cout][cin]
                        # pad: contributes 0
                # shortcut: 取值 (sign-extend 当 INT8) 或 0
                sc_val = 0
                if residual_en and shortcut_arr is not None:
                    raw = shortcut_arr[yout][px][cout] & 0xFF
                    sc_val = raw - 256 if raw >= 128 else raw
                ofm[yout][px][cout] = sdp_sim(
                    psum, sdp_mult, sdp_shift, sdp_zp_out,
                    sdp_clip_min, sdp_clip_max, sdp_round_en, sdp_relu_en,
                    residual_en=residual_en, shortcut_val=sc_val,
                    shortcut_mult=shortcut_mult, shortcut_shift=shortcut_shift)
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
TYPE_NOP, TYPE_CONV, TYPE_BARRIER, TYPE_CFG, TYPE_END = 0x0, 0x1, 0x2, 0x3, 0xF
FLAG_IS_FIRST     = 1 << 0
FLAG_IS_LAST      = 1 << 1
FLAG_STREAMING_EN = 1 << 2

# cfg_regs.sv address map: 直接来自 params.py (single source of truth)
CFG_ADDR_MAP   = _PARAMS_CSR_ADDR_MAP
BOOT_REG_ADDRS = _PARAMS_BOOT_REG_ADDRS


def _pack_desc(type_, flags, pad_t, pad_b, pad_l, pad_r,
               strip_y_start, n_yout, ifb_off, ifb_len, ofb_off, ofb_len):
    """CONV / NOP / BARRIER / END descriptor (256 bit) → (beat0, beat1) 每 128 bit。"""
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


def _pack_cfg_desc(reg_addr, reg_data):
    """CFG_WRITE descriptor (256 bit). type=0x3, addr in word0[15:4], data in word1[31:0]."""
    desc256 = (TYPE_CFG & 0xF) | ((reg_addr & 0xFFF) << 4) | ((reg_data & 0xFFFFFFFF) << 32)
    beat0 = desc256 & ((1 << 128) - 1)
    beat1 = (desc256 >> 128) & ((1 << 128) - 1)
    return beat0, beat1


def write_descriptors(
    out_dir, cfg_dict,
    H_IN, W_IN, H_OUT, W_OUT, cin_slices, cout_slices,
    pad_top, pad_bot, pad_left, pad_right,
    strip_rows=0, streaming=True,
):
    """
    写 desc_list.hex: 先 CFG_WRITE 段 (从 cfg_dict 推每条 layer cfg 寄存器写),
    再 CONV strips, 再 END。返回 (n_desc, n_strips, strip_rows_eff)。

    cfg_dict: hw_files.cfg_to_dict() 的输出, 含所有 cfg 字段 + META + 边界字段.
              本函数会跳过 _META_* / boot regs / 非 CSR 字段 (PAD_TOP / PAD_LEFT 等).
    """
    descs = []

    # ---- CFG_WRITE 段: 把 cfg_dict 里所有 layer cfg 寄存器一条条 emit ----
    for key, val in cfg_dict.items():
        if key.startswith('_META_'):
            continue
        if key not in CFG_ADDR_MAP:
            # boot regs (DESC_*, DMA_MODE, CTRL) 或非 CSR 字段 (PAD_TOP / PAD_LEFT)
            continue
        addr = CFG_ADDR_MAP[key]
        descs.append(_pack_cfg_desc(addr, val & 0xFFFFFFFF))

    # ---- CONV 段 ----
    strip_rows_eff = strip_rows if (strip_rows > 0 and strip_rows < H_OUT) else H_OUT
    n_strips = (H_OUT + strip_rows_eff - 1) // strip_rows_eff

    AXI_BYTES = 16
    ifb_bytes_per_row   = W_IN  * AXI_BYTES
    ifb_bytes_per_slice = H_IN  * ifb_bytes_per_row
    ifb_total_bytes     = ifb_bytes_per_slice * cin_slices
    ofb_bytes_per_row   = W_OUT * AXI_BYTES
    ofb_bytes_per_slice = H_OUT * ofb_bytes_per_row
    ofb_total_bytes     = ofb_bytes_per_slice * cout_slices

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

        flags = FLAG_STREAMING_EN
        if is_first:  flags |= FLAG_IS_FIRST
        if is_last:   flags |= FLAG_IS_LAST

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
# 多层 desc list 拼接 (M2.5 P0): 一个核跑多 layer 的 desc list.
#   layout: [L0 CFG_WRITEs + CONV strips] BARRIER [L1 ...] BARRIER ... [Last] END
#   sequencer 内部 S_BARRIER → S_FETCH 自动消费下一段 desc.
# ---------------------------------------------------------------------------
def build_layer_desc_segment(cfg_dict, H_IN, W_IN, H_OUT, W_OUT,
                              cin_slices, cout_slices,
                              pad_top, pad_bot, pad_left, pad_right,
                              strip_rows=0):
    """
    返回一个 layer 的 desc list (CFG_WRITE 段 + CONV strips), 不含 END.
    cfg_dict 来自 hw_files.cfg_to_dict(), 已含正确的 IDMA/ODMA/SKIP_IDMA/SDP cfg.
    """
    descs = []

    # CFG_WRITE 段
    for key, val in cfg_dict.items():
        if key.startswith('_META_'):
            continue
        if key not in CFG_ADDR_MAP:
            continue
        addr = CFG_ADDR_MAP[key]
        descs.append(_pack_cfg_desc(addr, val & 0xFFFFFFFF))

    # CONV 段
    strip_rows_eff = strip_rows if (strip_rows > 0 and strip_rows < H_OUT) else H_OUT
    n_strips = (H_OUT + strip_rows_eff - 1) // strip_rows_eff

    AXI_BYTES = 16
    ifb_bytes_per_row   = W_IN  * AXI_BYTES
    ifb_bytes_per_slice = H_IN  * ifb_bytes_per_row
    ifb_total_bytes     = ifb_bytes_per_slice * cin_slices
    ofb_bytes_per_row   = W_OUT * AXI_BYTES
    ofb_bytes_per_slice = H_OUT * ofb_bytes_per_row
    ofb_total_bytes     = ofb_bytes_per_slice * cout_slices

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

        flags = FLAG_STREAMING_EN
        if is_first: flags |= FLAG_IS_FIRST
        if is_last:  flags |= FLAG_IS_LAST

        descs.append(_pack_desc(
            TYPE_CONV, flags,
            local_pad_top, local_pad_bot, pad_left, pad_right,
            strip_y_start, n_yout,
            ifb_off, ifb_len, ofb_off, ofb_len))
    return descs


def write_multilayer_desc_list(out_path, layer_segments):
    """
    写一个核的多 layer desc list.
    layer_segments: list of desc lists (来自 build_layer_desc_segment)
    每两层之间插 BARRIER, 末尾加 END.
    """
    descs = []
    for i, seg in enumerate(layer_segments):
        descs.extend(seg)
        if i < len(layer_segments) - 1:
            # BARRIER 隔开层 (sequencer 等所有 done 后进 S_FETCH 拉下层)
            descs.append(_pack_desc(TYPE_BARRIER, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    # 末尾 END
    descs.append(_pack_desc(TYPE_END, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    with open(out_path, 'w') as f:
        for (beat0, beat1) in descs:
            f.write(f"{beat0:032X}\n")
            f.write(f"{beat1:032X}\n")
    return len(descs)


# ---------------------------------------------------------------------------
# sim_params.f
# ---------------------------------------------------------------------------
def write_sim_params(out_dir, H_OUT, W_OUT, cout_slices, ifb_words, wb_words,
                     desc_count, sram_depth, num_cin, num_cout):
    """SRAM_DEPTH / SHORTCUT_DEPTH 作编译期 -g 参数, 跟 derive_layer_cfg 内 strip
    计算用的 IFB_SRAM_WORDS 一致 (双向都来自 params.py IFB_DEPTH). 其它 meta 走
    config.txt 供 TB 运行期读.
    SHORTCUT_DEPTH 单独 sim 给 8192: 单核 tb_core_dma 整图模式 SHB 段需装整图
    shortcut (H_OUT*W_OUT*cs ≤ 8192); SMC tb_smc_chain W slice 模式 SHB 段只装本核段
    (≤ 2048 trim 后). 综合 / 真硬件用 svh 默认 SHORTCUT_DEPTH=2048."""
    with open(_out_path(out_dir, 'sim_params.f'), 'w') as f:
        f.write(f"-gSRAM_DEPTH={sram_depth}\n")
        f.write(f"-gSHORTCUT_DEPTH=8192\n")


# ---------------------------------------------------------------------------
# Derived cfg helpers (shared derive logic)
# ---------------------------------------------------------------------------
def derive_layer_cfg(H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride,
                     pad_top, pad_left, TILE_W=32,
                     HW_PE=_PARAMS_NUM_PE, HW_COL=_PARAMS_NUM_COL,
                     WRF_DEPTH=32, ARF_DEPTH=32,
                     IFB_SRAM_WORDS=_PARAMS_IFB_DEPTH,
                     OFB_SRAM_WORDS=_PARAMS_OFB_DEPTH,
                     streaming=None, KY=None,
                     pad_right=None, pad_bot=None,
                     stride_h=None, stride_w=None):
    """
    派生 conv layer 的所有硬件 cfg 字段。返回 dict。

    K        = 卷积核 Kx 维
    KY       = 卷积核 Ky 维, 默认 = K (方形核). Ky-fold 时 < K 缩短 Ky 迭代
    kk       = K * KY (虚拟 kk, 用于 wgt_buffer/parf 时间迭代)

    pad_right / pad_bot : 默认 None = 对称 pad (= pad_left / pad_top).
                          非对称 pad 用于 W slice / H slice 多核场景, 每核两侧 pad 不同.

    J-1 起，单一数据路径 = streaming。
    """
    if KY is None:
        KY = K
    if pad_bot   is None: pad_bot   = pad_top
    if pad_right is None: pad_right = pad_left
    # 独立 H/W stride (默认 = stride). AvgPool/AdaptiveAvgPool 的 sh≠sw (如 adapool sh4 sw7).
    # cfg_to_dict 会用 _STRIDE_H/_STRIDE_W 算 STRIDE/STRIDE_H/IFB_ROW_STEP/IFB_ISS_STEP/TILE_PIX_STEP.
    if stride_h is None: stride_h = stride
    if stride_w is None: stride_w = stride

    cin_slices  = (NUM_CIN  + HW_PE  - 1) // HW_PE
    cout_slices = (NUM_COUT + HW_COL - 1) // HW_COL

    H_OUT = (H_IN + pad_top + pad_bot - KY) // stride_h + 1
    W_OUT = (W_IN + pad_left + pad_right - K) // stride_w + 1
    if H_OUT <= 0 or W_OUT <= 0:
        raise ValueError(f"invalid output {H_OUT}x{W_OUT}")

    kk          = K * KY
    total_wrf   = kk * cin_slices

    # ARF reuse: cur_fill_len = cur_valid_w + K - 1 ≤ ARF_DEPTH → tile_w ≤ 33-K
    # ARF 复用是 x 方向滑窗, 只受 Kx (=K) 约束, 与 Ky 无关
    arf_reuse_en = (stride_w == 1 and K > 1)   # ARF 滑窗复用是 W 方向, 受 W stride 约束
    max_tile_w = (ARF_DEPTH - K + 1) if arf_reuse_en else ARF_DEPTH
    if max_tile_w < 1:
        max_tile_w = 1
    if TILE_W > max_tile_w:
        TILE_W = max_tile_w

    # K=1 时 parf_accum 的 fill / drain 节拍相等 (fill=kk*cins*cur_valid_w 退化成 cur_valid_w),
    # 短尾 tile 会让下一 tile 的 fill 停等上一 tile 的 drain 完成 (drain_stall_fill), 成为瓶颈.
    # 选 TILE_W 让 last_valid_w 尽量接近 TILE_W 可消除该停顿.
    # K>1 时 fill >> drain, drain 天然藏在 fill 后面, 优先留大 TILE_W 减少 tile 边界 overhead.
    if K == 1:
        if W_OUT <= max_tile_w:
            TILE_W = W_OUT
        else:
            # 优先 TILE_W 大 + 短尾不严重 (last_valid_w ≥ tw/2). 这样:
            #   * num_tiles 小, parf_accum fill/drain 切换 overhead 低
            #   * 短尾 tile 不会让 drain_stall_fill 占比高
            # reverse iter 让我们拿到第一个满足条件的最大 tw.
            # W_OUT 是质数也不会退化到 tw=1 (W_OUT=67 → tw=23 nt=3 lvw=21).
            best_tw = 1
            for tw in range(max_tile_w, 0, -1):
                nt  = (W_OUT + tw - 1) // tw
                lvw = W_OUT - (nt - 1) * tw
                if lvw * 2 >= tw:           # 短尾 ≥ 半个 tile
                    best_tw = tw
                    break
            TILE_W = best_tw

    num_tiles    = (W_OUT + TILE_W - 1) // TILE_W
    last_valid_w = W_OUT - (num_tiles - 1) * TILE_W

    rounds_per_cins = (kk + WRF_DEPTH - 1) // WRF_DEPTH
    # 均匀切 round (避免短 round 紧接长 round 时 wgt_buffer loader 在 small-output 下来不及
    # 预取长 round 低 slot → compute 读到 stale 权重). rounds==2 时 非末=ceil(kk/2)、末=floor(kk/2),
    # 与 wgt_buffer RTL 派生的 round_len_nonlast=(kk+1)>>1 一致 (两者和=kk). rounds≥3 (kk>64, 极罕见)
    # 保持原 32 分块 (RTL 兜底同). 详见 memory/wgt_buffer_uneven_round_stale.md.
    if rounds_per_cins == 2:
        round_len_last = kk // 2                                 # floor(kk/2); 非末 = ceil(kk/2)
    else:
        round_len_last = kk - (rounds_per_cins - 1) * WRF_DEPTH   # rounds 1: =kk; rounds≥3: 旧 32 分块

    IFB_ROW_STEP  = stride_h * W_IN * cin_slices         # H 方向行步进
    WB_COUT_STEP  = kk * cin_slices
    TILE_IN_STEP  = TILE_W * stride_w * cin_slices        # W 方向 tile 步进

    ifb_words = H_IN * W_IN * cin_slices
    wb_words  = kk * cout_slices * cin_slices              # 纯 weight, bias 走 rdma
    ofb_words = H_OUT * W_OUT * cout_slices
    # rdma_data: bias section (cout_slices × 4 个 128-bit 字), R.1 默认无 shortcut
    rdma_words = cout_slices * 4

    # Strip 计算（整图装得下 SRAM 就 strip=H_IN/H_OUT，退化为原 batch 行为；
    # 装不下就按 SRAM 容量切小 strip，ring-buffer 流式跑）。
    ifb_row_words = W_IN * cin_slices
    ifb_strip_rows_max = IFB_SRAM_WORDS // ifb_row_words
    # K=1 stride=1 时不需要 sliding window, ring 1 行就够 (mac 跟 IDMA 串行,
    # 性能降级但功能正确). K>1 需要至少 K+1 行 (K 行 mac 用 + 1 行 IDMA prefetch).
    ifb_strip_rows_min = (K + 1) if K > 1 else 1
    if ifb_strip_rows_max < ifb_strip_rows_min:
        raise ValueError(
            f"IFB SRAM 容量不足: W_IN*cin_slices={ifb_row_words}, "
            f"max rows={ifb_strip_rows_max} < min={ifb_strip_rows_min} (K={K})")
    if H_IN <= ifb_strip_rows_max:
        # 整图装得下：strip = 全图（ring 不 wrap，相当于原 batch 语义）
        ifb_strip = H_IN
    else:
        ifb_strip = min(ifb_strip_rows_min + 2, ifb_strip_rows_max)

    ofb_row_words_calc = W_OUT * cout_slices
    # 留至少 1 行 slack: 防止 ring_words == OFB_SRAM_WORDS 时 wptr=rptr 二义性死锁.
    # ring_words 必须 < OFB_DEPTH, 不能 == OFB_DEPTH (即使理论上整图装得下).
    ofb_strip_rows_max = (OFB_SRAM_WORDS - 1) // ofb_row_words_calc
    if ofb_strip_rows_max < 2:
        raise ValueError(
            f"OFB SRAM 容量不足: W_OUT*cout_slices={ofb_row_words_calc}, "
            f"max rows={ofb_strip_rows_max} < 2")
    # J-5 timing fix: ring_full 寄存器化引入 1-cycle latency, 多写 ≤1 cell.
    # 让 ring 物理 wrap 多 1 行, ring_full 阈值不变, 防止多写 wrap 覆盖 row 0.
    RING_FULL_SLACK = 1
    if H_OUT <= ofb_strip_rows_max:
        ofb_strip = H_OUT      # 整图装下, 无 ring wrap, 不需 slack
        ofb_ring_words = ofb_strip * W_OUT * cout_slices
    else:
        # strip mode: 留 1 行 slack 给 ring_full 寄存延迟
        ofb_strip = min(8, ofb_strip_rows_max - RING_FULL_SLACK)
        if ofb_strip < 2:
            raise ValueError(
                f"OFB SRAM 容量不足 (slack 后): max-{RING_FULL_SLACK}={ofb_strip_rows_max-RING_FULL_SLACK}")
        # 物理 ring 多 1 行 slack: ring_words = (strip + slack) * row_words
        ofb_ring_words = (ofb_strip + RING_FULL_SLACK) * W_OUT * cout_slices

    # sram_depth 跟 strip 计算用的 IFB_SRAM_WORDS 绑定 (来自 params.py IFB_DEPTH,
    # 调用方可 ifb_sram_words_override 覆盖). sim_params.f 用此值 -gSRAM_DEPTH 编译期
    # override RTL parameter, 跟 strip 行为一致, 杜绝 sim/RTL 双源参数偏差.
    sram_depth = IFB_SRAM_WORDS

    ifb_ring_words = ifb_strip * W_IN  * cin_slices
    ofb_row_words  = W_OUT * cout_slices

    return {
        # 尺寸 / slicing
        'H_IN': H_IN, 'W_IN': W_IN, 'K': K, 'KY': KY,
        'NUM_CIN': NUM_CIN, 'NUM_COUT': NUM_COUT,
        'H_OUT': H_OUT, 'W_OUT': W_OUT,
        'cin_slices': cin_slices, 'cout_slices': cout_slices,
        'kk': kk, 'total_wrf': total_wrf,
        'rounds_per_cins': rounds_per_cins, 'round_len_last': round_len_last,
        'num_tiles': num_tiles, 'last_valid_w': last_valid_w,
        'TILE_W': TILE_W, 'stride': stride,
        '_STRIDE_H': stride_h, '_STRIDE_W': stride_w,   # 独立 H/W stride (cfg_to_dict 用)
        'pad_top': pad_top, 'pad_bot': pad_bot,
        'pad_left': pad_left, 'pad_right': pad_right,
        'arf_reuse_en': arf_reuse_en,
        # 步长
        'IFB_ROW_STEP': IFB_ROW_STEP, 'WB_COUT_STEP': WB_COUT_STEP,
        'TILE_IN_STEP': TILE_IN_STEP,
        # 大小
        'ifb_words': ifb_words, 'wb_words': wb_words, 'ofb_words': ofb_words,
        'rdma_words': rdma_words,         # R.1: bias-only (cout_slices × 4)
        'sram_depth': sram_depth,
        # Ring / strip
        'ifb_strip': ifb_strip, 'ofb_strip': ofb_strip,
        'ifb_ring_words': ifb_ring_words, 'ofb_row_words': ofb_row_words,
        'ofb_ring_words': ofb_ring_words,
    }


def compute_w_slice_geom(W_full, K, stride, pad_left_full, my_core, n_split):
    """
    Mode C.2 W slice 几何: 给定整层 (W_full, K, stride, pad_left_full) 和切片
    (n_split, my_core), 算出本核的 (sub_W, pad_l, pad_r, w_in_start, w_out_start, my_w_out)
    满足 (sub_W + pad_l + pad_r - K)/stride + 1 == my_w_out.

    切片策略: w_out_full / n_split 等分 (最后一核拿余数), 反推 IFM 段含 halo,
    边界核两侧用整层 pad, 中间核两侧用 halo (= 邻居核重叠区, computed redundancy).
    """
    pad_right_full = pad_left_full
    w_out_full = (W_full + pad_left_full + pad_right_full - K) // stride + 1
    if w_out_full <= 0:
        raise ValueError(f"W slice: W_OUT={w_out_full} ≤ 0")

    # Round-A 切片公平 (2026-05-07): 余数分散给前 rem 核 (跟 mesh_cmd.compute_smc_w_segments 一致)
    base = w_out_full // n_split
    rem  = w_out_full %  n_split
    if my_core < rem:
        w_out_start = my_core * (base + 1)
        my_w_out    = base + 1
    else:
        w_out_start = rem * (base + 1) + (my_core - rem) * base
        my_w_out    = base
    w_out_end = w_out_start + my_w_out

    # IFM x 范围: 输出 ox ∈ [w_out_start, w_out_end) 时, 输入 x = stride*ox + kx - pad_left_full
    in_lo_unb = stride * w_out_start         - pad_left_full
    in_hi_unb = stride * (w_out_end - 1) + (K - 1) - pad_left_full

    pad_l = max(0, -in_lo_unb)
    pad_r = max(0, in_hi_unb - (W_full - 1))
    in_lo = max(0, in_lo_unb)
    in_hi = min(W_full - 1, in_hi_unb)
    sub_W = in_hi - in_lo + 1
    if sub_W <= 0:
        raise ValueError(f"W slice core {my_core}/{n_split}: sub_W={sub_W} ≤ 0")

    # Sanity check
    expect_my_w_out = (sub_W + pad_l + pad_r - K) // stride + 1
    if expect_my_w_out != my_w_out:
        raise ValueError(
            f"W slice geom mismatch core {my_core}/{n_split}: "
            f"sub_W={sub_W} pad_l={pad_l} pad_r={pad_r} K={K} stride={stride} "
            f"→ {expect_my_w_out}, expected {my_w_out}")

    return {
        'sub_W'         : sub_W,
        'pad_left'      : pad_l,
        'pad_right'     : pad_r,
        'w_in_start'    : in_lo,
        'w_out_start'   : w_out_start,
        'my_w_out'      : my_w_out,
        'w_full'        : W_full,
        'w_out_full'    : w_out_full,
    }


def compute_w_slice_chain_geom(layers, n_cores):
    """
    [Phase 6.6 shared-nothing 分区] 反向递推 chain W slice geom (累积 halo overlap).

    假设输入是 linear chain (layer i+1 input = layer i output, 无 cross-layer skip).
    从 last layer 起每核 W_out 段不含 halo, 倒推每层每核需要的 W_in 段 (= 上层 W_out
    段 + 本层 halo); 上层 W_out 段 = 下层 W_in 段 (即上层多算 halo overlap 给下层用).

    每核 layer 0 IFM 段就是反向感受野, 包含所有层累积的 halo. 每核完全独立运行
    chain, mem 之间不需要任何 halo 同步.

    返回 geoms[L][c] = {
        'w_in_lo'/'w_in_hi': 该核该层 IFM 段在整图坐标的范围
        'w_out_lo'/'w_out_hi': 该核该层 OFM 段在整图坐标的范围 (含给下层 halo overlap)
        'sub_W_in': 该核 IFM 段宽度 (= w_in_hi - w_in_lo + 1)
        'sub_W_out': 该核 OFM 段宽度
        'pad_l'/'pad_r': 该核左右 padding
    }
    """
    n_layers = len(layers)
    last_layer = layers[-1]
    n = n_cores

    # 初始化: last layer 每核 w_out 段 (整图坐标, 不含 halo)
    # Round-A 切片公平: 余数分散给前 rem 核 (跟 compute_w_slice_geom / compute_smc_w_segments 一致)
    base_o = last_layer.w_out // n
    rem_o  = last_layer.w_out %  n
    cur_lo = [0] * n
    cur_hi = [0] * n
    _cur = 0
    for c in range(n):
        _w = (base_o + 1) if c < rem_o else base_o
        cur_lo[c] = _cur
        cur_hi[c] = _cur + _w
        _cur += _w

    geoms = [[None] * n for _ in range(n_layers)]
    for L in range(n_layers - 1, -1, -1):
        layer = layers[L]
        for c in range(n):
            w_out_lo = cur_lo[c]
            w_out_hi = cur_hi[c]
            # 反推 W_in segment (含 halo)
            #   w_in_x = w_out_x * stride + kx - pad_left  (kx in [0, K-1])
            w_in_lo_unb = w_out_lo * layer.stride - layer.pad
            w_in_hi_unb = (w_out_hi - 1) * layer.stride + (layer.k - 1) - layer.pad
            pad_l = max(0, -w_in_lo_unb)
            pad_r = max(0, w_in_hi_unb - (layer.w_in - 1))
            w_in_lo = max(0, w_in_lo_unb)
            w_in_hi = min(layer.w_in - 1, w_in_hi_unb)
            sub_W_in = w_in_hi - w_in_lo + 1
            sub_W_out = w_out_hi - w_out_lo

            geoms[L][c] = {
                'w_in_lo'  : w_in_lo,   'w_in_hi'  : w_in_hi,
                'w_out_lo' : w_out_lo,  'w_out_hi' : w_out_hi,
                'sub_W_in' : sub_W_in,  'sub_W_out': sub_W_out,
                'pad_l'    : pad_l,     'pad_r'    : pad_r,
            }

            # 上层 W_out 段 = 当前层 W_in 段 (含本层 halo, 给上层做 overlap 输出参考)
            cur_lo[c] = w_in_lo
            cur_hi[c] = w_in_hi + 1

    return geoms


def derive_w_slice_cfg_chain(layer, geom_entry, NUM_CIN, NUM_COUT,
                              TILE_W=32, KY=None, **kwargs):
    """
    [Phase 6.6 shared-nothing] 用 compute_w_slice_chain_geom 反向递推的 geom 派生
    每核 layer cfg, 含 halo overlap, **紧凑本核 layout**.

    跟 derive_w_slice_cfg 的差别:
      - W_IN 用 geom_entry['sub_W_in'] (含本层 halo + 累积下层 halo overlap)
      - W_OUT 应等于 geom_entry['sub_W_out'] (含给下层的 halo overlap)
      - DDR row stride 用本核紧凑段宽 (不是整图 W_full), 让 mem 内每核段从 0 紧凑存
      - _W_SLICE_W_IN/OUT_START 都设 0 (mem 内本核段从 0 起)
    """
    cfg = derive_layer_cfg(
        H_IN=layer.h_in, W_IN=geom_entry['sub_W_in'], K=layer.k,
        NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT, stride=layer.stride,
        pad_top=layer.pad, pad_left=geom_entry['pad_l'],
        pad_bot=layer.pad, pad_right=geom_entry['pad_r'],
        TILE_W=min(TILE_W, geom_entry['sub_W_out']),
        KY=KY,
        **kwargs,
    )
    if cfg['W_OUT'] != geom_entry['sub_W_out']:
        raise ValueError(
            f"derive_w_slice_cfg_chain: W_OUT mismatch: "
            f"derive={cfg['W_OUT']}, geom={geom_entry['sub_W_out']}")

    # 紧凑 layout: 本核 mem 内每核段从 0 起, row stride = 本核段宽 × c_slices × 16
    cfg['_W_SLICE_W_IN_START']   = 0
    cfg['_W_SLICE_W_OUT_START']  = 0
    cfg['_W_SLICE_W_FULL']       = geom_entry['sub_W_in']     # 紧凑: 用本核段宽
    cfg['_W_SLICE_W_OUT_FULL']   = geom_entry['sub_W_out']
    cfg['_W_SLICE_GLOBAL_W_IN_LO']  = geom_entry['w_in_lo']   # 整图坐标 (TB preload 切片用)
    cfg['_W_SLICE_GLOBAL_W_OUT_LO'] = geom_entry['w_out_lo']
    return cfg


def derive_w_slice_cfg(H_IN, W_full, K, NUM_CIN, NUM_COUT, stride,
                        pad_top, pad_left_full,
                        my_core, n_split,
                        TILE_W=32, KY=None, **kwargs):
    """
    Mode C.2 W slice 的 per-core layer cfg 派生.
    返回 cfg dict, 兼容 cfg_to_dict, 额外携带 _W_SLICE_* 字段告诉
    cfg_to_dict 用整层 W 算 DDR_*_ROW_STRIDE.

    H 不切片 (每核跑全 H). W_OUT 按 n_split 等分, 输入含 halo (computed redundancy).
    Asymmetric pad: 边界核两侧用 pad_left_full, 中间核两侧用 0 (halo 已包含输入).

    返回 cfg dict + 新增 keys:
        '_W_SLICE_W_IN_START'   per-core IFM 起点 (W 列)
        '_W_SLICE_W_OUT_START'  per-core OFM 起点 (W 列)
        '_W_SLICE_W_FULL'       整层 W (用于 DDR_IFM_ROW_STRIDE)
        '_W_SLICE_W_OUT_FULL'   整层 W_OUT (用于 DDR_OFM_ROW_STRIDE)
    """
    geom = compute_w_slice_geom(W_full, K, stride, pad_left_full, my_core, n_split)

    cfg = derive_layer_cfg(
        H_IN=H_IN, W_IN=geom['sub_W'], K=K,
        NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT, stride=stride,
        pad_top=pad_top, pad_left=geom['pad_left'],
        pad_bot=pad_top, pad_right=geom['pad_right'],
        TILE_W=min(TILE_W, geom['my_w_out']),
        KY=KY,
        **kwargs,
    )

    # Sanity: derive_layer_cfg 算的 W_OUT 必须等于 my_w_out (asymmetric pad 已传入)
    if cfg['W_OUT'] != geom['my_w_out']:
        raise ValueError(
            f"derive_w_slice_cfg: W_OUT mismatch core {my_core}/{n_split}: "
            f"derive={cfg['W_OUT']}, geom={geom['my_w_out']}")

    cfg['_W_SLICE_W_IN_START']  = geom['w_in_start']
    cfg['_W_SLICE_W_OUT_START'] = geom['w_out_start']
    cfg['_W_SLICE_W_FULL']      = geom['w_full']
    cfg['_W_SLICE_W_OUT_FULL']  = geom['w_out_full']
    return cfg


def derive_cout_slice_cfg(H_IN, W_IN, K, NUM_CIN, NUM_COUT_full, stride,
                          pad_top, pad_left,
                          my_core, n_split,
                          TILE_W=32, KY=None, **kwargs):
    """
    Mode C.3 cout slice 的 per-core layer cfg 派生.

    H/W 不切, 每核 NUM_COUT 是整图 cout 的一段 (PE col 维 NUM_COL block 整除分配).
    适用场景: H_OUT × W_OUT 很小 + cout >= n_split × NUM_COL (e.g., FC W=H=1).
    上层 IFM 必须是 mode A 集中存放, 4 核共享拉同一份 IFM.

    返回 cfg dict + 新增 keys:
        '_COUT_SLICE_COUT_START' : per-core cout 段起点 (cout idx)
        '_COUT_SLICE_COUT_FULL'  : 整层 cout (用于 rdma 切片 / OFM stitch)
    """
    # 延迟 import (避免顶层 circular dep): mesh_cmd → hw_files
    from backend import sg_cmd_emit as _mc
    seg_starts, seg_widths, seg_cs = _mc.compute_cout_segments(NUM_COUT_full, n_split)
    my_cout       = seg_widths[my_core]
    my_cout_start = seg_starts[my_core]

    cfg = derive_layer_cfg(
        H_IN=H_IN, W_IN=W_IN, K=K,
        NUM_CIN=NUM_CIN, NUM_COUT=my_cout, stride=stride,
        pad_top=pad_top, pad_left=pad_left,
        TILE_W=TILE_W, KY=KY,
        **kwargs,
    )

    cfg['_COUT_SLICE_COUT_START'] = my_cout_start
    cfg['_COUT_SLICE_COUT_FULL']  = NUM_COUT_full
    cfg['_COUT_SLICE_MY_COUT']    = my_cout
    return cfg


def cfg_to_dict(cfg, shift_amt=0, sdp_mult=1, sdp_zp_out=0,
                sdp_clip_min=0, sdp_clip_max=255, sdp_round_en=0, sdp_relu_en=1,
                case_name="",
                ddr_ifb_base=None, ddr_wb_base=None,
                ddr_ofb_base=None, ddr_desc_base=None, ddr_rdma_base=None,
                skip_ifb_preload=0, skip_ofb_clear=0,
                residual_en=0, shortcut_mult=0, shortcut_shift=0,
                rdma_words_total=None,
                skip_idma=0,
                # Phase 7 SMC + NUMA: IDMA / ODMA SG cmd list cfg
                idma_cmd_list_ptr=None, idma_cmd_count=None, idma_cmds_per_row=None,
                odma_cmd_list_ptr=None, odma_cmd_count=None, odma_cmds_per_row=None):
    """
    把 derive_layer_cfg 的结果转成 config.txt 用的有序 dict。
    包含 SDP 量化参数（F-1a/F-1b 补齐）。

    多层场景（Phase G）用的 DDR base 参数：
      ddr_*_base : 该层 DDR 的 byte 基址。None = 保持 TB 默认 localparam。
      skip_ifb_preload : 非首层时=1，TB 跳过 $readmemh ifb.txt（数据已由上层 OFB 写入）。
      skip_ofb_clear   : 非终层时=1，TB 不清 OFB 区（它是下层 IFB）。
    """
    out = {
        # --- META (TB 读取，不写 cfg_regs) ---
        # TB 从 config.txt 解析这些字段做仿真控制（OFB 对比循环 / 打印等）
        '_META_CASE_NAME'   : case_name,
        '_META_IFB_WORDS'   : cfg['ifb_words'],
        '_META_WB_WORDS'    : cfg['wb_words'],
        '_META_OFB_WORDS'   : cfg['ofb_words'],
        '_META_RDMA_WORDS'  : (rdma_words_total if rdma_words_total is not None
                                else cfg.get('rdma_words', 0)),
        '_META_SRAM_DEPTH'  : cfg['sram_depth'],
        '_META_NUM_CIN'     : cfg['NUM_CIN'],        # fold 后 cin_fake
        '_META_NUM_COUT'    : cfg['NUM_COUT'],       # fold 后 cout_fake
        # 原始卷积维度 (pre-fold), TB 用来算真实 MAC 利用率:
        #   useful_mac = H_out × W_out × K_orig² × Cin_orig × Cout_orig
        #   mac_util   = useful_mac / (cycles × NUM_COL × NUM_PE)
        '_META_K_ORIG'       : cfg.get('K_orig',        cfg['K']),
        '_META_NUM_CIN_ORIG' : cfg.get('NUM_CIN_orig',  cfg['NUM_CIN']),
        '_META_NUM_COUT_ORIG': cfg.get('NUM_COUT_orig', cfg['NUM_COUT']),
        # --- cfg_regs 字段 ---
        'H_OUT'          : cfg['H_OUT'],
        'W_OUT'          : cfg['W_OUT'],
        'W_IN'           : cfg['W_IN'],
        'K'              : cfg['K'],
        'KY'             : cfg.get('KY', cfg['K']),   # fold 时 < K, 否则默认 = K
        'STRIDE'         : cfg.get('_STRIDE_W', cfg['stride']),   # Round J: W 维 stride (默认 = stride). W 压缩时 = 1
        'STRIDE_H'       : cfg.get('_STRIDE_H', cfg['stride']),   # Round I: H 维 stride (默认 = stride)
        'CIN_SLICES'     : cfg['cin_slices'],
        'COUT_SLICES'    : cfg['cout_slices'],
        'TILE_W'         : cfg['TILE_W'],
        'TOTAL_WRF'      : cfg['total_wrf'],
        'KK'             : cfg['kk'],
        'ROUNDS_PER_CINS': cfg['rounds_per_cins'],
        'ROUND_LEN_LAST' : cfg['round_len_last'],
        'IFB_BASE'       : 0,
        'WB_BASE'        : 0,                       # R.1: WB 不再有 bias prefix
        'OFB_BASE'       : 0,
        # Round I: IFB_ROW_STEP 跟 stride_h 关联 (H stride compress 时 IFB 内行 dense, step=W*cs 而非 stride*W*cs)
        # Round J: W 压缩时 IFB 内每行只有 sub_W_out 列, 用 _IFB_ROW_DENSE_W (默认 = W_IN)
        'IFB_ROW_STEP'   : cfg.get('_STRIDE_H', cfg['stride']) * cfg.get('_IFB_ROW_DENSE_W', cfg['W_IN']) * cfg['cin_slices'],
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
        'H_IN_TOTAL'     : cfg.get('_H_IN_IDMA', cfg['H_IN']),    # Round I: ds layer H 维 strided 拉时 = H_OUT (而非 H_IN)
        'IFB_STRIP_ROWS' : cfg['ifb_strip'],
        'OFB_STRIP_ROWS' : cfg['ofb_strip'],
        # W slice 下 DDR row stride 用整层 W (DDR 存全图, 每核只读自己一段)
        'DDR_IFM_ROW_STRIDE' : cfg.get('_W_SLICE_W_FULL',     cfg['W_IN'])  * cfg['cin_slices']  * 16,
        'DDR_OFM_ROW_STRIDE' : cfg.get('_W_SLICE_W_OUT_FULL', cfg['W_OUT']) * cfg['cout_slices'] * 16,
        'DMA_MODE'       : 3,   # 统一 streaming (J-1)
        'IFB_RING_WORDS' : cfg['ifb_ring_words'],
        'OFB_ROW_WORDS'  : cfg['ofb_row_words'],
        'OFB_RING_WORDS' : cfg['ofb_ring_words'],
        'IFB_ISS_STEP'   : cfg.get('_STRIDE_W', cfg['stride']) * cfg['cin_slices'],   # Round J: W 压缩时 IFB 内 W 维 dense
        'IFB_KY_STEP'    : cfg['W_IN']   * cfg['cin_slices'],
        'TILE_PIX_STEP'  : cfg['TILE_W'] * cfg.get('_STRIDE_W', cfg['stride']),
        'ARF_REUSE_EN'   : 1 if cfg['arf_reuse_en'] else 0,
        # --- R.1/R.2: residual / shortcut / bias / rdma ---
        'RESIDUAL_EN'    : 1 if residual_en else 0,
        'SHORTCUT_MULT'  : shortcut_mult,                   # signed INT16
        'SHORTCUT_SHIFT' : shortcut_shift,
        'BIAS_BASE'      : 0,                                # bias 在 Shortcut Bank word index 0
        # --- M2: 跨核 consumer 跳过本地 IDMA, 等远端核 push 进 IFB ---
        'SKIP_IDMA'      : 1 if skip_idma else 0,
        'RDMA_BYTE_LEN'  : (rdma_words_total if rdma_words_total is not None
                            else cfg.get('rdma_words', 0)) * 16,
        # --- DMA base/len (descriptor-driven, per-layer) ---
        # 从前 TB 在 load_config 之外 axi_lite_write 这些; 现在走 CFG_WRITE descriptor.
        'IDMA_SRC_BASE'  : (ddr_ifb_base  if ddr_ifb_base  is not None else 0),
        'IDMA_BYTE_LEN'  : cfg['ifb_words'] * 16,
        'WDMA_SRC_BASE'  : (ddr_wb_base   if ddr_wb_base   is not None else 0),
        'WDMA_BYTE_LEN'  : cfg['wb_words'] * 256,            # WB 一行 = 2048 bit = 256 byte
        'ODMA_DST_BASE'  : (ddr_ofb_base  if ddr_ofb_base  is not None else 0),
        'ODMA_BYTE_LEN'  : cfg['ofb_words'] * 16,
        'RDMA_SRC_BASE'  : (ddr_rdma_base if ddr_rdma_base is not None else 0),
    }
    # --- Phase 7 SMC + NUMA: SG cmd list cfg (None 时不写入, simple mode 下硬件不读) ---
    if idma_cmd_list_ptr is not None: out['IDMA_CMD_LIST_PTR']  = idma_cmd_list_ptr
    if idma_cmd_count    is not None: out['IDMA_CMD_COUNT']     = idma_cmd_count
    if idma_cmds_per_row is not None: out['IDMA_CMDS_PER_ROW']  = idma_cmds_per_row
    if odma_cmd_list_ptr is not None: out['ODMA_CMD_LIST_PTR']  = odma_cmd_list_ptr
    if odma_cmd_count    is not None: out['ODMA_CMD_COUNT']     = odma_cmd_count
    if odma_cmds_per_row is not None: out['ODMA_CMDS_PER_ROW']  = odma_cmds_per_row
    # --- META DDR base (Phase G 多层)：None 跳过该 key，TB fallback 到默认 ---
    if ddr_ifb_base  is not None: out['_META_DDR_IFB_BASE']  = ddr_ifb_base
    if ddr_wb_base   is not None: out['_META_DDR_WB_BASE']   = ddr_wb_base
    if ddr_ofb_base  is not None: out['_META_DDR_OFB_BASE']  = ddr_ofb_base
    if ddr_desc_base is not None: out['_META_DDR_DESC_BASE'] = ddr_desc_base
    if ddr_rdma_base is not None: out['_META_DDR_RDMA_BASE'] = ddr_rdma_base
    out['_META_SKIP_IFB_PRELOAD'] = 1 if skip_ifb_preload else 0
    out['_META_SKIP_OFB_CLEAR']   = 1 if skip_ofb_clear   else 0
    return out
