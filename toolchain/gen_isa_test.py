"""
gen_isa_test.py — 随机数据生成器（给 RTL 回归用）

职责：生成随机 ifm + weight + 软件模拟出 expected_ofm，再调 hw_files 写文件。
所有文件 I/O 与 cfg 派生逻辑共用 hw_files 模块（和 compile_layer.py 共享）。

用法：
  python gen_isa_test.py --k 3 --h_in 48 --w_in 48 --num_cin 8 --num_cout 8 --pad 1
  # 输出默认 ../sim/tb_core_dma/
  python gen_isa_test.py ... --out-dir /tmp/foo
"""
import argparse
import os
import random
import sys

from backend import hw_cfg_derive as hw_files


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "sim", "tb_core_dma"))


def parse_args():
    p = argparse.ArgumentParser(description='Generate random test data for tb_core_dma.')
    p.add_argument('--h_in',     type=int, default=48)
    p.add_argument('--w_in',     type=int, default=48)
    p.add_argument('--k',        type=int, default=3)
    p.add_argument('--stride',   type=int, default=1)
    p.add_argument('--num_cin',  type=int, default=8)
    p.add_argument('--num_cout', type=int, default=8)
    p.add_argument('--hw_pe',    type=int, default=16)
    p.add_argument('--hw_col',   type=int, default=16)
    p.add_argument('--tile_w',   type=int, default=32)
    p.add_argument('--seed',     type=int, default=42)
    p.add_argument('--shift',    type=int, default=0)
    # J-1: 硬件统一为 streaming 数据路径；flag 保留仅为兼容，默认 True
    p.add_argument('--streaming', action='store_true', default=True,
                   help='(deprecated, always streaming) emit streaming-ring cfg')
    p.add_argument('--pad',      type=int, default=0, help='symmetric pad')
    p.add_argument('--pad_top',  type=int, default=None)
    p.add_argument('--pad_left', type=int, default=None)
    p.add_argument('--strip_rows', type=int, default=0,
                   help='0 = single strip (default); >0 = multi-strip')
    p.add_argument('--ky-fold', action='store_true',
                   help='Cin<HW_PE 且 K>1 时启用 Ky 折叠: 把 Ky 维展开到 cin_fake, 提升 PE 空间利用率')
    p.add_argument('--s2d', action='store_true',
                   help='stride>=2 时启用 Space-to-Depth: 把 (kx%stride, ky%stride) 相位折到 cin, 等价 stride=1 conv. K 不被 stride 整除时内核补零.')
    p.add_argument('--residual', action='store_true',
                   help='R.2: 启用 SDP 残差 fusion. 生成随机 shortcut 张量, 经 mult=1/shift=0 (1:1 rescale) 加到 SDP 量化结果上.')
    p.add_argument('--depthwise', action='store_true',
                   help='深度卷积: 权重对角化 (仅 cin==cout 非零), 复用普通卷积数据流 (零 RTL/ISA 改动). 要求 num_cin==num_cout.')
    p.add_argument('--avgpool', action='store_true',
                   help='AvgPool 等价: depthwise + 对角权重全 1 (窗口求和), 配 --shift=log2(K²) 由 SDP 求均值.')
    p.add_argument('--ky', type=int, default=None,
                   help='ky 方向核高度 (异型 kernel kh≠kw). 默认 = --k (方核). hw 原生支持 (derive_layer_cfg KY).')
    p.add_argument('--out-dir',  default=DEFAULT_OUT_DIR,
                   help='output directory (default: ../sim/tb_core_dma/)')
    p.add_argument('--case-name', default="",
                   help='case name to embed in config.txt (F-2 multi-case)')
    return p.parse_args()


def generate_random(
    H_IN, W_IN, K, NUM_CIN, NUM_COUT, TILE_W, seed,
    shift_amt=0, stride=1, KY=None, stride_h=None, stride_w=None, HW_PE=16, HW_COL=16,
    streaming=False, pad_top=0, pad_left=0, strip_rows=0,
    out_dir=DEFAULT_OUT_DIR, case_name="", ky_fold=False, s2d=False,
    residual=False, depthwise=False, avgpool=False,
    # ---- chained-CASES overrides (run_regression in-process call) ----
    ifm_arr_in=None,        # [H_IN][W_IN][NUM_CIN] int (skip random gen if 不为 None)
    shortcut_arr_in=None,   # [H_OUT][W_OUT][NUM_COUT] int (residual 时, 优先于随机)
    # ---- D-1 PyTorch 真权重 override (跳 random weight gen) ----
    wb_arr_in=None,         # [K][K][NUM_COUT][NUM_CIN] int8 (None = random)
    bias_arr_in=None,       # [NUM_COUT] int32 (None = no bias)
    sdp_mult=None, sdp_zp_out=None,
    sdp_clip_min=None, sdp_clip_max=None,
    sdp_round_en=None, sdp_relu_en=None,
    shortcut_mult=None, shortcut_shift=None,
    ddr_ifb_base=None, ddr_wb_base=None,
    ddr_ofb_base=None, ddr_desc_base=None, ddr_rdma_base=None,
    skip_ifb_preload=False, skip_ofb_clear=False,
    skip_idma=False,
    # chain_data single-core 整图跑时, IFB/OFB SRAM 容量需求跟 SMC W slice 段不一样
    # (W=135 cs=4 整图 row_words=540 > IFB_DEPTH=1024 / 540=1 < K+1). chain_data
    # 用 IFB/OFB 大值 override 模拟"理想化"硬件容量, 仅生成 expected_ofm.txt golden.
    # SMC sim 跑 W slice cfg 仍用 params.py 的小值.
    ifb_sram_words_override=None,
    ofb_sram_words_override=None,
):
    """随机 ifm + weight + SDP 量化的测试生成器。
    单 case CLI 用法: 全 None override → 用 F-1a 默认 SDP 参数 (mult=1, clip[0,255], ReLU).
    run_regression 链式调用: 传入 ifm_arr_in / shortcut_arr_in / sdp_*  覆盖默认.
    """
    os.makedirs(out_dir, exist_ok=True)
    random.seed(seed)

    if stride < 1 or stride > 7:
        sys.exit(f"ERROR: stride={stride} out of range [1..7].")
    if KY is None:
        KY = K   # 方核默认; 异型 kernel (kh≠kw) 时 KY=ky 高度, K=kx 宽度 (hw 原生支持)
    # 独立 H/W stride (默认 = stride). AvgPool/AdaptiveAvgPool 的 sh≠sw (如 adapool sh4 sw7).
    if stride_h is None: stride_h = stride
    if stride_w is None: stride_w = stride
    if not (1 <= stride_h <= 7 and 1 <= stride_w <= 7):
        sys.exit(f"ERROR: stride_h={stride_h}/stride_w={stride_w} out of range [1..7].")

    # AvgPool = depthwise + 权重全 1 (算 K×K 窗口和), 再由 SDP mult/shift 实现 ÷N 求均值.
    #   N=K² (方核). N 为 2 的幂时 shift 精确 (K=2→shift2), 否则 mult/shift 近似.
    if avgpool:
        depthwise = True
    # depthwise: 权重对角化 (cin==cout 才非零), 复用普通卷积数据流. 要求 cin==cout.
    if depthwise and NUM_CIN != NUM_COUT:
        sys.exit(f"ERROR: depthwise 要求 NUM_CIN==NUM_COUT, got cin={NUM_CIN} cout={NUM_COUT}")

    # ---- 派生 cfg ----
    _cfg_kwargs = dict(
        H_IN=H_IN, W_IN=W_IN, K=K, KY=KY, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
        stride=stride, stride_h=stride_h, stride_w=stride_w,
        pad_top=pad_top, pad_left=pad_left,
        TILE_W=TILE_W, HW_PE=HW_PE, HW_COL=HW_COL,
        streaming=streaming)
    if ifb_sram_words_override is not None:
        _cfg_kwargs['IFB_SRAM_WORDS'] = ifb_sram_words_override
    if ofb_sram_words_override is not None:
        _cfg_kwargs['OFB_SRAM_WORDS'] = ofb_sram_words_override
    try:
        cfg = hw_files.derive_layer_cfg(**_cfg_kwargs)
    except ValueError as e:
        sys.exit(f"ERROR: {e}")

    H_OUT, W_OUT = cfg['H_OUT'], cfg['W_OUT']
    cin_slices, cout_slices = cfg['cin_slices'], cfg['cout_slices']

    PYTHON_SRAM_LIMIT = 524288
    for name, sz in [('IFB', cfg['ifb_words']), ('WB', cfg['wb_words']), ('OFB', cfg['ofb_words'])]:
        if sz > PYTHON_SRAM_LIMIT:
            sys.exit(f"ERROR: {name} overflow! size={sz} > {PYTHON_SRAM_LIMIT}")

    print(f"Config : H_IN={H_IN}, W_IN={W_IN}, K={K}x{K}, stride={stride}, "
          f"Cin={NUM_CIN} ({cin_slices} slice(s)), Cout={NUM_COUT} ({cout_slices} slice(s)), "
          f"TILE_W={cfg['TILE_W']}")
    print(f"Output : H_OUT={H_OUT}, W_OUT={W_OUT}, num_tiles={cfg['num_tiles']}, "
          f"last_valid_w={cfg['last_valid_w']}")
    print(f"Weight : total_wrf={cfg['total_wrf']}, "
          f"rounds/cins={cfg['rounds_per_cins']}, round_len_last={cfg['round_len_last']}")
    print(f"SRAM   : IFB={cfg['ifb_words']}, WB={cfg['wb_words']}, OFB={cfg['ofb_words']} -> "
          f"SRAM_DEPTH={cfg['sram_depth']}")

    # ---- ifm_arr: 链式 case 用 ifm_arr_in (上一层 expected_ofm), 否则随机 ----
    if ifm_arr_in is not None:
        # 维度断言 (帮调试)
        assert len(ifm_arr_in) == H_IN, f"ifm_arr_in H={len(ifm_arr_in)} vs H_IN={H_IN}"
        assert len(ifm_arr_in[0]) == W_IN, f"ifm_arr_in W vs W_IN={W_IN}"
        assert len(ifm_arr_in[0][0]) == NUM_CIN, f"ifm_arr_in C vs NUM_CIN={NUM_CIN}"
        ifm_arr = ifm_arr_in
    else:
        ifm_arr = [[[0] * NUM_CIN for _ in range(W_IN)] for _ in range(H_IN)]
        for cins in range(cin_slices):
            local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
            for y in range(H_IN):
                for x in range(W_IN):
                    for cin_local in range(local_cin):
                        cin = cins * HW_PE + cin_local
                        ifm_arr[y][x][cin] = random.randint(0, 7)

    # ---- w_arr: D-1 真权重 override, 否则随机 ----
    if wb_arr_in is not None:
        # 维度断言
        assert len(wb_arr_in) == K, f"wb_arr_in K={len(wb_arr_in)} vs K={K}"
        assert len(wb_arr_in[0]) == K, f"wb_arr_in Kx vs K={K}"
        assert len(wb_arr_in[0][0]) == NUM_COUT, \
            f"wb_arr_in Cout={len(wb_arr_in[0][0])} vs NUM_COUT={NUM_COUT}"
        assert len(wb_arr_in[0][0][0]) == NUM_CIN, \
            f"wb_arr_in Cin={len(wb_arr_in[0][0][0])} vs NUM_CIN={NUM_CIN}"
        w_arr = wb_arr_in
    else:
        # 生成 random w_arr: [K][K][NUM_COUT][NUM_CIN] int (-3..3, small)
        w_arr = [[[[0] * NUM_CIN for _ in range(NUM_COUT)]
                  for _ in range(K)] for _ in range(KY)]
        for cs in range(cout_slices):
            local_cout = min(HW_COL, NUM_COUT - cs * HW_COL)
            for cins in range(cin_slices):
                local_cin = min(HW_PE, NUM_CIN - cins * HW_PE)
                for ky in range(KY):
                    for kx in range(K):
                        for lc in range(local_cout):
                            cout = cs * HW_COL + lc
                            for cin_local in range(local_cin):
                                cin = cins * HW_PE + cin_local
                                # depthwise: 只在对角 (cin==cout) 填真权重, 其余保持 0.
                                # 普通卷积数据流的 mac_col 加法树自动把 off-diag 的 0 加掉,
                                # col c 输出 = act[c]×w[c] = 单通道 depthwise 乘积.
                                if depthwise and cin != cout:
                                    continue
                                # avgpool: 对角权重全 1 (窗口求和); 否则普通/depthwise random 权重.
                                w_arr[ky][kx][cout][cin] = 1 if avgpool else random.randint(-3, 3)

    # ---- SDP 参数: 优先 override, 否则 F-1a 默认 (mult=1, clip[0,255], ReLU on, no round)
    if sdp_mult     is None: sdp_mult     = 1
    if sdp_zp_out   is None: sdp_zp_out   = 0
    if sdp_clip_min is None: sdp_clip_min = 0
    if sdp_clip_max is None: sdp_clip_max = 255
    if sdp_round_en is None: sdp_round_en = 0
    if sdp_relu_en  is None: sdp_relu_en  = 1

    # ---- shortcut tensor + 参数:
    #   shortcut_arr_in 优先 (run_regression 链式: 上一层 ofm 当 shortcut)
    #   residual && 无 shortcut_arr_in: 随机 [-16, 16] (单 case --residual CLI)
    shortcut_arr = None
    if shortcut_arr_in is not None:
        # 维度断言
        assert len(shortcut_arr_in) == H_OUT, \
            f"shortcut_arr_in H={len(shortcut_arr_in)} vs H_OUT={H_OUT}"
        assert len(shortcut_arr_in[0]) == W_OUT
        assert len(shortcut_arr_in[0][0]) == NUM_COUT
        shortcut_arr = shortcut_arr_in
        residual = True
        if shortcut_mult  is None: shortcut_mult  = 1
        if shortcut_shift is None: shortcut_shift = 0
        print(f"Residual: ON (chained), shortcut [{H_OUT}×{W_OUT}×{NUM_COUT}], "
              f"mult={shortcut_mult} shift={shortcut_shift}")
    elif residual:
        if shortcut_mult  is None: shortcut_mult  = 1
        if shortcut_shift is None: shortcut_shift = 0
        shortcut_arr = [[[0] * NUM_COUT for _ in range(W_OUT)] for _ in range(H_OUT)]
        for yout in range(H_OUT):
            for px in range(W_OUT):
                for cout in range(NUM_COUT):
                    v = random.randint(-16, 16)
                    shortcut_arr[yout][px][cout] = v & 0xFF
        print(f"Residual: ON (random), shortcut [{H_OUT}×{W_OUT}×{NUM_COUT}] INT8 [-16,16]")
    else:
        if shortcut_mult  is None: shortcut_mult  = 0
        if shortcut_shift is None: shortcut_shift = 0

    # ---- 模拟硬件算 expected_ofm（bias=None 默认; D-1 PyTorch 真权重传 bias_arr_in） ----
    #   注意 expected_ofm 用 *原始* conv 计算 (pre-S2D, pre-fold), bit-exact 对照.
    ofm_arr, _H, _W = hw_files.compute_expected_ofm(
        H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride, pad_top, pad_left,
        ifm_arr, w_arr, bias_arr=bias_arr_in,
        sdp_mult=sdp_mult, sdp_shift=shift_amt, sdp_zp_out=sdp_zp_out,
        sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
        sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en,
        residual_en=residual, shortcut_arr=shortcut_arr,
        shortcut_mult=shortcut_mult, shortcut_shift=shortcut_shift, KY=KY,
        stride_h=stride_h, stride_w=stride_w)
    assert _H == H_OUT and _W == W_OUT

    # ---- S2D 变换 (软件层): stride>=2 时把 stride 维折到 cin
    #   等价为 stride=1, K_new=ceil(K/stride), Cin_new=stride²·Cin 的 conv.
    #   下游 Ky/Kx fold + HW 完全无感, 走标准 stride=1 路径.
    #   保留原始 (K, NUM_CIN, stride) 用于 META 算 MAC util.
    K_pre_s2d, NUM_CIN_pre_s2d, stride_pre_s2d = K, NUM_CIN, stride
    H_IN_pre_s2d, W_IN_pre_s2d, pad_top_pre, pad_left_pre = H_IN, W_IN, pad_top, pad_left
    s2d_active = False
    if s2d:
        K_new, Cin_new, applicable, pad_waste = \
            hw_files.compute_s2d_params(K, NUM_CIN, stride, HW_PE)
        if applicable:
            print(f"S2D    : K={K}→{K_new}, Cin={NUM_CIN}→{Cin_new}, stride={stride}→1, "
                  f"kernel_pad_waste={pad_waste*100:.1f}%")
            ifm_arr, H_IN, W_IN, NUM_CIN = hw_files.s2d_input(
                ifm_arr, H_IN, W_IN, NUM_CIN, pad_top, pad_left, stride)
            w_arr = hw_files.s2d_weights(w_arr, K, NUM_CIN_pre_s2d, NUM_COUT, stride)
            K = K_new
            KY = K_new   # s2d 产出方核 K_new×K_new; KY 必须跟 K 一起缩, 否则 write_wb 按旧 KY 越界
            stride = 1
            pad_top = 0    # pad 已 absorb 到 pre-pad 输入
            pad_left = 0
            s2d_active = True

            # 重派生 cfg (新维度) — 同样支持 IFB/OFB SRAM override
            _s2d_kwargs = dict(
                H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
                stride=stride, pad_top=pad_top, pad_left=pad_left,
                TILE_W=TILE_W, HW_PE=HW_PE, HW_COL=HW_COL,
                streaming=streaming)
            if ifb_sram_words_override is not None:
                _s2d_kwargs['IFB_SRAM_WORDS'] = ifb_sram_words_override
            if ofb_sram_words_override is not None:
                _s2d_kwargs['OFB_SRAM_WORDS'] = ofb_sram_words_override
            try:
                cfg = hw_files.derive_layer_cfg(**_s2d_kwargs)
            except ValueError as e:
                sys.exit(f"ERROR (s2d): {e}")
            # H_OUT/W_OUT 不变 (S2D 是等价变换)
            assert cfg['H_OUT'] == H_OUT and cfg['W_OUT'] == W_OUT, \
                f"S2D cfg dims mismatch: {cfg['H_OUT']}x{cfg['W_OUT']} vs {H_OUT}x{W_OUT}"
            cin_slices = cfg['cin_slices']
            cout_slices = cfg['cout_slices']

    # ---- Ky-fold (软件层): expand cin_fake 填满 PE 行
    #   Ky-fold: 虚拟 Cin=cin_fake, Ky→ky_per_group (输入 y-shift 复制到 cin 不同行)
    #   expected_ofm 用原始 conv 计算, bit-exact 对照.
    groups_y, ky_per_group, cin_fake, _ = (1, K, NUM_CIN, 0)
    if ky_fold:
        groups_y, ky_per_group, cin_fake, _ = \
            hw_files.compute_fold_params(K, NUM_CIN, HW_PE)

    fold_active = (groups_y > 1)

    if fold_active:
        print(f"Fold   : Ky={K}→{ky_per_group}×{groups_y} (cin {NUM_CIN}→{cin_fake})")

        ifm_virt, H_IN_virt = hw_files.fold_input(
            ifm_arr, H_IN, W_IN, NUM_CIN, pad_top,
            H_OUT, stride, groups_y, ky_per_group)
        pad_top_hw = 0

        w_virt = hw_files.fold_weights(
            w_arr, K, NUM_CIN, NUM_COUT, groups_y, ky_per_group)

        # 重新派生 cfg: 虚拟维度 (Ky=ky_per_group, Cin=cin_fake) — 同样支持 IFB/OFB SRAM override
        _fold_kwargs = dict(
            H_IN=H_IN_virt, W_IN=W_IN, K=K,
            NUM_CIN=cin_fake, NUM_COUT=NUM_COUT,
            stride=stride, pad_top=pad_top_hw, pad_left=pad_left,
            TILE_W=TILE_W, HW_PE=HW_PE, HW_COL=HW_COL,
            streaming=streaming, KY=ky_per_group)
        if ifb_sram_words_override is not None:
            _fold_kwargs['IFB_SRAM_WORDS'] = ifb_sram_words_override
        if ofb_sram_words_override is not None:
            _fold_kwargs['OFB_SRAM_WORDS'] = ofb_sram_words_override
        try:
            cfg = hw_files.derive_layer_cfg(**_fold_kwargs)
        except ValueError as e:
            sys.exit(f"ERROR (fold): {e}")
        assert cfg['W_OUT'] == W_OUT, \
            f"fold cfg W 不一致: got {cfg['W_OUT']} vs {W_OUT}"
        assert cfg['H_OUT'] == H_OUT, \
            f"fold cfg H 不一致: got {cfg['H_OUT']} vs {H_OUT}"

        hw_files.write_ifb(out_dir, ifm_virt, H_IN_virt, W_IN, cin_fake, HW_PE)
        hw_files.write_wb(out_dir, w_virt,
                          K=K, NUM_CIN=cin_fake, NUM_COUT=NUM_COUT,
                          HW_PE=HW_PE, HW_COL=HW_COL, KY=ky_per_group)
        H_IN_hw = H_IN_virt
    else:
        # no fold: 原始路径
        hw_files.write_ifb(out_dir, ifm_arr, H_IN, W_IN, NUM_CIN, HW_PE)
        hw_files.write_wb(out_dir, w_arr,
                          K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
                          HW_PE=HW_PE, HW_COL=HW_COL, KY=KY)
        H_IN_hw = H_IN
        pad_top_hw = pad_top

    # R.1/R.2: rdma_data.txt = bias 段 + 可选 shortcut 段
    #   D-1 fix: 传 bias_arr_in 给 emit, 跟 compute_expected_ofm 同源 (RTL bias_rf 通路才有数据)
    #   bias_arr_in = None → 全 0 bias (合成 demo 一直这样)
    n_rdma_lines, _ = hw_files.write_rdma_data(
        out_dir, bias_arr=bias_arr_in, shortcut_arr=shortcut_arr,
        NUM_COUT=NUM_COUT, HW_COL=HW_COL,
        H_OUT=H_OUT, W_OUT=W_OUT)

    # 原始卷积维度 (给 TB 算 MAC util 用): 取 *S2D 之前* 的原始值, 与 fold/S2D 无关
    cfg['K_orig']        = K_pre_s2d
    cfg['NUM_CIN_orig']  = NUM_CIN_pre_s2d
    cfg['NUM_COUT_orig'] = NUM_COUT

    # expected_ofm 始终以原始 conv 输出为基准 (bit-exact 对照)
    hw_files.write_expected_ofm(out_dir, ofm_arr, H_OUT, W_OUT, NUM_COUT, HW_COL)

    cfg_dict = hw_files.cfg_to_dict(cfg, shift_amt=shift_amt,
                                     sdp_mult=sdp_mult, sdp_zp_out=sdp_zp_out,
                                     sdp_clip_min=sdp_clip_min, sdp_clip_max=sdp_clip_max,
                                     sdp_round_en=sdp_round_en, sdp_relu_en=sdp_relu_en,
                                     case_name=case_name,
                                     residual_en=1 if residual else 0,
                                     shortcut_mult=shortcut_mult,
                                     shortcut_shift=shortcut_shift,
                                     rdma_words_total=n_rdma_lines,
                                     ddr_ifb_base =ddr_ifb_base,
                                     ddr_wb_base  =ddr_wb_base,
                                     ddr_ofb_base =ddr_ofb_base,
                                     ddr_desc_base=ddr_desc_base,
                                     ddr_rdma_base=ddr_rdma_base,
                                     skip_ifb_preload=1 if skip_ifb_preload else 0,
                                     skip_ofb_clear  =1 if skip_ofb_clear   else 0,
                                     skip_idma       =1 if skip_idma        else 0)
    hw_files.write_config(out_dir, cfg_dict)

    # fold 后 cin_slices/cout_slices 取 fold-aware 的值 (cfg 已更新)
    n_desc, n_strips, strip_rows_eff = hw_files.write_descriptors(
        out_dir, cfg_dict, H_IN_hw, W_IN, H_OUT, W_OUT,
        cfg['cin_slices'], cfg['cout_slices'],
        pad_top=pad_top_hw, pad_bot=pad_top_hw, pad_left=pad_left, pad_right=pad_left,
        strip_rows=strip_rows, streaming=streaming)
    print(f"Desc   : n_strips={n_strips} (+1 END), total={n_desc} descriptors, "
          f"strip_rows={strip_rows_eff}")

    hw_files.write_sim_params(
        out_dir, H_OUT=H_OUT, W_OUT=W_OUT, cout_slices=cout_slices,
        ifb_words=cfg['ifb_words'], wb_words=cfg['wb_words'],
        desc_count=n_desc, sram_depth=cfg['sram_depth'],
        num_cin=NUM_CIN, num_cout=NUM_COUT)

    print(f"Files  : ifb.txt  wb.txt  expected_ofm.txt  config.txt  sim_params.f  desc_list.hex")
    # 链式调用需要 ofm_arr (作下层 ifm) + 输出维度
    return {
        'ofm_arr'   : ofm_arr,
        'H_OUT'     : H_OUT,
        'W_OUT'     : W_OUT,
        'ifb_words' : cfg['ifb_words'],
        'wb_words'  : cfg['wb_words'],
        'ofb_words' : cfg['ofb_words'],
        'rdma_words': n_rdma_lines,
    }


if __name__ == '__main__':
    args = parse_args()
    pad_t = args.pad_top  if args.pad_top  is not None else args.pad
    pad_l = args.pad_left if args.pad_left is not None else args.pad
    generate_random(
        H_IN=args.h_in, W_IN=args.w_in, K=args.k,
        NUM_CIN=args.num_cin, NUM_COUT=args.num_cout,
        TILE_W=args.tile_w, seed=args.seed,
        shift_amt=args.shift, stride=args.stride,
        HW_PE=args.hw_pe, HW_COL=args.hw_col,
        streaming=args.streaming,
        pad_top=pad_t, pad_left=pad_l,
        strip_rows=args.strip_rows,
        out_dir=args.out_dir, case_name=args.case_name,
        ky_fold=args.ky_fold, s2d=args.s2d,
        residual=args.residual, depthwise=args.depthwise, avgpool=args.avgpool, KY=args.ky)
