#!/usr/bin/env python3
"""Stage 3a Phase 2: 稍大 case (K=3 H=8 W=8 Cin=16 Cout=16) + 多轮启停.

利用 toolchain.hw_files 自动派生 cfg + desc list, 避免 hand-craft 50+ cfg 字段.
Phase 2 仍 MAC bypass, OFM 期望 0x7F 饱和 (CLIP_MAX), 不做 bit-exact (那是 Stage 4).

验证目标:
  1. 大维度 cfg (cin_slices/cout_slices 派生正确)
  2. sequencer 在多周期循环下 (kk=9, total_wrf=9, ifb_strip=8) 不 stuck
  3. IFB ring (ifb_ring_words=64) + OFB ring 翻页正确
  4. WDMA 9 word + RDMA 4 word + IDMA/ODMA 各 1 SG cmd 不冲突
  5. 多轮启停 (N=200 默认) 仍稳定

BRAM layout (BRAM_BASE=0xA4100000, 64KB):
  0x00000 desc list  (~40 CFG + CONV + END, ~1.3KB)
  0x01800 IDMA SG cmd (32 byte, 1 cmd)
  0x01820 ODMA SG cmd (32 byte, 1 cmd)
  0x02000 IFM data    (1024 byte = 8*8*16)
  0x04000 Weight data (2304 byte = 9*256)
  0x07000 RDMA bias   (64 byte = 4*16)
  0x08000 OFM area    (1024 byte)
"""
import sys, struct, time, argparse, statistics
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import hw_files

# ====== BRAM layout (v6 fix: ISG/OSG 各 4KB = 128 cmd 容量) ======
# 原 1KB = 32 cmd, H_OUT > 32 时 cmd 越界写到 IFM, dispatcher 拉错 cmd_data.
BRAM_BASE = 0xA4100000
DESC_OFF  = 0x00000
ISG_OFF   = 0x01000   # 4 KB 区
OSG_OFF   = 0x02000   # 4 KB 区
IFM_OFF   = 0x03000
WB_OFF    = 0x10000
RDMA_OFF  = 0x20000
OFM_OFF   = 0x30000

BRAM_IFM  = BRAM_BASE + IFM_OFF
BRAM_WB   = BRAM_BASE + WB_OFF
BRAM_RDMA = BRAM_BASE + RDMA_OFF
BRAM_OFM  = BRAM_BASE + OFM_OFF
BRAM_ISG  = BRAM_BASE + ISG_OFF
BRAM_OSG  = BRAM_BASE + OSG_OFF


def beat_pair_to_bytes(beat0, beat1):
    """desc (beat0, beat1) 128-bit tuple → 32 byte little-endian."""
    return beat0.to_bytes(16, 'little') + beat1.to_bytes(16, 'little')


def make_sg_idma(src_addr, btt, sram_off=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', src_addr, w1, sram_off & 0x1FFF, 0) + b'\x00' * 16


def make_sg_odma(dst_addr, btt, ofb_w_start=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', dst_addr, w1, ofb_w_start & 0xFFFF, 0) + b'\x00' * 16


def build_case(H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride, pad):
    """用 hw_files.derive_layer_cfg + build_layer_desc_segment 派生整 case.
    返回 (desc_list_bytes, n_desc, cfg, ifm_data, wb_data, rdma_data, isg_cmd, osg_cmd)."""

    # 1) 派生 cfg
    cfg = hw_files.derive_layer_cfg(
        H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
        stride=stride, pad_top=pad, pad_left=pad, TILE_W=32)

    # 2) cfg_to_dict + 填充 BRAM-specific 地址 + SG cmd cfg
    # ODMA dispatcher 每行 OFM 收 row_done_pulse, 每 pulse 跑 cmds_per_row 条 cmd.
    # 单 BRAM 单 seg → cmds_per_row=1, cmd_count=H_OUT 行 cmd. IDMA 对称: H_IN 行.
    H_IN, H_OUT = cfg['H_IN'], cfg['H_OUT']
    cfg_dict = hw_files.cfg_to_dict(
        cfg,
        shift_amt=0,
        sdp_mult=1, sdp_zp_out=0,
        sdp_clip_min=0xFFFFFF80, sdp_clip_max=0x7F,  # signed int8 全范围
        sdp_round_en=0, sdp_relu_en=0,
        ddr_ifb_base=BRAM_IFM,
        ddr_wb_base=BRAM_WB,
        ddr_ofb_base=BRAM_OFM,
        ddr_rdma_base=BRAM_RDMA,
        rdma_words_total=cfg['rdma_words'],
        idma_cmd_list_ptr=BRAM_ISG, idma_cmd_count=H_IN,  idma_cmds_per_row=1,
        odma_cmd_list_ptr=BRAM_OSG, odma_cmd_count=H_OUT, odma_cmds_per_row=1,
        skip_idma=0)

    # 3) desc list (CFG_WRITE × N + 1 CONV strip)
    seg = hw_files.build_layer_desc_segment(
        cfg_dict,
        H_IN=cfg['H_IN'], W_IN=cfg['W_IN'],
        H_OUT=cfg['H_OUT'], W_OUT=cfg['W_OUT'],
        cin_slices=cfg['cin_slices'], cout_slices=cfg['cout_slices'],
        pad_top=cfg['pad_top'], pad_bot=cfg['pad_bot'],
        pad_left=cfg['pad_left'], pad_right=cfg['pad_right'],
        strip_rows=0)  # 单 strip 整图

    # 加 END
    end_desc = hw_files._pack_desc(hw_files.TYPE_END, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    seg.append(end_desc)

    # 转 bytes
    desc_bytes = b''.join(beat_pair_to_bytes(b0, b1) for (b0, b1) in seg)
    n_desc = len(seg)

    # 4) 数据 (Phase 2 garbage, MAC bypass)
    ifm_data  = bytes([0x11] * (cfg['ifb_words'] * 16))
    wb_data   = bytes([0x22] * (cfg['wb_words']  * 256))
    rdma_data = bytes([0x33] * (cfg['rdma_words'] * 16))

    # 5) SG cmd list: 每行一条 cmd (IDMA H_IN 条, ODMA H_OUT 条).
    #    IDMA: 每行 btt = W_IN * cin_slices * 16, sram_offset 累加.
    #    ODMA: 每行 btt = W_OUT * cout_slices * 16, ofb_w_start=0 (dispatcher 内 yout_base 推进).
    cs_in  = cfg['cin_slices']
    cs_out = cfg['cout_slices']
    row_bytes_ifm = cfg['W_IN']  * cs_in  * 16
    row_bytes_ofm = cfg['W_OUT'] * cs_out * 16

    isg_list = []
    sram_off_word = 0
    for r in range(H_IN):
        last = 1 if r == H_IN - 1 else 0
        isg_list.append(make_sg_idma(
            BRAM_IFM + r * row_bytes_ifm,
            btt=row_bytes_ifm,
            sram_off=sram_off_word,
            last_cmd=last))
        sram_off_word += cfg['W_IN'] * cs_in
    isg_blob = b''.join(isg_list)

    osg_list = []
    for r in range(H_OUT):
        last = 1 if r == H_OUT - 1 else 0
        osg_list.append(make_sg_odma(
            BRAM_OFM + r * row_bytes_ofm,
            btt=row_bytes_ofm,
            ofb_w_start=0,
            last_cmd=last))
    osg_blob = b''.join(osg_list)

    return desc_bytes, n_desc, cfg, ifm_data, wb_data, rdma_data, isg_blob, osg_blob


def decode_status(st, sd):
    return {
        'STATUS': st, 'SEQ_DBG': sd,
        'core_done':  (st >> 0) & 1, 'core_busy': (st >> 1) & 1,
        'idma_busy':  (st >> 2) & 1, 'wdma_busy': (st >> 3) & 1,
        'odma_busy':  (st >> 4) & 1,
        'idma_done':  (st >> 5) & 1, 'wdma_done': (st >> 6) & 1,
        'odma_done':  (st >> 7) & 1,
        'dfe_busy':   (st >> 8) & 1, 'dfe_done':  (st >> 9) & 1,
        'layer_busy': (st >> 10) & 1,'layer_done':(st >> 11) & 1,
        'seq_st':     sd & 0xF,       'fifo_st':   (sd >> 4) & 0xF,
        'odma_sg_st': (sd >> 8) & 0xF,'idma_sg_st':(sd >> 12) & 0xF,
    }


def chunked_load_(rpc, addr, data, chunk=1024):
    for off in range(0, len(data), chunk):
        rpc.load_ddr(addr + off, data[off:off+chunk])

def chunked_read_(rpc, addr, length, chunk=1024):
    out = bytearray()
    for off in range(0, length, chunk):
        n = min(chunk, length - off)
        out.extend(rpc.read_ddr(addr + off, n))
    return bytes(out)


def run_one_iteration(rpc, iteration, n_desc, expected_ofm_len, verbose=False):
    info = {'iter': iteration}

    # OFM pre-clear: incrementing pattern (chunked, OFM 可能 > 1KB 触发 lwIP bug)
    pre_clear = bytes([(iteration + j * 7 + 1) & 0xFF for j in range(expected_ofm_len)])
    chunked_load_(rpc, BRAM_OFM, pre_clear)
    ofm_before = chunked_read_(rpc, BRAM_OFM, expected_ofm_len)
    if ofm_before != pre_clear:
        info['err'] = f'pre-clear mismatch'
        return False, 0, info

    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)

    t = time.perf_counter()
    rpc.poke_csr(0, 0x000, 1 << 4)
    dfe_ok = False
    for _ in range(50):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 9) & 1:
            dfe_ok = True; break
        time.sleep(0.01)
    if not dfe_ok:
        sd = rpc.peek_csr(0, 0x008)
        info['err'] = f'dfe stuck STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x}'
        info.update(decode_status(st, sd))
        return False, 0, info

    t_lstart = time.perf_counter()
    rpc.poke_csr(0, 0x000, 1 << 5)
    layer_ok = False; layer_ms = 0
    for _ in range(200):       # 大 case 给 2s timeout
        st = rpc.peek_csr(0, 0x004)
        if (st >> 11) & 1:
            layer_ok = True
            layer_ms = (time.perf_counter() - t_lstart) * 1000
            break
        time.sleep(0.01)
    if not layer_ok:
        sd = rpc.peek_csr(0, 0x008)
        info['err'] = f'layer stuck after 2s STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x}'
        info.update(decode_status(st, sd))
        return False, 0, info

    sd_final = rpc.peek_csr(0, 0x008)
    final = decode_status(st, sd_final)
    info.update(final)
    info['layer_done_ms'] = layer_ms

    # 验证 OFM 真写 (Phase 2 MAC bypass, 期望 ODMA 写 expected_ofm_len byte != pre_clear)
    ofm_after = chunked_read_(rpc, BRAM_OFM, expected_ofm_len)
    info['ofm_head'] = ofm_after[:16].hex()
    if ofm_after == ofm_before:
        info['err'] = f'OFM unchanged'
        return False, layer_ms, info
    # 期望: MAC bypass 累加 0x11*0x22*K*K*Cin 严重溢出 → SDP clip_max=0x7F 饱和
    # 但 K_orig=3, cin=16, 9 cycles 累积, sum 远 > 127 → 全饱和 0x7F
    # 这里不严格 bit-exact 比对 (Phase 2 = bypass), 只看 ODMA 真有写
    return True, layer_ms, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--host', default='169.254.111.10')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--k',  type=int, default=3)
    ap.add_argument('--h',  type=int, default=8)
    ap.add_argument('--w',  type=int, default=8)
    ap.add_argument('--cin', type=int, default=16)
    ap.add_argument('--cout',type=int, default=16)
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--pad', type=int, default=1)
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--stop-on-fail', action='store_true', default=True)
    args = ap.parse_args()

    rpc = Vd100Rpc(args.host, args.port); rpc.connect()
    print(f'=== Stage 3a Phase 2: 稍大 case ===')
    print(f'    K={args.k} H={args.h} W={args.w} Cin={args.cin} Cout={args.cout} '
          f'stride={args.stride} pad={args.pad}  N={args.n}')

    # 派生
    desc_bytes, n_desc, cfg, ifm_data, wb_data, rdma_data, isg, osg = build_case(
        args.h, args.w, args.k, args.cin, args.cout, args.stride, args.pad)
    print(f'[Setup] cfg: H_OUT={cfg["H_OUT"]} W_OUT={cfg["W_OUT"]} '
          f'cin_slices={cfg["cin_slices"]} cout_slices={cfg["cout_slices"]} '
          f'TILE_W={cfg["TILE_W"]} kk={cfg["kk"]} total_wrf={cfg["total_wrf"]}')
    print(f'[Setup] sizes: IFM={len(ifm_data)}B WB={len(wb_data)}B RDMA={len(rdma_data)}B '
          f'OFM={cfg["ofb_words"]*16}B desc={len(desc_bytes)}B ({n_desc} desc)')

    # 容量 sanity check
    total_used = (DESC_OFF + len(desc_bytes), ISG_OFF + 32, OSG_OFF + 32,
                  IFM_OFF + len(ifm_data), WB_OFF + len(wb_data),
                  RDMA_OFF + len(rdma_data), OFM_OFF + cfg['ofb_words']*16)
    max_off = max(total_used)
    print(f'[Setup] max BRAM offset = 0x{max_off:05x} / 0x10000 (64KB cap)')
    if max_off > 0x10000:
        print(f'        [ERR] overflow!'); return

    # 加载 BRAM
    def chunked_load(addr, data, chunk=1024):
        for off in range(0, len(data), chunk):
            rpc.load_ddr(addr + off, data[off:off+chunk])
    chunked_load(BRAM_BASE + DESC_OFF, desc_bytes)
    chunked_load(BRAM_ISG, isg)
    chunked_load(BRAM_OSG, osg)
    chunked_load(BRAM_IFM, ifm_data)
    chunked_load(BRAM_WB,  wb_data)
    rpc.load_ddr(BRAM_RDMA, rdma_data)
    print(f'[Setup] BRAM loaded')

    expected_ofm_len = cfg['ofb_words'] * 16

    # 跑 N 轮
    pass_cnt = 0; fail_cnt = 0; times = []
    first_fail = None
    t0 = time.perf_counter()
    print(f'[Run]   start {args.n} iterations')
    print()
    for i in range(args.n):
        ok, ms, info = run_one_iteration(rpc, i, n_desc, expected_ofm_len, args.verbose)
        if ok:
            pass_cnt += 1; times.append(ms)
            if args.verbose or (i < 3) or ((i+1) % max(1, args.n // 10) == 0):
                seq = info['seq_st']; isg_st = info['idma_sg_st']; osg_st = info['odma_sg_st']
                print(f'  [{i:4d}] PASS  layer={ms:6.1f}ms  seq={seq} iSG={isg_st} oSG={osg_st} '
                      f'OFM[0..16]={info["ofm_head"]}')
        else:
            fail_cnt += 1
            if first_fail is None: first_fail = info
            print(f'  [{i:4d}] FAIL  {info.get("err","?")}')
            print(f'         STATUS=0x{info.get("STATUS",0):08x} '
                  f'SEQ_DBG=0x{info.get("SEQ_DBG",0):08x}')
            print(f'         seq={info.get("seq_st",0)} fifo={info.get("fifo_st",0)} '
                  f'iSG={info.get("idma_sg_st",0)} oSG={info.get("odma_sg_st",0)}')
            print(f'         dfe_done={info.get("dfe_done",0)} '
                  f'idma_done={info.get("idma_done",0)} '
                  f'wdma_done={info.get("wdma_done",0)} '
                  f'odma_done={info.get("odma_done",0)} '
                  f'layer_done={info.get("layer_done",0)} '
                  f'layer_busy={info.get("layer_busy",0)}')
            if args.stop_on_fail: break

    t_total = time.perf_counter() - t0
    print()
    print('=' * 60)
    print(f'[Summary]')
    print(f'  total: {pass_cnt+fail_cnt}/{args.n}  PASS={pass_cnt}  FAIL={fail_cnt}')
    print(f'  time:  {t_total:.1f}s ({(pass_cnt+fail_cnt)/max(t_total, 0.001):.1f} iter/s)')
    if times:
        print(f'  layer_done ms:  min={min(times):.1f}  max={max(times):.1f}  '
              f'mean={statistics.mean(times):.1f}  stdev={statistics.pstdev(times):.2f}')
        if len(times) >= 8:
            q = len(times) // 4
            head = statistics.mean(times[:q]); tail = statistics.mean(times[-q:])
            drift = tail - head
            print(f'  drift: head={head:.2f}ms tail={tail:.2f}ms Δ={drift:+.2f}ms '
                  f'({drift/head*100:+.1f}%)')
            if drift > head * 0.5:
                print(f'  [WARN] tail slowdown > 50%')

    rpc.close()
    sys.exit(0 if fail_cnt == 0 else 1)


if __name__ == '__main__':
    main()
