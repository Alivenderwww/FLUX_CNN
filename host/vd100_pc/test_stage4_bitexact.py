#!/usr/bin/env python3
"""Stage 4: 真 MAC bit-exact 验证.

跟 Stage 3a Phase 2 的差异:
  - 不再 garbage IFM/W (0x11/0x22), 用 toolchain gen_isa_test 生成真随机数据
  - 不再期望全 0x7F 饱和, 用 hw_files.compute_expected_ofm 算 bit-exact 期望
  - 读 OFM 后字节级比对 board 输出 vs expected_ofm

这是论文 "board-level bit-exact validated" 核心里程碑.

用法:
  # 默认 case: K=3 H=W=8 Cin=Cout=16
  python test_stage4_bitexact.py
  # 自定义 case
  python test_stage4_bitexact.py --k 3 --h 16 --w 16 --cin 16 --cout 16
  # 多轮 (验证多次跑结果一致, 也验证软 reset 后状态干净)
  python test_stage4_bitexact.py --n 10
"""
import sys, struct, time, argparse, os, subprocess
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
import hw_files

# ====== BRAM layout (v6 fix: ISG/OSG 各 4KB 装 128 cmd) ======
# 关键修复 2026-05-15: 原 ISG/OSG 各 1KB = 32 cmd 容量, H_OUT > 32 时 ODMA cmd_count
# 越出 OSG 区写到 BRAM_IFM, ODMA dispatcher 拉 cmd 32 时拿到 IFM data 当 cmd → r_dst_addr
# 是 random IFM byte (不是合法地址) → axi_dm 写到错位置, row 31 被污染 + row 32 没写.
# 这是 H>32 deterministic mismatch 643 byte @ row 31 的真因 (不是 RTL bug).
# Fix: ISG/OSG 各 4KB (128 cmd 容量), DESC 4KB.
BRAM_BASE = 0xA4100000
DESC_OFF  = 0x00000   # 4 KB
ISG_OFF   = 0x01000   # 4 KB (128 cmd 容量, 够 H≤128)
OSG_OFF   = 0x02000   # 4 KB
IFM_OFF   = 0x03000   # 起点延后 0x1000
WB_OFF    = 0x10000
RDMA_OFF  = 0x20000
OFM_OFF   = 0x30000

BRAM_IFM  = BRAM_BASE + IFM_OFF
BRAM_WB   = BRAM_BASE + WB_OFF
BRAM_RDMA = BRAM_BASE + RDMA_OFF
BRAM_OFM  = BRAM_BASE + OFM_OFF
BRAM_ISG  = BRAM_BASE + ISG_OFF
BRAM_OSG  = BRAM_BASE + OSG_OFF

# CTRL bit
CTRL_START_DFE   = 1 << 4
CTRL_START_LAYER = 1 << 5
CTRL_SOFT_RESET  = 1 << 7   # v2 new: 写 1 软 reset (16 拍低 → sub-module 回 IDLE)


def parse_hex_file(path):
    """每行 N hex char = N/2 byte word (ifb=32 char/16 byte, wb=512 char/256 byte).
    hex line 大端, to_bytes('little') LSB byte 在低地址."""
    out = bytearray()
    with open(path, 'r') as f:
        for line in f:
            line = line.strip().split('//')[0].strip()
            if not line:
                continue
            assert len(line) % 2 == 0, f"bad line len: {len(line)}"
            n_bytes = len(line) // 2
            val = int(line, 16)
            out.extend(val.to_bytes(n_bytes, 'little'))
    return bytes(out)


def parse_rdma_data(path):
    """rdma_data.txt 跟 ifb / wb 同格式 (32 hex / line = 16 byte / word)."""
    return parse_hex_file(path)


def make_sg_idma(src_addr, btt, sram_off=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', src_addr, w1, sram_off & 0x1FFF, 0) + b'\x00' * 16


def make_sg_odma(dst_addr, btt, ofb_w_start=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', dst_addr, w1, ofb_w_start & 0xFFFF, 0) + b'\x00' * 16


def beat_pair_to_bytes(beat0, beat1):
    return beat0.to_bytes(16, 'little') + beat1.to_bytes(16, 'little')


def build_case(H_IN, W_IN, K, NUM_CIN, NUM_COUT, stride, pad, case_dir):
    """跑 gen_isa_test 生成 case, parse 输出文件.
    return: (desc_bytes, n_desc, cfg, ifm, wb, rdma, expected_ofm, isg, osg)"""
    os.makedirs(case_dir, exist_ok=True)
    cmd = [sys.executable,
           os.path.join(r'C:/_Project/FLUX_CNN/toolchain', 'gen_isa_test.py'),
           '--k', str(K), '--h_in', str(H_IN), '--w_in', str(W_IN),
           '--num_cin', str(NUM_CIN), '--num_cout', str(NUM_COUT),
           '--stride', str(stride), '--pad', str(pad),
           '--out-dir', case_dir, '--seed', '42']
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=r'C:/_Project/FLUX_CNN/toolchain')
    if r.returncode != 0:
        print(r.stdout); print(r.stderr)
        raise RuntimeError("gen_isa_test failed")

    # parse hex 文件
    ifm  = parse_hex_file(os.path.join(case_dir, 'ifb.txt'))
    wb   = parse_hex_file(os.path.join(case_dir, 'wb.txt'))
    rdma = parse_rdma_data(os.path.join(case_dir, 'rdma_data.txt'))
    expected_ofm = parse_hex_file(os.path.join(case_dir, 'expected_ofm.txt'))

    # cfg dict 再生成一次 (因 gen_isa_test 不能直接返 cfg dict)
    # VD100 fix 2026-05-15: 强制 OFB_SRAM_WORDS small 让 hw_files 走 streaming
    # (避免整图 fit ofb_strip=H_OUT 时 ring_full chicken-egg deadlock).
    h_out_est = (H_IN + 2*pad - K) // stride + 1
    w_out_est = (W_IN + 2*pad - K) // stride + 1
    cs_out_est = (NUM_COUT + 15) // 16
    OFB_FORCE = 9 * w_out_est * cs_out_est  # 8 strip + 1 slack
    cfg = hw_files.derive_layer_cfg(
        OFB_SRAM_WORDS=OFB_FORCE,
        H_IN=H_IN, W_IN=W_IN, K=K, NUM_CIN=NUM_CIN, NUM_COUT=NUM_COUT,
        stride=stride, pad_top=pad, pad_left=pad, TILE_W=32)

    H_IN_eff, H_OUT = cfg['H_IN'], cfg['H_OUT']
    cfg_dict = hw_files.cfg_to_dict(
        cfg, shift_amt=0, sdp_mult=1, sdp_zp_out=0,
        sdp_clip_min=0, sdp_clip_max=255, sdp_round_en=0, sdp_relu_en=1,
        # ↑ 跟 gen_isa_test 默认 SDP 参数一致 (F-1a: clip[0,255], ReLU on)
        ddr_ifb_base=BRAM_IFM, ddr_wb_base=BRAM_WB,
        ddr_ofb_base=BRAM_OFM, ddr_rdma_base=BRAM_RDMA,
        rdma_words_total=cfg['rdma_words'],
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
    n_desc = len(seg)

    # SG cmd: 每行一条 (H_IN/H_OUT 条)
    cs_in  = cfg['cin_slices']
    cs_out = cfg['cout_slices']
    row_bytes_ifm = cfg['W_IN']  * cs_in  * 16
    row_bytes_ofm = cfg['W_OUT'] * cs_out * 16

    # 注意: streaming row-ring 模式 (ifb_strip < H_IN) 下 sram_offset 必须按
    # ifb_ring_words wrap! 否则后续 cmd 写到 ring 外, line_buffer 读 ring 内
    # stale data, board IDMA 卡 S_RING_WAIT.
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

    return desc_bytes, n_desc, cfg, ifm, wb, rdma, expected_ofm, isg_blob, osg_blob


def chunked_load(rpc, addr, data, chunk=1024):
    for off in range(0, len(data), chunk):
        rpc.load_ddr(addr + off, data[off:off+chunk])

def chunked_read(rpc, addr, length, chunk=1024):
    out = bytearray()
    for off in range(0, length, chunk):
        n = min(chunk, length - off)
        out.extend(rpc.read_ddr(addr + off, n))
    return bytes(out)


def soft_reset(rpc):
    """v2: 写 CTRL.bit7 触发软 reset (16 拍低), 把 sequencer/dispatcher 拉回 IDLE."""
    rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET)
    time.sleep(0.001)  # 等 16 拍 + margin


def run_one(rpc, n_desc, expected_ofm, args):
    """跑一轮 + bit-exact 比对. return (ok, layer_ms, err_msg, mismatch_bytes)."""
    # 每 run 前软 reset, 清 sub-module FSM + axi_dm (v4 fix 后 axi_dm 也在 soft reset 范围)
    rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET)
    time.sleep(0.001)

    expected_len = len(expected_ofm)
    # OFM 预清成跟 expected 不同的 pattern (检测 ODMA 真有写)
    pre_clear = bytes((j ^ 0xAA) & 0xFF for j in range(expected_len))
    chunked_load(rpc, BRAM_OFM, pre_clear)

    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)

    # start_dfe
    rpc.poke_csr(0, 0x000, CTRL_START_DFE)
    for _ in range(50):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 9) & 1: break
        time.sleep(0.01)
    else:
        return False, 0, f'dfe stuck STATUS=0x{st:08x}', 0

    # start_layer
    t = time.perf_counter()
    rpc.poke_csr(0, 0x000, CTRL_START_LAYER)
    for _ in range(500):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 11) & 1:
            ms = (time.perf_counter() - t) * 1000
            break
        time.sleep(0.01)
    else:
        sd = rpc.peek_csr(0, 0x008)
        return False, 0, f'layer stuck STATUS=0x{st:08x} SEQ=0x{sd:08x}', 0

    # 读 OFM bit-exact 比对
    ofm = chunked_read(rpc, BRAM_OFM, expected_len)
    if ofm == expected_ofm:
        return True, ms, None, 0
    # 不等 → 算 mismatch
    diff = sum(1 for a, b in zip(ofm, expected_ofm) if a != b)
    # 首 mismatch 偏移
    for i, (a, b) in enumerate(zip(ofm, expected_ofm)):
        if a != b:
            first_diff = i
            break
    return False, ms, f'OFM mismatch ({diff}/{expected_len} bytes), first @ offset {first_diff}: got 0x{ofm[first_diff]:02x} expect 0x{expected_ofm[first_diff]:02x}', diff


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=1)
    ap.add_argument('--host', default='169.254.111.10')
    ap.add_argument('--k',   type=int, default=3)
    ap.add_argument('--h',   type=int, default=8)
    ap.add_argument('--w',   type=int, default=8)
    ap.add_argument('--cin', type=int, default=16)
    ap.add_argument('--cout',type=int, default=16)
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--pad',    type=int, default=1)
    ap.add_argument('--reset-each-iter', action='store_true',
                    help='每轮跑前发软 reset (验证软 reset 通路)')
    ap.add_argument('--case-dir', default=r'C:/_Project/FLUX_CNN/sim/tb_stage4_case')
    args = ap.parse_args()

    rpc = Vd100Rpc(args.host); rpc.connect()
    print(f'=== Stage 4 bit-exact: K={args.k} H={args.h} W={args.w} '
          f'Cin={args.cin} Cout={args.cout} pad={args.pad} N={args.n} ===')

    desc, n_desc, cfg, ifm, wb, rdma, expected, isg, osg = build_case(
        args.h, args.w, args.k, args.cin, args.cout, args.stride, args.pad, args.case_dir)

    print(f'[Setup] sizes: ifm={len(ifm)} wb={len(wb)} rdma={len(rdma)} expected={len(expected)} '
          f'desc={len(desc)}({n_desc} desc) isg={len(isg)} osg={len(osg)}')
    used = max(DESC_OFF + len(desc), ISG_OFF + len(isg), OSG_OFF + len(osg),
               IFM_OFF + len(ifm), WB_OFF + len(wb), RDMA_OFF + len(rdma),
               OFM_OFF + len(expected))
    cap = 256 * 1024
    print(f'[Setup] max BRAM offset = 0x{used:05x} / 0x{cap:05x} ({used*100/cap:.1f}%)')
    if used > cap:
        print(f'        [ERR] BRAM overflow!'); return

    # Load BRAM
    chunked_load(rpc, BRAM_BASE + DESC_OFF, desc)
    chunked_load(rpc, BRAM_ISG, isg)
    chunked_load(rpc, BRAM_OSG, osg)
    chunked_load(rpc, BRAM_IFM, ifm)
    chunked_load(rpc, BRAM_WB,  wb)
    chunked_load(rpc, BRAM_RDMA, rdma)
    print(f'[Setup] BRAM loaded')

    # Run iterations
    pass_cnt = 0; fail_cnt = 0
    for i in range(args.n):
        if args.reset_each_iter and i > 0:
            soft_reset(rpc)
        ok, ms, err, mismatch = run_one(rpc, n_desc, expected, args)
        if ok:
            pass_cnt += 1
            print(f'  [{i:3d}] PASS  layer_done={ms:6.1f}ms  OFM 全 {len(expected)} byte bit-exact')
        else:
            fail_cnt += 1
            print(f'  [{i:3d}] FAIL  {err}')

    print()
    print(f'[Summary] {pass_cnt}/{args.n} PASS  {fail_cnt}/{args.n} FAIL')
    rpc.close()
    sys.exit(0 if fail_cnt == 0 else 1)


if __name__ == '__main__':
    main()
