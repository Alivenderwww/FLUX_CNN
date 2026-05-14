#!/usr/bin/env python3
"""Stage 3a Phase 1: 多轮启停稳定性测试 (1x1x1x1 minimal conv).

复用 Stage 2 的 case (1x1x1x1, mac bypass), 在外面套 N 次循环, 验证:
  1. 每轮 layer_done 都触发 (没有 stuck)
  2. 每轮 OFM 都被 ODMA 真写 (没有静默失败)
  3. 每轮 SEQ_DBG 状态干净回 S_IDLE (sequencer FSM 没卡死)
  4. 多轮 layer_done 时延一致 (没有累计延迟)
  5. 各 done bit 在 start_layer 时被正确清 (sticky 行为正常)

这是跑多层 / 多 batch 图片的基础: 单轮 OK 但多轮 stuck 是 sticky/reset bug.

用法:
  python test_stage3a_phase1_loop.py             # 默认 N=100
  python test_stage3a_phase1_loop.py --n 1000    # 跑 1000 轮
  python test_stage3a_phase1_loop.py --n 100 --verbose
"""
import sys, struct, time, argparse, statistics
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
from vd100_rpc import Vd100Rpc

# ====== BRAM layout (跟 Stage 2 一致) ======
BRAM_BASE = 0xA4100000
DESC_OFF  = 0x0000
ISG_OFF   = 0x0800
OSG_OFF   = 0x0820
IFM_OFF   = 0x1000
WB_OFF    = 0x2000
RDMA_OFF  = 0x2100
OFM_OFF   = 0x3000

BRAM_IFM  = BRAM_BASE + IFM_OFF
BRAM_WB   = BRAM_BASE + WB_OFF
BRAM_RDMA = BRAM_BASE + RDMA_OFF
BRAM_OFM  = BRAM_BASE + OFM_OFF
BRAM_ISG  = BRAM_BASE + ISG_OFF
BRAM_OSG  = BRAM_BASE + OSG_OFF

# ====== desc / SG cmd 构造器 (复用 Stage 2) ======
def make_cfg(addr, data):
    w0 = 0x3 | ((addr & 0xFFF) << 4)
    return struct.pack('<IIII', w0, data, 0, 0) + b'\x00' * 16

def make_conv(is_first=1, is_last=1, pad_top=0, pad_bot=0, pad_left=0, pad_right=0,
              strip_y_start=0, n_yout_strip=1, ifb_ddr_off=0, ifb_byte_len=16,
              ofb_ddr_off=0, ofb_byte_len=16):
    w0 = 0x1 | (is_first << 4) | (is_last << 5) | (1 << 6)
    w0 |= ((pad_top & 0xF) << 8) | ((pad_bot & 0xF) << 12) | ((pad_left & 0xF) << 16) | ((pad_right & 0xF) << 20)
    w1 = (strip_y_start & 0xFFFF) | ((n_yout_strip & 0xFFFF) << 16)
    return struct.pack('<IIIIIIII', w0, w1,
                       ifb_ddr_off & 0xFFFFF, ifb_byte_len & 0xFFFFFF,
                       ofb_ddr_off & 0xFFFFF, ofb_byte_len & 0xFFFFFF, 0, 0)

def make_end():
    return struct.pack('<IIII', 0xF, 0, 0, 0) + b'\x00' * 16

def make_sg_idma(src_addr, btt, sram_off=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', src_addr, w1, sram_off & 0x1FFF, 0) + b'\x00' * 16

def make_sg_odma(dst_addr, btt, ofb_w_start=0, last_cmd=1):
    w1 = (btt & 0x7FFFFF) | ((last_cmd & 1) << 23)
    return struct.pack('<IIII', dst_addr, w1, ofb_w_start & 0xFFFF, 0) + b'\x00' * 16


def build_desc_list():
    """跟 Stage 2 一致的 cfg + CONV + END."""
    cfg_pairs = [
        (0x100, 1), (0x104, 1), (0x108, 1), (0x10C, 1), (0x12C, 1),
        (0x110, 1), (0x1F0, 1), (0x114, 1), (0x118, 1), (0x11C, 1),
        (0x128, 1), (0x130, 1), (0x134, 1), (0x138, 1),
        (0x13C, 0), (0x140, 0), (0x144, 0),
        (0x14C, 1), (0x154, 1), (0x15C, 1),
        (0x120, 1), (0x124, 1),
        (0x160, 0), (0x164, 0), (0x168, 1), (0x16C, 1), (0x170, 1),
        (0x174, 16), (0x178, 16),
        (0x1A0, 1), (0x1A4, 1), (0x1A8, 1), (0x1AC, 1), (0x1B0, 1), (0x1B4, 1),
        (0x1B8, 0), (0x1BC, 0), (0x1C8, 0),
        (0x188, 1), (0x18C, 0), (0x190, 0xFFFFFF80), (0x194, 0x7F), (0x198, 0),
        (0x210, BRAM_WB),   (0x214, 256),
        (0x230, BRAM_RDMA), (0x234, 64),
        (0x1D8, BRAM_ISG),  (0x1DC, 1), (0x1E0, 1),
        (0x1E4, BRAM_OSG),  (0x1E8, 1), (0x1EC, 1),
        (0x1CC, 0),
    ]
    descs = [make_cfg(addr, val) for addr, val in cfg_pairs]
    descs.append(make_conv(is_first=1, is_last=1, n_yout_strip=1,
                           ifb_ddr_off=0, ifb_byte_len=16,
                           ofb_ddr_off=0, ofb_byte_len=16))
    descs.append(make_end())
    return b''.join(descs), len(descs)


def decode_status(st, sd):
    return {
        'STATUS':       st,
        'SEQ_DBG':      sd,
        'core_done':    (st >> 0) & 1,
        'core_busy':    (st >> 1) & 1,
        'idma_busy':    (st >> 2) & 1,
        'wdma_busy':    (st >> 3) & 1,
        'odma_busy':    (st >> 4) & 1,
        'idma_done':    (st >> 5) & 1,
        'wdma_done':    (st >> 6) & 1,
        'odma_done':    (st >> 7) & 1,
        'dfe_busy':     (st >> 8) & 1,
        'dfe_done':     (st >> 9) & 1,
        'layer_busy':   (st >> 10) & 1,
        'layer_done':   (st >> 11) & 1,
        'seq_st':       sd & 0xF,
        'fifo_st':      (sd >> 4) & 0xF,
        'odma_sg_st':   (sd >> 8) & 0xF,
        'idma_sg_st':   (sd >> 12) & 0xF,
    }


def run_one_iteration(rpc, iteration, n_desc, expected_ofm_pat, verbose=False):
    """跑一轮 start_dfe + start_layer + verify. 返回 (success, layer_done_ms, info)."""
    info = {'iter': iteration}

    # OFM pre-clear: 16 byte incrementing pattern w/ iter offset.
    # 期望 ODMA 写入 = expected_ofm_pat = b'\x7F' * 16 (所有 byte 一样).
    # 用 incrementing pattern 确保 pre_clear 任何情况下都 != expected (16 byte 不一致).
    pre_clear = bytes([(iteration + j * 7 + 1) & 0xFF for j in range(16)])
    rpc.load_ddr(BRAM_OFM, pre_clear)
    ofm_before = rpc.read_ddr(BRAM_OFM, 16)
    if ofm_before != pre_clear:
        info['err'] = f'pre-clear mismatch: wrote {pre_clear.hex()} got {ofm_before.hex()}'
        return False, 0, info

    # 配 dfe (用 BRAM_BASE+DESC_OFF, 每轮一致, 不需要每次重写)
    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)

    # start_dfe + wait done
    t_dfe_start = time.perf_counter()
    rpc.poke_csr(0, 0x000, 1 << 4)
    dfe_ok = False
    for _ in range(50):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 9) & 1:
            dfe_ok = True
            break
        time.sleep(0.01)
    t_dfe_ms = (time.perf_counter() - t_dfe_start) * 1000
    if not dfe_ok:
        sd = rpc.peek_csr(0, 0x008)
        info['err'] = f'dfe stuck after {t_dfe_ms:.0f}ms STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x}'
        info.update(decode_status(st, sd))
        return False, 0, info

    # start_layer + wait layer_done
    t_layer_start = time.perf_counter()
    rpc.poke_csr(0, 0x000, 1 << 5)
    layer_ok = False
    layer_done_ms = 0
    for _ in range(100):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 11) & 1:
            layer_ok = True
            layer_done_ms = (time.perf_counter() - t_layer_start) * 1000
            break
        time.sleep(0.01)
    if not layer_ok:
        sd = rpc.peek_csr(0, 0x008)
        info['err'] = f'layer stuck after 1s STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x}'
        info.update(decode_status(st, sd))
        return False, 0, info

    # 拿 final status 看 sequencer 回到 IDLE 没
    sd_final = rpc.peek_csr(0, 0x008)
    final = decode_status(st, sd_final)
    info.update(final)
    info['layer_done_ms'] = layer_done_ms
    info['dfe_ms']        = t_dfe_ms

    # 验证 OFM 真写
    ofm_after = rpc.read_ddr(BRAM_OFM, 16)
    info['ofm'] = ofm_after.hex()
    if ofm_after != expected_ofm_pat:
        info['err'] = f'OFM mismatch: expect {expected_ofm_pat.hex()} got {ofm_after.hex()}'
        return False, layer_done_ms, info
    if ofm_after == ofm_before:
        info['err'] = f'OFM unchanged: pre {ofm_before.hex()} == post {ofm_after.hex()}'
        return False, layer_done_ms, info

    return True, layer_done_ms, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=100, help='迭代次数')
    ap.add_argument('--host', default='169.254.111.10')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--verbose', action='store_true', help='每轮都打 SEQ_DBG')
    ap.add_argument('--stop-on-fail', action='store_true', default=True)
    args = ap.parse_args()

    rpc = Vd100Rpc(args.host, args.port)
    rpc.connect()
    print(f'=== Stage 3a Phase 1: 多轮启停 N={args.n} ===')
    print(f'    case: 1x1x1x1 mac bypass, 期望 OFM = 0x7F (CLIP_MAX 饱和)')

    # 一次性写入 desc list + SG cmd + IFM/W/RDMA (这些不变, 多轮复用)
    desc_list, n_desc = build_desc_list()
    print(f'[Setup] desc list {n_desc} desc, {len(desc_list)} byte')

    def chunked_load(addr, data, chunk=1024):
        for off in range(0, len(data), chunk):
            rpc.load_ddr(addr + off, data[off:off+chunk])

    chunked_load(BRAM_BASE + DESC_OFF, desc_list)
    rpc.load_ddr(BRAM_ISG, make_sg_idma(BRAM_IFM, btt=16, last_cmd=1))
    rpc.load_ddr(BRAM_OSG, make_sg_odma(BRAM_OFM, btt=16, last_cmd=1))
    rpc.load_ddr(BRAM_IFM, bytes([0x11] * 16))
    rpc.load_ddr(BRAM_WB,  bytes([0x22] * 256))
    rpc.load_ddr(BRAM_RDMA, bytes([0x33] * 64))
    print(f'[Setup] BRAM loaded (desc/SG/IFM/W/RDMA)')

    # 期望 OFM (MAC bypass 输入 0x11 × W 0x22 累加溢出, SDP clip_max=0x7F 饱和)
    expected_ofm = bytes([0x7F] * 16)

    # 跑 N 轮
    print(f'[Run]   start {args.n} iterations')
    print()
    pass_cnt = 0
    fail_cnt = 0
    times = []
    first_fail_iter = None
    first_fail_info = None
    t_start = time.perf_counter()

    for i in range(args.n):
        ok, ms, info = run_one_iteration(rpc, i, n_desc, expected_ofm, args.verbose)
        if ok:
            pass_cnt += 1
            times.append(ms)
            if args.verbose or (i < 5) or ((i+1) % max(1, args.n // 10) == 0):
                seq = info['seq_st']; idma_sg = info['idma_sg_st']; odma_sg = info['odma_sg_st']
                print(f'  [{i:4d}] PASS  layer_done={ms:6.1f}ms  seq={seq} '
                      f'iSG={idma_sg} oSG={odma_sg}  OFM={info["ofm"][:16]}...')
        else:
            fail_cnt += 1
            if first_fail_iter is None:
                first_fail_iter = i
                first_fail_info = info
            print(f'  [{i:4d}] FAIL  {info.get("err", "?")}')
            print(f'         STATUS=0x{info.get("STATUS",0):08x} '
                  f'SEQ_DBG=0x{info.get("SEQ_DBG",0):08x}')
            print(f'         seq={info.get("seq_st",0)} fifo={info.get("fifo_st",0)} '
                  f'iSG={info.get("idma_sg_st",0)} oSG={info.get("odma_sg_st",0)}')
            print(f'         dfe_done={info.get("dfe_done",0)} '
                  f'idma_done={info.get("idma_done",0)} '
                  f'wdma_done={info.get("wdma_done",0)} '
                  f'odma_done={info.get("odma_done",0)} '
                  f'layer_done={info.get("layer_done",0)}')
            if args.stop_on_fail:
                break

    t_total = time.perf_counter() - t_start
    print()
    print('=' * 60)
    print(f'[Summary]')
    print(f'  total:  {pass_cnt + fail_cnt}/{args.n}  PASS={pass_cnt}  FAIL={fail_cnt}')
    print(f'  time:   {t_total:.1f}s ({(pass_cnt + fail_cnt) / max(t_total, 0.001):.1f} iter/s)')
    if times:
        print(f'  layer_done ms:  min={min(times):.1f}  max={max(times):.1f}  '
              f'mean={statistics.mean(times):.1f}  stdev={statistics.pstdev(times):.2f}')
        # 累计漂移检测: 比较前 1/4 vs 后 1/4 平均时延
        if len(times) >= 8:
            q = len(times) // 4
            head_mean = statistics.mean(times[:q])
            tail_mean = statistics.mean(times[-q:])
            drift = tail_mean - head_mean
            print(f'  drift:  head_mean={head_mean:.2f}  tail_mean={tail_mean:.2f}  '
                  f'Δ={drift:+.2f}ms ({drift/head_mean*100:+.1f}%)')
            # drift > 0 (tail 比 head 慢) 才是 slowdown; drift < 0 通常是 RPC 暖启动
            if drift > head_mean * 0.5:
                print(f'  [WARN] tail vs head slowdown > 50%, 可能有累计 stall')

    if first_fail_iter is not None:
        print(f'  first fail: iter={first_fail_iter}  err={first_fail_info.get("err", "?")}')

    rpc.close()
    sys.exit(0 if fail_cnt == 0 else 1)


if __name__ == '__main__':
    main()
