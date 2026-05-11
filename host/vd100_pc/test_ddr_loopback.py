"""
test_ddr_loopback.py — VD100 板 PS↔DDR 通路验证 (Phase 1)

跑这个之前必须有:
  1. VD100 板烧好 PDI (Versal 启动 a72 baremetal ELF)
  2. ELF 是 vd100_rpc_server.c 编出来的 (Vitis workspace2/resnet11_app)
  3. 网线接通, 板 IP 169.254.111.10 (跟 PC 同网段)

测试矩阵:
  T1 ping            — TCP 通 + RPC 协议通
  T2 LOAD/READ 4 byte         — 最简 PS DDR 写读
  T3 LOAD/READ 4 KB pattern   — 中等大小, 验证 cache flush
  T4 LOAD/READ 1 MB random    — 大块, 验证流式 memcpy + 不丢数据
  T5 POKE/PEEK CSR scratchpad — CSR aperture 通 (写 0/读不应该 hang)

PS↔DDR 通路 OK 之后才能做 Phase 1.5 (PL→DDR 通路: 让 ConvCore IDMA 拉数据验证).
"""
import argparse
import random
import socket
import struct
import sys
import time

import vd100_rpc
from vd100_rpc import Vd100Rpc, csr_addr, CSR_CTRL, CSR_STATUS, NUM_CORES


# DDR 测试区段 (避开 a72 ELF 0x0-0x01000000)
TEST_DDR_BASE = 0x10000000
TEST_DDR_SIZE = 16 * 1024 * 1024   # 16 MB 区间够了


def t1_ping(rpc):
    print("[T1] PING ...", end=' ', flush=True)
    t0 = time.time()
    val = rpc.ping()
    rtt_ms = (time.time() - t0) * 1000
    assert val == 0xCAFEBABE, f"got 0x{val:08x}"
    print(f"OK (0x{val:08x}, RTT={rtt_ms:.1f}ms)")


def t2_small_loopback(rpc):
    print("[T2] LOAD/READ 4 byte ...", end=' ', flush=True)
    addr = TEST_DDR_BASE
    pattern = struct.pack('<I', 0xDEADBEEF)
    rpc.load_ddr(addr, pattern)
    got = rpc.read_ddr(addr, 4)
    if got != pattern:
        print(f"FAIL: wrote {pattern.hex()} got {got.hex()}")
        return False
    print(f"OK ({pattern.hex()})")
    return True


def t3_pattern_4k(rpc):
    print("[T3] LOAD/READ 4 KB pattern ...", end=' ', flush=True)
    addr = TEST_DDR_BASE + 0x10000   # +64 KB offset
    # 0x00 0x01 0x02 ... 0xFF 0x00 ... 4 KB
    pattern = bytes(i & 0xFF for i in range(4096))
    rpc.load_ddr(addr, pattern)
    got = rpc.read_ddr(addr, 4096)
    if got != pattern:
        diff = next((i for i in range(4096) if got[i] != pattern[i]), -1)
        print(f"FAIL: first diff at byte {diff}, expected {pattern[diff]:02x} got {got[diff]:02x}")
        return False
    print(f"OK (4096 byte all match)")
    return True


def t4_random_1m(rpc, seed=0xBEEF):
    size = 1 * 1024 * 1024
    print(f"[T4] LOAD/READ {size//1024} KB random (seed=0x{seed:x}) ...", end=' ', flush=True)
    addr = TEST_DDR_BASE + 0x100000   # +1 MB offset
    rng = random.Random(seed)
    pattern = rng.randbytes(size)
    t0 = time.time()
    rpc.load_ddr(addr, pattern)
    t_load = time.time() - t0
    t0 = time.time()
    got = rpc.read_ddr(addr, size)
    t_read = time.time() - t0
    if got != pattern:
        diff = next((i for i in range(size) if got[i] != pattern[i]), -1)
        print(f"FAIL: first diff at byte {diff}")
        return False
    print(f"OK (LOAD {size/t_load/1e6:.1f} MB/s, READ {size/t_read/1e6:.1f} MB/s)")
    return True


def t5_csr_scratch(rpc):
    """用 STATUS reg 验证 CSR aperture 通. STATUS 是 read-only sticky, 验证读不 hang.
    再用 CTRL 写 0x0 (no-op, bit 4=0 不触发 dfe) 验证写不 hang.
    跑这两个的前提: 板 reset 之后没人触发过 ConvCore (STATUS bit 0=0)."""
    print("[T5] CSR aperture (PEEK STATUS, POKE CTRL=0) ...", end=' ', flush=True)
    for c in range(NUM_CORES):
        v = rpc.peek_csr(c, CSR_STATUS)
        # STATUS 应该是 0 (没启动过) 或 0x1 (sticky done from prev run); 不能 timeout
        rpc.poke_csr(c, CSR_CTRL, 0)   # no-op
        # 再读一次, 不应该 hang
        v2 = rpc.peek_csr(c, CSR_STATUS)
    print(f"OK (STATUS[0..{NUM_CORES-1}] read+write 通)")
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default='169.254.111.10', help='VD100 板 IP')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--skip-large', action='store_true', help='跳过 T4 1MB 大块测试')
    args = p.parse_args()

    print(f"Connect VD100 @ {args.ip}:{args.port}")
    try:
        with Vd100Rpc(args.ip, args.port) as rpc:
            t1_ping(rpc)
            ok = t2_small_loopback(rpc)
            if ok:
                ok = t3_pattern_4k(rpc)
            if ok and not args.skip_large:
                ok = t4_random_1m(rpc)
            if ok:
                ok = t5_csr_scratch(rpc)
            if ok:
                print("\n=== ALL TESTS PASSED ===")
                print("PS<->DDR + PS->CSR aperture 通路 OK, 可跑 deploy_smc_case / run_resnet11_demo")
                return 0
            else:
                print("\n=== TEST FAILED ===")
                return 1
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"\nERROR: 连不上 VD100 ({e})")
        print("  检查: 1) 板上电 2) 网线接好 3) PDI/ELF 烧写 4) IP 配对")
        return 2
    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}")
        return 3


if __name__ == '__main__':
    sys.exit(main())
