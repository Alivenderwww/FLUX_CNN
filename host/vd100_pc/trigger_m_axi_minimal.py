"""trigger_m_axi_minimal.py — 触发 ConvCore m_axi 发 AR 到 NoC (最小用例)

只让 core_0 启动 DFE 拉一次 desc_list (从 DDR), 看 ILA 是否触发 pc_asserted.
不依赖完整 case 数据, 只写最小 dummy desc 让 IDMA 发个 AR.

Usage:
    python trigger_m_axi_minimal.py --vd100-ip 169.254.111.10
"""
import argparse
import struct
import sys
import time

from vd100_rpc import Vd100Rpc, CSR_CTRL, CSR_STATUS, CSR_DESC_LIST_BASE, CSR_DESC_COUNT, CTRL_START_DFE

DDR_BASE = 0x10000000   # 跟 FLUX_SMC_GLOBAL_BASE 一致, 避开 A72 ELF
DESC_BASE = DDR_BASE + 0x00A00000    # core_0 desc list base
DESC_COUNT = 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--vd100-ip', default='169.254.111.10')
    p.add_argument('--vd100-port', type=int, default=5000)
    args = p.parse_args()

    print(f"Connecting VD100 @ {args.vd100_ip}:{args.vd100_port}")
    with Vd100Rpc(args.vd100_ip, args.vd100_port) as rpc:
        ret = rpc.ping()
        print(f"  PING ok (RTT={ret*1000:.1f}ms)" if isinstance(ret, float) else f"  PING ret={ret}")

        # Step 1: 写一个 dummy desc 到 DDR (32 byte / desc, 跟 sequencer 一致)
        # desc 内容随便, 重点是让 IDMA 发 AR 拉这个地址
        dummy_desc = b'\x00' * 32
        print(f"  LOAD dummy desc to 0x{DESC_BASE:08x}")
        rpc.load_ddr(DESC_BASE, dummy_desc)

        # Step 2: 写 core_0 CSR: DESC_LIST_BASE + DESC_COUNT
        print(f"  POKE core_0 CSR DESC_LIST_BASE=0x{DESC_BASE:08x} COUNT={DESC_COUNT}")
        rpc.poke_csr(0, CSR_DESC_LIST_BASE, DESC_BASE)
        rpc.poke_csr(0, CSR_DESC_COUNT, DESC_COUNT)

        # Step 3: 触发 START_DFE
        print(f"  POKE core_0 CSR CTRL=START_DFE (0x{CTRL_START_DFE:x})")
        rpc.poke_csr(0, CSR_CTRL, CTRL_START_DFE)

        # Step 4: 等几秒看 ILA 是否触发 (本脚本不读 ILA, 由 arm_ila_and_wait.tcl 监听)
        print("  Wait 5s for ConvCore m_axi to issue AR to NoC...")
        time.sleep(5)

        # Step 5: 读 status 看 ConvCore 状态
        status = rpc.peek_csr(0, CSR_STATUS)
        print(f"  core_0 STATUS = 0x{status:08x}")

    print("Done.")


if __name__ == '__main__':
    sys.exit(main())
