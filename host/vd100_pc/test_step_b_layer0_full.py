#!/usr/bin/env python3
"""Step B: layer 0 完整 240 cmd 测试 (mac bypass 但不 patch CMD_COUNT).

验证:
- IDMA 拉全部 240 cmd → line_buffer 充足
- mac_array compute_en 串完整数据流 (bypass 下数据是 garbage, 但控制流完整)
- ofb_writer 写 240 行 OFM → ow_done 真正 fire
- ODMA s2mm 写 240 cmd 到 DDR
- sequencer 看到 core_strip_done && idma_strip_done && odma_strip_done → S_FETCH → 处理 END → layer_done bit[11]
"""
import os, sys, struct, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

HOST = '169.254.111.10'; PORT = 5000
ADDR_CTRL = 0x000; ADDR_STATUS = 0x004; ADDR_SEQ_DBG = 0x008

CORE_BASES = [0x10A00000, 0x11A00000, 0x12A00000]
N_DESCS = 63

def main():
    print("=== Step B: layer 0 完整 240 cmd 测试 ===")
    # phase 0: deploy 数据
    case_dir = r"C:/_Project/FLUX_CNN/sim/tb_smc/cases/vd100_resnet11_n3"
    print("[1/5] deploy_smc_case --skip-run")
    subprocess.run([
        r"C:/_Project/FLUX_CNN/toolchain/.venv/Scripts/python.exe",
        r"C:/_Project/FLUX_CNN/host/vd100_pc/deploy_smc_case.py",
        "--case-dir", case_dir, "--vd100-ip", HOST, "--port", str(PORT), "--skip-run"
    ], check=False, timeout=120, capture_output=True)

    rpc = Vd100Rpc(HOST, PORT); rpc.connect()

    # 清 OFM 区 (验证 ODMA 真写数据)
    OFM_BASES = [0x10D00000, 0x11D00000, 0x12D00000]
    print("[2/5] 清 OFM 区 (每 core 1MB) - 验证 ODMA 真写")
    for base in OFM_BASES:
        rpc.load_ddr(base, b'\x00' * (1024 * 1024))

    # 不 patch CMD_COUNT, 让它保留原值 240 (3 core 都跑完整 layer 0)
    print("[3/5] 不 patch CMD_COUNT, 跑完整 240 cmd")

    # explicit set DESC_LIST_BASE/COUNT (修复后必须)
    for c, base in enumerate(CORE_BASES):
        rpc.poke_csr(c, 0x180, base)
        rpc.poke_csr(c, 0x184, N_DESCS)
        b = rpc.peek_csr(c, 0x180); n = rpc.peek_csr(c, 0x184)
        print(f"  c{c} DESC_LIST_BASE=0x{b:08x} DESC_COUNT={n}")

    # start_dfe 3 cores
    print("[4/5] start_dfe + start_layer")
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 4)
    time.sleep(0.5)

    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 5)

    # poll 30s (layer 0 整 strip 估计 < 1s on FPGA, 但 mac bypass 数据流跑空可能慢)
    layer_done_mask = 0
    t_start = time.time()
    print("[5/5] poll layer_done (30s max)")
    for t in range(300):
        time.sleep(0.1)
        for c in range(3):
            if not (layer_done_mask & (1 << c)):
                st = rpc.peek_csr(c, ADDR_STATUS)
                if st & (1 << 11):
                    layer_done_mask |= (1 << c)
                    print(f"  c{c} layer_done @ t={t*0.1:.1f}s STATUS=0x{st:08x}")
        if t % 20 == 0:
            sts = [rpc.peek_csr(c, ADDR_STATUS) for c in range(3)]
            sds = [rpc.peek_csr(c, ADDR_SEQ_DBG) for c in range(3)]
            print(f"  t={t*0.1:.1f}s STATUS=[{sts[0]:08x},{sts[1]:08x},{sts[2]:08x}] "
                  f"SEQ_DBG=[{sds[0]:08x},{sds[1]:08x},{sds[2]:08x}] done_mask=0x{layer_done_mask:x}")
        if layer_done_mask == 0b111:
            print(f"  ✓ all 3 cores layer_done @ t={t*0.1:.1f}s")
            break

    # 最终状态
    print("\n=== 最终状态 ===")
    for c in range(3):
        st = rpc.peek_csr(c, ADDR_STATUS)
        sd = rpc.peek_csr(c, ADDR_SEQ_DBG)
        seq = sd & 0xF; fifo = (sd>>4)&0xF
        oSG = (sd>>8)&0xF; iSG = (sd>>12)&0xF
        print(f"  c{c} STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x} (seq={seq} fifo={fifo} oSG={oSG} iSG={iSG})")

    # OFM 区扫描
    print("\n=== OFM 区扫描 (验证 ODMA 写了 240 行 garbage 数据) ===")
    for c, base in enumerate(OFM_BASES):
        # layer 0 OFM 大小: 240 row × 45 col × 16 (cout=16) = 172800 byte
        # 但 mac bypass 数据 garbage, 我们只看 nonzero ratio
        data = rpc.read_ddr(base, 172800)
        nz = sum(1 for b in data if b != 0)
        print(f"  c{c} OFM @ 0x{base:08x}: nonzero={nz}/172800 ({nz/172800*100:.1f}%)")

    if layer_done_mask == 0b111:
        print("\n[PASS] Step B: 3 core 都 layer_done, 完整 240 cmd 跑通")
    else:
        print(f"\n[FAIL] Step B: 仅 {bin(layer_done_mask)} core layer_done")

if __name__ == '__main__':
    main()
