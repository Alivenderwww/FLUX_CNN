#!/usr/bin/env python3
"""
Layer 0 但 ODMA_CMD_COUNT 强制改 1: 复用 driver 生成的完整 cfg (所有 mac/ofb dependency),
但只跑 1 个 ODMA cmd. 验证 axi_dm + axi_noc 单 cmd 是否能写 DDR.

策略:
1. deploy_smc_case 加载 layer 0 全数据 (跟 board stuck 时一样)
2. 在 desc list DDR 上 patch ODMA_CMD_COUNT 那条 CFG_WRITE 的 cfg_data 改 1
3. 同样 patch IDMA_CMD_COUNT 改 1 (避免 IDMA 拉 240 row 数据)
4. start_dfe → start_layer
5. 看 ODMA 是否能完成 1 cmd

结果:
- layer_done + DDR 写入 → axi_dm + axi_noc 单 cmd 路径 OK, vendor bug 触发要 N>1 cmd
- 同 stuck → axi_dm 单 cmd 都不行
"""
import os, sys, struct, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

# 直接复用 probe_seq_dbg 的 deploy 流程
HOST = '169.254.111.10'
PORT = 5000

ADDR_CTRL          = 0x000
ADDR_STATUS        = 0x004
ADDR_SEQ_DBG       = 0x008
ADDR_ODMA_CMD_COUNT= 0x1E8
ADDR_IDMA_CMD_COUNT= 0x1DC

def patch_desc_cmd_count(rpc: Vd100Rpc, core_id: int, desc_base: int, n_descs: int, target_addr: int, new_val: int):
    """扫 desc list 找 CFG_WRITE 写 target_addr 的 desc, patch cfg_data = new_val"""
    # 读全 desc list
    data = rpc.read_ddr(desc_base, n_descs * 32)
    patched = 0
    for i in range(n_descs):
        off = i * 32
        word0 = struct.unpack('<I', data[off:off+4])[0]
        word1 = struct.unpack('<I', data[off+4:off+8])[0]
        d_type = word0 & 0xF
        d_addr = (word0 >> 4) & 0xFFF
        if d_type == 0x3 and d_addr == target_addr:
            print(f"  core{core_id} desc[{i}] CFG_WRITE addr=0x{d_addr:03x} old={word1} → new={new_val}")
            # write back new word1
            new_word1 = struct.pack('<I', new_val)
            rpc.load_ddr(desc_base + off + 4, new_word1)
            patched += 1
    return patched

def main():
    print("=== layer 0 with CMD_COUNT=1 force test ===")
    # Phase 0: 复用 deploy_smc_case (跑一次完整 deploy)
    case_dir = r"C:/_Project/FLUX_CNN/sim/tb_smc/cases/vd100_resnet11_n3"
    print(f"[1/6] Run deploy_smc_case for layer 0 only")
    cmd = [
        r"C:/_Project/FLUX_CNN/toolchain/.venv/Scripts/python.exe",
        r"C:/_Project/FLUX_CNN/host/vd100_pc/deploy_smc_case.py",
        "--case-dir", case_dir, "--vd100-ip", HOST, "--port", str(PORT), "--skip-run"
    ]
    subprocess.run(cmd, check=False, timeout=120)

    rpc = Vd100Rpc(HOST, PORT); rpc.connect()

    # Read meta (deploy_smc_case 保存到 active_case.txt 或类似)
    # Hardcode 3 core base + desc count for layer 0
    core_bases = [0x10A00000, 0x11A00000, 0x12A00000]
    n_descs = 63   # layer 0 desc count

    print("[2/6] Patch desc list — IDMA/ODMA_CMD_COUNT → 1")
    for c, base in enumerate(core_bases):
        p1 = patch_desc_cmd_count(rpc, c, base, n_descs, 0x1DC, 1)
        p2 = patch_desc_cmd_count(rpc, c, base, n_descs, 0x1E8, 1)
        print(f"  core{c}: patched IDMA_CMD_COUNT={p1}x, ODMA_CMD_COUNT={p2}x")

    # Also patch ODMA SG cmd[0] last_cmd bit (word1 bit 23) — driver 只在 cmd[239] set last_cmd
    # cmd_count=1 后 ODMA dispatcher 需要 r_last_cmd 触发 streaming_all_done
    print("[2b/6] Patch ODMA SG cmd[0] last_cmd bit + IDMA SG cmd[0] last_cmd bit")
    odma_sg_bases = [0x10C00000, 0x11C00000, 0x12C00000]
    idma_sg_bases = [0x10B00000, 0x11B00000, 0x12B00000]
    for c in range(3):
        for tag, base in [('ODMA', odma_sg_bases[c]), ('IDMA', idma_sg_bases[c])]:
            cmd0 = rpc.read_ddr(base, 32)
            word1 = struct.unpack('<I', cmd0[4:8])[0]
            new_word1 = word1 | (1 << 23)   # set last_cmd
            rpc.load_ddr(base + 4, struct.pack('<I', new_word1))
            print(f"  c{c} {tag} cmd[0] word1: 0x{word1:08x} → 0x{new_word1:08x}")

    print("[3/6] start_dfe 3 cores")
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 4)
    time.sleep(0.5)
    for c in range(3):
        st = rpc.peek_csr(c, ADDR_STATUS)
        print(f"  c{c} STATUS after dfe = 0x{st:08x}")

    print("[4/6] PEEK CMD_COUNT after dfe pulls desc")
    for c in range(3):
        cc = rpc.peek_csr(c, ADDR_ODMA_CMD_COUNT)
        ic = rpc.peek_csr(c, ADDR_IDMA_CMD_COUNT)
        print(f"  c{c} ODMA_CMD_COUNT={cc}, IDMA_CMD_COUNT={ic}")

    print("[5/6] start_layer + poll 10s")
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 5)
    for t in range(100):
        time.sleep(0.1)
        st0 = rpc.peek_csr(0, ADDR_STATUS)
        sd0 = rpc.peek_csr(0, ADDR_SEQ_DBG)
        if t % 5 == 0:
            seq = sd0 & 0xF
            fifo = (sd0 >> 4) & 0xF
            odma_st = (sd0 >> 8) & 0xF
            idma_st = (sd0 >> 12) & 0xF
            print(f"  t={t*0.1:.1f}s c0 STATUS=0x{st0:08x} seq={seq} fifo={fifo} odma_st={odma_st} idma_st={idma_st}")
        if st0 & (1 << 11):
            print(f"  [PASS] layer_done @ t={t*0.1:.1f}s")
            break

    print("[6/6] OFM scan at 0x10D80000 (layer 0 OFM area)")
    data = rpc.read_ddr(0x10D80000, 4096)
    nz = sum(1 for b in data if b != 0)
    print(f"  4KB nonzero: {nz}/4096")

    print("\n=== 结论 ===")
    if st0 & (1 << 11):
        print("[PASS] cmd_count=1 layer 0 完成 → axi_dm + axi_noc 单 cmd 路径 OK")
        print("  下一步: 二分搜索 cmd_count, 找 stuck 转折点")
    else:
        print("[FAIL] cmd_count=1 layer 0 仍 stuck → axi_dm + axi_noc 单 cmd 都死锁")
        print("  hardware bug 锁定")

if __name__ == '__main__':
    main()
