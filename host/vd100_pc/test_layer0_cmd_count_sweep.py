#!/usr/bin/env python3
"""
Layer 0 cmd_count 二分搜索: 找 axi_dm + axi_noc 多 cmd 死锁转折点.

策略:
- 复用 test_layer0_cmd_count_1.py 的 patch 流程, 把 IDMA/ODMA_CMD_COUNT
  patch 成不同值 (1/2/4/8/16/32/64/128/240).
- 跑每个 N: 看 layer_done 是否 set, ODMA OFM 是否写入, ODMA dispatcher state.
- 输出表格便于看到 N=多少卡死.

ODMA SG cmd 的 last_cmd bit 也要相应 patch (driver 默认只在 cmd[239] set).

注意: N > 1 时, OFM 数据可能错乱 (mac bypass + cmd 数跟 layer cfg 不一致),
  但只关心 layer_done + dispatcher state, 不验数据正确性.
"""
import os, sys, struct, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))
from vd100_rpc import Vd100Rpc

HOST = '169.254.111.10'
PORT = 5000

ADDR_CTRL          = 0x000
ADDR_STATUS        = 0x004
ADDR_SEQ_DBG       = 0x008
ADDR_ODMA_CMD_COUNT= 0x1E8
ADDR_IDMA_CMD_COUNT= 0x1DC

CORE_BASES = [0x10A00000, 0x11A00000, 0x12A00000]
ODMA_SG_BASES = [0x10C00000, 0x11C00000, 0x12C00000]
IDMA_SG_BASES = [0x10B00000, 0x11B00000, 0x12B00000]
N_DESCS = 63   # layer 0 desc count

def patch_desc_cmd_count(rpc, desc_base, n_descs, target_addr, new_val):
    """扫 desc list 找 CFG_WRITE 写 target_addr 的 desc, patch cfg_data = new_val"""
    data = rpc.read_ddr(desc_base, n_descs * 32)
    patched = 0
    for i in range(n_descs):
        off = i * 32
        word0 = struct.unpack('<I', data[off:off+4])[0]
        d_type = word0 & 0xF
        d_addr = (word0 >> 4) & 0xFFF
        if d_type == 0x3 and d_addr == target_addr:
            new_word1 = struct.pack('<I', new_val)
            rpc.load_ddr(desc_base + off + 4, new_word1)
            patched += 1
    return patched

def patch_last_cmd_bit(rpc, sg_base, cmd_idx_to_mark):
    """SG cmd[cmd_idx_to_mark] word1 bit 23 = 1 (last_cmd flag)"""
    cmd = rpc.read_ddr(sg_base + cmd_idx_to_mark * 32, 32)
    word1 = struct.unpack('<I', cmd[4:8])[0]
    new_word1 = word1 | (1 << 23)
    rpc.load_ddr(sg_base + cmd_idx_to_mark * 32 + 4, struct.pack('<I', new_word1))
    return word1, new_word1

def run_one(rpc, N: int, deploy_run: bool):
    """Run layer 0 with CMD_COUNT=N, return (layer_done, status, seq_dbg, ofm_nz)"""
    if deploy_run:
        case_dir = r"C:/_Project/FLUX_CNN/sim/tb_smc/cases/vd100_resnet11_n3"
        cmd = [
            r"C:/_Project/FLUX_CNN/toolchain/.venv/Scripts/python.exe",
            r"C:/_Project/FLUX_CNN/host/vd100_pc/deploy_smc_case.py",
            "--case-dir", case_dir, "--vd100-ip", HOST, "--port", str(PORT), "--skip-run"
        ]
        subprocess.run(cmd, check=False, timeout=120)
        rpc.connect()

    # patch desc list IDMA/ODMA_CMD_COUNT = N
    for c, base in enumerate(CORE_BASES):
        patch_desc_cmd_count(rpc, base, N_DESCS, 0x1DC, N)
        patch_desc_cmd_count(rpc, base, N_DESCS, 0x1E8, N)

    # patch SG cmd[N-1] last_cmd bit (driver 只在 cmd[239] set, 我们改成在 N-1)
    # 同时清除原 cmd[239] 的 last_cmd (避免如果 dispatcher 用 last_cmd 提前终止)
    for c in range(3):
        # ODMA
        patch_last_cmd_bit(rpc, ODMA_SG_BASES[c], N - 1)
        # IDMA (per-row cmd, IDMA 没那么严格 last_cmd, 但安全起见)
        patch_last_cmd_bit(rpc, IDMA_SG_BASES[c], N - 1)

    # start_dfe 3 cores
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 4)
    time.sleep(0.5)

    # start_layer + poll 10s
    for c in range(3):
        rpc.poke_csr(c, ADDR_CTRL, 1 << 5)

    layer_done = False
    for t in range(100):
        time.sleep(0.1)
        st0 = rpc.peek_csr(0, ADDR_STATUS)
        if st0 & (1 << 11):
            layer_done = True
            break

    st0  = rpc.peek_csr(0, ADDR_STATUS)
    sd0  = rpc.peek_csr(0, ADDR_SEQ_DBG)
    data = rpc.read_ddr(0x10D80000, 4096)
    nz = sum(1 for b in data if b != 0)
    return layer_done, st0, sd0, nz, t * 0.1

def main():
    print("=== layer 0 cmd_count sweep ===")
    rpc = Vd100Rpc(HOST, PORT)
    rpc.connect()

    # 顺序: 1 → 2 → 4 → 8 → 16 → 32 → 64 → 128 → 240
    # 找到第一个 fail 的 N 就停 (二分定位 vendor 转折)
    sweep_n = [1, 2, 4, 8, 16, 32, 64, 128, 240]
    print(f"{'N':>4}  {'pass':>5}  {'STATUS':>10}  {'seq':>4}  {'fifo':>5}  {'oSG':>4}  {'iSG':>4}  {'ofm_nz':>7}  {'tend':>6}")

    for i, N in enumerate(sweep_n):
        deploy = (i == 0)   # 第一次跑 deploy 全部数据, 后续只 patch
        layer_done, st0, sd0, nz, tend = run_one(rpc, N, deploy_run=deploy)
        seq = sd0 & 0xF
        fifo = (sd0 >> 4) & 0xF
        odma_st = (sd0 >> 8) & 0xF
        idma_st = (sd0 >> 12) & 0xF
        tag = 'PASS' if layer_done else 'FAIL'
        print(f"  {N:>2}  {tag:>5}  0x{st0:08x}  {seq:>4}  {fifo:>5}  {odma_st:>4}  {idma_st:>4}  {nz:>5}/4K  {tend:>5.1f}s")
        if not layer_done:
            print(f"  → vendor 边界在 N={N} (前一档 PASS)")
            break

if __name__ == '__main__':
    main()
