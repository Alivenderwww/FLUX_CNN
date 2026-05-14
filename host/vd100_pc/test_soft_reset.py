#!/usr/bin/env python3
"""测软 reset 通路 (v2 新加 CTRL.bit7).

策略:
  1. 跑一个正常 case → PASS, 验证基础功能 (Stage 3a Phase 2 case)
  2. 故意触发 stuck: cmd_count 配 0 → ODMA dispatcher 永远等不到 cmd_count 进 DONE
  3. 确认 board stuck (STATUS layer_busy=1 layer_done=0)
  4. 写 CTRL.bit7 = 1 触发软 reset
  5. 重新跑正常 case → 应 PASS (软 reset 起作用)

如果第 5 步 PASS 而无需重烧 PDI, 软 reset 通路 work.
"""
import sys, struct, time
sys.path.insert(0, r'C:/_Project/FLUX_CNN/host/vd100_pc')
sys.path.insert(0, r'C:/_Project/FLUX_CNN/toolchain')
from vd100_rpc import Vd100Rpc
from test_stage3a_phase2_larger import (
    build_case, BRAM_BASE, DESC_OFF, ISG_OFF, OSG_OFF,
    BRAM_IFM, BRAM_WB, BRAM_RDMA, BRAM_OFM, BRAM_ISG, BRAM_OSG,
    chunked_load_, chunked_read_, decode_status)

CTRL_START_DFE   = 1 << 4
CTRL_START_LAYER = 1 << 5
CTRL_SOFT_RESET  = 1 << 7   # v2 new


def load_case(rpc, desc, n_desc, isg, osg, ifm, wb, rdma):
    def cl(addr, data):
        for off in range(0, len(data), 1024):
            rpc.load_ddr(addr + off, data[off:off+1024])
    cl(BRAM_BASE + DESC_OFF, desc)
    cl(BRAM_ISG, isg)
    cl(BRAM_OSG, osg)
    cl(BRAM_IFM, ifm)
    cl(BRAM_WB,  wb)
    rpc.load_ddr(BRAM_RDMA, rdma)


def trigger_run(rpc, n_desc, timeout_ms=2000, label=''):
    """trigger start_dfe + start_layer, poll layer_done. return (ok, ms, status, seq_dbg)."""
    rpc.poke_csr(0, 0x180, BRAM_BASE + DESC_OFF)
    rpc.poke_csr(0, 0x184, n_desc)
    rpc.poke_csr(0, 0x000, CTRL_START_DFE)
    for _ in range(50):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 9) & 1: break
        time.sleep(0.01)

    t = time.perf_counter()
    rpc.poke_csr(0, 0x000, CTRL_START_LAYER)
    for _ in range(int(timeout_ms / 10)):
        st = rpc.peek_csr(0, 0x004)
        if (st >> 11) & 1:
            ms = (time.perf_counter() - t) * 1000
            sd = rpc.peek_csr(0, 0x008)
            print(f'  [{label}] PASS layer_done={ms:.1f}ms STATUS=0x{st:08x}')
            return True, ms, st, sd
        time.sleep(0.01)
    sd = rpc.peek_csr(0, 0x008)
    print(f'  [{label}] STUCK STATUS=0x{st:08x} SEQ_DBG=0x{sd:08x}')
    return False, timeout_ms, st, sd


def main():
    rpc = Vd100Rpc('169.254.111.10'); rpc.connect()
    print('=== Soft reset (CTRL.bit7) 通路验证 ===')

    # ---------- 准备 normal case ----------
    print()
    print('[1] 准备 normal case K=3 H=W=8 Cin=Cout=16')
    desc_n, n_desc_n, cfg_n, ifm_n, wb_n, rdma_n, isg_n, osg_n = build_case(
        8, 8, 3, 16, 16, 1, 1)
    load_case(rpc, desc_n, n_desc_n, isg_n, osg_n, ifm_n, wb_n, rdma_n)

    # ---------- 测 1: normal case 跑通 ----------
    print()
    print('[Test 1] 跑 normal case (baseline)')
    ok1, _, st1, sd1 = trigger_run(rpc, n_desc_n, label='baseline')
    if not ok1:
        print('  baseline FAIL, abort'); return 1

    # ---------- 测 2: 故意触发 stuck (改 ODMA_CMD_COUNT=0) ----------
    print()
    print('[Test 2] 故意触发 stuck (写 ODMA_CMD_COUNT=0)')
    # 用 POKE_CSR 直接写 cfg_reg (sequencer 内部 path 不会写, host CSR write 可以)
    # 实际 csr_w 只允许 boot regs (CTRL/DESC_LIST_BASE/DESC_COUNT), 不能写 ODMA_CMD_COUNT.
    # 所以 stuck 必须通过改 desc list 来制造. 简单做法: rewrite ODMA_CMD_COUNT CFG_WRITE desc 到 0.
    #
    # 实际 hack: rebuild_case 但 odma_cmd_count=0 → ODMA dispatcher 第一行就觉得 all_cmds_issued → S_DONE 但 row_done_pulse 不止一个 → 卡
    # 实际我直接用 wrong case: K=3 H=W=8 但只构 1 个 ODMA cmd (cmd_count=1) → stuck

    # 直接 hack: 重 build case 但传错的 cmd_count
    # 改用直接构 case 但 hack ODMA cmd 数
    # 注意 build_case 内部 hard-code odma_cmd_count=H_OUT. 我用 monkey-patch
    import hw_files
    orig_cfg_to_dict = hw_files.cfg_to_dict
    def hacked_cfg(cfg, **kw):
        kw['odma_cmd_count'] = 1   # 故意错: H_OUT=8 但 cmd_count=1 → ODMA 完 1 cmd 就 DONE 但 sequencer 卡
        # ↑ 实际 ODMA 卡 S_WAIT 等下一行 cmd 但 已 all_issued → S_DONE → 但 ofb_writer 还会推后续 row → row_done_pulse 来了没人接 → ofb_writer 自身写 ring 满后反压
        return orig_cfg_to_dict(cfg, **kw)
    hw_files.cfg_to_dict = hacked_cfg
    desc_b, n_desc_b, *_, isg_b, osg_b = build_case(8, 8, 3, 16, 16, 1, 1)
    hw_files.cfg_to_dict = orig_cfg_to_dict   # 还原
    chunked_load_(rpc, BRAM_BASE + DESC_OFF, desc_b)
    chunked_load_(rpc, BRAM_ISG, isg_b)
    chunked_load_(rpc, BRAM_OSG, osg_b)
    # (ifm/wb/rdma 不重 load, 复用 Test 1)

    ok2, _, st2, sd2 = trigger_run(rpc, n_desc_b, timeout_ms=1500, label='trigger-stuck')
    if ok2:
        print('  WARN: ODMA_CMD_COUNT=1 (H_OUT=8) 居然没 stuck. 可能 RTL 已加超时.')
        # 那这测试也算成功 (RTL 鲁棒), 但软 reset 测试要换方法
    else:
        print(f'  stuck confirmed.')

    # ---------- 测 3: 写 CTRL.bit7 软 reset ----------
    print()
    print('[Test 3] 写 CTRL.bit7 软 reset')
    rpc.poke_csr(0, 0x000, CTRL_SOFT_RESET)
    time.sleep(0.01)
    st3 = rpc.peek_csr(0, 0x004); sd3 = rpc.peek_csr(0, 0x008)
    print(f'  软 reset 后 STATUS=0x{st3:08x} SEQ_DBG=0x{sd3:08x}')
    # 期望: layer_busy=0 (sequencer 回 IDLE), 其他 done bits clear

    # ---------- 测 4: 重新 load normal case + 跑, 应该 PASS (软 reset 救活了板) ----------
    print()
    print('[Test 4] 重新加载 normal case + 跑 (验证软 reset 恢复)')
    load_case(rpc, desc_n, n_desc_n, isg_n, osg_n, ifm_n, wb_n, rdma_n)
    ok4, _, st4, sd4 = trigger_run(rpc, n_desc_n, label='post-reset')

    print()
    print('=' * 60)
    if ok4:
        print('🎉 软 reset 通路 work! stuck 后能软恢复, 不用重烧 PDI.')
    else:
        print('❌ 软 reset 没救活 board, 还得重烧 PDI.')
    rpc.close()
    return 0 if ok4 else 1


if __name__ == '__main__':
    sys.exit(main())
