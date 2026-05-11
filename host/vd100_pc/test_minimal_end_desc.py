"""
test_minimal_end_desc.py — 最小 sequencer/dfe 链路测试

只构造 1 条 END descriptor (type=0xF) 写入 DDR. 然后:
  1. POKE DESC_LIST_BASE / DESC_COUNT
  2. POKE CTRL[4]=1 (start_dfe), poll dfe_done
  3. POKE CTRL[5]=1 (start_layer), poll layer_done sticky

预期: sequencer 走 S_IDLE → S_FETCH → 解析 END → S_END (wdma_done && rdma_done && idma_strip_done && odma_strip_done 都是 1, 因为 r_is_first=0 默认 wdma 没启动) → S_IDLE → layer_done sticky.

如果 layer_done=1 → sequencer FSM 完全 OK, 那 layer 0 卡住是因为 desc[0..60] CFG_WRITE 处理或 desc[61] CONV 路径有 bug.
如果 layer_done=0 → sequencer 自身有问题或 desc 数据没真到 fifo.
"""
import os, sys, time, struct
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '.')
from vd100_rpc import Vd100Rpc

# END descriptor: word0 = 0x0000_000F (type=0xF, no flags), word 1..7 = 0
end_desc_bytes = bytearray(32)
end_desc_bytes[0:4] = struct.pack('<I', 0x0000000F)

# DDR 地址 (任意, 选一个空闲区, 不跟 layer 数据冲突)
DESC_BASE = 0x20000000

with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()
    print(f'=== Write 1 END descriptor to DDR 0x{DESC_BASE:08x} ===')
    rpc.load_ddr(DESC_BASE, bytes(end_desc_bytes))

    # 验证写对了
    rb = rpc.read_ddr(DESC_BASE, 32)
    print(f'  wrote = {end_desc_bytes.hex()}')
    print(f'  read  = {rb.hex()}')
    if rb != bytes(end_desc_bytes):
        print('  *** MISMATCH! DDR write/read broken ***')
        sys.exit(1)
    print('  OK DDR LOAD/READ OK')

    print()
    print('=== Test on core 0 only ===')
    core = 0

    # 清掉 sticky (写 start_layer 自清, 但当前可能没 sticky 反正写了无害)
    print(f'  STATUS before = 0x{rpc.peek_csr(core, 0x004):08x}')

    # phase 1: start_dfe
    print(f'--- phase 1: write DESC_LIST_BASE/COUNT + start_dfe ---')
    rpc.poke_csr(core, 0x180, DESC_BASE)
    rpc.poke_csr(core, 0x184, 1)
    rpc.poke_csr(core, 0x000, 0x10)  # CTRL[4]=1

    # poll dfe_done
    for i in range(50):
        time.sleep(0.02)
        s = rpc.peek_csr(core, 0x004)
        dfe_done = (s >> 9) & 1
        if dfe_done:
            print(f'  t={(i+1)*0.02:.2f}s STATUS=0x{s:08x} dfe_done=1 OK')
            break
    else:
        print(f'  *** dfe_done TIMEOUT, STATUS=0x{rpc.peek_csr(core, 0x004):08x} ***')
        sys.exit(1)

    # phase 2: start_layer
    print(f'--- phase 2: start_layer (CTRL[5]=1) ---')
    rpc.poke_csr(core, 0x000, 0x20)

    # poll layer_done sticky (STATUS[0])
    for i in range(50):
        time.sleep(0.02)
        s = rpc.peek_csr(core, 0x004)
        sticky = s & 1
        if sticky:
            print(f'  t={(i+1)*0.02:.2f}s STATUS=0x{s:08x} layer_done sticky=1 OKOKOK')
            print()
            print('  *** SEQUENCER FSM works! ***')
            break
        if i % 5 == 0:
            print(f'  t={(i+1)*0.02:.2f}s STATUS=0x{s:08x}')
    else:
        s = rpc.peek_csr(core, 0x004)
        print(f'  *** TIMEOUT, STATUS=0x{s:08x} ***')
        if s & (1 << 10):
            print('  -> layer_busy=1 stuck, sequencer FSM blocked (need ILA波)')
        sys.exit(1)
