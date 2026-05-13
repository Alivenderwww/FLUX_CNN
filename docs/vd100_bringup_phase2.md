# VD100 板上 bringup Phase 2 (2026-05-13)

> Phase 1 = ALINX 板基本 boot + lwIP server work (handoff 之前已完成).
> Phase 2 = SMC + NUMA ResNet11 layer 0 端到端真上跑通 (本次, 未完成).

## 一句话现状

10+ 轮重综合 (v21..v32) 累计 5 项 sim-validated fix, **sim 3 核 + 4 核 IDEAL_SMC PASS bit-exact**;
**board layer 0 仍卡 ODMA S_TX**, OFM 写到 166KB / 169KB 期望 = 96.6% 后停; 已基本排除 RTL 嫌疑,
root cause likely 在 **axi_dm IP + axi_noc S00..S02_AXI 配置** (没声明 single-outstanding / single-thread
master capability, 跟 ALINX demo 02_pl_rw_ddr 对齐缺失).

## 已 committed sim-validated fix (commit 6490d27)

1. **deploy bug (driver, 真 阶段-1 根因)**: `load_desc_smc` 写 desc 只写一半 (`words[:n_descs]`),
   每 desc 32 byte = 2 word, 应该 `words[:n_descs * 2]`. 后半 DDR 是 stale garbage, sequencer
   消化到 garbage 后看到 type=0xF (END) 或 0x2 (BARRIER) 误进 stuck. 这是 handoff 误判 "RTL fix"
   的根本来源.

2. **dispatcher 4KB-boundary split** (idma_sg_dispatcher): cap_btt = min(r_btt, 4KB - src[11:0]),
   S_DATA tlast 后 r_btt -= cap_btt, > 0 时回 S_ISSUE 拉 sub-cmd. axi_dm IP 在 sim axi_smc
   路径上跨 4KB burst 不出 tlast 卡死. board axi_noc 实测不在 dispatcher 卡 (没用上).

3. **mm2s_arb 去 sts gate**: cmd_tvalid 不再看 sts FIFO. 老 gate 在 axi_dm 偶丢 sts 时让 sts_cnt
   累积, sts_empty_w 永 0, cmd 全卡. sim 4 核 case 性能 -9.7% (196K → 177K cycles).

4. **IDMA r_done 不依 sts_drained**: r_done 改用 r_cmd_idx == cfg_cmd_count 触发, 不等 sts_fire.

5. **ODMA S_TX tlast 跳过 S_STS**: streaming_all_done 用 data tlast 替代 s2mm_sts_fire. v31 PEEK
   板上看到 ODMA 卡 state=8=S_STS, 老逻辑等 axi_dm s2mm sts (板上偶不出).

## 调试入口 (commit 046ecc5)

- `cfg_regs` 加 ADDR_SEQ_DBG (0x008) RO: `{master_rvalid[3:0], master_arvalid[3:0], fifo_count[3:0], seq_state[3:0]}`.
  master_arvalid / master_rvalid 字段复用为 idma_sg / odma_sg dispatcher.state (诊断价值大于实际 AR/R 信号).
- PC 端 PEEK 脚本: `host/vd100_pc/probe_seq_dbg.py`, `probe_stuck.py`, `probe_cfg_after_dfe.py`.
- ILA capture: `host/vd100_ps_baremetal/capture_ila_dfe_phase.tcl`, `capture_ila_layer_busy_trigger.tcl`.

## board 实测 (v32 = 最新)

- Phase 1 (start_dfe): dfe 拉 63 desc 进 fifo, fifo_count=15 ✓
- Phase 2 (start_layer): STATUS=0x656 持续 stuck:
  - sequencer = **S_WAIT** (等 core_strip_done + idma_strip_done + odma_strip_done)
  - dispatcher.st = **S_TX (state=7)** (v32 fix 后跳过 S_STS, 现在卡 S_TX 拉数据给 axi_dm s2mm)
  - fifo_count=1 (剩 END desc), arv=0b111 (ODMA state), rv=0b100 (IDMA state)
- OFM 实际写: **166 KB / 169 KB 期望 = 96.6%** (跑 232/240 行后停)
- 持续 stuck 15s+, OFM 不增长 = 真死锁, 不是慢

## 关键诊断 (v31 之前的 PEEK)

- 板上 ConvCore wdma_done=1 (拉 weight 1024 byte 成功) → **axi_dm mm2s 单 cmd work OK**
- 板上 ODMA 跑了大部分 cmd (232/240 cmd 完成 OFM 写) → axi_dm s2mm 也部分 work
- 但 ODMA cmd[232+] 卡 → **axi_dm s2mm 在某种状态下 W 通道死锁**

## 真根因 hypothesis (未验证, 下次 session 验证)

**axi_noc S00..S02_AXI 缺 master capability declaration** — 跟 ALINX demo 02_pl_rw_ddr 对比:

| Property | Demo (work) | 我们 (stuck) |
|---|---|---|
| `NUM_WRITE_OUTSTANDING` | 1 | 默认 (likely > 1) |
| `NUM_WRITE_THREADS` | 1 | 默认 |
| `NUM_READ_OUTSTANDING` | 1 | 默认 |
| `NUM_READ_THREADS` | 1 | 默认 |
| `ID_WIDTH` | 2 | 默认 |
| `READ_WRITE_MODE` | READ_WRITE | 默认 |

axi_noc 期望 multi-thread master 时 axi_dm IP 用同 ID (axi_dm 内部 ARID=0/AWID=0 same thread),
axi_noc 路由 R/B response 可能错位 → axi_dm 内部 cmd 卡.

## 下次 session 应该做

### 优先级 1: BD axi_noc S00..S02_AXI 加 master capability (v33)

修 `Syn/vd100_bd/pl_config.tcl` line 55-63:
```tcl
for {set i 0} {$i < 3} {incr i} {
  set port [format "S%02d_AXI" $i]
  set_property -dict [list \
    CONFIG.CONNECTIONS [format "MC_0 \{read_bw \{500\} write_bw \{500\} ...\}"] \
    CONFIG.CATEGORY {pl} \
    CONFIG.DATA_WIDTH {128} \
    CONFIG.NUM_WRITE_OUTSTANDING {1} \
    CONFIG.NUM_WRITE_THREADS {1} \
    CONFIG.NUM_READ_OUTSTANDING {1} \
    CONFIG.NUM_READ_THREADS {1} \
    CONFIG.READ_WRITE_MODE {READ_WRITE} \
  ] [get_bd_intf_pins /axi_noc_0/$port]
}
```
然后重综合 (~80 min) + 烧板验证. 如果是真根因, board layer 0 直接 PASS.

### 优先级 2: 如果 v33 没解, 加 ILA 抓 axi_dm m_axi 出口

ila_dbg 已删 (LUT 100% 紧). 需要换策略:
- 删 sequencer dbg port 释放 LUT
- 只抓 1 ConvCore[0] 的 axi_dm m_axi 出口 (AW + W + B + AR + R)
- 用 layer_busy=1 触发, 抓 stuck 瞬间

### 优先级 3: 如果 ILA 都看不出 axi 协议错, 试 axi_dm IP 更保守 config

`Syn/gen_axi_datamover.tcl`:
- `CONFIG.c_s2mm_burst_size 256 → 16` (强制小 burst 避跨 4KB)
- `CONFIG.c_addr_pipe_depth` 调小
- 重 gen IP + 重综合

### 优先级 4: 自下而上 minimum bring-up (用户建议方法论)

实施一个 minimum test 验证每层独立:
- L1: ConvCore axi_dm 直接 read 1 word DDR (不走 dispatcher) — 通过 cfg_regs trigger
- L3: idma_ctrl (SG_MODE=0) 单 cmd 拉 1 row IFM
- L5: odma_sg 单 cmd 写 1 row OFM
- L6: single core layer 0
- L7: 3 core SMC layer 0

## 不要做的事

- ❌ 不要再用 sim axi_smc IP path 验证 board RTL: vsim 处理 axi_smc IP sim model 的 vendor bug 让 sim
  crash (Signal 11), 这条路径不可用. **sim IDEAL_SMC 是唯一 sim 验证途径**.
- ❌ 不要再盲改 RTL: 当前 5 项 fix 都是 sim PASS 但 board 行为 0 变化, 强证 root cause 不在 RTL.
  下次 session 应该从 BD 配置 + ILA 实测 入手, 不是改 RTL.
- ❌ 不要因为 axi_dm IP 是黑盒就放弃: 可以改 IP config (gen_axi_datamover.tcl) + 重 gen.
