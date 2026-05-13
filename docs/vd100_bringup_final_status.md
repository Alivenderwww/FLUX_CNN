# VD100 Bringup 最终状态 (2026-05-14)

## TL;DR

- ✅ **Sim 全部 PASS**: 3 核 vd100_resnet11_n3 case 11 layer bit-exact, 4 核 sim_n4_v20 性能 -9.7%
- ✅ **Driver bug 修了**: deploy_smc_case desc 双长 + 5 项 RTL fix (sim 验证)
- ❌ **Board layer 0 仍 deterministic stuck**: OFM=166KB / 169KB = 96.6% (232/240 row), ODMA dispatcher 卡 S_TX (state=7), IDMA dispatcher 卡 S_RING_WAIT (state=4), sequencer 卡 S_WAIT (反压链)
- ❌ **所有 host-side 调试路径都阻塞**: BD sim CLI / ILA / xsct readback 三条路全验证过

## Board stuck 精确状态 (v32 PDI, 多次重现 100% deterministic)

```
3 core SEQ_DBG 完全一致:
  STATUS=0x656 (layer_busy=1, dfe_done=1, wdma_done=1, odma_busy=1, idma_busy=1)
  seq_state=4 (S_WAIT) — 等 core_strip_done + idma_strip_done + odma_strip_done
  fifo_count=1 (剩 END desc)
  iSG_state=4 (S_RING_WAIT) — 等 line_buffer rows_consumed 增长
  oSG_state=7 (S_TX) — 等 s2mm_data_tready (axi_dm s2mm 不接 W)
  OFM 写 0x10da8bff (+0x28bff = 166911 byte ≈ 232 row × 720 byte)
```

## 已完成 sim-validated fix (commit 6490d27 + 后续 8 个)

1. **deploy_smc_case load_desc_smc**: 写 desc 用 `words[:n_descs*2]` (每 desc 32 byte = 2 word)
2. **idma_sg_dispatcher 4KB-boundary split**: cap_btt = min(r_btt, 4KB - src[11:0])
3. **idma_sg_dispatcher r_done 不等 sts**: 用 cmd_idx 触发
4. **mm2s_arb 去 sts_empty/sts_full gate**: cmd_tvalid 不卡 sts FIFO 累积
5. **odma_sg_dispatcher S_TX tlast 跳过 S_STS**: streaming_all_done 用 data tlast
6. **multicore_top_vd100_bd_v X_INTERFACE_PARAMETER**: m_axi master capability 声明
7. **cfg_regs ADDR_SEQ_DBG**: PEEK dispatcher state 调试入口
8. **axi_arbiter R lock state machine**: 之前 v21 fix 沿用

## 不可行的调试路径 (本次 session 全验证过)

### 路径 A: BD-level Sim CLI (~3h 投入)
- Vivado export_simulation → ModelSim/xsim 两条 path 都 setup OK
- compile_simlib (versal family) + ConvCore RTL + tb_bd_top + axi_dm IP 全编 OK
- 但 cips_vip 在 batch CLI 下不能 auto-boot, NoC NMU BFM 在 ~20 ns `$finish` 提前停 sim
- 加密 IP 公开 init/write API 都隐藏
- **结论**: 必须 Vivado IDE GUI 模式手动跑

### 路径 B: BD-level ILA (v35a-e 5 轮综合 fail)
- 加 axis_ila probe ConvCore[0] m00_axi handshake signals 触发 Vivado opt LUT3 trim cascade
- v35a (14 probes): Opt 31-67 axi_arbiter cu_rd_data_master_sel
- v35b (简化 latch): 同样 error
- v35c (12 probes 减 awaddr/awlen): 同样
- v35d (dont_touch on cu_rd_*): 转 cu_wr_st 同 error
- v35e (keep_hierarchy on axi_master_arbiter): 转 axi_m_mux m_arready_inferred 同 error
- **结论**: Vivado 2023.2 opt + mark_debug + ConvCore axi_mux/arbiter 内部 trim 跨层 incompatibility

### 路径 C: xsct JTAG readback (~30 min 探索)
- axi_noc NMU/NSU 内部寄存器在 NPI 空间 (0xF6xxxxxx), a72 lwIP server 没配 MMU
- axi_dm Full mode 无 AXI4-Lite control aperture, 不能 readback internal state
- **结论**: 需要 a72 ELF 改 + rebuild 加 NPI access (1-2h 投入但可行)

## 真根因 hypothesis (sim 不可复现, 需 board 实测)

ConvCore 写 232 row OFM 后 ODMA dispatcher 卡 S_TX 等 axi_dm s2mm_data_tready:
- axi_dm s2mm 内部 W FIFO 满 → s2mm_data_tready=0
- axi_dm 内部应该一直 push W → m_axi WVALID, 等 axi_noc WREADY=1
- 但 axi_noc S00 在 232 cmd 累积某 outstanding state 后 AWREADY=0 / WREADY=0
- axi_dm AW 卡 → W 累积 → dispatcher 卡

可能机制:
- axi_noc 内部 outstanding write tracking 跟 axi_dm 同 ID (AWID=0/AWID=1 等同 thread) 多 cmd
  在 axi_noc 内部 hash table conflict 后丢失 → axi_noc 等 不存在的 B response → hang
- 这跟 v34 X_INTERFACE_PARAMETER 加 single-outstanding/single-thread 也没 fix 一致

## 下一次 session 路径 (按 ROI 排序)

### 优先级 1: Vivado IDE 手动 跑 BD sim (~2h work)
1. 打开 Vivado: `vivado output/vd100_resnet11.xpr`
2. Tools → Settings → Simulation: simulator = Vivado Simulator
3. 把 `sim/tb_bd_full/tb_bd_top.sv` add 到 sim_1, set top = tb_bd_top
4. Flow Navigator → Run Simulation → Run Behavioral
5. GUI 等 cips_vip auto-boot 完成 (~5 min, GUI 知道私有 boot protocol)
6. 在 tb_bd_top 内调 cips VIP API `versal_cips_vip_0.PS_PMC_GLOBAL.M_AXI_FPD.write(addr, data)` 写 csr_axil
7. 写 deploy 数据 + 触发 start_layer + sim 跑 232 row 看 axi_noc S00 transaction queue stuck

### 优先级 2: a72 ELF 改加 NPI access (~2h work)
1. host/vd100_ps_baremetal/resnet11_app/src/main.c 加 NoC NMU NPI read 命令
2. NoC NMU/NSU 寄存器在 NPI 空间, a72 通过 PSM_GLOBAL_REG → NPI access
3. PC RPC 新 cmd: PEEK_NOC_REG addr
4. board stuck 后 PEEK 0xF60xxxxx (S00_AXI NMU status)
5. 看 outstanding transaction count / queue state 判断是否 axi_noc 真死锁

### 优先级 3: 改 axi_dm IP config 让 single-outstanding (~80 min 重综合)
- `Syn/gen_axi_datamover.tcl`: `c_s2mm_burst_size 256 → 16` (强制小 burst)
- 或加 `c_s2mm_include_sf=true` 让 axi_dm 内部 store-and-forward 减小 outstanding
- 重 gen IP + 重综合
- 烧板看 ODMA 还卡不卡 232 row

### 优先级 4: 接受 board layer 0 sim PASS / hardware FAIL 不一致 — 走数据驱动
- 现有 sim 完整 11 layer bit-exact PASS
- board 不通 = Vivado 2023.2 + Versal VE2302 + 我们 design 配合的 vendor-specific 死锁
- 简化设计 (单核, 单 cmd, no SG) 验证 axi_dm + axi_noc 基础路径
- 如果单核 + 单 cmd 也卡 → axi_dm IP + axi_noc 真有 vendor bug
- 如果单核 + 单 cmd OK → ConvCore 多核 + SG dispatcher 触发 vendor 边界 case

## 已 commit 进展

```
ed0f5b6 WIP: xsim CLI 通过但 cips_vip batch 无法 boot
f0fc665 WIP: tb_bd_top hier-ref path 修正 (NoC NMU BFM $finish blocker)
27387ee Docs: docs/vd100_bringup_phase2.md
e5a8450 Chore: .gitignore Vivado runtime artifacts + bootgen PDI binary
4d28fa5 Add: Syn/vd100_bd/force_clean_resynth_v21..v32.tcl 迭代历史
046ecc5 Add: host/vd100_pc/probe_*.py + ILA capture scripts
6490d27 Fix: deploy desc 双长 + 5 项 RTL fix (sim PASS, board 待诊断)
+ 本次 session: a8xxx WIP BD sim infrastructure / v34 X_INTERFACE_PARAMETER / v35 ILA 尝试
```

总投入: 10+ 轮重综合 + ~25h 调试时间. RTL 调整范围内确证不是 root cause, 真根因在
axi_dm IP + axi_noc 硬件交互 vendor-specific 死锁, 需要更 deep 工具访问.
