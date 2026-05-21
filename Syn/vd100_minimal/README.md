# VD100 板级 bring-up 工作流

Versal AI Edge (xcve2302) board bring-up + 调试指南。N=1 单核已经端到端 PASS bit-exact。

## 目录

- [项目结构](#项目结构)
- [完整工作流（从 RTL 到烧板）](#完整工作流)
- [脚本清单](#脚本清单)
- [调试方法论](#调试方法论)
- [已知 bug 与 fix](#已知-bug-与-fix)
- [踩坑总结](#踩坑总结)

---

## 项目结构

```
Syn/vd100_minimal/                ← 综合 + BD + bitstream 烧录工程
├── output/ps_hello.xpr           ← Vivado 工程文件 (BD: design_1)
├── run_synth_impl.tcl            ← 综合 + 实现 + 写 PDI 一条龙
├── arm_ila_and_wait.tcl          ← ILA 命令行触发 + 抓波形
├── vd100_minimal_with_elf.bif    ← bootgen 配置 (PDI + A72 ELF)
├── vd100_minimal_with_elf.pdi    ← 最终烧板 PDI (含 ConvCore bitstream + A72 lwIP RPC server)
└── migrate_to_n4.tcl             ← BD 改造脚本 (单核→N=3 多核, 参考)

RTL/Versal/                       ← Board 专用 RTL wrapper
├── multicore_top_n4.sv           ← SV wrapper (NUM_CORES 参数化, 默认 1)
├── multicore_top_n4_v.v          ← Verilog wrapper for BD (gen_multicore_top_n4_v.py 自动生成)
└── gen_multicore_top_n4_v.py     ← Wrapper generator

host/vd100_ps_baremetal/          ← PS A72 baremetal ELF (lwIP RPC server)
├── workspace2/resnet11_app/
│   ├── src/echo.c                ← RPC handler (CMD_LOAD_DDR / READ_DDR / RUN_LAYERS / PEEK_CSR / POKE_CSR)
│   └── Debug/resnet11_app.elf    ← 编译产出 (bootgen 打进 PDI)
└── program_pdi_via_vivado.tcl    ← Vivado HW Manager 烧 PDI (JTAG)

host/vd100_pc/                    ← PC 端测试驱动 (Python)
├── vd100_rpc.py                  ← TCP RPC 客户端 (NUM_CORES=1 时)
├── deploy_smc_case.py            ← 单 case 端到端部署 (LOAD_DDR → RUN_LAYERS → READ_OFM)
├── sweep_board_cases.py          ← 多 case 批量回归 + bit-exact 比对
├── trigger_m_axi_minimal.py      ← 最小 m_axi 触发 (debug 用)
└── test_stage4_bitexact.py 等    ← 历史 stage 测试

toolchain/
├── run_multicore_chain.py        ← 编译器主入口 (--demo NAME --case_name X --n_cores 1)
├── hw_files.py                   ← 配置 derive (含 ifb_ring_words)
└── mesh_cmd.py                   ← SG cmd 生成 (IDMA / ODMA scatter-gather list)
```

---

## 完整工作流

### Phase 0：环境检查

- **Vivado**：`D:/Xilinx/Vivado/2023.2/bin/vivado.bat`
- **Vitis (bootgen + arm gcc)**：`D:/Xilinx/Vitis/2023.2/bin/{bootgen.bat,xsct.bat}`
- **ARM toolchain**：`D:/Xilinx/Vitis/2023.2/gnu/aarch64/nt/aarch64-none/bin/aarch64-none-elf-gcc`
- **ModelSim** (sim)：`C:/modeltech64_2020.4/win64/vsim.exe`
- **Python venv**：`C:/_Project/FLUX_CNN/toolchain/.venv/Scripts/python.exe`
- **Board IP**：`169.254.111.10`（USB-C 直连 RNDIS / Ethernet）

### Phase 1：改 RTL / BD 后重新综合

```bash
# 1.1 改完 RTL (multicore_top_n4.sv 之类), 重新生成 Verilog wrapper
cd C:/_Project/FLUX_CNN/RTL/Versal
python gen_multicore_top_n4_v.py        # 输出 multicore_top_n4_v.v

# 1.2 BD 改动: GUI 操作（addr map / SmartConnect / axi_noc 配置等）
#     或者 batch tcl: 见 migrate_to_n4.tcl 参考

# 1.3 综合 + 实现 + write_device_image (一条龙)
D:/Xilinx/Vivado/2023.2/bin/vivado.bat -mode batch \
    -source C:/_Project/FLUX_CNN/Syn/vd100_minimal/run_synth_impl.tcl \
    -log    C:/_Project/FLUX_CNN/Syn/vd100_minimal/rebuild.log \
    -nojournal
# 关键监控信号: 见 [调试方法论 → 监控综合]
# 时间: 30-60 min (16 jobs)
# 产物: output/ps_hello.runs/impl_1/design_1_wrapper.pdi (~3.5 MB N=1)
```

### Phase 2：bootgen 打包 PDI + A72 ELF

```bash
# 2.1 (可选) 重编 ELF: 改了 echo.c 之后必须
cd C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/workspace2/resnet11_app/Debug
PATH="/d/Xilinx/Vitis/2023.2/gnu/aarch64/nt/aarch64-none/bin:$PATH" make all

# 2.2 bootgen 合 PDI + ELF → with_elf.pdi (JTAG 烧, 不烧 SD/QSPI)
D:/Xilinx/Vitis/2023.2/bin/bootgen.bat -arch versal \
    -image C:/_Project/FLUX_CNN/Syn/vd100_minimal/vd100_minimal_with_elf.bif \
    -o     C:/_Project/FLUX_CNN/Syn/vd100_minimal/vd100_minimal_with_elf.pdi \
    -w on
```

### Phase 3：JTAG 烧 PDI

```bash
D:/Xilinx/Vivado/2023.2/bin/vivado.bat -mode batch \
    -source C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/program_pdi_via_vivado.tcl \
    -tclargs C:/_Project/FLUX_CNN/Syn/vd100_minimal/vd100_minimal_with_elf.pdi \
    -nojournal -nolog
# 成功标志: "INFO: [Labtools 27-3439] Successfully programmed PDI" + "DONE bit: HIGH"
# 板上 A72 自动启动 lwIP server, ping 169.254.111.10 应通
```

### Phase 4：编译 case + 部署测试

```bash
# 4.1 toolchain 生成 N=1 case (单层或多层)
cd C:/_Project/FLUX_CNN/toolchain
FLUX_SMC_GLOBAL_BASE=0x10000000 .venv/Scripts/python.exe run_multicore_chain.py \
    --smc --demo wslice1 --case_name vd100_wslice1_n1 --n_cores 1
# case 输出到 sim/tb_smc/cases/vd100_wslice1_n1/
# 重要 demo: wslice1 (no wrap), h32_ref (wrap), sw_h64_w32 (大 wrap), sw_h*_s2 (stride 2)

# 4.2 部署单 case
cd C:/_Project/FLUX_CNN/host/vd100_pc
.venv/Scripts/python.exe deploy_smc_case.py \
    --case-dir ../../sim/tb_smc/cases/vd100_wslice1_n1 \
    --vd100-ip 169.254.111.10
# 输出: hw cycles + OFM 比对 (调用 read_ofm_smc, 会算 bit-exact)

# 4.3 批量回归
cd C:/_Project/FLUX_CNN/host/vd100_pc
.venv/Scripts/python.exe sweep_board_cases.py --vd100-ip 169.254.111.10
# 8 个 demo 全跑 + bit-exact 比对, 输出 PASS/FAIL 表
```

---

## 脚本清单

| 用途 | 脚本 | 说明 |
|------|------|------|
| 综合+实现+PDI | `Syn/vd100_minimal/run_synth_impl.tcl` | 不动 BD, 跑 synth_1 → impl_1 → write_device_image |
| BD 改造（参考） | `Syn/vd100_minimal/migrate_to_n4.tcl` | 单核→N=3 历史脚本（现 N=1 GUI 改更直接） |
| Wrapper 生成 | `RTL/Versal/gen_multicore_top_n4_v.py` | 改 NUM_CORES → 重生成 wrapper |
| bootgen | `Syn/vd100_minimal/vd100_minimal_with_elf.bif` | PDI + ELF 合并 |
| ELF 编译 | `host/vd100_ps_baremetal/workspace2/resnet11_app/Debug/make all` | A72 baremetal |
| 烧 PDI | `host/vd100_ps_baremetal/program_pdi_via_vivado.tcl` | Vivado HW Manager |
| ILA 命令行 | `Syn/vd100_minimal/arm_ila_and_wait.tcl` | arm + 等触发 + 抓波形 |
| BD 状态 inspect | `Syn/vd100_minimal/inspect_bd.tcl` | 看 BD cells / clk / reset |
| NoC 配置 inspect | `Syn/vd100_minimal/inspect_noc.tcl` | axi_noc 当前配置 |
| 单 case 部署 | `host/vd100_pc/deploy_smc_case.py` | LOAD + RUN + READ + bit-exact |
| 多 case 回归 | `host/vd100_pc/sweep_board_cases.py` | 8+ demo 自动跑 |
| 最小 m_axi 触发 | `host/vd100_pc/trigger_m_axi_minimal.py` | START_DFE only, 不跑 layer |
| toolchain case gen | `toolchain/run_multicore_chain.py --demo NAME --n_cores 1` | 编译器入口 |

---

## 调试方法论

### 监控综合关键事件（不只看末尾日志）

跑 Vivado batch 综合时 **必须** monitor `^ERROR` 和 `^CRITICAL WARNING`，否则可能没注意资源超用 / DRC 失败：

```bash
# 启动综合 (background)
D:/Xilinx/Vivado/2023.2/bin/vivado.bat -mode batch -source run_synth_impl.tcl \
    -log rebuild.log > /dev/null 2>&1 &

# 同步 monitor (例如 Claude Code 的 Monitor 工具或 tail -f | grep)
tail -f rebuild.log | grep --line-buffered -E \
  "^ERROR|^CRITICAL WARNING|synth_1 =|impl_1 =|UTLZ-1|over-utilized|DONE\.|Failed runs"
```

关键 milestone 和异常：
- `synth_1 = synth_design Complete!` → synth 完成
- `impl_1 = write_device_image Complete!` → 实现完成出 PDI
- `[DRC UTLZ-1] over-utilized` → 资源超 (URAM/LUT/DSP 用太多, 必须减容量或减核数)
- `Failed runs(s)` → 必须看 runme.log 找原因

### 板上调试：从浅到深

#### Level 1：板活着吗？

```bash
ping -n 2 169.254.111.10
# 不通 → PDI 没烧成功 / ELF 没启 lwIP / 网线没插
```

#### Level 2：PS A72 ELF 跑起来了吗？

```python
from vd100_rpc import Vd100Rpc
with Vd100Rpc('169.254.111.10') as rpc:
    rpc.ping()                 # 0xCAFEBABE 回包
```

#### Level 3：ConvCore CSR 通吗？

```python
with Vd100Rpc('169.254.111.10') as rpc:
    print(f'STATUS = 0x{rpc.peek_csr(0, 0x004):08x}')
    # 0x000 = reset 后 idle
    # 0x200 = bit[9] dfe_done only (只跑了 START_DFE 没启 layer)
    # 0xAE1 = layer_done sticky + dfe/odma/wdma_done (完整跑过 1 layer)
```

#### Level 4：layer 卡住时看内部状态

读 `ADDR_SEQ_DBG (0x008)` debug word 看 sequencer/IDMA/ODMA 内部 FSM：
```
[7:4]=fifo_count  [3:0]=seq_state
[11:8]=odma_sg_state  [15:12]=idma_sg_state
```

例如 `0x00004914` = `idma_sg=4 (S_RING_WAIT)`, `odma_sg=9 (S_DONE)`, `seq=4 (S_WAIT)`, `fifo=1` —
说明 IDMA 卡在等 ring 空间，ODMA 已完成，sequencer 在等 IDMA。这种情况通常是 ring wrap 数据通路 bug。

CSR 关键 offset 速查：
| Offset | 名字 | 用途 |
|--------|------|------|
| 0x000 | CTRL | `0x10` START_DFE / `0x20` START_LAYER / `0x30` 同时启 |
| 0x004 | STATUS | bit 0 = layer_done sticky |
| 0x008 | SEQ_DBG | 内部 FSM debug |
| 0x100 | H_OUT | 实际配的输出 H |
| 0x108 | W_IN | 实际配的输入 W |
| 0x16C | IFB_STRIP_ROWS | IFB ring 行数 |
| 0x1A0 | IFB_RING_WORDS | IFB ring 总 word 数 = STRIP × W × cin_slices |
| 0x180 | DESC_LIST_BASE | DDR 上 desc list 起点 |
| 0x184 | DESC_COUNT | desc 个数 |

#### Level 5：用 ILA 抓信号波形

```bash
# 5.1 BD 加 axi_protocol_checker + ILA debug core (GUI 操作: Set Up Debug 标 mark_debug)
# 5.2 重综合烧板
# 5.3 命令行 arm ILA + 等触发
D:/Xilinx/Vivado/2023.2/bin/vivado.bat -mode batch \
    -source Syn/vd100_minimal/arm_ila_and_wait.tcl \
    -tclargs 2 \                # timeout 2 min (单位是分钟不是秒!)
    -nojournal -log ila_arm.log

# 5.4 同时跑 deploy_smc_case 触发 bug
# 5.5 ILA 触发后 csv 自动 dump 到 ila_capture.csv
```

ILA debug pitfalls：
- `wait_on_hw_ila -timeout N`：N 单位是 **分钟** 不是秒（90 = 90min 不是 90s）
- Set Up Debug 阶段提示"没 clock domain"的 net 全选 "Assign All Clock Domains"
- `pc_status` 是 160-bit 切多个 sub-vector（mark_debug 时可能没全标，注意补全）

### 多 case 批量验证

改 RTL 后必须跑 **多个不同 case** 看是否有 corner case bug：

```python
# host/vd100_pc/sweep_board_cases.py 覆盖维度:
DEMOS = [
    ('wslice1',         32, 32, 16, 'no wrap baseline'),        # 验整图装下
    ('sw_h64_w32',      64, 32, 16, 'wrap 2x'),                 # 验 ring wrap
    ('sw_h96_w32',      96, 32, 16, 'wrap 3x'),
    ('sw_h128_w32',    128, 32, 16, 'wrap 4x'),
    ('sw_h32_w32_s2',   32, 32, 16, 'stride 2 no wrap'),        # 验 stride 路径
    ('sw_h64_w32_s2',   64, 32, 16, 'stride 2 wrap'),
    ('h32_ref',         32, 75, 16, 'W=75 wrap'),               # 验不同 row_words
]
```

新增 case 需要：
1. 在 `toolchain/run_multicore_chain.py` 的 `argparse choices` 加 demo 名
2. 在对应 `elif args.demo == 'XXX'` 加 layer 定义
3. 在 `sweep_board_cases.py` `DEMOS` 加一行

---

## 如何找 bug 的根因（这次的实战教训）

**核心教训：不要凭猜测，做实证排查。每个 hypothesis 都做对比实验证伪。**

这次 board 跑 `h32_ref` (H=32 W=75 K=3) 在 row 5 后 OFM 全错，调了一整天才定位。经验复盘：

### 错误的猜测路径（浪费时间）

| 猜测 | 实证排查 | 结论 |
|------|----------|------|
| m_axi 不符合 AXI4 spec, NoC 拒绝 | 加 axi_protocol_checker IP + ILA, **pc_asserted 2 分钟没触发** | 协议没违规 |
| NoC 长 latency 引起 race | sim axi_slave_mem 加 AR_LATENCY=300 cycle, **仍 PASS bit-exact** | latency 不是 |
| BRAM read-during-write 行为差异 | 改 sram_model 模拟 WRITE_FIRST mode, **仍 PASS** | 不是 |
| wrapper 版本差异 (minimal vs n4) | sim 换 multicore_top_n4_v wrapper, **仍 PASS** | 不是 |
| Race / non-determinism | 重跑 5 次 OFM md5 完全一致 | deterministic, 不是 race |
| PS A72 cache flush 不完整 | RPC READ_DDR 把 IFM 读回比对原始, **bit-exact** | cache OK |

### 正确的根因排查（用户提示后才走对）

**用户提示：「查地址，我很怀疑是地址寻址问题。为什么只有一定行后的数据出错，分析在这里 ConvCore 做了什么，向 axi_noc 发了什么，与之前的没出错的地方有什么区别。」**

1. **看 IDMA SG cmd 列表 `layer00_idma_sg.hex`**
   - 每行 cmd 32 byte: `[src_addr | btt | sram_offset | reserved]`
   - 发现 cmd N (N≥STRIP) 的 `sram_offset = N × row_words` **没 mod ring_words**

2. **对比 PASS case (wslice1) vs FAIL case (h32_ref) 的 CSR cfg**
   - 唯一差异：`IFB_RING_WORDS`（1024 vs 450）
   - 1024 是 2 的幂（物理 BRAM 自然 wrap），450 不是
   - → 假设是地址 mod 而非 bit mask 的问题

3. **看 RTL line_buffer 和 toolchain 双方处理 ring**
   - `line_buffer.sv` `wrap_addr()` 函数：`if (val >= ring_words) val -= ring_words`（正确）
   - `toolchain/run_multicore_chain.py:655` mode A 路径：`sram_offset += row_words`（**漏 `% ring_words`**）
   - 对比 `mesh_cmd.py:338` cout_slice 路径：正确 `sram_offset % ifb_ring_words`
   - → **mode A 单核 SG 生成漏 mod，其他模式有，bingo**

4. **fix：一行加 `% ring_words` 即解**

### 调试方法论提炼

1. **从 host/PC 端开始向下**：ping → RPC → CSR → 内部 dbg word → ILA。**每层都验证 OK 再深入**
2. **PASS case 和 FAIL case 必须对比**：哪些 CSR 值不同？哪些数据流不同？差异是触发点
3. **每个 hypothesis 必须可证伪**：用 sim / 对照 case 做实验，不要陷在"可能是 X"的循环
4. **遇到 deterministic 且 board specific 的 bug，问"hardware 实际收到什么"**：
   - SG cmd 内容（DDR 上的实际字节）
   - CSR cfg 实际写入值（PEEK_CSR 读回来，**注意 addr 别 typo**）
   - m_axi AR/AW addr 序列（要 ILA）
   - 对照 sim 跑同 case 的等价值
5. **deterministic ≠ algorithm bug**：也可能是 toolchain/编译器算错给 hardware 的"错正确"数据
6. **覆盖度优先**：多个 case 对比比一个 case 深挖更快定位规律
7. **看 first_bad row 的精确数字**：first_bad=121 ≠ STRIP-1 规律 → 不是 ring 反压，可能是 DDR 地址边界 / IFM/OFM 重叠等
8. **直接查 DDR 地址覆盖**：算 IFM 起点+大小 vs OFM 起点+大小是否重叠。`SMC_FINAL_OFM_BASE - SMC_INPUT_BASE = N MB`，IFM size = H × W × cin × 16 byte，比一比就清楚
9. **不要乱用相似名字的信号**：`ofb_writer.row_done_pulse` ≠ `IFB row consumed`。它是 OFM 写完一行的事件，跟 IFB ring 释放语义无关。**先理解模块语义再用，别凭名字猜**

### 调试 patch1 (K=1 c=64 H=240 W=135) row 121+ 错的真实复盘

错误路径（耗时长）：
- 怀疑 ring=1 同步 race → 计划 RTL line_buffer Mode B 改造（大改）
- 误用 `ofb_writer.row_done_pulse` 替代 `evt_iss_cs_wrap`（语义错乱）

正确路径（用户提示后 2 步定位）：
- 用户提示 "ofb 跟 ifb 有啥关系" → 停止乱用信号，重新看现象
- 注意到 first_bad=121 不是 STRIP-1 → 不是 ring 反压问题
- 看 SG cmd row 120/121/122 src addr → row 122 起 src `0x10E01580` 跨过 `0x10E00000`
- 算 IFM range = `0x10D00000` ~ `0x10D00000 + 240×8640 = 0x10E95000`
- 比 OFM base = `0x10E00000` 落在 IFM range 内 → **IDMA 读 IFM row 121+ 时 OFM 已经写过这段 DDR**
- 修：SMC_FINAL_OFM_BASE 0xE→0xF（1MB→2MB 偏移）

教训：**RTL 改前先穷尽 toolchain/host 端可能性**。这次差点改 RTL 大动作（Mode B 新模式），实际 toolchain 一行 hardcode 改完事。

---

## 已知 bug 与 fix（历史）

| Bug | Root cause | Fix |
|-----|-----------|-----|
| Ring wrap 第 STRIP-1 行后 OFM 全错 | toolchain `run_multicore_chain.py:655` mode A SG cmd 漏 `% ifb_ring_words` | 加 mod |
| Layer 启动不了, 只 dfe_done=1 stuck | PS `echo.c:141` 漏写 `CTRL[5]=START_LAYER`（只写了 `CTRL[4]=START_DFE`） | `CTRL_START_DFE 0x10 → CTRL_START_BOTH 0x30`（同时触发两个 pulse） |
| ofb_strip_rows 6-bit 截断, H_OUT=64 死锁 | `ofb_writer.sv` strip_rows 6 bit, `64 & 0x3F = 0` | 6→16 bit (跨 cfg_regs/core_top/ofb_writer/idma_ctrl 一致化) |
| H=200 W=16 row 175 mismatch | host BRAM 静态 layout `IFM_OFF=0x05000 WB_OFF=0x10000` 给 IFM 仅 44KB，IFM 50KB 溢出覆盖 | host `compute_layout` 动态分配 BRAM region |
| run_layers payload N=3 hardcode | `vd100_rpc.py` `LayerCfg` 固定 3 core, N=1 时 base[2] 越界 | base/count list pad 到 3, 多余填 0 |
| toolchain K=1 拒编译 (ifb_strip_rows_min=K+1=2 > IFB cap) | `hw_files.py:770` 保守约束, K=1 不需要 sliding window | K=1 时 `ifb_strip_rows_min = 1`（单行 ring 串行 valid-ready 反压本就 work） |
| Patch S2D (K=1 c=64 H=240 W=135) row 121+ 错 | toolchain `SMC_FINAL_OFM_BASE 0xE00000 - SMC_INPUT_BASE 0xD00000 = 1 MB` 不够 patch1 IFM 1.98 MB, OFM 起点落在 IFM range 内 | SMC_FINAL_OFM_BASE 0xE00000 → 0xF00000 (2 MB 偏移) + host 同步改 |

---

## 踩坑总结

1. **`wait_on_hw_ila -timeout N` 单位是分钟**（不是秒），写 90 会等 90 分钟而不是 90 秒
2. **toolchain `--smc --n_cores 1`** 是合法的（line 1344 已允许），但需要小输入（IFB=1024 word, W×cin_slices 必须 ≤ IFB / strip_rows）
3. **`run_multicore_chain.py` `--demo resnet11` 默认输入 960×540×4**，N=1 单核装不下 patch K=4（需要小输入）
4. **bootgen 输出 `.pdi` 不是 BOOT.BIN**：`.pdi` 是 JTAG 烧的，`BOOT.BIN` 是 SD/QSPI。`.bif` 输出名决定哪种
5. **ConnectionReset 不等于 A72 crash**：PS 在 RUN_LAYERS 的 poll loop 期间 lwIP TCP keepalive 超时，client 看到 reset，但 A72 还在跑。RPC 立刻重连可以继续 peek
6. **`COUT_SLICES` peek=30 但 cycles 正常**：cfg_regs 某些 RO 读路径有 cache 残留 bug，不影响 sequencer 实际用值（sequencer 直接读 internal reg 不走 mux 读口）
7. **GUI Vivado 开着不能 batch 抢 hw_target**：Open Hardware Manager 状态会锁 cable，必须 Close Target 才能 batch 烧
8. **Vivado batch 跑 `cd dir && vivado.bat`** vs **绝对路径 vivado.bat -source /abs/path**：前者可能让 vivado 内部 init.tcl 找不到 `testbench.tcl`（cwd 影响 IP DB 初始化）。**用绝对路径更稳**
9. **PowerShell 跟 git-bash 处理 `/F` 等参数不一样**：`taskkill /F /PID N` 在 git-bash 被解析成 F: 路径。用 `powershell -Command "Stop-Process -Id N -Force"` 更稳

---

## 当前状态（2026-05-16）

- ✅ N=1 单核 board 端到端 PASS bit-exact
- ✅ Streaming row-ring 任意 H/W 工作正常（fix toolchain mod 后）
- ✅ axi_noc 直连 DDR 无 SmartConnect bridge
- ✅ 8+ case 批量回归全 PASS（wrap 程度 1-4×、stride 1/2、不同 W）

**下一步**：
- 扩 BD 到 N=3 多核（复用 v12 综合架构: SmartConnect 1×3 fanout CSR + 3 PL SI 各占 axi_noc 一通道）
- 跑 resnet11 完整 chain 端到端验证

**memory 历史包袱清理**：`memory/project_axi_protocol_checker_debt.md` 的 "axi_noc 不响应"
workaround **作废**。真 root cause 不是协议，是 toolchain SG cmd 漏 mod + PS ELF 漏
START_LAYER。N=3 多核也能直连 axi_noc，不需要 SmartConnect bridge 整形。
