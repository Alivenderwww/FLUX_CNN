# Descriptor-Driven Sequencer 架构设计

**状态**：**设计阶段**（Phase C），尚未实现。本文档定义新架构的 RTL 接口契约、descriptor 二进制格式、host 编程模型、与现状的兼容路径，作为 Phase C-1 ~ C-5 实现的 source of truth。

**动机**（详见 `claude_history.md`）：

- 现状粒度太粗 —— 一次 `start` 吞下整张图，核内 6 层循环跨度 = 整图，padding/多层/多核扩展都被这个粗粒度卡住
- 中等粒度 (1 descriptor = 1 strip = 数行 OFM) 在 `claude_history.md` 中已论证为最优工程平衡点
- Padding 自然挂在 descriptor 上 —— 边缘 strip 非零，内部 strip 全零，line_buffer 不需要全局行号
- 跨核 mesh 的 MemTile 协议天然复用现有 streaming row-ring 反压

---

## 1. 顶层架构

```
  AXI-Lite Slave (host 控制)         AXI4 Master (DDR)
        │                                  │
        ▼                                  ▼
  ┌──────────────┐                  ┌──────────────┐
  │ axi_lite_csr │                  │  axi_m_mux   │
  └──────┬───────┘                  └─┬──┬──┬──┬───┘
         │                            │  │  │  │
         ▼                          IDMA WDMA ODMA DFE
  ┌──────────────┐                    ▲  ▲  ▲  │
  │  cfg_regs    │                    │  │  │  │
  │ (全局 cfg +  │                    │  │  │  │
  │  CTRL/STAT) │                     │  │  │  │
  └──────┬───────┘                    │  │  │  │
         │                            │  │  │  ▼
         │                            │  │  │ ┌──────────┐
         │      ┌──────────────┐ push │  │  │ │  desc    │
         │      │  Sequencer   │      │  │  │ │  FIFO    │
         └─────►│ - pop desc   │◄─────┘  │  │ │ (32 deep)│
                │ - 翻译字段   │         │  │ └────┬─────┘
                │ - 发 start   │         │  │      │
                │   pulses     │         │  │      ▼
                │ - 收 done    │         │  │  pop desc
                └─┬──┬──┬──┬──┬┘         │  │
                  │  │  │  │  │          │  │
                  ▼  ▼  ▼  ▼  ▼          │  │
        line_buf wgt_buf mac parf ofb     │  │
        (5 模块, 内部 counter 上限改为   │  │
         strip 大小, 其他完全不动)        │  │
                  │           │           │  │
                  └─rows_credit ──────────┘  │
                              │              │
                              └──────────────┘
                              (ODMA row credit)
```

**关键点**：

1. **DFE (Descriptor Fetch Engine)** 是新的第 4 个 DMA，从 DDR 拉 descriptor 列表填充片内 desc FIFO。占用 axi_m_mux 现 reserved 的 M[3] 槽。
2. **Sequencer** 是新模块，从 desc FIFO 弹 descriptor → 翻译字段 → 发 start pulse 给 5 模块和 IDMA/ODMA → 等所有 done 聚合 → 弹下一条。
3. **5 个核流水模块**：内部 counter 上限从"整图 h_out"改为"strip n_yout_strip"，其他完全不动；新增 padding 字段输入（line_buffer 用）。
4. **WDMA 仍按 layer 启动一次**（权重整层共享）；IDMA/ODMA 每 strip 启动一次。
5. **cfg_regs**：保留全局 cfg（K/stride/SDP/...）；CTRL 增加 `start_dfe` `start_layer` pulse；STATUS 增加 `layer_done` `dfe_done`。

---

## 2. Descriptor 格式

**宽度** 256 bit = 32 byte = 8 × 32-bit word（精简版，紧贴 BUS_DATA_W=128 的 2 beat 单位）。

```
+---------+---------+---------+---------+---------+---------+---------+---------+
| Word 0  | Word 1  | Word 2  | Word 3  | Word 4  | Word 5  | Word 6  | Word 7  |
+---------+---------+---------+---------+---------+---------+---------+---------+
```

### 2.1 字段布局

| Word | bits  | 字段                | 说明                                                     |
|------|-------|--------------------|---------------------------------------------------------|
| 0    | [3:0]  | `type`             | `0x0`=NOP / `0x1`=CONV_STRIP / `0x2`=BARRIER / `0xF`=END |
| 0    | [7:4]  | `flags`            | bit0=is_first, bit1=is_last, bit2=streaming_en, bit3=rsvd |
| 0    | [11:8] | `pad_top`          | 本 strip 顶部 padding 行数 (0..15)                       |
| 0    | [15:12]| `pad_bot`          | 本 strip 底部 padding 行数 (0..15)                       |
| 0    | [19:16]| `pad_left`         | 本 strip 左侧 padding 列数 (0..15)                       |
| 0    | [23:20]| `pad_right`        | 本 strip 右侧 padding 列数 (0..15)                       |
| 0    | [31:24]| `rsvd0`            |                                                         |
| 1    | [15:0] | `strip_y_start`    | 本 strip 起始 OFM 行号（layer 内绝对坐标，调试 + sanity） |
| 1    | [31:16]| `n_yout_strip`     | 本 strip 产出几行 OFM（=核流水模块的 yout 上限）         |
| 2    | [19:0] | `ifb_ddr_offset`   | IDMA 相对 IFB DDR base 的字节偏移（本 strip 输入起点）   |
| 2    | [31:20]| `rsvd2`            |                                                         |
| 3    | [23:0] | `ifb_byte_len`     | IDMA 本 strip 要搬的字节数                              |
| 3    | [31:24]| `rsvd3`            |                                                         |
| 4    | [19:0] | `ofb_ddr_offset`   | ODMA 相对 OFB DDR base 的字节偏移（本 strip 输出起点）   |
| 4    | [31:20]| `rsvd4`            |                                                         |
| 5    | [23:0] | `ofb_byte_len`     | ODMA 本 strip 要搬的字节数                              |
| 5    | [31:24]| `rsvd5`            |                                                         |
| 6    | [31:0] | `rsvd6`            | 预留（多层 / residual src offset / Cout > 16 切片 ID）   |
| 7    | [31:0] | `rsvd7`            | 预留                                                    |

### 2.2 type 语义

| type        | 行为                                                                 |
|-------------|---------------------------------------------------------------------|
| `0x0` NOP   | Sequencer 跳过，不发 start pulse                                     |
| `0x1` CONV  | 触发一次 strip 计算（IDMA + 核 + ODMA 三路并发）                     |
| `0x2` BARRIER | 等所有 in-flight strip 完成（done 聚合）后才弹下一条；用于 layer 内同步点（如 padding 边界 sanity） |
| `0xF` END   | 标记 layer 末尾，Sequencer 等所有 in-flight 完成 → 置 STATUS.layer_done |

### 2.3 flags 语义

- `is_first` (bit 0)：layer 内第一个 strip。Sequencer 用它判断是否需要先等 WDMA done。
- `is_last`  (bit 1)：layer 内最后一个 strip。冗余于 END descriptor，但保留方便单条 descriptor 自描述。
- `streaming_en` (bit 2)：本 strip 是否启用 IFB/OFB ring wrap（单 strip 模式 = 0，跨 strip 流水模式 = 1）。
- bit 3 预留。

---

## 3. 全局 cfg 与 descriptor 字段对照表

**判定原则**：单层内不变 → 全局 cfg；strip 间变化 → descriptor。

| 类别       | 字段                                              | 来源       |
|------------|--------------------------------------------------|-----------|
| 卷积参数   | `K`, `stride`, `cin_slices`, `cout_slices`, `kk`, `total_wrf`, `wrf_packed`, `rounds_per_cins`, `round_len_last` | **全局 cfg** |
| 图尺寸     | `h_in_total`, `w_in`, `h_out_total`, `w_out`     | **全局 cfg** |
| Tile 参数  | `tile_w`, `num_tiles`, `last_valid_w`            | **全局 cfg** |
| SRAM 布局  | `ifb_base`, `wb_base`, `ofb_base`, `ifb_cin_step`, `ifb_row_step`, `wb_cin_step`, `wb_cout_step`, `ofb_cout_step`, `tile_in_step` | **全局 cfg** |
| Streaming ring | `ifb_strip_rows`, `ofb_strip_rows`, `ddr_ifm_row_stride`, `ddr_ofm_row_stride` | **全局 cfg** |
| SDP        | `sdp_mult`, `sdp_shift`, `sdp_zp_out`, `sdp_clip_min`, `sdp_clip_max`, `sdp_round_en`, `sdp_relu_en` | **全局 cfg** |
| WDMA       | `wdma_src_base`, `wdma_byte_len`                 | **全局 cfg**（layer 一次） |
| Padding    | `pad_top`, `pad_bot`, `pad_left`, `pad_right`    | **descriptor**（per-strip） |
| Strip 描述 | `strip_y_start`, `n_yout_strip`                  | **descriptor** |
| IDMA per-strip | `ifb_ddr_offset`, `ifb_byte_len`             | **descriptor** |
| ODMA per-strip | `ofb_ddr_offset`, `ofb_byte_len`             | **descriptor** |
| Descriptor list | `desc_list_base`, `desc_count`              | **CTRL 寄存器** |

---

## 4. Sequencer 状态机

### 4.1 接口

```
module sequencer (
    input  clk, rst_n,
    input  start_layer_pulse,         // host 写 CTRL.start_layer
    output layer_done,                // → STATUS.layer_done

    // FIFO 接口
    input  [255:0]  desc_data,
    input           desc_valid,
    output          desc_pop,

    // 全局 cfg 旁路（descriptor 字段叠加）
    input  [..]     cfg_*,            // 来自 cfg_regs

    // 输出给核流水 + DMA
    output          start_core_pulse,
    output          start_idma_pulse,
    output          start_odma_pulse,
    output          start_wdma_pulse, // 仅 is_first 时发一次
    output [15:0]   strip_n_yout,     // 本 strip yout 上限
    output [3:0]    strip_pad_top,
    output [3:0]    strip_pad_bot,
    output [3:0]    strip_pad_left,
    output [3:0]    strip_pad_right,
    output [19:0]   strip_ifb_ddr_offset,
    output [23:0]   strip_ifb_byte_len,
    output [19:0]   strip_ofb_ddr_offset,
    output [23:0]   strip_ofb_byte_len,
    output          strip_streaming_en,

    // done 聚合
    input  core_strip_done,
    input  idma_strip_done,
    input  odma_strip_done,
    input  wdma_done
);
```

### 4.2 状态机

```
S_IDLE
  └─ start_layer_pulse → S_FETCH

S_FETCH
  └─ desc_valid → 锁存 descriptor 字段
                  type ?
                    NOP     → S_FETCH (continue)
                    CONV    → S_DISPATCH
                    BARRIER → S_BARRIER
                    END     → S_END

S_DISPATCH
  └─ 发 start pulses (core + IDMA + ODMA, 加 WDMA 若 is_first)
     → S_WAIT

S_WAIT
  └─ (core_strip_done & idma_strip_done & odma_strip_done) → S_FETCH

S_BARRIER
  └─ 所有 in-flight done → S_FETCH

S_END
  └─ 所有 in-flight done → 置 layer_done, 回 S_IDLE
```

**注**：S_DISPATCH 之后核流水 + IDMA + ODMA 并发；它们的 done 才是 strip 完成的判定。WDMA 的 done 只在 is_first 那个 strip 检查。

### 4.3 Strip-level cfg 输出语义

Sequencer 把 descriptor 字段和全局 cfg 一起拼出 strip-level 的有效 cfg，喂给 5 模块和 IDMA/ODMA。比如：

- 给 line_buffer：`cfg_n_yout = strip_n_yout`，`cfg_pad_top/bot/left/right = strip_pad_*`，其他 cfg 透传 cfg_regs。
- 给 ofb_writer：`cfg_n_yout = strip_n_yout`，其他透传。
- 给 IDMA：`src_base = cfg_ifb_ddr_base + strip_ifb_ddr_offset`，`byte_len = strip_ifb_byte_len`。
- 给 ODMA：类似。

---

## 5. DFE (Descriptor Fetch Engine)

### 5.1 接口

```
module dfe (
    input  clk, rst_n,
    input  start_dfe_pulse,                // host 写 CTRL.start_dfe
    input  [31:0]  desc_list_base,         // DDR 字节地址
    input  [15:0]  desc_count,             // descriptor 条数
    output         dfe_done,
    output         dfe_busy,

    // AXI4 R-only master (M[3])
    output [...]   M_AR*, input [...] M_R*,

    // FIFO 写口
    output [255:0] fifo_wdata,
    output         fifo_we,
    input          fifo_full
);
```

### 5.2 行为

- `start_dfe_pulse` 触发后，按 burst 拉 `desc_count × 32 byte` = `desc_count × 2 beat` (BUS_DATA_W=128)
- 每 2 个 beat 拼成 1 个 256-bit descriptor，写 FIFO
- `fifo_full` 反压（暂停 AR 发起）
- `dfe_done` 在所有 descriptor 写入 FIFO 后置 1

### 5.3 时序

DFE 可以**先于** Sequencer 启动（`start_dfe_pulse` 比 `start_layer_pulse` 早），让 prefetch 提前填 FIFO，避免 Sequencer 取空等待。

---

## 6. 5 模块接口变化（最小改动原则）

### 6.1 line_buffer

**新增 cfg 输入**：
- `cfg_pad_top  [3:0]`
- `cfg_pad_bot  [3:0]` (用于 streaming rows_needed 上限计算)
- `cfg_pad_left [3:0]`
- `cfg_pad_right[3:0]`

**改动**：`cfg_h_out` 改名为 `cfg_n_yout`（语义：本 strip 产出的 OFM 行数）。

**内部状态机改造**（详见 Phase C-2）：
- Y 方向加 PAD_TOP_Y / DATA_Y / PAD_BOT_Y 三阶段 FSM
- X 方向加 PAD_LEFT_X / DATA_X / PAD_RIGHT_X 三阶段 FSM
- pad 阶段不发 IFB 读，arrival 时 act_buf 写 0

**Streaming 反压**：`rows_needed = max(yout·stride + K - pad_top, 0)`，前 pad_top 行不等 IDMA。

### 6.2 wgt_buffer

**RTL 0 改动**（方案 E）。

**协议**：Sequencer 只在 `flags.is_first` 的 descriptor 发 `start_wgt_pulse`。wgt_buffer 按 `cfg_h_out = h_out_total` 跑完整 layer 的 6 层循环（cs/yout/tile/cins/ky/kx），跨 strip 无感知。核流水 `wgt_valid / wgt_ready` 握手自然处理对齐：line_buffer 跨 strip 重启导致 act 流间隙时，mac_array `wgt_ready=0` 会 stall wgt_buffer 直到新 strip 的 act 流到位。

**done 聚合**：wgt_buffer 的 done 进 `layer_done`，**不**进 `strip_done`。Sequencer 的 `S_WAIT` 只 wait `(core_strip_done & idma_strip_done & odma_strip_done)`，BARRIER 和 END 才额外 wait `wgt_done`。

### 6.3 ofb_writer

**改动**：`cfg_h_out` → `cfg_n_yout`。其他不动。

**OFB SRAM 复用**：每 strip 写 `n_yout × W_OUT × Cout` 个像素到 `cfg_ofb_base` 起始；ODMA 每 strip 搬出后 OFB 区域复用（streaming 模式仍是 ring）。

### 6.4 mac_array, parf_accum

**0 改动**。它们消费 act/wgt 流，根据 `cfg_kk` / `cfg_cin_slices` 等全局参数推导内部状态，对 strip 粒度无感知。

---

## 7. cfg_regs 改造

### 7.1 保留的字段（全部）

现有 0x100 ~ 0x17C 的所有 cfg 字段保留语义不变（除了 `H_OUT` 改为"全图 h_out_total"，单 strip 上限由 Sequencer 通过 strip_n_yout 输出）。

### 7.2 新增字段

| 地址  | 字段                | 说明                                       |
|-------|--------------------|--------------------------------------------|
| 0x180 | `DESC_LIST_BASE`   | DDR 中 descriptor list 起始地址（32 bit）  |
| 0x184 | `DESC_COUNT`       | descriptor 条数（16 bit）                  |
| 0x188 | `SDP_MULT`         | 32 bit signed                              |
| 0x18C | `SDP_ZP_OUT`       | 9 bit signed                               |
| 0x190 | `SDP_CLIP_MIN`     | 9 bit signed                               |
| 0x194 | `SDP_CLIP_MAX`     | 9 bit signed                               |
| 0x198 | `SDP_ROUND_EN`     | 1 bit                                      |

**原地扩展**：0x160 `SDP_SHIFT` 从 5 bit 就地扩展到 6 bit（bit[5:0]），保持地址不变以最小化 TB 改动。

### 7.3 CTRL 扩展

| bit | 名称              | 行为                                   |
|-----|------------------|---------------------------------------|
| 0   | `start_core`     | **deprecated**（兼容老 TB，保留）     |
| 1   | `start_idma`     | **deprecated**                        |
| 2   | `start_wdma`     | **deprecated**                        |
| 3   | `start_odma`     | **deprecated**                        |
| 4   | `start_dfe`      | 启动 DFE 拉 descriptor list           |
| 5   | `start_layer`    | 启动 Sequencer 消费 descriptor        |

老 bit 0~3 保留是为了 Phase C-1 兼容旧回归（详见 §9）。

### 7.4 STATUS 扩展

| bit | 名称              |
|-----|------------------|
| 0~7 | (现有 core/dma busy/done) |
| 8   | `dfe_busy`       |
| 9   | `dfe_done`       |
| 10  | `layer_busy`     |
| 11  | `layer_done`     |

---

## 8. Host 编程序列（新模式）

```c
// ---- Layer setup ----
// 1. 写全局 cfg（K, stride, h_in_total, w_in, h_out_total, w_out, ...
//    cin_slices, cout_slices, tile_w, num_tiles, last_valid_w,
//    kk, total_wrf, wrf_packed, rounds_per_cins, round_len_last,
//    ifb_base, wb_base, ofb_base, all _step,
//    ifb_strip_rows, ofb_strip_rows, ddr_*_row_stride, dma_mode,
//    SDP: mult, shift, zp_out, clip_min, clip_max, round_en, relu_en,
//    WDMA: src_base, byte_len)
write_csr(ADDR_K, 3);
write_csr(ADDR_STRIDE, 1);
... (~30 寄存器写)

// 2. 写 descriptor list 控制
write_csr(ADDR_DESC_LIST_BASE, ddr_addr_of_desc_list);
write_csr(ADDR_DESC_COUNT, num_descriptors);

// 3. 启动 DFE prefetch
write_csr(ADDR_CTRL, CTRL_START_DFE);

// 4. 启动 Sequencer 消费
write_csr(ADDR_CTRL, CTRL_START_LAYER);

// 5. 等 layer_done
while (!(read_csr(ADDR_STATUS) & STATUS_LAYER_DONE)) ;
```

Descriptor list 由 host 离线生成（`gen_isa_test.py` 改造，详见 Phase C-4），一条 layer 通常 N+1 条：
- N 条 `CONV_STRIP`（每 strip 一条，pad 字段在边缘 strip 非零）
- 最后 1 条 `END`

---

## 9. 老回归兼容路径

Phase C-1 完成后，老回归（tb_core_isa 14 case + tb_core_dma batch 14 case + streaming 11 case）必须**仍然 PASS**，不改预期。兼容做法：

### 方案：单条 descriptor 等价整图

`gen_isa_test.py` 加 `--legacy_single_strip` 模式（默认开启），生成 1 条 CONV_STRIP descriptor：
- `n_yout_strip = h_out_total`（单 strip = 整图）
- `pad_*` 全 0
- `ifb_ddr_offset = 0`, `ifb_byte_len = 整图字节数`
- `ofb_ddr_offset = 0`, `ofb_byte_len = 整图字节数`
- 加 1 条 END

加上全局 SDP cfg 用 legacy 默认值（`mult=1, shift=shift_amt_legacy, round=0, zp=0, clip=[0,255], relu=1`），新 SDP 严格等价旧 SDP。

老 TB（直接写 CTRL=0xB 启 IDMA/WDMA/ODMA/core）仍可用 —— 兼容期间 cfg_regs 保留老 CTRL bit 0~3 的 pulse 路径（旁路 Sequencer），让老 TB 不改也能跑。Phase C-5 完成后老 CTRL bit 可以打 deprecated 注解，待新 TB 验证全绿后再删。

---

## 10. Phase 边界与里程碑

| Phase | 主要改动                                           | 验收标准                                                                                  |
|-------|--------------------------------------------------|----------------------------------------------------------------------------------------|
| C-1   | desc FIFO + Sequencer + DFE + cfg_regs 扩展      | (a) 老 TB 39 case 走老 CTRL[3:0] pulse 路径 100% PASS（验证无 regression）；(b) 新增 1~2 个 case 走新 DFE + Sequencer 路径，n_yout_strip = h_out_total 单 strip 模式，结果与 (a) 等价 |
| C-2   | line_buffer Y/X 双向 FSM + padding               | 加 pad 的新 case：K=3 pad=1, K=5 pad=2, K=7 pad=3，各 1 case 验证；老回归仍 PASS         |
| C-3   | sdp.sv 改 mult-shift-zp-clip                      | 老回归（legacy SDP 参数）仍 PASS；新增 PyTorch 对称 int8 case（mult ≠ 1, shift = 31）验证数值匹配 numpy ref |
| C-4   | gen_isa_test.py descriptor 编译器                | 输出 descriptor list bin 文件 + golden；支持 `--pad`, `--quant_mode {legacy, sym_int8}`  |
| C-5   | TB 改造：从 CSR write 切到 descriptor list       | 全部新老回归过；老 TB 路径标 deprecated                                                  |

---

## 11. 演进展望（不在本 Phase 范围）

- **Phase D**：跨核 MemTile + 双 row counter（`claude_history.md` §3）；Sequencer 增加 src_event_id / dst_event_id 字段
- **Phase E**：sideband token 化（`claude_history.md` §5）；mac/parf/ofb 之间的 tile_done/cs_done 用 first-class 协议
- **Phase F**：多层 cfg override（descriptor 携带 cfg 增量）；residual / pooling / depthwise

---

## 12. 已敲定决策

1. **desc FIFO**：DFF-based，深度 32，宽度 256 bit。主机负责持续提供新 descriptor，不追求片内大 buffer。
2. **BARRIER 期间 IDMA prefetch 允许**：Sequencer 在 BARRIER 聚合 done 期间，仍可发 `start_idma_pulse` 给下一个 CONV_STRIP 的 IDMA（因为下一 descriptor 已在 FIFO 里，IDMA 的搬运与核流水无强时序耦合）。代价是 IDMA 的 start/done 协议需要支持背对背。
3. **wgt_buffer 跨 strip 持续运行（方案 E）**：Sequencer 只在 `flags.is_first` 的 descriptor 发 `start_wgt_pulse`，wgt_buffer 内部按 `cfg_h_out = h_out_total` 跑完整 layer 循环，跨 strip 无感知。核流水 `wgt_valid / wgt_ready` 握手自然处理 line_buffer 跨 strip 重启时的对齐问题。strip_done 聚合**不**等 wgt_buffer，它的 done 聚合到 layer_done。wgt_buffer RTL **0 改动**。
4. **Descriptor 字节序**：Little-endian，DFE 拉的第 1 个 128-bit beat 构成 word [3..0]（低位），第 2 个 beat 构成 word [7..4]（高位）。host 和 gen_isa_test.py 按此约定排布。
