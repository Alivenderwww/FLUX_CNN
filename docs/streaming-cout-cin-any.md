# Streaming 模式支持任意 Cin / Cout / xy（方式 1 设计文档）

**状态**：**设计阶段（Phase D）**，Phase D-2 ~ D-7 实施待开工。本文档定义数据布局、循环嵌套、各模块改动契约和编译器切片推导规则，作为 Phase D 实现的 source of truth。

---

## 1. 背景与目标

### 1.1 当前限制（Phase B/C 遗留）

Streaming 模式当前硬要求：

```
cin_slices = 1  →  Cin ≤ 16
cout_slices = 1  →  Cout ≤ 16
```

原因：
- **cout_slices=1**：`ofb_writer` 的 6 层循环里 cs 在最外层，意味着整层先跑 cs=0 的全部 OFM 行，再跑 cs=1 的全部。同一 OFM 行的不同 cs 段写入 DDR 被拆到 "整 layer 时间尺度" 上，ODMA 的单调 `rows_drained` 行级 credit 协议失效。
- **cin_slices=1**：IDMA 的搬运假设 IFM DDR 布局是 "cin-slice-major"（外 cins → 内 y → 内 x），cin_slices > 1 时单个连续 byte range 无法覆盖一个 strip 所需的全部行。

### 1.2 目标

Streaming 模式下解除上述两个限制，支持任意组合：
- Cin > 16（cin_slices > 1）
- Cout > 16（cout_slices > 1）
- 任意 xy 尺寸（已支持，VGA 480×640 已验证）

### 1.3 方式 1（采用方案）

核心思路：**把 cs 循环从最外层下移到 yout 内**，配套改 OFB SRAM 布局和 ODMA 扫描模式。同时把 DDR layout 统一改成 NHWC（先 y，再 x，再 cout），IDMA 按线性行搬运即可。

好处：
- 输出时序 **"先出 1 个像素的全部通道，再下一个像素"**，天然契合未来多核层融合（下一层 Core2 的 line_buffer 正好按此顺序消费）。
- IDMA 不需要 gather/stride 搬运逻辑（改动最小）。

---

## 2. DDR 数据布局（NHWC）

### 2.1 IFM 布局

```
DDR[ifm_base + y * (W_IN × Cin) + x * Cin + cin]
```

- 最外层 y，次 x，最内 cin（所有通道相邻）
- 一行字节数 = `W_IN × Cin`
- 整图字节数 = `H_IN × W_IN × Cin`

**AXI word 粒度**：每 AXI beat 16 字节 = 16 个 cin 通道值。对 Cin 不是 16 倍数的情况，每行末尾 pad 0 到 `ceil(Cin/16) × 16`（= `cin_slices × 16`）。实际一行占用：

```
ifb_bytes_per_row = W_IN × cin_slices × 16       (字节)
                  = W_IN × cin_slices              (word, 1 word = 16 byte)
```

### 2.2 OFM 布局

对称：

```
DDR[ofm_base + y * (W_OUT × Cout_padded) + x * Cout_padded + cout]
ofb_bytes_per_row = W_OUT × cout_slices × 16
```

### 2.3 权重布局

权重保持现状（与 NHWC 解耦，独立管理）：

```
WB SRAM word = NUM_COL × NUM_PE × 8 bit = 16 × 16 × 8 = 2048 bit
按 (cs, cin_slice, ky, kx) 编址，word 内部按 (cout[16..0], cin[16..0]) 排布
```

### 2.4 与老布局的兼容

老 streaming 下 Cin ≤ 16 时 `cin_slices=1`，NHWC 的 "每行 W_IN × 16 字节" 和老 cin-slice-major 的 "每行 W_IN × 16 字节" 完全一致 —— 布局退化相同，老回归无需改动 IFM 数据。老 Cout ≤ 16 类似。

**Cin > 16 或 Cout > 16 的 case 一定是新增 case**（老路径不支持），所以 NHWC 切换对老回归零影响。

---

## 3. 核流水循环嵌套（方式 1）

```
for yout      = 0 .. n_yout_strip - 1
  for cs      = 0 .. cout_slices - 1          ← cs 从最外下移到这里
    for tile  = 0 .. num_tiles - 1
      for cin_slice = 0 .. cin_slices - 1
        for ky   = 0 .. K - 1
          for kx = 0 .. K - 1
            for iss_pos = 0 .. cur_valid_w - 1
              → 一次握手 fire
```

- **一个 yout 内先完成所有 cs 段**，再推进到下一 yout
- cs 段之间共享同一份 IFB 数据（yout 对应的 K 行输入）
- cs 段之间需要切换 WRF 的权重（packed 模式下每次切换 LOAD 一次）

对 "640×480×32→32 K=3 stride=1" 的例子：
- 每 yout 执行 2 cs × 20 tile × 2 cin × 9 ky kx × 32 iss_pos = **23040 次 fire**
- 每 yout 产出 W_OUT=638 个像素 × 32 cout = 20416 字节 OFM

---

## 4. OFB SRAM 新布局（行内 cs 段）

### 4.1 写地址规则

```
ofb_ptr = yout_ring_base + cs × W_OUT + x
```

其中 `yout_ring_base` 是当前 yout 在 OFB ring 中的基址（按 ofb_strip_rows wrap）。

**图示（1 个 yout 在 OFB 里占的区段）**：

```
地址：  0     W_OUT    2×W_OUT   ...   (cout_slices-1)×W_OUT
        │       │         │                │
        ▼       ▼         ▼                ▼
       [cs=0 的 W_OUT 个 word][cs=1 的 W_OUT 个 word]...
```

### 4.2 ring wrap

```
ofb_row_words_per_yout = W_OUT × cout_slices
ofb_ring_size_words    = ofb_strip_rows × ofb_row_words_per_yout
```

ofb_ptr 在达到 `ofb_ring_size_words` 时 wrap 回 ring 起点。

### 4.3 `row_done_pulse` 的新语义

**不是 tile wrap 就发**，而是 **"一个 yout 的全部 cs 段 drain 完成"** 时发。

实现：在 ofb_writer FSM 的事件链里，`row_done_pulse = evt_fire_tile_wrap && tile_is_last && cs_is_last`（而不是 `evt_fire_tile_wrap && tile_is_last`）。

---

## 5. ODMA 新扫描模式（gather）

### 5.1 读顺序

```
for x = 0 .. W_OUT - 1
  for cs = 0 .. cout_slices - 1
    read OFB[yout_ring_base + cs × W_OUT + x]
```

对每个 yout，ODMA 发出 `W_OUT × cout_slices` 个读地址。

### 5.2 DDR 写顺序

按 NHWC 线性：

```
DDR[ofm_base + y × ofb_bytes_per_row + pixel_offset]
pixel_offset 从 0 递增到 ofb_bytes_per_row - 1
```

ODMA 的 cur_addr 简单递增，每 AXI beat 16 字节 = 1 个 OFB word。一个 yout 行的 AW burst 总长 = `W_OUT × cout_slices` beats（可能分多个 burst，每 burst 最多 256 beats）。

### 5.3 实现方式

ODMA 的 `rd_ptr` 计算改为：

```
rd_ptr = yout_ring_base + (beat_in_row % cout_slices) × W_OUT
                        + (beat_in_row / cout_slices)
```

其中 `beat_in_row` 是本行已发的 beat 计数（0..W_OUT × cout_slices - 1）。

等价：把 `beat_in_row` 解读为 `(x, cs)` 二维坐标 `x = beat_in_row / cout_slices, cs = beat_in_row % cout_slices`。

---

## 6. IDMA 新 NHWC 线性搬运

### 6.1 每行字节数

```
ifb_bytes_per_row = W_IN × cin_slices × 16
```

每行连续存放所有 cin_slice 的数据，无跨段。

### 6.2 IDMA 行级反压

保留现有机制：
- `rows_written` = IDMA 已写入 IFB SRAM 的行数
- `rows_consumed` = line_buffer 已用完的行数
- IDMA 等 `rows_written - rows_consumed < ifb_strip_rows` 才继续写下一行

**改动**：`cfg_ifb_cin_step` 不再是 ring wrap 模数（因为每行已含所有 cin_slice）。ring wrap 模数直接改为 `ifb_strip_rows × ifb_bytes_per_row / 16 = ifb_strip_rows × W_IN × cin_slices`（word 单位）。

IDMA.sv 里的 `wr_ptr` wrap 逻辑沿用现有框架，但 wrap 阈值改名：

```
cfg_ifb_ring_words = ifb_strip_rows × W_IN × cin_slices
```

（新 cfg 字段 或 复用 cfg_ifb_cin_step 语义 = 每 slice 的 ring 行范围？让编译器一次性算好）

---

## 7. 编译器推导 strip_rows 上下限

### 7.1 OFB 侧约束

```python
# 上限（容量）
ofb_row_words = W_OUT * cout_slices
ofb_strip_rows_max = OFB_SRAM_WORDS // ofb_row_words         # 8192 word

# 下限（ODMA 流水）
ofb_strip_rows_min = 2   # 至少 2 让 ofb_writer 写当前行同时 ODMA 搬上一行

# 推荐值
ofb_strip_rows = min(max(ofb_strip_rows_min, 2), ofb_strip_rows_max, H_OUT)

# 违反
if ofb_strip_rows_max < ofb_strip_rows_min:
    sys.exit(f"ERROR: OFB SRAM 容量不足，W_OUT={W_OUT} × cout_slices={cout_slices} "
             f"> OFB_SRAM_WORDS({OFB_SRAM_WORDS}) / ofb_strip_rows_min({ofb_strip_rows_min})")
```

### 7.2 IFB 侧约束

```python
# 下限（流水）
ifb_strip_rows_min = K + 1     # K 行参与当前 yout 计算 + 1 行预取

# 上限（容量）
ifb_row_words = W_IN * cin_slices
ifb_strip_rows_max = IFB_SRAM_WORDS // ifb_row_words          # 8192 word

# 推荐值
ifb_strip_rows = min(ifb_strip_rows_min + 2, ifb_strip_rows_max, H_IN)

# 违反
if ifb_strip_rows_max < ifb_strip_rows_min:
    sys.exit(f"ERROR: IFB SRAM 容量不足，W_IN={W_IN} × cin_slices={cin_slices} "
             f"> IFB_SRAM_WORDS({IFB_SRAM_WORDS}) / ifb_strip_rows_min({K+1})")
```

### 7.3 典型 case 推导（验证合理性）

| case | Cin | Cout | W_IN | W_OUT | cin_slices | cout_slices | ifb_row_words | ifb_rows_max | ofb_row_words | ofb_rows_max |
|---|---|---|---|---|---|---|---|---|---|---|
| VGA | 3 | 16 | 640 | 638 | 1 | 1 | 640 | 12 | 638 | 12 |
| VGA Cout=32 | 3 | 32 | 640 | 638 | 1 | 2 | 640 | 12 | 1276 | 6 |
| VGA Cin=32 | 32 | 16 | 640 | 638 | 2 | 1 | 1280 | 6 | 638 | 12 |
| VGA Cin=32 Cout=32 | 32 | 32 | 640 | 638 | 2 | 2 | 1280 | 6 | 1276 | 6 |
| 小图 Cout=64 | 3 | 64 | 128 | 126 | 1 | 4 | 128 | 64 | 504 | 16 |
| 超大 K=7 Cin=32 | 32 | 16 | 640 | 634 | 2 | 1 | 1280 | 6 | 634 | 12 |

K=7 case：ifb_strip_rows_min = 8，而 ifb_rows_max = 6 < 8 → **不可行**，编译器报错。这 case 编译器建议减小 W_IN（切成 strip-wise 多 Core 处理，属于 multi-core 话题，Phase E 之后）。

---

## 8. line_buffer 循环重排

### 8.1 counter 优先级变化

| counter | 老位置（cs 最外） | 新位置（cs 下移） |
|---|---|---|
| cs_cnt | 最外，在 yout 之外 | yout 之内、tile 之外 |
| yout_cnt | cs 之内 | 最外 |

### 8.2 事件链调整

老：`iss_pos_wrap → kx_wrap → ky_wrap → cins_wrap → tile_wrap → yout_wrap → cs_wrap → all_done`

新：`iss_pos_wrap → kx_wrap → ky_wrap → cins_wrap → tile_wrap → cs_wrap → yout_wrap → all_done`

即 cs_wrap 和 yout_wrap 交换位置。

### 8.3 ptr_*_base 推进

`ptr_yout_base` 每 cs_wrap（!yout_is_last）时 += cfg_ifb_row_step × stride ... 实际上 **同一 yout 内不同 cs 段共享相同的 IFB 行范围**，所以 cs 切换时 ptr_yout_base 应**不变**（只是重新扫描一遍同样的 IFB 区域）。

具体：
- cs 切换 (`evt_iss_tile_wrap && cs_is_last && !yout_is_last`)：ptr_yout_base += row_step × stride
- cs 切换但还有下一 cs（`evt_iss_tile_wrap && !cs_is_last`）：ptr_yout_base **不动**（回到本 yout 的 IFB 起点）
- 同一 yout 内的 tile / cins / ky / kx 推进同老规则

### 8.4 Padding y_row_base / x_tile_base

现有 pad 累加器（Phase C-2 引入）在方式 1 下只需调整"什么时候 += stride"：
- y_row_base += stride **只在 yout 推进时**，不在 cs 切换时
- x_tile_base 推进逻辑不变（tile wrap 时推进）

---

## 9. wgt_buffer 循环重排 + cs 切换 LOAD

### 9.1 counter 重排

同 line_buffer，cs 和 yout 交换。

### 9.2 packed 模式每 yout × cs 一次 LOAD

**老行为（cs 最外）**：每个 cs 的开头 LOAD 一次，整个 cs 内 h_out × num_tiles × cins × ky × kx 次 stream 都复用这份 WRF。一共 cout_slices 次 LOAD。

**新行为（cs 下移）**：每个 yout × cs 的开头 LOAD 一次，一共 h_out × cout_slices 次 LOAD。

每次 LOAD 耗时 = K² × cin_slices 拍（packed 模式下一次性全部载入）。

### 9.3 chunked 模式 rolling

chunked 模式本来就持续 rolling LOAD + STREAM 交替。cs 切换时 wgt_buffer 内部状态（rolling pointer）重置到新 cs 的权重起点，rolling 继续跑。相对 packed 模式的固定 LOAD 阶段，chunked 模式的 cs 切换开销更小（在 rolling 中自然吸收）。

### 9.4 WB SRAM 地址算法

`wrf_load_addr = cs × (kk × cin_slices) + cins_cnt × kk + ky × K + kx`
（与老相同，不变，因为 WB SRAM 布局不变）

---

## 10. ofb_writer 改动

### 10.1 counter 重排

cs 从最外层下移到 yout 内：`yout → cs → tile → x`。

事件链：
- `evt_fire_x_wrap`（原）：x 扫完 cur_valid_w
- `evt_fire_tile_wrap`（原）：tile 扫完 cfg_num_tiles
- **新** `evt_fire_cs_wrap`：cs 扫完 cfg_cout_slices（= 一个 yout 的全部 cs 段完成）
- `evt_fire_yout_wrap`（原）：yout 扫完 cfg_n_yout

### 10.2 ofb_ptr 计算

```
ofb_ptr = yout_ring_base + cs_cnt × cfg_w_out + x_in_yout
```

- `yout_ring_base` = `(yout_cnt % cfg_ofb_strip_rows) × cfg_w_out × cfg_cout_slices`（streaming）
                 或 `yout_cnt × cfg_w_out × cfg_cout_slices`（batch，保留）
- `x_in_yout = tile_cnt × cfg_tile_w + x_cnt`

cfg_regs 新增字段 `cfg_ofb_row_words = cfg_w_out × cfg_cout_slices` 或在 core_top 组合算出。

### 10.3 row_done_pulse 新时机

```
assign row_done_pulse = evt_fire_cs_wrap;   // 整个 yout 的所有 cs 段 drain 完成
```

（原来是 `evt_fire_tile_wrap`）

---

## 11. ODMA 改动

### 11.1 新 cfg 端口

```
input logic [5:0] cfg_odma_cout_slices;
input logic [15:0] cfg_w_out;
```

### 11.2 raddr 生成

```systemverilog
// 每行 beats 总数
wire [15:0] beats_per_row_odma = cfg_w_out * cfg_odma_cout_slices;

// 本行内第几 beat
reg [15:0] row_beat_idx;     // 0 .. beats_per_row_odma - 1

// 解码 (x, cs)
wire [15:0] x_in_yout = row_beat_idx / cfg_odma_cout_slices;
wire [5:0]  cs_in_yout = row_beat_idx % cfg_odma_cout_slices;

// rd_ptr
wire [SRAM_ADDR_W-1:0] rd_ptr_base = /* yout_ring_base */;
assign ofb_raddr = rd_ptr_base + cs_in_yout * cfg_w_out + x_in_yout;
```

注意：上面是组合表达，实际要流水化以避开除法/取模的组合路径，在 FSM 里按 row_beat_idx 递增时维护 x_in_yout 和 cs_in_yout 两个独立 counter。

### 11.3 cs 变成内层 counter 的理由

**注意时序**：原要求 "先 x=0 所有 cs，再 x=1 所有 cs" 即 cs 是内层。所以 ODMA 的扫描顺序 cs 内层、x 外层。和 ofb_writer 写入顺序 "cs 外层、x 内层" **正好相反**。正是这个反向使 DDR 得到 NHWC 顺序。

---

## 12. Sequencer 改动（极小）

Sequencer 本身不管循环顺序，只管 strip-level descriptor。方式 1 下 descriptor 格式和字段**完全不变**。

唯一要确认：descriptor 里的 `ofb_byte_len` 按 NHWC 算 = `n_yout_strip × W_OUT × cout_slices × 16`。gen_isa_test.py 已经按这个公式算（Phase C-4 已实现）。

---

## 13. 各模块改动量预估

| 模块 | 改动行数 | 关键点 |
|---|---|---|
| `line_buffer.sv` | ~30 | 6 层 counter 重排，cs/yout 交换；ptr_* 推进逻辑 |
| `wgt_buffer.sv` | ~50 | counter 重排；packed LOAD 触发时机改为 "每 yout × cs" |
| `ofb_writer.sv` | ~30 | ofb_ptr 地址公式；row_done 改为 cs_wrap |
| `odma.sv` | ~40 | raddr 改 gather；row_beats_left 按 cout_slices 扩展 |
| `idma.sv` | ~20 | ring wrap 模数统一为 `ifb_strip_rows × W_IN × cin_slices`；去掉 cin-slice-major 假设 |
| `cfg_regs.sv` | ~5 | 可能新增 `OFB_ROW_WORDS` 或 Sequencer 组合给 ODMA（看实现偏好） |
| `sequencer.sv` | 0 | 不动 |
| `gen_isa_test.py` | ~120 | NHWC 数据生成；strip_rows 动态推导 + 超限报错；去掉 cin/cout_slices=1 断言；golden 按 NHWC |
| `tb_core_dma.sv` | ~10 | 新 CSR 地址（如果有新字段）|

**总计**：约 300 行 RTL + 130 行 Python。

---

## 14. Phase D 子阶段划分

| 阶段 | 任务 | 验收标准 |
|---|---|---|
| D-1 | 本文档（设计规范）| 用户 review 通过 |
| D-2 | `gen_isa_test.py` 改 NHWC + 动态 strip_rows + 去限制 | 老 streaming 回归（Cin=3/16, Cout=16）全过；新增 1 个 NHWC 单 case 手动验证 |
| D-3 | `line_buffer` + `wgt_buffer` 循环重排 | 老 streaming 回归全过（cin_slices=cout_slices=1 时循环退化到老行为，cycles 零变化）|
| D-4 | `ofb_writer` 行内 cs 段布局 | 同上 |
| D-5 | `ODMA` gather 模式 | 同上 |
| D-6 | `IDMA` 线性搬运清理 | 同上 |
| D-7 | 集成 + 新回归 | `cin_slices=2 cout_slices=1 Cin=32`、`cin_slices=1 cout_slices=2 Cout=32`、`cin_slices=2 cout_slices=2 Cin=Cout=32` 三种组合小图 case 全过；VGA 640×480×32→32 大图 case 验证，MAC 利用率 > 95% |

---

## 15. 向后兼容策略

**老 batch 模式保留**：batch 模式（cin_slices/cout_slices 任意但串行）不动。

**老 streaming cin_slices=1 & cout_slices=1 行为**：方式 1 下自然退化等价：
- cs 下移：cout_slices=1 时 cs 循环只 1 次，外层 cs_wrap 在 yout_wrap 前一拍触发，语义与"cs 在最外层"一致
- NHWC 布局：cin_slices=1 / cout_slices=1 时每行字节数和旧的 cin-slice-major 完全一致
- ODMA gather：cout_slices=1 时 `cs_in_yout` 恒为 0，等价线性扫描

**预期**：老 streaming 11 case 通过 D-3 ~ D-6 每阶段都应保持 PASS，cycles 零变化。

---

## 16. 多核层融合的准备

方式 1 的输出时序 **per-pixel all-cout** 是多核 cascade stream 的天然单位。未来 Phase E 多核互联时：

- Core1 的 ofb_writer 输出流（SDP 之后）= NHWC 一拍一像素的所有 cout
- Core2 的 line_buffer 输入端可以直接串上这个流（用 row-ring FIFO + 双向行级反压，和单核 streaming 同协议）

这是 Phase E 的事，不在 D 的范围内，但 D 的 NHWC 布局和方式 1 循环为此铺路。

---

## 17. 待定与讨论点

- **cfg_ofb_row_words vs 组合算**：新增 cfg 字段（明确）vs 在 core_top 组合 `cfg_w_out × cfg_cout_slices`（简洁）。推荐后者（1 行 verilog）。
- **IDMA ring wrap 复用**：是否把 `cfg_ifb_cin_step` 的 streaming 语义改为 `ifb_strip_rows × W_IN × cin_slices`（word 单位），还是新 cfg？推荐复用老 cfg，只是编译器算 cfg 值的公式改一下。
- **WRF 双缓冲**：方式 1 下 packed 模式每 yout × cs 一次 LOAD ~18 拍，开销 ~1%。是否加双缓冲把 LOAD 完全隐藏？Phase D 不做，留 Phase E。
