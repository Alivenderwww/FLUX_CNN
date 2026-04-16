# 未来工作规划

已完成：
- ✅ Macro-ISA → 配置寄存器驱动 FSM (已退役)
- ✅ **握手驱动去中心化流水** (v1 → v3 迭代，MAC 利用率到 99.95%)
- ✅ cross-round pipeline + parf_accum FILL/DRAIN overlap
- ✅ 20-bit 内部地址，支持 224×224 单切片
- ✅ 10/10 packed 回归用例全通

---

## Phase 2 — 架构特性补齐（v1 未做）

### Chunked 权重调度（K≥7 / Cin>32 支持）

v1 `wgt_buffer` 假设 packed `K²·cin_slices ≤ 32`，chunked 用例跑不通（K=3 C64+ 和所有 K=7 用例）。补齐方向：

- `wgt_buffer` 内加 `round_cnt` 和 `cur_round_len` 计数
- 每 `(cins, round)` 触发一次 LOAD（chunked 无轮次：每 cins 一次；多轮次：每 round 一次）
- `wrf_raddr = pos_in_round`（不用 running base 算 `cins*kk+ky*K+kx`）
- state machine 扩展：COMPUTE 阶段 `pos_in_round == round_len-1 && !round_is_last` 时切回 LOAD
- `total_wrf` / `wrf_packed` / `rounds_per_cins` / `round_len_last` cfg 字段已经在 `cfg_regs` 里，直接用

预计改动量：~100 行 wgt_buffer，回归用例打开 K=7 和 C64 后验证。

### stride=1 滑窗复用

原 cfg-driven FSM 支持 stride=1 下 ARF 滑窗复用：`kx=0` 载入 `TILE_W=32` 像素，`kx=1..K-1` 每个只加载 1 个新像素（5-bit ARF 地址自然 wrap）。v1 没有此优化，IFB 读 = `TILE_W × K` (vs 原架构 `TILE_W + K-1`)。

- **K=3 stride=1**：可省 2/3 IFB 读
- **K=7 stride=1**：可省 6/7 IFB 读

改动点：`line_buffer` 的 issue 逻辑在 stride=1 + 整 tile 下进入"滑窗模式"，`kx=0` 发 cur_valid_w 个 issue，`kx=1..K-1` 每个只发 1 个 issue。`ring buffer` 要支持"部分重叠窗口"的读（类似旋转）。实现复杂度较高，是典型的 "再压 MAC 利用率到下一级" 优化（会让 ACT idle 进一步趋近 0，目前已经是 0，所以实际收益有限 —— 主要是 IFB 带宽节约）。

### Padding（NVDLA 风格零代价硬件 padding）

在 `line_buffer` 的 issue 处做坐标越界判定，越界时不发 IFB 读、直接给 act 流喂 0：

```systemverilog
// 新 cfg 字段
logic [3:0] cfg_pad_top, cfg_pad_left;

// line_buffer 内部坐标换算
logic signed [16:0] src_y = yout_cnt * stride + ky_cnt - cfg_pad_top;
logic signed [16:0] src_x = tile_cnt * tile_w + iss_pos * stride + kx_cnt - cfg_pad_left;

wire is_pad = (src_y < 0) | (src_y >= cfg_h_in) |
              (src_x < 0) | (src_x >= cfg_w_in);

// padding 位置不发 IFB 读，arrival 数据改成 0
assign ifb_re = issue_ok & !is_pad;
// ring buffer 直接写 0（不等 IFB 回）
```

需要 `is_pad` 信号跟着 ring buffer 的 wr_idx 流到 arrival 时机。简单做法：把 `(is_pad, data)` 一起入队。

### Residual Add（SDP 融合）

扩展 `sdp.sv` 在 shift 之前加一路残差加法：

```systemverilog
// sdp 新增输入
input logic signed [NUM_COL*PSUM_WIDTH-1:0] residual_in;
input logic                                 residual_en;

// 每列组合：psum + residual → shift → relu → clip
wire signed [PSUM_WIDTH-1:0] combined_ch = 
    residual_en ? (psum_ch[c] + residual_ch[c]) : psum_ch[c];
```

需要新模块（或扩展 `ofb_writer`）从另一块 SRAM 读残差。cfg 加 `cfg_sdp_res_en`, `cfg_res_base`。

### Pooling（Max / Avg）

- 新增 `pool_mode` cfg 字段
- `sdp` 或独立 pool 模块实现 max 比较和 avg 累加
- `ofb_writer` 根据 pool stride 调整写地址步进

### Depthwise Conv

每通道独立卷积，MAC 阵列的"16×16 广播"结构不适用。选择：
- **方案 A**：每列独占一个输入通道、一个输出通道，关闭广播
- **方案 B**：`DW_MODE` 把 K² 个 tap 展平成 16 个 PE 的工作

---

## Phase 3 — 时序优化

### 加法树流水化

`mac_col` 的加法树当前是全组合（16 个 16-bit prod 相加到 32-bit），在 100 MHz 基本不成为瓶颈。拉更高频（250-400 MHz）需要：

- 把 16-PE 求和拆成 2-3 级流水（8+8 → 8 + 剩余 → 最终和）
- 相应调整 `parf_accum` 的 pipe valid 追踪（从 2 级到 3-4 级）
- 关键路径从 16-way adder tree 缩到 4-way

### SDP 拆流水

纯组合的 `sdp` 在 int8 下不是关键路径，如果进一步拉频可拆成 1 shift + 1 relu + 1 clip。相应 `ofb_writer` 的 OFB 写地址要延后 2-3 拍对齐。

---

## Phase 4 — 多核与系统级

### 多核 Mesh 互联

每个核 256 MAC，多核时用片上 mesh 互联做：
- **权重广播**：Cin 小、图大场景，权重分发到多核
- **激活广播**：Cout 大场景
- **Psum 流动**：Cin 和 Cout 都大，Psum 在核间接力累加

### 行流式跨层融合

IFB 容量小，不需要整层特征图存下。下游核攒够 K 行就启动，延迟从"整层"降到"K 行"。

### Task Descriptor + Router

编译器/调度器生成每个核的 Task Descriptor：
- 卷积配置（现有 cfg 寄存器）
- 数据路由（input 来源 / output 目的）
- 核间同步（`wait_mask`, `signal_done`）

握手流水架构对这种 DMA 不确定时序**天然友好** —— 模块间的 valid-ready 背压可以直接接到 DMA 的送数节奏上。

---

## Phase 5 — 工程化 / 验证 / 编译器

- **真实 SRAM 替换**：`sram_model` → BRAM 原语
- **UVM 验证平台**：定向测试 → 约束随机覆盖边界场景
- **性能模型（Roofline）**：算术强度 vs 带宽分析
- **编译器**：`gen_isa_test.py` → 图编译器，支持多层网络
- **多层自动调度器**：ONNX → Task Descriptor 链

---

## 优先级建议

| 优先级 | 工作 | 理由 |
|--------|------|------|
| 高 | Chunked 支持 | 解锁 K=7 和大 Cin 用例（实用模型如 AlexNet、ResNet 首层） |
| 高 | Padding | 任何实用 CNN 都要；零代价硬件实现 |
| 中 | Residual + SDP 融合 | ResNet 是当代标准 |
| 中 | stride=1 滑窗复用 | IFB 带宽节约，次要但实用 |
| 中 | Pooling | 大部分 CNN 需要；实现简单 |
| 低 | Depthwise / 多核 | 复杂度大，需要完整系统级验证 |
| 低 | 时序优化 | 100 MHz 够用了；拉频不是紧迫需求 |
