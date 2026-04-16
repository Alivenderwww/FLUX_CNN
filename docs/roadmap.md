# 未来工作规划

已完成（phase 1）：
- ✅ Macro-ISA → 配置寄存器 + 自驱动 FSM
- ✅ 轮次分块（K² > WRF_DEPTH，支持 K=7/9/...）
- ✅ 20-bit 内部地址，64K word IFB 容量
- ✅ 17/17 回归用例通过（含 224×224 大图）

---

## Phase 2 — 算子支持扩展

### Padding（零代价硬件 padding，NVDLA 风格）

在 `core_ctrl` 的 PF (prefetch) 阶段做坐标越界判定，越界时不发 SRAM 读、直接给 MAC 喂 0：

```systemverilog
// 新增 cfg 寄存器
logic [3:0] cfg_pad_top, cfg_pad_left;

// PF stage 坐标换算
logic signed [16:0] src_y = yout_cnt * stride + ky_cnt - cfg_pad_top;
logic signed [16:0] src_x = x_tile_in_r + sub_cnt * stride + kx_cnt - cfg_pad_left;

// 越界判定
wire is_pad = (src_y < 0) | (src_y >= cfg_h_in) |
              (src_x < 0) | (src_x >= cfg_w_in);

// padding 像素：不读 SRAM，ARF 写 0
assign ifb_re   = !is_pad;
// bypass MUX 需要对 is_pad 扩展：act_to_mac = is_pad ? '0 : (原有 mux)
```

优势：不占 SRAM 空间、不占 SRAM 带宽、不需额外指令。

### Residual Add（SDP 融合）

扩展 SDP，在 `shift` 之前加一路残差加法：

```systemverilog
// SDP 新增输入
input logic [PSUM_WIDTH-1:0] residual_in;
input logic                  residual_en;

wire signed [PSUM_WIDTH-1:0] combined =
    residual_en ? (psum_in + residual_in) : psum_in;
wire [7:0] result = clamp(combined >>> shift_amt);
```

配置寄存器新增 `cfg_sdp_res_en`, `cfg_res_base`。FSM 在 `ST_OFM` 阶段并发发起 `RES_SRAM`（可复用 IFB 或单独给端口）读请求。

### Pooling（Max / Avg）

- 新增 `S_POOL` FSM 状态或复用 `S_SUBOP` 改走 SDP 的 pooling 模式
- SDP 内增加比较器（max）或累加+除法（avg）

### Depthwise Conv

每通道独立卷积，MAC 阵列的 16×16 广播结构不再适用。可以：
- **方案 A**：把 DW conv 映射成 "每列一个通道，每列独立权重"，广播关闭
- **方案 B**：新增 `DW_MODE` 标志位，软件把 K² 个 tap 展平成 16 个 PE 各自的工作（单列满载）

---

## Phase 3 — 性能优化

### 消除 sub_op 预取气泡

当前每个 sub_op 首拍（cnt=0）是 IFB 预取，不做 MAC。理论可融合相邻 sub_op：前一 sub_op 的最后一拍 MAC 同时发起下一 sub_op 的 IFB 预取。

- 预期收益：MAC 利用率 87% → 95%+（K=3）、92% → 96%+（K=7）
- 实现复杂度：中等 —— 需要在 FSM 里提前 1 拍计算下一 sub_op 的地址

### ST_OFM 与下一 tile MAC 重叠

需要 PARF 双 bank（读写分离）：
- Bank A 持有上一 tile 的 PARF 内容，`ST_OFM` 从它读
- Bank B 是新 tile 的累加目标，MAC 往它写
- 每 tile 结束后 bank 指针翻转

面积代价：PARF 加倍（当前 16 列 × 32 深 × 32-bit = 16384 bit → 32K bit）
预期收益：消除每 tile 的 `valid_w` 拍气泡（K=3 68x120 约节省 `66 × (32+32+32+22) / 80267 ≈ 10%`）

### 流水线深度优化（拉高时钟）

见 [`synthesis.md`](synthesis.md)。关键改动：
- `mac_col` 加法树插入 1-2 级流水寄存器（100 → 250-400 MHz）
- `sdp` 拆成 3 拍组合路径（1 拍 shift + 1 拍 relu + 1 拍 clip）
- 相应把 `ofb_we_d2` 改成 `ofb_we_d3/d4`

---

## Phase 4 — 多核与系统级

### 多核 Mesh 互联

每个核 256 MAC，多核时用片上 mesh 互联做：
- **权重广播**：Cin 小、图大场景，权重分发到多核
- **激活广播**：Cout 大场景，激活值给到多核
- **Psum 流动**：Cin 和 Cout 都大，Psum 在核间接力累加

### 行流式跨层融合

IFB 容量小（Line Buffer 概念，8K~64K 字），不需要整层特征图存下。下游核攒够 K 行就启动，延迟从"整层"降到"K 行"。

### Task Descriptor + Router

编译器/调度器生成每个核的 Task Descriptor：
- 卷积配置（现有 cfg 寄存器）
- 数据路由（input 来源 / output 目的）
- 核间同步（`wait_mask`, `signal_done`）

核间 Router/DMA 负责把一个核的 OFB 输出搬到另一个核的 IFB。

### Element-wise Add（ResNet 残差）

融合进 SDP（见 Phase 2），或者放核外专门的 ElemAdd 单元。

---

## Phase 5 — 工程化 / 验证 / 编译器

- **真实 SRAM 替换**：`sram_model` → BRAM 原语（减少综合 RAMB 占用，精确建模时序）
- **UVM 验证平台**：从当前定向测试升级到约束随机测试，覆盖边界场景
- **性能模型（Roofline）**：建立算术强度 vs 带宽的 Roofline 分析，指导 tile size 选择
- **编译器完善**：`gen_isa_test.py` 扩展为真正的图编译器，支持多层网络（AlexNet / MobileNet / ResNet 子网）的端到端代码生成
- **多层自动调度器**：给定 ONNX 模型，自动切分层、分配核、生成 Task Descriptor 链

---

## 优先级建议

| 优先级 | 工作 | 理由 |
|--------|------|------|
| 高 | Padding | 任何实用 CNN 都要；NVDLA 风格零代价实现简单 |
| 高 | Residual + SDP 融合 | ResNet 是当代标准；SDP 已有基础设施 |
| 中 | 相邻 sub_op 预取融合 | 纯性能优化，收益 ~5-10% 利用率 |
| 中 | Pooling | 大部分 CNN 需要；实现简单 |
| 低 | Depthwise / 多核 | 复杂度大，需要完整系统级验证 |
