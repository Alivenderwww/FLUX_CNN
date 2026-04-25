# 未来工作

## 已完成

### 架构 / 数据通路
- 握手驱动去中心化流水（line_buffer / wgt_buffer / mac_array / parf_accum / ofb_writer，valid-ready 对齐）
- cross-round pipeline + parf_accum FILL/DRAIN overlap
- 20-bit 内部 SRAM 地址
- chunked 权重调度（K² > WRF_DEPTH 时按 round 切片，cins-ahead 流水）
- streaming row-ring 架构（IFB / OFB ring 行级反压，IDMA / core / ODMA 三阶段并发）
- 任意 Cin / Cout 切片支持（cin_slices / cout_slices > 1）
- Padding（line_buffer 坐标越界判定，零 RTL 代价）

### DMA / AXI
- DMA 子系统：IDMA / WDMA / ODMA / DFE + axi_m_mux + axi_lite_csr
- descriptor 驱动：DFE 拉取 + desc_fifo + sequencer 分发

### 编译 / 端到端
- PyTorch 多层编译器：`nn.Sequential` Conv2d+ReLU 链 bit-exact
- MNIST all-conv 端到端部署验证

### PE 利用率优化
- K=1 layer TILE_W 均衡（消除 parf_accum 短尾 drain_stall）
- Ky-fold（Cin<16 时把 Ky 折到 cin_fake，编译器侧）
- Kx-fold（Cout<16 时把 Kx 折到 cout_fake，systolic psum shift；含 parf_col 拆分 + psum_reshape 归约）
- Space-to-Depth（stride≥2 时把 4 相位折到 cin，编译器侧重排无复制）

### 工具链
- 多 case 单 vsim 回归
- handshake profile 报告（每 case 4 个 V/R 接口的 fire/stall/idle 统计）

详见 [`pe-fold.md`](pe-fold.md) / [`simulation.md`](simulation.md) / 各模块文档 [`modules/`](modules/)。

---

## 未来工作

### Residual Add（SDP 融合）

扩展 `sdp.sv` 在 shift 之前加一路残差加法：

```systemverilog
input logic signed [NUM_COL*PSUM_WIDTH-1:0] residual_in;
input logic                                 residual_en;
combined_ch[c] = residual_en ? (psum_ch[c] + residual_ch[c]) : psum_ch[c];
```

需要从另一块 SRAM 读残差，扩展 `ofb_writer` 或新模块。cfg 加 `cfg_sdp_res_en` / `cfg_res_base`。当前 22 层 ResNet 回归把残差拆成软件加法。

### Pooling（Max / Avg）

- cfg 加 `pool_mode`
- `sdp` 或独立 pool 模块实现 max 比较和 avg 累加
- `ofb_writer` 按 pool stride 调整写地址步进

### Depthwise Conv

每通道独立卷积，16×16 广播结构不适用。可选：
- 每列独占一个输入通道 + 一个输出通道，关闭广播
- `DW_MODE` 把 K² 个 tap 展平到 16 PE 的工作

### Kx systolic tail 跨 tile 重叠

Stem K=8 stride=2 fold 路径里测得 ~7% util gap 来自 Kx-fold 的 head + tail 列 mask（每 tile 4 partial 拍）。把当前 tile 的 tail 拍和下一 tile 的 head 拍时间叠合（让 group 0 提前算下一 tile、group 1 还在补当前 tile 末尾）可消掉这部分。需要 PARF 同时存两 tile 的 psum、wgt_buffer 给两组提供不同 (kx_v, ky_local) 序列、line_buffer iss_pos 跨 tile 连续推进。

---

## 时序优化（频率）

100 MHz 下当前 RTL 的全组合路径都不卡。拉到 250-400 MHz 需要：

- mac_col 加法树拆成 2-3 级流水（16-PE 求和 → 8+8 → 4+4 + 终和）；相应 `mac_array` 的 pipe valid 追踪从 2 级扩到 3-4 级
- `sdp` 拆 1 shift + 1 relu + 1 clip 三级，`ofb_writer` 的 OFB 写地址延后 2-3 拍对齐

---

## 多核与系统级

### 多核 Mesh 互联

每核 256 MAC，多核场景：
- 权重广播：Cin 小 + 图大时，权重分发到多核
- 激活广播：Cout 大时
- Psum 流动：Cin 和 Cout 都大，Psum 在核间接力累加

### 行流式跨层融合

下游核攒够 K 行就启动，延迟从"整层"降到"K 行"。利用现有 streaming row-ring 反压，无需新硬件机制。

### Task Descriptor + Router

每核独立 descriptor 队列：卷积 cfg、数据路由（input 来源 / output 目的）、核间同步（`wait_mask` / `signal_done`）。

---

## 验证 / 工程化

- 真实 SRAM 替换：`sram_model` → BRAM 原语
- UVM 验证平台：定向 → 约束随机
- 性能模型（Roofline）：算术强度 vs 带宽
- 多层自动调度器：ONNX → 多层 descriptor 链

---

## 优先级建议

| 优先级 | 项 | 备注 |
| --- | --- | --- |
| 高 | Residual + SDP 融合 | ResNet 主流必备；当前残差走软件 |
| 高 | 多核 Mesh 互联 | 单核已饱和，下一阶段性能依赖多核 |
| 中 | Pooling | 多数 CNN 需要 |
| 中 | Kx tail 跨 tile 重叠 | 收益 ~7% util，仅 fold 场景受益 |
| 中 | 板级验证 | FPGA 平台选型 + RTL 冻结下板 |
| 中 | Depthwise Conv | MobileNet / EfficientNet 主导 |
| 低 | 频率提升（mac_col 加法树流水化） | 100 MHz 够用 |
| 低 | 门控时钟 / 功耗优化 | 后期 PPA |
