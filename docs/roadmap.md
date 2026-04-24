# 未来工作规划

## 已完成

- ✅ **握手驱动去中心化流水**（v1 → v3 迭代，MAC 利用率到 99.95%）
- ✅ **Macro-ISA → cfg-driven FSM**（已退役）
- ✅ **cross-round pipeline + parf_accum FILL/DRAIN overlap**
- ✅ **20-bit 内部地址**，支持大图单切片
- ✅ **chunked 权重调度**（K=7 kk=49 + rolling WRF pipeline）
- ✅ **DMA 子系统**（IDMA / WDMA / ODMA + axi_m_mux + axi_lite_csr）
- ✅ **Streaming row-ring 架构**（VGA 480×640 MAC 99.93%）
- ✅ **RTL 风格规范**（`RTL代码编写原则.md`）
- ✅ **多 case 回归**（单 vsim + 多 case reuse start/done）
- ✅ **Phase G：PyTorch 多层编译器**（`nn.Sequential` Conv2d+ReLU 链 bit-exact）
- ✅ **Phase H：MNIST all-conv 端到端部署验证**
- ✅ **Phase J-1：batch/streaming 数据路径统一**
- ✅ **Phase J-2：chunked 多 cout_slices bug 修复 + bias 位宽修复**
- ✅ **Phase J-3：packed/chunked 统一为单路径**（wgt_buffer 删除 packed 分支 ~200 行）
- ✅ **Phase J-3：Padding**（line_buffer 坐标越界判定，零代价硬件 padding）
- ✅ **Phase J-3：Cout/Cin > 16 切片**（streaming 支持任意通道数，v2 `cout_slices=1` 限制打破）
- ✅ **Phase K-1：K=1 PE 利用率优化**（TILE_W 均衡，消除 parf_accum 短尾 drain_stall）
- ✅ **Phase K-2：Ky-fold**（Cin < 16 时把 Ky 折到 cin_fake，纯软件，零 RTL 改动）
- ✅ **Phase K-3：Kx-fold**（Cout < 16 时把 Kx 折到 cout_fake，systolic psum shift）
  - `parf_col` per-col 存储拆分
  - `psum_reshape` 组合归约级
  - Stem 5.74× / L1 2.07× / 端到端 3.29× 加速，MAC util 22% → 77%

详见 [`pe-fold.md`](pe-fold.md)。

---

## 未来工作

### stride=1 滑窗复用

原 cfg-driven FSM 支持 stride=1 下 ARF 滑窗复用：`kx=0` 载入 `TILE_W=32` 像素，`kx=1..K-1` 每个只加载 1 个新像素。当前 streaming 架构没有此优化，IFB 读 = `TILE_W × K`。

- **K=3 stride=1**：可省 2/3 IFB 读
- **K=7 stride=1**：可省 6/7 IFB 读

主要是 IFB 带宽节约；当前 MAC utilization 已经很高，实际 cycle 收益有限。

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
| 高 | Residual + SDP 融合 | ResNet 是当代标准；当前 22 层 ResNet 回归仍把残差拆成软件加法 |
| 高 | 多核 Mesh 互联 | 单核已饱和 PE 利用率，下一步性能必须多核 |
| 中 | Pooling | 大部分 CNN 需要；实现简单（max/avg 比较累加）|
| 中 | 板级验证 | FPGA 平台选型 + RTL 冻结下板 |
| 中 | Depthwise Conv | MobileNet / EfficientNet 主导，现有 16×16 广播结构不匹配 |
| 低 | stride=1 滑窗复用 | IFB 带宽节约，次要但实用 |
| 低 | 时序优化 | 100 MHz 够用；拉到 400 MHz 需要 mac_col 加法树流水化 |
| 低 | 门控时钟 / 功耗优化 | 后期 PPA 优化项 |
