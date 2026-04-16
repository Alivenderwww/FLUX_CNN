# 硬件架构

## 1. 顶层数据通路

```
┌──────────────────────────────────────────────────────────────────────┐
│                          core_top                                    │
│                                                                      │
│  cfg_regs  ──►  core_ctrl (自驱动 FSM)                               │
│      │               │                                               │
│      │               ├─► IFB (ifmap buffer, 128b × SRAM_DEPTH)       │
│      │               │        │                                      │
│      │               │        ▼ ifb_rdata (1-cycle latency)         │
│      │               ├─► ARF (activation RF, 32 × 128b)              │
│      │               │        │  bypass MUX: arf_waddr==arf_read_addr│
│      │               │        ▼ act_to_mac (128b)                   │
│      │               ├─► mac_array (16 col × 16 PE = 256 MACs)       │
│      │               │        │  每 PE 内含 WRF (32 × 8b)            │
│      │               │        │  每 col 底部含 PARF (32 × 32b)       │
│      │               │        ▼ psum_out_vec (16 × 32b)              │
│      │               └─► SDP (>> shift, ReLU, clip to [0,255])       │
│      │                         │                                     │
│      │                         ▼ ofb_wdata (16 × 8b)                 │
│      │                    OFB (outmap buffer, 128b × SRAM_DEPTH)     │
│      │                                                               │
│      └─► WB (weight buffer, 2048b × SRAM_DEPTH) ──► mac_array        │
└──────────────────────────────────────────────────────────────────────┘
```

TB 通过层次引用（`u_core_top.u_ctrl.cfg_*`）直接写配置寄存器，拉 `start`，等 `done` —— 无指令总线、无外部 AXI 接口。

---

## 2. 模块清单

| 模块 | 文件 | 说明 |
|------|------|------|
| `core_top` | `RTL/core_top.sv` | 顶层连线；含 LD32MAC bypass MUX |
| `core_ctrl` | `RTL/core_ctrl.sv` | 配置寄存器 + 6 层循环 FSM，驱动所有控制信号 |
| `mac_array` | `RTL/mac_array.sv` | 16×16 MAC 阵列；激活值跨列广播 |
| `mac_col` | `RTL/mac_col.sv` | 单列：16 PE + 加法树 + PARF 时间累加 |
| `mac_pe` | `RTL/mac_pe.sv` | 单 PE：WRF + 1 拍流水乘法器 |
| `std_rf` | `RTL/std_rf.sv` | 通用同步写/组合读 RF（WRF/ARF/PARF 共用） |
| `sdp` | `RTL/sdp.sv` | 后处理：反量化右移 → ReLU → 饱和到 uint8 |
| `sram_model` | `RTL/sram_model.sv` | 行为级 SRAM（1 拍读延迟），综合映射到 RAMB36 |

原 `core_isa_pkg.sv`（ISA 包）已随 Macro-ISA 方案退役删除。

---

## 3. 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `NUM_COL` / `NUM_PE` | 16 / 16 | 输出通道并行度 / 输入通道并行度 |
| `WRF_DEPTH` | 32 | 权重 RF 深度（5-bit 地址，最多 32 个 kernel tap 或跨 cins 拼装） |
| `ARF_DEPTH` | 32 | 激活值 RF 深度（= TILE_W 空间像素数） |
| `PARF_DEPTH` | 32 | 部分和 RF 深度（= TILE_W 输出像素数） |
| `SRAM_DEPTH` | 8192（TB 可 override 到 ≤ 65536） | IFB / WB / OFB 深度 |
| `ADDR_W` | 20 | `core_ctrl` 内部地址/指针位宽 |
| `DATA_WIDTH` | 8 | 激活值 / 权重位宽（int8） |
| `PSUM_WIDTH` | 32 | 部分和位宽（int32） |
| `TILE_W` | 32（固定，由 ARF_DEPTH 决定） | 空间 tile 宽度 |

ARF_DEPTH=32 的值**不是**随便选的 —— 5-bit ARF 地址在 stride=1 滑窗模式下依赖自然回绕（arf[32] = arf[0]）。改动 ARF_DEPTH 需要重新设计 `sub_arf_rd_base` / `sub_arf_wr_base` 的逻辑。

---

## 4. 三层数据复用

### 4.1 权重静止复用（Weight Stationary）

WRF 一次加载，在整个 `yout × tile × cins × ky × kx` 循环内保持不动。

- **Packed 模式**（`K² × cin_slices ≤ 32`）：每个 `cs` 开头加载一次 `cfg_total_wrf` 个权重，所有 cins 的权重都驻留 WRF。`wrf_raddr = cins·K² + ky·K + kx`。
- **Chunked 模式**（`K² × cin_slices > 32` 但 `K² ≤ 32`）：每个 `(cins)` 开头加载一次 `K²` 个权重。`wrf_raddr = ky·K + kx`（相对每 cins 的 WRF 重装）。
- **Chunked with rounds 模式**（`K² > 32`，即 K ≥ 7）：每个 `(cins, round)` 加载 `cur_round_len` 个权重（除末轮外均为 32）。`wrf_raddr = pos_in_round`。

### 4.2 ARF 滑动窗口（stride=1 + 整 tile）

对 stride=1 的整 tile 卷积，`kx=0` 加载 `TILE_W=32` 个像素到 ARF[0..31]；`kx=1..K-1` 每个只加载 **1 个新像素** 滑入。MAC 按 `arf_read_addr = kx + (cnt-1)` 读，利用 5-bit ARF 地址的自然回绕：

```
kx=0:  load pixel[0..31]  → arf[0..31]     MAC reads arf[0..31]
kx=1:  load pixel[32]      → arf[0]         MAC reads arf[1..32]=arf[1..31,0]
kx=2:  load pixel[33]      → arf[1]         MAC reads arf[2..33]=arf[2..31,0,1]
...
```

相比每个 kx 都全载 32 像素（K×TILE_W 次 IFB 读），滑窗模式只需 `TILE_W + (K-1)` 次 IFB 读 —— **K=3 节省 49%，K=7 节省 79% 的 IFB 带宽**。

Partial tile（`valid_w < TILE_W`）或 `stride > 1` 时关闭滑窗，所有 kx 全载。

### 4.3 输出通道广播（Output Channel Broadcast）

每拍一次 IFB 读产生一个 128-bit `act_to_mac`（16 通道 int8），**同时广播给所有 16 列**。每列用自己 PE 内的 WRF 权重独立计算各自通道的部分和。单次 ARF/IFB 读 → 16 个输出通道的 MAC 结果。

---

## 5. LD32MAC Bypass MUX（core_top 内部）

IFB 读延迟 1 拍，ARF 写也是延迟 1 拍（`arf_we_d1`）。`cnt=0` IFB 读 → `cnt=1` ARF 写。但 MAC 也在 `cnt=1` 读 ARF 相同位置（读新装入的像素）。ARF 是"同写同读"同周期场景，`std_rf` 的组合读端口会返回**旧值**（当前周期写尚未生效）。

Bypass MUX 在 `core_top` 内解决：

```systemverilog
always_comb begin
    if (arf_we && (arf_waddr == arf_read_addr))
        act_to_mac = ifb_rdata;   // 直接前向 IFB 输出给 MAC
    else
        act_to_mac = act_out_vec; // 正常从 ARF 读
end
```

这是**功能正确性的关键**，不是优化 —— 没有它，滑窗模式和 `kx=0` 的首次 MAC 都会读错。
