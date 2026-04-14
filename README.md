# FLUX CNN 加速器架构设计

本文档记录了 FLUX CNN 加速器从零到现在的完整设计历程，包含架构理念、模块说明、数据流、指令集、已验证功能以及未来规划。

---

## 目录

1. [设计理念与架构演进](#1-设计理念与架构演进)
2. [硬件模块总览](#2-硬件模块总览)
3. [数据复用策略](#3-数据复用策略)
4. [数据流动方式](#4-数据流动方式)
5. [Macro-ISA 指令集](#5-macro-isa-指令集)
6. [控制方式（微状态机）](#6-控制方式微状态机)
7. [已验证测试用例](#7-已验证测试用例)
8. [综合评估结果](#8-综合评估结果)
9. [仿真运行指南](#9-仿真运行指南)
10. [未来工作规划](#10-未来工作规划)

---

## 1. 设计理念与架构演进

### 1.1 从固定功能硬件到 Macro-ISA

早期设计依赖固定功能状态机处理卷积，边界处理、换行、步长等逻辑全部硬化在 RTL 中。当图像宽度与 1D 移位寄存器深度不匹配时，2D 换行会导致严重的数据污染。

架构随后转变为**软件调度驱动的 Macro-ISA**：

- **为什么用 ISA？** 2D 图像换行、任意 K×K 卷积核、任意步长全部由编译器（Python）显式调度，硬件只负责执行宏指令，彻底消除了边界处理硬逻辑。
- **为什么是"宏"指令？** 纯 VLIW 每条指令都要取指，而一次 SRAM 读取 ~50 pJ，一次 8-bit MAC 仅需 ~0.2 pJ。Macro-ISA 中一条 64-bit 宏指令（如 `OP_LD32MAC length=32`）只被取指一次，内部微状态机自动驱动 32 拍连续流水线，本质上将"取指税"均摊到数十上百拍计算中。
- **架构参考：** 设计思路与 Google TPU、Apache TVM/VTA 的 VLIW Tile 指令类似。

### 1.2 开发演进历程

| 阶段 | 提交 | 主要内容 |
|------|------|----------|
| 0 | `012294a` | 初始框架：固定功能 8×8 MAC 阵列 |
| 1 | `4fb1812` | 标准化寄存器堆（std_rf），统一读写接口 |
| 2 | `864b3ef` | Macro-ISA 双流水线优化：引入 OP_LD1MAC / OP_LD32MAC |
| 3 | `9a63633` | 标量寄存器文件 + 控制流指令（LI/ALU/JMP/BNZ）+ 多轮权重调度 + Vivado 综合 |
| 4 | `259fe10` | SDP 独立模块化：反量化（右移）+ ReLU + OP_LD_SDP 配置指令 |
| 5 | `48cbdfe` | Stride 支持：ISA 扩展 stride 字段 + 硬件寻址 + Python 编译器 |

---

## 2. 硬件模块总览

```
┌──────────────────────────────────────────────────────────────────────┐
│                          core_top                                     │
│                                                                        │
│  INST_SRAM ──► core_ctrl (微状态机)                                   │
│                    │                                                   │
│                    ├─► IFB (ifmap buffer, SRAM 64-bit×8192)           │
│                    │       │                                           │
│                    │       ▼ ifb_rdata (1-cycle latency)              │
│                    ├─► ARF (activation RF, 32-deep × 64-bit)          │
│                    │       │  LD32MAC bypass MUX (forward path)        │
│                    │       ▼ act_to_mac                                │
│                    ├─► mac_array (8 col × 8 PE)                       │
│                    │       │  每 PE 内含 WRF (32-deep × 8-bit)         │
│                    │       │  每 col 底部含 PARF (32-deep × 32-bit)    │
│                    │       ▼ psum_out_vec (8×32-bit)                  │
│                    └─► SDP (shift + ReLU + clip)                      │
│                              │                                         │
│                              ▼ ofb_wdata (8×8-bit)                    │
│                          OFB (outmap buffer, SRAM 64-bit×8192)        │
│                                                                        │
│  WB (weight buffer, SRAM 512-bit×1024) ──► mac_array.wrf_wdata       │
└──────────────────────────────────────────────────────────────────────┘
```

| 模块 | 文件 | 说明 |
|------|------|------|
| `core_ctrl` | `RTL/core_ctrl.sv` | 指令解码与微状态机，驱动所有控制信号 |
| `core_top` | `RTL/core_top.sv` | 顶层连线，含 bypass MUX |
| `mac_array` | `RTL/mac_array.sv` | 8×8 MAC 阵列，广播激活值，每列独立计算一个输出通道 |
| `mac_col` | `RTL/mac_col.sv` | 单列：8 个 PE + 加法树 + PARF 累加 |
| `mac_pe` | `RTL/mac_pe.sv` | 单 PE：WRF（std_rf）+ 乘法器（1 拍流水） |
| `std_rf` | `RTL/std_rf.sv` | 通用同步读/写寄存器堆（WRF / ARF / PARF 共用） |
| `sdp` | `RTL/sdp.sv` | 单数据处理器：反量化右移 + ReLU + 饱和截断至 [0,255] |
| `sram_model` | `RTL/sram_model.sv` | 行为级 SRAM（IFB / WB / OFB / INST），1-cycle 同步读 |
| `core_isa_pkg` | `RTL/core_isa_pkg.sv` | ISA 包：opcode 枚举 + inst_t 结构体 |

### 关键参数（当前默认值）

| 参数 | 值 | 说明 |
|------|----|------|
| NUM_COL / NUM_PE | 8 / 8 | 输出通道并行度 / 输入通道并行度 |
| WRF_DEPTH | 32 | 权重 RF 深度（5-bit 地址，最多存 32 个 kernel tap） |
| ARF_DEPTH | 32 | 激活值 RF 深度（对应 TILE_W=32 个空间像素） |
| PARF_DEPTH | 32 | 部分和 RF 深度（对应 32 个空间输出像素） |
| SRAM_DEPTH | 8192 | IFB / OFB 深度（限制 H×W ≤ 8192） |
| DATA_WIDTH | 8 | 激活值 / 权重位宽（int8） |
| PSUM_WIDTH | 32 | 部分和位宽（int32） |

---

## 3. 数据复用策略

加速器的高效性来自三个层次的数据复用：

### 3.1 权重静止复用（Weight Stationary，WS）

权重通过 `OP_LD_WGT` 一次性加载到每个 PE 内部的 WRF 中。在后续所有输出像素的 MAC 计算期间，权重保持不动。只有当 K²>32 需要多轮时，才在轮次边界重新加载。

```
              WRF[0..K*K-1] 在整个 yout 循环内静止
              ──────────────────────────────────────
  OP_LD_WGT (once per layer or per round)
      │
      ▼
  WRF 存放所有 kernel tap 权重
      │
      ├── yout=0: MAC 计算所有像素，WRF 保持不变
      ├── yout=1: MAC 计算所有像素，WRF 保持不变
      └── ...
```

**K²>32 时的多轮调度（K=7, 49 个 tap）：**
- Round 0：加载 tap[0..31]，遍历全部 yout 行
- Round 1：加载 tap[32..48]，再次遍历全部 yout 行
- ARF 状态在轮次间保持，不重置

### 3.2 激活值复用（ARF 滑窗，stride=1 专用）

对 stride=1 的卷积，`OP_LD32MAC` + `OP_LD1MAC` 组合实现 **ARF 滑动窗口**：

```
  kx=0: OP_LD32MAC  加载 pixel[x .. x+31] → ARF[0..31]
                    MAC 读 ARF[0..31]（对应 output[0..31]）

  kx=1: OP_LD1MAC   只加载 1 个新像素 pixel[x+32] → ARF[31]
                    MAC 读 ARF[1..32]（滑动窗口右移 1 格）

  kx=2: OP_LD1MAC   只加载 1 个新像素 pixel[x+33] → ARF[32]
                    MAC 读 ARF[2..33]

  kx=K-1: ...
```

每个新 kx 只需一次 IFB 读取（而非 32 次），**极大减少了 IFB 带宽需求**。

### 3.3 输出通道并行复用（Output Channel Broadcast）

同一组激活值 `act_to_mac`（64-bit，8 通道 int8）**广播**给所有 8 列，每列用各自的 WRF 权重独立计算一个输出通道的部分和。单次 IFB 读取产生 8 个输出通道的乘累加结果。

---

## 4. 数据流动方式

### 4.1 端到端数据流（stride=1, K=3, 单轮）

```
初始化阶段
──────────
OP_LD_SDP   →  SDP.shift_amt 寄存器写入（持久有效）
OP_LD_WGT   →  WB[0..K*K-1] → WRF[0..K*K-1]（所有 PE 同步写入）
OP_LI r0=0  →  scalar_rf[0] = IFB 基地址
OP_LI r1=0  →  scalar_rf[1] = OFB 基地址
OP_LI r2=H_OUT-1 → r2 = 行循环计数器

行循环体（每次迭代处理一行输出）
────────────────────────────────
for 每个 x_tile (通常只有 1 个 tile):
    for 每个 kernel tap (ky, kx):
        if kx==0:
            OP_LD32MAC  IFB[r0 + ky*W_IN + x_tile_in .. +31]
                        → ARF[0..31] (d1 流水写入)
                        → MAC 读 ARF[0..31]（bypass MUX 处理前向冲突）
                        → PARF 累加（clr_parf=1 仅在第一个 tap）
        else:
            OP_LD1MAC   IFB[r0 + ky*W_IN + x_tile_in + TILE_W + kx - 1]
                        → ARF[kx-1]（单像素写入，d1 流水）
                        → MAC 读 ARF[kx .. kx+31]（滑动窗口）
                        → PARF 累加

    OP_ST_OFM  sdp_en=1  →  PARF[0..W_OUT-1]
                         →  SDP（右移 + ReLU + clip）
                         →  OFB[r1 + x_tile_out .. +valid_w-1]

r0 += stride * W_IN    （IFB 基址向下移一步，stride=1 则 += W_IN）
r1 += W_OUT
BNZ r2, LOOP_START     （r2-- 并跳回，直到 r2 归零）
OP_FINISH
```

### 4.2 流水线对齐时序（关键路径）

```
周期 T:   core_ctrl 发出 ifb_re / ifb_raddr
周期 T+1: IFB SRAM 输出 ifb_rdata（1-cycle 读延迟）
          → ARF 写入（arf_we_d1 流水）
          → LD32MAC bypass MUX：若 arf_waddr == arf_read_addr，直接前向

周期 T+1 同步:
          MAC 读 act_to_mac → PE 乘法（1 拍流水）→ prod_out
          加法树（组合逻辑）→ adder_tree_reg（1 拍）→ PARF 累加（2 拍控制延迟对齐）

PARF 读出（ST_OFM）→ psum_out_vec（组合）→ SDP（组合）→ ofb_wdata
OFB 写地址延迟: ofb_waddr_d1 → ofb_waddr_d2（2 拍，与 psum_out_vec 对齐）
```

### 4.3 stride>1 的寻址变化

stride>1 时放弃 LD1MAC 滑窗优化，每个 (ky, kx) 均使用 LD32MAC，stride 字段编码在指令 [7:5]：

```
IFB 读地址 = r0 + sram_addr + cnt * eff_stride
其中 sram_addr = ky * W_IN + x_tile_in + kx
    x_tile_in = x_tile_out * stride
    eff_stride = (inst.stride == 0) ? 1 : inst.stride
```

r0 每行递增 `stride × W_IN`（输入行步进 stride 行）。

---

## 5. Macro-ISA 指令集

指令格式：64-bit，字段布局：

```
[63:60] opcode  [59] clr_parf  [58] sdp_en
[57:53] arf_addr   [52:48] wgt_rf_addr  [47:43] parf_addr
[42:28] sram_addr  [27:12] length  [11:8] reserved  [7:5] stride  [4:0] ld_arf_addr
```

### 指令表

| Opcode | 名称 | 功能 | 执行周期数 |
|--------|------|------|------------|
| 0 | `OP_NOP` | 空操作 | 1 |
| 1 | `OP_LD_WGT` | WB → WRF（批量加载权重） | length |
| 2 | `OP_LD_ARF` | IFB → ARF（批量加载激活值） | length |
| 4 | `OP_ST_OFM` | PARF → SDP → OFB（输出写回） | length |
| 5 | `OP_MAC_RUN` | 纯 MAC 计算（ARF 已就绪） | length |
| 6 | `OP_LD1MAC` | 加载 1 像素到 ARF[ld_arf_addr] + MAC 并行运行 | length |
| 7 | `OP_LD32MAC` | 加载 length 像素到 ARF + MAC 并行运行 | length+1 |
| 8 | `OP_LI` | scalar_rf[rd] ← imm16 | 0 (DECODE) |
| 9 | `OP_ALU` | scalar_rf[rd] ← rs1 ± rs2/imm16 | 0 (DECODE) |
| 10 | `OP_JMP` | PC ← target | 0 (DECODE) |
| 11 | `OP_BNZ` | if rs!=0: rs--, PC←target | 0 (DECODE) |
| 12 | `OP_LD_SDP` | SDP.shift_amt ← ld_arf_addr[4:0] | 0 (DECODE) |
| 15 | `OP_FINISH` | 停机，拉高 done | — |

**标量寄存器约定：**
- `r0` = IFB 基地址偏移（叠加到所有 IFB 读地址）
- `r1` = OFB 基地址偏移（叠加到所有 OFB 写地址）
- `r2`–`r7` = 通用（循环计数器、临时变量）

### 指令规模举例

| 配置 | 展开指令数 | 循环压缩后 | 压缩比 |
|------|-----------|------------|--------|
| K=3, H=10, W=10, stride=1 | ~2422 | 19 | ~127× |
| K=7, H=60, W=45, stride=1 | — | 112 | — |
| K=7, H=30, W=30, stride=2 | — | 60 | — |

---

## 6. 控制方式（微状态机）

`core_ctrl` 采用两段式 FSM（时序 + 组合分离）：

```
IDLE → FETCH → DECODE ─┬─► EXEC_LD_WGT  (cnt: 0..length-1)
                        ├─► EXEC_LD_ARF  (cnt: 0..length-1)
                        ├─► EXEC_MAC     (cnt: 0..length-1)
                        ├─► EXEC_ST_OFM  (cnt: 0..length-1)
                        ├─► EXEC_LD1MAC  (cnt: 0..length-1)
                        ├─► EXEC_LD32MAC (cnt: 0..length，多 1 拍预取)
                        ├─► FETCH        (NOP/LI/ALU/JMP/BNZ/LD_SDP，DECODE 内完成)
                        └─► FINISH
```

**关键控制流程：**

1. **FETCH**：发出 `inst_re`，从 INST_SRAM 读取指令（1-cycle 延迟）
2. **DECODE**：锁存指令到 `inst_reg`；标量指令（LI/ALU/JMP/BNZ/LD_SDP）在此状态内直接执行，零额外周期；EXEC 类指令跳转到对应状态
3. **EXEC_LD32MAC**：cnt=0 为 IFB 预取拍（无 MAC），cnt=1..length 为 IFB 写 ARF 与 MAC 重叠拍；利用模块级 `eff_stride`/`strided_off` 变量计算步进地址
4. **延迟流水管理**：
   - WRF 写：`wrf_we_d1`（1 拍，对齐 WB 读延迟）
   - ARF 写：`arf_we_d1`（1 拍，对齐 IFB 读延迟）
   - OFB 写：`ofb_we_d2`（2 拍，对齐 PARF 读 + 加法树延迟）

---

## 7. 已验证测试用例

所有测试在 ModelSim SE 2020.4 下通过，结果均为 **0 mismatch**。

| K | H_IN | W_IN | stride | shift | H_OUT×W_OUT | WRF 轮次 | 结果 |
|---|------|------|--------|-------|-------------|---------|------|
| 3 | 10 | 10 | 1 | 0 | 8×8=64 | 1 | PASS |
| 3 | 20 | 20 | 2 | 0 | 9×9=81 | 1 | PASS |
| 3 | 20 | 20 | 3 | 1 | 6×6=36 | 1 | PASS |
| 7 | 60 | 45 | 1 | 2 | 54×39=2106 | 2 | PASS |
| 7 | 30 | 30 | 2 | 2 | 12×12=144 | 2 | PASS |
| 1 | 123 | 45 | 1 | 0 | 123×45=5535 | 1 | PASS |

**SRAM 容量限制：** `H_IN × W_IN ≤ 8192`，超出时 Python 编译器报错退出。

---

## 8. 综合评估结果

目标器件：**Xilinx UltraScale+ xcku060-ffva1156-2-e**，OOC（Out-of-Context）模式，时钟 100 MHz。

### 资源利用

| 资源 | 使用量 | 器件总量 | 利用率 |
|------|--------|---------|--------|
| LUT | 17,415 | 331,680 | 5.25% |
| FF | 28,334 | 663,360 | 4.27% |
| RAMB36 | 156 | 1,080 | 14.44% |
| RAMB18 | 3 | — | — |
| DSP | 0 | — | 0%（乘法用 LUT 实现） |

> RAMB36 主导面积：156 块均来自 SRAM 模型（IFB/WB/OFB/INST 各占若干），这是行为级仿真 SRAM 被综合工具映射的结果。

### 功耗

| 类型 | 功耗 |
|------|------|
| 总片上功耗 | 1.046 W |
| 动态功耗 | 0.412 W |
| 静态功耗 | 0.633 W |
| Block RAM（动态） | 0.352 W（占动态功耗 85%） |
| CLB 逻辑 | 0.004 W |

### 时序

| 指标 | 值 |
|------|----|
| 时钟约束 | 10 ns（100 MHz） |
| WNS（最差负时序裕量） | +0.910 ns |
| 时序状态 | **全部满足** |

---

## 9. 仿真运行指南

**所有命令必须在 `sim/tb_core_isa/` 目录下执行。**

### 生成测试数据

```bash
cd sim/tb_core_isa

# 基本用法（K=3, 10×10 输入, stride=1）
python gen_isa_test.py --k 3 --h_in 10 --w_in 10 --stride 1 --shift 0

# 参数说明
python gen_isa_test.py \
    --h_in 60     # 输入高度（默认 123）
    --w_in 45     # 输入宽度（默认 45）
    --k 7         # 卷积核大小 K×K（默认 3）
    --stride 2    # 卷积步长 1~7（默认 1）
    --num_cin 8   # 输入通道数（默认 8）
    --num_cout 8  # 输出通道数（默认 8）
    --tile_w 32   # ARF 宽度（默认 32，勿改）
    --shift 2     # SDP 反量化右移位数 0~31（默认 0）
    --seed 42     # 随机种子（默认 42）
```

成功后生成：`inst.txt`（汇编）、`ifb.txt`（输入）、`wb.txt`（权重）、`expected_ofm.txt`（金标准）、`sim_params.f`（仿真参数）。

### 运行仿真

```bash
vsim -c -do tb_core_isa.tcl
```

携带`-c`参数为命令行运行，如果需要打开窗口看波形，去掉`-c`即可。

仿真结束后输出验证结果及性能统计（总周期数、MAC 利用率、各 SRAM 读写次数、RF 流量）。

### 快速批处理

```bash
do.bat
```

---

## 10. 未来工作规划

### 10.1 算子支持扩展

| 优先级 | 项目 | 说明 |
|--------|------|------|
| 高 | **Depthwise Conv（DW）** | 每个通道独立卷积，MAC 阵列利用率与通道分配需重新调度 |
| 高 | **Pooling（Max/Avg）** | 需新增 OP_POOL 指令及对应比较/累加单元 |
| 中 | **Pointwise Conv (1×1)** | 已支持（K=1 单轮），可验证大 Cin/Cout 场景 |
| 中 | **SDP 扩展：BN fuse** | 将 BN 的 scale/bias 融合到反量化流程（scale×psum+bias→shift） |
| 低 | **Padding 支持** | 编译器在 IFB 边界填零，或硬件在地址越界时自动返回 0 |

### 10.2 性能与效率提升

| 项目 | 说明 |
|------|------|
| **Cin/Cout > 8 支持** | 多轮 Cin 累加（Cin 切片，PARF 跨切片叠加）；Cout 切片（多次 ST_OFM 写不同 OFB 区域） |
| **ARF/PARF 深度扩展** | 当前 DEPTH=32；扩展到 64 可减少 LD32MAC 次数（特别对 TILE_W>32 场景） |
| **流水线深度优化** | 加法树（8-PE 求和）当前全组合；插入流水寄存器以提升时钟频率至 500 MHz+ |
| **双 bank PARF** | 读写 bank 分离，允许 ST_OFM 与下一条 MAC 流水重叠，提升吞吐 |
| **IFB 预取缓冲** | 在 core_ctrl 内增加小型预取队列，隐藏 SRAM 读延迟 |

### 10.3 多核扩展

| 项目 | 说明 |
|------|------|
| **多核 Mesh 互联** | 核间低功耗寄存器路由（~1 pJ/hop），支持 Psum 流动（核间接力累加，避免大 SRAM Read-Modify-Write） |
| **灵活数据流映射** | Weight Stationary / Output Stationary / Row Stationary 可通过编译器切换，无需改 RTL |
| **多核同步控制** | 广播权重（小 Cin 大图）/ 广播激活值（大 Cout）/ Psum 流动（大 Cin 大 Cout）三种调度模式 |

### 10.4 工程化与验证

| 项目 | 说明 |
|------|------|
| **真实 SRAM 替换** | 将行为级 sram_model 替换为 BRAM 原语（减少综合 RAMB 占用） |
| **UVM 测试平台** | 从当前 directed test 升级为约束随机验证，覆盖边界场景 |
| **性能模型（Roofline）** | 建立算术强度与带宽的 Roofline 分析，指导 tile size 选择 |
| **编译器完善** | gen_isa_test.py 扩展为真正的图编译器，支持多层网络（AlexNet / MobileNet 子网）的端到端代码生成 |
