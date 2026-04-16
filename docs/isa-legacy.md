# Macro-ISA 方案（已退役）

> **状态**：已废弃。架构演进两次：
>
> 1. Macro-ISA（64-bit 宏指令 + 取指译码）
> 2. cfg-driven FSM（单 FSM 读配置寄存器自驱动 6 层循环，无取指）— 也已退役
> 3. **当前**：[valid-ready 握手驱动的去中心化流水](handshake-pipeline.md)
>
> 本文档保留 Macro-ISA 历史。cfg-driven FSM 阶段的 `core_ctrl.sv` 还在磁盘但已从 file list 剔除。

---

## 1. 设计背景

项目最初采用 Macro-ISA（宏指令集架构）方案，由软件编译器生成 64-bit 宏指令流，硬件取指/译码/执行：

- **为什么用 ISA？** 2D 图像换行、任意 K×K 卷积核、任意步长全部由编译器（Python）显式调度，硬件只负责执行宏指令，彻底消除了边界处理硬逻辑。
- **为什么是"宏"指令？** 纯 VLIW 每条指令都要取指，而一次 SRAM 读取 ~50 pJ，一次 8-bit MAC 仅需 ~0.2 pJ。Macro-ISA 中一条 64-bit 宏指令（如 `OP_LD32MAC length=32`）只被取指一次，内部微状态机自动驱动 32 拍连续流水线，本质上将"取指税"均摊到数十上百拍计算中。
- **架构参考**：设计思路与 Google TPU、Apache TVM/VTA 的 VLIW Tile 指令类似。

---

## 2. 为什么放弃 ISA

随着功能扩展（padding、pooling、residual add），指令序列复杂度爆炸：

| 配置 | 当时指令数 | 加 padding 后估计 |
|------|-----------|-------------------|
| C16C16 K=3 66×118 | ~86 | 边界行需要不同指令序列，无法循环压缩，~300+ |
| C64C32 K=3 66×118 | ~500+ | ~1500+，逼近 INST_SRAM=8192 |

单核场景是"跑单层标准卷积"，ISA 的灵活性用不上。既然要加 H/W/Pad 参数、多核 Task Descriptor，那和配置寄存器已经没有本质区别了。

---

## 3. ISA 指令格式（64-bit）

```
[63:60] opcode   [59] clr_parf   [58] sdp_en
[57:53] arf_addr  [52:48] wgt_rf_addr  [47:43] parf_addr
[42:28] sram_addr  [27:12] length   [11:8] rsvd
[7:5] stride  [4:0] ld_arf_addr
```

### 指令表

| Opcode | 名称 | 功能 | 执行周期数 |
|--------|------|------|------------|
| 0 | `OP_NOP` | 空操作 | 1 |
| 1 | `OP_LD_WGT` | WB → WRF | length |
| 2 | `OP_LD_ARF` | IFB → ARF | length |
| 4 | `OP_ST_OFM` | PARF → SDP → OFB | length |
| 5 | `OP_MAC_RUN` | 纯 MAC 计算 | length |
| 6 | `OP_LD1MAC` | 加载 1 像素 + MAC | length |
| 7 | `OP_LDnMAC` | 加载 n 像素 + MAC | length+1 |
| 8 | `OP_LI` | scalar_rf[rd] ← imm16 | 0 (DECODE) |
| 9 | `OP_ALU` | 标量 ±，rd ← rs1±rs2/imm | 0 (DECODE) |
| 10 | `OP_JMP` | PC ← target | 0 (DECODE) |
| 11 | `OP_BNZ` | if rs!=0: rs--, PC←target | 0 (DECODE) |
| 12 | `OP_LD_SDP` | SDP.shift_amt ← ld_arf_addr[4:0] | 0 (DECODE) |
| 15 | `OP_FINISH` | 停机 | — |

### 标量寄存器约定

- `r0` = IFB 基地址偏移（叠加到所有 IFB 读地址）
- `r1` = OFB 基地址偏移（叠加到所有 OFB 写地址）
- `r2`–`r7` = 通用（循环计数器、临时变量）

### 指令规模示例

| 配置 | 展开指令数 | 循环压缩后 | 压缩比 |
|------|-----------|------------|--------|
| K=3, H=10, W=10, stride=1 | ~2422 | 19 | ~127× |
| K=7, H=60, W=45, stride=1 | — | 112 | — |
| K=7, H=30, W=30, stride=2 | — | 60 | — |

---

## 4. 原状态机

`core_ctrl` 两段式 FSM：

```
IDLE → FETCH → DECODE ─┬─► EXEC_LD_WGT  (cnt: 0..length-1)
                        ├─► EXEC_LD_ARF  (cnt: 0..length-1)
                        ├─► EXEC_MAC     (cnt: 0..length-1)
                        ├─► EXEC_ST_OFM  (cnt: 0..length-1)
                        ├─► EXEC_LDnMAC  (cnt: 0..length，多 1 拍预取)
                        ├─► FETCH        (NOP/LI/ALU/JMP/BNZ/LD_SDP，DECODE 内完成)
                        └─► FINISH
```

3 级指令流水（IF → ID → EX）：EX 末尾提前发 IF，`cnt == end_cnt-2` 触发预取，`cnt == end_cnt-1` 锁存到 `pf_inst`（完成 ID），`cnt == end_cnt` EX 结束直通下一条指令的 EX。分支和短指令（`end_cnt < 2`）回退到普通 FETCH 路径。

---

## 5. 原开发演进历程

| 阶段 | 提交 | 主要内容 |
|------|------|----------|
| 0 | `012294a` | 初始框架：固定功能 8×8 MAC 阵列 |
| 1 | `4fb1812` | 标准化寄存器堆（std_rf），统一读写接口 |
| 2 | `864b3ef` | Macro-ISA 双流水线优化：引入 OP_LD1MAC / OP_LD32MAC |
| 3 | `9a63633` | 标量寄存器文件 + 控制流指令 + 多轮权重调度 + Vivado 综合 |
| 4 | `259fe10` | SDP 独立模块化：反量化（右移）+ ReLU + OP_LD_SDP 配置指令 |
| 5 | `48cbdfe` | Stride 支持：ISA 扩展 stride 字段 + 硬件寻址 + Python 编译器 |
| 6 | `33aa2e2` | PE 阵列扩展为 16×16（256 MACs） |
| 7 | `0fc21d0` | 支持 Cin/Cout < PE 宽度的子通道配置 |
| 8 | `be0b0bc` | 支持 Cout > HW_COL 的输出通道分片 |
| 9 | `7a4d616` | 支持 Cin > HW_PE 的输入通道分片（ISA 方案最终版本） |
| 10 | (当前) | ISA → 配置寄存器 + 自驱动 FSM，轮次分块，20-bit 地址 |

---

## 6. ISA 方案回归基线（历史）

以下是 16×16 MAC 阵列 + ISA 方案的最终回归结果，保存在 `sim/tb_core_isa/regression_16x16.txt`：

| 用例 | Cycles | MAC% |
|------|--------|------|
| K=3 10×10 s=1 | 798 | 72.18% |
| K=3 20×20 s=2 | 985 | 74.01% |
| K=7 60×45 s=1 | 116,327 | 88.71% |
| K=7 100×80 s=1 | 376,199 | 90.60% |
| K=7 68×120 s=1 | 378,211 | 91.57% |

配置寄存器方案在同配置下略优（详见 [simulation.md](simulation.md) 的对比表）。

---

## 7. 若要回顾 ISA 代码

ISA 方案的代码已从 `main` 分支删除。git 历史可查 `7a4d616` 及之前的 commit：

```bash
git log --all --follow -- RTL/core_ctrl.sv RTL/core_isa_pkg.sv
git checkout 7a4d616 -- RTL/core_ctrl.sv RTL/core_isa_pkg.sv  # 临时恢复到工作区
```

或者查 `history.md` 有设计演进时的完整对话记录（含 ISA → 配置寄存器的决策讨论）。
