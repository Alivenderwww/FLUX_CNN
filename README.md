# OpenCode CNN 神经网络加速器架构设计

本文档总结了 OpenCode CNN 加速器的核心设计理念、硬件模块功能以及数据流流动模式。

## 1. 设计理念 (Design Philosophy)

### 1.1 从固定功能硬件到 Macro-ISA 的架构演进
在早期的设计中，加速器依赖于僵化的、基于状态机的固定功能（Fixed-Function）架构来处理卷积（包括 Padding、步长和滑动窗口）。然而，当图像宽度与内部 1D 移位寄存器深度不匹配时，这种方式在处理 2D 图像换行时会导致严重的数据污染问题。

为了解决这一问题，我们的架构转变为**软件调度驱动的 Macro-ISA（宏指令集架构）**。
- **为什么选择 ISA？** 它彻底消除了复杂的边界处理和步长计算硬件逻辑。2D 图像的换行现在由编译器通过发出显式的加载/移位指令来控制，这不仅解决了数据对齐问题，还原生支持了任意的数据流映射（Dataflow）以及多核（Multi-core）同步。
- **攻破“指令提取税（Instruction Fetch Tax）”：** 纯 VLIW（超长指令字）芯片会将 90% 以上的能量浪费在从 SRAM 提取指令上（取指约 10~50pJ，而一次 8-bit MAC 仅需 0.2pJ）。为了避开高昂的取指代价，我们采用了 **Macro-ISA** 策略（类似于 Google TPU 或 Apache TVM 的 VTA）。一条 64-bit 的宏指令（例如 `OP_MAC_RUN length=32`）只需被抓取和解码一次，其内部的微状态机即可自动驱动长达 32 拍的无缝流水线计算。

### 1.2 高能效的数据搬移
在 SRAM 中搬移数据的代价是极其昂贵的。在我们的多核 Mesh 架构中，核心间的路由利用低功耗寄存器（~1pJ）而非深层 SRAM（~100pJ）。这种 Mesh 路由网络允许关键的部分和（Psum）在核心本地进行累加，避免了与中央大 SRAM 之间代价高昂的 Read-Modify-Write 循环，同时也极大地便利了数据在阵列中的广播。

---

## 2. 核心硬件模块 (Hardware Modules)

单核（Single Core）的拓扑结构围绕一个居中的 8x8 处理单元（PE）阵列和几个专用的 SRAM/寄存器堆（RF）构建：

- **`core_ctrl` (指令解码与微状态机控制器):** 核心的大脑。它从 `INST_SRAM` 抓取 64-bit 指令进行解码，并驱动底层计算流水线（`FETCH` -> `DECODE` -> `WAIT/SHIFT` -> `EXEC`）。
- **IFB (Ifmap Buffer):** 存储输入特征图（Input Feature Maps）的高速缓存。
- **ARF (Activation Register File - `arf_buffer`):** 一个深度为 32 的移位寄存器阵列，用于存放输入激活值。在卷积计算期间，它在每个时钟周期将像素数据向左移位并喂给 MAC 阵列，优雅地实现了滑动窗口的 1D 空间展开。
- **PE Array (`mac_array` & `mac_pe`):** 8x8 的乘加运算（MAC）阵列。
  - **WRF (Weight Register File - `std_rf`):** 位于每个 PE 内部，是一个深度为 32 的标准寄存器堆，用于存放权重。在连续的 MAC 计算期间，权重保持静止（Weight Stationary 模式）。
- **PARF (Partial Sum Adder RF - `mac_col` & `std_rf`):** 位于 PE 阵列的底部/列端。它在多个周期内用于暂存并累加 32-bit 的部分和（Psums）。
- **SDP (Single Data Processor):** 单数据处理器，在数据离开核心前进行硬件后处理。受 `sdp_en` 指令标志位控制，它执行 ReLU（将负数部分和置 0）以及 8-bit 饱和/截断（将大于 255 的值钳位至 255）。
- **OFB (Output Feature Map Buffer):** 输出特征图缓存。经过 SDP 处理后，最终的 8-bit 输出激活值会写回至此。

---

## 3. 核心数据流与执行模型 (Core Dataflow)

CNN Core 基于编译器（`gen_isa_test.py`）调度的 **1D 空间展开（1D Spatial Unroll）** 序列运行。

1. **软件分块 (Software Tiling):** Python 编译器将庞大的 2D 特征图（例如 123x45）在宽度维度切割为长度为 32 的 1D 数据块（与 ARF 深度匹配）。
2. **数据加载 (Load):** `OP_LD_WGT` 指令将卷积核权重加载到 PE 内部的 WRF 中。`OP_LD_ARF` 指令从 IFB 中读取 32 个像素加载至 ARF 阵列。
3. **流水线执行 (Execute):** `OP_MAC_RUN` 指令命令核心启动连续的计算流水线（例如执行 32 拍）。在这期间，ARF 逐拍移位，PE 将激活值与权重相乘，结果持续在 PARF 中累加。
4. **输出写回与边界处理 (Output & Boundary Handling):** 硬件会“盲目”地执行完 32 拍流水线，但 `OP_ST_OFM`（存储输出特征图）指令利用了 `length` 参数。它通过仅将数学上有效的宽度数据写回 OFB，安全地丢弃了 32 拍运行中因为边界 Padding 而产生的无效垃圾部分和。

---

## 4. 多核 Mesh 互联与复用策略 (Multi-Core Mesh Strategy)

当扩展到多核 Mesh 拓扑时，架构能够根据当前层的维度规模（$C_{in}$, $C_{out}$, $H$, $W$）支持极其灵活的路由策略：

1. **小 $C_{in}$, 小 $C_{out}$ (例如 8x8):**
   - **策略:** 空间维度切分 + 权重广播 (Spatial Tiling + Weight Broadcast)。
   - 图像在空间上被切片（左上/右上/左下/右下）。相同的权重被广播给 Mesh 中的所有 Core。
2. **小 $C_{in}$, 大 $C_{out}$ (例如 8x32):**
   - **策略:** 输入广播 + 输出通道切分 (Input Broadcast + Output Channel Tiling)。
   - 相同的 IFM 像素被广播给所有 Core。每个 Core 加载不同的权重，独立计算 $C_{out}$ 通道的不同切片。
3. **大 $C_{in}$, 大 $C_{out}$ (例如 32x32):**
   - **策略:** 输入通道切分 + 输出复用 / Psum 流动 (Input Channel Tiling + Output Reuse)。
   - 不同的 Core 被分配不同的 $C_{in}$ 深度切片。计算得到的部分和（Psums）不再写回中央大 SRAM，而是通过 Mesh 网络流动（Core 0 $\rightarrow$ Core 1 $\rightarrow$ ...），在飞行的过程中完成接力累加，彻底避免了海量 SRAM 读写带来的能量惩罚。
4. **2D 终极混合数据流 (2D Mixed Dataflow):**
## 5. 项目文件结构与运行指南 (Project Structure & Run Instructions)

本工程在Windows环境下开发，使用 ModelSim 进行仿真验证。

### 5.1 目录结构 (Directory Structure)
- `RTL/` : 存放所有 SystemVerilog 硬件核心代码。
- `sim/` : 仿真目录。
  - `sim/tb_core_isa/` : 核心指令集验证模块。**注意：所有的测试数据生成与仿真必须在此目录下执行！**
    - `gen_isa_test.py` : Python 数据生成与编译器脚本，用于生成汇编指令 (`inst.txt`) 和相关读写数据文本。
    - `tb_core_isa.sv` : 核心 Testbench 文件。
    - `tb_core_isa.tcl` : ModelSim 仿真脚本。
    - `do.bat` : 包含完整的编译与运行 ModelSim 的快捷批处理脚本。

### 5.2 运行仿真步骤 (How to Run)

**非常重要：所有命令必须在 `sim/tb_core_isa` 目录下执行！** 否则 Python 脚本会将巨大的 `.txt` 数据文件错误地生成在工程根目录，并且 ModelSim 将找不到这些文件而导致仿真失败或跑飞（Hang）。

1. **进入工作目录**
   ```bash
   cd sim/tb_core_isa
   ```

2. **运行 Python 编译器生成数据**
   执行 Python 脚本，它会将卷积循环解卷为 Macro-ISA 汇编，并生成相应的特征图与权重数据：
   ```bash
   python gen_isa_test.py
   ```
   *成功后会输出提示信息，并生成多个 `*.txt` 数据文件（`inst.txt`, `ifb.txt`, `wb.txt` 等）。*

3. **运行 ModelSim 仿真**
   ```bash
   vsim -c -do tb_core_isa.tcl
   ```
   仿真结束后，控制台将输出对齐验证的结果（如 `0 mismatches`）以及详细的性能与寄存器堆 (RF) 资源利用率报告。