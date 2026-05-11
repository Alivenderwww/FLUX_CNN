# 面向端侧流式计算场景的卷积加速器设计

**Convolutional Accelerator Design for Edge-Side Streaming Computation Scenarios**

## 4 加速器具体实现

### 4.1 引言

本章按模块顺序给出 FLUX_CNN 加速器的具体硬件与编译器实现。首先介绍加速核内 5 个数据通路模块的内部结构与实现流程，依次为行缓存模块、MAC 阵列模块、部分和累加模块、SDP 后处理模块、权重缓存模块；接着介绍配置寄存器与 6 层嵌套 FSM 的实现；随后介绍编译器侧的 Ky 折叠与空间到深度（S2D）两项 PE 利用率优化；然后介绍 DMA 子系统、AXI 接口集成以及多核 W 切片扩展层；最后介绍 16×16 INT8 MAC 阵列在 DSP48E1 块上的跨列复用映射方案。

### 4.2 行缓存模块

行缓存模块（line_buffer）承担将片外像素流转换为片上 *K*×*K* 卷积窗口的职能，是片外 IFM 与 MAC 阵列之间的桥梁。如图 4.1 所示行缓存模块结构图。

**图 4.1 行缓存模块结构图**
**Figure 4.1 Line-buffer module structure**

行缓存模块的存储核心是一块容量为 IFB=8192 word 的输入特征缓冲（Input Feature Buffer，IFB），按行粒度组织为环形缓冲。每行宽度为 *W_in*，所需行数为 *strip_rows*，由编译器在 cfg 派生阶段算出，使 *strip_rows* × *W_in* 个 word 不超过 IFB 容量。环形缓冲在物理层面是一段连续 SRAM，逻辑层面通过模 *strip_rows* 取地址实现。

行缓存模块的实现流程分为三步。第 1 步是行写入：IDMA 控制器（idma_ctrl）按行向 IFB 写入新行，每写完一行向行缓存模块汇报"写指针 +1"。第 2 步是窗口读取：行缓存模块按 (*y_out*, *x_out*) 输出位置组合，读出对应的 *K*×*K* 窗口，沿 Cin 维聚成 16 元向量后向 MAC 阵列发射。第 3 步是行释放：当某行不再被任何 *y_out* 窗口需要时，行缓存模块向 IDMA 控制器拉起 row_credit，释放该行的 ring 槽位，IDMA 控制器据此发起下一行的搬运。

行缓存模块的反压机制采用前向压力（forward-pressure）发射策略：仅当 `rows_available ≥ y_out × stride + K_y` 时才发射本行的窗口流，否则停顿等待 IDMA 写入。该策略确保任意 *H*×*W* 输入特征图都可在一次 start 内单遍跑完，片上仅占用 *strip_rows* × *W_in* 个 word 的 ring 容量，不需要把整图缓存到片上。一张 480×640 的 VGA 图像约 4.9 MB，IFB 仅占用 10 KB 量级即可流式跑完，环形缓冲读写指针在跨 round 处不复位，相邻 tile 与相邻 round 之间允许指针连续推进。

行缓存模块对外提供两路 valid-ready 接口：上游接 idma_ctrl 的写口，下游接 mac_array 的窗口口；上游反压通过 row_credit 实现，下游反压通过窗口 valid-ready 握手实现。模块内部不维护中心 FSM，仅由 (*y_out*, *x_out*, *k_x*, *k_y*) 四级本地计数器自维护循环边界，循环边界由 cfg_regs 提供。

### 4.3 MAC 阵列模块

MAC 阵列模块（mac_array）是加速核的算力核心，承担卷积层的乘加运算。如图 4.2 所示 MAC 阵列模块结构图。

**图 4.2 MAC 阵列模块结构图**
**Figure 4.2 MAC-array module structure**

MAC 阵列模块由 16 列 × 16 PE 共 256 个 INT8 乘加单元（NUM_COL=NUM_PE=16，DATA=8 bit）组成。16 列在结构上彼此独立，每列对应一个输出通道；列内 16 个 PE 沿 Cin 方向并行排布，承担单输出通道在 16 个输入通道方向上的并行乘加。这一二维结构与卷积循环嵌套中的 cout 维与 cin 维自然对齐：列方向广播激活值、列内方向沿 cin 累加。

MAC 阵列模块采用权重静止（Weight-Stationary，WS）+ 激活值滑窗复用 + 输出通道广播的复合数据流。第 1 步权重加载：在一次 cout_slice 启动时，权重缓存模块（wgt_buffer）将当前 cout_slice 所需的全部权重从权重缓冲（Weight Buffer，WB）一次性读入权重寄存器堆（Weight Register File，WRF=32），整段 cout_slice 内的全部空间位置共享这份权重，不再访问 WB。第 2 步激活值缓存：行缓存模块输出的当前窗口激活值向量进入激活值寄存器堆（Activation Register File，ARF=32），ARF 内的激活值供同一窗口在 *k_x*/*k_y* 子周期内重复读取。第 3 步广播与乘加：每周期由 ARF 广播 1 拍激活值向量到 16 列，每列消费各自的 WRF 项产生 1 路输出通道的 16 路 PE 并行乘积，列内 16 路乘积经一棵加法树规约为 1 路 PSUM=32 bit 部分和。第 4 步部分和发射：每周期 16 列同时向部分和累加模块发射 16 路 PSUM。

MAC 阵列模块的并行度可分解为三个维度。Cout 维并行度 16（列数）、Cin 维并行度 16（每列内 PE 数）、Kx/Ky 维顺序展开（同一窗口在 *K*² 个子周期内顺次计算）。理论峰值算力为 256 ops/cy × Fmax。当 Cin < 16 时，Cin 维存在列空转，需配合编译器侧 Ky 折叠优化（见 §4.8）；当 Cout < 16 时，硬件不复用，对应 PE 列空转，本工作不在此层做硬件级优化。

MAC 阵列模块对外提供三路接口。上游接行缓存模块的窗口口（激活值向量 valid-ready）；侧路接权重缓存模块的权重口（权重广播 valid，无反压）；下游接部分和累加模块的部分和口（16 路 PSUM valid-ready）。模块内部由 (*k_x*, *k_y*, *cin_pe*) 三级本地计数器自维护，循环边界由 cfg_regs 提供。每个 PE 由 1 个 INT8 乘加器构成，256 个 PE 的具体硬件实现细节与综合后的资源映射比例详见 §5 实测分析。

### 4.4 部分和累加模块

部分和累加模块（parf_accum）位于 MAC 阵列模块下游，对 16 列发射的部分和按 cin_slice 维度累加，等当前 (*y_out*, *x_out*) 输出位置在所有 cin_slice 上累加完毕后向 SDP 后处理模块发射。如图 4.3 所示部分和累加模块结构图。

**图 4.3 部分和累加模块结构图**
**Figure 4.3 Partial-sum accumulator module structure**

部分和累加模块的存储核心是部分和寄存器堆(Partial-Sum Register File，PARF=32)，逻辑容量为 16 列 × 32 项。在物理实现上，本设计将 PARF 拆分为 16 个独立 SRAM 实例（parf_col × NUM_COL=16），每列一个 SRAM，避免 16 列共用一个大 SRAM 时的端口冲突。16 列共享同一组写地址（wr_addr）与写使能（we），由外壳模块统一驱动；每列的写数据来自本列的 mac_array 输出 PSUM，每列的读数据独立返回，各列的累加在结构上完全隔离。

部分和累加模块的实现流程分为三步。第 1 步累加器读出：当 cin_slice 索引大于 0 时，从 PARF 当前 (*tile_w*, *cout_col*) 槽位读出已累加的部分和。第 2 步加法：将本周期 mac_array 发射的 16 路 PSUM 与读出的累加值相加，结果回写 PARF 同一槽位；当 cin_slice=0 时跳过读出，直接写入新值。第 3 步累加完成发射：当一个 (*tile_w*, *cout_col*) 槽位的所有 cin_slice 累加完毕（由 cfg_regs 中的 cin_slice_total 边界标识），将累加结果通过 valid-ready 握手发射到 SDP 后处理模块。

PARF=32 容量对应一个 cout_slice 内 32 个 *tile_w* 输出位置的部分和缓存。当 *tile_w* 输出数大于 32 时，编译器会进一步切分 tile，由 6 层 FSM 中的 tile 维循环承担。每列独立 SRAM 让 16 列累加在物理端口上完全隔离，匹配 MAC 阵列模块输出通道广播的数据流形态。

部分和累加模块对外提供两路接口。上游接 MAC 阵列模块的部分和口（16 路 PSUM valid-ready）；下游接 SDP 后处理模块的累加完成口（16 路 PSUM valid-ready）。模块内部由 (*tile_w*, *cin_slice*) 二级本地计数器自维护，循环边界由 cfg_regs 提供，cin_slice=0 与 cin_slice_total-1 两个边界条件分别控制累加器初值与发射时机。

### 4.5 SDP 后处理模块

SDP 后处理模块（ofb_writer）位于部分和累加模块下游，对累加完成的 32 位部分和依次完成偏置加、量化移位、饱和截断到 INT8、可选的残差融合，再写入输出特征缓冲（Output Feature Buffer，OFB=2048 word）。SDP 即标量数据通路（Scalar Data Path），是 INT8 量化推理通路上从 PSUM 到 OFM 的标量后处理链。如图 4.4 所示 SDP 后处理模块结构图。

**图 4.4 SDP 后处理模块结构图**
**Figure 4.4 SDP post-processing module structure**

SDP 后处理模块的实现流程分为五步。第 1 步偏置加：从偏置寄存器堆（bias_rf）读出当前输出通道的 32 位偏置，与累加完成的 32 位部分和相加。偏置由 RDMA 控制器从 DDR 中预先拉入 bias_rf。第 2 步量化移位：对偏置加结果按 cfg_regs 中配置的右移量进行算术右移，将 32 位部分和按层量化尺度缩放到目标位宽。第 3 步饱和截断：对移位结果按上下限（INT8 通常为 \[−128, 127\]）做饱和裁剪，超出范围的值钳位到边界。

第 4 步残差融合：当当前层启用残差时，从残差缓冲（Shortcut Bank）读出对应空间位置的残差值（INT8），按 shortcut_mult / shortcut_shift 进行通道级缩放后与饱和结果相加，结果再次饱和到 INT8。残差缓冲由上一层的 OFM 直接写入，无需经主机介入。第 5 步 OFB 写入：将最终 INT8 结果按 (*y_out*, *x_out*, *c_out*) 地址写入 OFB ring。

SDP 后处理模块在 ofb_writer 内集成偏置加、量化移位、饱和、残差融合，使 ResNet 残差通路在加速器内部一站式完成。Shortcut Bank 与 bias_rf 让残差加与偏置加都在片上完成，主机不需要为每层介入；ofb_writer 内部的 SDP 链以全 INT8 后处理形式落地（PSUM→bias→shift→clip→residual→clip→OFB），单层 cfg_regs 配置完成后整段后处理无需进一步指令。

SDP 后处理模块的 OFB 写入端同样按行粒度组织为环形缓冲。每写完一行向 ODMA 控制器（odma_ctrl）汇报"写指针 +1"，ODMA 控制器据此发起 S2MM 突发写入 DDR。当 ODMA 排空一行后向 ofb_writer 拉起 row_credit，让 ofb_writer 继续写入下一段。OFB ring 的 *strip_rows* 由编译器派生，与 IFB ring 相互独立。

SDP 后处理模块对外提供三路接口。上游接部分和累加模块的累加完成口；侧路接 bias_rf 与 Shortcut Bank 两个寄存器堆；下游接 OFB 写口与 ODMA 行信用口。模块内部由 (*y_out*, *x_out*, *cout_col*, *cout_slice*) 四级本地计数器自维护，循环边界由 cfg_regs 提供。SDP 量化组合链（偏置加 → 移位 → 饱和 → 残差 → 饱和）目前在单 stage 内组合实现，未做 stage 切分；时序裕度与 *F_max* 影响详见 §5 实测分析。

### 4.6 权重缓存模块

权重缓存模块（wgt_buffer）侧路供给 MAC 阵列模块的权重，配合权重静止数据流让权重在一段时间内保持稳定。如图 4.5 所示权重缓存模块结构图。

**图 4.5 权重缓存模块结构图**
**Figure 4.5 Weight buffer module structure**

权重缓存模块的存储核心是权重缓冲（Weight Buffer，WB=1024 word）与权重寄存器堆（Weight Register File，WRF=32）双层结构。WB 是较大容量的 SRAM，存放一层卷积所需的全部权重；WRF 是更小、更靠近 PE 的寄存器堆，缓存当前 cout_slice 所需的权重。

权重缓存模块的实现流程分为两步。第 1 步 WB 加载：在一层卷积启动时，WDMA 控制器（wdma_ctrl）按 (*c_out*, *c_in*, *k_x*, *k_y*) 顺序将该层全部权重从 DDR 经 MM2S 通道写入 WB。第 2 步 WRF 加载：在每个 cout_slice 启动时，权重缓存模块将本 cout_slice 所需的 16 列 × 32 项权重从 WB 一次性读入 WRF；之后整段 cout_slice 内所有空间位置的乘加都直接消费 WRF 内的权重，不再访问 WB。

WB 容量按一层最大权重需求设计，每层加载一次；WRF 容量按一个 cout_slice 设计，cout_slice 内反复使用，稳态下不再访问 DDR。

权重缓存模块对外提供两路接口。上游接 WDMA 控制器的写口；侧路接 MAC 阵列模块的权重广播口（无反压）。模块内部由 (*c_out*, *c_in*, *k_x*, *k_y*) 四级本地计数器自维护，循环边界由 cfg_regs 提供。

### 4.7 配置寄存器与 6 层嵌套 FSM

加速核内的层级控制由配置寄存器（cfg_regs）与 6 层嵌套 FSM（sequencer）协同完成。如图 4.6 所示配置寄存器与 6 层嵌套 FSM 结构图。

**图 4.6 配置寄存器与 6 层嵌套 FSM 结构图**
**Figure 4.6 Configuration registers and 6-level nested FSM structure**

cfg_regs 是加速核与外部交互的配置寄存器堆，承载一层卷积的全部静态参数。其内部包含约 50 余个寄存器，可分为四类：循环边界类（*H_in*、*W_in*、*C_in*、*C_out*、*K*、stride、pad）、片上缓冲基址类（IFB / WB / OFB / bias / shortcut 基址）、量化参数类（移位量、饱和上下限）、调度参数类（cin_slice 数、cout_slice 数、tile 宽度、strip 行数）。一层卷积启动前，整段 cfg 一次性写入完成；启动后整层硬件状态由 cfg 完整描述，运行期间不再变化。

cfg_regs 采用双写口设计。第 1 个写口接 AXI-Lite CSR（csr_w），由控制核写入启动期 4 个寄存器（描述符链表基址、链表长度、控制位、start_dfe 触发位）。第 2 个写口接 sequencer（seq_w），由 sequencer 消费描述符获取引擎（DFE）拉取的 CFG_WRITE 描述符流写入逐层配置。两路写口互不干扰，控制核与描述符通路在结构上隔离。这一设计将控制核的 AXI-Lite 写次数从 50 余次/层降到 4 次/层，多层链式调用时控制核仅需在系统启动时写一次 4 个寄存器并触发一次 start_dfe。

sequencer 实现 6 层嵌套 FSM。一层卷积处理过程由 (cs → yout → tile → cins → round → pos) 6 层循环依次推进：cs 是 cout_slice，遍历输出通道切片；yout 是输出行；tile 是 *W* 维 tile；cins 是 cin_slice 累加；round 是同一 tile 内的累加轮次；pos 是 PE 阵列位置。每一层循环由对应模块的本地计数器自维护，循环边界来自 cfg_regs，循环嵌套关系隐含在模块互连拓扑中——行缓存模块走 yout/tile，MAC 阵列模块与权重缓存模块走 round/pos，部分和累加模块走 cins，SDP 后处理模块走 cs。模块之间通过 valid-ready 握手互联，没有中心 FSM 协调。

sequencer 的另一项职责是描述符流处理。DFE 在系统启动后自动按描述符链表顺序从 DDR 拉取描述符，并按类型分发：IDMA_DESC 送入 idma_ctrl，WDMA_DESC 送入 wdma_ctrl，ODMA_DESC 送入 odma_ctrl，CFG_WRITE 描述符则由 sequencer 从 desc_fifo 中取出并写入 cfg_regs 的 seq_w 端口。一层卷积的描述符段长度可达 50 余个 CFG_WRITE，desc_fifo 容量从 32 扩到 128 以容纳一层的完整 cfg。

层完成后 sequencer 拉起 done_sticky 状态位，控制核可通过 AXI-Lite 读 STATUS 寄存器或接 GIC level IRQ 检测层完成。done_sticky 在 start_layer 上升沿自动清零，避免与下一层启动产生竞态。

### 4.8 编译器优化：Ky 折叠

针对 *C_in* < NUM_PE=16 的卷积层（如 ResNet 早期层 *C_in*=3、MobileNet 深度卷积 *C_in*=1），MAC 阵列模块在 Cin 维存在严重列空转，理论 PE 利用率仅 *C_in*/16。本工作在编译器侧提出 Ky 折叠（Ky-fold）优化，将 *K_y*（卷积 *Y* 方向）维度折叠到 cin 维，在不改动 RTL 的前提下提升 Cin 维利用率。

Ky 折叠的数学等价关系如下。原卷积按 (1) 式计算：

*Y*(*y*, *x*, *c_o*) = Σ\_{*k_y*=0..*K*-1} Σ\_{*k_x*=0..*K*-1} Σ\_{*c_i*=0..*C_in*-1} *X*(*y* + *k_y*, *x* + *k_x*, *c_i*) · *W*(*k_y*, *k_x*, *c_i*, *c_o*)    (1)

为按 NUM_PE=16 的硬件 cin 维容量分组折叠，定义每组容纳 *groups_y* = ⌊NUM_PE / *C_in*⌋ 个 *k_y* 行（以 *C_in*=3 为例，*groups_y* = ⌊16/3⌋ = 5），剩余 *k_y* 维分到 *kyper* = ⌈*K* / *groups_y*⌉ 组顺次执行。将组内 *k_y* 维与 *c_i* 维合并为 cin_fake 维，得到 (2) 式：

*Y*(*y*, *x*, *c_o*) = Σ\_{*g*=0..*kyper*-1} Σ\_{*k_x*=0..*K*-1} Σ\_{cin_fake=0..*groups_y*·*C_in*-1} *X*\'(*y*, *x* + *k_x*, *g*, cin_fake) · *W*\'(*k_x*, *g*, cin_fake, *c_o*)    (2)

其中 cin_fake = (*k_y* mod *groups_y*) × *C_in* + *c_i*，*g* = ⌊*k_y* / *groups_y*⌋，*X*\'(*y*, *x*, *g*, cin_fake) = *X*(*y* + *g* · *groups_y* + (*k_y* mod *groups_y*), *x*, *c_i*)（将 *y* 方向偏移复制到 cin 维），*W*\'(*k_x*, *g*, cin_fake, *c_o*) = *W*(*k_y*, *k_x*, *c_i*, *c_o*)。等效卷积的硬件视角变为 *K_y_eff* = *kyper*、*C_in_eff* = *groups_y* · *C_in* 的卷积。当 *K* · *C_in* ≤ NUM_PE（如 ResNet 首层 *K*=3、*C_in*=3）时 *kyper* = 1，整个 *K_y* 维一次折入，cin 维上 PE 利用率提升至 *K* · *C_in* / 16；当 *K* · *C_in* > NUM_PE（如 *K*=7、*C_in*=4）时 *kyper* > 1，剩余 *k_y* 维仍走硬件 *k_y* 顺序展开（详见 docs/pe-fold.md）。

Ky 折叠的实现完全在编译器侧。第 1 步条件判定：scheduler 在编译期判断 *C_in* < 16 ∧ *K* > 1 时启用 Ky 折叠。第 2 步权重重排：编译器把权重张量按 cin_fake 索引重排，在 DDR 中按硬件视角的等效卷积布局摆放。第 3 步特征图复制：在 Ky 折叠路径下，输入特征图按 *k_y* 偏移在 cin 维上复制，由 IDMA 控制器按重排后的布局拉取。第 4 步 cfg 派生：scheduler 用 cin_fake 维度算 cin_slice 数与循环边界，写入 cfg_regs。整个过程无需 RTL 改动，硬件继续按"普通卷积"运行。

Ky 折叠的代价是输入特征图在片外的存储量随 *K_y* 倍增——这是数据复制的直接代价。但由于权重静止数据流下激活值访问规模相对权重小，在 ResNet 早期层等场景下这一代价可被 PE 利用率提升所抵消。Ky 折叠让硬件保持最简固定阵列，编译器侧承担相位重排与权重布局，无需 RTL 改动即可完成 PE 利用率优化。

### 4.9 编译器优化：空间到深度（S2D）

针对 stride ≥ 2 的卷积层，传统硬件实现会让 stride² 个相位的 PE 在大部分时间空转。本工作在编译器侧提出空间到深度（Space-to-Depth，S2D）优化，将 (kx mod stride, ky mod stride) 的 stride² 个相位折到 cin 维，等价转换为 stride=1 的卷积，提升 PE 利用率。

S2D 的数学等价关系如下。设原卷积参数为 *K*、stride=*s*，将输入 *x* 与 *y* 坐标按 (3) 式分解：

*y* = *q_y* · *s* + *r_y*， *x* = *q_x* · *s* + *r_x*， 其中 *r_y*, *r_x* ∈ \[0, *s*-1\]    (3)

对每个相位 (*r_y*, *r_x*)，构造重排后的输入特征图 *X*\'\_{*r_y*, *r_x*}(*q_y*, *q_x*, *c_i*) = *X*(*q_y* · *s* + *r_y*, *q_x* · *s* + *r_x*, *c_i*)，将 stride² 个相位沿 cin 维拼接，得到 cin_new = *s*² × *C_in* 的等效输入；同时把卷积核按相位重排为 *K_new* = ⌈*K*/*s*⌉、stride_new = 1 的等效卷积核。原卷积按 (4) 式等价于：

*Y*(*q_y*, *q_x*, *c_o*) = Σ\_{*k_y_eff*=0..*K_new*-1} Σ\_{*k_x_eff*=0..*K_new*-1} Σ\_{cin_new=0..*s*²·*C_in*-1} *X*\'\_{*r_y*, *r_x*}(*q_y* + *k_y_eff*, *q_x* + *k_x_eff*, *c_i*) · *W*\'(*k_y_eff*, *k_x_eff*, cin_new, *c_o*)    (4)

S2D 的核心性质是输入数据在 DDR 中只是按相位重排索引，不需要复制——这与 Ky 折叠的"y 偏移复制"不同。S2D 在编译器侧完成数据重排，硬件视角看到的是 *K_new* × *K_new*、stride=1、cin 加宽至 *s*²·*C_in* 的普通卷积，PE 阵列在 Cin 维充分填充。

S2D 的实现完全在编译器侧。第 1 步条件判定：scheduler 在编译期判断 stride ≥ 2 ∧ *K* ≥ stride 时自动启用 S2D；与 Ky 折叠判定独立，两者可叠加触发。第 2 步权重与特征图重排：编译器将 stride² 个相位的权重按 cin_new 索引拼接，将输入特征图按相位拼接为 cin 加宽形式，DDR 中按重排后的布局摆放。第 3 步 cfg 派生：scheduler 用 *K_new*、stride=1、cin_new 等效维度算 cfg_regs，硬件按等效卷积运行。整个过程无需 RTL 改动，硬件继续按"普通卷积"运行。

S2D 配合大 stride 的首层 Patch 卷积（典型如 *K*=4、stride=4）效果显著。本工作在 ResNet11 网络的 Patch 层（*K*=4、stride=4）上实测：未启用 S2D 时该层 wall cycles 为 654,404，启用 S2D 后等效转换为 *K*=1、stride=1、*C_in*=64 标准卷积，PE 行被填满，wall cycles 降至 129,594，单层加速 5.05×（详见 §5.5.2） <!-- 来自 STATUS.md §2.8 -->。Ky 折叠与 S2D 在 scheduler.Layer 中由 force_fold / force_s2d 自动决策，亦可由用户通过命令行选项强制启用。

S2D 与 Ky 折叠的联合作用让编译器可以在不改动硬件的前提下，对 *C_in* 小、stride 大两类常见低利用率场景同时给出优化方案，硬件保持固定 16×16 阵列与最简 cfg 接口。

### 4.10 DMA 子系统与 AXI 接口集成

DMA 子系统位于加速核外，对接 AXI4 主接口，承担片外 DDR 与片上各缓冲区之间的批量数据搬运。如图 4.7 所示 DMA 子系统结构图。

**图 4.7 DMA 子系统结构图**
**Figure 4.7 DMA subsystem structure**

DMA 子系统的核心是 1 个 Xilinx axi_dm IP（AXI DataMover，由 Vivado 2023.1 IP 集成生成），提供 MM2S（Memory-Mapped to Stream）与 S2MM（Stream to Memory-Mapped）两条搬运通道。axi_dm IP 自身实现 AXI4 突发协议，对外暴露简化的命令—状态—数据流接口，简化自研控制器的实现。

DMA 子系统的自研控制器分为四类。第 1 类是输入特征图 DMA 控制器（idma_ctrl），将 IFM 从 DDR 经 MM2S 通道写入 IFB。第 2 类是权重 DMA 控制器（wdma_ctrl），将权重从 DDR 经 MM2S 通道写入 WB。第 3 类是输出特征图 DMA 控制器（odma_ctrl），将 OFM 从 OFB 经 S2MM 通道写出到 DDR。第 4 类是残差/偏置 DMA 控制器（rdma_ctrl），将偏置与残差数据从 DDR 经 MM2S 通道写入 bias_rf 与 Shortcut Bank。

由于 axi_dm IP 仅有 1 条 MM2S 通道，idma_ctrl 与 wdma_ctrl 通过 mm2s 仲裁器（mm2s_arb）串行共享。mm2s_arb 实现简单的轮转仲裁：每次完成一段 burst 后切换到对端，避免 idma 长时段独占导致 wdma 饥饿。S2MM 通道由 odma_ctrl 独占，无需仲裁。rdma_ctrl 在多核扩展层中也参与 MM2S 通道竞争，由 mm2s_arb 统一仲裁。

DMA 子系统对外仅暴露 1 个 AXI4 主接口，由 AXI 主口聚合器（axi_m_mux）实现。axi_m_mux 把 axi_dm 的 MM2S/S2MM 主口与描述符获取引擎（DFE）的主口三路聚合后挂到唯一的 AXI4 主接口。这一组织方式让加速器对外的 AXI 拓扑与 SoC 内常见的"加速器 + 1 主 + 1 Lite"结构对齐，简化集成。

AXI-Lite 从口由 AXI-Lite CSR 模块（axi_lite_csr）实现。axi_lite_csr 暴露 4 个启动期寄存器（描述符链表基址、链表长度、控制位、start_dfe 触发位）以及若干状态寄存器（done_sticky、错误码等）给控制核。控制核在系统启动时写一次 4 个寄存器并触发 start_dfe，整段多层卷积的层切换、权重切换、配置切换均由 DFE 与 sequencer 自动完成，无需控制核逐层介入。

描述符获取引擎（DFE）是 DMA 子系统与 sequencer 之间的桥梁。DFE 在 start_dfe 拉起后自动从 DDR 中拉取描述符链表，按描述符类型分发：TYPE_IDMA 送入 idma_ctrl 命令口、TYPE_WDMA 送入 wdma_ctrl 命令口、TYPE_ODMA 送入 odma_ctrl 命令口、TYPE_RDMA 送入 rdma_ctrl 命令口、TYPE_CFG=0x3 送入 sequencer 的 cfg 写口。描述符 FIFO（desc_fifo）容量为 128，可容纳一层完整 cfg 的全部 CFG_WRITE 描述符。一层卷积的描述符段顺序为：CFG_WRITE × N → IDMA_DESC → WDMA_DESC → RDMA_DESC → ODMA_DESC，由编译器在描述符链生成阶段确定。

DMA 子系统采用 Xilinx axi_dm IP 替代原版自研 DMA 控制器，目的在于降低维护代价并改善与 Vivado 工具链协同；自研控制器仅承担命令拼装、地址生成与反压逻辑，AXI4 突发协议交由 IP 完成。集成前后的端到端性能与搬运效率对比详见 §5 实测分析。

### 4.11 多核 W 切片扩展

为支持更高吞吐与多核扩展，加速器在加速核之上提供多核扩展层（multicore_top）。如图 4.8 所示多核扩展层结构图。

**图 4.8 多核扩展层结构图**
**Figure 4.8 Multi-core extension layer structure**

多核扩展层包含 *N* 个加速核（*N*=1/2/4 已综合通过）、AXI 互连交换（基于 Xilinx SmartConnect IP）、AXI-Lite 一对多分发模块（axi_lite_1to4）。各核之间通过 W 维切片调度并行，核间不共享数据通路与控制 FSM，仅在层边界通过握手寄存器同步。

W 切片调度的实现流程分为三步。第 1 步切片分配：编译器在 scheduler.analyze_slicing 中按核数 *N* 把 *W_in* 维分给 *N* 个核，每核处理一段 sub_*W* 子图。为保证卷积窗口在核边界处的语义一致性，每核额外保留 halo 列：当 *K*=3 且 stride=1 时，相邻核之间需要重叠 2 列输入；halo 由 computed redundancy 实现（重叠输入像素分别在两核 IFB 中各存一份），不引入跨核同步开销。第 2 步非对称 pad：边界核的外侧使用 cfg 中配置的真实 pad，内侧使用 0 pad（与重叠列衔接）；这一非对称 pad 由 cfg_regs 在多核场景下分别配置。第 3 步层边界握手：每核完成一层后拉起 done_sticky，多核扩展层在所有核 done_sticky 拉起后才允许下一层启动。

跨核 SRAM 直送（cross-core SRAM push）是多核扩展层的核心优化。生产者核的 odma_ctrl 在写出本层 OFM 时，目标地址不是 DDR 而是消费者核的 IFB 物理地址（按多核地址映射 `0x8000_0000 + i × 0x1000_0000`）。消费者核的 IFB 通过 ifb_axi_slave 模块挂载在 AXI 互连上，可从外部接收写入，将数据直接写入 IFB ring。这一方式避免了上一层 OFM 写 DDR、下一层 IFM 再从 DDR 读的双向带宽消耗，使多核加速比不被 DDR 带宽过早限制。

多核扩展层在代码组织上保持单核 RTL 不变，扩展通过外层完成：单核 RTL（line_buffer / mac_array / parf_accum / ofb_writer / wgt_buffer / cfg_regs / sequencer / 各 DMA 控制器）在 *N*=1/2/4 配置下完全相同，仅多核扩展层与编译器侧 scheduler.analyze_slicing 在不同 *N* 之间切换。增加新核数仅需扩展多核扩展层的外壳与 AXI 互连配置，不需要改单核 RTL。多核扩展层目前已在 *N*=1/2/4 三种配置下完成综合 \[CHECK: 综合环境与各配置下的资源占用以 §5 实测表为准\]。

### 4.12 DSP 阵列映射方案

MAC 阵列模块在 OC-broadcast 数据流下，每周期同一行激活值同时被 16 列消费、各列权重独立，相邻列在物理实现层面共享同一拍激活向量、仅权重不同。本节据此结构性特征给出一种把单核 mac_array 中相邻两列的 INT8 乘法器合并映射到一个 DSP48E1 块上的实现方案，使整个 16×16 阵列的 256 个乘法器从查找表实现迁移至 DSP 块实现。该方案在保持数据通路对外接口与位级行为完全等价的前提下，由综合阶段在两种映射之间作选择。如图 4.9 所示 DSP 阵列映射方案原理图。

**图 4.9 DSP 阵列映射方案原理图**
**Figure 4.9 DSP-array mapping scheme**

DSP48E1 块内部提供一个 25 bit × 18 bit 有符号乘法器，其乘积位宽为 43 bit。映射规则将相邻两列的两个 8 bit 权重 *w*₀、*w*₁ 打包到 25 bit A 端：低 8 bit 放 *w*₀、中间 9 bit 置零作为进位隔离位、高 8 bit 放 *w*₁，可写为 *A*={*w*₁, 9'b0, *w*₀}；将该行激活值 *act* 符号扩展为 18 bit B 端。一次乘法的 43 bit 乘积 *P* = *A* × *B* 同时承载两路独立 INT8 乘积：低 16 位经位提取后作符号校正得 *w*₀×*act* 的有符号结果，高位段（按打包偏移取出）经位提取后同样作符号校正得 *w*₁×*act* 的有符号结果。9 bit 进位隔离位保证了两路乘积在 43 bit 宽度内不会跨段串扰。符号校正逻辑承担将无符号视图与有符号视图之间的偏置项扣除，按 *w*₀、*w*₁ 的符号位与 *act* 拼出常数补偿项即可，整段以查找表组合电路实现，不再调用乘法器资源。

按上述映射规则展开至阵列层，16×16 阵列以 8 列对 × 16 行的方式重组：每行的相邻两列共用同一 DSP48E1 块，行间结构独立。单核 mac_array 的乘法器从原查找表实现的 256 个 INT8 乘法器折合为 8×16=128 个 DSP48E1 块，*N*=4 配置下整个加速器的 MAC 阵列共占用 4×128=512 个 DSP 块；上游接行缓存模块的窗口口、侧路接权重缓存模块的权重口、下游接部分和累加模块的部分和口三组接口的位宽、握手协议、循环边界与原查找表实现完全一致，加法树结构、PSUM 位宽以及流水节拍亦保持不变。综合阶段可在两种映射之间选择走查找表实现还是 DSP 块实现，加速核外其余模块不感知该选择。

将 MAC 阵列乘法器从查找表实现迁移至 DSP 块实现，可使加速器整体的查找表占用显著下降、DSP 资源利用率明显提高；与此同时，由于 DSP 块内乘法器为硬连线结构、其传播延迟短于查找表搭建的多级乘法树，关键路径的时序裕度也相应改善。该映射方案在端侧 ASIC 工艺迁移场景下不构成主线设计的依赖项，仅作为 FPGA 验证平台上的资源映射选项，体现固定阵列数据通路在不同实现工艺下的可移植性。

### 4.13 本章小结

本章按模块顺序给出了 FLUX_CNN 加速器的具体实现。首先依次介绍了加速核内 5 个数据通路模块：行缓存模块以 IFB=8192 word 行级环形缓冲组织 *K*×*K* 窗口生成，前向压力发射策略支持任意 *H*×*W* 输入；MAC 阵列模块由 16 列 × 16 PE 共 256 个 INT8 乘加单元组成，采用权重静止 + 滑窗复用 + 输出通道广播复合数据流；部分和累加模块以 16 列独立 SRAM 实现 PARF=32 累加，每列共享写地址写使能；SDP 后处理模块在 ofb_writer 内实现"偏置 → 移位 → 饱和 → 残差 → 饱和 → OFB"全 INT8 后处理链；权重缓存模块以 WB=1024 word + WRF=32 双层结构供给权重。其次介绍了 cfg_regs 双写口设计与 sequencer 6 层嵌套 FSM 的协同：50 余个 cfg 寄存器完整描述一层卷积，控制核仅写 4 个启动寄存器，描述符流由 DFE 自动消费。然后介绍了编译器侧的 Ky 折叠与空间到深度（S2D）两项 PE 利用率优化方案，分别针对 *C_in* < 16 与 stride ≥ 2 两类低利用率场景，硬件保持最简固定阵列。接着介绍了 DMA 子系统（axi_dm IP + 自研 4 类控制器 + mm2s_arb + axi_m_mux + axi_lite_csr + DFE）与多核 W 切片扩展层（*N*=1/2/4，halo + 跨核 SRAM 直送）。此外，本章给出了 16×16 INT8 MAC 阵列在 DSP48E1 块上的两种可选映射方案，为综合阶段在查找表与 DSP 块之间作权衡保留了入口。下一章将给出加速器的功能验证、综合实测与性能分析结果。

---

---
