# RTL 代码编写原则

本文档是 FLUX_CNN 项目 SystemVerilog 代码的 **强约束** 风格规范。新增或重构代码必须遵守；遇到冲突按规则改。Subagent 与人工改写均适用。

## 1. 模块规模

- 一个 `.sv` 文件一个 `module`。
- 模块代码量 **≤ 500 行**；逻辑实在难以拆分可放宽至 **≤ 750 行**。超过视为设计问题，应拆子模块或把独立子逻辑抽成 comb 辅助信号。
- **子模块例化语句（`u_xxx ( ... )` 整块）不计入行数限制**。顶层连线多的模块（如 `core_top`）允许例化段撑大行数。

## 2. 文件头与端口

- 首行 `` `timescale 1ns/1ps ``。
- 顶部注释说明：模块职责、对外握手 / 数据通路语义、关键时序约束。不复述代码本身。
- 端口顺序：`clk, rst_n → 控制 (start 等) → cfg_* → 数据输入 → 数据输出`。同一握手组 `(data, valid, ready)` 紧挨着。
- 可调参数用 `parameter`，内部推导常量用 `localparam`。

## 3. 命名

### 3.1 禁用裸词

以下词不得单独作变量名：`true` / `real` / `fake` / `temp` / `tmp` / `data1` / `data2` / `flag` / `done2` 等。加明确前/后缀后可用（例如 `is_real_op`），前提是读者瞬间知道含义。

### 3.2 数字标号

禁止用 `done1` / `done2` / `sig_a` / `sig_b` 区分功能不同的信号；名字应反映语义差别（如 `load_done` / `compute_done`）。

### 3.3 缩略词

- **模块内部变量** 原则上写全拼；仅以下 RTL 约定俗成词允许缩略：
  - `cnt`（counter）/ `ptr`（pointer）/ `idx`（index）/ `addr`（address）
  - `we` `re` `waddr` `raddr` `wdata` `rdata`
  - `_d1` `_d2`（延迟 1 / 2 拍）
  - `_nxt` / `_next`（comb 下一态）
- **模块名、端口名** 可更自由使用缩略。

### 3.4 词缀语义一致

带相同词缀的变量语义必须一致。项目已约定的后缀：

| 后缀 | 语义 |
|------|------|
| `*_cnt` | counter |
| `*_base` | running base（加法推进的基地址） |
| `*_is_last` | 「最后一步」布尔 |
| `*_valid` `*_ready` | 握手信号 |
| `*_fire` | `valid & ready` 的单拍成功标志 |
| `*_d1` `*_d2` | 延迟 1 / 2 拍的副本 |

新增变量沿用既有后缀。新造后缀须在整模块内一致推广。

## 4. always 块组织

### 4.1 单信号单维护

**一个 `always_ff` 原则上只维护一个寄存器。** 多个寄存器触发条件与推进规则完全相同可合并，但 **同一 `always_ff` 维护的寄存器数量不超过 4 个**。

**例外 1——共享 if-else 结构的并列寄存器**：若多个寄存器的 if-else / case 分支结构 **完全一致**（触发条件、优先级相同，仅 RHS 不同），可并入同一 `always_ff` 作一体处理，**不计入 ≤ 4 的限制**。典型场景：同一触发下需要并行更新的一组 running base。

```systemverilog
// ↓ 分支结构完全一致，仅赋值不同，合写合理；不计 4-reg 限制
always_ff @(posedge clk) begin
    if (evt_cs_start) begin
        ptr_tile_base <= cfg_ifb_base;
        ptr_cins_base <= cfg_ifb_base;
        ptr_ky_base   <= cfg_ifb_base;
        ptr_kx_base   <= cfg_ifb_base;
    end else begin
        ptr_tile_base <= ptr_tile_base;
        ptr_cins_base <= ptr_cins_base;
        ptr_ky_base   <= ptr_ky_base;
        ptr_kx_base   <= ptr_kx_base;
    end
end
```

**例外 2——寄存器 bank（地址译码同体系写入）**：若多个寄存器虽然没有强相关，但同属一个 **寄存器 bank 体系**（共享外层使能 `reg_w_en`、通过 `reg_w_addr` case 分发到各自目标），可写在同一 `always_ff` 里，**不计入 ≤ 4 的限制**。典型场景：CSR / MMIO 寄存器集合、AXI-Lite 下挂的一整组状态寄存器。每条 case 都显式命名要写的寄存器，仍易于 grep 定位。

```systemverilog
// ↓ 典型 CSR bank：共享 reg_w_en 门控 + addr 解码
always_ff @(posedge clk) begin
    if (reg_w_en) begin
        case (reg_w_addr)
            ADDR_H_OUT : r_h_out <= reg_w_data[15:0];
            ADDR_W_IN  : r_w_in  <= reg_w_data[15:0];
            ADDR_K     : r_k     <= reg_w_data[3:0];
            // ... 更多寄存器
            default    : ;  // 未命中地址不写入任何寄存器
        endcase
    end
end
```

此例下，§4.5 的「所有路径显式赋值」也豁免：未命中 case 时所有 bank 寄存器隐式保持（flop 固有行为），不要求为每个寄存器逐一写 `reg <= reg`。

### 4.2 命名事件信号模式

FSM 推进涉及多寄存器联动时，先在 `always_comb` 里抽出命名事件，再让每个寄存器的 `always_ff` 独立消费：

```systemverilog
// 事件（纯组合）
logic evt_pos_advance, evt_round_wrap, evt_tile_wrap;
always_comb begin
    evt_pos_advance = (state == S_RUN) && wgt_fire && x_is_last;
    evt_round_wrap  = evt_pos_advance && pos_is_last;
    evt_tile_wrap   = evt_round_wrap  && round_is_last && cins_is_last;
end

// 各寄存器独立 always_ff
always_ff @(posedge clk or negedge rst_n) begin
    if      (!rst_n)          pos_cnt <= '0;
    else if (evt_round_wrap)  pos_cnt <= '0;
    else if (evt_pos_advance) pos_cnt <= pos_cnt + 6'd1;
end
```

### 4.3 if 嵌套深度

**最多 3 层 if**。超过 3 层：
- 优先抽命名事件信号把条件打平；或把子条件抽成 comb 辅助信号。
- 若逻辑固有深、无法解耦：**停下汇报**，说明具体结构，人工决定是否放行。

### 4.4 赋值与敏感列表

- `always_ff` 只用非阻塞 `<=`；`always_comb` 只用阻塞 `=`。同一块禁止混用。
- 时序：默认 `always_ff @(posedge clk)`；是否写复位分支、以及复位风格，见 §6。
- 组合：`always_comb`，不要写 `always @(*)`。所有分支必须给信号赋值，防止推 latch（常用技巧：块开头先给默认值）。

### 4.5 赋值完备性

每个寄存器在其 `always_ff` 内必须对 **任意触发路径** 都有明确赋值。

- `if / else if` 链必须以 `else` 收尾；保持值时显式写 `reg <= reg`。
- `case` 必须有 `default` 分支；保持值时显式写 `reg <= reg`。
- 例外：设计本身明确不关心该触发路径下的寄存器值（通常是上游 valid 遮蔽），在注释里写明「此路径值不被下游观察，故保留任意」。

```systemverilog
// ✗ 不推荐：依赖 always_ff 的隐式 flop 保持语义
always_ff @(posedge clk) begin
    if      (evt_a) reg <= val_a;
    else if (evt_b) reg <= val_b;
end

// ✓ 推荐：任意路径下赋值明确
always_ff @(posedge clk) begin
    if      (evt_a) reg <= val_a;
    else if (evt_b) reg <= val_b;
    else            reg <= reg;
end
```

这避免工具在工艺 / 综合选项差异下对保持行为做不同推断，并让仿真 / 综合 / STA 三者语义一致。

## 5. FSM 规范

### 5.1 三段式

```systemverilog
typedef enum logic [1:0] { S_IDLE, S_LOAD, S_RUN, S_DONE } state_t;
state_t state, state_next;

// Seg 1: 下一态组合
always_comb begin
    state_next = state;
    case (state)
        S_IDLE : if (start)     state_next = S_LOAD;
        S_LOAD : if (load_done) state_next = S_RUN;
        // ...
        default:                state_next = state;
    endcase
end

// Seg 2: 状态寄存器（只维护 state）
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= state_next;
end

// Seg 3: Moore 输出（组合）
always_comb begin
    wgt_valid = (state == S_RUN);
    // ...
end
```

### 5.2 编码

- `typedef enum logic [N-1:0]`，显式指定宽度。
- 状态名 `S_` 前缀。
- `case` 必须有 `default` 分支。

### 5.3 FSM 与数据通路解耦

FSM 段只管 `state`。计数器、running base、指针等数据寄存器写各自的 `always_ff`，把 `state` 当作输入条件消费。见 §4.2。

## 6. 复位策略

复位是控制信号，不是寄存器的「默认行为」。盲目复位会增加复位网络面积、时序压力，在工艺迁移时带来不确定性。ASIC 前端设计约定：**默认不写复位，确定需要再加**。

### 6.1 何时需要复位

| 类别 | 是否需要复位 |
|------|------|
| FSM 的 `state` 寄存器 | **必须** |
| 控制回路中影响启停 / 错误的标志（如 `done`、`busy`） | **必须** |
| 影响 pipeline 起始正确性的控制 counter / handshake valid | **通常需要** |
| 纯数据寄存器（乘积、累加、ring buffer 存储、地址 base） | **通常不需要**。上电后最初几拍的无效数据会被上游 valid 遮蔽，不进入下游。 |

### 6.2 评估流程

每个 `always_ff` 写前先问一句：**该寄存器的上电值是否影响下游可观测行为？**

- 是 → 写复位分支。
- 否 → 不写复位分支，也不写 `logic foo = '0;` 形式的初值（仿真依赖初值即算设计 bug）。

### 6.3 同步复位优先

需要复位时，**优先同步复位**：

```systemverilog
always_ff @(posedge clk) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= state_next;
end
```

敏感列表不含 `rst_n`。相比异步复位的优势：
- D-FF 少一个异步控制端口，面积 / 时序更优。
- 复位路径走正常组合网络，STA 模型简单。
- 工艺迁移时行为确定性更强。

复位信号 `rst_n` 低电平有效。

### 6.4 数组批量清 0 的特例

WRF / PARF / act_buf 等寄存器阵列一般 **不** 需要复位清 0（上游 valid 遮蔽未初始化数据即可）。若仿真中要避免 X-propagation 污染，用 `initial` 块做仿真初值（仅仿真可见），不要在 `always_ff` 里写 reset loop：

```systemverilog
// synthesis translate_off
initial begin
    for (int i = 0; i < DEPTH; i++) mem[i] = '0;
end
// synthesis translate_on
```

## 7. 综合相关

- 可综合 RTL 禁止 `#delay`（TB 例外）。
- 禁止 `full_case` / `parallel_case` pragma；用 `default` 分支代替。
- `for` 只用于 `generate` 或固定边界的寄存器数组处理。
- 禁止多驱动；组合赋值必须覆盖所有分支避免 latch。
- 魔数用 `localparam` 命名。

## 8. 注释

- 写「为什么」（意图、约束、约束的历史原因、握手契约），不写「做什么」（代码已表达）。
- 文件头：模块职责 + 握手语义 + 关键时序。
- 关键 `always` 块前一行注释说明作用。
- 禁止留 `TODO` / `FIX LATER` / `see issue #` 等易腐烂内容；需要跟踪用 git issue。

## 9. 握手契约不可破坏

- 模块间 valid-ready 语义重构时须原样保留。`*_valid` 与 `*_ready` 不得互相依赖成环（elastic join 是已验证的特例）。
- 握手成功 = `valid & ready` 同拍同时为高；该拍 `data` 必须稳定。
- 重构只改内部实现，不改端口行为；若确需改端口，停下汇报。

## 10. 完成标准

每次重构完成前自检：

- [ ] 每个模块 ≤ 500（≤ 750）行（例化语句不计）
- [ ] 无禁用裸词、无数字标号
- [ ] if 嵌套 ≤ 3 层
- [ ] 每个 `always_ff` ≤ 4 个寄存器（共享 if-else 结构的并列寄存器豁免）
- [ ] 每个寄存器在所有触发路径下有明确赋值（`else` / `default` 显式 `reg <= reg` 或注释豁免）
- [ ] FSM 三段式；`state` 独立 `always_ff`
- [ ] 复位按 §6 策略评估；需要时用同步复位；纯数据寄存器无复位
- [ ] 端口与握手契约未变
- [ ] `toolchain/` 下 `python run_regression.py` 22/22 PASS，MAC% 不降；加 `--fold --kx-fold` 跑一遍 fold 路径
