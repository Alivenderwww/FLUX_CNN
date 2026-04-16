# 仿真运行指南

所有命令必须在 `sim/tb_core_isa/` 目录下执行（TCL 和 `.f` 文件使用相对路径）。

## 1. 生成测试数据

```bash
cd sim/tb_core_isa

# 默认配置（K=3, 68x120, stride=1, Cin=Cout=8）
python gen_isa_test.py

# 自定义
python gen_isa_test.py \
    --k 7           # 卷积核大小 K×K（v1 只支持 K²·cin_slices ≤ 32, 即 packed）
    --h_in 224      # 输入高度
    --w_in 224      # 输入宽度
    --stride 1      # 卷积步长 1~7
    --num_cin 16    # 输入通道数
    --num_cout 32   # 输出通道数
    --tile_w 32     # ARF tile 宽度（不要改，硬件强依赖 32）
    --shift 2       # SDP 反量化右移量 0~31
    --seed 42       # 随机数种子
```

生成文件：
- `config.txt` —— 配置寄存器（key=value 格式，TB 解析后层次引用写入）
- `ifb.txt` —— 输入特征图十六进制值（每行一个 128-bit 字）
- `wb.txt` —— 权重十六进制值（每行一个 2048-bit 字）
- `expected_ofm.txt` —— 软件金标准（每行一个 128-bit 输出字）
- `sim_params.f` —— 传给 TB 的 plusargs（`+H_OUT=...`, `-gSRAM_DEPTH=...`）

### SRAM 容量限制

单切片 IFB 大小 = `H_IN × W_IN`，单切片 OFB 大小 = `H_OUT × W_OUT`。多切片时乘以切片数。Python 的 `PYTHON_SRAM_LIMIT = 65536`，超出则报错。典型可容纳场景：

| 场景 | 尺寸 | 是否通过 |
|------|------|----------|
| K=3 C3C8 224×224 s=1 | IFB=50176, OFB=49284 | ✓ |
| K=3 C16C16 224×224 s=1 | IFB=50176 | ✓ |
| K=3 C32C16 224×224 s=1 | IFB=100352 | ✗（超限） |

---

## 2. 运行单个仿真

```bash
# 批处理模式（快）
vsim -c -do tb_core_isa.tcl

# GUI 模式（波形调试）
vsim -do tb_core_isa.tcl
# 或 Windows 下：
do.bat
```

### 输出解读（v3 架构）

```
Config loaded from config.txt
========================================
Starting config-driven CNN Core...
========================================
(... WRF/SRAM 调试信息 ...)
========================================
Core execution finished. Verifying OFB...
OFM dims: H_OUT=66 W_OUT=118 COUT_SLICES=1 TOTAL=7788
Check Complete! 0 mismatches found. Simulation PASSED.
========================================
 PERFORMANCE & RESOURCE UTILIZATION STATS
========================================
Total Active Cycles:  70128

--- SRAM Traffic ---
IFB: Reads=70092  Writes=0
WB : Reads=9      Writes=0
OFB: Reads=0      Writes=7788

--- PE Array Utilization ---
True MAC Fire Cycles: 70092
Total MAC Ops:        17943552 (= real_cycles * 256 PEs)
Theoretical Max MACs: 17952768 (= cycles * 256 PEs)
MAC Utilization:      99.95 %

--- RF Traffic ---
WRF  Writes: 2304

--- Handshake Stats (fire / stall=V&!R / idle=!V&R) ---
ACT  (lb  -> mac): fire=70092 stall=9    idle=0
WGT  (wb  -> mac): fire=70092 stall=0    idle=9
PSUM (mac -> prf): fire=70092 stall=0    idle=42
ACC  (prf -> ofb): fire=7788  stall=0    idle=62339
```

**True MAC Fire Cycles** 是精确 MAC 计数（不含 bubble 拍）。MAC 利用率 = True MAC / (cycles × 256)。

**握手统计**四项解读：
- `fire` = V&R 都 1，实际握手成功
- `stall` = V=1 R=0，上游被下游堵
- `idle` = V=0 R=1，下游空等上游
- 诊断气泡主要看 stall/idle 谁大，定位慢方

---

## 3. TB 如何注入配置

`tb_core_isa.sv::load_config()` 做两件事：

1. `$fopen("config.txt", "r")`，用 `$sscanf("%s = %d", key, val)` 逐行解析
2. 按 `key` 通过**层次引用**直接写入 `cfg_regs` 内部寄存器：

```systemverilog
case (key)
    "H_OUT"        : u_core_top.u_cfg.r_h_out        = val[15:0];
    "IFB_CIN_STEP" : u_core_top.u_cfg.r_ifb_cin_step = val[19:0];
    ...
endcase
```

这意味着：

- **配置寄存器没有外部写端口**；目前仅用于功能验证
- 想加新的 cfg 字段：
  1. 在 `cfg_regs.sv` 里声明 `logic [N-1:0] r_xxx = 0;` + 对外只读端口
  2. 在要使用的模块（如 `line_buffer` / `wgt_buffer`）加 input port 和使用逻辑
  3. 在 `core_top.sv` 连线
  4. 在 `gen_isa_test.py` 的 `cfg` dict 里加 `'XXX': value`
  5. 在 `tb_core_isa.sv::load_config()` 的 case 里加一行
- 后期上真实总线（AXI-Lite 或自研）时，把 cfg_regs 的内部 reg 改成带外部写端口即可，TB 逻辑不变

---

## 4. 回归测试

```bash
python run_regression.py --label v3-handshake --out regression_v3.txt
```

脚本会依次：
1. 对 `CASES` 列表里每个用例调用 `gen_isa_test.py`
2. 跑 `vsim -c -do tb_core_isa.tcl`
3. 用正则从仿真输出里抓取 `Total Active Cycles`、`MAC Utilization`
4. 生成格式化报告写入文件

当前活跃 CASES 覆盖 10 个 packed 场景，全 PASS。K=7 和超大图用例被注释（K=7 chunked 不支持；222×222 跑太慢）。

### 修改回归用例

在 `run_regression.py::CASES` 里加一行 `(name, c_in, c_out, k, h_in, w_in, stride, shift)`。

### 基线文件

- `regression_v3.txt` —— **当前 v3 握手架构基线**
- `regression_v2_parf.txt`, `regression_v1_handshake.txt` —— 握手架构演进过程中的中间快照
- `regression_baseline.txt` / `16x16.txt` / `cslice.txt` / `pipeline.txt` / `subchan.txt` / `cfg_driven.txt` —— 已退役架构的历史快照，保留用于对比

### 关键性能参考（K=3 packed）

| 配置 | Cycles | MAC% |
|------|---:|---:|
| K=3 C8C8 66×118 s=1 | 70,128 | 99.95% |
| K=3 C32C32 66×118 s=1 | 280,433 | 99.98% |
| K=5 C16C16 30×56 s=2 | 43,556 | 99.87% |

v3 相对原 cfg-driven FSM 在 K=3 C8C8 用例快 **12.6%**。详见 `docs/handshake-pipeline.md` 的气泡分析。

---

## 5. 文件清单和编译顺序

`sim_file_list.f`：

```
+incdir+../../RTL
../../RTL/std_rf.sv
../../RTL/sdp.sv
../../RTL/sram_model.sv
../../RTL/cfg_regs.sv
../../RTL/mac_pe.sv
../../RTL/mac_col.sv
../../RTL/mac_array.sv
../../RTL/parf_accum.sv
../../RTL/line_buffer.sv
../../RTL/wgt_buffer.sv
../../RTL/ofb_writer.sv
../../RTL/core_top.sv
tb_core_isa.sv
```

添加新 RTL 模块时要**同时**更新这个文件和 `Syn/run_syn.tcl`。
