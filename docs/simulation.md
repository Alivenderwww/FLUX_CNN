# 仿真运行指南

所有命令必须在 `sim/tb_core_isa/` 目录下执行（TCL 和 `.f` 文件使用相对路径）。

## 1. 生成测试数据

```bash
cd sim/tb_core_isa

# 默认配置（K=3, 68x120, stride=1, Cin=Cout=8）
python gen_isa_test.py

# 自定义
python gen_isa_test.py \
    --k 7           # 卷积核大小 K×K，支持 {1,3,5,7,9,...}（K²>32 自动走 round chunking）
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
- `config.txt` —— 配置寄存器（key=value 格式，TB 会解析并 poke 进 FSM）
- `ifb.txt` —— 输入特征图十六进制值（每行一个 128-bit 字）
- `wb.txt` —— 权重十六进制值（每行一个 2048-bit 字）
- `expected_ofm.txt` —— 软件金标准（每行一个 128-bit 输出字）
- `sim_params.f` —— 传给 TB 的 plusargs（`+H_OUT=...`, `-gSRAM_DEPTH=...`）

### SRAM 容量限制

单切片 IFB 大小 = `H_IN × W_IN`，单切片 OFB 大小 = `H_OUT × W_OUT`。多切片时这些要乘以切片数。Python 的 `PYTHON_SRAM_LIMIT = 65536`，超出则报错。目前可容纳：

| 场景 | 尺寸 | 是否通过 |
|------|------|----------|
| K=3 C3C8 224×224 s=1 | IFB=50176, OFB=49284 | ✓ |
| K=7 C3C8 224×224 s=1 | IFB=50176, OFB=47524 | ✓ |
| K=3 C16C16 224×224 s=1 | IFB=50176×1=50176 | ✓ |
| K=3 C32C16 224×224 s=1 | IFB=50176×2=100352 | ✗（超限） |

要进一步放大，需要调整 `core_ctrl` 的 `ADDR_W` 以及 `PYTHON_SRAM_LIMIT`。

---

## 2. 运行单个仿真

```bash
# 批处理模式（快）
vsim -c -do tb_core_isa.tcl

# GUI 模式（波形调试）
vsim -do tb_core_isa.tcl
# 或者 Windows 下：
do.bat
```

### 输出解读

```
Config loaded from config.txt
========================================
Starting config-driven CNN Core...
========================================
(... sram/arf/parf 调试信息 ...)
========================================
Core execution finished. Verifying OFB...
OFM dims: H_OUT=66 W_OUT=118 COUT_SLICES=1 TOTAL=7788
Check Complete! 0 mismatches found. Simulation PASSED.
========================================
 PERFORMANCE & RESOURCE UTILIZATION STATS
========================================
Total Active Cycles:  80267

--- SRAM Traffic ---
IFB: Reads=33264  Writes=0
WB : Reads=9  Writes=0
OFB: Reads=0  Writes=7788

--- PE Array Utilization ---
Total MAC Ops:        17943552
Theoretical Max MACs: 20548352 (= cycles * 256 PEs)
MAC Utilization:      87.32 %

--- RF Traffic ---
ARF  Writes: 33264
ARF  Reads : 70092 (combinational, = MAC active cycles)
WRF  Writes: 2304
PARF Writes: 1121472
```

---

## 3. TB 如何注入配置

`tb_core_isa.sv::load_config()` 做两件事：

1. `$fopen("config.txt", "r")` 打开配置文件，用 `$sscanf("%s = %d", key, val)` 逐行解析
2. 按 `key` 通过**层次引用**直接写入 FSM 内部寄存器：

```systemverilog
case (key)
    "H_OUT"        : u_core_top.u_ctrl.cfg_h_out        = val[15:0];
    "IFB_CIN_STEP" : u_core_top.u_ctrl.cfg_ifb_cin_step = val[19:0];
    ...
endcase
```

这意味着：

- **配置寄存器没有外部写端口**；目前仅用于功能验证。
- 想加新的 cfg 字段只需：1) 在 `core_ctrl` 里声明 `logic [N-1:0] cfg_xxx;`，2) 在 `gen_isa_test.py` 的 `cfg` dict 里加 `'XXX': value`，3) 在 `tb_core_isa.sv::load_config()` 的 case 里加一行。
- 后期上真实总线（AXI-Lite 或自研）时，把写端口替换成寄存器堆即可，TB 逻辑不变。

---

## 4. 回归测试

```bash
python run_regression.py --label cfg-driven --out regression_cfg_driven.txt
```

脚本会依次：
1. 对 `CASES` 列表里每个用例调用 `gen_isa_test.py`
2. 跑 `vsim -c -do tb_core_isa.tcl`
3. 用正则从仿真输出里抓取 `Total Active Cycles`、`MAC Utilization`、`ARF Writes/Reads`
4. 生成格式化报告写入文件

当前 CASES 覆盖 17 个场景，全 PASS。

### 修改回归用例

在 `run_regression.py::CASES` 里加一行 `(name, c_in, c_out, k, h_in, w_in, stride, shift)`。

### 历史 baseline

- `regression_baseline.txt`, `regression_16x16.txt`, `regression_cslice.txt`, `regression_pipeline.txt`, `regression_subchan.txt` —— 原 ISA 方案的历史快照，保留用于对比
- `regression_cfg_driven.txt` —— 当前配置寄存器方案的基线

### 关键性能参考

| 配置 | cfg-driven Cycles | cfg-driven MAC% | 对比 ISA Cycles | 对比 ISA MAC% |
|------|----|-----|-----|-----|
| K=7 C8C8 62×114 s=1 | 377 706 | 91.69% | 378 211 | 91.57% |
| K=7 C8C8 12×12 s=2  | 8 378   | 84.22% | 8 483   | 83.18% |

新 FSM 省下了指令 fetch/decode 和标量指令（LI/ALU/BNZ）的开销。

---

## 5. 文件清单和编译顺序

`sim_file_list.f`：

```
+incdir+../../RTL
../../RTL/std_rf.sv
../../RTL/sdp.sv
../../RTL/core_ctrl.sv
../../RTL/core_top.sv
../../RTL/mac_array.sv
../../RTL/mac_col.sv
../../RTL/mac_pe.sv
../../RTL/sram_model.sv
tb_core_isa.sv
```

添加新 RTL 模块时要**同时**更新这个文件和 `Syn/run_syn.tcl`。
