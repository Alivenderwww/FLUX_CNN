# 仿真运行指南

主仿真目录是 `sim/tb_core_dma/`（端到端 descriptor-driven 测试）。工具链脚本在 `toolchain/`。

## 1. 端到端回归（推荐入口）

```bash
cd toolchain
python run_regression.py                      # 无 fold 基线, 22 case
python run_regression.py --fold               # Ky-fold 启用
python run_regression.py --s2d                # Space-to-Depth (stride>=2 case)
python run_regression.py --fold --s2d         # 两个叠加

# 只跑 name 含指定子串的 case
python run_regression.py --case "C8C8"
python run_regression.py --case "C64C64"

# 手动 watchdog 超时
python run_regression.py --timeout-ns 2000000000
```

脚本流程：
1. 对 `CASES` 列表里每个用例调用 `gen_isa_test.py` 生成 `cases/caseNN/*.txt`
2. 把 case 0 的 `sim_params.f` 拷到 sim dir 并追加 `+N_CASES` / `+TIMEOUT_NS` plusarg
3. 跑 `vsim -c -do run.tcl`（单 vsim 跑完整个 case 列表，核之间 start/done reuse）
4. 解析每个 `CASE_RESULT` 行，汇总成 `regression_report.txt`，含 PROFILE 表（每接口 fire/stall/idle %）

---

## 2. 单 case 生成 + 手动跑

```bash
cd toolchain
python gen_isa_test.py \
    --k 3 --h_in 68 --w_in 120 \
    --num_cin 8 --num_cout 8 \
    --stride 1 --pad 1 --shift 0

# 带 Ky-fold
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 \
    --num_cin 4 --num_cout 8 --stride 2 --pad 4 \
    --ky-fold

# 带 S2D (stride>=2)
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 \
    --num_cin 4 --num_cout 8 --stride 2 --pad 4 \
    --s2d

# 跑仿真 (--out-dir 默认 ../sim/tb_core_dma)
cd ../sim/tb_core_dma
vsim -c -do run.tcl
```

生成文件（在 `--out-dir` 里）：
- `config.txt` — cfg 寄存器 key=value 格式
- `ifb.txt` / `wb.txt` — IFB / WB SRAM 初始值十六进制
- `expected_ofm.txt` — 软件金标准（原始 conv 计算，fold 不影响）
- `desc_list.hex` — DMA descriptor 列表
- `sim_params.f` — vsim plusarg / generic

---

## 3. TB 配置注入

`tb_core_dma.sv` 通过 **AXI-Lite** 写入 cfg 寄存器（不是层次引用）：

```systemverilog
task load_config(string case_dir);
    // 读 config.txt, 按 key 调 axi_lite_write(ADDR_XXX, val)
    case (key)
        "K"              : axi_lite_write(ADDR_K,              val);
        "KY"             : axi_lite_write(ADDR_KY,             val);
        "STRIDE"         : axi_lite_write(ADDR_STRIDE,         val);
        ...
    endcase
endtask
```

加新 cfg 字段要同步更新：
1. `cfg_regs.sv` 声明 `r_xxx` + `ADDR_XXX` + 输出端口
2. 消费模块加 input port
3. `core_top.sv` 连线
4. `hw_files.py::cfg_to_dict()` 把新字段写入 `config.txt`
5. `tb_core_dma.sv::load_config` 加 case 分支

---

## 4. PyTorch 端到端部署

```bash
cd toolchain
python models/train_mnist.py                      # 训练 (已有 ckpt 可跳过)
python models/dump_mnist_png.py -c 20             # MNIST → PNG
python models/run_model.py --model mnist_allconv \
    --image-dir models/images/mnist_test --limit 10
```

脚本：
1. 加载 PyTorch ckpt → 量化 → 逐层 `compile_model` 生成 per-layer DDR layout
2. 拼成多层 descriptor list 写 DDR
3. 一次 `vsim` 跑完整个 `nn.Sequential`，中间层 OFB 直接当下层 IFB
4. 最后 ODMA 输出 → 软件 argmax 和 FP 参考对比

报告：`models/model_report.txt`（top-3 置信度 + 硬件 cycle + argmax 对照）

---

## 5. 仿真输出 CASE_RESULT 格式

```
CASE_RESULT 0 PASS  cycles=1124175  mac_fire=1119768  mac_util=92.76%
                    arf_w=1119768  arf_r=1119768  parf_f=1119768  parf_d=130351
                    ifb_r=1106300  wb_r=...        ofb_w=130351
                    name=K=8_C4C8____960x540_s2_p4|conv

CASE_PROFILE 0 cycles=1124175 act_fire=1085057 act_stall=...   acc_idle=994067 ...
```

字段定义：
- `mac_fire`：mac_array 实际 fire 的 cycle 数
- `mac_util = useful_mac_ops / peak_mac_ops`
  - `useful = H_out × W_out × K² × Cin × Cout`（原始 conv 维度，fold 前）
  - `peak   = cycles × NUM_COL × NUM_PE`
  - fold pad 和 Kx tail partial 不算 useful → 真实反映 MAC 有多少做了有用工作
- `arf_w` / `arf_r`：ARF 写/读计数；`arf_r / arf_w` 比值 >1 表示 kx 滑窗复用
- `parf_f` / `parf_d`：psum 累加 fill / drain 次数
- `ifb_r` / `wb_r` / `ofb_w`：SRAM 访问次数

FAIL 分支只打 `cycles + mismatches + name`（无 perf counters）。

`CASE_PROFILE` 行包含 4 个核心 V/R 接口（act / wgt / psum / acc）的 {fire, stall, idle} 三计数（解析见 `run_regression.py`）。`run_regression` 把这些计数转成 % 后输出到 `regression_report.txt` 的 PROFILE 表，供分析瓶颈用：fire 高 → 接口顺畅；stall 高 → 该接口被下游卡；idle 高 → 该接口在等上游。

---

## 5.5 SMC chain 自动性能报告（PERF_* → CSV，架构无关）

多核 SMC chain TB（`sim/tb_smc/tb_smc_chain.sv`）跑完**自动**生成 CSV 性能报告，
无论什么模型 / 层数 / 输入 / 核数都能直接拿到原始数据（机制类比 `regression_report`）。

**机制**：TB 打印一组**机读 tag 行**（`run -all` 末尾），`run.tcl` 用 `-onfinish stop`
让 `$finish` 停回 tcl 后自动调 `toolchain/gen_perf_report.py transcript` 解析成 CSV：

| tag 行 | 内容 | → CSV |
|---|---|---|
| `PERF_RUN result= layers= cores= ddrs= wall_cyc= clk_mhz=` | run 总览 | `<case>_run.csv`（+派生 fps / mac_pipe_pct） |
| `PERF_LAYER l= cyc=` | 每层周期 | `<case>_layer.csv`（+pct_of_wall / mac_pipe_pct） |
| `PERF_LAYER_CORE l= c= cyc= fire= util_pct= *_stall= *_idle=` | 每层每核 | `<case>_layer_core.csv`（最细粒度原始数据） |
| `PERF_CORE c= idma_grant= … data_cyc= done_cyc=` | 每核 arb+dispatcher | `<case>_core.csv` |
| `PERF_DDR d= busy_cyc= aw_fire= w_beats= ar_fire= r_beats=` | 每 DDR 流量 | `<case>_ddr.csv`（+busy_pct） |

**架构无关**：`cores/layers/ddrs` 由 `PERF_RUN` 自描述，parser 用通用 `key=value` 解析
（新增字段自动进 CSV header），缺某类行则不出该 CSV。`mac_pipe_pct = Σfire/(cores×wall)`
是**硬件 pipe 利用率**（非 ops 算术利用率，后者需每层理论 MAC）。

**手动重生成**（事后从任意 log）：
```bash
python toolchain/gen_perf_report.py <sim_log> --out-dir <case_dir> --case <name>
```

新 TB 要接入这套报告，只需照上表打印 `PERF_*` 行即可被同一 parser 解析。

---

## 6. 文件清单和编译顺序

`sim/tb_core_dma/run.tcl`：

```
vlog -sv ... \
    ../../RTL/std_rf.sv \
    ../../RTL/sdp.sv \
    ../../RTL/sram_model.sv \
    ../../RTL/AXI4/axi_arbiter.sv \
    ../../RTL/AXI4/axi_m_mux.sv \
    ../../RTL/AXI4/axi_lite_csr.sv \
    ../../RTL/DMA/idma_ctrl.sv ../../RTL/DMA/wdma_ctrl.sv ../../RTL/DMA/odma_ctrl.sv \
    ../../RTL/DMA/mm2s_arb.sv \
    ../../RTL/DMA/dfe.sv ../../RTL/desc_fifo.sv \
    ../../RTL/sequencer.sv ../../RTL/cfg_regs.sv \
    ../../RTL/mac_pe.sv ../../RTL/mac_col.sv ../../RTL/mac_array.sv \
    ../../RTL/parf_col.sv ../../RTL/parf_accum.sv \
    ../../RTL/line_buffer.sv ../../RTL/wgt_buffer.sv ../../RTL/ofb_writer.sv \
    ../../RTL/core_top.sv \
    ../tb_axi_m_mux/axi_slave_mem.sv \
    tb_core_dma.sv

vsim -c -voptargs="+acc" -sva -f sim_params.f -L work work.tb_core_dma
run -all
```

添加新 RTL 模块时同步：`run.tcl` 和 `Syn/run_syn.tcl`。

---

## 7. 子模块单测

```bash
cd sim/tb_axi_lite_csr && vsim -c -do run.tcl    # AXI-Lite CSR
cd sim/tb_axi_m_mux    && vsim -c -do run.tcl    # AXI4 M mux
cd sim/tb_axi_dm_smoke && vsim -c -do run.tcl    # axi_dm IP elab smoke
cd sim/tb_idma_ctrl      && vsim -c -do run.tcl    # idma_ctrl + axi_dm DDR→IFB
```

DMA 子系统改用 Xilinx AXI DataMover IP 后, 单 DMA 单测合并: idma_ctrl 有
独立 TB; wdma_ctrl / odma_ctrl 通过 tb_core_dma 的端到端回归覆盖.
