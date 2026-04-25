# 仿真运行指南

主仿真目录是 `sim/tb_core_dma/`（端到端 descriptor-driven 测试）。工具链脚本在 `toolchain/`。

## 1. 端到端回归（推荐入口）

```bash
cd toolchain
python run_regression.py                      # 无 fold 基线, 22 case
python run_regression.py --fold               # Ky-fold 启用
python run_regression.py --fold --kx-fold     # Ky + Kx fold
python run_regression.py --s2d                # Space-to-Depth (stride>=2 case)
python run_regression.py --fold --kx-fold --s2d   # 三个一起

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

# 带 fold
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 \
    --num_cin 4 --num_cout 8 --stride 2 --pad 4 \
    --ky-fold --kx-fold

# 带 S2D (stride>=2)
python gen_isa_test.py --k 8 --h_in 960 --w_in 540 \
    --num_cin 4 --num_cout 8 --stride 2 --pad 4 \
    --s2d --kx-fold

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
        "FOLD_COUT_ORIG" : axi_lite_write(ADDR_FOLD_COUT_ORIG, val);
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
    ../../RTL/DMA/idma.sv ../../RTL/DMA/wdma.sv ../../RTL/DMA/odma.sv \
    ../../RTL/DMA/dfe.sv ../../RTL/desc_fifo.sv \
    ../../RTL/sequencer.sv ../../RTL/cfg_regs.sv \
    ../../RTL/mac_pe.sv ../../RTL/mac_col.sv ../../RTL/mac_array.sv \
    ../../RTL/parf_col.sv ../../RTL/parf_accum.sv ../../RTL/psum_reshape.sv \
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
cd sim/tb_idma         && vsim -c -do run.tcl    # IDMA descriptor
cd sim/tb_wdma         && vsim -c -do run.tcl    # WDMA
cd sim/tb_odma         && vsim -c -do run.tcl    # ODMA
```
