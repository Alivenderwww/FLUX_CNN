# 仿真运行指南

主仿真目录是 `sim/tb_core_dma/`（端到端 descriptor-driven 测试）。工具链脚本在 `toolchain/`。

## 1. 端到端回归（推荐入口）

```bash
cd toolchain
python run_regression.py                      # 无 fold 基线, 22 case
python run_regression.py --fold               # Ky-fold 启用
python run_regression.py --fold --kx-fold     # Ky + Kx fold (最优)

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
4. 解析每个 `CASE_RESULT` 行，汇总成 `regression_report.txt`

### 报告示例（Ky+Kx fold）

```
[ 0] PASS  cycles=1,107,469  MAC= 99.9%  K=7_C4C8____960x540_s2_p3|conv   (stem)
[ 1] PASS  cycles=  144,520  MAC= 96.3%  K=3_C8C8____240x135_s1_p1|conv   (L1.B1.Conv1)
[ 6] PASS  cycles=   75,988  MAC= 96.7%  K=3_C16C16__120x68__s1_p1|conv   (L2.B1.Conv2)
[16] PASS  cycles=   78,934  MAC= 93.0%  K=3_C64C64__30x17___s1_p1|conv   (L4.B1.Conv2)
[20] PASS  cycles=    9,843  MAC=  5.2%  FC1_C256C512_1x1|conv            (FC1, 1x1 输出低 util)
```

---

## 2. 单 case 生成 + 手动跑

```bash
cd toolchain
python gen_isa_test.py \
    --k 3 --h_in 68 --w_in 120 \
    --num_cin 8 --num_cout 8 \
    --stride 1 --pad 1 --shift 0

# 带 fold
python gen_isa_test.py --k 7 --h_in 960 --w_in 540 \
    --num_cin 4 --num_cout 8 --stride 2 --pad 3 \
    --ky-fold --kx-fold

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
CASE_RESULT 0 PASS  cycles=1107469  mac_fire=1106208  mac_util=71.68%
                    arf_w=1105920  arf_r=1105920  parf_f=1105920  parf_d=129600
                    ifb_r=...      wb_r=...        ofb_w=129600
                    name=K=7_C4C8____960x540_s2_p3|conv
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
