---
name: rtl-verify
description: 在RTL级别验证设计功能和性能，使用仿真工具进行功能回归测试和性能分析。
---

## Simulation Execution
**CRITICAL**: All simulation commands (like `vsim`, `python gen_isa_test.py`, etc.) MUST be executed specifically from within the `sim\tb_core_isa` directory. 
Do NOT run simulation scripts from the project root using relative or absolute paths, as it will cause file generation and path resolution errors.

- **Correct**: `cd C:\_Project\FLUX_CNN\sim\tb_core_isa && vsim -c -do "do tb_core_isa.tcl; quit -f"`
- **Incorrect**: `vsim -do C:\_Project\FLUX_CNN\sim\tb_core_isa\tb_core_isa.tcl`

## RTL Debugging: debug.tcl 波形抓取法
当仿真出现 mismatch 或行为异常时，**不要盲猜**，用自动化 TCL 脚本逐周期追踪 FSM 内部信号。
这比手动打开 GUI 看波形快得多，且输出可以直接在 transcript 文件中 grep 分析。

### 步骤
1. **生成测试数据**（选最小的失败 case）:
   ```bash
   python gen_isa_test.py --k 3 --h_in 10 --w_in 10 --stride 1 --shift 0
   ```

2. **编写 debug.tcl**（按需调整信号列表和循环次数）:
   ```tcl
   vlib work
   vmap work work
   # 必须去掉 -incr，确保用最新 RTL；必须加 -voptargs="+acc" 才能 examine 内部信号
   vlog -sv -work work -mfcu -suppress 2902 -f sim_file_list.f
   vsim -suppress 3486,3680,3781 -voptargs="+acc" +nowarn1 -c -sva \
        -f sim_params.f -L work work.tb_core_isa

   set ns 0
   for {set i 0} {$i < 200} {incr i} {
       run 10ns
       incr ns 10
       set st  [examine -radix symbolic  sim:/tb_core_isa/u_core_top/u_ctrl/current_state]
       set pc  [examine -radix unsigned  sim:/tb_core_isa/u_core_top/u_ctrl/pc]
       set cnt [examine -radix unsigned  sim:/tb_core_isa/u_core_top/u_ctrl/cnt]
       # 按需添加更多信号，例如：
       # set pfv [examine -radix unsigned sim:/tb_core_isa/u_core_top/u_ctrl/pf_valid]
       # set ire [examine -radix unsigned sim:/tb_core_isa/u_core_top/u_ctrl/inst_re]
       puts "T=${ns} st=$st pc=$pc cnt=$cnt"
       if {$st eq "FINISH"} { break }
   }
   quit -f
   ```

3. **运行并过滤输出**:
   ```bash
   vsim -c -do debug.tcl 2>&1 | grep "^T=" | head -80
   ```
   输出会保存在 `transcript` 文件中，也可以直接 `Read transcript` 查看。

### 关键注意事项
- **必须加 `-voptargs="+acc"`**：否则优化器会裁剪内部信号，`examine` 报 "No objects found"
- **必须去掉 `-incr`**：增量编译可能使用缓存的旧 RTL，导致调试的不是最新代码
- **若怀疑编译缓存问题**：先 `rm -rf work` 再编译
- **正式回归 (tb_core_isa.tcl) 不要加 `+acc`**：会拖慢仿真 2-5 倍
- **只打印感兴趣的周期**：用条件过滤（如 `if {$st ne "EXEC_LDnMAC" || $cnt eq "0"}` ）减少输出量
- **examine 路径格式**：`sim:/tb_core_isa/u_core_top/u_ctrl/<signal_name>`，结构体字段用 `.` 分隔（如 `inst_reg.opcode`）

## TCL 仿真性能优化
正式回归用的 `tb_core_isa.tcl` 应保持精简以最大化仿真速度：
- **不加 `-voptargs="+acc"`**：让优化器充分优化
- **不加 `-wlf`**：不生成波形文件（batch 模式用不到）
- **不加 `add wave`**：`-c` 模式下没有 GUI，add wave 纯浪费时间
- **可以用 `-incr`**：增量编译减少重复编译时间（但调试时去掉）