# 打开 axi_dm 的 example project, 看 Xilinx 自己的 testbench 怎么填 cmd word
set PROJ_DIR  C:/_Project/FLUX_CNN/Syn/ip_managed
open_project "$PROJ_DIR/ip_managed.xpr"

# 生成 example project (如果还没有)
set EX_DIR C:/_Project/FLUX_CNN/Syn/axi_dm_example
if {![file exists "$EX_DIR/axi_dm_ex/axi_dm_ex.xpr"]} {
    open_example_project -force -dir $EX_DIR [get_ips axi_dm]
}
puts "==> example dir: $EX_DIR"
