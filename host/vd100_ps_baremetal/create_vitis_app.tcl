# =============================================================================
# create_vitis_app.tcl  --  Vitis 2023.2 自动创建 ResNet11 baremetal lwIP app
#
# 前提: Vivado full_flow.tcl 跑完, 拿到 .xsa:
#   C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa
#
# 用法 (PowerShell):
#   "D:\Xilinx\Vitis\2023.2\bin\xsct.bat" -nodisp C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/create_vitis_app.tcl
#
# 注: Vitis 2023.2 用 xsct (Xilinx Software Command-line Tool), 不是 vitis -i
#
# 跑完: 在 host/vd100_ps_baremetal/workspace/ 下产生:
#   - vd100_platform/    Versal AI Edge platform (从 .xsa 派生, 含 BSP)
#   - resnet11_app/      baremetal app (lwIP echo server template + 用户改 echo.c)
#   - resnet11_app_system/  system project (上板 elf 包)
# =============================================================================

set XSA       "C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xsa"
set WORKSPACE "C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/workspace"
set PLATFORM_NAME "vd100_platform"
set APP_NAME      "resnet11_app"
set DOMAIN_NAME   "standalone_psv_cortexa72_0"
# Versal A72 完整 BD path (Vitis 2023.2): versal_cips_0_pspmc_0_psv_cortexa72_0
set PROCESSOR     "versal_cips_0_pspmc_0_psv_cortexa72_0"

# 1. 创建 workspace
file mkdir $WORKSPACE
setws $WORKSPACE
puts "Workspace: $WORKSPACE"

# 2. Platform from XSA (Versal AI Edge VE2302)
if {![file exists $XSA]} {
    puts "ERROR: XSA not found: $XSA"
    puts "Run Vivado full_flow.tcl first."
    exit 1
}
puts "Creating platform from $XSA ..."
platform create -name $PLATFORM_NAME -hw $XSA -no-boot-bsp
platform active $PLATFORM_NAME

# 3. Domain (baremetal A72 standalone)
domain create -name $DOMAIN_NAME -proc $PROCESSOR -os standalone
domain active $DOMAIN_NAME

# 4. lwIP library (BSP add) — baremetal echo server template 自带 lwip213
puts "Adding lwIP library to BSP..."
bsp setlib -name lwip213
bsp regenerate

# 5. Build platform
puts "Building platform (5-10 min)..."
platform generate

# 6. Application: lwIP echo server template
puts "Creating app $APP_NAME from lwIP echo template..."
app create -name $APP_NAME \
    -platform $PLATFORM_NAME \
    -domain   $DOMAIN_NAME \
    -template "lwIP Echo Server"

# 7. 替换 echo.c 为我们的 resnet11_main.c
set src_dir "$WORKSPACE/$APP_NAME/src"
set our_main "C:/_Project/FLUX_CNN/host/vd100_ps_baremetal/resnet11_main.c"
if {[file exists "$src_dir/echo.c"]} {
    file delete "$src_dir/echo.c"
    puts "  removed echo.c"
}
file copy -force $our_main "$src_dir/echo.c"
puts "  copied resnet11_main.c -> $src_dir/echo.c (name kept for lwIP template's main.c reference)"

# 8. Build app
puts "Building app $APP_NAME (3-5 min)..."
app build -name $APP_NAME

puts ""
puts "============================================================"
puts " Vitis app build done!"
puts " ELF: $WORKSPACE/$APP_NAME/Debug/$APP_NAME.elf"
puts ""
puts " Next: Vitis GUI Run As 'Launch Hardware (Single Application Debug)'"
puts " or xsct: jtag connect; jtag target ...; dow elf; con"
puts "============================================================"
