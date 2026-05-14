# =============================================================================
# run_minimal_test.ps1  --  vd100_minimal 烧 PDI + 跑 Stage 0/1a 测试
#
# Step 1: vivado HW Manager 烧 PDI
# Step 2: vivado HW Manager 跑 Stage 0 (BRAM loopback)
# Step 3: vivado HW Manager 跑 Stage 1a (CSR peek/poke)
#
# 用法: .\run_minimal_test.ps1
# =============================================================================

param(
    [string]$PDI = 'C:\_Project\FLUX_CNN\Syn\vd100_minimal\output\vd100_minimal.runs\impl_1\design_1_wrapper.pdi',
    [switch]$SkipProgram = $false
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Vivado = 'D:\Xilinx\Vivado\2023.2\bin\vivado.bat'

if (-not (Test-Path $PDI)) {
    Write-Error "PDI not found: $PDI"
    Write-Error "Run create_project.tcl + run_synth.tcl first"
    exit 1
}

# 把 PDI 烧入 + 跑测试用一个 tcl 完成
$comboTcl = Join-Path $env:TEMP 'vd100_minimal_run.tcl'
@"
# Step 1: 烧 PDI
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices xcve*] 0]
if {`$dev eq ""} { error "No Versal device found" }
current_hw_device `$dev
puts "INFO target device: `$dev"

if {!$SkipProgram} {
    puts "=== Step 1: Program PDI ==="
    set_property PROGRAM.FILE [list $PDI] `$dev
    program_hw_devices `$dev
    puts "  programmed"
}

# Step 2: Stage 0 BRAM loopback
puts ""
puts "=== Step 2: Stage 0 BRAM loopback ==="
source $ScriptDir/test_stage0_bram_loopback.tcl

# Step 3: Stage 1a CSR peek/poke
puts ""
puts "=== Step 3: Stage 1a CSR peek/poke ==="
source $ScriptDir/test_stage1a_csr_peek_poke.tcl

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
"@ | Set-Content -Path $comboTcl -Encoding UTF8

Write-Host "=== Run vivado + program PDI + tests ==="
& $Vivado -mode batch -source $comboTcl -nojournal -nolog
$rc = $LASTEXITCODE
Remove-Item -Path $comboTcl -ErrorAction SilentlyContinue
Write-Host "vivado exit code: $rc"
exit $rc
