# =============================================================================
# run_demo_full.ps1  --  端到端自动化 demo bring-up:
#   Step 1: Vivado tcl 烧 PDI (跟 HW Manager 同 backend, 重烧无需 power-cycle)
#   Step 2: xsct connect Vivado hw_server + dow ELF + con (启 lwIP server)
#
# 用法:
#   .\run_demo_full.ps1                        # 用 default PDI/ELF 路径
#   .\run_demo_full.ps1 -PDI ... -ELF ...      # 自定义路径
# =============================================================================

param(
    # default 烧 with_elf.pdi (含嵌入 A72 ELF), PLM 自启 A72; ELF 参数仅供 fallback xsct dow
    [string]$PDI = 'C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\vd100_with_elf.pdi',
    [string]$ELF = 'C:\_Project\FLUX_CNN\host\vd100_ps_baremetal\workspace2\resnet11_app\Debug\resnet11_app.elf',
    [switch]$SkipDow = $true   # default skip Step 2 (PDI 自带 ELF, PLM load); 加 -SkipDow:$false 强制 xsct dow
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Vivado = 'D:\Xilinx\Vivado\2023.2\bin\vivado.bat'
$RunXsct = Join-Path $ScriptDir 'run_xsct.ps1'

# 检查文件
foreach ($f in @($PDI, $ELF, $Vivado, $RunXsct)) {
    if (-not (Test-Path $f)) { Write-Error "File missing: $f"; exit 1 }
}

# Step 1: Vivado tcl 烧 PDI
Write-Host "=================================================="
Write-Host " Step 1/2: Program PDI via Vivado HW Manager backend"
Write-Host "          PDI: $PDI"
Write-Host "=================================================="
$progTcl = Join-Path $ScriptDir 'program_pdi_via_vivado.tcl'
& $Vivado -mode batch -source $progTcl -tclargs $PDI -nojournal -nolog
if ($LASTEXITCODE -ne 0) {
    Write-Error "Step 1 PDI program failed (Vivado exit code $LASTEXITCODE)"
    exit 1
}
Write-Host "Step 1 OK"

# Step 2 跳过: PDI 内嵌 A72 ELF (bootgen 打包), PLM 是 root 自动 load + 启 A72.
# 不需要 xsct dow ELF 走 firewall.
if ($SkipDow) {
    Write-Host ""
    Write-Host "=================================================="
    Write-Host " Step 2 SKIPPED (PDI 内嵌 A72 ELF, PLM 自动 load + 启 A72)"
    Write-Host "          PMC boot ~3s + A72 lwIP init ~2s after Step 1."
    Write-Host "          ping 169.254.111.10 → 应该通"
    Write-Host "=================================================="
    Write-Host ""
    Write-Host "Run PC GUI now:"
    Write-Host '   & C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe `'
    Write-Host '       C:\_Project\FLUX_CNN\host\vd100_pc\resnet11_gui.py'
    Write-Host '   IP=169.254.111.10  Port=5000'
    exit 0
}

# 强制走 xsct dow ELF (debugging only) — A72 DAP 写 DDR 受 firewall 限制
Write-Host ""
Write-Host "=================================================="
Write-Host " Step 2/2: xsct dow ELF + con on A72 #0 (forced)"
Write-Host "          ELF: $ELF"
Write-Host "=================================================="
$dowTcl = Join-Path $ScriptDir 'dow_elf_only.tcl'
@"
# auto-gen by run_demo_full.ps1
puts "INFO Selecting A72 #0..."
targets -set -filter {name =~ "*A72 #0*"}
puts "INFO rst -processor..."
catch {rst -processor}
puts "INFO Downloading ELF: $($ELF -replace '\\','/')"
dow $($ELF -replace '\\','/')
puts "INFO Continue execution..."
con
puts ""
puts "============================================================"
puts " A72 lwIP server running."
puts " 板 IP 169.254.111.10:5000  (PC 169.254.111.133)"
puts "============================================================"
"@ | Set-Content -Path $dowTcl -Encoding UTF8

& $RunXsct $dowTcl
if ($LASTEXITCODE -ne 0) {
    Write-Error "Step 2 ELF dow failed (xsct exit code $LASTEXITCODE)"
    exit 1
}

Write-Host ""
Write-Host "=================================================="
Write-Host " ALL DONE. Run PC GUI now:"
Write-Host '   & C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe `'
Write-Host '       C:\_Project\FLUX_CNN\host\vd100_pc\resnet11_gui.py'
Write-Host '   IP=169.254.111.10  Port=5000'
Write-Host "=================================================="
