# =============================================================================
# reset_and_capture_layer_busy.ps1
#  Step 1: Vivado 烧 vd100_with_elf.pdi (PMC + a72 reboot)
#  Step 2: 等 a72 lwIP up (ping 通)
#  Step 3: Vivado armed ILA with trigger on dbg_layer_busy_0 (background)
#  Step 4: PC test_layer0_real.py (触发 ILA)
#  Step 5: ILA dump csv 后 vivado 自动退
# =============================================================================
$ErrorActionPreference = 'Continue'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Vivado = 'D:\Xilinx\Vivado\2023.2\bin\vivado.bat'
$PDI = "$ScriptDir\vd100_with_elf.pdi"
$Python = 'C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe'
$TestScript = 'C:\_Project\FLUX_CNN\host\vd100_pc\test_layer0_real.py'

Write-Host "=== Step 1: 烧 PDI (a72 reboot) ==="
& $Vivado -mode batch -source "$ScriptDir\program_pdi_via_vivado.tcl" -tclargs $PDI -nojournal -nolog
if ($LASTEXITCODE -ne 0) { Write-Error "Step 1 fail"; exit 1 }

Write-Host "=== Step 2: 等 lwIP up (ping board) ==="
Start-Sleep -Seconds 6
$pingOK = $false
for ($i=0; $i -lt 10; $i++) {
    if (Test-Connection -ComputerName 169.254.111.10 -Count 1 -Quiet -ErrorAction SilentlyContinue) {
        $pingOK = $true; break
    }
    Start-Sleep -Seconds 2
}
if (-not $pingOK) { Write-Error "board ping fail"; exit 1 }
Write-Host "  board ping OK"

Write-Host "=== Step 3: 启动 ILA armed (background) ==="
$ilaJob = Start-Job -Name 'ila_armed' -ScriptBlock {
    param($Vivado, $ScriptDir)
    Set-Location $ScriptDir
    & $Vivado -mode batch -source "$ScriptDir\capture_ila_layer_busy_trigger.tcl" -nojournal -log ila_armed.log
} -ArgumentList $Vivado, $ScriptDir

# 等 ILA armed (vivado 启动 + load probes ~25s)
Write-Host "  等 ILA armed (30s)..."
Start-Sleep -Seconds 30

Write-Host "=== Step 4: PC test_layer0_real.py (触发 ILA) ==="
Set-Location 'C:\_Project\FLUX_CNN\host\vd100_pc'
& $Python $TestScript
Write-Host "  test_layer0_real done"

Write-Host "=== Step 5: 等 ILA dump CSV (vivado job 退) ==="
Wait-Job -Job $ilaJob -Timeout 120
$out = Receive-Job -Job $ilaJob
Write-Host "--- ILA job output (tail) ---"
$out | Select-Object -Last 30 | Write-Host
Remove-Job -Job $ilaJob -Force

if (Test-Path 'C:\_Project\FLUX_CNN\Syn\vd100_bd\output\ila_layer_busy.csv') {
    Write-Host "  CSV: C:\_Project\FLUX_CNN\Syn\vd100_bd\output\ila_layer_busy.csv"
} else {
    Write-Error "  ILA CSV missing"
    exit 1
}
