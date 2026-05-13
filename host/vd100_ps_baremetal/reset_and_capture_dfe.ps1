$ErrorActionPreference = 'Continue'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Vivado = 'D:\Xilinx\Vivado\2023.2\bin\vivado.bat'
$PDI = "$ScriptDir\vd100_with_elf.pdi"
$Python = 'C:\_Project\FLUX_CNN\toolchain\.venv\Scripts\python.exe'
$TestScript = 'C:\_Project\FLUX_CNN\host\vd100_pc\test_layer0_real.py'

Write-Host '=== Step 1: 烧 PDI (a72 reboot) ==='
& $Vivado -mode batch -source "$ScriptDir\program_pdi_via_vivado.tcl" -tclargs $PDI -nojournal -nolog | Select-Object -Last 5
Start-Sleep -Seconds 6
$pingOK = $false
for ($i=0; $i -lt 10; $i++) {
    if (Test-Connection -ComputerName 169.254.111.10 -Count 1 -Quiet -ErrorAction SilentlyContinue) { $pingOK = $true; break }
    Start-Sleep -Seconds 2
}
if (-not $pingOK) { Write-Error 'ping fail'; exit 1 }
Write-Host '  board ping OK'

Write-Host '=== Step 3: ILA armed on start_dfe_pulse (bg) ==='
$ilaJob = Start-Job -Name 'ila_dfe' -ScriptBlock {
    param($Vivado, $ScriptDir)
    Set-Location $ScriptDir
    & $Vivado -mode batch -source "$ScriptDir\capture_ila_dfe_phase.tcl" -nojournal -log ila_dfe.log
} -ArgumentList $Vivado, $ScriptDir
Start-Sleep -Seconds 30

Write-Host '=== Step 4: PC test_layer0_real.py ==='
Set-Location 'C:\_Project\FLUX_CNN\host\vd100_pc'
& $Python $TestScript 2>&1 | Select-Object -First 30 | Out-Null
Write-Host '  test_layer0_real triggered (truncated output)'

Write-Host '=== Step 5: 等 ILA job 退 ==='
Wait-Job -Job $ilaJob -Timeout 120 | Out-Null
$out = Receive-Job -Job $ilaJob
Write-Host '--- ILA tail ---'
$out | Select-Object -Last 15 | Write-Host
Remove-Job -Job $ilaJob -Force
