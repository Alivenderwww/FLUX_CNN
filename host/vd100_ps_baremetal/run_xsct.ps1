# =============================================================================
# run_xsct.ps1  --  跑 xsct .tcl 脚本的 wrapper (无人工干预)
#
# 修复: xsct/xsdb 默认启动的 hw_server (Vitis 路径下) 不加载 Digilent JTAG plugin,
# 看不到 cable. Vivado 路径下的 hw_server 能. 此 wrapper:
#   1. 杀所有残留 hw_server.exe
#   2. 直接启动 Vivado 路径下 hw_server.exe (不通过 .bat 中转, 子进程可控)
#   3. 等 3 秒让它 listen tcp:3121
#   4. 启 xsct + 注入 connect -url + source 用户 tcl
#   5. 退出后 kill 所有 hw_server.exe (彻底清理, 不留残留窗口)
#
# 用法:
#   .\run_xsct.ps1 jtag_probe_robust.tcl
#   .\run_xsct.ps1 C:\full\path\to\script.tcl
# =============================================================================

param([Parameter(Mandatory=$true)][string]$TclScript)

# Vivado .bat 实际调的是 unwrapped/win64.o/hw_server.exe; 直接调 .exe 这样 PID 就是 hw_server 本身,
# 不是 cmd.exe 中转, kill 时直接生效. 跟 .bat 启的功能等价 (Vivado 安装时 PATH 应该已含必要 DLL)
$VivadoHwServerBat = 'D:\Xilinx\Vivado\2023.2\bin\hw_server.bat'
$Xsct              = 'D:\Xilinx\Vitis\2023.2\bin\xsct.bat'

if (-not (Test-Path $TclScript))        { Write-Error "Tcl script not found: $TclScript"; exit 1 }
if (-not (Test-Path $VivadoHwServerBat)) { Write-Error "Vivado hw_server.bat not found: $VivadoHwServerBat"; exit 1 }

function Stop-AllHwServer {
    Get-Process hw_server -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
}

# 1. 杀残留
Write-Host "[1/5] Kill stale hw_server.exe (all instances)..."
Stop-AllHwServer
Start-Sleep -Milliseconds 500

# 2. 启 Vivado hw_server.bat (.bat 里有 plugin path 设置, 直接 .exe 不行)
#    cleanup 不靠 PID (.bat 启的是 cmd 中转), 而是用 Stop-AllHwServer 按名字杀
Write-Host "[2/5] Launch Vivado hw_server.bat (含 Digilent plugin path setup)..."
$hwsProc = Start-Process -FilePath $VivadoHwServerBat -PassThru -WindowStyle Minimized
Write-Host "      cmd PID = $($hwsProc.Id)  (hw_server.exe 是子进程, cleanup 按 name 杀)"

# 3. 等 listen
Write-Host "[3/5] Waiting 3s for hw_server to listen tcp:3121..."
Start-Sleep -Seconds 3

# 4. 生成 wrapper tcl: 先 connect -url 再 source 用户 tcl
$tmpTcl = [System.IO.Path]::GetTempFileName() + '.tcl'
@"
connect -url tcp:127.0.0.1:3121
source $($TclScript -replace '\\','/')
"@ | Set-Content -Path $tmpTcl -Encoding UTF8
Write-Host "[4/5] Run xsct (sources $TclScript)"

# xsct 跑用户脚本, 加 120 秒 timeout 防 hang
$xsctProc = Start-Process -FilePath $Xsct -ArgumentList $tmpTcl -PassThru -NoNewWindow
$timeoutMs = 120000
if (-not $xsctProc.WaitForExit($timeoutMs)) {
    Write-Warning "xsct hang for ${timeoutMs}ms, killing it (检查板状态 / power-cycle)..."
    Stop-Process -Id $xsctProc.Id -Force -ErrorAction SilentlyContinue
    $exitCode = 99
} else {
    $exitCode = $xsctProc.ExitCode
}

# 5. 清理: kill 所有 hw_server.exe + cmd shell 中转 + 删 tmp tcl
Write-Host "[5/5] Cleanup..."
Remove-Item -Path $tmpTcl -ErrorAction SilentlyContinue
Stop-AllHwServer
Stop-Process -Id $hwsProc.Id -Force -ErrorAction SilentlyContinue   # cmd shell 中转
Start-Sleep -Milliseconds 500

# Verify cleanup
$residual = Get-Process hw_server -ErrorAction SilentlyContinue
if ($residual) {
    Write-Warning "hw_server.exe still alive (PID: $($residual.Id -join ',')). Try Stop-Process manually."
} else {
    Write-Host "      cleanup OK, no hw_server.exe left."
}

Write-Host "Done. xsct exit code: $exitCode"
exit $exitCode
