@echo off
setlocal
REM ===========================================================================
REM boot.bat -- program VD100 PDI via JTAG (Vivado HW Manager backend).
REM             PLM auto-loads embedded A72 ELF -> lwIP RPC server.
REM
REM Usage:
REM   boot.bat                program default N=3 ResNet11/FR-Net release PDI
REM   boot.bat  <PDI_PATH>    program a specific PDI (e.g. minimal single-core)
REM
REM After ~10s the board runs the lwIP server; ping 169.254.111.10 should reply.
REM Then run:  host\vd100_pc\run_gui.bat
REM ===========================================================================

if "%VIVADO%"=="" set "VIVADO=D:\Xilinx\Vivado\2023.2\bin\vivado.bat"

set "PDI=%~1"
if "%PDI%"=="" set "PDI=%~dp0vd100_ps_baremetal\release_n3_100mhz_2026_05_19\vd100_with_elf.pdi"

set "TCL=%~dp0vd100_ps_baremetal\program_pdi_via_vivado.tcl"

if not exist "%VIVADO%" goto :no_vivado
if not exist "%PDI%" goto :no_pdi
if not exist "%TCL%" goto :no_tcl

echo ==================================================
echo  Programming PDI via JTAG
echo  PDI: %PDI%
echo ==================================================
echo  (If Vivado GUI Hardware Manager is open, Close Target first to free JTAG.)
echo.

call "%VIVADO%" -mode batch -source "%TCL%" -tclargs "%PDI%" -nojournal -nolog
if errorlevel 1 goto :prog_fail

echo.
echo [OK] PDI programmed. Waiting ~10s for A72 lwIP server...
ping -n 11 127.0.0.1 >nul

echo Pinging board 169.254.111.10 ...
ping -n 3 169.254.111.10

echo.
echo Next: run  host\vd100_pc\run_gui.bat
pause
exit /b 0

:no_vivado
echo [ERROR] Vivado not found: %VIVADO%
echo         Override with:  set VIVADO=^<path-to-vivado.bat^>
pause
exit /b 1

:no_pdi
echo [ERROR] PDI not found: %PDI%
pause
exit /b 1

:no_tcl
echo [ERROR] program tcl not found: %TCL%
pause
exit /b 1

:prog_fail
echo.
echo [ERROR] PDI programming failed.
echo         Check: JTAG cable / HW Manager busy in Vivado GUI / board power.
pause
exit /b 1
