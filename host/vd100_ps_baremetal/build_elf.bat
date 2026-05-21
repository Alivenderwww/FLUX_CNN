@echo off
REM build_elf.bat -- 编译 resnet11_app.elf (用 Vitis makefile)
setlocal
call "D:\Xilinx\Vitis\2023.2\settings64.bat"
cd /d "%~dp0workspace2\resnet11_app\Debug"
make all 2>&1
endlocal
