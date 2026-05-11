@echo off
REM =============================================================================
REM run_gui.bat -- 双击启动 VD100 ResNet11 demo GUI
REM
REM 用 toolchain venv 的 Python (依赖 Pillow + tkinterdnd2)
REM =============================================================================
setlocal

set "VENV_PY=%~dp0..\..\toolchain\.venv\Scripts\python.exe"
set "GUI=%~dp0resnet11_gui.py"

if not exist "%VENV_PY%" (
    echo [ERROR] toolchain venv not found: %VENV_PY%
    echo Please run: cd toolchain ^&^& python -m venv .venv ^&^& .venv\Scripts\pip install Pillow tkinterdnd2
    pause
    exit /b 1
)

echo Launching GUI with: %VENV_PY%
"%VENV_PY%" "%GUI%"

if errorlevel 1 (
    echo.
    echo [ERROR] GUI exited with error code %errorlevel%
    pause
)

endlocal
