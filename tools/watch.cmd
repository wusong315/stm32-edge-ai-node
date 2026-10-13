@echo off
rem ============================================================
rem  Double-click launcher: live 3-axis view (MPU6050)
rem  Runs: calib_dev.py --watch   (auto-detects the COM port)
rem
rem  Kept ASCII-only on purpose: this box is a Chinese Windows
rem  (console codepage GBK), so non-ASCII text inside a .cmd can
rem  get garbled. All Chinese output comes from Python instead.
rem ============================================================
cd /d "%~dp0"

set "PY=D:\STM32Projects\pc-venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

title MPU6050 - live view (Ctrl-C to stop)
"%PY%" calib_dev.py --watch
echo.
echo ------------------------------------------------------------
echo  Exited. Read the lines above before closing this window.
echo ------------------------------------------------------------
pause
