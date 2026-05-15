@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID WING PLANFORM PERFORMANCE PLOTTER
chcp 65001 > nul
cls
echo ======================================================================
echo     RAPID WING PLANFORM AERODYNAMIC PERFORMANCE PLOTTER - STANDALONE
echo ======================================================================
echo.
echo [i] Loading performance map (Range Efficiency vs. Endurance Factor)...
echo.

".\system_files\python_runtime\python.exe" ".\system_files\plotter_performance.py"

echo.
echo Execution completed. Press any key to close...
pause > nul
