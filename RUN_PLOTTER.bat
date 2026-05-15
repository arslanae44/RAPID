@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID OPTIMIZATION PLOTTER
chcp 65001 > nul
cls
echo ======================================================================
echo             RAPID PARETO VISUALIZATION MODULE
echo ======================================================================
echo.

".\system_files\python_runtime\python.exe" ".\system_files\plotter.py"

echo.
echo Visualization completed successfully.
echo Press any key to close...
pause > nul
