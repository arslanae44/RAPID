@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID WING PLANFORM OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo        RAPID WING PLANFORM OPTIMIZATION ENGINE - STANDALONE
echo ======================================================================
echo.

".\system_files\python_runtime\python.exe" ".\system_files\planform_opti.py"

echo.
echo Execution completed or stopped.
echo Press any key to close...
pause > nul
