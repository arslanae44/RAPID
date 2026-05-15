@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title TAI WING PLANFORM OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo        TAI WING PLANFORM OPTIMIZATION ENGINE - STANDALONE
echo ======================================================================
echo.

".\system_files\python_runtime\python.exe" ".\system_files\planform_opti.py"

echo.
echo Calisma tamamlandi veya durduruldu. 
echo Kapatmak icin bir tusa basin...
pause > nul
