@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title TAI OPTIMIZATION PLOTTER
chcp 65001 > nul
cls
echo ======================================================================
echo              TAI PARETO VISUALIZATION MODULE
echo ======================================================================
echo.

".\system_files\python_runtime\python.exe" ".\system_files\plotter.py"

echo.
echo Grafik olusturuldu ve kaydedildi.
echo Kapatmak icin bir tusa basin...
pause > nul
