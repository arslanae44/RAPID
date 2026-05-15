@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title TAI WING PLANFORM PERFORMANCE PLOTTER
chcp 65001 > nul
cls
echo ======================================================================
echo      TAI WING PLANFORM AERODYNAMIC PERFORMANCE PLOTTER - STANDALONE
echo ======================================================================
echo.
echo [i] Performans haritasi yukleniyor (Menzil vs. Havada Kalis)...
echo.

".\system_files\python_runtime\python.exe" ".\system_files\plotter_performance.py"

echo.
echo Calisma tamamlandi. Kapatmak icin bir tusa basin...
pause > nul
