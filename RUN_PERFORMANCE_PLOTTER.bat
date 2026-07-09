@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID PERFORMANCE PLOTTER
chcp 65001 > nul
cls
echo ======================================================================
echo   RAPID PERFORMANCE PLOTTER
echo ======================================================================
echo.
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=%SCRIPT_DIR%.venv\Scripts\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\plotter_performance.py"
echo.
echo Done. Press any key to close...
pause > nul
