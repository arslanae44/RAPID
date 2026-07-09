@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID BWB (TAILLESS) PLANFORM OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo   RAPID BWB / TAILLESS PLANFORM OPTIMIZER
echo ======================================================================
echo.
echo  Uses system_files\bwb_config.json (BWB mode ON): static-margin +
echo  trim stability constraints and a reflexed airfoil catalog.
echo.
set "RAPID_CONFIG_FILE=%SCRIPT_DIR%system_files\bwb_config.json"
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=%SCRIPT_DIR%.venv\Scripts\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\planform_opti.py"
echo.
echo Execution completed or stopped. Press any key to close...
pause > nul
