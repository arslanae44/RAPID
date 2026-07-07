@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID BWB (TAILLESS) PLANFORM OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo      RAPID BWB / TAILLESS PLANFORM OPTIMIZER
echo ======================================================================
echo.
echo  Uses system_files\bwb_config.json (BWB mode ON):
echo    - Stability via static margin (SM = -dCm/dCL) as a constraint
echo    - Trim enforced near the design point
echo    - Reflexed airfoil catalog for the co-optimization sweep
echo.

set "RAPID_CONFIG_FILE=%SCRIPT_DIR%system_files\bwb_config.json"
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\planform_opti.py"

echo.
echo Execution completed or stopped. Press any key to close...
pause > nul
