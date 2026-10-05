@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID STANDALONE AIRFOIL OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo   RAPID STANDALONE AIRFOIL OPTIMIZER
echo ======================================================================
echo.
echo  Reads the wing in the .vsp3 named by system_files\airfoil_config.json,
echo  sweeps XFOIL over an airfoil catalog per spanwise station and writes
echo  system_files\active_airfoils.json for the planform optimizer.
echo.
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=%SCRIPT_DIR%.venv\Scripts\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\airfoil_opti.py" %*
echo.
echo Done. Press any key to close...
pause > nul
