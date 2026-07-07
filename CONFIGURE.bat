@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID CONFIGURATION EDITOR
chcp 65001 > nul
cls
echo ======================================================================
echo             RAPID CONSTRAINT AND PARAMETER EDITOR
echo ======================================================================
echo.
echo  Edit constraints, design bounds and flight conditions.
echo  Each value shows its default and the value you entered last time.
echo.

set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\configure_constraints.py"

echo.
echo Configuration saved. Run RUN_OPTIMIZATION.bat to apply. Press any key...
pause > nul
