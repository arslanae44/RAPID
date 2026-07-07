@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID SETUP
chcp 65001 > nul
cls
echo ======================================================================
echo             RAPID ONE-TIME SETUP
echo ======================================================================
echo.
echo  Installs the pieces that are not stored in the repository:
echo    - Python dependencies (numpy, scipy, matplotlib, pymoo)
echo    - OpenVSP 3.47.0 engine
echo    - XFOIL 6.99 solver
echo    - Full UIUC airfoil database (~1650 airfoils)
echo.

rem Use the embedded runtime if present, otherwise the system Python.
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"

echo [1/4] Installing Python dependencies...
"%PYEXE%" -m pip install -r requirements.txt

echo.
echo [2/4] Downloading OpenVSP 3.47.0...
"%PYEXE%" ".\system_files\download_openvsp.py"

echo.
echo [3/4] Downloading XFOIL 6.99...
"%PYEXE%" ".\system_files\download_xfoil.py"

echo.
echo [4/4] Downloading UIUC airfoil database...
"%PYEXE%" ".\system_files\download_airfoils.py"

echo.
echo ======================================================================
echo  Setup finished. If 'import openvsp' fails at run time, see SETUP.md
echo  (the OpenVSP Python module must match your Python version).
echo ======================================================================
pause > nul
