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

rem OpenVSP 3.47.0 only ships for Python 3.13 / 3.11. Find one via the py launcher.
set "PYLAUNCH="
py -3.13 --version >nul 2>&1 && set "PYLAUNCH=py -3.13"
if not defined PYLAUNCH ( py -3.11 --version >nul 2>&1 && set "PYLAUNCH=py -3.11" )
if not defined PYLAUNCH (
    echo [!] Python 3.13 or 3.11 ^(64-bit^) was not found.
    echo     OpenVSP 3.47.0 only works with those versions.
    echo     Install Python 3.13 from https://www.python.org/downloads/ and re-run SETUP.bat.
    pause
    exit /b 1
)

echo [i] Creating local environment (.venv) with %PYLAUNCH% ...
%PYLAUNCH% -m venv .venv
set "VPY=%SCRIPT_DIR%.venv\Scripts\python.exe"

echo.
echo [1/4] Installing Python dependencies...
"%VPY%" -m pip install --upgrade pip
"%VPY%" -m pip install -r requirements.txt

echo.
echo [2/4] Downloading OpenVSP 3.47.0...
"%VPY%" ".\system_files\download_openvsp.py"

echo.
echo [3/4] Downloading XFOIL 6.99...
"%VPY%" ".\system_files\download_xfoil.py"

echo.
echo [4/4] Downloading UIUC airfoil database...
"%VPY%" ".\system_files\download_airfoils.py"

echo.
echo ======================================================================
echo  Setup finished. RUN_*.bat and CONFIGURE.bat use .venv automatically.
echo ======================================================================
pause > nul
