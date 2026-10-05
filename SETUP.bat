@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID SETUP
chcp 65001 > nul
cls
echo ======================================================================
echo             RAPID ONE-TIME SETUP
echo ======================================================================
echo.
rem  SETUP.bat /unattended   - no prompts, no final pause (CI / scripted installs)
set "UNATTENDED="
if /i "%~1"=="/unattended" set "UNATTENDED=1"

rem OpenVSP 3.47.0 only ships for Python 3.13 / 3.11. Find one via the py launcher.
call :find_python
if defined PYLAUNCH goto :have_python
echo [!] Python 3.13 or 3.11 (64-bit) was not found.
echo     OpenVSP 3.47.0 only works with those versions (3.12 / 3.14 will NOT work).
where winget >nul 2>&1
if errorlevel 1 goto :no_python
if defined UNATTENDED goto :no_python
set "ANS="
set /p "ANS=    Install Python 3.13 for this user now with winget? (Y/N): "
if /i not "%ANS%"=="Y" goto :no_python
winget install -e --id Python.Python.3.13 --scope user
call :find_python
if not defined PYLAUNCH goto :no_python

:have_python

echo [i] Creating local environment (.venv) with %PYLAUNCH% ...
%PYLAUNCH% -m venv .venv
set "VPY=%SCRIPT_DIR%.venv\Scripts\python.exe"
if not exist "%VPY%" (
    echo [!] Could not create .venv
    goto :end_fail
)

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
echo  Verifying installation
echo ======================================================================
"%VPY%" ".\system_files\check_install.py"
if errorlevel 1 goto :end_fail
echo.
echo  Setup finished. RUN_*.bat and CONFIGURE.bat use .venv automatically.
echo ======================================================================
if not defined UNATTENDED pause > nul
exit /b 0

:find_python
set "PYLAUNCH="
py -3.13 -c "import struct,sys; sys.exit(struct.calcsize('P')!=8)" >nul 2>&1 && set "PYLAUNCH=py -3.13"
if not defined PYLAUNCH ( py -3.11 -c "import struct,sys; sys.exit(struct.calcsize('P')!=8)" >nul 2>&1 && set "PYLAUNCH=py -3.11" )
exit /b 0

:no_python
echo     Install 64-bit Python 3.13 from https://www.python.org/downloads/ and re-run SETUP.bat.
:end_fail
echo.
echo [!] SETUP did not complete. See the messages above (and SETUP.md).
if not defined UNATTENDED pause
exit /b 1
