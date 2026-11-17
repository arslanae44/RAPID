@echo off
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
title RAPID TRANSONIC PLANFORM OPTIMIZER
chcp 65001 > nul
cls
echo ======================================================================
echo   RAPID TRANSONIC PLANFORM OPTIMIZER  (Navy jet trainer cruise wing)
echo ======================================================================
echo.
echo  Minimizes cruise C_D (induced + profile + wave) and Raymer wing weight
echo  at CL = 0.272, M 0.75, subject to M_dd, pitch-up, tip stall, fuel and Cm.
echo  Results: wing_models\transonic\report\  (report.md, pareto.png, spanwise.png)
echo.
set "POP=40"
set "GENS=30"
set /p POP="Wings per generation [40]: "
set /p GENS="Generations [30]: "
set "PYEXE=%SCRIPT_DIR%system_files\python_runtime\python.exe"
if not exist "%PYEXE%" set "PYEXE=%SCRIPT_DIR%.venv\Scripts\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
"%PYEXE%" ".\system_files\transonic_opti.py" --pop %POP% --gens %GENS% %*
echo.
echo Done. Press any key to close...
pause > nul
