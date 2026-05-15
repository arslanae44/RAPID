@echo off
cls
echo ==========================================
echo             SYSTEM RESET
echo ==========================================
echo This operation permanently deletes all evaluated wing models.
echo You must confirm 3 times to proceed.
echo.

set /p o1="Confirm 1/3 [Y/N]: "
if /i "%o1%" neq "Y" goto END

set /p o2="Confirm 2/3 [Y/N]: "
if /i "%o2%" neq "Y" goto END

set /p o3="Confirm 3/3 [Y/N]: "
if /i "%o3%" neq "Y" goto END

echo.
echo Purging directories...
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

if exist "%SCRIPT_DIR%wing_models" (
    rd /s /q "%SCRIPT_DIR%wing_models"
)
md "%SCRIPT_DIR%wing_models"

echo.
echo Cleanup completed successfully.
pause > nul
exit

:END
echo Canceled.
pause > nul
