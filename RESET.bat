@echo off
cls
echo ==========================================
echo         VERILERI SIFIRLA (RESET)
echo ==========================================
echo Bu islem kanat modellerini tamamen temizler.
echo Devam etmek icin 3 kez onay vermeniz gerekmektedir.
echo.

set /p o1="Onay 1/3 [Y/N]: "
if /i "%o1%" neq "Y" goto BITIR

set /p o2="Onay 2/3 [Y/N]: "
if /i "%o2%" neq "Y" goto BITIR

set /p o3="Onay 3/3 [Y/N]: "
if /i "%o3%" neq "Y" goto BITIR

echo.
echo Siliniyor...
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

if exist "%SCRIPT_DIR%kanat_modeller" (
    rd /s /q "%SCRIPT_DIR%kanat_modeller"
)
md "%SCRIPT_DIR%kanat_modeller"

echo.
echo Tamamlandi.
pause > nul
exit

:BITIR
echo Iptal edildi.
pause > nul
