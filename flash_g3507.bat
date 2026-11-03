@echo off
setlocal
cd /d "%~dp0"
title Flash MSPM0G3507 Car Firmware

set UV4=D:\Application\Keil5\UV4\UV4.exe
if not exist "%UV4%" set UV4=C:\Keil_v5\UV4\UV4.exe
if not exist "%UV4%" set UV4=D:\Keil_v5\UV4\UV4.exe

set KEIL_DIR=%~dp0NUEDC_2026\[Demo] MSPM0_Car_v5\project\keil
set UVPROJX=%KEIL_DIR%\NUEDC2025_MSPM0G3507.uvprojx

echo ========================================================
echo   Flashing MSPM0G3507 TianMengXing Firmware (Keil)...
echo ========================================================
echo Project: %UVPROJX%
echo Tool:    %UV4%
echo.

if not exist "%UVPROJX%" goto NO_PROJECT
if not exist "%UV4%" goto NO_TOOL

cd /d "%KEIL_DIR%"
"%UV4%" -f "%UVPROJX%" -t NUEDC2025_MSPM0G3507 -j0
if errorlevel 1 goto FLASH_FAIL

echo.
echo ========================================================
echo [SUCCESS] MSPM0G3507 flashed successfully!
echo The onboard OLED should enter "RC Car (MSP430)" task.
echo ========================================================
goto END

:NO_PROJECT
echo [ERROR] Keil project not found: %UVPROJX%
goto END

:NO_TOOL
echo [ERROR] Keil UV4.exe not found: %UV4%
echo Please check Keil5 installation directory.
goto END

:FLASH_FAIL
echo.
echo ========================================================
echo [NOTICE] Flash command executed with code: %errorlevel%
echo If flash failed, please check:
echo 1. CMSIS-DAP / J-Link / Wireless Debugger is plugged into PC.
echo 2. Car expansion board power switch is turned ON.
echo 3. You can also open the Keil project and press F8 to flash.
echo ========================================================

:END
cd /d "%~dp0"
pause
