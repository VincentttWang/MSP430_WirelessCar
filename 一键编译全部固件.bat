@echo off
setlocal
cd /d "%~dp0"
title Build All Firmwares - MSP430 Wireless Remote Car

echo ============================================================
echo   MSP430 Wireless Remote Car - Dual Firmware One-Click Build
echo   Author: Wang Jianing (U202413712)
echo ============================================================
echo.

echo ============================================================
echo [1/2] Compiling Host Firmware: MSP430F5529 (TI CCS cl430)...
echo ============================================================
cd /d "%~dp0firmware_MSP430F5529"
call build.bat
if %errorlevel% neq 0 (
    echo [ERROR] MSP430 compilation failed!
    pause
    exit /b 1
)

echo.
echo ============================================================
echo [2/2] Compiling Actuator Firmware: MSPM0G3507 v5 (Keil ARMCLANG)...
echo ============================================================
cd /d "%~dp0NUEDC_2026\[Demo] MSPM0_Car_v5\project\keil"
call build_keil.bat
if %errorlevel% neq 0 (
    echo [ERROR] MSPM0G3507 compilation failed!
    pause
    exit /b 1
)

echo.
echo ============================================================
echo   ALL FIRMWARES BUILT SUCCESSFULLY!
echo ============================================================
echo   1. MSP430 Master:
echo      - %~dp0firmware_MSP430F5529\MSP430_Car_Master.hex
echo      - %~dp0firmware_MSP430F5529\MSP430_Car_Master.txt
echo   2. G3507 Actuator:
echo      - %~dp0NUEDC_2026\[Demo] MSPM0_Car_v5\project\keil\Objects\NUEDC2025_MSPM0G3507.hex
echo ============================================================
echo.
pause
