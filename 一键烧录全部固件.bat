@echo off
setlocal
cd /d "%~dp0"
title Firmware Flasher - MSP430 Remote Car

:MENU
cls
echo ============================================================
echo   MSP430 Wireless Remote Car - Firmware Flasher
echo   Author: Wang Jianing (U202413712)
echo ============================================================
echo.
echo   [1] Flash MSP430F5529 Only (TI DSLite)
echo   [2] Flash MSPM0G3507 Only (Keil ARMCLANG)
echo   [3] Flash BOTH Boards (MSP430 + G3507)
echo   [4] Build All then Flash Both
echo   [0] Exit
echo.
echo ============================================================
set /p CHOICE="Enter your choice [1-4, 0]: "

if "%CHOICE%"=="1" goto FLASH_MSP430
if "%CHOICE%"=="2" goto FLASH_G3507
if "%CHOICE%"=="3" goto FLASH_ALL
if "%CHOICE%"=="4" goto BUILD_THEN_FLASH
if "%CHOICE%"=="0" exit /b 0
goto MENU

:FLASH_MSP430
call "%~dp0flash_msp430.bat"
goto MENU

:FLASH_G3507
call "%~dp0flash_g3507.bat"
goto MENU

:FLASH_ALL
echo.
echo ============================================================
echo   [1/2] Flashing MSP430F5529...
echo ============================================================
call "%~dp0flash_msp430.bat"
echo.
echo ============================================================
echo   [2/2] Flashing MSPM0G3507...
echo ============================================================
call "%~dp0flash_g3507.bat"
echo.
echo ============================================================
echo   All flashing tasks completed!
echo ============================================================
pause
goto MENU

:BUILD_THEN_FLASH
call "%~dp0一键编译全部固件.bat"
goto FLASH_ALL
