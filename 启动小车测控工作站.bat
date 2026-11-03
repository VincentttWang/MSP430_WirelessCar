@echo off
setlocal
cd /d "%~dp0"
title MSP430 and G3507 Car Ground Station

echo ============================================================
echo   MSP430 and G3507 Car Ground Station (PC GUI)
echo   Project: MSP430 Wireless Remote Control Car
echo   Author: Wang Jianing (U202413712)
echo ============================================================
echo.

set "SCRIPT=%~dp0tools_pc_remote\car_remote_gui.py"

if not exist "%SCRIPT%" (
    echo [ERROR] Ground station script not found: %SCRIPT%
    pause
    exit /b 1
)

set "PY_BIN="
if exist "D:\Application\Anaconda\python.exe" (
    set "PY_BIN=D:\Application\Anaconda\python.exe"
) else (
    set "PY_BIN=python"
)

echo [1/2] Checking Python environment...
echo Python: %PY_BIN%
echo Script: %SCRIPT%
echo.
echo [2/2] Launching GUI workstation window...
echo.

"%PY_BIN%" "%SCRIPT%"

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Process exited with code: %errorlevel%
    pause
)
