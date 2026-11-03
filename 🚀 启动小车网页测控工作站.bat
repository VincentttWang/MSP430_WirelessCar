@echo off
setlocal
cd /d "%~dp0"
title MSP430 Web Ground Station

echo ============================================================
echo   MSP430 and G3507 Car Web Ground Station
echo   Project: MSP430 Wireless Remote Control Car
echo   Author: Wang Jianing (U202413712)
echo ============================================================
echo.

set "SCRIPT=%~dp0tools_pc_remote\web_server.py"

if not exist "%SCRIPT%" (
    echo [ERROR] Web server script not found: %SCRIPT%
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
echo Server Script: %SCRIPT%
echo.
echo [2/2] Launching Web Ground Station at http://localhost:8080 ...
echo.

"%PY_BIN%" "%SCRIPT%"

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Web server exited with code: %errorlevel%
    pause
)
