@echo off
setlocal
cd /d "%~dp0firmware_MSP430F5529"
title Build MSP430F5529 Firmware
call build.bat
cd /d "%~dp0"
pause
