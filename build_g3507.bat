@echo off
setlocal
cd /d "%~dp0NUEDC_2026\[Demo] MSPM0_Car_v5\project\keil"
title Build MSPM0G3507 Firmware
call build_keil.bat
cd /d "%~dp0"
pause
