@echo off
cd /d "%~dp0"
"D:\Application\Keil5\UV4\UV4.exe" -b NUEDC2025_MSPM0G3507.uvprojx -t NUEDC2025_MSPM0G3507 -o build.log
type build.log
