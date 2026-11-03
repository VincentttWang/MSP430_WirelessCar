@echo off
setlocal
cd /d "%~dp0firmware_MSP430F5529"
title Flash MSP430F5529 Firmware

call flash_msp430.bat

cd /d "%~dp0"
