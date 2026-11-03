@echo off
setlocal
cd /d "%~dp0"
title Flash MSP430F5529 Firmware

set DSLITE=D:\TI\CCS\ccs\ccs_base\DebugServer\bin\DSLite.exe
if not exist "%DSLITE%" set DSLITE=D:\TI\UniFlash\dslite.bat

set CCXML=MSP430F5529.ccxml
set IMG=MSP430_Car_Master.out
set HEX=MSP430_Car_Master.hex

echo ========================================================
echo   MSP430F5529 firmware download
echo ========================================================
echo   image : %IMG%
echo   config: %CCXML%
echo.

if not exist "%IMG%"   goto NO_IMG
if not exist "%CCXML%" goto NO_CCXML
if not exist "%DSLITE%" goto NO_TOOL

echo Flashing firmware to MSP430F5529...
echo.
"%DSLITE%" load -c "%CCXML%" -t 60 -f "%IMG%"
set RC=%ERRORLEVEL%
echo.
if not "%RC%"=="0" goto LOAD_FAIL
echo [ OK ] Firmware successfully flashed and target is running. (rc=0)

echo.
echo ========================================================
echo  WHAT YOU SHOULD SEE NOW  (takes about 3 seconds)
echo   1. all 8 LEDs light steady for 0.8 s
echo   2. all 8 LEDs blink slowly 5 times
echo   3. LED1 (red) stays on
echo   4. LCD backlight blinks 3 times, then a checkerboard
echo   5. green LED blinks 1, 2, 3, 4 times
echo ========================================================
goto DONE

:LOAD_FAIL
echo ========================================================
echo [FAILED] Download failed. (rc=%RC%)
echo.
echo  1. MSP-EXP430F5529 USB plugged into this PC?
echo  2. Close anything holding the eZ-FET COM port
echo     (web station, serial monitor, CCS debug session).
echo  3. Power / RST / TEST jumpers on the board fitted?
echo  4. Unplug USB, wait 3 s, plug back in, run again.
echo ========================================================
goto DONE


:NO_IMG
echo [ERROR] %IMG% not found - run build.bat first.
goto DONE

:NO_CCXML
echo [ERROR] %CCXML% not found.
goto DONE

:NO_TOOL
echo [ERROR] TI DSLite not found: %DSLITE%
goto DONE

:DONE
echo.
pause
endlocal
