@echo off
setlocal
cd /d "%~dp0"
set CCS_COMPILER="D:\TI\CCS\ccs\tools\compiler\ti-cgt-msp430_21.6.3.LTS\bin\cl430.exe"
set CCS_HEX="D:\TI\CCS\ccs\tools\compiler\ti-cgt-msp430_21.6.3.LTS\bin\hex430.exe"
set INC_DIR1="D:\TI\CCS\ccs\tools\compiler\ti-cgt-msp430_21.6.3.LTS\include"
set INC_DIR2="D:\TI\CCS\ccs\ccs_base\msp430\include"
set LIB_DIR1="D:\TI\CCS\ccs\tools\compiler\ti-cgt-msp430_21.6.3.LTS\lib"
set LIB_DIR2="D:\TI\CCS\ccs\ccs_base\msp430\lib"
set CMD_FILE="D:\TI\CCS\ccs\ccs_base\msp430\include\lnk_msp430f5529.cmd"

echo [1/3] Compiling C source files...
%CCS_COMPILER% --c99 --silicon_version=mspx --define=__MSP430F5529__ -I%INC_DIR1% -I%INC_DIR2% -I. -c main.c system_pre_init.c HAL_Board.c HAL_Dogs102x6.c HAL_UCS.c HAL_PMM.c --obj_directory=.
if errorlevel 1 goto BUILD_FAIL

echo [2/3] Linking firmware...
%CCS_COMPILER% -z --rom_model --stack_size=512 -e _c_int00 main.obj system_pre_init.obj HAL_Board.obj HAL_Dogs102x6.obj HAL_UCS.obj HAL_PMM.obj -i%LIB_DIR1% -i%INC_DIR2% -i%LIB_DIR2% -l%CMD_FILE% -llibc.a -mMSP430_Car_Master.map -oMSP430_Car_Master.out
if errorlevel 1 goto BUILD_FAIL

echo [3/3] Generating HEX and TXT formats...
%CCS_HEX% --ti_txt MSP430_Car_Master.out -o MSP430_Car_Master.txt
if errorlevel 1 goto BUILD_FAIL
rem NOTE: --memwidth=8 --romwidth=8 is mandatory.
rem MSP430 is a 16-bit core; by default hex430 emits one byte per record with
rem a 2-step address, which drops every high byte. The reset vector 0x5C9E
rem then becomes 0xFF9E and the chip never starts (black screen, no UART).
rem Any .hex built without these flags is corrupt - use .out or .txt instead.
%CCS_HEX% --intel --memwidth=8 --romwidth=8 MSP430_Car_Master.out -o MSP430_Car_Master.hex
if errorlevel 1 goto BUILD_FAIL

echo ===================================================
echo MSP430 Firmware Build Succeeded!
echo Output files: MSP430_Car_Master.out / .hex / .txt
echo ===================================================
goto END

:BUILD_FAIL
echo.
echo *** BUILD FAILED ***
exit /b 1

:END
endlocal
