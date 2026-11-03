# MSP430F5529 PowerShell Flash Script
$dslite = "D:\TI\CCS\ccs\ccs_base\DebugServer\bin\DSLite.exe"
$ccxml = "$PSScriptRoot\MSP430F5529.ccxml"
$targetOut = "$PSScriptRoot\MSP430_Car_Master.out"

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "  Flashing MSP430F5529 Firmware (TI DSLite)..." -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "Target: $targetOut"

& $dslite load -c $ccxml -f $targetOut

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] MSP430F5529 flashed successfully!" -ForegroundColor Green
    Write-Host "Screen should display '* MSP430 MASTER *' now." -ForegroundColor Green
} else {
    Write-Host "`n[FAILED] Flash failed. Please verify board USB connection." -ForegroundColor Red
}
