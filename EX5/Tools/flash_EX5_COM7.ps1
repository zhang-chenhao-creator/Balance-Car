param(
    [string]$Port = "COM7",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
$Bin = Join-Path $Root "Firmware\EX5_Ultrasonic_Avoid.bin"
$Loader = Join-Path $env:APPDATA "Python\Python310\Scripts\stm32loader.exe"

if (!(Test-Path $Bin)) {
    throw "Firmware BIN not found: $Bin"
}

if (!(Test-Path $Loader)) {
    Write-Host "stm32loader not found. Installing pyserial + stm32loader..."
    python -m pip install --user pyserial stm32loader
}

if (!(Test-Path $Loader)) {
    throw "stm32loader.exe still not found after install: $Loader"
}

Write-Host "Flashing EX5 firmware to $Port ..."
Write-Host "Firmware: $Bin"

& $Loader --port $Port --baud $Baud --family F1 `
    --reset-active-high --boot0-active-low `
    --erase --write --verify --go-address 0x08000000 --no-progress `
    $Bin

if ($LASTEXITCODE -ne 0) {
    throw "Flash failed with exit code $LASTEXITCODE"
}

Write-Host "Flash finished. Look for 'Verification OK' above."
