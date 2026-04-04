#Requires -Version 5.0
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$CMake = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" `
    -latest -requires Microsoft.Component.MSBuild `
    -find "**\CMake\bin\cmake.exe" | Select-Object -First 1
if (-not $CMake) { throw "CMake not found (install VS CMake component)." }
$Ninja = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" `
    -latest -find "**\ninja.exe" | Select-Object -First 1
if ($Ninja) {
    $env:PATH = "$(Split-Path $Ninja);$env:PATH"
}

$ArmBin = $null
if (Get-Command arm-none-eabi-g++ -ErrorAction SilentlyContinue) {
    $ArmBin = Split-Path (Get-Command arm-none-eabi-g++).Source
}
if (-not $ArmBin) {
    $candidates = @(
        "${env:ProgramFiles(x86)}\Arm GNU Toolchain arm-none-eabi\*\bin",
        "${env:ProgramFiles}\Arm GNU Toolchain arm-none-eabi\*\bin"
    )
    foreach ($pat in $candidates) {
        $d = Get-ChildItem -Path $pat -ErrorAction SilentlyContinue |
            Where-Object { Test-Path (Join-Path $_.FullName "arm-none-eabi-g++.exe") } |
            Select-Object -First 1
        if ($d) { $ArmBin = $d.FullName; break }
    }
}
if ($ArmBin) {
    $env:PATH = "$ArmBin;$env:PATH"
}
if (-not (Get-Command arm-none-eabi-g++ -ErrorAction SilentlyContinue)) {
    throw "arm-none-eabi-g++ not found. Install Arm GNU Toolchain and/or add its bin to PATH."
}

$ToolchainFile = (Resolve-Path (Join-Path $PSScriptRoot "cmake\arm-none-eabi-gcc.cmake")).Path
$BuildDir = Join-Path $Root "build-m4"

& $CMake `
    -S $Root `
    -B $BuildDir `
    -G Ninja `
    "-DCMAKE_BUILD_TYPE=Release" `
    "-DCMAKE_TOOLCHAIN_FILE=$ToolchainFile"

if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $CMake --build $BuildDir --parallel
exit $LASTEXITCODE
