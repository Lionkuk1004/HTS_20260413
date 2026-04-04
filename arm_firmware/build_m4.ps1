#Requires -Version 5.0
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$CMake = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" `
    -latest -requires Microsoft.Component.MSBuild `
    -find "**\CMake\bin\cmake.exe" | Select-Object -First 1
if (-not $CMake) { throw "CMake not found (install VS CMake component)." }
$Ninja = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" `
    -latest -find "**\ninja.exe" | Select-Object -First 1
if ($Ninja) {
    $env:PATH = "$(Split-Path $Ninja);$env:PATH"
}
if (-not (Get-Command arm-none-eabi-g++ -ErrorAction SilentlyContinue)) {
    throw "arm-none-eabi-g++ not in PATH. Install GNU Arm Embedded Toolchain and add ...\bin to PATH."
}
$BuildDir = Join-Path $Root "build-m4"
& $CMake -S $Root -B $BuildDir -G Ninja `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_TOOLCHAIN_FILE=(Join-Path $PSScriptRoot "cmake\arm-none-eabi-gcc.cmake")
& $CMake --build $BuildDir --parallel
