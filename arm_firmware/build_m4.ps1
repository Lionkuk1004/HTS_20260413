# HTS Cortex-M4 펌웨어 크로스 빌드 (GNU Arm Embedded + CMake/Ninja)
# 사용:  PS> cd D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware
#        PS> .\build_m4.ps1
# 산출물: ..\build-m4\arm_firmware\hts_m4_firmware.elf, hts_m4_firmware.bin

$ErrorActionPreference = "Stop"
# arm_firmware -> HTS_LIM (저장소 루트)
$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDir = Join-Path $RepoRoot "build-m4"
$Toolchain = Join-Path $PSScriptRoot "cmake\arm-none-eabi-gcc.cmake"

$cmake = $null
foreach ($c in @(
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
        "${env:ProgramFiles}\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    )) {
    if (Test-Path $c) { $cmake = $c; break }
}
if (-not $cmake) {
    $cmd = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmd) { $cmake = $cmd.Source }
}
if (-not $cmake) {
    Write-Error "cmake.exe not found. Install VS CMake component or add cmake to PATH."
}

$jobs = [Math]::Max(1, [Environment]::ProcessorCount - 1)
Push-Location $RepoRoot
try {
    if (-not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))) {
        # 최상위 CMakeLists는 HTS_LIM 루트 (add_subdirectory arm_firmware)
        & $cmake -B $BuildDir -S $RepoRoot -G Ninja `
            -DCMAKE_BUILD_TYPE=Release `
            -DCMAKE_TOOLCHAIN_FILE=$Toolchain
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    & $cmake --build $BuildDir --config Release -j $jobs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    $bin = Join-Path $BuildDir "arm_firmware\hts_m4_firmware.bin"
    if (-not (Test-Path $bin)) {
        Write-Error "Expected output missing: $bin"
    }
    $i = Get-Item $bin
    Write-Host ""
    Write-Host "=== M4 build OK ===" -ForegroundColor Green
    Write-Host ("BIN: {0} ({1} bytes) @ {2}" -f $i.FullName, $i.Length, $i.LastWriteTime)
}
finally {
    Pop-Location
}
