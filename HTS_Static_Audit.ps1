#requires -Version 5.1
<#
.SYNOPSIS
  B-CDMA HTS_LIM 트리에 대해 MSBuild 코드 분석(/analyze)과 Cppcheck를 실행하고, 레이어별 마크다운 리포트를 생성한다.

.PARAMETER RepositoryRoot
  저장소 루트(기본: 이 스크립트가 있는 HTS_LIM 폴더의 상위 한 단계).

.PARAMETER HtsLimSourceDir
  펌웨어 소스 루트(기본: RepositoryRoot\HTS_LIM\HTS_LIM).

.PARAMETER SolutionPath
  MSBuild 솔루션(기본: RepositoryRoot\HTS_LIM\HTS_Development.sln).

.PARAMETER Configuration / Platform
  MSVC 구성(기본 Release / x64).

.PARAMETER SkipMsbuild / SkipCppcheck
  해당 도구 단계를 건너뛴다.

.NOTES
  - Cppcheck는 PATH에 있어야 한다. 없으면 리포트에 미실행으로 기록한다.
  - MSVC 분석은 솔루션 Rebuild가 수반될 수 있다(시간 소요).
  - 동시에 두 도구 결과를 합산하며, 파일명 규칙으로 6개 레이어 그룹에 분류한다.
#>
[CmdletBinding()]
param(
    [string]$RepositoryRoot = "",
    [string]$HtsLimSourceDir = "",
    [string]$SolutionPath = "",
    [string]$Configuration = "Release",
    [string]$Platform = "x64",
    [switch]$SkipMsbuild,
    [switch]$SkipCppcheck
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

function Get-ScriptRoot {
    if ($PSScriptRoot) { return $PSScriptRoot }
    return Split-Path -Parent $MyInvocation.MyCommand.Path
}

$ScriptDir = Get-ScriptRoot
if (-not $RepositoryRoot) {
    $RepositoryRoot = Split-Path -Parent $ScriptDir
}
if (-not $HtsLimSourceDir) {
    $HtsLimSourceDir = Join-Path $ScriptDir "HTS_LIM"
}
if (-not $SolutionPath) {
    $SolutionPath = Join-Path $ScriptDir "HTS_Development.sln"
}

$ReportPath = Join-Path $ScriptDir "HTS_Static_Audit_Report.md"
$LogDir = Join-Path $ScriptDir ".hts_static_audit_logs"
if (-not (Test-Path -LiteralPath $LogDir)) {
    New-Item -ItemType Directory -Path $LogDir -Force | Out-Null
}
$MsbuildLog = Join-Path $LogDir ("msbuild_ca_{0:yyyyMMdd_HHmmss}.log" -f (Get-Date))
$CppcheckRaw = Join-Path $LogDir ("cppcheck_{0:yyyyMMdd_HHmmss}.txt" -f (Get-Date))

function Find-MsBuildExe {
    $candidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe") |
            Where-Object { Test-Path -LiteralPath $_ }
    )
    if ($candidates.Count -lt 1) { return $null }
    $vw = $candidates[0]
    $found = @(
        & $vw -latest -products * -requires Microsoft.Component.MSBuild `
            -find "MSBuild\**\Bin\MSBuild.exe" 2>$null |
            Where-Object { $null -ne $_ -and "$_" -ne '' }
    )
    if ($found.Count -ge 1) { return $found[0] }
    return $null
}

function Sanitize-AuditPathFragment {
    param($Raw)

    if ($null -eq $Raw) { return '' }

    $one = $Raw
    if ($Raw -is [System.Array]) {
        if (@($Raw).Count -lt 1) { return '' }
        $one = $Raw[0]
    }

    $p = [string]$one
    if ([string]::IsNullOrWhiteSpace($p)) { return '' }

    $p = $p.Trim()
    $p = [regex]::Replace($p, '[\x00-\x1F]', '')
    $bad = [char[]]@(
        [char]0x3C, [char]0x3E, [char]0x7C, [char]0x3A,
        [char]0x2A, [char]0x3F, [char]0x22
    )
    foreach ($ch in $bad) {
        $p = $p.Replace([string]$ch, '')
    }
    return $p.Trim("`r`n`t ")
}

function Normalize-RepoPath {
    param($FullPath)
    $FullPath = Sanitize-AuditPathFragment $FullPath
    if ([string]::IsNullOrWhiteSpace([string]$FullPath)) { return "" }
    try {
        $r = Resolve-Path -LiteralPath $FullPath -ErrorAction Stop
        $root = Resolve-Path -LiteralPath $RepositoryRoot -ErrorAction Stop
        $rel = $r.Path.Substring($root.Path.Length).TrimStart('\', '/')
        return $rel -replace '\\', '/'
    }
    catch {
        return ($FullPath -replace '\\', '/')
    }
}

function Get-HtsLayerGroupKey {
    param($RelativePath)
    $safeRel = Sanitize-AuditPathFragment $RelativePath
    if ([string]::IsNullOrWhiteSpace([string]$safeRel)) {
        return '기타: Common/Unknown'
    }
    $pathForName = [string]$safeRel.Trim()
    $name = [System.IO.Path]::GetFileName($pathForName)
    $lower = ([string]$name).ToLowerInvariant()
    $relLower = ([string]$safeRel).ToLowerInvariant().Replace('\', '/')

    # 그룹 6 (Layer 15~17) — 구체 키워드 우선
    $g6 = @(
        'HTS_OTA_', 'HTS_AMI_', 'HTS_Meter_', 'HTS_Modbus_', 'HTS_BLE_', 'HTS_IoT_',
        'HTS_Sensor_', 'HTS_CCTV_', 'HTS_Emergency_', 'HTS_Location_', 'HTS_Gyro_',
        'HTS_Priority_Scheduler', 'HTS_Unified_Scheduler', 'HTS_Console_Manager',
        'HTS_Universal_API', 'HTS_API.', 'AMI_Protocol', 'OTA_Manager'
    )
    foreach ($p in $g6) {
        if ($name -like "$p*" -or $name -like "*$p*") {
            return '그룹 6: Layer 15~17 (OTA·AMI·센서·스케줄러·최상위 API)'
        }
    }

    $g5 = @(
        'HTS_AEAD_', 'HTS_Security_Session', 'HTS_Security_Pipeline', 'HTS_Session_Gateway',
        'HTS_Remote_Attestation', 'HTS_Role_Auth', 'HTS_Storage_', 'HTS_IPC_',
        'HTS_Network_', 'HTS_KT_DSN', 'HTS_CoAP_', 'HTS_Mesh_', 'HTS_Neighbor_',
        'HTS_Universal_Adapter', 'HTS_AP_Bridge'
    )
    foreach ($p in $g5) {
        if ($name -like "$p*" -or $name -like "*$p*") { return '그룹 5: Layer 12~14 (세션·스토리지·프로토콜)' }
    }
    if ($relLower -match '/host_aarch64/') { return '그룹 5: Layer 12~14 (세션·스토리지·프로토콜)' }

    $g4 = @(
        'HTS_Holo_Tensor', 'HTS_Dual_Tensor', 'HTS_3D_Tensor', 'HTS_FEC_', 'HTS_Tx_Scheduler',
        'BB1_Core', 'HTS64_Native_ECCM', 'HTS_Holo_Dispatcher', 'HTS_V400_Dispatcher'
    )
    foreach ($p in $g4) {
        if ($name -like "$p*" -or $name -like "*$p*") { return '그룹 4: Layer 9~11 (텐서·FEC·디스패처)' }
    }

    $g3 = @(
        'HTS_Anti_Debug', 'HTS_Anti_Glitch', 'HTS_Tamper_', 'HTS_AntiAnalysis', 'HTS_Polymorphic_',
        'HTS_Secure_Logger', 'HTS_Config', 'HTS_Dynamic_Config', 'HTS_Device_', 'HTS_Creator_',
        'HTS_Gaussian_', 'HTS_Rx_Matched', 'HTS_Rx_Sync', 'HTS_Antipodal', 'HTS_Adaptive_BPS',
        'HTS_Orbital_', 'HTS_Sparse_Recovery', 'HTS_Quantum_Decoy', 'HTS_Voice_'
    )
    foreach ($p in $g3) {
        if ($name -like "$p*" -or $name -like "*$p*") { return '그룹 3: Layer 6~8 (물리 보안·로깅·DSP/PHY)' }
    }

    if ($lower -match '^(lea|lsh|hmac|kisa_)' -or $lower -match '^aria|^aes|^sha') {
        return '그룹 2: Layer 3~5 (KCMVP·KAT·키·부팅)'
    }
    $g2 = @(
        'HTS_Key_', 'HTS_Secure_Boot',
        'HTS_Anchor_', 'HTS_Entropy_', 'HTS_CTR_DRBG', 'HTS_TRNG', 'HTS_Crypto_KAT',
        'HTS_Conditional_SelfTest', 'HTS_LEA_', 'HTS_LSH', 'HTS_ARIA_', 'Dynamic_Key_Rotator'
    )
    foreach ($p in $g2) {
        if ($name -like "*$p*") { return '그룹 2: Layer 3~5 (KCMVP·KAT·키·부팅)' }
    }

    $g1 = @(
        'HTS_Types', 'HTS_BitOps', 'HTS_PHY_Config', 'HTS_Hardware_', 'HTS_Power_', 'HTS_POST_',
        'HTS_Secure_Memory', 'HTS_ConstantTime', 'HTS_Crc32', 'common.h', 'config.h', 'util.h',
        'arm_arch', 'HTS_RS_GF16'
    )
    foreach ($p in $g1) {
        if ($name -like "*$p*") { return '그룹 1: Layer 0~2 (플랫폼·HW·메모리 코어)' }
    }

    if ($relLower -match '/hts_test/|/verify_|/benchmark/') {
        return '기타: Common/Unknown (테스트·검증·벤치)'
    }
    return '기타: Common/Unknown'
}

function Map-Severity {
    param([string]$Tool, [string]$Level, [string]$Code)
    $u = ($Level + ' ' + $Code).ToUpperInvariant()
    if ($u -match '\bERROR\b|C00\d{4}') { return '높음' }
    if ($u -match '\bWARNING\b|STYLE') { return '중간' }
    return '낮음'
}

function Add-Finding {
    param(
        [string]$FilePath,
        [int]$Line,
        [string]$DefectType,
        [string]$Severity,
        [string]$Tool,
        [System.Collections.Generic.List[object]]$List
    )
    $FilePath = Sanitize-AuditPathFragment $FilePath
    if ([string]::IsNullOrWhiteSpace([string]$FilePath)) { return }
    $rel = Normalize-RepoPath $FilePath
    if (-not $rel) {
        $rel = ($FilePath -replace '\\', '/')
    }
    $grp = Get-HtsLayerGroupKey $rel
    $List.Add([pscustomobject]@{
            Group       = $grp
            File        = $rel
            Line        = $Line
            DefectType  = $DefectType
            Severity    = $Severity
            Tool        = $Tool
        })
}

$findings = New-Object 'System.Collections.Generic.List[object]'

# --- MSBuild + RunCodeAnalysis ---
if (-not $SkipMsbuild) {
    if (-not (Test-Path -LiteralPath $SolutionPath)) {
        Write-Warning "솔루션을 찾을 수 없음: $SolutionPath (MSBuild 단계 생략)"
    }
    else {
        $msb = Find-MsBuildExe
        if (-not $msb) {
            Write-Warning "MSBuild를 찾을 수 없음(vswhere). MSBuild 단계 생략."
        }
        else {
            Write-Host "MSBuild 코드 분석 실행: $SolutionPath"
            $logOpt = "logfile=`"$MsbuildLog`";verbosity=normal"
            $msArgs = @(
                $SolutionPath,
                "/t:Rebuild",
                "/p:Configuration=$Configuration",
                "/p:Platform=$Platform",
                "/p:RunCodeAnalysis=true",
                "/p:CodeAnalysisTreatWarningsAsErrors=false",
                "/m",
                "/nologo",
                "/fl",
                "/flp:$logOpt"
            )
            $proc = Start-Process -FilePath $msb -ArgumentList $msArgs -Wait -NoNewWindow -PassThru
            if ($proc.ExitCode -ne 0) {
                Write-Warning "MSBuild 종료 코드: $($proc.ExitCode) (로그는 계속 파싱)"
            }
            if (Test-Path -LiteralPath $MsbuildLog) {
                $rxMs = [regex]'^(?<file>.+?)\((?<line>\d+)\)\s*:\s*(?<kind>warning|error)\s+(?<code>[A-Za-z]?\d+)\s*:\s*(?<msg>.*)$'
                $srcExt = [regex]'\.(c|cpp|cc|cxx|h|hpp|hxx)$'
                Get-Content -LiteralPath $MsbuildLog -ErrorAction SilentlyContinue | ForEach-Object {
                    $m = $rxMs.Match($_)
                    if (-not $m.Success) { return }
                    $fp = Sanitize-AuditPathFragment $m.Groups['file'].Value
                    if ([string]::IsNullOrWhiteSpace([string]$fp)) { return }
                    if (-not $srcExt.IsMatch($fp)) { return }
                    $ln = 0
                    [void][int]::TryParse($m.Groups['line'].Value, [ref]$ln)
                    $code = $m.Groups['code'].Value
                    $msg = $m.Groups['msg'].Value.Trim()
                    $kind = $m.Groups['kind'].Value
                    $sev = Map-Severity -Tool "MSVC" -Level $kind -Code $code
                    $dtype = if ($code) { "MSVC $code : $msg" } else { $msg }
                    Add-Finding -FilePath $fp -Line $ln -DefectType $dtype -Severity $sev -Tool "MSBuild/CA" -List $findings
                }
            }
        }
    }
}

# --- Cppcheck ---
if (-not $SkipCppcheck) {
    $cp = Get-Command cppcheck -ErrorAction SilentlyContinue
    if (-not $cp) {
        Write-Warning "cppcheck 명령을 PATH에서 찾을 수 없음. Cppcheck 단계 생략."
        "" | Out-File -LiteralPath $CppcheckRaw -Encoding utf8
    }
    elseif (-not (Test-Path -LiteralPath $HtsLimSourceDir)) {
        Write-Warning "소스 디렉터리 없음: $HtsLimSourceDir"
    }
    else {
        Write-Host "Cppcheck 실행: $HtsLimSourceDir"
        $inc = $HtsLimSourceDir
        $template = '{file}:{line}:{severity}:{id}:{message}'
        try {
            & cppcheck `
                --enable=all `
                --inconclusive `
                --inline-suppr `
                --suppress=missingIncludeSystem `
                --template=$template `
                -I $inc `
                $HtsLimSourceDir `
                2>&1 | Tee-Object -FilePath $CppcheckRaw
        }
        catch {
            Write-Warning "Cppcheck 실행 예외: $_"
        }
        $rxCp = [regex]'^(?<file>[^:]+):(?<line>\d+):(?<sev>[^:]+):(?<id>[^:]+):(?<msg>.*)$'
        if (Test-Path -LiteralPath $CppcheckRaw) {
            Get-Content -LiteralPath $CppcheckRaw -ErrorAction SilentlyContinue | ForEach-Object {
                $m = $rxCp.Match($_.Trim())
                if (-not $m.Success) { return }
                $fp = Sanitize-AuditPathFragment $m.Groups['file'].Value
                if ([string]::IsNullOrWhiteSpace([string]$fp)) { return }
                $ln = 0
                [void][int]::TryParse($m.Groups['line'].Value, [ref]$ln)
                $id = $m.Groups['id'].Value
                $msg = $m.Groups['msg'].Value.Trim()
                $sevLvl = $m.Groups['sev'].Value
                $sev = Map-Severity -Tool "cppcheck" -Level $sevLvl -Code $id
                $dtype = if ($id) { "cppcheck $id : $msg" } else { $msg }
                Add-Finding -FilePath $fp -Line $ln -DefectType $dtype -Severity $sev -Tool "Cppcheck" -List $findings
            }
        }
    }
}

# --- Markdown report ---
$groups = @(
    '그룹 1: Layer 0~2 (플랫폼·HW·메모리 코어)',
    '그룹 2: Layer 3~5 (KCMVP·KAT·키·부팅)',
    '그룹 3: Layer 6~8 (물리 보안·로깅·DSP/PHY)',
    '그룹 4: Layer 9~11 (텐서·FEC·디스패처)',
    '그룹 5: Layer 12~14 (세션·스토리지·프로토콜)',
    '그룹 6: Layer 15~17 (OTA·AMI·센서·스케줄러·최상위 API)',
    '기타: Common/Unknown',
    '기타: Common/Unknown (테스트·검증·벤치)'
)

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine("# HTS 정적 분석 리포트")
[void]$sb.AppendLine("")
[void]$sb.AppendLine("| 항목 | 값 |")
[void]$sb.AppendLine("|------|-----|")
[void]$sb.AppendLine("| 생성 시각 (로컬) | $((Get-Date).ToString('yyyy-MM-dd HH:mm:ss')) |")
[void]$sb.AppendLine("| 저장소 루트 | ``$($RepositoryRoot -replace '\\','/')`` |")
[void]$sb.AppendLine("| MSVC 솔루션 | ``$($SolutionPath -replace '\\','/')`` |")
[void]$sb.AppendLine("| 구성 | ``$Configuration`` / ``$Platform`` |")
[void]$sb.AppendLine("| Cppcheck 소스 | ``$($HtsLimSourceDir -replace '\\','/')`` |")
[void]$sb.AppendLine("| MSBuild 로그 | ``$($MsbuildLog -replace '\\','/')`` |")
[void]$sb.AppendLine("| Cppcheck 원본 | ``$($CppcheckRaw -replace '\\','/')`` |")
$findingCount = @($findings).Count
[void]$sb.AppendLine("| 총 적발 건수 | $findingCount |")
[void]$sb.AppendLine("")
[void]$sb.AppendLine("MSBuild는 ``/p:RunCodeAnalysis=true`` 로 실행한다. Cppcheck는 ``--enable=all --inconclusive`` 를 사용한다.")
[void]$sb.AppendLine("파일 경로·이름 규칙으로 레이어 그룹을 나누며, 교차 모듈 검증은 도구 설정에 따른다.")
[void]$sb.AppendLine("")

$grouped = @($findings | Group-Object -Property Group)

foreach ($gname in $groups) {
    $items = @(
        $grouped |
            Where-Object { $_.Name -eq $gname } |
            ForEach-Object { $_.Group }
    )
    if ($items.Count -lt 1) { continue }

    [void]$sb.AppendLine("## $gname")
    [void]$sb.AppendLine("")
    [void]$sb.AppendLine("| 파일명(상대 경로) | 라인 | 결함 종류 | 위험도 | 도구 |")
    [void]$sb.AppendLine("|------------------|------|-----------|--------|------|")
    foreach ($it in ($items | Sort-Object File, Line, Tool)) {
        $f = $it.File -replace '\|', '\|'
        $d = ($it.DefectType -replace '\|', '/' -replace "`r`n", ' ')
        if ($d.Length -gt 240) { $d = $d.Substring(0, 237) + "..." }
        [void]$sb.AppendLine("| ``$f`` | $($it.Line) | $d | $($it.Severity) | $($it.Tool) |")
    }
    [void]$sb.AppendLine("")
}

# 그룹 정의에 없는 잔여 키
$ungrouped = @($grouped | Where-Object { $groups -notcontains $_.Name })
if (@($ungrouped).Count -gt 0) {
    [void]$sb.AppendLine("## 기타 분류")
    [void]$sb.AppendLine("")
    foreach ($ug in $ungrouped) {
        [void]$sb.AppendLine("### $($ug.Name)")
        [void]$sb.AppendLine("")
        [void]$sb.AppendLine("| 파일명(상대 경로) | 라인 | 결함 종류 | 위험도 | 도구 |")
        [void]$sb.AppendLine("|------------------|------|-----------|--------|------|")
        foreach ($it in ($ug.Group | Sort-Object File, Line, Tool)) {
            $f = $it.File -replace '\|', '\|'
            $d = ($it.DefectType -replace '\|', '/' -replace "`r`n", ' ')
            if ($d.Length -gt 240) { $d = $d.Substring(0, 237) + "..." }
            [void]$sb.AppendLine("| ``$f`` | $($it.Line) | $d | $($it.Severity) | $($it.Tool) |")
        }
        [void]$sb.AppendLine("")
    }
}

[System.IO.File]::WriteAllText($ReportPath, $sb.ToString(), [System.Text.UTF8Encoding]::new($false))
Write-Host "리포트 작성: $ReportPath"
