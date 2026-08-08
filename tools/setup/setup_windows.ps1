#Requires -Version 5.1
<#
.SYNOPSIS
  Idempotent Drifty bootstrap for Windows + MSYS2 UCRT64.

.DESCRIPTION
  Locates MSYS2 at C:\msys64 (or $env:MSYS2_ROOT), installs the required UCRT64
  packages via pacman when missing, and prints verified tool paths/versions.

  Does not rewrite the user PATH permanently. Does not use Chocolatey. Does not
  download or compile raylib from source — raylib comes from the MSYS2 package.

.NOTES
  If MSYS2 itself is absent, this script prints the winget install command and exits
  nonzero rather than driving the installer (installer UX is fragile from automation).
#>
[CmdletBinding()]
param(
    [string]$Msys2Root = $(if ($env:MSYS2_ROOT) { $env:MSYS2_ROOT } else { 'C:\msys64' })
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Write-Info([string]$Message) { Write-Host "[setup] $Message" }
function Write-Err([string]$Message)  { Write-Host "[setup] ERROR: $Message" -ForegroundColor Red }

if ($env:OS -ne 'Windows_NT') {
    Write-Err 'This script requires Windows.'
    exit 1
}

$ucrtBin = Join-Path $Msys2Root 'ucrt64\bin'
$pacman  = Join-Path $Msys2Root 'usr\bin\pacman.exe'
$bash    = Join-Path $Msys2Root 'usr\bin\bash.exe'

if (-not (Test-Path -LiteralPath $bash)) {
    Write-Err "MSYS2 not found at '$Msys2Root'."
    Write-Host ''
    Write-Host 'Install MSYS2, then re-run this script:'
    Write-Host '  winget install -e --id MSYS2.MSYS2 --accept-package-agreements --accept-source-agreements'
    Write-Host ''
    Write-Host 'Expected root: C:\msys64  (override with -Msys2Root or $env:MSYS2_ROOT)'
    exit 1
}

if (-not (Test-Path -LiteralPath $pacman)) {
    Write-Err "pacman.exe missing under '$Msys2Root'. MSYS2 install looks incomplete."
    exit 1
}

$requiredPackages = @(
    'mingw-w64-ucrt-x86_64-gcc',
    'mingw-w64-ucrt-x86_64-raylib',
    'mingw-w64-ucrt-x86_64-pkgconf',
    'mingw-w64-ucrt-x86_64-binutils',
    'mingw-w64-ucrt-x86_64-clang',
    'mingw-w64-ucrt-x86_64-compiler-rt',
    'mingw-w64-ucrt-x86_64-clang-tools-extra',
    'mingw-w64-ucrt-x86_64-cppcheck',
    'mingw-w64-ucrt-x86_64-gcovr',
    'mingw-w64-ucrt-x86_64-python',
    'make',
    # Windows git is not visible from inside MSYS2. Without git here the Makefile stamps
    # every build DRIFTY_BUILD_COMMIT="unknown" / DIRTY="dirty", so failure bundles record
    # no usable provenance, and `make tidy-changed` cannot tell which files changed.
    'git'
)

Write-Info "Using MSYS2 at $Msys2Root"
Write-Info 'Checking / installing required packages (idempotent)...'

# Sync DBs quietly; ignore benign "nothing to do" outcomes from -Sy.
& $bash -lc "pacman -Sy --noconfirm" | Out-Host

$installList = ($requiredPackages -join ' ')
& $bash -lc "pacman -S --needed --noconfirm $installList" | Out-Host
if ($LASTEXITCODE -ne 0) {
    Write-Err "pacman failed while installing: $installList"
    exit $LASTEXITCODE
}

function Assert-Tool {
    param(
        [Parameter(Mandatory)][string]$Name,
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][scriptblock]$VersionCmd
    )
    if (-not (Test-Path -LiteralPath $Path)) {
        Write-Err "$Name not found at $Path"
        return $false
    }
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '(?i)[\\/]ucrt64[\\/]bin[\\/]' -and $Name -ne 'make') {
        # make lives in /usr/bin; compilers and pkg-config must be UCRT64.
        if ($Name -ne 'make') {
            Write-Err "$Name resolved outside UCRT64: $resolved"
            return $false
        }
    }
    Write-Info "$Name : $resolved"
    & $VersionCmd
    return $true
}

Write-Info 'Verifying toolchain...'
$ok = $true

$gcc = Join-Path $ucrtBin 'gcc.exe'
$ok = (Assert-Tool 'gcc' $gcc { & $gcc --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$makePath = Join-Path $Msys2Root 'usr\bin\make.exe'
$ok = (Assert-Tool 'make' $makePath { & $makePath --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$pkgConfig = Join-Path $ucrtBin 'pkg-config.exe'
if (-not (Test-Path -LiteralPath $pkgConfig)) {
    $pkgConfig = Join-Path $ucrtBin 'pkgconf.exe'
}
$ok = (Assert-Tool 'pkg-config' $pkgConfig {
    & $pkgConfig --version | ForEach-Object { Write-Host "         version $_" }
}) -and $ok

$objdump = Join-Path $ucrtBin 'objdump.exe'
$ok = (Assert-Tool 'objdump' $objdump { & $objdump --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$clang = Join-Path $ucrtBin 'clang.exe'
$ok = (Assert-Tool 'clang' $clang { & $clang --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$clangFormat = Join-Path $ucrtBin 'clang-format.exe'
$ok = (Assert-Tool 'clang-format' $clangFormat { & $clangFormat --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$cppcheck = Join-Path $ucrtBin 'cppcheck.exe'
$ok = (Assert-Tool 'cppcheck' $cppcheck { & $cppcheck --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

$gcovr = Join-Path $ucrtBin 'gcovr.exe'
$ok = (Assert-Tool 'gcovr' $gcovr { & $gcovr --version | Select-Object -First 1 | ForEach-Object { Write-Host "         $_" } }) -and $ok

# raylib via pkg-config inside a real UCRT64 environment
$raylibScriptPath = Join-Path $env:TEMP ("drifty_raylib_check_{0}.sh" -f [guid]::NewGuid().ToString('N'))
$raylibScript = @"
pkg-config --exists raylib || { echo "pkg-config cannot find raylib" >&2; exit 2; }
echo "raylib modversion : `$(pkg-config --modversion raylib)"
echo "raylib cflags     : `$(pkg-config --cflags raylib)"
echo "raylib libs       : `$(pkg-config --libs raylib)"
test -f /ucrt64/bin/libraylib.dll || { echo "missing /ucrt64/bin/libraylib.dll" >&2; exit 3; }
test -f /ucrt64/lib/libraylib.dll.a || { echo "missing import library" >&2; exit 4; }
test -f /ucrt64/lib/libraylib.a || { echo "missing static archive" >&2; exit 5; }
echo "raylib dll        : /ucrt64/bin/libraylib.dll"
echo "raylib import lib : /ucrt64/lib/libraylib.dll.a"
echo "raylib static lib : /ucrt64/lib/libraylib.a"
"@
# ASCII, no BOM — bash rejects a UTF-8 BOM as part of the first command token.
[System.IO.File]::WriteAllText($raylibScriptPath, $raylibScript.Replace("`r`n", "`n"))
$msysRepoCheck = (& (Join-Path $Msys2Root 'usr\bin\cygpath.exe') -u $raylibScriptPath).Trim()
$raylibCheck = & (Join-Path $Msys2Root 'usr\bin\env.exe') MSYSTEM=UCRT64 CHERE_INVOKING=1 `
    $bash --login -c "source '$msysRepoCheck'"
$raylibStatus = $LASTEXITCODE
Remove-Item -LiteralPath $raylibScriptPath -Force -ErrorAction SilentlyContinue
if ($raylibStatus -ne 0) {
    Write-Err 'raylib verification failed inside UCRT64.'
    $ok = $false
} else {
    $raylibCheck | ForEach-Object { Write-Host "[setup] $_" }
}

if (-not $ok) {
    Write-Err 'Setup incomplete.'
    exit 1
}

Write-Info 'Setup complete. Build with build.bat from cmd.exe, or ./build.sh from an MSYS2 UCRT64 shell.'
exit 0
