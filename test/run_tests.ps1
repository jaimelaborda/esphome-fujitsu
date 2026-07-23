# Builds and runs the Fujitsu protocol unit tests with MSVC (Visual Studio 2022).
#
#   powershell -ExecutionPolicy Bypass -File test\run_tests.ps1
#
# Requires Visual Studio 2022 with the "Desktop development with C++" workload.
# No external test framework is needed.

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot            # repo root (esphome-fujitsu)
$test = Join-Path $root "test"
$build = Join-Path $test "build"
New-Item -ItemType Directory -Force -Path $build | Out-Null

# Locate vcvars64.bat (try common editions, then vswhere).
$vcvars = $null
$candidates = @(
  "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
)
foreach ($c in $candidates) { if (Test-Path $c) { $vcvars = $c; break } }
if (-not $vcvars) {
  $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
  if (Test-Path $vswhere) {
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($vsPath) { $vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat" }
  }
}
if (-not $vcvars -or -not (Test-Path $vcvars)) {
  throw "Could not find vcvars64.bat. Install Visual Studio 2022 with the C++ workload."
}

$src = @(
  (Join-Path $test "test_protocol.cpp"),
  (Join-Path $root "components\fujitsu\fujitsu_protocol.cpp")
) -join '" "'
$exe = Join-Path $build "fujitsu_tests.exe"

$cmd = "`"$vcvars`" && cl /nologo /std:c++17 /EHsc /W3 `"$src`" /Fe:`"$exe`" /Fo:`"$build\\`" && `"$exe`""
cmd /c $cmd
exit $LASTEXITCODE
