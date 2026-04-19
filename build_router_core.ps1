$ErrorActionPreference = "Stop"

$KiCadBin = $env:KICAD_BIN
if (-not $KiCadBin) {
    $CandidateRoots = @()
    if ($env:LOCALAPPDATA) {
        $CandidateRoots += Join-Path $env:LOCALAPPDATA "Programs\KiCad"
    }
    if ($env:ProgramFiles) {
        $CandidateRoots += Join-Path $env:ProgramFiles "KiCad"
    }
    if (${env:ProgramFiles(x86)}) {
        $CandidateRoots += Join-Path ${env:ProgramFiles(x86)} "KiCad"
    }

    foreach ($Root in $CandidateRoots) {
        if (-not $Root -or -not (Test-Path $Root)) {
            continue
        }
        try {
            $VersionDirs = Get-ChildItem $Root -Directory -ErrorAction Stop | Sort-Object Name -Descending
        } catch {
            continue
        }
        foreach ($VersionDir in $VersionDirs) {
            $CandidateBin = Join-Path $VersionDir.FullName "bin"
            $CandidatePython = Join-Path $CandidateBin "python.exe"
            if (Test-Path $CandidatePython) {
                $KiCadBin = $CandidateBin
                break
            }
        }
        if ($KiCadBin) {
            break
        }
    }
}

$Python = Join-Path $KiCadBin "python.exe"
if (-not (Test-Path $Python)) {
    Write-Host "KiCad Python was not found. Set KICAD_BIN to your KiCad bin folder."
    exit 1
}

$BuildPython = $env:ROUTER_BUILD_PYTHON
if (-not $BuildPython) {
    $BuildPython = $Python
}
if (-not (Test-Path $BuildPython)) {
    Write-Host "Build Python was not found: $BuildPython"
    Write-Host "Set ROUTER_BUILD_PYTHON to a Python 3.11 python.exe with development files."
    exit 1
}

$BuildPythonDir = Split-Path $BuildPython -Parent
$PythonHeader = Join-Path $BuildPythonDir "include\Python.h"
$PythonLib = Join-Path $BuildPythonDir "libs\python311.lib"
if (-not (Test-Path $PythonHeader) -or -not (Test-Path $PythonLib)) {
    Write-Host "The selected build Python does not include Python development files."
    Write-Host "Missing one or both:"
    Write-Host "  $PythonHeader"
    Write-Host "  $PythonLib"
    Write-Host ""
    Write-Host "Install official Python 3.11 x64, then run:"
    Write-Host '$env:ROUTER_BUILD_PYTHON="<Python 3.11 python.exe>"'
    Write-Host 'powershell -ExecutionPolicy Bypass -File .\run_with_kicad_python.ps1'
    exit 1
}

$CMake = $env:CMAKE_EXE
if (-not $CMake) {
    $CMake = "cmake"
}
if (-not (Get-Command $CMake -ErrorAction SilentlyContinue) -and -not (Test-Path $CMake)) {
    Write-Host "CMake was not found. Install CMake or add it to PATH."
    Write-Host 'You can also set CMAKE_EXE to a cmake executable.'
    exit 1
}

& $BuildPython -c "import pybind11" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build Python does not have pybind11 installed."
    Write-Host "Install the required build package first:"
    Write-Host "`"$BuildPython`" -m pip install pybind11"
    exit 1
}
$PybindCMakeDir = & $BuildPython -m pybind11 --cmakedir

& $CMake -S . -B build -Dpybind11_DIR="$PybindCMakeDir" -DPython_EXECUTABLE="$BuildPython"
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake configure failed."
    exit $LASTEXITCODE
}
& $CMake --build build --config Release
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake build failed."
    exit $LASTEXITCODE
}

Write-Host "router_core build finished. The GUI will search ./build and ./build/Release automatically."
