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

$KiCadPython = Join-Path $KiCadBin "python.exe"
if (-not (Test-Path $KiCadPython)) {
    Write-Host "KiCad Python was not found."
    Write-Host "Set KICAD_BIN to your KiCad bin folder and run this script again."
    Write-Host 'Example: $env:KICAD_BIN="<KiCad bin folder>"'
    exit 1
}

$env:KICAD_PCBNEW_PATH = $KiCadBin
$env:PATH = "$KiCadBin;$env:PATH"

& $KiCadPython -c "import PyQt5" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "KiCad Python does not have PyQt5 installed."
    Write-Host "Install the required packages first:"
    Write-Host "`"$KiCadPython`" -m pip install -r requirements.txt"
    exit 1
}

$RouterCore = Get-ChildItem -Path "build" -Recurse -Filter "router_core*.pyd" -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$CoreSources = Get-ChildItem -Path "cpp_core", "CMakeLists.txt" -Recurse -File -ErrorAction SilentlyContinue
$NeedsBuild = -not $RouterCore
if (-not $NeedsBuild -and $CoreSources) {
    $NewestSource = $CoreSources | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    $NeedsBuild = $NewestSource.LastWriteTime -gt $RouterCore.LastWriteTime
}

if ($NeedsBuild) {
    if ($RouterCore) {
        Write-Host "router_core is older than C++ sources. Rebuilding it now..."
    } else {
        Write-Host "router_core is not built yet. Building it now..."
    }
    powershell -ExecutionPolicy Bypass -File .\build_router_core.ps1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "router_core build failed."
        exit $LASTEXITCODE
    }
    $RouterCore = Get-ChildItem -Path "build" -Recurse -Filter "router_core*.pyd" -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1
}

if (-not $RouterCore) {
    Write-Host "router_core build did not produce a .pyd file."
    Write-Host "Check the build output above, then rerun this script."
    exit 1
}

& $KiCadPython -m router_app.main @args
