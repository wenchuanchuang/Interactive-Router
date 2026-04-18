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

        $VersionDirs = Get-ChildItem $Root -Directory | Sort-Object Name -Descending
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
    Write-Host 'Example: $env:KICAD_BIN="C:\Path\To\KiCad\9.0\bin"'
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
& $KiCadPython -m router_app.main @args
