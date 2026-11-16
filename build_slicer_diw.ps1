# Builds OrcaSlicer itself (release) with a fresh PATH. Log: build_slicer.log
# Extra arguments are passed to build_win.bat, e.g.  build_slicer_diw.ps1 --no-configure
$extra = $args -join " "
$env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
            [Environment]::GetEnvironmentVariable("Path", "User")
Set-Location $PSScriptRoot
$start = Get-Date
cmd /c "`"$PSScriptRoot\build_win.bat`" -s $extra 2>&1" | Tee-Object -FilePath "$PSScriptRoot\build_slicer.log"
"SLICER EXIT CODE: $LASTEXITCODE   elapsed: $((Get-Date) - $start)" | Tee-Object -FilePath "$PSScriptRoot\build_slicer.log" -Append
