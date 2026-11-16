# Builds the OrcaSlicer dependencies (release) with a fresh PATH so the newly installed
# CMake / Strawberry Perl / VS Build Tools are found. Log: build_deps.log
$env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
            [Environment]::GetEnvironmentVariable("Path", "User")
Set-Location $PSScriptRoot
"cmake: " + (Get-Command cmake).Source
"perl:  " + (Get-Command perl).Source
$start = Get-Date
cmd /c "`"$PSScriptRoot\build_win.bat`" -d 2>&1" | Tee-Object -FilePath "$PSScriptRoot\build_deps.log"
"DEPS EXIT CODE: $LASTEXITCODE   elapsed: $((Get-Date) - $start)" | Tee-Object -FilePath "$PSScriptRoot\build_deps.log" -Append
