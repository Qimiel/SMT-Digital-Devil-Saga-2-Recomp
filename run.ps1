param(
    [int]$Seconds = 30,
    [string]$GameDir = '',
    [string]$Log = '',
    [int]$Heartbeat = 3,
    [double]$Snapshot = 0
)
# Launches the native build for a bounded time and captures its console output.
$root = $PSScriptRoot
if (-not $GameDir) {
    # The files build_from_iso.ps1 extracted, else the disc directory this
    # tree lives in.
    $GameDir = Join-Path $root 'dist/game'
    if (-not (Test-Path (Join-Path $GameDir 'SLUS_211.52'))) { $GameDir = Split-Path -Parent (Split-Path -Parent $root) }
}
if (-not $Log) { $Log = Join-Path $root 'run_log.txt' }
$exe = Join-Path $root 'build/dds2.exe'
$err = "$Log.err"
$env:DDS2_HEARTBEAT = "$Heartbeat"
$env:PS2X_GUEST_PRINTF = '1'
if ($Snapshot -gt 0) {
    $env:PS2X_SNAPSHOT_SECONDS = "$Snapshot"
    Remove-Item (Join-Path $root 'build/snapshot_*.png') -ErrorAction SilentlyContinue
}
$p = Start-Process -FilePath $exe -ArgumentList "`"$GameDir`"" -WorkingDirectory (Split-Path $exe) `
    -RedirectStandardOutput $Log -RedirectStandardError $err -PassThru -WindowStyle Normal
if (-not $p.WaitForExit($Seconds * 1000)) {
    Stop-Process -Id $p.Id -Force
    $p.WaitForExit()
    Write-Output "[run.ps1] stopped after $Seconds s"
} else {
    Write-Output "[run.ps1] exited with code $($p.ExitCode)"
}
Remove-Item Env:\DDS2_HEARTBEAT
Remove-Item Env:\PS2X_GUEST_PRINTF
Remove-Item Env:\PS2X_SNAPSHOT_SECONDS -ErrorAction SilentlyContinue
