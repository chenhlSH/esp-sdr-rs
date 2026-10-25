# Flash then monitor for a bounded time, saving defmt output to a log file.
# Usage: powershell -ExecutionPolicy Bypass -File tools\probe_run.ps1 <elf> [seconds=20]
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$elf = $args[0]
$seconds = if ($args.Count -ge 2) { [int]$args[1] } else { 20 }
if (-not $elf) { throw "usage: probe_run.ps1 <elf> [seconds]" }

$elfPath = Join-Path $root $elf
$flasher = Join-Path $root "tools\espflash\espflash.exe"
$log = Join-Path $root "target\probe_monitor.log"

$ErrorActionPreference = "Continue"
& $flasher flash -S -p COM4 $elfPath 2>&1 | Out-Null
$ErrorActionPreference = "Stop"
if ($LASTEXITCODE -ne 0) { throw "flash failed" }

$job = Start-Job -ScriptBlock {
    param($flasher, $elfPath)
    & $flasher monitor -S -p COM4 --elf $elfPath -L defmt --non-interactive 2>&1
} -ArgumentList $flasher, $elfPath

Start-Sleep -Seconds $seconds
Stop-Job $job
$job | Receive-Job | Set-Content $log -Encoding UTF8
Remove-Job $job -Force
Write-Host "monitor output saved to $log"
