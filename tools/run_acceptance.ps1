# C4 acceptance: PS bridge (COM -> 127.0.0.1:1234) + Rust rtl_tcp client.
# The bridge speaks C5IQ type 2 (diagstream IQ8 frames) and re-issues the
# 'G' start handshake every 2 s, so the board may boot before or after the
# client connects.
# Usage: powershell -File tools\run_acceptance.ps1 [com=COM4] [seconds=60]
$ErrorActionPreference = "Continue"
$com = if ($args.Count -ge 1) { $args[0] } else { "COM4" }
$seconds = if ($args.Count -ge 2) { [int]$args[1] } else { 60 }
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

Get-Job | Remove-Job -Force -ErrorAction SilentlyContinue

$bridge = Start-Job -ScriptBlock {
    param($root, $com)
    Set-Location $root
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root "tools\rtl_tcp_bridge.ps1") $com 1234
} -ArgumentList $root, $com

Write-Host "bridge up, settling 15 s (Add-Type compile + serial open)..."
# No readiness probe: a probe connect is consumed by the listener and can
# wedge the following real session. The bridge drains+handshakes meanwhile.
Start-Sleep -Seconds 15

$client = Start-Job -ScriptBlock {
    param($root, $seconds)
    Set-Location $root
    & (Join-Path $root "target\release\rtltcp_client.exe") --rtltcp 127.0.0.1 $seconds
} -ArgumentList $root, $seconds

Start-Sleep -Seconds ($seconds + 8)

Write-Host "=== client ==="
Receive-Job $client -Keep
Write-Host "=== bridge ==="
Receive-Job $bridge -Keep
Stop-Job $client, $bridge
Remove-Job $client, $bridge -Force