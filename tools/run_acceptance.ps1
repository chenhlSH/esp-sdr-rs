# C3 acceptance: PS bridge (COM4 -> 127.0.0.1:1234) + Rust client, N seconds.
# Usage: powershell -File tools\run_acceptance.ps1 [seconds=45]
$ErrorActionPreference = "Continue"
$seconds = if ($args.Count -ge 1) { [int]$args[0] } else { 45 }
Set-Location (Split-Path -Parent $PSScriptRoot)

Get-Process rtl_tcp_bridge, rtltcp_client -ErrorAction SilentlyContinue | Stop-Process -Force
Get-Job | Remove-Job -Force -ErrorAction SilentlyContinue

$bridge = Start-Job -ScriptBlock {
    Set-Location "D:\desktop2\rust\esp-sdr-rs"
    powershell -NoProfile -ExecutionPolicy Bypass -File tools\rtl_tcp_bridge.ps1 COM4 1234
}
# 覆盖：serialport 打开可能的复位边沿 + capburst 20 s 握手超时
Write-Host "bridge up, settling 25 s (board reset + handshake)..."
Start-Sleep -Seconds 25

$client = Start-Job -ScriptBlock {
    Set-Location "D:\desktop2\rust\esp-sdr-rs"
    & target\release\rtltcp_client.exe --rtltcp 127.0.0.1 $args[0]
} -ArgumentList $seconds

Start-Sleep -Seconds ($seconds + 10)

Write-Host "=== client ==="
Receive-Job $client -Keep
Write-Host "=== bridge ==="
Receive-Job $bridge -Keep
Stop-Job $client, $bridge
Remove-Job $client, $bridge -Force
