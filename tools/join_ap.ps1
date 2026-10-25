# Join the board's open AP "esp-sdr" and wait for a 192.168.2.x lease.
# Usage: powershell -File tools\join_ap.ps1
$ErrorActionPreference = "Stop"

$ssid = "esp-sdr"
$xml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>$ssid</name>
  <SSIDConfig>
    <SSID><name>$ssid</name></SSID>
  </SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption>
  </security></MSM>
</WLANProfile>
"@
$xmlPath = Join-Path $env:TEMP "esp-sdr-profile.xml"
[System.IO.File]::WriteAllText($xmlPath, $xml)

# 用接口名定位（本机 WLAN 适配器名固定为 "WLAN"）
netsh wlan add profile filename="$xmlPath" interface="WLAN" 2>&1 | Out-Null
netsh wlan connect name=$ssid interface="WLAN" | Out-Null

$deadline = (Get-Date).AddSeconds(30)
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 2
    $ip = (Get-NetIPAddress -InterfaceAlias "WLAN" -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -like "192.168.2.*" } |
        Select-Object -First 1).IPAddress
    if ($ip) {
        Write-Host "joined esp-sdr, ip = $ip"
        exit 0
    }
}
Write-Host "FAILED to obtain 192.168.2.x lease within 30 s"
exit 1
