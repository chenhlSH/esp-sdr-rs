# Serial loopback test: open COM4, send 'G', print whatever comes back in 2 s.
$ErrorActionPreference = "Stop"
$port = New-Object System.IO.Ports.SerialPort COM4, 115200
$port.ReadTimeout = 500
$port.Open()
$port.DtrEnable = $true
$port.RtsEnable = $true
Start-Sleep -Milliseconds 200
$port.DiscardInBuffer()
$port.Write("G")
$reply = ""
$deadline = (Get-Date).AddSeconds(2)
while ((Get-Date) -lt $deadline) {
    try {
        $chunk = $port.ReadExisting()
        if ($chunk.Length -gt 0) { $reply += $chunk }
    } catch {}
    Start-Sleep -Milliseconds 50
}
$port.Close()
if ($reply.Length -eq 0) {
    Write-Host "NO REPLY"
} else {
    $bytes = [Text.Encoding]::GetEncoding(28591).GetBytes($reply)
    Write-Host ("REPLY {0} bytes:" -f $bytes.Length)
    ($bytes | Select-Object -First 16 | ForEach-Object { Write-Host ("  0x{0:x2}" -f $_) })
}
