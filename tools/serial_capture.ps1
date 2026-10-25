# Continuous serial capture to a file (no buffer overflow: read from open).
# Usage: powershell -File tools\serial_capture.ps1 <seconds> <outfile>
$ErrorActionPreference = "Stop"
$seconds = [int]$args[0]
$outfile = $args[1]

$port = New-Object System.IO.Ports.SerialPort COM4, 115200
$port.ReadTimeout = 200
$port.ReadBufferSize = 1048576
$port.WriteBufferSize = 1048576
$port.Open()
$port.DiscardInBuffer()

# RTS 复位芯片（重跑固件，帧序号从头开始），结束后 DTR/RTS 均为 false
$port.RtsEnable = $true
Start-Sleep -Milliseconds 100
$port.RtsEnable = $false

$ms = New-Object System.IO.MemoryStream
$buffer = New-Object byte[] 65536
$deadline = (Get-Date).AddSeconds($seconds)
while ((Get-Date) -lt $deadline) {
    try {
        $n = $port.Read($buffer, 0, $buffer.Length)
        if ($n -gt 0) { $ms.Write($buffer, 0, $n) }
    } catch [TimeoutException] {}
}
$port.Close()
[System.IO.File]::WriteAllBytes($outfile, $ms.ToArray())
Write-Host ("captured {0} bytes -> {1}" -f $ms.Length, $outfile)
