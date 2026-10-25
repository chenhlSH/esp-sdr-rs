# C3 rtl_tcp bridge (fast path): C# hot loop (serial read + frame parse + IQ8 pack
# + TCP forward), PowerShell glue only. Capburst frames on COM -> rtl_tcp for SDR++.
# Usage: powershell -ExecutionPolicy Bypass -File tools\rtl_tcp_bridge.ps1 <COM> [port=1234]
$ErrorActionPreference = "Stop"
$com = $args[0]
$listenPort = if ($args.Count -ge 2) { [int]$args[1] } else { 1234 }
if (-not $com) { Write-Host "usage: rtl_tcp_bridge.ps1 <COM> [port]"; exit 2 }

Add-Type -TypeDefinition @"
using System;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;

public static class BridgeCore {
    public static SerialPort OpenSerial(string port) {
        var s = new SerialPort(port, 115200);
        s.ReadTimeout = 100;
        s.ReadBufferSize = 1 << 20;
        s.Open();
        s.DiscardInBuffer();
        return s;
    }

    public static TcpClient AcceptClient(int port) {
        var listener = new TcpListener(IPAddress.Loopback, port);
        listener.Start();
        var client = listener.AcceptTcpClient();
        listener.Stop();
        client.NoDelay = true;
        return client;
    }

    public static void WriteHeader(NetworkStream s) {
        s.Write(new byte[] { 0x52, 0x54, 0x4C, 0x30, 0, 0, 0, 1, 0, 0, 0, 0 }, 0, 12);
        s.Flush();
    }

    // Drains the serial port into the ring. Returns bytes added.
    public static int DrainSerial(SerialPort s, byte[] buf, byte[] ring, ref int ringLen) {
        int total = 0;
        try {
            while (true) {
                int n = s.Read(buf, 0, buf.Length);
                if (n <= 0) break;
                if (ringLen + n > ring.Length) return total; // ring full: drop, resync next pass
                Array.Copy(buf, 0, ring, ringLen, n);
                ringLen += n;
                total += n;
                if (total >= buf.Length * 4) break;
            }
        } catch (TimeoutException) {}
        return total;
    }

    // Parses complete C5IQ frames from ring[0..ringLen), packs IQ8 into out[0..),
    // writes to the stream, compacts the ring. Returns frames forwarded.
    // out must be >= 32760 bytes (one frame of IQ8).
    public static int ForwardFrames(byte[] ring, ref int ringLen, NetworkStream stream, byte[] outBuf) {
        int frames = 0;
        int pos = 0;
        while (true) {
            int remaining = ringLen - pos;
            if (remaining < 4) break;
            int magicAt = -1;
            for (int i = pos; i <= ringLen - 4; i++) {
                if (ring[i] == 0x43 && ring[i+1] == 0x35 && ring[i+2] == 0x49 && ring[i+3] == 0x51) {
                    magicAt = i; break;
                }
            }
            if (magicAt < 0) { pos = ringLen; break; }
            pos = magicAt;
            if (ringLen - pos < 24) break;
            int wordCount = ring[pos + 8] | (ring[pos + 9] << 8);
            int frameLen = 24 + wordCount * 4;
            if (ringLen - pos < frameLen) break;

            int o = 0;
            int p = pos + 24;
            for (int w = 0; w < wordCount; w++) {
                uint u = (uint)(ring[p] | (ring[p+1] << 8) | (ring[p+2] << 16) | (ring[p+3] << 24));
                outBuf[o++] = (byte)((u >> 2) & 0xFF);
                outBuf[o++] = (byte)((u >> 12) & 0xFF);
                p += 4;
            }
            stream.Write(outBuf, 0, wordCount * 2);
            frames++;
            pos += frameLen;
        }
        if (pos > 0) {
            Array.Copy(ring, pos, ring, 0, ringLen - pos);
            ringLen -= pos;
        }
        return frames;
    }
}
"@

$serial = [BridgeCore]::OpenSerial($com)
Write-Host ("bridge: {0} -> 127.0.0.1:{1} (rtl_tcp, C# hot path)" -f $com, $listenPort)

while ($true) {
    Write-Host "waiting for rtl_tcp client..."
    $client = [BridgeCore]::AcceptClient($listenPort)
    $tcp = $client.GetStream()
    Write-Host ("client connected: " + $client.Client.RemoteEndPoint)

    [BridgeCore]::WriteHeader($tcp)

    # consume client commands (sent once at connect)
    $tcp.ReadTimeout = 500
    try {
        $junk = New-Object byte[] 128
        $null = $tcp.Read($junk, 0, $junk.Length)
    } catch [TimeoutException] {}
    $tcp.ReadTimeout = 50

    $serialBuf = New-Object byte[] 65536
    $ring = New-Object byte[] (1 -shl 19)   # 512 KB ring
    $ringLen = 0
    $outBuf = New-Object byte[] 32760
    $frames = 0
    $started = [DateTime]::Now
    $lastReport = $started
    $up = $true

    while ($up) {
        $null = [BridgeCore]::DrainSerial($serial, $serialBuf, $ring, [ref]$ringLen)
        $f = [BridgeCore]::ForwardFrames($ring, [ref]$ringLen, $tcp, $outBuf)
        $frames += $f

        if (([DateTime]::Now - $lastReport).TotalSeconds -ge 5) {
            $rate = $frames / ([DateTime]::Now - $started).TotalSeconds * 8190.0 / 1000.0
            Write-Host ("forwarded {0} frames, {1:N0} kS/s, ring {2}" -f $frames, $rate, $ringLen)
            $lastReport = [DateTime]::Now
        }
        if ($client.Client.Poll(0, [System.Net.Sockets.SelectMode]::SelectRead) -and $client.Available -eq 0) {
            Write-Host "client disconnected"
            break
        }
        Start-Sleep -Milliseconds 1
    }
    $tcp.Close()
    $client.Close()
    Write-Host "session ended"
}
