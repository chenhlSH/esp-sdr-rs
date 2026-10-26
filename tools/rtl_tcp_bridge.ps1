# rtl_tcp bridge (fast path): C# hot loop (serial read + frame parse + IQ8 pack
# + TCP forward), PowerShell glue only.
# Supports C5IQ frame type 1 (capburst: 10-bit IQ words, packed here) and
# type 2 (diagstream: pre-packed IQ8 pairs, forwarded as-is).
# Before a client connects the bridge keeps draining the serial port and
# re-issues the 'G' start handshake every 2 s, so the firmware's blocking
# defmt prints never deadlock the USB-SJ TX endpoint (C2 lesson).
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

    public static void SendStart(SerialPort s) {
        // NOTE: no BaseStream.Flush() here -- FlushFileBuffers on the USB-SJ
        // COM port blocks forever (measured). The 1-byte write goes out with
        // the driver's next transfer.
        try { s.Write("G"); } catch (Exception) {}
    }

    public static TcpListener StartListener(int port) {
        var listener = new TcpListener(IPAddress.Loopback, port);
        listener.Start();
        return listener;
    }

    public static void WriteHeader(NetworkStream s) {
        s.Write(new byte[] { 0x52, 0x54, 0x4C, 0x30, 0, 0, 0, 1, 0, 0, 0, 0 }, 0, 12);
        s.Flush();
    }

    // Drains the serial port into the ring. Returns bytes added.
    // Per-call cap is small (16 KB ~= 100 ms of stream) so the caller keeps
    // forwarding frames to TCP at a steady cadence; a 256 KB cap measured as
    // a 1.6 s stall on the client side.
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
                if (total >= 16384) break;
            }
        } catch (TimeoutException) {}
        return total;
    }

    static uint CrcUpdate(uint crc, byte[] d, int off, int len) {
        for (int i = 0; i < len; i++) {
            crc ^= d[off + i];
            for (int k = 0; k < 8; k++) {
                uint mask = 0u - (crc & 1u);
                crc = (crc >> 1) ^ (0xEDB88320u & mask);
            }
        }
        return crc;
    }

    // Parses complete C5IQ frames from ring[0..ringLen), forwards IQ8 to the
    // stream, compacts the ring. Returns frames forwarded.
    // type 1 (capburst): header + N x u32 10-bit IQ words -> pack to IQ8.
    // type 2 (diagstream): header + N x 2 bytes IQ8 -> forward as-is.
    // outBuf must be >= 32760 bytes.
    public static int ForwardFrames(byte[] ring, ref int ringLen, NetworkStream stream,
                                    byte[] outBuf, ref int crcErrors,
                                    ref long pairs, ref int seqGaps, ref int lastSeq) {
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
            int frameType = ring[pos + 4];
            int seq = ring[pos + 6] | (ring[pos + 7] << 8);
            int wordCount = ring[pos + 8] | (ring[pos + 9] << 8);
            int payloadBytes;
            if (frameType == 2) { payloadBytes = wordCount * 2; }
            else if (frameType == 1) { payloadBytes = wordCount * 4; }
            else { pos += 4; continue; }
            int frameLen = 24 + payloadBytes;
            if (ringLen - pos < frameLen) break;

            // CRC32 over header[0..20) + payload (both frame types).
            uint stored = (uint)(ring[pos+20] | (ring[pos+21] << 8) | (ring[pos+22] << 16) | (ring[pos+23] << 24));
            uint crc = 0xFFFFFFFFu;
            crc = CrcUpdate(crc, ring, pos, 20);
            crc = CrcUpdate(crc, ring, pos + 24, payloadBytes);
            crc ^= 0xFFFFFFFFu;
            if (crc != stored) crcErrors++;

            // Sequence continuity (drop detection): stream type only.
            if (frameType == 2) {
                if (lastSeq >= 0 && ((seq - lastSeq) & 0xFFFF) != 1) seqGaps++;
                lastSeq = seq;
                stream.Write(ring, pos + 24, payloadBytes);
            } else {
                int o = 0;
                int p = pos + 24;
                for (int w = 0; w < wordCount; w++) {
                    uint u = (uint)(ring[p] | (ring[p+1] << 8) | (ring[p+2] << 16) | (ring[p+3] << 24));
                    outBuf[o++] = (byte)((u >> 2) & 0xFF);
                    outBuf[o++] = (byte)((u >> 12) & 0xFF);
                    p += 4;
                }
                stream.Write(outBuf, 0, wordCount * 2);
            }
            pairs += wordCount;
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
$listener = [BridgeCore]::StartListener($listenPort)

# IMPORTANT ordering (measured on the USB-SJ COM port): write the 'G' start
# handshake immediately after opening, BEFORE any reads. A SerialPort.Write
# issued after a large drain deadlocks in usbser.sys (both Write and
# BaseStream.Flush block forever). After that the bridge only reads.
[BridgeCore]::SendStart($serial)

# Keep the board alive before any client connects (see header comment).
$preBuf = New-Object byte[] 65536
$preRing = New-Object byte[] (1 -shl 18)
$preLen = 0
$lastTick = [DateTime]::Now
$drainedTotal = 0L
Write-Host "waiting for rtl_tcp client (draining serial)..."
while (-not $listener.Pending()) {
    $n = [BridgeCore]::DrainSerial($serial, $preBuf, $preRing, [ref]$preLen)
    $drainedTotal += $n
    $preLen = 0  # discard pre-client bytes (defmt logs)
    if (([DateTime]::Now - $lastTick).TotalSeconds -ge 5) {
        Write-Host ("pre-client: drained={0} pending={1}" -f $drainedTotal, $listener.Pending())
        $lastTick = [DateTime]::Now
    }
    Start-Sleep -Milliseconds 20
}

while ($true) {
    $client = $listener.AcceptTcpClient()
    $client.NoDelay = $true
    $tcp = $client.GetStream()
    Write-Host ("client connected: " + $client.Client.RemoteEndPoint)

    $serialBuf = New-Object byte[] 65536
    $ring = New-Object byte[] (1 -shl 19)   # 512 KB ring
    $ringLen = 0
    $outBuf = New-Object byte[] 32760
    $frames = 0
    $crcErrors = 0
    $pairs = 0L
    $seqGaps = 0
    $lastSeq = -1
    $started = [DateTime]::Now

    try {
        [BridgeCore]::WriteHeader($tcp)

        # consume client commands (sent once at connect)
        $tcp.ReadTimeout = 500
        try {
            $junk = New-Object byte[] 128
            $null = $tcp.Read($junk, 0, $junk.Length)
        } catch [TimeoutException] {}
        $tcp.ReadTimeout = 50

        $lastReport = $started
        $up = $true
        while ($up) {
            $null = [BridgeCore]::DrainSerial($serial, $serialBuf, $ring, [ref]$ringLen)
            $frames += [BridgeCore]::ForwardFrames($ring, [ref]$ringLen, $tcp, $outBuf,
                                                   [ref]$crcErrors, [ref]$pairs, [ref]$seqGaps, [ref]$lastSeq)

            if (([DateTime]::Now - $lastReport).TotalSeconds -ge 5) {
                $ksps = $pairs / ([DateTime]::Now - $started).TotalSeconds / 1000.0
                Write-Host ("frames {0}, {1:N1} kS/s, crcErr {2}, seqGaps {3}, ring {4}" -f $frames, $ksps, $crcErrors, $seqGaps, $ringLen)
                $lastReport = [DateTime]::Now
            }
            if ($client.Client.Poll(0, [System.Net.Sockets.SelectMode]::SelectRead) -and $client.Available -eq 0) {
                Write-Host "client disconnected"
                break
            }
            Start-Sleep -Milliseconds 1
        }
    } catch {
        # Probe connects / abrupt client closes land here; keep the loop alive.
        Write-Host ("session error: " + $_.Exception.Message)
    } finally {
        try { $tcp.Close() } catch {}
        try { $client.Close() } catch {}
        Write-Host ("session ended: frames {0}, crcErr {1}, seqGaps {2}, pairs {3}" -f $frames, $crcErrors, $seqGaps, $pairs)
    }
}