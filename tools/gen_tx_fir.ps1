# Generate the polyphase FIR tables for the paced TX path (shared/src/tx_fir_*.rs).
# Design matches zephyr esp_sdr_paced.c fir_design():
#   Blackman-windowed sinc low-pass at the input Nyquist frequency,
#   split into L phases, Q15 coefficients: coef[ph][k] = round(h[ph + k*L] * L / sum * 32767).
# Interpolation factors: 2, 4, 8 (8 taps each). L=1 is the identity (no table).
# Usage: powershell -ExecutionPolicy Bypass -File tools/gen_tx_fir.ps1
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$taps = 8

function Sinc([double]$x) {
    if ([math]::Abs($x) -lt 1e-12) { return 1.0 }
    $p = [math]::PI * $x
    return [math]::Sin($p) / $p
}

function Blackman([int]$k, [int]$n) {
    if ($n -le 0) { return 1.0 }
    $w = 2.0 * [math]::PI * $k / $n
    return 0.42 - 0.5 * [math]::Cos($w) + 0.08 * [math]::Cos(2.0 * $w)
}

foreach ($l in @(2, 4, 8)) {
    $n = $l * $taps
    $h = New-Object double[] $n
    $sum = 0.0
    for ($k = 0; $k -lt $n; $k++) {
        $x = (($k - ($n - 1) / 2.0) / $l)
        $h[$k] = (Sinc $x) * (Blackman $k ($n - 1))
        $sum += $h[$k]
    }
    $lines = @()
    for ($ph = 0; $ph -lt $l; $ph++) {
        $row = @()
        for ($k = 0; $k -lt $taps; $k++) {
            $v = [int][math]::Round($h[$ph + $k * $l] * $l / $sum * 32767.0)
            $row += $v.ToString()
        }
        $lines += ('    [' + ($row -join ', ') + '],')
    }
    $text = '[' + [char]10 + ($lines -join [char]10) + [char]10 + ']' + [char]10
    $path = Join-Path $root ("shared\src\tx_fir_{0}.rs" -f $l)
    [System.IO.File]::WriteAllText($path, $text, (New-Object System.Text.ASCIIEncoding))
    Write-Host ("wrote {0} ({1} phases x {2} taps)" -f $path, $l, $taps)
}