# Fetch the pinned espflash binary into tools/espflash/ and verify SHA-256.
# Usage: powershell -ExecutionPolicy Bypass -File tools/fetch_espflash.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

$asset = @{
    Url = "https://github.com/esp-rs/espflash/releases/download/v4.6.0/espflash-x86_64-pc-windows-msvc.zip"
    Zip = "tools\espflash\espflash-4.6.0-x86_64-pc-windows-msvc.zip"
    # sha256 of the zip, recorded on first fetch
    Sha256 = "b2cb4656b067716fe2b3794cf0604b461b2476d06dee7f481704cb6c8495b11c"
}

$zipdir = Join-Path $root "tools\espflash"
New-Item -ItemType Directory -Force -Path $zipdir | Out-Null

$dest = Join-Path $root $asset.Zip
if (-not (Test-Path $dest)) {
    Invoke-WebRequest -Uri $asset.Url -OutFile $dest
}
$hash = (Get-FileHash $dest -Algorithm SHA256).Hash.ToLower()
if ($asset.Sha256 -ne "") {
    if ($hash -ne $asset.Sha256) {
        throw "SHA-256 mismatch: $dest expected $($asset.Sha256) got $hash"
    }
    Write-Host "sha256 ok: $dest"
} else {
    Write-Host "first fetch, sha256: $hash"
}

$zip = Join-Path $root $asset.Zip
$outdir = Join-Path $root "tools\espflash"
Expand-Archive -Path $zip -DestinationPath $outdir -Force
Write-Host "extracted to $outdir"
