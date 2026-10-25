# Fetch vendored binary blobs (librftest.a) and verify SHA-256.
# Usage: powershell -ExecutionPolicy Bypass -File tools/fetch_vendor_blobs.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

# Source: Espressif official esp-phy-lib repo, pinned commit
# (same esp-phy-lib commit as the libphy used by esp-sdr-zephyr).
$blobs = @(
    @{
        Url = "https://github.com/espressif/esp-phy-lib/raw/e37ecdc9c1e93fefda5b1a99f09bd979046d0720/esp32c5/librftest.a"
        Dest = "vendor/librftest.a"
        Sha256 = "0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba"
    }
)

foreach ($blob in $blobs) {
    $dest = Join-Path $root $blob.Dest
    if (Test-Path $dest) {
        Write-Host "exists, skip: $dest"
    } else {
        Invoke-WebRequest -Uri $blob.Url -OutFile $dest
    }
    $hash = (Get-FileHash $dest -Algorithm SHA256).Hash.ToLower()
    if ($hash -ne $blob.Sha256) {
        throw "SHA-256 mismatch: $dest expected $($blob.Sha256) got $hash"
    }
    Write-Host "sha256 ok: $dest"
}
