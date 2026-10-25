# Fetch vendored binary blobs and verify SHA-256.
# Usage: powershell -ExecutionPolicy Bypass -File tools/fetch_vendor_blobs.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

# Source: Espressif official esp-phy-lib repo, pinned commit 20f1db053a0e6cb9f1c09d255c43bf42483041d0
# (2026-09-22 master; identical librftest.a to the commit zephyr's esp-sdr-zephyr pins).
# librftest.a  = RF test entry points (adctrig / force_rx_gain / rftest shell)
# libbttestmode.a = BT test-mode glue; librftest.a's shell references its
#                   ble_rx_start / bt_testmode_init / esp_ble_rx_func / zb_rx.
$blobs = @(
    @{
        Url = "https://raw.githubusercontent.com/espressif/esp-phy-lib/20f1db053a0e6cb9f1c09d255c43bf42483041d0/esp32c5/librftest.a"
        Dest = "vendor\librftest.a"
        Sha256 = "0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba"
    },
    @{
        Url = "https://raw.githubusercontent.com/espressif/esp-phy-lib/20f1db053a0e6cb9f1c09d255c43bf42483041d0/esp32c5/libbttestmode.a"
        Dest = "vendor\libbttestmode.a"
        Sha256 = "a69f6ece10de8d639937be7d64e419f3f6bf0f4bcd0569996f7a63aa643a6c1b"
    }
)

foreach ($blob in $blobs) {
    $dest = Join-Path $root $blob.Dest
    if (-not (Test-Path $dest)) {
        Invoke-WebRequest -Uri $blob.Url -OutFile $dest
    }
    $hash = (Get-FileHash $dest -Algorithm SHA256).Hash.ToLower()
    if ($hash -ne $blob.Sha256) {
        throw "SHA-256 mismatch: $dest expected $($blob.Sha256) got $hash"
    }
    Write-Host "sha256 ok: $dest"
}
