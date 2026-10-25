# Check the dump-bank handover: parse the linker map and verify static memory
# ends below 0x40820000 (SRAM block 1 belongs to the WiFi dump engine).
# Usage: powershell -ExecutionPolicy Bypass -File tools/check_dump_bank.ps1 [map path]
# Default map: target\sdr_map.ld (emitted by -Map in .cargo/config.toml)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$mapPath = if ($args.Count -ge 1) { $args[0] } else { Join-Path $root "target\sdr_map.ld" }

if (-not (Test-Path $mapPath)) {
    throw "map file not found: $mapPath (build a riscv32 firmware target first)"
}

$content = Get-Content $mapPath -Raw

$symbols = @{
    "data_end"    = $null
    "bss_end"     = $null
    "noinit_end"  = $null
    "stack_start" = $null
}
foreach ($name in @("data_end", "bss_end", "noinit_end", "stack_start")) {
    # GNU ld map lines look like:  40818200 40818200 0 1 _bss_end = ABSOLUTE(.)
    $line = Select-String -Path $mapPath -Pattern ("^\s*[0-9a-fA-F]+\s+[0-9a-fA-F]+\s+\S+\s+\S+\s+" + [regex]::Escape("_" + $name) + "\b") | Select-Object -First 1
    if ($line) {
        $firstField = ($line.Line.Trim() -split "\s+")[0]
        $symbols[$name] = [Convert]::ToUInt64($firstField, 16)
    }
}

$bankLower = 0x40820000
$bankUpper = 0x40840000
$failed = $false

foreach ($name in @("data_end", "bss_end", "noinit_end")) {
    $value = $symbols[$name]
    if ($null -eq $value) {
        Write-Host "warning: $name not found in map"
        continue
    }
    $status = "PASS"
    if ($value -gt $bankLower) { $status = "FAIL"; $failed = $true }
    Write-Host ("  {0,-14} = 0x{1:x8}  {2}" -f ("_" + $name), $value, $status)
}

$stackStart = $symbols["stack_start"]
if ($null -ne $stackStart) {
    Write-Host ("  {0,-14} = 0x{1:x8}  (boot stack top anchor, info)" -f "_stack_start", $stackStart)
}

Write-Host ("  dump bank     : 0x{0:x8} .. 0x{1:x8}" -f $bankLower, $bankUpper)

if ($failed) {
    throw "static memory crosses the dump bank! the linker assert should have failed the build; check firmware statics."
}
Write-Host "dump bank handover check PASSED."
