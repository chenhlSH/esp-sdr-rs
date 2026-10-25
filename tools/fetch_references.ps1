# 克隆参考仓库到 reference/（该目录不入库，commit 记录在 REFERENCES.md）
# 用法：powershell -ExecutionPolicy Bypass -File tools/fetch_references.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$refdir = Join-Path $root "reference"

$repos = @(
    @{ Name = "esp-sdr";        Url = "https://github.com/ESPARGOS/esp-sdr.git" },
    @{ Name = "esp-sdr-zephyr"; Url = "https://github.com/cyber-murmel/esp-sdr-zephyr.git" }
)

foreach ($repo in $repos) {
    $dest = Join-Path $refdir $repo.Name
    if (Test-Path $dest) {
        Write-Host "已存在，跳过：$dest"
    } else {
        git clone $repo.Url $dest
    }
}
Write-Host "完成。请把各仓库的 git rev-parse HEAD 结果更新到 REFERENCES.md"
