#requires -Version 7.0
[CmdletBinding()]
param([string]$OutputName = "review_tiny_gui_lap_bf0498a")
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repo "tmp/tiny_gui_lap_20260907"
$execution = "bf0498aea8761dac74e04d764907519d30843056"
$attempt = "gui_lap_bf0498a_01"
if ($OutputName -notmatch '^[a-zA-Z0-9_-]+$') { throw "Invalid name" }
$packet = Join-Path $taskRoot $OutputName
$zip = "$packet.zip"
if ((Test-Path $packet) -or (Test-Path $zip)) { throw "Refusing overwrite" }
if (@(& git -C $repo status --porcelain).Count) { throw "Require clean committed source" }
$head = (& git -C $repo rev-parse HEAD).Trim()
& git -C $repo diff --quiet $execution $head -- Makefile src tests integrations ros2_ws tools/run_tiny_lidar_dev.py tools/tiny_dev_runner.py tools/tiny_gui_integration.py
if ($LASTEXITCODE -ne 0) { throw "Runtime changed since execution" }
$tar = Join-Path $taskRoot "source_bf0498a.tar"
if ((Get-FileHash $tar).Hash.ToLowerInvariant() -ne "faf52fc8513e72dbeef3b82cf6f4ee26fbeaabbe1d860eb667d08638a941ea80") { throw "Source hash mismatch" }
$saved = Get-Content (Join-Path $taskRoot "saved_check_final/saved_evidence.json") -Raw | ConvertFrom-Json
foreach ($record in $saved.input_sha256.PSObject.Properties) {
    if ((Get-FileHash (Join-Path $taskRoot "$attempt/$($record.Name)")).Hash.ToLowerInvariant() -ne $record.Value) { throw "Evidence changed" }
}
New-Item -ItemType Directory $packet | Out-Null
function Copy-Review([string]$Source, [string]$Relative) {
    $target = Join-Path $packet $Relative
    New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $target
}
Copy-Review $tar "source/source_bf0498a.tar"
Copy-Review (Join-Path $repo "docs/tiny_gui_lap_20260907_results.md") "README_REVIEW.md"
Copy-Review (Join-Path $repo "docs/tiny_gui_lap_20260907.md") "execution_plan.md"
foreach ($relative in @("tools/summarize_tiny_gui_lap.py", "tools/build_tiny_gui_lap_review_packet.ps1", "tests/test_tiny_gui_integration.py", "tests/test_tiny_lidar_sim.py")) {
    Copy-Review (Join-Path $repo $relative) "source/$relative"
}
foreach ($folder in @("evidence", "saved_check_final", $attempt)) {
    $sourceRoot = Join-Path $taskRoot $folder
    foreach ($file in Get-ChildItem -LiteralPath $sourceRoot -Recurse -File) {
        $relative = [IO.Path]::GetRelativePath($sourceRoot,$file.FullName).Replace('\','/')
        if ($folder -eq $attempt -and ($relative.StartsWith("playerconfig/") -or $relative.StartsWith("rviz_config/"))) { continue }
        if ($file.Extension -notin @(".txt", ".json", ".jsonl", ".xml", ".log", ".png", ".xwd")) { throw "Unexpected file $relative" }
        Copy-Review $file.FullName "$folder/$relative"
    }
}
$manifest = [ordered]@{schema="TINY_GUI_LAP_REVIEW_V1"; execution_commit=$execution; package_commit=$head
    official_commit="1f54dff995d02625566341f9e1be1c39369224f2"
    weight_sha256="7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963"
    weights_included=$false; raw_sensor_included=$false; auto_push=$false
    excluded_preserved_in="original attempt tar: playerconfig and rviz_config"; files=@()}
foreach ($file in Get-ChildItem $packet -Recurse -File | Sort-Object FullName) {
    $manifest.files += @{path=[IO.Path]::GetRelativePath($packet,$file.FullName).Replace('\','/'); bytes=$file.Length
        sha256=(Get-FileHash $file.FullName).Hash.ToLowerInvariant()}
}
$manifest | ConvertTo-Json -Depth 10 | Set-Content (Join-Path $packet "PACKAGE_MANIFEST.json") -Encoding utf8NoBOM
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($packet,$zip,[IO.Compression.CompressionLevel]::Optimal,$false)
$archive = [IO.Compression.ZipFile]::OpenRead($zip)
try {
    if ($archive.Entries.Count -ne $manifest.files.Count+1) { throw "Entry count mismatch" }
    foreach ($record in $manifest.files) {
        $entry = $archive.GetEntry($record.path)
        if ($null -eq $entry -or $entry.Length -ne $record.bytes) { throw "Entry mismatch" }
        $stream = $entry.Open()
        try { $hash = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($stream)).ToLowerInvariant() }
        finally { $stream.Dispose() }
        if ($hash -ne $record.sha256) { throw "Entry hash mismatch" }
    }
} finally { $archive.Dispose() }
@{zip=$zip; sha256=(Get-FileHash $zip).Hash.ToLowerInvariant(); bytes=(Get-Item $zip).Length
  entries=$manifest.files.Count+1; verified=$true; package_commit=$head} | ConvertTo-Json
