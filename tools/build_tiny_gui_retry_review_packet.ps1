#requires -Version 7.0
[CmdletBinding()]
param([string]$OutputName = "review_tiny_gui_retry_af80183_v1")
$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repository "tmp/tiny_gui_retry_20260907"
$execution = "3bddc665efdec7d9ab4f4cc5e89821a2cc4b424e"
$fixed = "af80183146e84f65f4fa8d56aa77e1ef9e8dd167"
$attempt = "gui_retry_3bddc66_01"
if ($OutputName -notmatch '^[a-zA-Z0-9_-]+$') { throw "Invalid output name" }
$packet = Join-Path $taskRoot $OutputName
$zip = "$packet.zip"
if ((Test-Path $packet) -or (Test-Path $zip)) { throw "Refusing overwrite" }
$dirty = @(& git -C $repository status --porcelain=v1)
if ($LASTEXITCODE -ne 0 -or $dirty.Count) { throw "Require clean committed Windows source" }
$head = (& git -C $repository rev-parse HEAD).Trim()
& git -C $repository merge-base --is-ancestor $fixed $head
if ($LASTEXITCODE -ne 0) { throw "Fixed version missing" }
& git -C $repository diff --quiet $fixed $head -- Makefile src tests integrations ros2_ws tools/run_tiny_lidar_dev.py tools/tiny_dev_runner.py tools/tiny_gui_integration.py
if ($LASTEXITCODE -ne 0) { throw "Runtime differs from tested source" }
$tar = Join-Path $taskRoot "source_3bddc66.tar"
if ((Get-FileHash $tar -Algorithm SHA256).Hash.ToLowerInvariant() -ne "7991b31f05fb412655dc4b9cc39229b4d070508adb85598698cc88f640b87efc") { throw "Execution source mismatch" }
$budget = Get-Content (Join-Path $taskRoot "$attempt/budget_after.json") -Raw | ConvertFrom-Json
if ($budget.used.powered -ne 5 -or $null -ne $budget.active) { throw "Unexpected budget" }
[xml]$junit = Get-Content (Join-Path $taskRoot "evidence/tests_af80183.xml") -Raw
$suite = $junit.testsuites.testsuite
if ([int]$suite.tests -ne 101 -or [int]$suite.failures -ne 0 -or [int]$suite.errors -ne 0 -or [int]$suite.skipped -ne 0) { throw "Unexpected tests" }
New-Item -ItemType Directory -Path $packet | Out-Null
function Copy-Review([string]$Source, [string]$Relative) {
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) { throw "Missing $Source" }
    $destination = Join-Path $packet $Relative
    New-Item -ItemType Directory -Path (Split-Path $destination) -Force | Out-Null
    Copy-Item -LiteralPath $Source -Destination $destination
}
Copy-Review $tar "source/source_3bddc66.tar"
Copy-Review (Join-Path $repository "docs/tiny_gui_retry_20260907_results.md") "README_REVIEW.md"
Copy-Review (Join-Path $repository "docs/tiny_gui_retry_20260907.md") "execution_plan.md"
Copy-Review $PSCommandPath "packaging/build_tiny_gui_retry_review_packet.ps1"
# Only dedicated evidence/output folders, no environment/model/data exploration.
foreach ($folder in @("evidence", $attempt)) {
    $sourceRoot = Join-Path $taskRoot $folder
    foreach ($file in Get-ChildItem -LiteralPath $sourceRoot -Recurse -File) {
        if ($file.Extension -notin @(".txt", ".json", ".jsonl", ".xml", ".log")) { throw "Unexpected evidence $($file.Name)" }
        $relative = [IO.Path]::GetRelativePath($sourceRoot, $file.FullName).Replace('\', '/')
        Copy-Review $file.FullName "$folder/$relative"
    }
}
& git -C $repository diff --binary "--output=$(Join-Path $packet 'source/post_execution.patch')" $execution $head
if ($LASTEXITCODE -ne 0) { throw "Diff failed" }
$versions = [ordered]@{
    schema="TINY_GUI_RETRY_REVIEW_V1"; origin=(& git -C $repository remote get-url origin).Trim()
    branch=(& git -C $repository branch --show-current).Trim(); package_commit=$head
    execution_commit=$execution; fixed_test_commit=$fixed; source_format="Execution tar plus post-execution patch"
    attempt=$attempt; official_commit="1f54dff995d02625566341f9e1be1c39369224f2"
    weight_sha256="7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963"
    final_used=$budget.used; powered_remaining=0; new_attempt_authorized=$false
    fixed_runtime_live_tested=$false; automatic_push=$false
}
$versions | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $packet "versions.json") -Encoding utf8NoBOM
$manifest = [ordered]@{schema="TINY_GUI_RETRY_MANIFEST_V1"; versions=$versions
    weights_included=$false; sensor_included=$false; video_included=$false
    manifest_self_excluded=$true; files=@()}
foreach ($file in Get-ChildItem -LiteralPath $packet -Recurse -File | Sort-Object FullName) {
    $relative = [IO.Path]::GetRelativePath($packet, $file.FullName).Replace('\', '/')
    $manifest.files += [ordered]@{path=$relative; bytes=$file.Length; sha256=(Get-FileHash $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
}
$manifest | ConvertTo-Json -Depth 10 | Set-Content (Join-Path $packet "PACKAGE_MANIFEST.json") -Encoding utf8NoBOM
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($packet, $zip, [IO.Compression.CompressionLevel]::Optimal, $false)
$archive = [IO.Compression.ZipFile]::OpenRead($zip)
try {
    if ($archive.Entries.Count -ne $manifest.files.Count+1) { throw "ZIP count mismatch" }
    foreach ($record in $manifest.files) {
        $entry = $archive.GetEntry($record.path)
        if ($null -eq $entry -or $entry.Length -ne $record.bytes) { throw "ZIP entry mismatch" }
        $stream = $entry.Open()
        try { $hash = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($stream)).ToLowerInvariant() }
        finally { $stream.Dispose() }
        if ($hash -ne $record.sha256) { throw "ZIP hash mismatch" }
    }
} finally { $archive.Dispose() }
$receipt = [ordered]@{zip=$zip; bytes=(Get-Item $zip).Length
    sha256=(Get-FileHash $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    files=$manifest.files.Count+1; verified_all_manifest_hashes_from_zip=$true; package_commit=$head}
$receipt | ConvertTo-Json | Set-Content "$zip.receipt.json" -Encoding utf8NoBOM
$receipt | ConvertTo-Json
