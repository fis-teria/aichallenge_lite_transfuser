[CmdletBinding()]
param(
    [string]$OutputName = "review_tiny_ready_dcf3f9c_v1"
)
$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repository "tmp/tiny_lidar_lap_20260907"
if ($OutputName -notmatch '^[a-zA-Z0-9_-]+$') { throw "Invalid output name" }
$packet = Join-Path $taskRoot $OutputName
$zip = "$packet.zip"
if ((Test-Path -LiteralPath $packet) -or (Test-Path -LiteralPath $zip)) { throw "Refusing to overwrite a review packet" }
New-Item -ItemType Directory -Path $packet | Out-Null
function Copy-PacketFile([string]$Source, [string]$Relative) {
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) { throw "Missing: $Source" }
    $target = Join-Path $packet $Relative
    New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $target
}
$sourceFiles = @(
    "Makefile", "configs/control/tiny_lidar_sim.yaml",
    "docs/tiny_lidar_lap_20260907.md", "docs/tiny_lidar_lap_20260907_results.md",
    "integrations/tiny_dev/runtime.sh", "integrations/awsim_dev_v4/simulator.sh",
    "integrations/awsim_dev_v4/cyclonedds.xml", "tools/spatial_dev_host_v4.py",
    "src/aic_transfuser_lite/runtime/tiny_lidar_sim.py", "tools/tiny_dev_runner.py",
    "tools/run_tiny_lidar_dev.py", "tests/test_tiny_lidar_sim.py", "tools/build_tiny_review_packet.ps1"
)
foreach ($relative in $sourceFiles) { Copy-PacketFile (Join-Path $repository $relative) "source/$relative" }
Copy-PacketFile (Join-Path $repository "docs/tiny_lidar_lap_20260907_review.md") "README_REVIEW.md"
foreach ($name in @("stationary_7b88981_01", "stationary_4d6ddd2_02", "short_4d6ddd2_03")) {
    $attempt = Join-Path $taskRoot $name
    foreach ($file in Get-ChildItem -LiteralPath $attempt -File -Recurse) {
        $relative = [IO.Path]::GetRelativePath($attempt, $file.FullName).Replace([IO.Path]::DirectorySeparatorChar, [char]'/')
        if ($relative.StartsWith("x11/")) { continue }
        Copy-PacketFile $file.FullName "attempts/$name/$relative"
    }
}
foreach ($name in @("evidence", "official_source")) {
    $folder = Join-Path $taskRoot $name
    foreach ($file in Get-ChildItem -LiteralPath $folder -File -Recurse) {
        $relative = [IO.Path]::GetRelativePath($folder, $file.FullName).Replace([IO.Path]::DirectorySeparatorChar, [char]'/')
        Copy-PacketFile $file.FullName "$name/$relative"
    }
}
$staticRoot = Join-Path $repository "tmp/spatial_sim_e2e_20260906/dev_review_packet/historical/simulator_static"
foreach ($relative in @("provenance.json", "LapCount.cs", "AWSIM/VehicleRosInput.cs", "AWSIM/Vehicle.cs", "AWSIM/OnCollisionRos2Publisher.cs")) {
    Copy-PacketFile (Join-Path $staticRoot $relative) "consumer_static/$relative"
}
$head = (& git -C $repository rev-parse HEAD).Trim()
$manifest = [ordered]@{
    schema = "TINY_REVIEW_PACKET_V1"
    created_utc = [DateTime]::UtcNow.ToString("o")
    package_source_commit = $head
    origin = (& git -C $repository remote get-url origin).Trim()
    branch = (& git -C $repository branch --show-current).Trim()
    working_tree_at_packaging = @(& git -C $repository status --porcelain=v1)
    execution_commits = @("7b88981c20d0afc85dba737bf996520f7c605c26", "4d6ddd284323fa73f38d6106b22cbd5a0eeb37ed")
    runtime_implementation_commit = "dcf3f9c248609ecd4e8a49e916f1ac510fff6e96"
    readiness_fix_live_tested = $false
    official_commit = "1f54dff995d02625566341f9e1be1c39369224f2"
    weight_sha256 = "7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963"
    weight_included = $false
    complete_lap_confirmed = $false
    blocker = "EXPLICIT_BUDGET_EXTENSION_REQUIRED"
    raw_logs_retained = $true
    real_screen_video_included = $false
    new_drive_authorized_by_packet = $false
    files = @()
}
foreach ($file in Get-ChildItem -LiteralPath $packet -File -Recurse | Sort-Object FullName) {
    $relative = [IO.Path]::GetRelativePath($packet, $file.FullName).Replace([IO.Path]::DirectorySeparatorChar, [char]'/')
    if ($relative -match '(^|/)(\.git|datasets|checkpoint|\.ssh)(/|$)' -or $relative -match '\.(pt|pth|npy|npz|mcap|db3)$') {
        throw "Forbidden artifact: $relative"
    }
    $manifest.files += [ordered]@{path=$relative; bytes=$file.Length; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
}
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $packet "manifest.json") -Encoding utf8NoBOM
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($packet, $zip, [IO.Compression.CompressionLevel]::Optimal, $false)
$archive = [IO.Compression.ZipFile]::OpenRead($zip)
try {
    if (-not ($archive.Entries.FullName -contains "README_REVIEW.md") -or -not ($archive.Entries.FullName -contains "manifest.json")) {
        throw "ZIP root layout mismatch"
    }
    $sha = [Security.Cryptography.SHA256]::Create()
    foreach ($record in $manifest.files) {
        $entry = $archive.GetEntry($record.path)
        if ($null -eq $entry -or $entry.Length -ne $record.bytes) { throw "ZIP entry mismatch: $($record.path)" }
        $stream = $entry.Open()
        try { $actual = [Convert]::ToHexString($sha.ComputeHash($stream)).ToLowerInvariant() }
        finally { $stream.Dispose() }
        if ($actual -ne $record.sha256) { throw "ZIP hash mismatch: $($record.path)" }
    }
    $sha.Dispose()
} finally { $archive.Dispose() }
$result = [ordered]@{
    zip = $zip
    bytes = (Get-Item -LiteralPath $zip).Length
    sha256 = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    file_count = $manifest.files.Count + 1
    verified_all_manifest_hashes_from_zip = $true
    local_task_bytes_including_source_archives_and_package_duplicates = (Get-ChildItem -LiteralPath $taskRoot -Recurse -File | Measure-Object -Property Length -Sum).Sum
}
$result | ConvertTo-Json | Set-Content -LiteralPath "$zip.receipt.json" -Encoding utf8NoBOM
$result | ConvertTo-Json
