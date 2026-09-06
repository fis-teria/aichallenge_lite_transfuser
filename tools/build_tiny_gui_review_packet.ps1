#requires -Version 7.0
[CmdletBinding()]
param([string]$OutputName = "review_tiny_gui_d0b86e3_v1")
$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repository "tmp/tiny_gui_control_method_20260907"
$execution = "52376121d52a44f678e13a20a87127e1532e203a"
$fixed = "d0b86e3cc89bbcecabfd841c7c9a772352043566"
$start = "5c8079dd3e0592d403c4be7992cdb6cc3e6bc4a9"
$attempt = "gui_short_5237612_01"
if ($OutputName -notmatch '^[a-zA-Z0-9_-]+$') { throw "Invalid output name" }
$packet = Join-Path $taskRoot $OutputName
$zip = "$packet.zip"
if ((Test-Path -LiteralPath $packet) -or (Test-Path -LiteralPath $zip)) { throw "Refusing overwrite" }
$dirty = @(& git -C $repository status --porcelain=v1)
if ($LASTEXITCODE -ne 0 -or $dirty.Count -ne 0) { throw "Require committed clean Windows source" }
$head = (& git -C $repository rev-parse HEAD).Trim()
& git -C $repository merge-base --is-ancestor $fixed $head
if ($LASTEXITCODE -ne 0) { throw "Fixed commit is not an ancestor" }
$runtime = @("Makefile", "configs/control/tiny_lidar_sim.yaml", "configs/control/tiny_gui_authorization_20260907.json",
    "tools/run_tiny_lidar_dev.py", "tools/tiny_dev_runner.py", "tools/tiny_gui_integration.py", "tools/spatial_dev_host_v4.py",
    "src/aic_transfuser_lite/__init__.py", "src/aic_transfuser_lite/runtime/__init__.py",
    "src/aic_transfuser_lite/runtime/tiny_lidar_sim.py", "tests/test_tiny_lidar_sim.py", "tests/test_tiny_gui_integration.py",
    "integrations/tiny_gui/Makefile", "integrations/tiny_gui/runtime.sh", "integrations/tiny_gui/simulator.sh",
    "integrations/awsim_dev_v4/cyclonedds.xml")
$ros = @(& git -C $repository ls-files ros2_ws/src/aic_tiny_sim_test)
if ($LASTEXITCODE -ne 0 -or $ros.Count -lt 7) { throw "ROS package listing missing" }
$runtime += $ros
& git -C $repository diff --quiet $fixed $head -- @runtime
if ($LASTEXITCODE -ne 0) { throw "Runtime differs from tested fixed version" }
$summary = Get-Content (Join-Path $taskRoot "$attempt/host_summary.json") -Raw | ConvertFrom-Json
$budget = Get-Content (Join-Path $taskRoot "$attempt/budget_after.json") -Raw | ConvertFrom-Json
if ($summary.source_commit -ne $execution -or $null -ne $summary.runtime_container_id -or
    -not $summary.error.Contains("conflicting options: hostname and the network mode") -or
    $budget.used.powered -ne 4 -or $null -ne $budget.active) { throw "Unexpected saved result" }
foreach ($version in @("5237612", "d0b86e3")) {
    [xml]$junit = Get-Content (Join-Path $taskRoot "evidence/tests_$version.xml") -Raw
    $suite = $junit.testsuites.testsuite
    if ([int]$suite.tests -ne 98 -or [int]$suite.failures -ne 0 -or [int]$suite.errors -ne 0 -or [int]$suite.skipped -ne 0) {
        throw "Unexpected test result: $version"
    }
}
New-Item -ItemType Directory -Path $packet | Out-Null
function Copy-PacketFile([string]$Source, [string]$Relative) {
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) { throw "Missing: $Source" }
    $target = Join-Path $packet $Relative
    New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $target
}
$documents = @("docs/tiny_gui_control_method_20260907.md", "docs/tiny_gui_control_method_20260907_results.md",
    "docs/tiny_gui_control_method_20260907_review.md", "tools/build_tiny_gui_review_packet.ps1",
    "tools/sync_to_wsl.ps1", "tools/with_wsl_training_lock.sh")
foreach ($file in ($runtime + $documents)) { Copy-PacketFile (Join-Path $repository $file) "source/$file" }
Copy-PacketFile (Join-Path $repository $documents[1]) "report/tiny_gui_control_method_20260907_results.md"
Copy-PacketFile (Join-Path $repository $documents[2]) "README_REVIEW.md"
# Only these task-owned log folders; no model/data/workspace traversal.
foreach ($folder in @($attempt, "evidence")) {
    $prefix = if ($folder -eq $attempt) { "attempts/$attempt" } else { "evidence" }
    foreach ($file in Get-ChildItem -LiteralPath (Join-Path $taskRoot $folder) -File) {
        if ($file.Extension -notin @(".json", ".jsonl", ".txt", ".xml")) { throw "Unexpected log: $($file.Name)" }
        Copy-PacketFile $file.FullName "$prefix/$($file.Name)"
    }
}
$versions = [ordered]@{
    schema="TINY_GUI_VERSIONS_V1"; origin=(& git -C $repository remote get-url origin).Trim()
    branch=(& git -C $repository branch --show-current).Trim(); start_commit=$start; start_tree="CLEAN"
    execution_commit=$execution; ros_build_commit=$execution; fixed_test_commit=$fixed
    package_commit=$head; package_tree="CLEAN"; source_matches_fixed_test=$true
    fixed_version_ros_start_verified=$false; gui_drive_verified=$false
    official_commit="1f54dff995d02625566341f9e1be1c39369224f2"
    weight_sha256="7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963"
    execution_archive_sha256="d1032fa907de427fb0452b7a3b5d94cdf775ab2a06ad7ea0dd2085451c255228"
    automatic_push=$false
}
$versions | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $packet "versions.json") -Encoding utf8NoBOM
New-Item -ItemType Directory -Path (Join-Path $packet "diff") | Out-Null
foreach ($record in @(@("implementation", $start, $execution), @("hostname_fix", $execution, $fixed), @("packaging", $fixed, $head))) {
    & git -C $repository diff --binary "--output=$(Join-Path $packet "diff/$($record[0]).patch")" $record[1] $record[2]
    if ($LASTEXITCODE -ne 0) { throw "Diff generation failed" }
}
$commands = @'
# Historical execution record — not permission to repeat

Windows at 5237612 and d0b86e3:
pwsh -NoProfile -File tools/sync_to_wsl.ps1 -CheckOnly
pwsh -NoProfile -File tools/sync_to_wsl.ps1
The first 5237612 synchronization used Windows PowerShell, produced a BOM/set warning,
and was repeated with pwsh. Both logs are preserved.

WSL /home/thistle/e2e_autonomous/e2e_lite_transfuser at each matching commit:
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py tests/test_tiny_gui_integration.py --junitxml=/home/thistle/e2e_autonomous/tiny_gui_control_method_20260907/tests_VERSION.xml
VERSION is the recorded 5237612 or d0b86e3. Tests do not forward the model.

Remote source cwd:
/home/graneple/e2e_autonomous/tiny_gui_control_method_20260907/source_5237612
make dev CONTROL_METHOD=tiny_lidar_net_guarded SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller TINY_INSTALL=/home/graneple/e2e_autonomous/tiny_gui_control_method_20260907/install_5237612 DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority TINY_OUTPUT=/home/graneple/e2e_autonomous/tiny_gui_control_method_20260907/gui_short_5237612_01 TINY_COMMIT=52376121d52a44f678e13a20a87127e1532e203a TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json

Exact child make/docker/cleanup argv and responses:
attempts/gui_short_5237612_01/make_invocation.json and host.jsonl.
Control service creation failed, make exit 2; no supervisor started.

ROS package build (5237612) in fixed image, network none, cap-drop ALL,
no-new-privileges, uid 1000, no physical device, original workspace not mounted:
source /autoware/install/setup.bash
colcon --log-base /tiny_build_log build --base-paths /v4/ros2_ws/src/aic_tiny_sim_test --packages-select aic_tiny_sim_test --build-base /tiny_build --install-base /tiny_install --event-handlers console_direct+
Read-only /v4 is source_5237612; dedicated build/install/build_log_5237612 directories
map to /tiny_build, /tiny_install, /tiny_build_log. PYTHONDONTWRITEBYTECODE=1.
Image: sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b4c6b96d1ba7.
Build exit 0 (1.48s), help/show-args exit 0; no live ROS node in these checks.

Post-fix non-ROS X11 probe: owned sleep-20-second network-none donor with hostname
graneple-local; client network=container:donor WITHOUT hostname, uid1000,
read-only X11 socket and /run/user/1000/gdm/Xauthority, DISPLAY=:1,
XAUTHORITY=/desktop_xauth, XAUTHLOCALHOSTNAME=graneple-local.
Python ctypes libX11 XOpenDisplay/XCloseDisplay only; exit 0, both containers cleaned.
No ROS, model or control invocation. This is a description of the observed probe;
its complete shell wrapper was not captured as a standalone command artifact.
'@
$commands | Set-Content (Join-Path $packet "evidence/execution_commands.md") -Encoding utf8NoBOM
$manifest = [ordered]@{
    schema="TINY_GUI_REVIEW_PACKET_V1"; created_utc=[DateTime]::UtcNow.ToString("o")
    versions=$versions; attempt=$attempt; powered_remaining=0; final_used=$budget.used
    measured_tiny_forward="NOT_STARTED_CONTROL_CONTAINER_NOT_CREATED"
    tiny_forward_charged=600; charged_exact=$false; gui_drive="NOT_EXECUTED"
    weights_included=$false; raw_sensor_included=$false; video_included=$false
    prior_attempt_raw_logs_included=$false; prior_budget_history_included=$true
    new_driving_authorized_by_packet=$false; independent_review_completed=$false
    manifest_self_excluded=$true; zip_hash_location="external .zip.receipt.json"; files=@()
}
foreach ($file in Get-ChildItem -LiteralPath $packet -Recurse -File | Sort-Object FullName) {
    $relative = [IO.Path]::GetRelativePath($packet, $file.FullName).Replace('\', '/')
    if ($relative -match '(^|/)(\.git|datasets|checkpoint|\.ssh|\.venv|__pycache__)(/|$)' -or
        $relative -match '\.(pt|pth|npy|npz|mcap|db3|pkl|pickle|pyc)$') { throw "Forbidden artifact: $relative" }
    $manifest.files += [ordered]@{path=$relative; bytes=$file.Length; sha256=(Get-FileHash $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
}
$manifest | ConvertTo-Json -Depth 12 | Set-Content (Join-Path $packet "PACKAGE_MANIFEST.json") -Encoding utf8NoBOM
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
        if ($hash -ne $record.sha256) { throw "ZIP hash mismatch: $($record.path)" }
    }
} finally { $archive.Dispose() }
$receipt = [ordered]@{
    zip=$zip; bytes=(Get-Item $zip).Length; sha256=(Get-FileHash $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    files=$manifest.files.Count+1; verified_all_manifest_hashes_from_zip=$true
    package_commit=$head; execution_commit=$execution; fixed_test_commit=$fixed
    new_drive_authorized=$false
}
$receipt | ConvertTo-Json | Set-Content "$zip.receipt.json" -Encoding utf8NoBOM
$receipt | ConvertTo-Json
