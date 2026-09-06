[CmdletBinding()]
param([string]$OutputName = "review_tiny_ready_lap_219eb2f_v1")
$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repository "tmp/tiny_lidar_ready_lap_20260907"
$previousRoot = Join-Path $repository "tmp/tiny_lidar_lap_20260907"
$execution = "219eb2f00b8e4277275072d0b5e9f344f365edc7"
$startCommit = "a87e17a089a75c7ab6ed3c9251fb6b8fefc7f13d"
$requestHash = "3210c97ce30a8af18abb9633378a0c9993f55c01f2231dda262142f17a646bcd"
if ($OutputName -notmatch '^[a-zA-Z0-9_-]+$') { throw "Invalid output name" }
$packet = Join-Path $taskRoot $OutputName
$zip = "$packet.zip"
if ((Test-Path -LiteralPath $packet) -or (Test-Path -LiteralPath $zip)) { throw "Refusing to overwrite a packet" }
$dirty = @(& git -C $repository status --porcelain=v1)
if ($LASTEXITCODE -ne 0 -or $dirty.Count -ne 0) { throw "Package requires committed clean Windows source" }
$head = (& git -C $repository rev-parse HEAD).Trim()
& git -C $repository merge-base --is-ancestor $execution $head
if ($LASTEXITCODE -ne 0) { throw "Execution commit is not an ancestor" }
$runtimeFiles = @("Makefile", "configs/control/tiny_lidar_sim.yaml", "tools/run_tiny_lidar_dev.py",
    "tools/tiny_dev_runner.py", "tools/spatial_dev_host_v4.py", "src/aic_transfuser_lite/runtime/tiny_lidar_sim.py",
    "tests/test_tiny_lidar_sim.py", "integrations/tiny_dev/runtime.sh", "integrations/awsim_dev_v4/simulator.sh",
    "integrations/awsim_dev_v4/cyclonedds.xml")
& git -C $repository diff --quiet $execution $head -- @runtimeFiles
if ($LASTEXITCODE -ne 0) { throw "Runtime/test source changed after the live execution" }
$requestFile = Join-Path $taskRoot "request/implementation_request.txt"
if ((Get-FileHash -LiteralPath $requestFile -Algorithm SHA256).Hash.ToLowerInvariant() -ne $requestHash) {
    throw "Request bytes changed"
}
$check = Get-Content -LiteralPath (Join-Path $taskRoot "evidence/saved_log_check.json") -Raw | ConvertFrom-Json
if ($check.attempts.Count -ne 2 -or $check.remaining.powered -ne 0) { throw "Unexpected saved result set" }
[xml]$junit = Get-Content -LiteralPath (Join-Path $taskRoot "evidence/tests_219eb2f.xml") -Raw
$suite = $junit.testsuites.testsuite
if ([int]$suite.tests -ne 72 -or [int]$suite.failures -ne 0 -or [int]$suite.errors -ne 0 -or [int]$suite.skipped -ne 0) {
    throw "Test result does not match the report"
}
New-Item -ItemType Directory -Path $packet | Out-Null
function Copy-PacketFile([string]$Source, [string]$Relative) {
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) { throw "Missing: $Source" }
    $target = Join-Path $packet $Relative
    New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $target
}
function Copy-PacketTree([string]$Folder, [string]$Prefix) {
    foreach ($file in Get-ChildItem -LiteralPath $Folder -File -Recurse) {
        $relative = [IO.Path]::GetRelativePath($Folder, $file.FullName).Replace([IO.Path]::DirectorySeparatorChar, [char]'/')
        if ($relative.StartsWith("x11/")) { continue } # Unix display socket is not a regular review artifact.
        Copy-PacketFile $file.FullName "$Prefix/$relative"
    }
}
$sourceFiles = $runtimeFiles + @("docs/tiny_ready_lap_20260907.md", "docs/tiny_ready_lap_20260907_results.md",
    "docs/tiny_ready_lap_20260907_review.md", "tools/summarize_tiny_ready_trials.ps1",
    "tools/build_tiny_ready_review_packet.ps1", "tools/sync_to_wsl.ps1", "tools/with_wsl_training_lock.sh")
foreach ($relative in $sourceFiles) { Copy-PacketFile (Join-Path $repository $relative) "source/$relative" }
Copy-PacketFile (Join-Path $repository "docs/tiny_ready_lap_20260907_review.md") "README_REVIEW.md"
Copy-PacketFile (Join-Path $repository "docs/tiny_ready_lap_20260907_results.md") "report/tiny_ready_lap_20260907_results.md"
Copy-PacketFile $requestFile "request/implementation_request.txt"
foreach ($name in @("short_219eb2f_01", "lap_219eb2f_02")) {
    Copy-PacketTree (Join-Path $taskRoot $name) "attempts/$name"
}
Copy-PacketTree (Join-Path $taskRoot "evidence") "evidence"
foreach ($name in @("stationary_7b88981_01", "stationary_4d6ddd2_02", "short_4d6ddd2_03")) {
    Copy-PacketTree (Join-Path $previousRoot $name) "history/attempts/$name"
}
Copy-PacketTree (Join-Path $previousRoot "evidence") "history/evidence"
Copy-PacketTree (Join-Path $previousRoot "official_source") "official_source"
Copy-PacketFile (Join-Path $repository "docs/tiny_lidar_lap_20260907_results.md") "history/previous_results.md"
Copy-PacketFile (Join-Path $previousRoot "review_tiny_ready_dcf3f9c_v1/manifest.json") "history/previous_manifest.json"
$staticRoot = Join-Path $previousRoot "review_tiny_ready_dcf3f9c_v1/consumer_static"
Copy-PacketTree $staticRoot "consumer_static"
$identity = Get-Content -LiteralPath (Join-Path $taskRoot "lap_219eb2f_02/official_identity.json") -Raw | ConvertFrom-Json
foreach ($property in $identity.source_sha256.PSObject.Properties) {
    $localSource = Join-Path $packet "official_source/tiny_lidar_net_controller/$($property.Name)"
    if ((Get-FileHash -LiteralPath $localSource -Algorithm SHA256).Hash.ToLowerInvariant() -ne $property.Value) {
        throw "Official source hash mismatch: $($property.Name)"
    }
}
$documentCommit = (& git -C $repository log -1 --format=%H -- docs/tiny_ready_lap_20260907_results.md).Trim()
$versions = [ordered]@{
    schema = "TINY_READY_TRIAL_VERSIONS_V1"
    created_utc = [DateTime]::UtcNow.ToString("o")
    origin = (& git -C $repository remote get-url origin).Trim()
    branch = (& git -C $repository branch --show-current).Trim()
    start_head = $startCommit
    start_working_tree = "CLEAN; recorded before implementation"
    implementation_commit = $execution
    test_commit = $execution
    short_commit = $execution
    lap_commit = $execution
    result_document_commit = $documentCommit
    package_source_commit = $head
    package_working_tree = $dirty
    runtime_unchanged_since_execution = $true
    archive_sha256 = "24997bdea58c92fec7d90844765d778256d05b8a3588c707b4b20508ed329d57"
    previous_packet_sha256 = "44a3e61f72da3c5ea6effec7aeeaab56c99e2f9a928b3702d4ca31b5103db429"
    official_commit = $identity.official_commit
    weight_sha256 = $identity.weight_sha256
    source_files = @($sourceFiles | ForEach-Object {
        [ordered]@{path=$_; git_blob=(& git -C $repository rev-parse "${head}:$_").Trim()}
    })
    automatic_push = $false
}
$versions | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $packet "versions.json") -Encoding utf8NoBOM
New-Item -ItemType Directory -Path (Join-Path $packet "diff") | Out-Null
& git -C $repository diff --binary "--output=$(Join-Path $packet 'diff/runtime.patch')" $startCommit $execution
if ($LASTEXITCODE -ne 0) { throw "Runtime diff failed" }
& git -C $repository diff --binary "--output=$(Join-Path $packet 'diff/results_packaging.patch')" $execution $head
if ($LASTEXITCODE -ne 0) { throw "Artifact diff failed" }
$receipts = [ordered]@{
    schema = "TINY_READY_PROVIDED_EXECUTION_RECEIPTS_V1"
    scope = "Previously observed tool exit codes; packaging reads logs only, does not rerun tests or simulator"
    test = [ordered]@{
        commit=$execution; command_file="evidence/execution_commands.txt"; host="codex-wsl"
        checkout="/home/thistle/e2e_autonomous/e2e_lite_transfuser"; lock="tools/with_wsl_training_lock.sh"
        stdout="evidence/tests_219eb2f.txt"; junit="evidence/tests_219eb2f.xml"
        exit_code=0; exit_evidence="Execution tool returned exit 0 and SSH_TEST_EXIT_CODE=0"
        passed=72; failed=0; skipped=0; elapsed_s=5.44
        stderr="No separate stderr artifact captured; do not infer independently verified empty stderr"
        full_pytest_executed=$false; model_forward_executed=$false
    }
    short = [ordered]@{attempt="short_219eb2f_01"; make_exit_code=0; runtime_exit_code=0; console="evidence/short_219eb2f_01_console.txt"}
    lap = [ordered]@{attempt="lap_219eb2f_02"; make_exit_code=2; runtime_exit_code=1; console="evidence/lap_219eb2f_02_console.txt"}
    simulator_host="graneple@192.168.3.10"
    image_sha256="8c650c13157ffabbc3a72bab08865ccff8338f9025b4c6b96d1ba7"
    current_tiny_forward=202; current_v4_forward=0; current_mpc=0
    video="NOT_AVAILABLE; no new acquisition tool installed"
}
$receipts | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $packet "evidence/execution_receipts.json") -Encoding utf8NoBOM
$commands = @'
HISTORICAL EXECUTION RECORD -- NOT AUTHORIZATION TO EXECUTE AGAIN

Windows source clean at 219eb2f00b8e4277275072d0b5e9f344f365edc7:
powershell -NoProfile -File tools/sync_to_wsl.ps1 -CheckOnly
powershell -NoProfile -File tools/sync_to_wsl.ps1

WSL checkout /home/thistle/e2e_autonomous/e2e_lite_transfuser at that same SHA:
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py --junitxml=/home/thistle/e2e_autonomous/tiny_lidar_ready_lap_20260907/tests_219eb2f.xml

graneple@192.168.3.10 source cwd /home/graneple/e2e_autonomous/tiny_lidar_ready_lap_20260907/source_219eb2f:
make dev DEV_CONTROLLER=tiny TINY_PHASE=short TINY_WALL_SECONDS=120 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root TINY_OUTPUT=/home/graneple/e2e_autonomous/tiny_lidar_ready_lap_20260907/short_219eb2f_01 TINY_COMMIT=219eb2f00b8e4277275072d0b5e9f344f365edc7 TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
make dev DEV_CONTROLLER=tiny TINY_PHASE=lap TINY_WALL_SECONDS=600 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root TINY_OUTPUT=/home/graneple/e2e_autonomous/tiny_lidar_ready_lap_20260907/lap_219eb2f_02 TINY_COMMIT=219eb2f00b8e4277275072d0b5e9f344f365edc7 TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json

Runtime child/docker/cleanup commands and responses: each attempts/*/host.jsonl.
Original racing-kart Makefile was not executed. Re-running these commands is not authorized by this packet.
'@
$commands | Set-Content -LiteralPath (Join-Path $packet "evidence/execution_commands.txt") -Encoding utf8NoBOM
$original = Get-Content -LiteralPath $requestFile -Raw
$marker = "独立レビュー依頼：TinyLidarNet Ready後短試験・1周試験"
$offset = $original.IndexOf($marker, [StringComparison]::Ordinal)
if ($offset -lt 0) { throw "Full independent review request not found" }
$prompt = $original.Substring($offset)
$replacements = [ordered]@{
    "今回開始HEAD: [実値]" = "今回開始HEAD: $startCommit (clean)"
    "今回Ready/budget修正・test版: [実値]" = "今回Ready/budget修正・test版: $execution"
    "Ready後short実行版／attempt: [実値またはNOT_EXECUTED]" = "Ready後short実行版／attempt: $execution / short_219eb2f_01"
    "lap実行版／attempt: [実値またはNOT_EXECUTED]" = "lap実行版／attempt: $execution / lap_219eb2f_02"
    "結果文書版／梱包版: [実値]" = "結果文書版／梱包版: $documentCommit / $head"
    "ZIP名: [実値]" = "ZIP名: $OutputName.zip"
}
foreach ($replacement in $replacements.GetEnumerator()) {
    if (-not $prompt.Contains($replacement.Key)) { throw "Review placeholder not found: $($replacement.Key)" }
    $prompt = $prompt.Replace($replacement.Key, $replacement.Value)
}
if ($prompt.Contains("[実値")) { throw "Unfilled review placeholder" }
$prompt += @"


今回の提供結果（独立再計算・再実行ではない）

- 今回test: 72 passed / failed 0 / skipped 0, 5.44s, exit 0。旧112とは別。
- short_219eb2f_01: Tiny forward165 / Tiny操舵送信153 / distinct scan149（観測移動中133）、最高1.395915985m/s。SHORT_MOTION_COMPLETE、制動25送信、停止帯16sample/0.524999989sim秒。
- lap_219eb2f_02: Tiny forward37 / Tiny操舵送信31 / distinct scan30（観測移動中14）、最高0.272549868m/s。HOST_MONITOR_STALEで停止、制動7送信、停止帯16sample/0.524999988sim秒。
- 両attemptでReady fence違反0 / worker→送信join不一致0。両方section0/lap0、一周未成立。host freeze/KILLあり、unpauseなし。無接触/無逸脱UNKNOWN。
- lapの直接終了条件はHOST_MONITOR_STALE。ARM-at-fault未保存のため、750ms更新遅延か読取前nowと並行ARM更新の競合かはUNKNOWN。競合の静的反例と実原因を区別する。
- 最終used: wall1253.793699550s / V4 forward124 / Tiny375 / shared499 / powered3 / powered_s21.259999524 / log61961373bytes / snapshots7 / MPC1、active=null。
- 残量: powered0 / sim278.740000476s / shared forward5501 / wall2346.206300450s / log474909539bytes / snapshots9。powered0のため再走行しない。
- 実終了05:52:06.821 JST、09:50cutoff/10:00期限より前。全5Tiny attemptを同梱。旧V4消費履歴はbudgetに保持、今回V4/MPC実行なし。
- 固定コード・今回結果は添付ZIPが正本。自動pushなし。実画面動画なし。提案は未実装であり、レビューは追加駆動を承認しない。
"@
$prompt | Set-Content -LiteralPath (Join-Path $packet "request/independent_review_request.md") -Encoding utf8NoBOM
$statuses = [ordered]@{
    READY_PATCH_LIVE_TESTED="CONFIRMED"; READY_POST_RECEIPT_SCAN_USED="CONFIRMED"
    TINY_UPDATED_STEERING_SENT_WHILE_MOVING="CONFIRMED"; JUDGE_ORDERED_ONE_LAP_CONFIRMED="NOT_CONFIRMED"
    BRAKING_STOP_AFTER_MOTION_CONFIRMED="CONFIRMED"; HOST_FREEZE_OR_KILL_USED="YES"
    COLLISION_FREE="UNKNOWN"; DEPARTURE_FREE="UNKNOWN"; DEADLINE_AND_BUDGET_COMPLIANCE="CONFIRMED"
}
$manifest = [ordered]@{
    schema="TINY_READY_LAP_REVIEW_PACKET_V1"; created_utc=[DateTime]::UtcNow.ToString("o")
    package_source_commit=$head; result_document_commit=$documentCommit; execution_commit=$execution
    source_origin=$versions.origin; branch=$versions.branch; working_tree=$dirty
    request_sha256=$requestHash; official_commit=$identity.official_commit; weight_sha256=$identity.weight_sha256
    current_attempts=@("short_219eb2f_01", "lap_219eb2f_02")
    historical_tiny_attempts=@("stationary_7b88981_01", "stationary_4d6ddd2_02", "short_4d6ddd2_03")
    result_flags=$statuses; first_terminal_lap_condition="RuntimeError: HOST_MONITOR_STALE"
    underlying_arm_fault_cause="UNKNOWN"; remaining=$check.remaining; final_used=$check.final_used
    weights_included=$false; full_sensor_included=$false; video_included=$false
    independent_review_completed=$false; new_driving_authorized_by_packet=$false
    manifest_self_excluded=$true; zip_hash_location="external .zip.receipt.json"; files=@()
}
foreach ($file in Get-ChildItem -LiteralPath $packet -File -Recurse | Sort-Object FullName) {
    $relative = [IO.Path]::GetRelativePath($packet, $file.FullName).Replace([IO.Path]::DirectorySeparatorChar, [char]'/')
    if ($relative -match '(^|/)(\.git|datasets|checkpoint|\.ssh|\.venv|__pycache__)(/|$)' -or $relative -match '\.(pt|pth|npy|npz|mcap|db3|pkl|pickle|pyc)$') {
        throw "Forbidden artifact: $relative"
    }
    $manifest.files += [ordered]@{path=$relative; bytes=$file.Length; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
}
$manifest | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $packet "PACKAGE_MANIFEST.json") -Encoding utf8NoBOM
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($packet, $zip, [IO.Compression.CompressionLevel]::Optimal, $false)
$archive = [IO.Compression.ZipFile]::OpenRead($zip)
try {
    if ($archive.Entries.Count -ne $manifest.files.Count+1) { throw "ZIP entry count mismatch" }
    foreach ($required in @("README_REVIEW.md", "PACKAGE_MANIFEST.json", "request/independent_review_request.md")) {
        if ($null -eq $archive.GetEntry($required)) { throw "ZIP root layout missing $required" }
    }
    $sha = [Security.Cryptography.SHA256]::Create()
    try {
        foreach ($record in $manifest.files) {
            $entry = $archive.GetEntry($record.path)
            if ($null -eq $entry -or $entry.Length -ne $record.bytes) { throw "ZIP entry mismatch: $($record.path)" }
            $stream = $entry.Open()
            try { $actual = [Convert]::ToHexString($sha.ComputeHash($stream)).ToLowerInvariant() }
            finally { $stream.Dispose() }
            if ($actual -ne $record.sha256) { throw "ZIP hash mismatch: $($record.path)" }
        }
    } finally { $sha.Dispose() }
} finally { $archive.Dispose() }
$result = [ordered]@{
    zip=$zip; bytes=(Get-Item -LiteralPath $zip).Length
    sha256=(Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    file_count=$manifest.files.Count+1; verified_all_manifest_hashes_from_zip=$true
    execution_commit=$execution; result_document_commit=$documentCommit; package_source_commit=$head
    remaining_powered=0; new_driving_authorized_by_packet=$false
    local_task_bytes_including_archives_and_review_duplicates=(Get-ChildItem -LiteralPath $taskRoot -Recurse -File | Measure-Object Length -Sum).Sum
}
$result | ConvertTo-Json | Set-Content -LiteralPath "$zip.receipt.json" -Encoding utf8NoBOM
$result | ConvertTo-Json
