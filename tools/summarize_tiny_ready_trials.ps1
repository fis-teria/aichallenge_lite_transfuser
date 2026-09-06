[CmdletBinding()]
param()
$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$taskRoot = Join-Path $repository "tmp/tiny_lidar_ready_lap_20260907"
$results = @()
foreach ($name in @("short_219eb2f_01", "lap_219eb2f_02")) {
    $attempt = Join-Path $taskRoot $name
    $events = @(Get-Content -LiteralPath (Join-Path $attempt "tiny_supervisor.jsonl") | ForEach-Object { $_ | ConvertFrom-Json })
    $workers = @(Get-Content -LiteralPath (Join-Path $attempt "tiny_worker.jsonl") | ForEach-Object { $_ | ConvertFrom-Json })
    $hostResult = Get-Content -LiteralPath (Join-Path $attempt "host_summary.json") -Raw | ConvertFrom-Json
    $summary = Get-Content -LiteralPath (Join-Path $attempt "tiny_supervisor_summary.json") -Raw | ConvertFrom-Json
    $workerSummary = Get-Content -LiteralPath (Join-Path $attempt "tiny_worker_summary.json") -Raw | ConvertFrom-Json
    $config = Get-Content -LiteralPath (Join-Path $attempt "resolved_config.json") -Raw | ConvertFrom-Json
    $velocities = @($events | Where-Object event -eq "VELOCITY_RECEIVED")
    $positive = @($events | Where-Object { $_.event -eq "COMMAND_REQUEST" -and $_.acceleration_mps2 -gt 0 })
    $sent = @($events | Where-Object event -eq "COMMAND_SENT_NOT_APPLIED_ACK")
    $tinySent = @($sent | Where-Object source -eq "TINY_STEERING_WITH_SPEED_LIMITER")
    $workerByIndex = @{}; foreach ($w in $workers) { $workerByIndex[[string]$w.tiny_forward_index] = $w }
    $velocityByStamp = @{}; foreach ($v in $velocities) { $velocityByStamp[[string]$v.source_ns] = $v }
    $fenceFailures = @($positive | Where-Object {
        $_.output_received_ns -le $_.ready_received_ns -or $_.output_scan_sequence -le $_.ready_scan_sequence -or $_.output_source_ns -lt $_.ready_sim_ns
    })
    $joinFailures = @($tinySent | Where-Object {
        $w = $workerByIndex[[string]$_.tiny_forward_index]
        $null -eq $w -or $w.input_id -ne $_.input_id -or $w.received_ns -ne $_.output_received_ns -or
        $w.scan_sequence -ne $_.output_scan_sequence -or $w.steering_rad -ne $_.steering_rad
    })
    $movingSent = @($tinySent | Where-Object {
        $v = $velocityByStamp[[string]$_.velocity_source_ns]
        $null -ne $v -and $v.speed_mps -ge .1
    })
    $integral = 0.0
    for ($i=1; $i -lt $velocities.Count; $i++) {
        $dt = ($velocities[$i].source_ns-$velocities[$i-1].source_ns)/1e9
        if ($dt -le 0) { throw "Regressing velocity stamp" }
        $integral += ($velocities[$i].speed_mps+$velocities[$i-1].speed_mps)*.5*$dt
    }
    $band = @()
    for ($i=$velocities.Count-1; $i -ge 0; $i--) {
        $v = $velocities[$i]
        if (-not $v.stopping -or [Math]::Abs($v.speed_mps) -gt .03) { break }
        $band = @($v)+$band
    }
    $stopBegin = @($events | Where-Object event -eq "STOP_BEGIN")[0]
    $negative = @($sent | Where-Object { $_.source -eq "INDEPENDENT_SUPERVISOR_BRAKE" -and $_.acceleration_mps2 -lt 0 })
    $steadyDuration = if ($band.Count -gt 1) { ($band[-1].source_ns-$band[0].source_ns)/1e9 } else { 0.0 }
    $processingMs = @($workers | ForEach-Object { ($_.finished_ns-$_.started_ns)/1e6 })
    $results += [ordered]@{
        attempt = $name
        execution_commit = $summary.source_commit
        host_wall_s = $hostResult.wall_s
        started_utc = [DateTimeOffset]::FromUnixTimeMilliseconds([long]($hostResult.started_unix_ns/1e6)).ToString("o")
        ended_utc = [DateTimeOffset]::FromUnixTimeMilliseconds([long]($hostResult.finished_unix_ns/1e6)).ToString("o")
        reserved_runtime_wall_s = $config.wall_seconds
        reserved_sim_s = $config.single_episode_sim_limit_s
        reserved_tiny_forwards = $config.forward_limit
        tiny_forwards = $workerSummary.tiny_forward_calls
        command_sent = $sent.Count
        tiny_sent = $tinySent.Count
        distinct_tiny_scan_sent = @($tinySent.input_id | Sort-Object -Unique).Count
        distinct_tiny_scan_sent_while_observed_moving = @($movingSent.input_id | Sort-Object -Unique).Count
        positive_requests = $positive.Count
        ready_fence_violation_count = $fenceFailures.Count
        worker_to_command_join_failure_count = $joinFailures.Count
        first_positive_request = $positive[0]
        last_positive_request = $positive[-1]
        max_abs_speed_mps = $summary.max_abs_speed_mps
        speed_integral_m_NOT_POSITION_OR_LAP = $integral
        tiny_process_ms_min = ($processingMs | Measure-Object -Minimum).Minimum
        tiny_process_ms_max = ($processingMs | Measure-Object -Maximum).Maximum
        powered_episodes = $summary.powered_episode_count
        powered_sim_s = $summary.powered_sim_seconds
        stop_reason = $summary.stop_reason
        stop_begin_sim_ns = $stopBegin.sim_ns
        stop_begin_monotonic_ns = $stopBegin.monotonic_ns
        negative_brake_sends = $negative.Count
        first_negative_brake = $negative[0]
        final_stop_band_samples = $band.Count
        final_stop_band_duration_sim_s = $steadyDuration
        final_stop_band_first_source_ns = $band[0].source_ns
        final_stop_band_last_source_ns = $band[-1].source_ns
        braking_stop_after_motion_confirmed = $summary.observed_braking_stop -and $negative.Count -gt 0 -and $steadyDuration -ge .5
        host_freeze_used = $hostResult.host_pause_verified
        unpause_called = $hostResult.cleanup.unpause_called
        host_cleanup_errors = $hostResult.cleanup.errors
        runtime_exitcode = $hostResult.runtime_exitcode
        host_error = $hostResult.error
        reported_collision_message_count = @($events | Where-Object event -eq "COLLISION_MESSAGE").Count
        collision_free = "UNKNOWN"
        departure_free = "UNKNOWN"
        judge_section_count = $hostResult.judge_section_events.Count
        judge_lap_count = $hostResult.judge_laps.Count
        judge_ordered_one_lap_confirmed = $hostResult.judge_lap_confirmed
        awsim_assets_unchanged = $hostResult.simulator_assets_unchanged
    }
}
$finalBudget = Get-Content -LiteralPath (Join-Path $taskRoot "lap_219eb2f_02/budget_after.json") -Raw | ConvertFrom-Json
$aggregate = [ordered]@{
    schema = "TINY_READY_TRIAL_SAVED_LOG_CHECK_V1"
    method = "Read saved JSONL only; no inference, sensor access, or ROS; raw logs remain authoritative"
    attempts = $results
    final_used = $finalBudget.used
    remaining = [ordered]@{
        powered = 3-$finalBudget.used.powered
        powered_sim_s = 300-$finalBudget.used.powered_s
        shared_forward = 6000-$finalBudget.used.forward-$finalBudget.used.tiny_forward
        host_wall_s = 3600-$finalBudget.used.wall_s
        log_bytes = 536870912-$finalBudget.used.log_bytes
        snapshots = 16-$finalBudget.used.snapshots
    }
}
$aggregate | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $taskRoot "evidence/saved_log_check.json") -Encoding utf8NoBOM
$aggregate | ConvertTo-Json -Depth 12
