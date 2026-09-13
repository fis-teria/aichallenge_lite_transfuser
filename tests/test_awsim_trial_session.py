import json
from pathlib import Path

import pytest

from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.runtime.awsim_trial_session import (
    JudgeLog, LowSpeedStall, requested_stop, trial_brake_reason, trial_duration_limits, encode_scan_values,
)


def test_lap_duration_is_explicit_and_short_trial_stays_short():
    assert trial_duration_limits() == (10., 30., 120.)
    assert trial_duration_limits("one_lap") == (600., 600., 720.)
    assert trial_brake_reason("bounded_10s", 10., 10.) == "SCHEDULED_BRAKE"
    assert trial_brake_reason("one_lap", 10., 10.) is None
    assert trial_brake_reason("one_lap", 599.9, 599.9) is None
    assert trial_brake_reason("one_lap", 600., 200.) == "SCHEDULED_BRAKE"
    assert trial_brake_reason("one_lap", 20., 600.) == "SCHEDULED_BRAKE"
    assert trial_brake_reason("one_lap", 285., 290., True) == "REQUESTED_BRAKE"
    with pytest.raises(ValueError):
        trial_brake_reason("one_lap", float("nan"), 1.)
    config = json.loads((Path(__file__).resolve().parents[1]/"configs/control/time_path_one_lap_5kmh_20260913.json").read_text())
    assert validate_trial_config(config) == "fixed_5kmh"
    with pytest.raises(ValueError, match="MISMATCH"):
        validate_trial_config({**config, "outer_limit_wall_s": 7200.})


def test_ordered_judge_and_stop_request_identity():
    judge = JudgeLog("lap06")
    for i, (a, b) in enumerate([(-1, 0), (0, 1), (1, 2), (2, 0)]):
        judge.feed(f"Section line hit: current={a}, next={b}, started={a != -1}", i*100)
    assert not judge.completed
    judge.feed("Lap completed: 285.00s, total laps: 1", 500)
    assert judge.completed and judge.laps[0]["byte_offset"] == 500
    assert requested_stop({"run_id":"lap06", "reason":"JUDGE_FIRST_LAP"}, "lap06") == "JUDGE_FIRST_LAP"
    for value in ({"run_id":"old_run","reason":"JUDGE_FIRST_LAP"}, {"run_id":"lap06","reason":"DRIVE"}):
        with pytest.raises(ValueError, match="IDENTITY"):
            requested_stop(value,"lap06")


def test_stall_uses_sim_time_and_resets_after_motion_or_clock_reset():
    stall = LowSpeedStall()
    assert not stall.update(1_000_000_000, 0.)
    assert not stall.update(5_999_999_999, .01)
    assert stall.update(6_000_000_000, .01)
    assert not stall.update(7_000_000_000, 1.2)
    assert not stall.update(8_000_000_000, 0.)
    assert not stall.update(2_000_000_000, 0.)
    assert stall.update(7_000_000_000, 0.)


def test_scan_diagnostic_roundtrip_preserves_finite_and_nonfinite_values():
    import math
    from aic_transfuser_lite.control.long_sim_tracking_v4 import check_scan
    values = [.1, 1.2345, float('inf'), -float('inf'), float('nan')]
    restored = [float(v) for v in json.loads(json.dumps(encode_scan_values(values), allow_nan=False))]
    assert restored[:4] == values[:4] and math.isnan(restored[4])
    ranges = [float('inf')]*750; ranges[375] = 2.1
    encoded = json.loads(json.dumps(encode_scan_values(ranges), allow_nan=False))
    for scan in (ranges, [float(v) for v in encoded]):
        with pytest.raises(ValueError, match='STOPPING_CORRIDOR_OCCUPIED'):
            check_scan(scan, -math.pi, 2*math.pi/750, .1, 25., 5/3.6)
