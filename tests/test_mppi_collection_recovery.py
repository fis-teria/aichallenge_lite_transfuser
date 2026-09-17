"""Collection stops must not become reverse commands; physical stuck remains."""
import math
from pathlib import Path
import sys

import pytest

VENDOR = Path(__file__).resolve().parents[1] / "integrations/mppi_v45"
sys.path.insert(0, str(VENDOR / "support"))
sys.path.insert(0, str(VENDOR / "source/mppi_recovery_controller"))
from mppi_recovery_controller.collection_intent import fresh_forward_speed_mps
from mppi_recovery_controller.core import Ego, RecoveryAdapter, RecoveryConfig, ReferenceGeometry, ReferencePoint


def tick(adapter: RecoveryAdapter, time_s: float, speed_mps: float | None):
    reference = ReferenceGeometry([
        ReferencePoint(float(x), 0., 0., 5./3.6, -6., 6.) for x in range(-20, 31)], closed=False)
    result = adapter.prepare(now_sec=time_s, ego=Ego(0., 0., 0., 0.), reference=reference,
                             commanded_forward_speed_mps=speed_mps)
    adapter.commit(result)
    return result


@pytest.mark.parametrize("speed", [0., .1, .2, math.nan])
def test_planned_wait_never_starts_reverse(speed: float) -> None:
    adapter = RecoveryAdapter(RecoveryConfig(), drive_gear=2, reverse_gear=20)
    for t in [0., 2., 4.1, 8., 20.]:
        result = tick(adapter, t, speed)
        assert result.update.after.state == "MONITORING"
        assert not result.active
        assert result.execution.gear_command is None


@pytest.mark.parametrize("speed", [None, 5./3.6])
def test_real_forward_stuck_and_legacy_policy_still_start_recovery(speed: float | None) -> None:
    adapter = RecoveryAdapter(RecoveryConfig(), drive_gear=2, reverse_gear=20)
    assert not tick(adapter, 0., speed).active
    assert not tick(adapter, 3.99, speed).active
    result = tick(adapter, 4.01, speed)
    assert result.update.after.state == "BACKING_UP"
    assert result.execution.gear_command == 20
    # The normal planner is paused during recovery: absence must not cancel it.
    assert tick(adapter, 4.11, 0.).update.after.state == "BACKING_UP"


def test_planned_stop_resets_pending_stuck_timer() -> None:
    adapter = RecoveryAdapter(RecoveryConfig(), drive_gear=2, reverse_gear=20)
    assert not tick(adapter, 0., 5./3.6).active
    assert not tick(adapter, 3.9, 0.).active
    assert not tick(adapter, 4.1, 5./3.6).active
    assert not tick(adapter, 7.9, 5./3.6).active
    assert tick(adapter, 8.2, 5./3.6).active


@pytest.mark.parametrize("stamp,speed,expected", [
    (9.9, 5./3.6, 5./3.6), (9.9, 0., 0.), (9.7, 5./3.6, 0.),
    (10.1, 5./3.6, 0.), (None, 5./3.6, 0.), (9.9, None, 0.),
    (9.9, math.nan, 0.), (9.9, math.inf, 0.), (math.nan, 1., 0.), (9.9, -1., 0.)])
def test_command_freshness_and_units(stamp, speed, expected) -> None:
    assert fresh_forward_speed_mps(now_sec=10., command_sec=stamp, speed_mps=speed,
                                   timeout_sec=.25) == expected


@pytest.mark.parametrize("timeout", [0., -1., math.nan])
def test_invalid_timeout_is_reported(timeout: float) -> None:
    with pytest.raises(ValueError, match="seconds"):
        fresh_forward_speed_mps(now_sec=1., command_sec=1., speed_mps=1., timeout_sec=timeout)
