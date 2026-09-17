import math

import pytest

from mppi_recovery_controller.propulsion_guard import PropulsionGuard, PropulsionGuardConfig


def update(guard, t, direction=1, x=0.0, y=0.0, propelling=True):
    return guard.update(t, direction, x, y, 0.0, propelling)


@pytest.mark.parametrize('direction', [1, -1])
def test_stationary_start_cuts_at_deadline_in_both_gears(direction):
    guard = PropulsionGuard()
    assert not update(guard, 0.0, direction)
    assert not update(guard, 0.399, direction)
    assert update(guard, 0.4, direction)
    assert guard.blocked_directions == {direction}


def test_same_direction_does_not_reopen_after_zero_command_or_gear_toggle():
    guard = PropulsionGuard()
    update(guard, 0.0)
    assert update(guard, 0.4)
    update(guard, 1.0, propelling=False)
    update(guard, 1.1, direction=0, propelling=False)
    assert update(guard, 5.0)
    guard.reset()
    assert not update(guard, 5.1)


def test_short_throttle_interruptions_do_not_erase_loaded_time():
    guard = PropulsionGuard()
    update(guard, 0.0)
    update(guard, 0.2, propelling=False)
    update(guard, 0.3)
    assert not update(guard, 0.49)
    assert update(guard, 0.5)


def test_new_reverse_direction_can_escape_and_restore_drive():
    guard = PropulsionGuard()
    update(guard, 0.0)
    assert update(guard, 0.4)
    assert not update(guard, 0.5, direction=-1)
    assert not update(guard, 0.6, direction=-1, x=-0.03)
    assert not guard.blocked_directions
    assert not update(guard, 0.7, direction=1, x=-0.03)


def test_both_directions_stuck_do_not_cycle_into_repeated_propulsion():
    guard = PropulsionGuard()
    update(guard, 0.0)
    update(guard, 0.4)
    update(guard, 0.5, direction=-1)
    assert update(guard, 0.9, direction=-1)
    assert update(guard, 1.0, direction=1)
    assert update(guard, 2.0, direction=-1)


def test_genuine_slow_progress_does_not_cut_but_position_jitter_does():
    moving, stalled = PropulsionGuard(), PropulsionGuard()
    for i in range(81):
        t = i * 0.025
        assert not update(moving, t, x=0.1 * t)
        cut = update(stalled, t, x=0.003 * math.sin(i))
    assert cut


def test_opposite_motion_does_not_count_as_progress_in_requested_direction():
    guard = PropulsionGuard()
    update(guard, 0.0)
    assert update(guard, 0.4, x=-0.1)


def test_duplicate_updates_do_not_accumulate_time_and_clock_reset_keeps_latch():
    guard = PropulsionGuard()
    for _ in range(30):
        assert not update(guard, 1.0)
    assert update(guard, 1.4)
    assert update(guard, 0.0)


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf])
def test_invalid_cutoff_configuration_is_rejected(value):
    with pytest.raises(ValueError):
        PropulsionGuardConfig(no_progress_timeout_sec=value)
