from mppi_recovery_controller.real_start import RealVehicleRecoveryGate


def gate():
    return RealVehicleRecoveryGate(1.0, 0.2, 250)


def test_manual_transport_cannot_arm_recovery():
    state = gate()
    for autonomous in (None, False):
        state.set_autonomous(autonomous, 100)
        state.observe_gear(True, 110)
        state.forwarded_command(3.0, 120)
        assert not state.observe_speed(2.0, 130, 140)
        assert not state.can_drive and not state.armed


def test_autonomous_and_drive_without_movement_do_not_arm():
    state = gate()
    state.set_autonomous(True, 100)
    state.observe_gear(True, 99)
    assert not state.can_drive
    state.observe_gear(False, 110)
    assert not state.can_drive
    state.observe_gear(True, 120)
    assert state.can_drive
    for stamp in range(130, 2000, 100):
        state.forwarded_command(2.0, stamp)
        assert not state.observe_speed(0.0, stamp + 1, stamp + 2)
    assert not state.armed


def test_motion_must_follow_a_fresh_forwarded_forward_command():
    state = gate()
    state.set_autonomous(True, 100)
    state.observe_gear(True, 110)
    assert not state.observe_speed(1.0, 120, 120)
    state.forwarded_command(2.0, 130)
    assert not state.observe_speed(1.0, 129, 140)
    assert not state.observe_speed(-1.0, 140, 140)
    assert not state.observe_speed(1.0, 140, 500)
    assert not state.observe_speed(1.0, 500, 500)
    state.forwarded_command(2.0, 510)
    assert state.observe_speed(0.21, 520, 520)
    assert state.armed


def test_stop_command_cancels_pending_motion_evidence():
    state = gate()
    state.set_autonomous(True, 100)
    state.observe_gear(True, 110)
    state.forwarded_command(2.0, 120)
    state.forwarded_command(0.0, 130)
    assert not state.observe_speed(1.0, 140, 140)


def test_manual_return_requires_drive_ack_and_new_motion_again():
    state = gate()
    state.set_autonomous(True, 100)
    state.observe_gear(True, 110)
    state.forwarded_command(2.0, 120)
    assert state.observe_speed(1.0, 130, 140)
    assert not state.set_autonomous(True, 150)
    assert state.armed
    state.set_autonomous(False, 200)
    assert not state.armed and not state.can_drive
    state.set_autonomous(True, 300)
    state.observe_gear(True, 250)
    assert not state.can_drive
    state.observe_gear(True, 310)
    assert not state.observe_speed(1.0, 320, 320)
    state.forwarded_command(2.0, 330)
    assert state.observe_speed(1.0, 340, 340)
