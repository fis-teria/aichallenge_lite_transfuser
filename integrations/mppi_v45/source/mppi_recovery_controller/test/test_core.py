import ast
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from mppi_recovery_controller.core import (
    Ego, RecoveryAdapter, RecoveryConfig, ReferenceGeometry, ReferencePoint,
)
from mppi_recovery_controller.geometry import build_reference


def reference(lower=-6, upper=6, speed=10):
    return ReferenceGeometry([ReferencePoint(x, 0, 0, speed, lower, upper)
                              for x in range(-20, 31)], closed=False)


def tick(adapter, t, ego=Ego(0, 0, 0, 0), ref=None, **kwargs):
    proposal = adapter.prepare(now_sec=t, ego=ego, reference=ref or reference(), **kwargs)
    adapter.commit(proposal)
    return proposal


def adapter(config=RecoveryConfig()):
    return RecoveryAdapter(config, drive_gear=2, reverse_gear=20)


def test_defaults_and_config_values_match_current_mpc():
    submit = Path(__file__).resolve().parents[2]
    source = submit / "multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py"
    tree = ast.parse(source.read_text())
    controller = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MPCController")
    constants = {}
    for statement in controller.body:
        if isinstance(statement, ast.Assign) and isinstance(statement.value, ast.Constant):
            for target in statement.targets:
                if isinstance(target, ast.Name):
                    constants[target.id] = statement.value.value
    mapping = {
        "stop_speed_threshold": "RECOVERY_STOP_SPEED_MPS",
        "min_forward_command_speed": "RECOVERY_MIN_FORWARD_COMMAND_MPS",
        "stuck_detection_sec": "RECOVERY_STUCK_DETECTION_SEC",
        "reverse_stop_speed_threshold": "RECOVERY_REVERSE_STOP_SPEED_MPS",
        "reverse_stop_hold_sec": "RECOVERY_REVERSE_STOP_HOLD_SEC",
        "grace_sec": "RECOVERY_GRACE_SEC",
        "backing_up_stuck_hold_sec": "RECOVERY_BACKING_UP_STUCK_HOLD_SEC",
        "aligning_forward_blocked_hold_sec": "RECOVERY_ALIGNING_FORWARD_BLOCKED_HOLD_SEC",
        "aligning_forward_stuck_hold_sec": "RECOVERY_ALIGNING_FORWARD_STUCK_HOLD_SEC",
        "aligning_forward_timeout_sec": "RECOVERY_ALIGNING_FORWARD_TIMEOUT_SEC",
        "aligning_forward_min_progress_m": "RECOVERY_ALIGNING_FORWARD_MIN_PROGRESS_M",
        "normal_cycle_limit": "RECOVERY_NORMAL_CYCLE_LIMIT",
        "cycle_reset_sec": "RECOVERY_CYCLE_RESET_SEC",
        "escape_turn_sec": "RECOVERY_ESCAPE_TURN_SEC",
        "forward_speed_mps": "RECOVERY_FORWARD_ESCAPE_SPEED_MPS",
        "forward_steer_deg": "RECOVERY_FORWARD_ESCAPE_STEER_DEG",
        "forward_lookahead_wp": "RECOVERY_FORWARD_LOOKAHEAD_WP",
        "heading_steer_gain": "RECOVERY_HEADING_STEER_GAIN",
        "vehicle_gap_m": "RECOVERY_CLEAR_CHECK_VEHICLE_GAP_M",
        "wall_margin_m": "RECOVERY_CLEAR_CHECK_WALL_MARGIN_M",
        "probe_distance_m": "RECOVERY_CLEAR_CHECK_PROBE_DISTANCE_M",
        "speed_gain": "KP",
    }
    cfg = RecoveryConfig()
    for field, constant in mapping.items():
        assert getattr(cfg, field) == constants[constant], (field, constant)
    raw = yaml.safe_load((submit / "multi_purpose_mpc_ros/config/config.yaml").read_text())
    loaded = RecoveryConfig.from_mpc_config(raw)
    assert loaded.backing_up_timeout_sec == 3.0
    assert loaded.reverse_speed_mps == -2.5
    assert loaded.steering_publish_gain == raw["mpc"]["steering_tire_angle_gain_var"]
    assert loaded.vehicle_width_m == raw["bicycle_model"]["width"]


def test_zero_selected_speed_does_not_mask_base_forward_intent():
    a = adapter()
    assert not tick(a, 0).active
    assert not tick(a, 3.99).active
    result = tick(a, 4.01)
    assert result.update.after.state == "BACKING_UP"
    assert result.execution.gear_command == 20
    assert result.decision.published_speed_mps == 2.5
    assert result.decision.published_acceleration_mps2 == pytest.approx(1.3)
    assert result.decision.steering_rad == 0.0


@pytest.mark.parametrize("speed,expected", [
    (0.0, 1.3), (-0.5, 1.3), (-2.49, 1.0), (-2.5, 0.0), (-2.6, -0.999),
])
def test_reverse_propulsion_cap_preserves_lower_demand_and_braking(speed, expected):
    a = adapter()
    tick(a, 0)
    tick(a, 4.01)
    result = tick(a, 4.1, Ego(-0.1, 0, 0, speed))
    assert result.update.after.state == "BACKING_UP"
    assert result.decision.publish_in_reverse_gear
    assert result.decision.published_acceleration_mps2 == pytest.approx(expected)


@pytest.mark.parametrize("ego,allowed,speed", [
    (Ego(0, 0, 0, 1), True, 10),
    (Ego(0, 0, 0, 0), False, 10),
    (Ego(0, 0, 0, 0), True, 0),
])
def test_moving_race_disabled_and_intended_stop_do_not_start_recovery(ego, allowed, speed):
    a = adapter()
    for t in (0, 5, 10):
        result = tick(a, t, ego, reference(speed=speed), recovery_allowed=allowed)
        assert not result.active
        assert result.decision is None


def test_reverse_stop_holds_gear_and_brakes_then_hands_back_in_drive():
    a = adapter()
    tick(a, 0)
    tick(a, 4.01)
    result = tick(a, 7.02, Ego(-2, 0, 0, -1))
    assert result.update.after.state == "STOPPING_TO_FORWARD"
    assert result.execution.gear_command is None
    assert result.execution.after.active_gear == 20
    assert result.decision.published_speed_mps == 0
    assert result.decision.published_acceleration_mps2 == pytest.approx(-0.999)
    tick(a, 7.1, Ego(-2, 0, 0, 0))
    assert tick(a, 7.29, Ego(-2, 0, 0, 0)).active
    result = tick(a, 7.31, Ego(-2, 0, 0, 0))
    assert not result.active
    assert result.execution.gear_command == 2


def test_pose_not_ready_uses_mpc_reference_alignment_and_progress_exit():
    a = adapter()
    ego = Ego(0, 2, 0, 0)
    tick(a, 0, ego)
    tick(a, 4.01, ego)
    tick(a, 7.02, ego)
    tick(a, 7.1, ego)
    result = tick(a, 7.31, ego)
    assert result.update.after.state == "ALIGNING_FORWARD"
    assert result.execution.gear_command == 2
    assert result.decision.steering_rad < 0  # target is back toward the reference
    assert result.decision.published_speed_mps == 2.5
    assert result.decision.published_acceleration_mps2 == pytest.approx(0.999)
    tick(a, 7.9, ego)
    assert not tick(a, 9.0, ego).active  # no progress during the existing 1 s window


@pytest.mark.parametrize("yaw,expected", [(math.pi / 2, "BACKING_UP"),
                                          (-math.pi / 2, "STOPPING_TO_FORWARD"),
                                          (0, "STOPPING_TO_FORWARD")])
def test_wall_contact_allows_only_improving_reverse(yaw, expected):
    a = adapter()
    ego = Ego(0, 4, yaw, 0)
    tick(a, 0, ego)
    result = tick(a, 4.01, ego)
    assert result.update.after.state == expected
    if expected == "BACKING_UP":
        assert result.rear_wall.status == "IMPROVING_ESCAPE"
    else:
        assert result.execution.gear_command is None  # no first unsafe reverse sample


def test_close_rear_vehicle_brakes_without_waiting_for_legacy_hold():
    a = adapter()
    tick(a, 0)
    tick(a, 4.01)
    result = tick(a, 4.1, Ego(-1, 0, 0, -1), opponents=[(-3, 0)])
    assert result.update.after.state == "STOPPING_TO_FORWARD"
    assert result.short_mode
    assert result.execution.after.active_gear == 20
    assert result.decision.published_acceleration_mps2 < 0


def test_close_rear_stroke_is_slow_and_turns_to_improve_heading():
    a = adapter()
    ego = Ego(0, 0, math.radians(30), 0)
    rear = [(-2.7 * math.cos(ego.yaw), -2.7 * math.sin(ego.yaw))]
    tick(a, 0, ego, opponents=rear)
    result = tick(a, 4.01, ego, opponents=rear)
    assert result.update.after.state == "BACKING_UP"
    assert result.short_mode
    assert result.decision.published_speed_mps == 0
    assert result.decision.steering_rad > 0
    result = tick(a, 4.72, ego, opponents=rear)
    assert 0 < result.decision.published_speed_mps <= .5
    assert 0 < result.decision.published_acceleration_mps2 <= a.config.short_acceleration_mps2
    assert result.decision.steering_rad > 0  # reverse steering reduces positive yaw error
    assert result.motion.available_m < a.config.short_stroke_m


def test_no_reverse_sample_when_body_has_no_rear_clearance():
    a = adapter()
    tick(a, 0, opponents=[(-2.2, 0)])
    result = tick(a, 4.01, opponents=[(-2.2, 0)])
    assert result.update.after.state == "STOPPING_TO_FORWARD"
    assert result.execution.gear_command is None
    assert result.decision.published_speed_mps == 0


def test_slow_but_moving_short_reverse_is_not_cut_off_by_center_gap_hold():
    a = adapter()
    rear = [(-2.7, 0)]
    tick(a, 0, opponents=rear)
    tick(a, 4.01, opponents=rear)
    tick(a, 4.6, Ego(-.04, 0, 0, -.10), opponents=rear)
    result = tick(a, 5.12, Ego(-.09, 0, 0, -.10), opponents=rear)
    assert result.update.after.state == "BACKING_UP"
    assert result.motion.available_m > 0
    # The preview changes short-motion criteria without changing MPC defaults.
    assert a.recovery.stop_speed_threshold == .20


def test_short_stroke_stops_at_distance_even_when_rear_stays_clear():
    a = adapter()
    tick(a, 0, opponents=[(-3, 0)])
    tick(a, 4.01, opponents=[(-3, 0)])
    # Opponent has moved away, but the committed stroke remains bounded.
    result = tick(a, 4.4, Ego(-.58, 0, 0, -.3), opponents=[])
    assert result.short_mode
    assert result.update.after.state == "STOPPING_TO_FORWARD"
    assert result.execution.after.active_gear == 20


def test_geometric_reference_heading_ignores_csv_orientation_convention():
    origin = NS(position=NS(x=-10, y=-10), orientation=NS(x=0, y=0, z=0, w=1))
    grid = NS(info=NS(width=60, height=40, resolution=.5, origin=origin), data=[0] * 2400)
    ref = build_reference([(float(x), 0.0, math.pi/2, 10.0) for x in range(12)], grid, 6)
    assert all(p.yaw == pytest.approx(0) for p in ref.points)


def test_short_stroke_steering_reduces_road_heading_error_when_lookahead_is_behind():
    from mppi_recovery_controller.short_motion import motion_pose
    a = adapter()
    ego = Ego(0, 4, math.radians(116), 0)
    close_waypoints = ReferenceGeometry([
        ReferencePoint(.2 * x, 0, 0, 10, -6, 6) for x in range(-20, 31)], closed=False)
    rear = [(ego.x - 2.9 * math.cos(ego.yaw), ego.y - 2.9 * math.sin(ego.yaw))]
    tick(a, 0, ego, ref=close_waypoints, opponents=rear)
    result = tick(a, 4.01, ego, ref=close_waypoints, opponents=rear)
    assert result.short_mode and result.motion is not None
    yaw_after = motion_pose(ego, -.1, result.motion.tire_steer_rad, a.config.physical_wheelbase_m)[2]
    assert abs(yaw_after) < abs(ego.yaw)


@pytest.mark.parametrize("propulsion", [.5, .999])
def test_short_reverse_with_actuator_delay_stops_before_rear_body(propulsion):
    from collections import deque
    from mppi_recovery_controller.short_motion import opponent_clearance

    a = adapter()
    x, speed, dt = 0.0, 0.0, .025
    delayed_acceleration = deque([0.0] * 8)  # 0.20 s response delay
    rear = [(-2.7, 0)]
    reversing_seen = False
    minimum_clearance = math.inf
    for i in range(500):
        result = tick(a, i * dt, Ego(x, 0, 0, speed), opponents=rear)
        if result.decision is not None:
            command = result.decision.acceleration_mps2
        else:
            command = 0.0
        delayed_acceleration.append(command)
        acceleration = max(-propulsion, min(.8, delayed_acceleration.popleft()))
        if result.execution.after.active_gear == 20:
            speed = min(0.0, speed + acceleration * dt)
            reversing_seen = True
        x += speed * dt
        minimum_clearance = min(minimum_clearance, opponent_clearance(
            (x, 0, 0), rear, a.config.body_length_m, a.config.body_width_m))
        if reversing_seen and result.execution.gear_command == 2:
            assert abs(speed) <= .05
            break
    else:
        pytest.fail("short reverse did not settle and return drive authority")
    assert x < -.05
    assert minimum_clearance >= a.config.short_clearance_m


def test_repeated_failure_reaches_existing_escape_turn():
    cfg = replace(RecoveryConfig(), stuck_detection_sec=0.1, backing_up_timeout_sec=0.2,
                  reverse_stop_hold_sec=0.05, aligning_forward_timeout_sec=0.2)
    a = adapter(cfg)
    states = []
    for i in range(240):
        result = tick(a, i * 0.025, Ego(0, 2, 0, 0))
        states.append(result.update.after.state)
        if states[-1] == "ESCAPE_TURN":
            assert result.decision.steering_rad == pytest.approx(-math.pi / 6)
            break
    assert "ESCAPE_TURN" in states


def test_disable_cancels_active_recovery_and_restores_drive():
    a = adapter()
    tick(a, 0)
    tick(a, 4.01)
    result = tick(a, 4.2, recovery_allowed=False)
    assert not result.active
    assert result.execution.gear_command == 2
    assert result.decision.published_speed_mps == 0


def test_proposal_does_not_advance_state_before_publication_commit():
    a = adapter()
    tick(a, 0)
    result = a.prepare(now_sec=4.01, ego=Ego(0, 0, 0, 0), reference=reference())
    assert result.active
    assert not a.active
    a.commit(result)
    assert a.active


def test_physical_map_bounds_include_translated_rotated_origin():
    # A map corridor rotated +90 degrees is vertical in world coordinates.
    origin = NS(position=NS(x=20.0, y=-10.0),
                orientation=NS(x=0, y=0, z=math.sqrt(0.5), w=math.sqrt(0.5)))
    data = [100 if row in (0, 19) else 0 for row in range(20) for col in range(40)]
    grid = NS(info=NS(width=40, height=20, resolution=0.5, origin=origin), data=data)
    samples = [(15.0, float(y), math.pi / 2, 10.0) for y in range(-8, 8)]
    geometry = build_reference(samples, grid, 6.0)
    assert geometry.points[5].lower == pytest.approx(-4.5, abs=0.26)
    assert geometry.points[5].upper == pytest.approx(4.5, abs=0.26)
