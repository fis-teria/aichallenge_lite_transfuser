"""ROS-free input and command adapter around the existing MPC recovery policy."""

from dataclasses import dataclass, replace
import math

from multi_purpose_mpc_ros.angle_domain import WrappedHeadingErrorRad
from multi_purpose_mpc_ros.control_authority import ControlAuthority, ControlDecision
from multi_purpose_mpc_ros.recovery_logic import (
    RECOVERY_STATE_ALIGNING_FORWARD,
    RECOVERY_STATE_BACKING_UP,
    RECOVERY_STATE_ESCAPE_TURN,
    RECOVERY_STATE_MONITORING,
    RECOVERY_STATE_STOPPING_TO_FORWARD,
    RecoveryExecutionState,
    RecoveryExecutionUpdateProposal,
    RecoveryDirectionalWallAssessment,
    StuckRecovery,
    StuckRecoveryUpdateProposal,
    assess_recovery_directional_wall_clearance,
    normalize_angle_rad,
    propose_recovery_execution_update,
    propose_stuck_recovery_update,
    select_recovery_escape_steer_sign,
    select_recovery_forward_steer_rad,
    select_recovery_intended_speed_mps,
)
from multi_purpose_mpc_ros.speed_domain import BaseReferenceSpeed, ConstrainedSpeed
from .short_motion import MotionBudget, body_samples, motion_budget


@dataclass(frozen=True)
class RecoveryConfig:
    # These defaults match MPCController's recovery constants. Configurable
    # values below are loaded from its existing config.yaml by the ROS adapter.
    stop_speed_threshold: float = 0.20
    min_forward_command_speed: float = 1.0
    stuck_detection_sec: float = 4.0
    reverse_stop_speed_threshold: float = 0.05
    reverse_stop_hold_sec: float = 0.20
    grace_sec: float = 0.5
    backing_up_stuck_hold_sec: float = 0.5
    aligning_forward_blocked_hold_sec: float = 0.5
    aligning_forward_stuck_hold_sec: float = 1.0
    aligning_forward_timeout_sec: float = 4.0
    aligning_forward_min_progress_m: float = 0.2
    normal_cycle_limit: int = 3
    cycle_reset_sec: float = 8.0
    escape_turn_sec: float = 2.0
    forward_speed_mps: float = 2.5
    forward_steer_deg: float = 30.0
    forward_lookahead_wp: int = 5
    heading_steer_gain: float = 0.5
    vehicle_gap_m: float = 3.0
    wall_margin_m: float = 0.5
    probe_distance_m: float = 2.0
    speed_gain: float = 100.0
    enabled: bool = True
    backing_up_timeout_sec: float = 3.0
    reverse_speed_mps: float = -2.5
    handback_heading_threshold_deg: float = 30.0
    handback_lateral_threshold_m: float = 1.0
    vehicle_width_m: float = 2.30
    opponent_radius_m: float = 0.5
    max_width_m: float = 6.0
    acceleration_min_mps2: float = -10.0
    acceleration_max_mps2: float = 0.999
    steering_publish_gain: float = 1.639
    body_length_m: float = 2.064
    body_width_m: float = 1.30
    physical_wheelbase_m: float = 1.087
    tire_to_command_ratio: float = 0.60
    maximum_tire_angle_rad: float = math.pi / 10
    short_speed_mps: float = 0.5
    short_speed_gain: float = 10.0
    short_acceleration_mps2: float = 0.999
    short_min_progress_m: float = 0.02
    short_stroke_m: float = 0.6
    short_clearance_m: float = 0.05
    short_response_delay_sec: float = 0.20
    short_braking_deceleration_mps2: float = 0.8
    short_path_step_m: float = 0.05
    short_steering_settle_sec: float = 0.70
    maximum_recovery_propulsion_mps2: float = 0.0
    maximum_reverse_propulsion_mps2: float = 1.3
    require_steering_feedback: bool = False
    steering_settle_tolerance_rad: float = 0.03

    @classmethod
    def from_mpc_config(cls, config):
        recovery = config["stuck_recovery"]
        return cls(
            **{key: recovery[key] for key in (
                "enabled", "backing_up_timeout_sec", "reverse_speed_mps",
                "handback_heading_threshold_deg", "handback_lateral_threshold_m",
            )},
            vehicle_width_m=float(config["bicycle_model"]["width"]),
            opponent_radius_m=float(config["v2x_obstacle_avoidance"]["vehicle_radius"]),
            max_width_m=float(config["reference_path"]["max_width"]),
            acceleration_min_mps2=float(config["mpc"]["a_min"]),
            acceleration_max_mps2=float(config["mpc"]["a_max"]),
            steering_publish_gain=float(config["mpc"]["steering_tire_angle_gain_var"]),
            body_length_m=float(config["v2x_obstacle_avoidance"]["vehicle_body_length_m"]),
            body_width_m=float(config["v2x_obstacle_avoidance"]["vehicle_body_width_m"]),
        )

    def make_recovery(self):
        return StuckRecovery(**{
            name: getattr(self, name) for name in (
                "stop_speed_threshold", "min_forward_command_speed",
                "stuck_detection_sec", "reverse_stop_speed_threshold",
                "reverse_stop_hold_sec", "grace_sec", "backing_up_stuck_hold_sec",
                "backing_up_timeout_sec", "aligning_forward_blocked_hold_sec",
                "aligning_forward_stuck_hold_sec", "aligning_forward_timeout_sec",
                "aligning_forward_min_progress_m", "normal_cycle_limit",
                "cycle_reset_sec", "escape_turn_sec",
            )
        })


@dataclass(frozen=True)
class Ego:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass(frozen=True)
class ReferencePoint:
    x: float
    y: float
    yaw: float
    speed: float
    lower: float
    upper: float


class ReferenceGeometry:
    def __init__(self, points, *, closed=True, is_free=None):
        self.points = tuple(points)
        self.closed = closed
        self.is_free = is_free

    def nearest(self, x, y):
        return min(range(len(self.points)), key=lambda i:
                   (self.points[i].x - x) ** 2 + (self.points[i].y - y) ** 2)

    def ahead(self, index, offset):
        index += offset
        return self.points[index % len(self.points) if self.closed
                           else min(index, len(self.points) - 1)]

    def occupied_body_samples(self, pose, footprint):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        occupied = 0
        for bx, by in footprint:
            px, py = x + c * bx - s * by, y + s * bx + c * by
            if self.is_free is not None:
                occupied += not self.is_free(px, py)
            else:
                point = self.points[self.nearest(px, py)]
                occupied += not point.lower <= lateral_offset(point, px, py) <= point.upper
        return occupied


def lateral_offset(point, x, y):
    return -math.sin(point.yaw) * (x - point.x) + math.cos(point.yaw) * (y - point.y)


@dataclass(frozen=True)
class RecoveryTick:
    update: StuckRecoveryUpdateProposal
    execution: RecoveryExecutionUpdateProposal
    decision: ControlDecision | None
    rear_wall: RecoveryDirectionalWallAssessment
    short_mode: bool = False
    cycle_heading_error: float = 0.0
    cycle_lateral_abs: float = 0.0
    motion: MotionBudget | None = None

    @property
    def active(self):
        return self.update.after.state != RECOVERY_STATE_MONITORING


class RecoveryAdapter:
    def __init__(self, config, *, drive_gear, reverse_gear):
        self.config = config
        self.recovery = config.make_recovery()
        self.execution = RecoveryExecutionState(initial_gear=drive_gear)
        self.drive_gear = drive_gear
        self.reverse_gear = reverse_gear
        self.short_mode = False
        self.cycle_heading_error = self.cycle_lateral_abs = 0.0
        self.footprint = body_samples(config.body_length_m, config.body_width_m)

    @property
    def active(self):
        return self.recovery.state != RECOVERY_STATE_MONITORING

    def _bounds(self, reference, x, y):
        point = reference.points[reference.nearest(x, y)]
        margin = self.config.vehicle_width_m / math.sqrt(2.0)
        return lateral_offset(point, x, y), point.lower + margin, point.upper - margin

    def direction_clear(self, ego, heading, reference, opponents):
        cfg = self.config
        c, s = math.cos(heading), math.sin(heading)
        for x, y in opponents:
            dx, dy = x - ego.x, y - ego.y
            if (0.0 < dx * c + dy * s <= cfg.vehicle_gap_m
                    and abs(-dx * s + dy * c)
                    <= cfg.vehicle_width_m / 2.0 + cfg.opponent_radius_m):
                return False
        point = reference.points[reference.nearest(
            ego.x + cfg.probe_distance_m * c, ego.y + cfg.probe_distance_m * s)]
        return point.upper - point.lower >= cfg.vehicle_width_m + cfg.wall_margin_m

    def prepare(self, *, now_sec, ego, reference, opponents=(), recovery_allowed=True,
                measured_tire_angle_rad=None):
        cfg = self.config
        previous = self.recovery.state
        index = reference.nearest(ego.x, ego.y)
        point = reference.points[index]
        current = self._bounds(reference, ego.x, ego.y)
        probe = self._bounds(reference,
                             ego.x - cfg.probe_distance_m * math.cos(ego.yaw),
                             ego.y - cfg.probe_distance_m * math.sin(ego.yaw))
        rear_wall = assess_recovery_directional_wall_clearance(
            current_lateral_offset_m=current[0],
            current_safe_lower_bound_m=current[1],
            current_safe_upper_bound_m=current[2],
            probe_lateral_offset_m=probe[0],
            probe_safe_lower_bound_m=probe[1],
            probe_safe_upper_bound_m=probe[2],
            wall_guard_margin_m=cfg.wall_margin_m,
        )
        move_entry = self.execution.move_entry_xy
        heading_error = normalize_angle_rad(ego.yaw - point.yaw)
        rear_c, rear_s = -math.cos(ego.yaw), -math.sin(ego.yaw)
        nearby_rear = any(
            0 < (x - ego.x) * rear_c + (y - ego.y) * rear_s <= cfg.vehicle_gap_m
            and abs(-(x - ego.x) * rear_s + (y - ego.y) * rear_c)
            <= cfg.body_width_m / 2 + math.hypot(cfg.body_length_m, cfg.body_width_m) / 2
            for x, y in opponents)
        short_mode = (nearby_rear and (self.active or abs(ego.speed) <= cfg.stop_speed_threshold)) or (self.short_mode and
                                     (self.active or abs(ego.speed) <= cfg.short_speed_mps))
        if not recovery_allowed:
            short_mode = False
        target = reference.ahead(index, cfg.forward_lookahead_wp)
        target_error = normalize_angle_rad(ego.yaw - math.atan2(target.y - ego.y, target.x - ego.x))
        forward_steer = select_recovery_forward_steer_rad(
            heading_error=WrappedHeadingErrorRad(target_error),
            heading_steer_gain=cfg.heading_steer_gain,
            max_steer_rad=math.radians(cfg.forward_steer_deg))
        reverse_budget = forward_budget = None
        if short_mode:
            # Near a wall the lookahead point can lie behind the body and
            # select the opposite turn. Short strokes first align to the road.
            forward_steer = select_recovery_forward_steer_rad(
                heading_error=WrappedHeadingErrorRad(heading_error),
                heading_steer_gain=cfg.heading_steer_gain,
                max_steer_rad=math.radians(cfg.forward_steer_deg))
            tire_steer = max(-cfg.maximum_tire_angle_rad, min(cfg.maximum_tire_angle_rad,
                forward_steer * cfg.steering_publish_gain * cfg.tire_to_command_ratio))
            distance = 0.0 if move_entry is None else math.hypot(ego.x - move_entry[0], ego.y - move_entry[1])
            if previous in (RECOVERY_STATE_MONITORING, RECOVERY_STATE_BACKING_UP):
                reverse_budget = motion_budget(ego, reference, opponents, cfg, reverse=True,
                    tire_steer=-tire_steer, footprint=self.footprint,
                    stroke_remaining=cfg.short_stroke_m - (distance if previous == RECOVERY_STATE_BACKING_UP else 0))
            else:
                forward_budget = motion_budget(ego, reference, opponents, cfg, reverse=False,
                    tire_steer=tire_steer, footprint=self.footprint,
                    stroke_remaining=cfg.short_stroke_m - (distance if previous in (
                        RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN) else 0))
        intended = select_recovery_intended_speed_mps(
            base_reference_speed=BaseReferenceSpeed(point.speed),
            constrained_speed=ConstrainedSpeed(0.0),
        )
        policy = self.recovery
        if short_mode:
            # A short stroke intentionally remains below the ordinary stuck
            # speed. Keep its progress test at the scale of the planned move.
            policy = replace(policy, stop_speed_threshold=cfg.reverse_stop_speed_threshold,
                             grace_sec=cfg.grace_sec + cfg.short_steering_settle_sec,
                             aligning_forward_min_progress_m=cfg.short_min_progress_m)
        update = propose_stuck_recovery_update(
            recovery=policy, now_sec=now_sec, measured_speed=ego.speed,
            previous_target_speed=intended,
            recovery_allowed=recovery_allowed and cfg.enabled,
            distance_since_move_entry_m=(0.0 if move_entry is None else
                                         math.hypot(ego.x - move_entry[0], ego.y - move_entry[1])),
            forward_is_clear=(self.direction_clear(ego, ego.yaw, reference, opponents)
                              if previous in (RECOVERY_STATE_BACKING_UP,
                                              RECOVERY_STATE_ALIGNING_FORWARD) else False),
            rear_is_blocked=(not self.direction_clear(ego, ego.yaw + math.pi,
                                                     reference, opponents)
                             if previous == RECOVERY_STATE_BACKING_UP and not short_mode else False),
            rear_wall_is_blocked=not rear_wall.allows_reverse,
            reverse_motion_limit_reached=reverse_budget.stop if reverse_budget else False,
            forward_motion_limit_reached=forward_budget.stop if forward_budget else False,
            ready_for_handback=(
                previous == RECOVERY_STATE_STOPPING_TO_FORWARD
                and abs(heading_error) <= math.radians(cfg.handback_heading_threshold_deg)
                and abs(current[0]) <= cfg.handback_lateral_threshold_m),
        )
        state = update.after.state
        cycle_heading_error, cycle_lateral_abs = self.cycle_heading_error, self.cycle_lateral_abs
        if short_mode and previous == RECOVERY_STATE_MONITORING and state != previous:
            cycle_heading_error, cycle_lateral_abs = abs(heading_error), abs(current[0])
        if short_mode and previous in (RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN) and state == RECOVERY_STATE_MONITORING:
            if (abs(heading_error) + math.radians(2) < cycle_heading_error or
                    abs(current[0]) + 0.10 < cycle_lateral_abs):
                update = replace(update, after=replace(update.after, consecutive_recovery_count=0))
        backing_steer = escape_steer = None
        motion = None
        if short_mode and state == RECOVERY_STATE_BACKING_UP:
            motion = reverse_budget
            backing_steer = motion.tire_steer_rad / (cfg.steering_publish_gain * cfg.tire_to_command_ratio)
        if state == RECOVERY_STATE_ALIGNING_FORWARD:
            if short_mode:
                motion = forward_budget
                forward_steer = motion.tire_steer_rad / (cfg.steering_publish_gain * cfg.tire_to_command_ratio)
        elif state == RECOVERY_STATE_ESCAPE_TURN and state != previous:
            sign = select_recovery_escape_steer_sign(
                preferred_sign=update.after.escape_turn_steer_sign,
                lateral_offset_m=current[0], safe_lower_bound_m=current[1],
                safe_upper_bound_m=current[2], wall_guard_margin_m=cfg.wall_margin_m,
            )
            update = replace(update, after=replace(update.after, escape_turn_steer_sign=sign))
            escape_steer = sign * math.radians(cfg.forward_steer_deg)
        if short_mode and state == RECOVERY_STATE_ESCAPE_TURN:
            motion = forward_budget
            escape_steer = motion.tire_steer_rad / (cfg.steering_publish_gain * cfg.tire_to_command_ratio)
        execution = propose_recovery_execution_update(
            state=self.execution, current_state=state, previous_state=previous,
            ego_x_m=ego.x, ego_y_m=ego.y, drive_gear=self.drive_gear,
            reverse_gear=self.reverse_gear, aligning_forward_steer_rad=(
                forward_steer if state == RECOVERY_STATE_ALIGNING_FORWARD else None),
            escape_turn_steer_rad=escape_steer,
            backing_up_steer_rad=backing_steer,
        )
        decision = None
        if state != RECOVERY_STATE_MONITORING or state != previous:
            authority, speed, steer = ControlAuthority.RECOVERY_STOPPING, 0.0, 0.0
            if state == RECOVERY_STATE_BACKING_UP:
                authority, speed = ControlAuthority.RECOVERY_REVERSE, cfg.reverse_speed_mps
                if short_mode:
                    speed, steer = -motion.speed_limit_mps, backing_steer
            elif state in (RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN):
                authority = (ControlAuthority.RECOVERY_FORWARD if state == RECOVERY_STATE_ALIGNING_FORWARD
                             else ControlAuthority.RECOVERY_ESCAPE_TURN)
                speed, steer = cfg.forward_speed_mps, execution.after.escape_steer_rad
                if short_mode:
                    speed = motion.speed_limit_mps
            if cfg.require_steering_feedback:
                steer = max(-cfg.maximum_tire_angle_rad / (cfg.steering_publish_gain * cfg.tire_to_command_ratio),
                            min(cfg.maximum_tire_angle_rad / (cfg.steering_publish_gain * cfg.tire_to_command_ratio), steer))
                target_tire = steer * cfg.steering_publish_gain * cfg.tire_to_command_ratio
                steering_ready = (measured_tire_angle_rad is not None and math.isfinite(measured_tire_angle_rad)
                                  and abs(target_tire - measured_tire_angle_rad) <= cfg.steering_settle_tolerance_rad)
                if not steering_ready and state in (RECOVERY_STATE_BACKING_UP,
                        RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN):
                    speed = 0.0
                    # Movement timeout/progress windows start after the real
                    # tire reaches its requested angle, not while it is turning.
                    update = replace(update, after=replace(update.after, move_since=now_sec,
                        move_stuck_since=None, move_stuck_since_distance_m=None, move_blocked_since=None))
            acceleration = max(cfg.acceleration_min_mps2, min(cfg.acceleration_max_mps2,
                                                           cfg.speed_gain * (speed - ego.speed)))
            if short_mode:
                if (not cfg.require_steering_feedback and motion is not None and update.after.move_since is not None and
                        now_sec - update.after.move_since < cfg.short_steering_settle_sec):
                    # Measured full-lock reversal takes ~0.6 s in AWSIM. A
                    # short stroke must start after the tires change direction.
                    speed = 0.0
                if state == RECOVERY_STATE_STOPPING_TO_FORWARD:
                    steer = execution.after.escape_steer_rad
                gain = cfg.speed_gain if speed == 0 else cfg.short_speed_gain
                acceleration = gain * (speed - ego.speed)
                if speed < 0:
                    acceleration = max(-cfg.short_acceleration_mps2, min(cfg.acceleration_max_mps2, acceleration))
                elif speed > 0:
                    acceleration = max(cfg.acceleration_min_mps2, min(cfg.short_acceleration_mps2, acceleration))
                else:
                    acceleration = max(cfg.acceleration_min_mps2, min(cfg.acceleration_max_mps2, acceleration))
            decision = ControlDecision(
                authority=authority, internal_speed_mps=speed, steering_rad=steer,
                acceleration_mps2=acceleration,
                publish_in_reverse_gear=execution.after.active_gear == self.reverse_gear,
                bug_acc_enabled=False,
            )
            # The signed braking bound becomes propulsion after reverse-gear
            # conversion, so limit the published reverse propulsion separately.
            if (decision.publish_in_reverse_gear and
                    decision.published_acceleration_mps2 > cfg.maximum_reverse_propulsion_mps2):
                decision = replace(decision, acceleration_mps2=-cfg.maximum_reverse_propulsion_mps2)
            if (cfg.maximum_recovery_propulsion_mps2 > 0.0 and
                    decision.published_acceleration_mps2 > cfg.maximum_recovery_propulsion_mps2):
                decision = replace(decision, acceleration_mps2=(
                    -cfg.maximum_recovery_propulsion_mps2 if decision.publish_in_reverse_gear
                    else cfg.maximum_recovery_propulsion_mps2))
        return RecoveryTick(update, execution, decision, rear_wall, short_mode,
                            cycle_heading_error, cycle_lateral_abs, motion)

    def commit(self, tick):
        self.recovery.commit_update(tick.update)
        self.execution.commit_update(tick.execution)
        self.short_mode = tick.short_mode
        self.cycle_heading_error = tick.cycle_heading_error
        self.cycle_lateral_abs = tick.cycle_lateral_abs
