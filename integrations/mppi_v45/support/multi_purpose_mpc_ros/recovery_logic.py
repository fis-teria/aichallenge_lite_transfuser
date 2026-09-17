# 野口追記：（壁衝突などで動けない場合の復帰状態判定。2026-07-20、ユーザー
# 提案で全面簡略化。「MPCへ戻していいほど良い状態か」を内部で細かく判定
# しようとする設計（向き閾値・速度閾値・保持時間の組み合わせ）は、閾値の
# 想定が少しでもズレると実害が出るバグを繰り返し生んだ（例: 実速度が閾値の
# わずか手前で壁に当たり続けるケース、逆に判定が緩すぎて向きが全く揃って
# いないまま即MPCへ戻すケース）。「1サイクル（後退→前進）やったら必ず
# MPCへ渡す。まだダメなら外側のスタック検知（_update_monitoring）が
# 2秒後にまた拾う」という設計に変更し、判定基準そのものの正しさに依存
# しない、外側のリトライで自然に収束する構造にした）
"""ROSやMPC本体に依存しない、スタック復帰用の軽量状態機械。

方針: まず後退して前方にスペースを開ける（後方の壁/他車にぶつかりそうに
なったら＝実際に下がれなくなったらそこで止める）。次にコース中央の少し先を
狙って前進する（前方が塞がって進めなくなったらそこで止める）。どちらの
フェーズも、塞がった・実際に動けていない・タイムアウト、のいずれかで終了し、
**判定の良し悪しを問わずそのままMPCへ制御を渡す**。まだ足りなければ、
MPC運転中の停止検知（_update_monitoring）が自然に次のBACKING_UPを起動する。

他車の位置は、後退/前進の可否判定そのものには使わない（一瞬の接近検知で
即座に動作を中断するとライブロックしうるため、実速度が出ているかどうかの
継続監視だけで壁・他車を区別せず「塞がれているか」を判定する）。
"""

import math
from dataclasses import dataclass, replace
from typing import Optional, cast

from multi_purpose_mpc_ros.angle_domain import WrappedHeadingErrorRad
from multi_purpose_mpc_ros.speed_domain import (
    BaseReferenceSpeed,
    BaseReferenceSpeedSample,
    ConstrainedSpeed,
    ConstrainedSpeedSample,
    SpeedSource,
    SpeedUnavailable,
)


RECOVERY_STATE_MONITORING = "MONITORING"
RECOVERY_STATE_BACKING_UP = "BACKING_UP"
RECOVERY_STATE_STOPPING_TO_FORWARD = "STOPPING_TO_FORWARD"
RECOVERY_STATE_ALIGNING_FORWARD = "ALIGNING_FORWARD"
# 野口追記：（2026-07-20、ユーザー報告。停止車両が自車の正面（センターライン
# 付近）を塞いでいると、後退→センターライン狙いの前進、を何度繰り返しても
# 同じ場所に戻ってしまい無限ループしうる。連続でこの通常サイクルが
# normal_cycle_limit回を超えたら、代わりにこのESCAPE_TURN状態（固定角度で
# 右へ曲がりながら前進し、一定時間で無条件にMPCへ戻す）を使い、センター
# ラインとは違う場所へ抜け出す試みをする）
RECOVERY_STATE_ESCAPE_TURN = "ESCAPE_TURN"

RECOVERY_DIRECTIONAL_WALL_SAFE = "SAFE"
RECOVERY_DIRECTIONAL_WALL_IMPROVING_ESCAPE = "IMPROVING_ESCAPE"
RECOVERY_DIRECTIONAL_WALL_BLOCKED = "BLOCKED"

_RECOVERY_DIRECTIONAL_WALL_STATUSES = frozenset(
    (
        RECOVERY_DIRECTIONAL_WALL_SAFE,
        RECOVERY_DIRECTIONAL_WALL_IMPROVING_ESCAPE,
        RECOVERY_DIRECTIONAL_WALL_BLOCKED,
    )
)

_RECOVERY_STATES = frozenset(
    (
        RECOVERY_STATE_MONITORING,
        RECOVERY_STATE_BACKING_UP,
        RECOVERY_STATE_STOPPING_TO_FORWARD,
        RECOVERY_STATE_ALIGNING_FORWARD,
        RECOVERY_STATE_ESCAPE_TURN,
    )
)


@dataclass(frozen=True)
class RecoveryDirectionalWallAssessment:
    """One immutable assessment of the existing straight Recovery motion."""

    status: str
    current_violation_m: float
    probe_violation_m: float

    def __post_init__(self) -> None:
        if self.status not in _RECOVERY_DIRECTIONAL_WALL_STATUSES:
            raise ValueError(f"unknown Recovery wall status: {self.status}")
        violations = (
            float(self.current_violation_m),
            float(self.probe_violation_m),
        )
        if any(math.isnan(value) or value < 0.0 for value in violations):
            raise ValueError("Recovery wall violations must be non-negative")
        if any(math.isinf(value) for value in violations) and (
            self.status != RECOVERY_DIRECTIONAL_WALL_BLOCKED
        ):
            raise ValueError("non-finite Recovery wall geometry must be blocked")
        object.__setattr__(self, "current_violation_m", violations[0])
        object.__setattr__(self, "probe_violation_m", violations[1])

    @property
    def allows_reverse(self) -> bool:
        return self.status != RECOVERY_DIRECTIONAL_WALL_BLOCKED


@dataclass(frozen=True)
class StuckRecoveryStateSnapshot:
    """The complete mutable state of one ``StuckRecovery`` instance."""

    state: str
    stopped_since: Optional[float]
    move_since: Optional[float]
    move_stuck_since: Optional[float]
    move_stuck_since_distance_m: Optional[float]
    move_blocked_since: Optional[float]
    stopping_since: Optional[float]
    stationary_since: Optional[float]
    monitoring_since: Optional[float]
    consecutive_recovery_count: int
    escape_turn_attempt_count: int
    escape_turn_steer_sign: float

    def __post_init__(self) -> None:
        if self.state not in _RECOVERY_STATES:
            raise ValueError(f"unknown recovery snapshot state: {self.state}")


@dataclass(frozen=True)
class StuckRecoveryUpdateProposal:
    """One state-machine fold that has not yet changed the live owner."""

    before: StuckRecoveryStateSnapshot
    after: StuckRecoveryStateSnapshot

    def __post_init__(self) -> None:
        if not isinstance(self.before, StuckRecoveryStateSnapshot):
            raise TypeError("recovery proposal before must be a snapshot")
        if not isinstance(self.after, StuckRecoveryStateSnapshot):
            raise TypeError("recovery proposal after must be a snapshot")


@dataclass(frozen=True)
class RecoveryTransitionProposal:
    """Cross-domain side effects requested by one recovery transition.

    The legacy Controller clears avoidance, the MPC lateral reference, and a
    pending boost together when a recovery episode leaves ``MONITORING``.
    Derive those requests from the transition instead of accepting independent
    booleans, so callers cannot construct a partially cleared episode.
    """

    previous_state: str
    current_state: str

    def __post_init__(self) -> None:
        if self.previous_state not in _RECOVERY_STATES:
            raise ValueError(f"unknown previous recovery state: {self.previous_state}")
        if self.current_state not in _RECOVERY_STATES:
            raise ValueError(f"unknown current recovery state: {self.current_state}")
        if self.previous_state == self.current_state:
            raise ValueError("recovery transition requires different states")

    @property
    def starts_episode(self) -> bool:
        return self.previous_state == RECOVERY_STATE_MONITORING

    @property
    def reset_avoidance(self) -> bool:
        return self.starts_episode

    @property
    def clear_mpc_lateral_reference(self) -> bool:
        return self.starts_episode

    @property
    def cancel_boost_request(self) -> bool:
        return self.starts_episode


@dataclass(frozen=True)
class RecoveryPriorityDecision:
    """Legacy cross-policy effects selected around one recovery update.

    Starting recovery resets the avoidance episode, including its squeeze and
    replan-BRAKE state. While recovery remains active it also prevents the
    avoidance pipeline from running and owns any stray fallback-Pure stop.
    This records the existing priority; it does not correct that policy.
    """

    current_state: str
    transition: Optional[RecoveryTransitionProposal]
    fallback_pure_was_active: bool

    def __post_init__(self) -> None:
        if self.current_state not in _RECOVERY_STATES:
            raise ValueError(
                f"unknown current recovery state: {self.current_state}"
            )
        if (
            self.transition is not None
            and self.transition.current_state != self.current_state
        ):
            raise ValueError(
                "recovery transition current state must match priority state"
            )
        if not isinstance(self.fallback_pure_was_active, bool):
            raise TypeError("fallback_pure_was_active must be bool")

    @property
    def recovery_active(self) -> bool:
        return self.current_state != RECOVERY_STATE_MONITORING

    @property
    def moving_reverse(self) -> bool:
        return self.current_state == RECOVERY_STATE_BACKING_UP

    @property
    def stopping(self) -> bool:
        return self.current_state == RECOVERY_STATE_STOPPING_TO_FORWARD

    @property
    def moving_forward(self) -> bool:
        return self.current_state == RECOVERY_STATE_ALIGNING_FORWARD

    @property
    def escape_turning(self) -> bool:
        return self.current_state == RECOVERY_STATE_ESCAPE_TURN

    @property
    def reset_avoidance(self) -> bool:
        return bool(
            self.transition is not None
            and self.transition.reset_avoidance
        )

    @property
    def clear_mpc_lateral_reference(self) -> bool:
        return bool(
            self.transition is not None
            and self.transition.clear_mpc_lateral_reference
        )

    @property
    def cancel_boost_request(self) -> bool:
        return bool(
            self.transition is not None
            and self.transition.cancel_boost_request
        )

    @property
    def stop_fallback_pure(self) -> bool:
        return (
            self.recovery_active
            and self.fallback_pure_was_active
            and not self.reset_avoidance
        )

    @property
    def evaluate_avoidance(self) -> bool:
        return not self.recovery_active

    @property
    def priority_reason(self) -> str:
        if self.reset_avoidance:
            return "recovery_episode_start"
        if self.recovery_active:
            return "recovery_active"
        if self.transition is not None:
            return "recovery_handoff"
        return "recovery_inactive"


@dataclass(frozen=True)
class RecoveryTickPreparation:
    """Stage-2 Recovery result derived from one frozen control tick.

    It contains decisions and immutable geometry only.  In particular, its
    construction neither publishes a command nor writes either live Recovery
    owner.  ``ALIGNING_FORWARD`` steer materialization remains a named
    2b-partial debt because it currently refreshes the avoidance preview.
    """

    update: StuckRecoveryUpdateProposal
    priority: RecoveryPriorityDecision
    heading_error_rad: float
    ego_x_m: float
    ego_y_m: float
    rear_wall_assessment: RecoveryDirectionalWallAssessment

    def __post_init__(self) -> None:
        if not isinstance(self.update, StuckRecoveryUpdateProposal):
            raise TypeError("recovery preparation requires an update proposal")
        if not isinstance(self.priority, RecoveryPriorityDecision):
            raise TypeError("recovery preparation requires a priority decision")
        if self.priority.current_state != self.update.after.state:
            raise ValueError("recovery priority and update state must match")
        if not all(math.isfinite(value) for value in (
            self.heading_error_rad,
            self.ego_x_m,
            self.ego_y_m,
        )):
            raise ValueError("recovery preparation geometry must be finite")
        if not isinstance(
            self.rear_wall_assessment,
            RecoveryDirectionalWallAssessment,
        ):
            raise TypeError(
                "recovery preparation requires a rear-wall assessment"
            )

    @property
    def rear_wall_is_blocked(self) -> bool:
        """Compatibility projection consumed by the existing state machine."""
        return not self.rear_wall_assessment.allows_reverse


@dataclass(frozen=True)
class RecoveryCommitPlan:
    """Stage-4 materialization whose owner writes wait for Stage 6."""

    preparation: RecoveryTickPreparation
    execution: "RecoveryExecutionUpdateProposal"
    mpc_previous_steering_rad: Optional[float]

    def __post_init__(self) -> None:
        if not isinstance(self.preparation, RecoveryTickPreparation):
            raise TypeError("recovery commit plan requires a preparation")
        if not isinstance(self.execution, RecoveryExecutionUpdateProposal):
            raise TypeError("recovery commit plan requires an execution proposal")
        if self.mpc_previous_steering_rad is not None and not math.isfinite(
            self.mpc_previous_steering_rad
        ):
            raise ValueError("recovery MPC previous steer must be finite")


def resolve_legacy_recovery_priority(
    *,
    current_state: str,
    transition: Optional[RecoveryTransitionProposal],
    fallback_pure_was_active: bool,
) -> RecoveryPriorityDecision:
    """Resolve existing recovery-first cross-policy effects without mutation."""

    return RecoveryPriorityDecision(
        current_state=current_state,
        transition=transition,
        fallback_pure_was_active=fallback_pure_was_active,
    )


def normalize_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def select_recovery_intended_speed_mps(
    *,
    base_reference_speed: BaseReferenceSpeedSample,
    constrained_speed: ConstrainedSpeedSample,
) -> float:
    """Return the forward intent used by the stuck monitor.

    ``Waypoint.v_ref`` is intentionally mutable: BRAKE and other transient
    limits can reduce it all the way to zero.  A stopped vehicle must still be
    recognised as stuck when the unmodified course profile asks it to move, so
    prefer that base profile here.  The limited value is only a defensive
    fallback for an invalid base sample.
    """
    if isinstance(base_reference_speed, BaseReferenceSpeed):
        base_value_mps = base_reference_speed.value_mps
    elif (
        isinstance(base_reference_speed, SpeedUnavailable)
        and base_reference_speed.source == SpeedSource.BASE_REFERENCE
    ):
        base_value_mps = None
    else:
        raise TypeError(
            "base_reference_speed requires base-reference semantics"
        )

    if isinstance(constrained_speed, ConstrainedSpeed):
        constrained_value_mps = constrained_speed.value_mps
    elif (
        isinstance(constrained_speed, SpeedUnavailable)
        and constrained_speed.source == SpeedSource.CONSTRAINED
    ):
        constrained_value_mps = None
    else:
        raise TypeError("constrained_speed requires constrained semantics")

    if base_value_mps is not None:
        return float(base_value_mps)
    if constrained_value_mps is not None:
        return float(constrained_value_mps)
    return 0.0


def select_recovery_escape_steer_sign(
    *,
    preferred_sign: float,
    lateral_offset_m: float,
    safe_lower_bound_m: float,
    safe_upper_bound_m: float,
    wall_guard_margin_m: float,
) -> float:
    """Keep an escape turn from steering farther into a nearby wall.

    ``preferred_sign`` is the alternating retry choice (right=-1, left=+1).
    It is preserved while that side has room.  If the vehicle is already
    within ``wall_guard_margin_m`` of the safe bound on the selected side, the
    turn is overridden toward the side with more remaining lateral room.
    """
    values = (
        preferred_sign,
        lateral_offset_m,
        safe_lower_bound_m,
        safe_upper_bound_m,
        wall_guard_margin_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("escape steering inputs must be finite")
    if preferred_sign == 0.0:
        raise ValueError("preferred_sign must be non-zero")
    if safe_upper_bound_m < safe_lower_bound_m:
        raise ValueError("safe upper bound must not be below lower bound")
    if wall_guard_margin_m < 0.0:
        raise ValueError("wall_guard_margin_m must be non-negative")

    preferred = 1.0 if preferred_sign > 0.0 else -1.0
    left_clearance = safe_upper_bound_m - lateral_offset_m
    right_clearance = lateral_offset_m - safe_lower_bound_m
    preferred_clearance = left_clearance if preferred > 0.0 else right_clearance
    if preferred_clearance >= wall_guard_margin_m:
        return preferred
    return 1.0 if left_clearance > right_clearance else -1.0


def _recovery_lateral_wall_violation_m(
    *,
    lateral_offset_m: float,
    safe_lower_bound_m: float,
    safe_upper_bound_m: float,
    wall_guard_margin_m: float,
) -> float:
    """Return distance outside one guarded Recovery band, or inf if invalid."""
    values = (
        lateral_offset_m,
        safe_lower_bound_m,
        safe_upper_bound_m,
        wall_guard_margin_m,
    )
    if not all(math.isfinite(value) for value in values):
        return math.inf
    if (
        safe_upper_bound_m < safe_lower_bound_m
        or wall_guard_margin_m < 0.0
    ):
        return math.inf
    guarded_lower_m = safe_lower_bound_m + wall_guard_margin_m
    guarded_upper_m = safe_upper_bound_m - wall_guard_margin_m
    if guarded_upper_m < guarded_lower_m:
        return math.inf
    return max(
        guarded_lower_m - lateral_offset_m,
        0.0,
        lateral_offset_m - guarded_upper_m,
    )


def assess_recovery_directional_wall_clearance(
    *,
    current_lateral_offset_m: float,
    current_safe_lower_bound_m: float,
    current_safe_upper_bound_m: float,
    probe_lateral_offset_m: float,
    probe_safe_lower_bound_m: float,
    probe_safe_upper_bound_m: float,
    wall_guard_margin_m: float,
) -> RecoveryDirectionalWallAssessment:
    """Classify the existing straight Recovery motion from its two endpoints.

    A pose already outside the conservative band must not make every escape
    direction unreachable.  The existing straight reverse is therefore also
    admitted when its probe strictly reduces the same violation.  Invalid,
    equal, and worsening geometry remain fail-closed.
    """
    current_violation_m = _recovery_lateral_wall_violation_m(
        lateral_offset_m=current_lateral_offset_m,
        safe_lower_bound_m=current_safe_lower_bound_m,
        safe_upper_bound_m=current_safe_upper_bound_m,
        wall_guard_margin_m=wall_guard_margin_m,
    )
    probe_violation_m = _recovery_lateral_wall_violation_m(
        lateral_offset_m=probe_lateral_offset_m,
        safe_lower_bound_m=probe_safe_lower_bound_m,
        safe_upper_bound_m=probe_safe_upper_bound_m,
        wall_guard_margin_m=wall_guard_margin_m,
    )
    if not all(math.isfinite(value) for value in (
        current_violation_m,
        probe_violation_m,
    )):
        status = RECOVERY_DIRECTIONAL_WALL_BLOCKED
    elif current_violation_m == 0.0 and probe_violation_m == 0.0:
        status = RECOVERY_DIRECTIONAL_WALL_SAFE
    elif (
        current_violation_m > 0.0
        and probe_violation_m < current_violation_m
    ):
        status = RECOVERY_DIRECTIONAL_WALL_IMPROVING_ESCAPE
    else:
        status = RECOVERY_DIRECTIONAL_WALL_BLOCKED
    return RecoveryDirectionalWallAssessment(
        status=status,
        current_violation_m=current_violation_m,
        probe_violation_m=probe_violation_m,
    )


# 野口追記：（2026-07-20、簡略化に伴い削除。壁ギャップ基準の左右バイアス
# select_escape_steering_direction_with_lookahead()は、前進フェーズの操舵が
# 「コース中央の少し先の点を狙う」方式に変わったことで不要になった——
# センターラインを狙えば、自車がどちらに寄っていても自然にその点へ戻る
# 向きの操舵になるため、左右どちらへ逃げるかを別途判定する必要がない）
def select_recovery_forward_steer_rad(
    *,
    heading_error: WrappedHeadingErrorRad,
    heading_steer_gain: float,
    max_steer_rad: float,
) -> float:
    """コース中央の少し先の点への向き直し操舵角を返す。
    heading_error = 車体の向き - (自車から狙い先の点への方位)。
    """
    if not isinstance(heading_error, WrappedHeadingErrorRad):
        raise TypeError(
            "heading_error must be a WrappedHeadingErrorRad"
        )
    if heading_steer_gain <= 0.0:
        raise ValueError("heading_steer_gain must be positive")
    if max_steer_rad <= 0.0:
        raise ValueError("max_steer_rad must be positive")

    raw = -heading_steer_gain * float(heading_error.value_rad)
    return max(-max_steer_rad, min(max_steer_rad, raw))


@dataclass(frozen=True)
class RecoveryExecutionStateSnapshot:
    """All Controller-side recovery command memory frozen for one tick."""

    active_gear: int
    escape_steer_rad: float
    move_entry_xy: Optional[tuple[float, float]]

    def __post_init__(self) -> None:
        if isinstance(self.active_gear, bool) or not isinstance(
            self.active_gear, int
        ):
            raise TypeError("recovery active gear must be int")
        if not math.isfinite(self.escape_steer_rad):
            raise ValueError("recovery escape steer must be finite")
        if self.move_entry_xy is not None:
            if len(self.move_entry_xy) != 2 or not all(
                math.isfinite(value) for value in self.move_entry_xy
            ):
                raise ValueError("recovery move entry must contain finite x/y")


@dataclass(frozen=True)
class RecoveryExecutionUpdateProposal:
    """A recovery execution-memory update that has not touched its owner."""

    before: RecoveryExecutionStateSnapshot
    after: RecoveryExecutionStateSnapshot
    gear_command: Optional[int]

    def __post_init__(self) -> None:
        if not isinstance(self.before, RecoveryExecutionStateSnapshot):
            raise TypeError("recovery execution before must be a snapshot")
        if not isinstance(self.after, RecoveryExecutionStateSnapshot):
            raise TypeError("recovery execution after must be a snapshot")
        if self.gear_command is not None and (
            isinstance(self.gear_command, bool)
            or not isinstance(self.gear_command, int)
        ):
            raise TypeError("recovery gear command must be int or None")


class RecoveryExecutionState:
    """Own Controller-side recovery command memory and its staged writes.

    Gear publication and recovery-path geometry can fail between the legacy
    assignments.  The three values therefore share one owner but retain
    separate named commits instead of being atomized into one transition.
    Values intentionally survive recovery handback, matching the legacy
    Controller lifetime.
    """

    def __init__(self, *, initial_gear: int) -> None:
        self._active_gear = initial_gear
        self._escape_steer_rad = 0.0
        self._move_entry_xy: Optional[tuple[float, float]] = None

    @property
    def active_gear(self) -> int:
        return self._active_gear

    @property
    def escape_steer_rad(self) -> float:
        return self._escape_steer_rad

    @property
    def move_entry_xy(self) -> Optional[tuple[float, float]]:
        return self._move_entry_xy

    def snapshot(self) -> RecoveryExecutionStateSnapshot:
        return RecoveryExecutionStateSnapshot(
            active_gear=self._active_gear,
            escape_steer_rad=self._escape_steer_rad,
            move_entry_xy=self._move_entry_xy,
        )

    def commit_update(self, proposal: RecoveryExecutionUpdateProposal) -> None:
        """Commit an execution preview only against its exact source state."""

        if not isinstance(proposal, RecoveryExecutionUpdateProposal):
            raise TypeError("recovery execution commit requires a proposal")
        if self.snapshot() != proposal.before:
            raise RuntimeError("stale recovery execution update proposal")
        self._active_gear = proposal.after.active_gear
        self._escape_steer_rad = proposal.after.escape_steer_rad
        self._move_entry_xy = proposal.after.move_entry_xy

    def record_active_gear(self, *, gear: int) -> None:
        self._active_gear = gear

    def record_escape_steer(self, *, steer_rad: float) -> None:
        self._escape_steer_rad = steer_rad

    def record_move_entry(self, *, x_m: float, y_m: float) -> None:
        self._move_entry_xy = (x_m, y_m)


def propose_recovery_execution_update(
    *,
    state: RecoveryExecutionState,
    current_state: str,
    previous_state: str,
    ego_x_m: float,
    ego_y_m: float,
    drive_gear: int,
    reverse_gear: int,
    aligning_forward_steer_rad: Optional[float],
    escape_turn_steer_rad: Optional[float],
    backing_up_steer_rad: Optional[float] = None,
) -> RecoveryExecutionUpdateProposal:
    """Derive recovery command memory and a transition gear publication.

    The function is deliberately pure with respect to ``state``.  Steer values
    are supplied by the selected executor materializer; this domain function
    only validates and records their intended post-publish ownership.
    """

    if current_state not in _RECOVERY_STATES:
        raise ValueError(f"unknown current recovery state: {current_state}")
    if previous_state not in _RECOVERY_STATES:
        raise ValueError(f"unknown previous recovery state: {previous_state}")
    if not math.isfinite(ego_x_m) or not math.isfinite(ego_y_m):
        raise ValueError("recovery ego position must be finite")
    if any(isinstance(gear, bool) or not isinstance(gear, int) for gear in (
        drive_gear,
        reverse_gear,
    )):
        raise TypeError("recovery gear values must be int")

    before = state.snapshot()
    active_gear = before.active_gear
    escape_steer_rad = before.escape_steer_rad
    move_entry_xy = before.move_entry_xy
    gear_command: Optional[int] = None
    transitioned = current_state != previous_state

    if backing_up_steer_rad is not None:
        if current_state != RECOVERY_STATE_BACKING_UP or not math.isfinite(backing_up_steer_rad):
            raise ValueError("backing steer requires BACKING_UP and a finite value")
        escape_steer_rad = backing_up_steer_rad

    if current_state == RECOVERY_STATE_ALIGNING_FORWARD:
        if aligning_forward_steer_rad is None or not math.isfinite(
            aligning_forward_steer_rad
        ):
            raise ValueError("ALIGNING_FORWARD requires a finite steer")
        escape_steer_rad = aligning_forward_steer_rad
    elif aligning_forward_steer_rad is not None:
        raise ValueError("aligning steer is only valid in ALIGNING_FORWARD")

    if current_state == RECOVERY_STATE_ESCAPE_TURN:
        if transitioned:
            if escape_turn_steer_rad is None or not math.isfinite(
                escape_turn_steer_rad
            ):
                raise ValueError("ESCAPE_TURN entry requires a finite steer")
            escape_steer_rad = escape_turn_steer_rad
        elif escape_turn_steer_rad is not None:
            if not math.isfinite(escape_turn_steer_rad):
                raise ValueError("escape turn steer must be finite")
            escape_steer_rad = escape_turn_steer_rad
    elif escape_turn_steer_rad is not None:
        raise ValueError("escape steer is only valid in ESCAPE_TURN")

    if transitioned and current_state == RECOVERY_STATE_BACKING_UP:
        active_gear = reverse_gear
        gear_command = reverse_gear
        move_entry_xy = (ego_x_m, ego_y_m)
    elif transitioned and current_state in (
        RECOVERY_STATE_ALIGNING_FORWARD,
        RECOVERY_STATE_ESCAPE_TURN,
    ):
        active_gear = drive_gear
        gear_command = drive_gear
        move_entry_xy = (ego_x_m, ego_y_m)
    elif transitioned and current_state == RECOVERY_STATE_MONITORING:
        active_gear = drive_gear
        gear_command = drive_gear

    return RecoveryExecutionUpdateProposal(
        before=before,
        after=RecoveryExecutionStateSnapshot(
            active_gear=active_gear,
            escape_steer_rad=escape_steer_rad,
            move_entry_xy=move_entry_xy,
        ),
        gear_command=gear_command,
    )


@dataclass
class StuckRecovery:
    """停止継続を検知し、後退（前方スペース確保）→向き直しながら前進、を
    向きが揃い前進できるようになるまで繰り返す。"""

    stop_speed_threshold: float
    min_forward_command_speed: float
    stuck_detection_sec: float
    reverse_stop_speed_threshold: float
    reverse_stop_hold_sec: float
    grace_sec: float
    backing_up_stuck_hold_sec: float
    backing_up_timeout_sec: float
    # 野口追記：（2026-07-20、簡略化。前進フェーズは「前方が塞がった」か
    # 「実際に進めていない」かのいずれかでMPCへ戻る。塞がり判定
    # （forward_is_clearの否定）はこの保持時間で継続確認してから使う
    # ——単発tickでの誤反応を防ぐため）
    aligning_forward_blocked_hold_sec: float
    aligning_forward_stuck_hold_sec: float
    aligning_forward_timeout_sec: float
    # 野口追記：（前進復帰の加速度はレギュレーション上限(a_max)を超えて
    # 強めることができないため、速度閾値だけでスタック判定すると、弱い加速度で
    # じわじわ前進できているケースまで誤って打ち切ってしまう。stuck_hold_sec
    # の窓の中で実際に進んだ距離で判定する）
    aligning_forward_min_progress_m: float
    # 野口追記：（2026-07-20、無限ループ対策。連続でBACKING_UPに
    # 再突入した回数がこの値を超えたら、次の前進フェーズは通常の
    # ALIGNING_FORWARD（センターライン狙い）ではなくESCAPE_TURN
    # （固定角度で右へ）を使う）
    normal_cycle_limit: int
    # 野口追記：（MONITORING状態がこの秒数途切れず続いたら「本当に
    # 復帰できた」とみなし、連続カウントを0に戻す。短すぎると、まだ
    # 本当は解決していないのにカウントがリセットされてESCAPE_TURNへ
    # 中々エスカレーションしない。長すぎると、一度解決した後の別の
    # 新しいスタックまで前回の続きとして数えてしまう）
    cycle_reset_sec: float
    # 野口追記：（ESCAPE_TURN状態を維持する固定秒数。この間、判定なしで
    # 一定角度・一定速度のまま前進し続け、経過後は無条件でMPCへ戻す）
    escape_turn_sec: float

    state: str = RECOVERY_STATE_MONITORING
    stopped_since: Optional[float] = None
    move_since: Optional[float] = None
    move_stuck_since: Optional[float] = None
    move_stuck_since_distance_m: Optional[float] = None
    move_blocked_since: Optional[float] = None
    stopping_since: Optional[float] = None
    stationary_since: Optional[float] = None
    # 野口追記：（直近でMONITORINGが途切れず続いている開始時刻。
    # cycle_reset_secとの比較に使う）
    monitoring_since: Optional[float] = None
    # 野口追記：（連続でBACKING_UPに再突入した回数。cycle_reset_sec分
    # 安定して運転できたらリセットされる、normal_cycle_limitを超えたら
    # ESCAPE_TURNへ切り替わる）
    consecutive_recovery_count: int = 0
    # 同じスタックepisode内でESCAPE_TURNへ入った回数。最初は右、次は左と
    # 交互に試し、固定右旋回を何度も繰り返すデッドロックを避ける。
    # cycle_reset_secだけ通常走行できた時に0へ戻す。
    escape_turn_attempt_count: int = 0
    # 現在のESCAPE_TURN操舵符号（右=-1、左=+1）。状態へ入った瞬間に固定し、
    # そのESCAPE_TURN中は変えない。
    escape_turn_steer_sign: float = -1.0

    def __post_init__(self) -> None:
        if self.stop_speed_threshold < 0.0:
            raise ValueError("stop_speed_threshold must be non-negative")
        if self.min_forward_command_speed < 0.0:
            raise ValueError("min_forward_command_speed must be non-negative")
        if self.stuck_detection_sec <= 0.0:
            raise ValueError("stuck_detection_sec must be positive")
        if self.reverse_stop_speed_threshold < 0.0:
            raise ValueError("reverse_stop_speed_threshold must be non-negative")
        if self.reverse_stop_hold_sec <= 0.0:
            raise ValueError("reverse_stop_hold_sec must be positive")
        if self.grace_sec < 0.0:
            raise ValueError("grace_sec must be non-negative")
        if self.backing_up_stuck_hold_sec <= 0.0:
            raise ValueError("backing_up_stuck_hold_sec must be positive")
        if self.backing_up_timeout_sec <= 0.0:
            raise ValueError("backing_up_timeout_sec must be positive")
        if self.aligning_forward_blocked_hold_sec <= 0.0:
            raise ValueError("aligning_forward_blocked_hold_sec must be positive")
        if self.aligning_forward_stuck_hold_sec <= 0.0:
            raise ValueError("aligning_forward_stuck_hold_sec must be positive")
        if self.aligning_forward_timeout_sec <= 0.0:
            raise ValueError("aligning_forward_timeout_sec must be positive")
        if self.aligning_forward_min_progress_m <= 0.0:
            raise ValueError("aligning_forward_min_progress_m must be positive")
        if self.normal_cycle_limit <= 0:
            raise ValueError("normal_cycle_limit must be positive")
        if self.cycle_reset_sec <= 0.0:
            raise ValueError("cycle_reset_sec must be positive")
        if self.escape_turn_sec <= 0.0:
            raise ValueError("escape_turn_sec must be positive")

    def snapshot(self) -> StuckRecoveryStateSnapshot:
        """Capture every mutable state-machine field without configuration."""

        return StuckRecoveryStateSnapshot(
            state=self.state,
            stopped_since=self.stopped_since,
            move_since=self.move_since,
            move_stuck_since=self.move_stuck_since,
            move_stuck_since_distance_m=self.move_stuck_since_distance_m,
            move_blocked_since=self.move_blocked_since,
            stopping_since=self.stopping_since,
            stationary_since=self.stationary_since,
            monitoring_since=self.monitoring_since,
            consecutive_recovery_count=self.consecutive_recovery_count,
            escape_turn_attempt_count=self.escape_turn_attempt_count,
            escape_turn_steer_sign=self.escape_turn_steer_sign,
        )

    def commit_update(self, proposal: StuckRecoveryUpdateProposal) -> None:
        """Apply one preview only if it was derived from the current state."""

        if not isinstance(proposal, StuckRecoveryUpdateProposal):
            raise TypeError("recovery commit requires an update proposal")
        if self.snapshot() != proposal.before:
            raise RuntimeError("stale recovery update proposal")
        next_state = proposal.after
        self.state = next_state.state
        self.stopped_since = next_state.stopped_since
        self.move_since = next_state.move_since
        self.move_stuck_since = next_state.move_stuck_since
        self.move_stuck_since_distance_m = (
            next_state.move_stuck_since_distance_m
        )
        self.move_blocked_since = next_state.move_blocked_since
        self.stopping_since = next_state.stopping_since
        self.stationary_since = next_state.stationary_since
        self.monitoring_since = next_state.monitoring_since
        self.consecutive_recovery_count = (
            next_state.consecutive_recovery_count
        )
        self.escape_turn_attempt_count = next_state.escape_turn_attempt_count
        self.escape_turn_steer_sign = next_state.escape_turn_steer_sign

    def reset(self, now_sec: float) -> None:
        """監視状態へ戻し、途中のあらゆる復帰関連タイマーを破棄する。"""
        self.state = RECOVERY_STATE_MONITORING
        self.stopped_since = None
        self.move_since = None
        self.move_stuck_since = None
        self.move_stuck_since_distance_m = None
        self.move_blocked_since = None
        self.stopping_since = None
        self.stationary_since = None
        self.monitoring_since = now_sec

    def record_escape_turn_steer_sign(self, *, steer_sign: float) -> None:
        """Record the wall-guarded sign without exposing a second writer."""
        self.escape_turn_steer_sign = steer_sign

    def _enter_move(self, now_sec: float, state: str) -> None:
        self.state = state
        self.move_since = now_sec
        self.move_stuck_since = None
        self.move_stuck_since_distance_m = None
        self.move_blocked_since = None

    def _enter_stopping(self, now_sec: float, stopping_state: str) -> None:
        self.state = stopping_state
        self.stopping_since = now_sec
        self.stationary_since = None

    def update(
        self,
        *,
        now_sec: float,
        measured_speed: float,
        previous_target_speed: float,
        recovery_allowed: bool,
        distance_since_move_entry_m: float = 0.0,
        forward_is_clear: bool = False,
        rear_is_blocked: bool = False,
        rear_wall_is_blocked: bool = False,
        ready_for_handback: bool = False,
        reverse_motion_limit_reached: bool = False,
        forward_motion_limit_reached: bool = False,
    ) -> str:
        """現在値で状態を1回更新し、更新後の状態を返す。

        ``distance_since_move_entry_m``は、現在のBACKING_UP/ALIGNING_FORWARDに
        入ってから実際に進んだ距離（呼び出し側が自車位置から計算）。
        ALIGNING_FORWARDのスタック判定に使う。
        ``forward_is_clear``/``rear_is_blocked``は、呼び出し側が車体正面/背面
        それぞれについて、他車ギャップ＋コリドー幅の両方から「詰まっていないか」
        を判定して渡す。``rear_is_blocked``はBACKING_UPを終了する条件、
        ``forward_is_clear``の否定（forward_is_blocked）はALIGNING_FORWARDを
        終了する条件として使う——どちらも「これ以上進めない」の判定であって、
        「MPCに戻していいほど良い状態か」の判定ではない。ALIGNING_FORWARDは
        塞がった・実際に進めていない・タイムアウトのいずれかで無条件にMPCへ
        戻る（2026-07-20、簡略化）。
        ``ready_for_handback``は、呼び出し側がSTOPPING_TO_FORWARD中に
        （コース進行方向への向き＋コース中央への近さから）判定する、
        「もうALIGNING_FORWARDを経由せず直接MPCへ戻してよいか」の判定
        （2026-07-21、ユーザー提案）。STOPPING_TO_FORWARDの停止確認完了時に
        だけ参照する。
        """
        if not recovery_allowed:
            self.reset(now_sec)
            return self.state

        # Optional geometric travel budgets from a recovery executor. They
        # include stopping room, so the legacy stalled-motion grace/hold must
        # not delay their braking request. Ordinary MPC callers omit them.
        if self.state == RECOVERY_STATE_BACKING_UP and reverse_motion_limit_reached:
            self._enter_stopping(now_sec, RECOVERY_STATE_STOPPING_TO_FORWARD)
            return self.state
        if self.state in (RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN) and forward_motion_limit_reached:
            self.reset(now_sec)
            return self.state

        if self.state in (RECOVERY_STATE_BACKING_UP, RECOVERY_STATE_ALIGNING_FORWARD):
            return self._update_move(
                now_sec, measured_speed,
                distance_since_move_entry_m, forward_is_clear, rear_is_blocked,
                rear_wall_is_blocked,
            )

        if self.state == RECOVERY_STATE_ESCAPE_TURN:
            return self._update_escape_turn(now_sec)

        if self.state == RECOVERY_STATE_STOPPING_TO_FORWARD:
            self._update_stopping(now_sec, measured_speed, ready_for_handback)
            if self.state in (RECOVERY_STATE_ALIGNING_FORWARD, RECOVERY_STATE_ESCAPE_TURN) and forward_motion_limit_reached:
                self.reset(now_sec)
            return self.state

        state = self._update_monitoring(now_sec, measured_speed, previous_target_speed)
        # A deterministic wall-boundary violation is unlike a noisy one-tick
        # vehicle observation.  If recovery was just triggered while the rear
        # sweep is already unsafe, never emit even the first reverse command;
        # settle and proceed to the forward alignment phase instead.
        if state == RECOVERY_STATE_BACKING_UP and (rear_wall_is_blocked or reverse_motion_limit_reached):
            self._enter_stopping(now_sec, RECOVERY_STATE_STOPPING_TO_FORWARD)
        return self.state

    def _update_monitoring(
        self,
        now_sec: float,
        measured_speed: float,
        previous_target_speed: float,
    ) -> str:
        # 野口追記：（2026-07-20、無限ループ対策。MONITORINGが途切れず
        # cycle_reset_sec続いたら「本当に復帰できた」とみなし、連続
        # 再突入カウントを0に戻す）
        if (
            self.monitoring_since is not None
            and now_sec - self.monitoring_since >= self.cycle_reset_sec
        ):
            self.consecutive_recovery_count = 0
            self.escape_turn_attempt_count = 0
            self.escape_turn_steer_sign = -1.0
            self.monitoring_since = None

        stopped_while_commanded_forward = (
            abs(measured_speed) <= self.stop_speed_threshold
            and previous_target_speed >= self.min_forward_command_speed
        )
        if not stopped_while_commanded_forward:
            self.stopped_since = None
            return self.state

        if self.stopped_since is None or now_sec < self.stopped_since:
            self.stopped_since = now_sec
            return self.state

        if now_sec - self.stopped_since >= self.stuck_detection_sec:
            self.stopped_since = None
            self.consecutive_recovery_count += 1
            # とにかくまず後退して前方にスペースを開ける。
            self._enter_move(now_sec, RECOVERY_STATE_BACKING_UP)

        return self.state

    def _update_move(
        self,
        now_sec: float,
        measured_speed: float,
        distance_since_move_entry_m: float,
        forward_is_clear: bool,
        rear_is_blocked: bool,
        rear_wall_is_blocked: bool,
    ) -> str:
        if self.move_since is None or now_sec < self.move_since:
            # シミュレーション時刻の巻き戻り時は、不定時間動かし続けない。
            self.reset(now_sec)
            return self.state

        elapsed = now_sec - self.move_since
        is_forward = self.state == RECOVERY_STATE_ALIGNING_FORWARD
        if not is_forward and rear_wall_is_blocked:
            self._enter_stopping(now_sec, RECOVERY_STATE_STOPPING_TO_FORWARD)
            return self.state
        timeout = (
            self.aligning_forward_timeout_sec if is_forward
            else self.backing_up_timeout_sec
        )

        if elapsed >= timeout:
            if is_forward:
                self.reset(now_sec)
            else:
                self._enter_stopping(now_sec, RECOVERY_STATE_STOPPING_TO_FORWARD)
            return self.state

        if elapsed < self.grace_sec:
            # 加速し始めの短時間はまだ判定しない（速度0付近を誤ってstuck扱いしない）。
            self.move_stuck_since = None
            self.move_stuck_since_distance_m = None
            self.move_blocked_since = None
            return self.state

        if is_forward:
            # 野口追記：（2026-07-20、簡略化。「向き+速度が十分良くなったら
            # 早期にMPCへ戻す」という品質判定は撤廃した——実速度が閾値の
            # わずか手前で壁に当たり続けたり、逆に判定が緩すぎて向きが
            # 全く揃っていないまま即MPCへ戻すバグを両方経験した。代わりに
            # 「前方が塞がった」か「実際に進めていない」かのいずれかで、
            # 良し悪しを問わず無条件でMPCへ戻す。まだ足りなければ外側の
            # スタック検知（_update_monitoring）が3秒後にまた拾う）
            forward_is_blocked = not forward_is_clear
            if forward_is_blocked:
                if self.move_blocked_since is None:
                    self.move_blocked_since = now_sec
                elif (
                    now_sec - self.move_blocked_since
                    >= self.aligning_forward_blocked_hold_sec
                ):
                    self.reset(now_sec)
                    return self.state
            else:
                self.move_blocked_since = None

            # 野口追記：（前進復帰の加速度はa_max=レギュレーション上限で頭打ちの
            # ため、速度が閾値を超えるまで時間がかかる。速度ではなく
            # stuck_hold_sec窓内の実移動距離でスタックを判定し、じわじわでも
            # 進めているうちは打ち切らない）
            if self.move_stuck_since is None:
                self.move_stuck_since = now_sec
                self.move_stuck_since_distance_m = distance_since_move_entry_m
            elif now_sec - self.move_stuck_since >= self.aligning_forward_stuck_hold_sec:
                progress_m = (
                    distance_since_move_entry_m
                    - cast(float, self.move_stuck_since_distance_m)
                )
                if progress_m < self.aligning_forward_min_progress_m:
                    self.reset(now_sec)
                else:
                    # 十分進んでいるので判定窓を仕切り直して様子を見続ける。
                    self.move_stuck_since = now_sec
                    self.move_stuck_since_distance_m = distance_since_move_entry_m
            return self.state

        # 野口追記：（「これ以上下がれない」の判定を、実速度の継続監視に加えて、
        # 後方の他車・壁ギャップの先読み（rear_is_blocked）でも検知する。
        # 単発tickでの中断は過去にライブロックを起こしたため、こちらも
        # stuck_hold_secで継続確認してから中断する——単発判定はしない）
        if abs(measured_speed) <= self.stop_speed_threshold or rear_is_blocked:
            if self.move_stuck_since is None:
                self.move_stuck_since = now_sec
            elif now_sec - self.move_stuck_since >= self.backing_up_stuck_hold_sec:
                self._enter_stopping(now_sec, RECOVERY_STATE_STOPPING_TO_FORWARD)
        else:
            self.move_stuck_since = None

        return self.state

    def _update_stopping(
        self,
        now_sec: float,
        measured_speed: float,
        ready_for_handback: bool = False,
    ) -> str:
        if self.stopping_since is None or now_sec < self.stopping_since:
            self.reset(now_sec)
            return self.state

        if abs(measured_speed) > self.reverse_stop_speed_threshold:
            # 一度しきい値内に入っても再び動いた場合は、停止確認をやり直す。
            self.stationary_since = None
            return self.state

        if self.stationary_since is None:
            self.stationary_since = now_sec
            return self.state

        if now_sec < self.stationary_since:
            self.reset(now_sec)
            return self.state

        if now_sec - self.stationary_since >= self.reverse_stop_hold_sec:
            self.stopping_since = None
            self.stationary_since = None
            # 野口追記：（2026-07-21、ユーザー提案。バック直後の時点で既に
            # コース進行方向に沿い、コース中央にも近ければ、ALIGNING_FORWARDを
            # 経由せず直接MPCへ戻す。判定はSTOPPING_TO_FORWARDのこの一度きり
            # （停止した静止状態での判定なので、ALIGNING_FORWARD中の走行中に
            # 判定するより単発誤反応のリスクが低い）——満たさなければ従来通り
            # ALIGNING_FORWARDで向き直しながら前進する）
            if ready_for_handback:
                self.reset(now_sec)
            # 野口追記：（2026-07-20、無限ループ対策。通常サイクル
            # （後退→センターライン狙いの前進）がnormal_cycle_limit回
            # 連続で解決しなかったら、次はESCAPE_TURN（固定角度で右へ）
            # を使う。エスカレーションしたら、通常戦略にもまた
            # チャンスを与えるためカウントを0に戻す）
            elif self.consecutive_recovery_count > self.normal_cycle_limit:
                self.consecutive_recovery_count = 0
                self.escape_turn_attempt_count += 1
                self.escape_turn_steer_sign = (
                    -1.0 if self.escape_turn_attempt_count % 2 == 1 else 1.0
                )
                self._enter_move(now_sec, RECOVERY_STATE_ESCAPE_TURN)
            else:
                self._enter_move(now_sec, RECOVERY_STATE_ALIGNING_FORWARD)

        return self.state

    def _update_escape_turn(self, now_sec: float) -> str:
        """固定角度・固定時間の脱出旋回。判定なしで、経過後は無条件でMPCへ戻す。"""
        if self.move_since is None or now_sec < self.move_since:
            self.reset(now_sec)
            return self.state

        if now_sec - self.move_since >= self.escape_turn_sec:
            self.reset(now_sec)

        return self.state


def propose_stuck_recovery_update(
    *,
    recovery: StuckRecovery,
    now_sec: float,
    measured_speed: float,
    previous_target_speed: float,
    recovery_allowed: bool,
    distance_since_move_entry_m: float = 0.0,
    forward_is_clear: bool = False,
    rear_is_blocked: bool = False,
    rear_wall_is_blocked: bool = False,
    ready_for_handback: bool = False,
    reverse_motion_limit_reached: bool = False,
    forward_motion_limit_reached: bool = False,
) -> StuckRecoveryUpdateProposal:
    """Run the existing state machine once on a same-type temporary value."""

    if not isinstance(recovery, StuckRecovery):
        raise TypeError("recovery preview requires StuckRecovery")
    before = recovery.snapshot()
    staged = replace(recovery)
    staged.update(
        now_sec=now_sec,
        measured_speed=measured_speed,
        previous_target_speed=previous_target_speed,
        recovery_allowed=recovery_allowed,
        distance_since_move_entry_m=distance_since_move_entry_m,
        forward_is_clear=forward_is_clear,
        rear_is_blocked=rear_is_blocked,
        rear_wall_is_blocked=rear_wall_is_blocked,
        ready_for_handback=ready_for_handback,
        reverse_motion_limit_reached=reverse_motion_limit_reached,
        forward_motion_limit_reached=forward_motion_limit_reached,
    )
    return StuckRecoveryUpdateProposal(before=before, after=staged.snapshot())
