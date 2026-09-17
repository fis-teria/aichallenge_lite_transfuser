# 野口追記：（AWSIMブースト条件をROSやMPC本体から分離）
"""ROSやMPC本体に依存しない、AWSIMブースト判定用の純粋関数。"""

from math import hypot
from typing import Iterable, Optional, Sequence


BOOST_MODE_START = "START"
BOOST_MODE_NORMAL = "NORMAL"


class AwsimRaceLifecycleState:
    """Own AWSIM vehicle-state evidence and the derived race-start lifetime.

    Vehicle state, gate result and start time remain separate named commits.
    The Controller has observable launch-guard and clock work between those
    stages, so this owner does not make the callback artificially atomic.
    """

    def __init__(self) -> None:
        self._vehicle_state: Optional[str] = None
        self._ready_seen = False
        self._race_started = False
        self._start_generation = 0
        self._started_at_sec: Optional[float] = None

    @property
    def vehicle_state(self) -> Optional[str]:
        return self._vehicle_state

    @property
    def ready_seen(self) -> bool:
        return self._ready_seen

    @property
    def race_started(self) -> bool:
        return self._race_started

    @property
    def start_generation(self) -> int:
        return self._start_generation

    @property
    def started_at_sec(self) -> Optional[float]:
        return self._started_at_sec

    def record_vehicle_state(self, *, vehicle_state: str) -> Optional[str]:
        previous_vehicle_state = self._vehicle_state
        self._vehicle_state = vehicle_state
        return previous_vehicle_state

    def record_gate(self, *, race_started: bool, ready_seen: bool) -> None:
        if race_started and not self._race_started:
            self._start_generation += 1
        self._race_started = race_started
        self._ready_seen = ready_seen

    def record_started_at_sec(self, *, started_at_sec: float) -> None:
        self._started_at_sec = started_at_sec

    def clear_started_at_sec(self) -> None:
        self._started_at_sec = None


class AwsimBoostStatusState:
    """Own the staged status values delivered by one AWSIM callback.

    The legacy callback marks receipt before converting the remaining-use and
    active fields.  Keep those commits separate so a malformed input cannot
    silently change the exceptional partial state while ownership is moved.
    """

    def __init__(self) -> None:
        self._received = False
        self._uses_left = 0.0
        self._is_boosting = False

    @property
    def received(self) -> bool:
        return self._received

    @property
    def uses_left(self) -> float:
        return self._uses_left

    @property
    def is_boosting(self) -> bool:
        return self._is_boosting

    def mark_received(self) -> None:
        self._received = True

    def record_uses_left(self, *, uses_left: float) -> None:
        self._uses_left = uses_left

    def record_is_boosting(self, *, is_boosting: bool) -> None:
        self._is_boosting = is_boosting


class BoostRequestState:
    """Own the selected boost mode and its independently suppressible request.

    The legacy Controller normally updates the mode and request together, but
    its pre-initialization guard clears only the request while retaining the
    previous mode.  That asymmetric state is kept explicit here so ownership
    can be consolidated without silently changing the old transition.
    """

    def __init__(self) -> None:
        self._requested = False
        self._mode: Optional[str] = None

    @property
    def requested(self) -> bool:
        return self._requested

    @property
    def mode(self) -> Optional[str]:
        return self._mode

    def record_selected_mode(self, *, mode: Optional[str]) -> None:
        self._mode = mode
        self._requested = mode is not None

    def clear(self) -> None:
        self._requested = False
        self._mode = None

    def suppress_preserving_mode(self) -> None:
        self._requested = False


class BoostCommandAckState:
    """Own the command-pending latch and the timestamp that makes it valid."""

    def __init__(self) -> None:
        self._pending = False
        self._sent_at_sec: Optional[float] = None

    @property
    def pending(self) -> bool:
        return self._pending

    @property
    def sent_at_sec(self) -> Optional[float]:
        return self._sent_at_sec

    def record_sent(self, *, sent_at_sec: float) -> None:
        self._pending = True
        self._sent_at_sec = sent_at_sec

    def clear(self) -> None:
        self._pending = False
        self._sent_at_sec = None


class BoostUsageState:
    """Own the local use budget and the first START-mode permission."""

    def __init__(self, *, uses_left: int) -> None:
        self._uses_left = uses_left
        self._start_pending = True

    @property
    def uses_left(self) -> int:
        return self._uses_left

    @property
    def start_pending(self) -> bool:
        return self._start_pending

    def consume_use(self) -> None:
        self._uses_left -= 1

    def mark_start_consumed(self) -> None:
        self._start_pending = False

    def reset(self, *, uses_left: int) -> None:
        self._uses_left = uses_left
        self._start_pending = True


def race_release_is_ready(*, ready_seen: bool, race_started: bool) -> bool:
    """Treat READY as grid release; Start remains a robust fallback."""

    return bool(ready_seen or race_started)


def update_race_start_gate(
    previous_started: bool,
    ready_seen: bool,
    vehicle_state: str,
    previous_vehicle_state: Optional[str] = "",
) -> tuple[bool, bool]:
    """Start/Readyの両方を受信後、(開始済み, Ready確認済み)を返す。"""
    state = vehicle_state.strip().lower()
    previous_state = (previous_vehicle_state or "").strip().lower()

    # AWSIMの版により解放順が Grounded->Ready->Start と
    # Grounded->Start->Ready のどちらにもなる。Grounded直後のStart単独では
    # 開始せず、Readyとの組を確認した時点だけゲートを開く。
    if state in ("spawned", "grounded", "finish"):
        return False, False
    if state == "ready":
        return previous_started or previous_state == "start", True
    if state == "start":
        return ready_seen, ready_seen
    return previous_started, ready_seen


# 野口追記：（V2Xが配信されない単独走行でも2回目ブーストを許可する判定）
def is_v2x_ready_for_normal_boost(
    *,
    v2x_received: bool,
    race_started: bool,
    race_elapsed_sec: Optional[float],
    solo_fallback_sec: float,
) -> bool:
    """V2X受信済み、またはレース開始後の単独走行待ち時間経過でTrue。"""
    if solo_fallback_sec < 0.0:
        raise ValueError("solo_fallback_sec must be non-negative")
    if v2x_received:
        return True
    return (
        race_started
        and race_elapsed_sec is not None
        and race_elapsed_sec >= solo_fallback_sec
    )


def _point_to_segment_distance_sq(
    point: Sequence[float],
    segment_start: Sequence[float],
    segment_end: Sequence[float],
) -> float:
    """点から線分への最短距離の二乗を返す。"""
    px, py = float(point[0]), float(point[1])
    ax, ay = float(segment_start[0]), float(segment_start[1])
    bx, by = float(segment_end[0]), float(segment_end[1])
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return (px - ax) ** 2 + (py - ay) ** 2

    # 線分方向への射影係数を[0, 1]に制限し、線分上の最近傍点を求める。
    ratio = ((px - ax) * dx + (py - ay) * dy) / length_sq
    ratio = min(max(ratio, 0.0), 1.0)
    nearest_x = ax + ratio * dx
    nearest_y = ay + ratio * dy
    return (px - nearest_x) ** 2 + (py - nearest_y) ** 2


def is_horizon_clear(
    prediction_xy: Iterable[Sequence[float]],
    obstacle_centers: Iterable[Sequence[float]],
    clearance: float,
) -> bool:
    """太らせたMPC予測折れ線に、他車中心が接触していなければTrueを返す。"""
    if clearance < 0.0:
        raise ValueError("clearance must be non-negative")

    prediction = list(prediction_xy)
    # 予測経路が無い状態を「空いている」と誤判定して発火しないようFalseにする。
    if not prediction:
        return False

    obstacles = list(obstacle_centers)
    if not obstacles:
        return True

    clearance_sq = clearance * clearance
    if len(prediction) == 1:
        px, py = float(prediction[0][0]), float(prediction[0][1])
        return all(
            hypot(float(obstacle[0]) - px, float(obstacle[1]) - py) > clearance
            for obstacle in obstacles
        )

    # 予測点だけでなく点間の線分も調べ、疎な予測点の間にいる他車を見落とさない。
    for obstacle in obstacles:
        for start, end in zip(prediction, prediction[1:]):
            if _point_to_segment_distance_sq(obstacle, start, end) <= clearance_sq:
                return False
    return True


def is_straight_horizon(
    curvatures: Iterable[float], max_abs_curvature: float
) -> bool:
    """ホライズン内の全曲率が閾値以下なら直線と判定する。"""
    if max_abs_curvature < 0.0:
        raise ValueError("max_abs_curvature must be non-negative")
    values = list(curvatures)
    return bool(values) and all(
        abs(float(curvature)) <= max_abs_curvature for curvature in values)


def select_boost_request_mode(
    *,
    start_pending: bool,
    prediction_ready: bool,
    v2x_received: bool,
    horizon_clear: bool,
    straight: bool,
    local_uses_left: int,
    current_lap: int,
) -> Optional[str]:
    """初回を優先し、現在要求すべきブースト種別を返す。"""
    if isinstance(current_lap, bool) or not isinstance(current_lap, int):
        raise TypeError("current_lap must be int")
    if current_lap < 1:
        raise ValueError("current_lap must be at least 1")
    if local_uses_left <= 0:
        return None

    # START要求はpendingのまま保持できるため、安全ゲートが開くまで待つ。
    # スタートグリッド上の他車が予測経路を塞いでいる状態で発火すると、
    # 約10秒継続するブースト中に回避・制動が間に合わない。
    if start_pending:
        if prediction_ready and v2x_received and horizon_clear and straight:
            return BOOST_MODE_START
        return None

    # 2回目のNORMALはLap 2以降で、従来の安全ゲートもすべて要求する。
    if current_lap < 2:
        return None
    if not prediction_ready or not v2x_received or not horizon_clear:
        return None
    if straight:
        return BOOST_MODE_NORMAL
    return None


def can_start_boost(
    *,
    requested: bool,
    race_started: bool,
    current_speed_mps: float,
    min_start_speed_mps: float,
    local_uses_left: int,
    status_received: bool,
    awsim_uses_left: float,
    is_boosting: bool,
    command_pending: bool,
    require_min_speed: bool = True,
) -> bool:
    """すべての安全・回数・状態ゲートが開いている場合だけ発火を許可する。"""
    return (
        requested
        and race_started
        # 野口追記：（初回STARTでは停止中や微小な負速度ノイズでも発火を許可）
        and (not require_min_speed or current_speed_mps >= min_start_speed_mps)
        and local_uses_left > 0
        and status_received
        and awsim_uses_left > 0.0
        and not is_boosting
        and not command_pending
    )
