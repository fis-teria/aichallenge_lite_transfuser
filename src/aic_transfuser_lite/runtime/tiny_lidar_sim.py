"""Official Tiny steering adapter and small SIM-ONLY speed/stop policy.

No V4 inputs, history, trajectory fitting, MPC, teacher, or ROS dependencies.
The network and scan preprocessing are imported unchanged from the pinned
official package. A trusted file digest is checked BEFORE its pickle is opened.
"""
from __future__ import annotations

import hashlib
import importlib
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

OFFICIAL_COMMIT = "1f54dff995d02625566341f9e1be1c39369224f2"
WEIGHT_SHA256 = "7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963"
TINY_AUTH_PROFILE = "TINY_READY_LAP_20260907"
AUTH_REQUEST_SHA256 = "3210c97ce30a8af18abb9633378a0c9993f55c01f2231dda262142f17a646bcd"
TINY_GUI_PROFILE = "TINY_GUI_SHORT_20260907"
GUI_AUTH_SHA256 = "b7d722c01979958ed92aa2e2d4cf21b8098d6edb56ed27b086eea106ef6283a7"
TINY_GUI_RETRY_PROFILE = "TINY_GUI_RETRY_20260907"
GUI_RETRY_AUTH_SHA256 = "90b60a4c99c6e17e490454a9410755b33b1f5fa36e5b3c29f27d46d69795156d"
TINY_GUI_RETRY2_PROFILE = "TINY_GUI_RETRY2_20260907"
GUI_RETRY2_AUTH_SHA256 = "599eee1622043222bef50de1288de693ec192055809d039f175d2eb26f9bef8b"
TINY_GUI_LAP_PROFILE = "TINY_GUI_LAP_20260907"
GUI_LAP_AUTH_SHA256 = "a799ad8000a8540660a7d905abbe3b2fa59a708be3928c1395f1bfb2001b62be"
GUI_AUTHORIZATIONS = {
    TINY_GUI_LAP_PROFILE: (GUI_LAP_AUTH_SHA256, "configs/control/tiny_gui_lap_authorization_20260907.json"),
    TINY_GUI_PROFILE: (GUI_AUTH_SHA256, "configs/control/tiny_gui_authorization_20260907.json"),
    TINY_GUI_RETRY_PROFILE: (GUI_RETRY_AUTH_SHA256, "configs/control/tiny_gui_retry_authorization_20260907.json"),
    TINY_GUI_RETRY2_PROFILE: (GUI_RETRY2_AUTH_SHA256, "configs/control/tiny_gui_retry2_authorization_20260907.json"),
}
GUI_CONTROL_METHOD = "tiny_lidar_net_guarded"
# 2026-09-07 09:50 JST, never rolled forward to another date.
DRIVE_CUTOFF_UNIX_S = 1788742200
SOURCE_SHA256 = {
    "__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "tiny_lidar_net_controller_core.py": "1bc7dbfee5d2e37861cc3245056654a0e6b004a2b1267a6bb8941d256d67b5a2",
    "model/__init__.py": "3ce6da246c8b1e131579462acb9c0eea44f48a313fdc3b44374fe9c0cd33e43d",
    "model/tinylidarnet.py": "1ebdf28ed2b42292400df262087fddf271b0834434d77136edffba1b705721c1",
    "model/numpy/initializers.py": "b5521071726d07b7a5f3976a66c23853fa642fe15f21b78fd2a0c3e10cf1d10d",
    "model/numpy/layers.py": "12c846af37938e10681d3c141ddfce510bee64c7c95120a61259dd47f2e2e0d4",
}


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(part)
    return h.hexdigest()


def strict_parameters(weights: dict, expected: dict) -> dict:
    """Require every official parameter, exact shape, float32, and finite data."""
    if not isinstance(weights, dict) or not all(isinstance(k, str) for k in weights):
        raise ValueError("WEIGHT_DICTIONARY")
    normalized = {key.replace(".", "_"): value for key, value in weights.items()}
    if len(normalized) != len(weights) or set(normalized) != set(expected):
        raise ValueError("WEIGHT_KEYS_NOT_EXACT")
    result = {}
    for key, target in expected.items():
        value = normalized[key]
        if not isinstance(value, np.ndarray) or value.shape != target.shape:
            raise ValueError("WEIGHT_SHAPE:" + key)
        if value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError("WEIGHT_DTYPE_OR_FINITE:" + key)
        result[key] = value
    return result


class OfficialTiny:
    """Raw LaserScan ranges [750] metres -> official steering scalar radians."""

    def __init__(self, package: Path):
        code = package / "tiny_lidar_net_controller"
        weights = package / "ckpt/tinylidarnet_weights.npy"
        if digest(weights) != WEIGHT_SHA256:
            raise ValueError("UNTRUSTED_WEIGHT_BEFORE_DESERIALIZATION")
        for relative, sha in SOURCE_SHA256.items():
            if digest(code / relative) != sha:
                raise ValueError("OFFICIAL_SOURCE_CHANGED:" + relative)
        # Official core uses absolute `from model...`. Run it in a dedicated
        # spawned worker so no other architecture can own this module name.
        for name in ("model", "tiny_lidar_net_controller_core"):
            if name in sys.modules:
                raise ValueError("OFFICIAL_MODULE_NAMESPACE_ALREADY_USED:" + name)
        sys.path.insert(0, str(code.resolve()))
        module = importlib.import_module("tiny_lidar_net_controller_core")
        prototype = module.TinyLidarNetNp(input_dim=750, output_dim=2)
        stored = np.load(weights, allow_pickle=True).item()
        validated = strict_parameters(stored, prototype.params)
        self.core = module.TinyLidarNetCore(input_dim=750, output_dim=2,
            architecture="normal", ckpt_path=str(weights), max_range=30.0,
            control_mode="fixed", acceleration=0.0)
        if len(validated) != 18:
            raise ValueError("OFFICIAL_PARAMETER_COUNT")
        for key, value in validated.items():
            if not np.array_equal(self.core.model.params[key], value):
                raise ValueError("OFFICIAL_LOADER_POSTCHECK:" + key)
        self.identity = dict(official_commit=OFFICIAL_COMMIT, weight_sha256=WEIGHT_SHA256,
            source_sha256=SOURCE_SHA256, architecture="normal", input_shape=[1, 1, 750],
            output_order=["UNUSED_ACCELERATION", "STEERING_RAD"], max_range_m=30.0,
            keys_shapes={key: list(value.shape) for key, value in validated.items()},
            loaded_parameter_tensors=len(validated), loaded_parameter_elements=sum(v.size for v in validated.values()),
            all_keys_shapes_finite_exact=True, partial_load=False, official_core_unchanged=True,
            fixed_acceleration_override=0.0, acceleration_from_model_used=False)

    def process(self, ranges: np.ndarray) -> float:
        if ranges.shape != (750,) or ranges.dtype != np.float32:
            raise ValueError("SCAN_ARRAY_SHAPE_DTYPE")
        _, steering = self.core.process(ranges)
        if not math.isfinite(steering):
            raise ValueError("NONFINITE_TINY_STEERING")
        return steering


def validate_scan(scan: dict) -> None:
    """Check observed sensor contract; cleaning stays solely in official core."""
    a = scan["ranges"]
    if a.shape != (750,) or a.dtype != np.float32:
        raise ValueError("SCAN_750_FLOAT32_REQUIRED")
    expected = {"angle_min": -1.5666074752807617, "angle_increment": .004188789986073971,
                "range_min": 0., "range_max": 25.}
    for key, value in expected.items():
        if not math.isfinite(scan[key]) or abs(scan[key]-value) > 1e-5:
            raise ValueError("SCAN_CONTRACT:" + key)
    if not scan["frame"]:
        raise ValueError("SCAN_FRAME_MISSING")
    if not np.any(np.isfinite(a) & (a > 0) & (a <= scan["range_max"])):
        raise ValueError("SCAN_NO_FINITE_RETURN")


def speed_acceleration(speed_mps: float, *, target_mps: float, maximum_mps: float,
                       stop: bool = False) -> float:
    """SIM-only proportional speed policy, SI units, NOT learned acceleration.

    Consumer ignores desired-speed field. Apply bounded acceleration directly.
    Negative speed is a fault, never accelerated toward zero using reverse gear.
    Near standstill use zero, not a persistent negative acceleration reversal.
    """
    if not all(math.isfinite(v) for v in (speed_mps, target_mps, maximum_mps)):
        raise ValueError("NONFINITE_SPEED")
    if not 0 < target_mps < maximum_mps <= 2.5:
        raise ValueError("SIM_SPEED_CONFIG")
    if speed_mps < -.03:
        raise ValueError("REVERSE_MOTION_FREEZE")
    if stop:
        return -min(1.0, max(0., speed_mps * 3.0)) if speed_mps > .03 else 0.0
    if speed_mps > maximum_mps:
        raise ValueError("OVERSPEED")
    return float(np.clip(2.0 * (target_mps-speed_mps), -1.0, .60))


def validate_operation(result: dict, *, now_ns: int, sim_ns: int, steering_limit_rad: float) -> None:
    if not 0 <= now_ns-result["received_ns"] <= 500_000_000:
        raise ValueError("TINY_OPERATION_WALL_EXPIRED")
    if not 0 <= sim_ns-result["source_ns"] <= 350_000_000:
        raise ValueError("TINY_OPERATION_SIM_EXPIRED")
    steer = result["steering_rad"]
    if not math.isfinite(steer) or abs(steer) > steering_limit_rad:
        raise ValueError("TINY_STEERING_PHYSICAL_LIMIT")


def input_clock_ready(record: dict, *, now_ns: int, sim_ns: int) -> bool:
    """A source report may arrive before /clock; wait, never rewrite its stamp.

    Do not accept a future-stamped sample for actuation. Bounded 100ms lead is
    only a transport wait, NOT extended operation validity or interpolation.
    """
    if not 0 <= now_ns-record["received_ns"] <= 500_000_000:
        raise ValueError("INPUT_WALL_STALE")
    age = sim_ns-record["source_ns"]
    if age < -100_000_000 or age > 350_000_000:
        raise ValueError("INPUT_SIM_STALE_OR_CLOCK_DOMAIN")
    return age >= 0


class SimReadiness:
    """Existing AWSIM Ready notification gates first positive send, not ML input."""
    def __init__(self):
        self.ready_sim_ns: int | None = None
        self.ready_received_ns: int | None = None
        self.ready_scan_sequence: int | None = None

    def observe(self, phase: str, sim_ns: int, received_ns: int, scan_sequence: int) -> None:
        if phase == "Ready" and self.ready_sim_ns is None and sim_ns >= 0:
            if type(received_ns) is not int or type(scan_sequence) is not int or min(received_ns, scan_sequence) < 0:
                raise ValueError("READY_RECEIPT_FENCE_INVALID")
            self.ready_sim_ns = sim_ns
            self.ready_received_ns = received_ns
            self.ready_scan_sequence = scan_sequence
        elif self.ready_sim_ns is not None and phase in ("Spawned", "Grounded"):
            raise ValueError("SIM_REINITIALIZED_AFTER_READY")

    def allow_inference(self, completed: int, stationary: bool) -> bool:
        return stationary or self.ready_sim_ns is not None or completed < 3

    def allow_drive(self, result: dict | None) -> bool:
        return bool(self.ready_sim_ns is not None and result is not None
                    and result["source_ns"] >= self.ready_sim_ns
                    and result["received_ns"] > self.ready_received_ns
                    and result["scan_sequence"] > self.ready_scan_sequence)


def finite_number(value: Any, name: str, *, integer: bool = False) -> float:
    if (type(value) not in (int, float) or not math.isfinite(value) or value < 0
            or (integer and type(value) is not int)):
        raise ValueError("INVALID_NONNEGATIVE_FINITE_NUMBER:"+name)
    return value


def tiny_phase_caps(phase: str, profile: str = TINY_AUTH_PROFILE) -> dict:
    """Fresh per-call limits, scoped solely to the explicit Tiny authorization."""
    caps = {
        "stationary": dict(wall_seconds=120, forward_limit=12, single_episode_sim_limit_s=0.),
        "short": dict(wall_seconds=120, forward_limit=300, single_episode_sim_limit_s=20.),
        "lap": dict(wall_seconds=600, forward_limit=5200, single_episode_sim_limit_s=240.),
    }
    if profile == TINY_GUI_LAP_PROFILE:
        caps = {"lap": caps["lap"]}
    elif profile in GUI_AUTHORIZATIONS:
        caps = {"short": dict(wall_seconds=120, forward_limit=600, single_episode_sim_limit_s=20.)}
    elif profile != TINY_AUTH_PROFILE:
        raise ValueError("UNKNOWN_TINY_PROFILE")
    if phase not in caps:
        raise ValueError("UNKNOWN_TINY_PHASE")
    return caps[phase].copy()


def validate_tiny_config(cfg: dict) -> None:
    """Shared host/runtime entry check. No boolean/NaN budgets or policy drift."""
    profile = cfg.get("authorization_profile")
    expected_hash = {TINY_AUTH_PROFILE: AUTH_REQUEST_SHA256,
                     **{key: value[0] for key, value in GUI_AUTHORIZATIONS.items()}}.get(profile)
    if expected_hash is None or cfg.get("authorization_request_sha256") != expected_hash:
        raise ValueError("EXPLICIT_TINY_AUTHORIZATION_PROFILE_REQUIRED")
    caps = tiny_phase_caps(cfg["phase"], profile)
    if profile in GUI_AUTHORIZATIONS and cfg.get("control_method") != GUI_CONTROL_METHOD:
        raise ValueError("GUI_CONTROL_METHOD_REQUIRED")
    for key, maximum in caps.items():
        value = finite_number(cfg[key], key, integer=key == "forward_limit")
        if value > maximum or (key != "single_episode_sim_limit_s" and value <= 0):
            raise ValueError("TINY_PHASE_CAP:"+key)
    if cfg["wall_seconds"] < 10:
        raise ValueError("TINY_RUNTIME_WALL_TOO_SHORT")
    if cfg["phase"] == "short" and cfg["single_episode_sim_limit_s"] < 14:
        raise ValueError("SHORT_BRAKING_RESERVE_MISSING")
    if cfg["phase"] == "lap" and cfg["single_episode_sim_limit_s"] <= 6:
        raise ValueError("LAP_BRAKING_RESERVE_MISSING")
    fixed = dict(target_speed_mps=2., maximum_speed_mps=2.4, command_period_s=.05,
        steering_limit_rad=math.pi/6, scan_wall_expiry_s=.5, scan_sim_expiry_s=.35,
        stop_wall_seconds=6., short_motion_sim_seconds=8., stationary_forwards=8)
    for key, expected in fixed.items():
        value = finite_number(cfg[key], key, integer=key == "stationary_forwards")
        if not math.isclose(value, expected, rel_tol=0., abs_tol=1e-12):
            raise ValueError("FIXED_TINY_CONTROL_POLICY_CHANGED:"+key)
    if cfg.get("official_commit") != OFFICIAL_COMMIT or cfg.get("weight_sha256") != WEIGHT_SHA256:
        raise ValueError("TINY_CONFIG_MODEL_IDENTITY")
    for key in ("physical_vehicle_authorized", "awsim_modification_authorized", "v4_or_mpc_used"):
        if cfg.get(key) is not False:
            raise ValueError("SIM_ONLY_POLICY_CHANGED:"+key)


def bounded_runtime_wall(phase: str, requested: float, available_wall: float, now_unix: float,
                         profile: str = TINY_AUTH_PROFILE) -> int:
    cap = tiny_phase_caps(phase, profile)["wall_seconds"]
    for name, value in (("requested", requested), ("available_wall", available_wall), ("now_unix", now_unix)):
        finite_number(value, name)
    if not 10 <= requested <= cap:
        raise ValueError("PHASE_RUNTIME_WALL_CAP")
    wall = math.floor(min(requested, available_wall-110., DRIVE_CUTOFF_UNIX_S-now_unix-110.))
    if wall < 10:
        raise ValueError("NO_FINITE_RESERVATION_BEFORE_CUTOFF")
    return wall


def motion_limit_reason(cfg: dict, powered_sim_seconds: float) -> str | None:
    finite_number(powered_sim_seconds, "powered_sim_seconds")
    limit = cfg["short_motion_sim_seconds"] if cfg["phase"] == "short" else cfg["single_episode_sim_limit_s"]-6.
    if powered_sim_seconds >= limit:
        return "SHORT_MOTION_COMPLETE" if cfg["phase"] == "short" else "EPISODE_LIMIT_NO_LAP"
    return None


def host_arm_status(arm: dict, *, project: str, observed_ns: int) -> dict:
    """Compare only a post-read monotonic time; expose the precise failed branch."""
    stamp = arm.get("monotonic_ns")
    age = observed_ns-stamp if type(stamp) is int else None
    reason = ("IDENTITY" if arm.get("token") != project else
              "NOT_ARMED" if arm.get("armed") is not True else
              "STOP_PROOF_MISSING" if arm.get("paused_kill_verified") is not True else
              "STAMP_INVALID" if age is None else
              "FUTURE_STAMP" if age < 0 else
              "EXPIRED" if age >= 750_000_000 else "OK")
    return dict(allowed=reason == "OK", reason=reason, observed_ns=observed_ns,
                arm_stamp_ns=stamp, age_ns=age, token=arm.get("token"),
                armed=arm.get("armed"), paused_kill_verified=arm.get("paused_kill_verified"))


def verify_container_contract(items: list[dict], project: str, *, gui: bool = False) -> None:
    """Pure check of current selected Docker inspect, before ROS initialization."""
    if len(items) != 2 or not project.startswith("codex-tiny-dev-"):
        raise ValueError("UNEXPECTED_COMPOSE_SCOPE")
    indexed = {c["Config"]["Labels"]["com.docker.compose.service"]: c for c in items}
    runtime_service = "autoware" if gui else "tiny"
    if set(indexed) != {"simulator", runtime_service}:
        raise ValueError("UNEXPECTED_SERVICES")
    allowed = {"/v4", "/evidence", "/official_tiny", "/aichallenge/simulator/AWSIM",
               "/aichallenge/run_simulator.bash", "/xvfb", "/tmp/.X11-unix", "/usr/bin/xkbcomp"}
    if gui:
        allowed = {"/v4", "/evidence", "/official_tiny", "/aichallenge/simulator/AWSIM",
                   "/aichallenge/run_simulator.bash", "/tmp/.X11-unix", "/desktop_xauth", "/tiny_install", "/.rviz2"}
    for name, c in indexed.items():
        h = c["HostConfig"]
        if (c["Config"]["Labels"].get("com.docker.compose.project") != project or h["Privileged"]
                or h["Devices"] or h["CapAdd"] or h["CapDrop"] != ["ALL"] or h["PidMode"]
                or h["IpcMode"] != "private" or "no-new-privileges:true" not in h["SecurityOpt"]):
            raise ValueError("UNSAFE_CONTAINER")
        if h["NetworkMode"] != ("none" if name == "simulator" else "container:"+indexed["simulator"]["Id"]):
            raise ValueError("UNSAFE_NETWORK")
        if c["Image"] != "sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7":
            raise ValueError("UNEXPECTED_RUNTIME_IMAGE")
        for mount in c["Mounts"]:
            if mount["Destination"] not in allowed:
                raise ValueError("UNEXPECTED_MOUNT")
            writable = {"/evidence", "/tmp/.X11-unix"}
            if gui and name == "autoware":
                writable.add("/.rviz2")
            if mount["RW"] and mount["Destination"] not in writable:
                raise ValueError("WRITABLE_SOURCE_OR_SIMULATOR")
        if gui:
            mounts = {m["Destination"]: m for m in c["Mounts"]}
            required = {"/v4", "/evidence", "/tmp/.X11-unix", "/desktop_xauth"}
            required |= {"/tiny_install", "/official_tiny"} if name == "autoware" else {
                "/aichallenge/simulator/AWSIM", "/aichallenge/run_simulator.bash"}
            if not required <= mounts.keys() or any(mounts[p]["RW"] for p in required-{"/evidence"}):
                raise ValueError("GUI_REQUIRED_READONLY_MOUNTS")
            if name == "autoware":
                rviz = mounts.get("/.rviz2", {})
                if (not rviz.get("RW") or rviz.get("Source") !=
                        str(Path(mounts["/evidence"]["Source"])/"rviz_config")):
                    raise ValueError("GUI_RVIZ_CONFIG_MUST_BE_OWN_EVIDENCE")
