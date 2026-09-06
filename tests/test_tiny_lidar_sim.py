"""Synthetic-only Tiny connection contracts; optional official load, no forward."""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from aic_transfuser_lite.runtime.tiny_lidar_sim import (
    OfficialTiny, strict_parameters, validate_scan, validate_operation,
    speed_acceleration, verify_container_contract, SOURCE_SHA256, WEIGHT_SHA256, input_clock_ready, SimReadiness,
    TINY_AUTH_PROFILE, AUTH_REQUEST_SHA256, DRIVE_CUTOFF_UNIX_S, tiny_phase_caps,
    validate_tiny_config, bounded_runtime_wall, motion_limit_reason,
)
from tiny_dev_runner import JudgeLog, TinyBudget, ASSETS, ORIGINAL_SCRIPT
from spatial_dev_host_v4 import HostWatch, cleanup_owned, AttemptBudget


def test_fixed_digest_format():
    assert all(len(value) == 64 and int(value, 16) >= 0
               for value in [*SOURCE_SHA256.values(), WEIGHT_SHA256, *ASSETS.values(), ORIGINAL_SCRIPT])


def test_weight_exact_and_normalization():
    a = np.ones((2, 3), dtype=np.float32)
    assert np.array_equal(strict_parameters({"layer.weight": a}, {"layer_weight": a})["layer_weight"], a)


@pytest.mark.parametrize("variant", ["missing", "extra", "collision", "shape", "dtype", "nan"])
def test_weight_reject(variant):
    a = np.ones((2, 3), dtype=np.float32)
    data = {"layer.weight": a}
    if variant == "missing": data = {}
    elif variant == "extra": data["another"] = a
    elif variant == "collision": data["layer_weight"] = a
    elif variant == "shape": data["layer.weight"] = a.T
    elif variant == "dtype": data["layer.weight"] = a.astype(np.float64)
    elif variant == "nan": data["layer.weight"][0, 0] = np.nan
    with pytest.raises(ValueError): strict_parameters(data, {"layer_weight": a})


def scan():
    return dict(ranges=np.full(750, 10., np.float32), angle_min=-1.5666074752807617,
        angle_increment=.004188789986073971, range_min=0., range_max=25., frame="observed_lidar_frame")


def test_scan_preserves_raw_invalid_values_for_official_core():
    value = scan()
    value["ranges"][:3] = [np.nan, np.inf, -1.]
    original = value["ranges"].copy()
    validate_scan(value)
    np.testing.assert_array_equal(value["ranges"], original)


@pytest.mark.parametrize("kind", ["count", "reversed_angle", "normalizer_used_as_sensor_range", "empty", "frame"])
def test_scan_reject(kind):
    value = scan()
    if kind == "count": value["ranges"] = value["ranges"][:-1]
    if kind == "reversed_angle": value["angle_increment"] *= -1
    if kind == "normalizer_used_as_sensor_range": value["range_max"] = 30.
    if kind == "empty": value["ranges"][:] = np.inf
    if kind == "frame": value["frame"] = ""
    with pytest.raises(ValueError): validate_scan(value)


def test_speed_feedback_and_actual_acceleration_not_speed_field():
    cfg = dict(target_mps=2., maximum_mps=2.4)
    assert speed_acceleration(0., **cfg) == .6
    assert speed_acceleration(2., **cfg) == 0.
    assert speed_acceleration(2.3, **cfg) < 0.
    assert speed_acceleration(2., stop=True, **cfg) == -1.
    assert speed_acceleration(.1, stop=True, **cfg) == pytest.approx(-.3)
    assert speed_acceleration(.01, stop=True, **cfg) == 0.


@pytest.mark.parametrize("v", [float("nan"), float("inf"), -.1, 2.401])
def test_speed_fault(v):
    with pytest.raises(ValueError): speed_acceleration(v, target_mps=2., maximum_mps=2.4)


@pytest.mark.parametrize("change", [{"steering_rad": math.nan}, {"steering_rad": .53},
    {"received_ns": -500_000_001}, {"source_ns": -350_000_001}, {"source_ns": 1}])
def test_expiry_and_steer_fault(change):
    result = dict(received_ns=0, source_ns=0, steering_rad=.1)
    result.update(change)
    with pytest.raises(ValueError):
        validate_operation(result, now_ns=0, sim_ns=0, steering_limit_rad=math.pi/6)


def test_clock_callback_order_waits_without_rewriting_stamp():
    record = dict(source_ns=70_000_000, received_ns=1_000_000_000)
    assert not input_clock_ready(record, now_ns=1_000_000_001, sim_ns=65_000_000)
    assert input_clock_ready(record, now_ns=1_000_000_002, sim_ns=70_000_000)
    assert record["source_ns"] == 70_000_000
    with pytest.raises(ValueError):
        input_clock_ready(record, now_ns=1_500_000_001, sim_ns=70_000_000)
    with pytest.raises(ValueError):
        input_clock_ready(record, now_ns=1_000_000_001, sim_ns=421_000_000)


def test_ready_gate_does_not_drive_during_spawn_or_use_pre_ready_output():
    gate = SimReadiness()
    gate.observe("Spawned", -1, 1, 0)
    gate.observe("Grounded", 1_000_000_000, 10, 1)
    assert not gate.allow_drive(dict(source_ns=2_000_000_000, received_ns=11, scan_sequence=2))
    assert gate.allow_inference(2, False)
    assert not gate.allow_inference(3, False)
    assert gate.allow_inference(8, True)  # Stationary check does not drive.
    gate.observe("Ready", 6_000_000_000, 100, 120)
    assert not gate.allow_drive(dict(source_ns=5_000_000_000, received_ns=101, scan_sequence=121))
    assert gate.allow_drive(dict(source_ns=6_000_000_000, received_ns=101, scan_sequence=121))
    with pytest.raises(ValueError): gate.observe("Grounded", 7_000_000_000, 200, 122)


def test_pre_ready_same_stamp_or_queued_old_scan_cannot_drive():
    gate = SimReadiness()
    gate.observe("Ready", 6_000_000_000, 100_000_000, 120)
    old = dict(source_ns=6_000_000_000, received_ns=99_000_000, scan_sequence=120,
               finished_ns=110_000_000, steering_rad=.1)
    assert not gate.allow_drive(old)  # Ready-post worker completion is irrelevant.
    old["finished_ns"] = 120_000_000  # Requeue does not change the original receipt.
    assert not gate.allow_drive(old)
    new = dict(old, received_ns=101_000_000, scan_sequence=121)
    assert gate.allow_drive(new)
    validate_operation(new, now_ns=120_000_000, sim_ns=6_000_000_000, steering_limit_rad=math.pi/6)
    with pytest.raises(ValueError, match="EXPIRED"):
        validate_operation(new, now_ns=601_000_001, sim_ns=6_000_000_000, steering_limit_rad=math.pi/6)
    assert not gate.allow_drive(dict(new, scan_sequence=120))


def test_runtime_entrypoint_parses_without_ros_imports():
    import ast
    source = Path(__file__).resolve().parents[1]/"tools/run_tiny_lidar_dev.py"
    ast.parse(source.read_text())


def test_host_monitor_arms_before_power_and_remains_armed():
    watch = HostWatch("tiny-test")
    assert watch.check(None, 0) is None
    assert watch.check(dict(token="tiny-test", monotonic_ns=10, logger_ok=True, powered=False), 10) is None
    assert watch.armed
    assert watch.check(None, 11) == "HEARTBEAT_MISSING"
    assert watch.check(dict(token="tiny-test", monotonic_ns=10, logger_ok=True, powered=False), 750_000_011) == "HEARTBEAT_STALE"


def test_cleanup_does_not_unpause():
    from types import SimpleNamespace
    calls = []
    def run(argv, timeout):
        calls.append(argv)
        return SimpleNamespace(stdout=json.dumps(dict(Running=True, Paused=True)))
    result = cleanup_owned(run, "owned-sim", "owned-runtime", paused_kill_verified=True)
    assert not result["unpause_called"] and not any("unpause" in c for c in calls)
    assert ["docker", "kill", "--signal", "KILL", "owned-sim"] in calls


def containers():
    items = []
    for name in ("simulator", "tiny"):
        items.append(dict(Id=name, Image="sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7",
            Config=dict(Labels={"com.docker.compose.service":name, "com.docker.compose.project":"codex-tiny-dev-test"}),
            HostConfig=dict(Privileged=False, Devices=[], CapAdd=None, CapDrop=["ALL"], PidMode="",
                IpcMode="private", SecurityOpt=["no-new-privileges:true"],
                NetworkMode="none" if name == "simulator" else "container:simulator"),
            Mounts=[dict(Destination="/evidence", RW=True), dict(Destination="/v4", RW=False)]))
    return items


def test_selected_container_contract():
    verify_container_contract(containers(), "codex-tiny-dev-test")


@pytest.mark.parametrize("kind", ["host_net", "device", "privileged", "writable_source", "unknown_mount", "project"])
def test_selected_container_reject(kind):
    items = containers()
    c = items[1]
    if kind == "host_net": c["HostConfig"]["NetworkMode"] = "host"
    if kind == "device": c["HostConfig"]["Devices"] = [{"PathOnHost":"/dev/ttyUSB0"}]
    if kind == "privileged": c["HostConfig"]["Privileged"] = True
    if kind == "writable_source": c["Mounts"][1]["RW"] = True
    if kind == "unknown_mount": c["Mounts"].append(dict(Destination="/var/run/docker.sock", RW=True))
    if kind == "project": c["Config"]["Labels"]["com.docker.compose.project"] = "another"
    with pytest.raises(ValueError): verify_container_contract(items, "codex-tiny-dev-test")


def test_judge_requires_order_not_proximity_or_completion_word():
    judge = JudgeLog()
    judge.feed("Lap completed: 60.00s, total laps: 1")
    assert not judge.completed
    judge = JudgeLog()
    for a, b in [(-1, 0), (0, 1), (1, 2), (2, 0)]:
        judge.feed(f"Section line hit: current={a}, next={b}, started={a != -1}")
    judge.feed("Lap completed: 63.00s, total laps: 1")
    assert judge.completed
    broken = JudgeLog()
    for a, b in [(-1, 0), (0, 2), (2, 3), (3, 0)]:
        broken.feed(f"Section line hit: current={a}, next={b}, started={a != -1}")
    broken.feed("Lap completed: 63.00s, total laps: 1")
    assert not broken.completed


@pytest.mark.skipif(os.name != "posix", reason="Existing cumulative lock uses Linux fcntl")
def test_budget_preserves_v4_and_separates_tiny(tmp_path):
    path = tmp_path/"budget.json"
    used = dict(wall_s=1189.1912133327914, forward=124, tiny_forward=173,
                mpc=1, snapshots=7, powered=1, powered_s=8.889999801, log_bytes=58002335)
    path.write_text(json.dumps(dict(used=used, active=None)))
    budget = TinyBudget(path, profile=TINY_AUTH_PROFILE)
    change = budget.authorize()
    assert change["used_at_change"] == used
    assert change["old_limits"]["forward"] == 3000
    assert change["new_limits"]["forward"] == 6000
    assert budget.authorize() == change
    reservation = {k:0 for k in budget.limits}
    reservation.update(wall_s=120., powered=1, powered_s=60.)
    budget.reserve_tiny("synthetic", reservation, 12)
    budget.finish_tiny(dict(wall_s=2., forward=0, mpc=0, snapshots=0, powered=0, powered_s=0, log_bytes=100), 8, exact=True)
    assert budget.value["used"]["forward"] == 124
    assert budget.value["used"]["tiny_forward"] == 181
    with pytest.raises(ValueError, match="COMMON_FORWARD"):
        budget.value["used"]["tiny_forward"] = 1000  # Synthetic exhausted state.
        budget.reserve_tiny("too_many", reservation, 5200)
    budget.close()


def config(phase="lap"):
    import yaml
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/"configs/control/tiny_lidar_sim.yaml").read_text())
    cfg.update(tiny_phase_caps(phase), phase=phase)
    return cfg


def test_shared_runtime_host_authorized_caps_and_braking_reserve():
    lap = config()
    assert (lap["single_episode_sim_limit_s"], lap["forward_limit"], lap["wall_seconds"]) == (240., 5200, 600)
    validate_tiny_config(lap)
    validate_tiny_config(config("short"))
    assert tiny_phase_caps("short") == dict(wall_seconds=120, forward_limit=300, single_episode_sim_limit_s=20.)
    assert motion_limit_reason(lap, 233.99) is None
    assert motion_limit_reason(lap, 234.) == "EPISODE_LIMIT_NO_LAP"
    assert motion_limit_reason(config("short"), 8.) == "SHORT_MOTION_COMPLETE"
    assert bounded_runtime_wall("lap", 600, 2410.8, DRIVE_CUTOFF_UNIX_S-900) == 600
    assert bounded_runtime_wall("lap", 600, 2410.8, DRIVE_CUTOFF_UNIX_S-400) == 290
    assert bounded_runtime_wall("lap", 600, 310, DRIVE_CUTOFF_UNIX_S-900) == 200
    with pytest.raises(ValueError): bounded_runtime_wall("lap", 600, 2410., DRIVE_CUTOFF_UNIX_S)
    with pytest.raises(ValueError): bounded_runtime_wall("short", 600, 2410., DRIVE_CUTOFF_UNIX_S-900)


@pytest.mark.parametrize("key", ["wall_seconds", "forward_limit", "single_episode_sim_limit_s", "target_speed_mps"])
@pytest.mark.parametrize("value", [True, False, math.nan, math.inf, -1])
def test_invalid_config_scalars_are_rejected(key, value):
    cfg = config(); cfg[key] = value
    with pytest.raises(ValueError): validate_tiny_config(cfg)


@pytest.mark.parametrize("key,value", [("wall_seconds", 601), ("forward_limit", 5201),
    ("single_episode_sim_limit_s", 240.001), ("forward_limit", 5200.0), ("authorization_profile", "V4")])
def test_config_overrun_or_wrong_profile(key, value):
    cfg = config(); cfg[key] = value
    with pytest.raises(ValueError): validate_tiny_config(cfg)


@pytest.mark.skipif(os.name != "posix", reason="Linux budget lock")
def test_short_then_lap_budget_authorization_does_not_leak(tmp_path):
    base = dict(AttemptBudget.limits)
    used = dict(wall_s=1189.1912133327914, forward=124, tiny_forward=173, mpc=1, snapshots=7,
                powered=1, powered_s=8.889999801, log_bytes=58002335)
    path = tmp_path/"budget.json"
    path.write_text(json.dumps(dict(used=used, active=None, attempts=[dict(id="OLD_ATTEMPT")])) )
    budget = TinyBudget(path, profile=TINY_AUTH_PROFILE)
    budget.authorize()
    short = dict(wall_s=230., forward=0, mpc=0, snapshots=0, powered=1, powered_s=20., log_bytes=96*1024**2)
    budget.reserve_tiny("short", short, 300)
    with pytest.raises(ValueError): budget.reserve_tiny("concurrent", short, 300)
    budget.finish_tiny(dict(wall_s=35., forward=0, mpc=0, snapshots=0, powered=1, powered_s=10., log_bytes=1000), 180, exact=True)
    lap = dict(short, wall_s=710., powered_s=240.)
    budget.reserve_tiny("lap", lap, 5200)
    budget.finish_tiny({}, None, exact=False)  # Conservative missing totals.
    assert budget.value["used"]["forward"] == 124
    assert budget.value["used"]["tiny_forward"] == 173+180+5200
    assert budget.value["used"]["powered"] == 3
    assert budget.value["attempts"][0] == dict(id="OLD_ATTEMPT")
    assert budget.value["active"] is None
    assert AttemptBudget.limits == base and base["powered_s"] == 180 and base["forward"] == 3000
    budget.limits["forward"] = 9999
    assert AttemptBudget.limits == base
    budget.close()


@pytest.mark.skipif(os.name != "posix", reason="Linux budget lock")
@pytest.mark.parametrize("value", [True, math.nan, -1, math.inf])
def test_invalid_budget_never_creates_reservation(tmp_path, value):
    path = tmp_path/"budget.json"
    path.write_text(json.dumps(dict(used=dict(wall_s=0., forward=124, tiny_forward=173, mpc=1, snapshots=7,
        powered=1, powered_s=8., log_bytes=100), active=None)))
    budget = TinyBudget(path, profile=TINY_AUTH_PROFILE); budget.authorize()
    reservation = dict(wall_s=value, forward=0, mpc=0, snapshots=0, powered=1, powered_s=20., log_bytes=100)
    with pytest.raises(ValueError): budget.reserve_tiny("bad", reservation, 300)
    assert budget.value["active"] is None
    budget.close()


def test_judge_lap_increment_run_and_offset():
    judge = JudgeLog("OWNED_RUN")
    for index, (a, b) in enumerate([(0, 0), (0, 1), (1, 2), (2, 0)]):
        judge.feed(f"Section line hit: current={a}, next={b}, started={index != 0}", index*100)
    judge.feed("Lap completed: 200.00s, total laps: 2", 400)
    assert not judge.completed  # Missing first-lap increment.
    assert judge.laps[0]["byte_offset"] == 400 and judge.laps[0]["run_id"] == "OWNED_RUN"


@pytest.mark.skipif(not os.environ.get("TINY_OFFICIAL_PACKAGE"), reason="Optional fixed official package not supplied")
def test_official_keys_shapes_and_unchanged_preprocessing_no_forward():
    model = OfficialTiny(Path(os.environ["TINY_OFFICIAL_PACKAGE"]))
    assert model.identity["loaded_parameter_tensors"] == 18
    values = np.full(750, 15., np.float32)
    values[:6] = [np.nan, np.inf, -np.inf, -1., 40., 0.]
    result = model.core._preprocess_ranges(values)
    np.testing.assert_array_equal(result[:6], [0., 1., 1., 0., 1., 0.])
    assert result.shape == (750,) and result[6] == .5
    # No model.forward/process call: this test consumes no inference budget.
