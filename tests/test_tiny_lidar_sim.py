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
    speed_acceleration, verify_container_contract, SOURCE_SHA256, WEIGHT_SHA256, input_clock_ready,
)
from tiny_dev_runner import JudgeLog, TinyBudget, ASSETS, ORIGINAL_SCRIPT
from spatial_dev_host_v4 import HostWatch, cleanup_owned


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
        judge.feed(f"Section line hit: current={a}, next={b}, started=True")
    judge.feed("Lap completed: 63.00s, total laps: 1")
    assert judge.completed
    broken = JudgeLog()
    for a, b in [(-1, 0), (0, 2), (2, 3), (3, 0)]:
        broken.feed(f"Section line hit: current={a}, next={b}, started=True")
    broken.feed("Lap completed: 63.00s, total laps: 1")
    assert not broken.completed


@pytest.mark.skipif(os.name != "posix", reason="Existing cumulative lock uses Linux fcntl")
def test_budget_preserves_v4_and_separates_tiny(tmp_path):
    path = tmp_path/"budget.json"
    used = dict(wall_s=1119.2905288418751, forward=124, mpc=1, snapshots=7, powered=0, powered_s=0, log_bytes=53527759)
    path.write_text(json.dumps(dict(used=used, active=None)))
    budget = TinyBudget(path)
    reservation = {k:0 for k in budget.limits}
    reservation.update(wall_s=120., powered=1, powered_s=60.)
    budget.reserve_tiny("synthetic", reservation, 12)
    budget.finish_tiny(dict(wall_s=2., forward=0, mpc=0, snapshots=0, powered=0, powered_s=0, log_bytes=100), 8, exact=True)
    assert budget.value["used"]["forward"] == 124
    assert budget.value["used"]["tiny_forward"] == 8
    with pytest.raises(ValueError, match="COMMON_FORWARD"):
        budget.reserve_tiny("too_many", reservation, 3000)
    budget.close()


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
