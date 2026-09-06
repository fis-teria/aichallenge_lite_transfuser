"""Synthetic GUI profile/ROS packaging checks. Never start ROS or infer."""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"tools"))
from aic_transfuser_lite.runtime.tiny_lidar_sim import (
    TINY_GUI_PROFILE, GUI_AUTH_SHA256, GUI_CONTROL_METHOD, TINY_AUTH_PROFILE,
    host_arm_status, tiny_phase_caps, validate_tiny_config, bounded_runtime_wall, digest,
    verify_container_contract,
    TINY_GUI_RETRY_PROFILE, GUI_RETRY_AUTH_SHA256,
)
from tiny_dev_runner import TinyBudget, IMAGE
from tiny_gui_integration import compose_gui, narrow_fingerprint, make_command, window_candidates
from spatial_dev_host_v4 import AttemptBudget


def config():
    result = yaml.safe_load((ROOT/"configs/control/tiny_lidar_sim.yaml").read_text())
    result.update(tiny_phase_caps("short", TINY_GUI_PROFILE), phase="short", authorization_profile=TINY_GUI_PROFILE,
                  authorization_request_sha256=GUI_AUTH_SHA256, control_method=GUI_CONTROL_METHOD, source_commit="a"*40)
    return result


def test_authorization_bytes_and_gui_only_caps():
    assert digest(ROOT/"configs/control/tiny_gui_authorization_20260907.json") == GUI_AUTH_SHA256
    validate_tiny_config(config())
    assert tiny_phase_caps("short")["forward_limit"] == 300
    assert tiny_phase_caps("short", TINY_GUI_PROFILE)["forward_limit"] == 600
    with pytest.raises(ValueError):
        tiny_phase_caps("lap", TINY_GUI_PROFILE)
    assert bounded_runtime_wall("short", 120, 230, 1788728000, TINY_GUI_PROFILE) == 120


@pytest.mark.parametrize("field,value", [("forward_limit", 601), ("wall_seconds", 121),
    ("single_episode_sim_limit_s", 21), ("control_method", "tiny_lidar_net"), ("forward_limit", True),
    ("forward_limit", float("nan")), ("forward_limit", -1), ("wall_seconds", float("inf"))])
def test_gui_entry_rejects(field, value):
    cfg = config(); cfg[field] = value
    with pytest.raises(ValueError): validate_tiny_config(cfg)


def test_post_read_arm_sampling_counterexample():
    arm = dict(token="ours", armed=True, paused_kill_verified=True, monotonic_ns=101)
    # Reading an ARM updated after a caller's earlier time must not reuse time=100.
    assert host_arm_status(arm, project="ours", observed_ns=100)["reason"] == "FUTURE_STAMP"
    assert host_arm_status(arm, project="ours", observed_ns=102)["allowed"]
    code = (ROOT/"tools/run_tiny_lidar_dev.py").read_text()
    block = code.split("    def host_armed()", 1)[1].split("    def receive", 1)[0]
    assert block.index("p.read_text()") < block.index("observed_ns=time.monotonic_ns()")
    assert 'log("HOST_ARM_REJECTED"' in block


@pytest.mark.parametrize("change,reason", [({"token":"other"}, "IDENTITY"), ({"armed":False}, "NOT_ARMED"),
    ({"paused_kill_verified":False}, "STOP_PROOF_MISSING"), ({"monotonic_ns":None}, "STAMP_INVALID"),
    ({"monotonic_ns":0}, "EXPIRED")])
def test_arm_precise_failure(change, reason):
    arm = dict(token="ours", armed=True, paused_kill_verified=True, monotonic_ns=800000000)
    arm.update(change)
    assert host_arm_status(arm, project="ours", observed_ns=800000001)["reason"] == reason


def ledger(tmp_path):
    path = tmp_path/"budget.json"
    limits = dict(AttemptBudget.limits, forward=6000, powered_s=300.)
    used = dict(wall_s=1253.7936995498367, forward=124, tiny_forward=375, mpc=1, snapshots=7,
                powered=3, powered_s=21.259999524, log_bytes=61961373)
    path.write_text(json.dumps(dict(used=used, active=None, attempts=[{"id":"prior"}],
        tiny_authorized_profile=TINY_AUTH_PROFILE, tiny_authorized_limits=limits)))
    return path, used


def reservation():
    return dict(wall_s=230., forward=0, mpc=0, snapshots=2, powered=1, powered_s=20., log_bytes=96*1024**2)


def test_gui_budget_continuity_single_attempt_no_shared_mutation(tmp_path):
    path, used = ledger(tmp_path)
    original = dict(AttemptBudget.limits)
    budget = TinyBudget(path, profile=TINY_GUI_PROFILE)
    record = budget.authorize()
    assert record["old_limits"]["powered"] == 3 and record["new_limits"]["powered"] == 4
    assert record["used_at_change"] == used
    assert budget.authorize() == record and len(budget.value["authorization_changes"]) == 1
    budget.reserve_tiny("gui", reservation(), 600)
    with pytest.raises(ValueError): budget.reserve_tiny("parallel", reservation(), 600)
    budget.finish_tiny(dict(wall_s=30., forward=0, mpc=0, snapshots=2, powered=1, powered_s=10., log_bytes=1000), 180, exact=True)
    assert budget.value["used"]["powered"] == 4 and budget.value["used"]["tiny_forward"] == 555
    assert budget.value["attempts"][0] == {"id":"prior"}
    with pytest.raises(ValueError, match="SINGLE_ATTEMPT"):
        budget.reserve_tiny("again", reservation(), 600)
    assert AttemptBudget.limits == original
    budget.close()
    old = TinyBudget(path, profile=TINY_AUTH_PROFILE)
    assert old.limits["powered"] == 3
    old.close()


def test_gui_missing_consumption_charged_not_reset(tmp_path):
    path, used = ledger(tmp_path)
    budget = TinyBudget(path, profile=TINY_GUI_PROFILE)
    budget.authorize(); budget.reserve_tiny("missing", reservation(), 600)
    budget.finish_tiny(dict(forward=0, mpc=0), None, exact=False)
    assert budget.value["used"]["powered"] == 4
    assert budget.value["used"]["tiny_forward"] == used["tiny_forward"]+600
    assert budget.value["used"]["wall_s"] == used["wall_s"]+230
    budget.close()


def test_explicit_retry_preserves_failed_charge_and_adds_exactly_one_attempt(tmp_path):
    path, _ = ledger(tmp_path)
    original = dict(AttemptBudget.limits)
    first = TinyBudget(path, profile=TINY_GUI_PROFILE)
    first.authorize(); first.reserve_tiny("failed_gui", reservation(), 600)
    first.finish_tiny(dict(forward=0, mpc=0), None, exact=False)
    prior = copy.deepcopy(first.value)
    first.close()
    retry = TinyBudget(path, profile=TINY_GUI_RETRY_PROFILE)
    record = retry.authorize()
    assert record["old_limits"]["powered"] == 4 and record["new_limits"]["powered"] == 5
    assert record["old_episode_sim_seconds"] == record["new_episode_sim_seconds"] == 20
    assert record["request_sha256"] == GUI_RETRY_AUTH_SHA256
    assert retry.value["used"] == prior["used"] and retry.value["attempts"] == prior["attempts"]
    assert retry.authorize() == record
    retry.reserve_tiny("approved_retry", reservation(), 600)
    retry.finish_tiny(dict(forward=0, mpc=0), None, exact=False)
    assert retry.value["used"]["powered"] == 5
    assert retry.value["used"]["tiny_forward"] == prior["used"]["tiny_forward"]+600
    with pytest.raises(ValueError, match="SINGLE_ATTEMPT"):
        retry.reserve_tiny("unapproved_third", reservation(), 600)
    retry.close()
    assert AttemptBudget.limits == original


def test_retry_requires_previous_grant_and_exact_request(tmp_path):
    path, _ = ledger(tmp_path)
    retry = TinyBudget(path, profile=TINY_GUI_RETRY_PROFILE)
    with pytest.raises(ValueError, match="PREVIOUS_GUI_AUTHORIZATION"):
        retry.authorize()
    retry.close()
    assert digest(ROOT/"configs/control/tiny_gui_retry_authorization_20260907.json") == GUI_RETRY_AUTH_SHA256
    cfg = config(); cfg["authorization_profile"] = TINY_GUI_RETRY_PROFILE
    with pytest.raises(ValueError): validate_tiny_config(cfg)
    cfg["authorization_request_sha256"] = GUI_RETRY_AUTH_SHA256
    validate_tiny_config(cfg)
    assert tiny_phase_caps("short", TINY_GUI_RETRY_PROFILE) == tiny_phase_caps("short", TINY_GUI_PROFILE)
    with pytest.raises(ValueError): tiny_phase_caps("lap", TINY_GUI_RETRY_PROFILE)


@pytest.mark.parametrize("field,value", [("wall_s",231.), ("powered_s",21.), ("snapshots",3), ("mpc",1)])
def test_gui_reservation_caps(tmp_path, field, value):
    path, _ = ledger(tmp_path)
    budget = TinyBudget(path, profile=TINY_GUI_PROFILE); budget.authorize()
    value_map = reservation(); value_map[field] = value
    with pytest.raises(ValueError): budget.reserve_tiny("bad", value_map, 600)
    budget.close()


def test_package_only_explicit_runtime_no_v4_weights():
    package = ROOT/"ros2_ws/src/aic_tiny_sim_test"
    assert ET.parse(package/"package.xml").getroot().findtext("name") == "aic_tiny_sim_test"
    setup = (package/"setup.py").read_text()
    assert "rglob" not in setup and "*.pt" not in setup and "ckpt/" not in setup
    launch = (package/"launch/guarded_tiny.launch.py").read_text()
    assert GUI_CONTROL_METHOD in launch and "tiny_sim_supervisor" in launch and "rviz2" in launch
    rviz = yaml.safe_load((package/"config/tiny_scan.rviz").read_text())
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "lidar"
    assert "SetGoal" not in str(rviz) and "PublishPoint" not in str(rviz)


def test_gui_compose_isolated_real_display_and_method(tmp_path):
    install = tmp_path/"install"; install.mkdir(); (install/"setup.bash").touch()
    auth = tmp_path/"Xauthority"; auth.touch()
    spec = compose_gui(ROOT, tmp_path/"sim", install, tmp_path/"out", tmp_path/"official",
                       "codex-tiny-dev-test", "a"*40, ":1", auth, IMAGE)
    assert set(spec["services"]) == {"simulator", "autoware"}
    for service in spec["services"].values():
        assert not service["privileged"] and service["cap_drop"] == ["ALL"]
        assert service["environment"]["DISPLAY"] == ":1"
        assert service["environment"]["CONTROL_METHOD"] == GUI_CONTROL_METHOD
        assert all(not v["read_only"] == (v["target"] == "/evidence") for v in service["volumes"])
        assert not any("/dev/" in v["target"] or v["target"] == "/aichallenge" for v in service["volumes"])
    assert spec["services"]["simulator"]["network_mode"] == "none"
    assert spec["services"]["autoware"]["network_mode"] == "service:simulator"
    assert "hostname" not in spec["services"]["autoware"]
    assert spec["services"]["autoware"]["environment"]["XAUTHLOCALHOSTNAME"] == os.uname().nodename


def test_make_uses_existing_recipe_and_new_method_only(tmp_path):
    fake = tmp_path/"racing"; fake.mkdir()
    (fake/"Makefile").write_text("capture-run-fingerprint:\n\t@echo ORIGINAL_FINGERPRINT\ndev: capture-run-fingerprint\n\t@echo ORIGINAL_DEV\n")
    args = ["make", "--no-print-directory", "-f", str(ROOT/"integrations/tiny_gui/Makefile"), "dev", "RACINGKART_REPO="+str(fake)]
    old = subprocess.run(args+["CONTROL_METHOD=pure_pursuit"], capture_output=True, text=True, check=True)
    assert "ORIGINAL_FINGERPRINT" in old.stdout and "ORIGINAL_DEV" in old.stdout
    # make -n on this entirely synthetic Makefile does not execute any recipe.
    new = subprocess.run(args+["-n", "CONTROL_METHOD="+GUI_CONTROL_METHOD], capture_output=True, text=True, check=True)
    assert "tiny_gui_integration.py" in new.stdout and "ORIGINAL_FINGERPRINT" not in new.stdout
    assert "ORIGINAL_DEV" in new.stdout
    command = make_command(ROOT, fake, tmp_path/"out", "test")
    assert "CONTROL_METHOD="+GUI_CONTROL_METHOD in command and "DEV_AUTO_START=false" in command


def test_narrow_fingerprint_does_not_walk_other_weights(tmp_path):
    out = tmp_path/"out"; out.mkdir()
    fake = tmp_path/"racing"; (fake/"aichallenge").mkdir(parents=True)
    (fake/"Makefile").write_text("synthetic")
    (fake/"aichallenge/run_simulator.bash").write_text("synthetic")
    (out/"compose.json").write_text("{}")
    (out/"resolved_config.json").write_text(json.dumps(config()))
    value = narrow_fingerprint(ROOT, fake, out)
    assert value["source_build_install_recursive_reads"] is False
    assert not any("pilot" in record["path"] for record in value["files"])
    assert len(value["files"]) == 18


def test_guarded_recursive_make_keeps_include_and_command_variables(tmp_path):
    # Match the real recipe's recursive command-mode call, but no Docker/ROS.
    fake = tmp_path/"racing"; fake.mkdir()
    source = tmp_path/"source"; source.mkdir()
    (source/"Makefile").write_text("dev:\n\t@echo WRONG_SOURCE_MAKEFILE\n")
    (fake/"Makefile").write_text(
        "dev: autoware-simulator\n"
        "autoware-simulator:\n"
        "\t@$(MAKE) autoware-command-mode-run AUTOWARE_SERVICE=autoware\n"
        "autoware-command-mode-run:\n"
        "\t@$(MAKE) synthetic-grandchild\n"
        "synthetic-grandchild:\n"
        "\t@echo GUARDED_CHILD $(CONTROL_METHOD) $(AUTOWARE_SERVICE)\n")
    result = subprocess.run(["make", "--no-print-directory", "-f", str(ROOT/"integrations/tiny_gui/Makefile"),
        "dev", "RACINGKART_REPO="+str(fake), "CONTROL_METHOD="+GUI_CONTROL_METHOD],
        cwd=source, capture_output=True, text=True, check=True)
    assert "GUARDED_CHILD tiny_lidar_net_guarded autoware" in result.stdout
    assert "WRONG_SOURCE_MAKEFILE" not in result.stdout


def test_windows_require_both_distinct_titles():
    tree = ' 0x100 "AWSIM": ("AWSIM" "AWSIM")\n 0x200 "tiny_scan.rviz - RViz": ()\n 0x300 "Terminal": ()'
    assert window_candidates(tree) == {"awsim":["0x100"], "rviz":["0x200"]}
    assert window_candidates("empty") == {"awsim":[], "rviz":[]}
