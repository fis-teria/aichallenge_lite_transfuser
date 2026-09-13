"""Create a compact, read-only report from a completed paired TimePath run."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def _read(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _content_sha256(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"),
                     allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _metric_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    horizons = {}
    for key, row in metrics.get("horizons", {}).items():
        horizons[key] = {name: row.get(name) for name in (
            "raw_error_m", "raw_count", "teacher_support_count", "coverage",
            "accepted_error_m", "accepted_count", "accepted_coverage", "total_count")}
    macro = {key: {name: row.get(name) for name in (
        "raw_error_m", "accepted_error_m", "run_count", "raw_supported_runs",
        "accepted_supported_runs")} for key, row in metrics.get("run_macro_mean", {}).items()}
    baselines = {}
    for name, value in metrics.get("baselines", {}).items():
        baselines[name] = {"horizons": {key: {field: row.get(field) for field in (
            "raw_error_m", "raw_count", "coverage", "accepted_error_m", "accepted_count")}
                           for key, row in value.get("horizons", {}).items()},
                           "run_macro_mean": value["run_macro_mean"],
                           "all_point_ade_m": value["all_point_ade_m"]}
    return {"anchor_count": metrics.get("anchor_count"),
            "input_invalid_count": metrics.get("input_invalid_count"),
            "prediction_invalid_count": metrics.get("prediction_invalid_count"),
            "target_invalid_count": metrics.get("target_invalid_count"),
            "all_point_ade_m": metrics.get("all_point_ade_m"),
            "all_point_count": metrics.get("all_point_count"),
            "all_teacher_support_count": metrics.get("all_teacher_support_count"),
            "all_point_coverage": metrics.get("all_point_coverage"),
            "horizons": horizons, "run_macro_mean": macro,
            "run_macro": metrics.get("run_macro"), "worst_run": metrics.get("worst_run"),
            "baselines": baselines, "invalid_reasons": metrics.get("invalid_reasons", {}),
            "teacher_reasons": metrics.get("teacher_reasons", {}),
            "stop_reasons": metrics.get("stop_reasons", {})}


def _plan_for_compare(plan: dict[str, Any]) -> dict[str, Any]:
    # plan.json embeds arm-specific config/identity; compare the training contract.
    return {key: value for key, value in plan.items() if key not in {"config", "identity"}}


def _load_epoch_metrics(arm: Path) -> list[dict[str, Any]]:
    rows = []
    for path in sorted(arm.glob("validation_epoch_*.json")):
        metrics = _read(path)
        rows.append({"epoch": int(path.stem.rsplit("_", 1)[1]), "metrics": metrics})
    return rows


def build_report(run: Path, output: Path) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    comparison_path = run / "comparison.json"
    teacher_path = run / "teacher_manifest.json"
    comparison = _read(comparison_path)
    teacher = _read(teacher_path)
    declared_teacher_sha = teacher.get("manifest_sha256")
    actual_teacher_sha = _content_sha256({k: v for k, v in teacher.items()
                                          if k != "manifest_sha256"})
    if declared_teacher_sha != actual_teacher_sha:
        raise ValueError("teacher manifest canonical digest mismatch")
    if comparison.get("status") != "COMPLETE" or comparison.get("test_evaluated") is not False:
        raise ValueError("comparison is incomplete or test evaluation is not sealed")
    arms = {row["arm"]: row for row in comparison.get("arms", [])}
    if set(arms) != {"command_off", "command_on"}:
        raise ValueError("comparison must contain command_off and command_on")
    loaded: dict[str, dict[str, Any]] = {}
    for name, row in arms.items():
        arm = run / name
        result = _read(arm / "result.json")
        if arms[name] != {"arm": name, **result} or result.get("reload_predictions_exact") is not True:
            raise ValueError(f"{name} comparison/reload evidence mismatch")
        plan = _read(arm / "plan.json")
        history = _read(arm / "history.json")
        metrics = _read(arm / "best_validation.json")
        if result.get("status") != "COMPLETE" or result.get("test_evaluated") is not False:
            raise ValueError(f"{name} is incomplete or evaluated test data")
        epochs = plan.get("epochs")
        if (type(epochs) is not int or epochs <= 0 or result.get("epochs_completed") != epochs
                or len(history) != epochs
                or [row.get("epoch") for row in history] != list(range(1, epochs + 1))):
            raise ValueError(f"{name} epoch completion gate failed")
        best_epoch = result.get("best_epoch")
        if type(best_epoch) is not int or not 1 <= best_epoch <= epochs:
            raise ValueError(f"{name} best epoch is outside completed range")
        selected_json = arm / f"validation_epoch_{best_epoch:02d}.json"
        selected_npy = arm / f"validation_epoch_{best_epoch:02d}.npy"
        if not selected_json.is_file() or not selected_npy.is_file():
            raise ValueError(f"{name} selected validation artifacts are missing")
        max_steps = plan.get("max_optimizer_steps")
        if (type(max_steps) is not int or max_steps < 0
                or type(result.get("optimizer_steps")) is not int
                or result["optimizer_steps"] > max_steps):
            raise ValueError(f"{name} optimizer completion gate failed")
        train_count = teacher.get("train_anchor_count")
        validation_count = teacher.get("validation_anchor_count")
        if type(train_count) is not int or train_count < 0:
            raise ValueError(f"{name} teacher train anchor count is invalid")
        expected_visits = train_count * epochs
        if (type(train_count) is not int
                or result.get("anchors_visited") != expected_visits
                or plan.get("max_anchors") != expected_visits):
            raise ValueError(f"{name} anchor visit completion gate failed")
        for row in history:
            if row.get("train_counts", {}).get("visited") != train_count:
                raise ValueError(f"{name} epoch visited count mismatch")
        identity = plan.get("identity")
        if (not isinstance(identity, dict)
                or identity.get("teacher_manifest_sha256") != declared_teacher_sha):
            raise ValueError(f"{name} checkpoint teacher identity mismatch")
        if metrics.get("anchor_count") != validation_count:
            raise ValueError(f"{name} validation anchor count mismatch")
        selected_history = history[best_epoch - 1]
        selected_metrics = _read(selected_json)
        if (metrics != selected_metrics or result["best_epoch"] != min(history,
                key=lambda r: r["validation_run_macro_3s_m"])["epoch"]
                or metrics["all_point_ade_m"] != result["best_validation_ade_m"]):
            raise ValueError(f"{name} selected/reloaded best metrics mismatch")
        if (selected_metrics.get("run_macro_mean", {}).get("3s", {}).get("raw_error_m")
                != selected_history.get("validation_run_macro_3s_m")
                or selected_metrics.get("all_point_ade_m") != selected_history.get("validation_ade_m")
                or result.get("best_validation_run_macro_3s_m")
                != selected_history.get("validation_run_macro_3s_m")):
            raise ValueError(f"{name} best validation selection mismatch")
        loaded[name] = {"path": arm, "result": result, "plan": plan,
                        "history": history, "metrics": metrics,
                        "epochs": _load_epoch_metrics(arm)}
    off, on = loaded["command_off"], loaded["command_on"]
    if ({k: v for k, v in off["result"]["config"].items() if k != "use_command_history"}
            != {k: v for k, v in on["result"]["config"].items() if k != "use_command_history"}
            or off["result"]["config"]["use_command_history"] is not False
            or on["result"]["config"]["use_command_history"] is not True):
        raise ValueError("model config differs beyond command history")
    orders = [r["epoch_order_sha256"] for r in off["history"]]
    if (orders != [r["epoch_order_sha256"] for r in on["history"]]
            or any(type(v) is not str or len(v) != 64 for v in orders)):
        raise ValueError("epoch presentation orders differ")
    if off["result"].get("initial_weights_sha256") != on["result"].get("initial_weights_sha256"):
        raise ValueError("initial weights differ")
    if off["result"].get("anchors_visited") != on["result"].get("anchors_visited"):
        raise ValueError("presentation budgets differ")
    if _plan_for_compare(off["plan"]) != _plan_for_compare(on["plan"]):
        raise ValueError("training plans differ")
    for field in ("optimizer_steps", "anchors_visited"):
        if off["result"].get(field) != on["result"].get(field):
            raise ValueError(f"{field} differs")
    for key in ("command_off", "command_on"):
        history = loaded[key]["history"]
        supported = [row.get("train_counts", {}).get("supported") for row in history]
        loaded[key]["supported_train_counts"] = supported
    if off["supported_train_counts"] != on["supported_train_counts"]:
        raise ValueError("supported train counts differ")
    for horizon in ("0.5s", "1s", "2s", "3s"):
        a = off["metrics"].get("run_macro_mean", {}).get(horizon, {})
        b = on["metrics"].get("run_macro_mean", {}).get(horizon, {})
        if a.get("raw_supported_runs") != b.get("raw_supported_runs"):
            raise ValueError(f"supported run count differs at {horizon}")
    if off["plan"].get("identity") != on["plan"].get("identity"):
        raise ValueError("checkpoint identities differ")
    output.mkdir(parents=True)
    arms_summary = {}
    for name, data in loaded.items():
        arm = data["path"]
        arms_summary[name] = {
            "result_path": str((arm / "result.json").resolve()),
            "plan_path": str((arm / "plan.json").resolve()),
            "history_path": str((arm / "history.json").resolve()),
            "metrics_path": str((arm / "best_validation.json").resolve()),
            "best_checkpoint_sha256": _sha256(arm / "best.pt"),
            "last_checkpoint_sha256": _sha256(arm / "last.pt"),
            "best_epoch": data["result"].get("best_epoch"),
            "optimizer_steps": data["result"].get("optimizer_steps"),
            "anchors_visited": data["result"].get("anchors_visited"),
            "supported_train_counts": data["supported_train_counts"],
            "metrics": _metric_summary(data["metrics"]),
        }
    report = {"status": "COMPLETE", "teacher_manifest_path": str(teacher_path.resolve()),
              "teacher_manifest_sha256": teacher.get("manifest_sha256"),
              "comparison_path": str(comparison_path.resolve()),
              "initial_weights_sha256": off["result"].get("initial_weights_sha256"),
              "arms": arms_summary, "scope": comparison.get("scope"),
              "test_evaluated": False, "runtime_ready": False,
              "checks": {"initial_weights_equal": True, "plans_equal": True,
                          "epoch_orders_equal": True, "config_only_command_differs": True,
                          "supported_train_counts_equal": True,
                          "supported_validation_runs_equal": True}}
    (output / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    _write_plot(output / "validation_curves.png", loaded)
    return report


def _write_plot(path: Path, loaded: dict[str, dict[str, Any]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required to generate validation_curves.png") from exc
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    styles = {"command_off": ("OFF", "tab:blue"), "command_on": ("ON", "tab:orange")}
    for name, data in loaded.items():
        label, color = styles[name]
        xs, score, ade = [], [], []
        for row in data["epochs"]:
            m = row["metrics"]
            xs.append(row["epoch"])
            score.append(m.get("run_macro_mean", {}).get("3s", {}).get("raw_error_m"))
            ade.append(m.get("all_point_ade_m"))
        axes[0].plot(xs, score, marker="o", label=label, color=color)
        axes[1].plot(xs, ade, marker="o", label=label, color=color)
    baseline = loaded["command_off"]["metrics"]["baselines"]["constant_observed_velocity"]
    axes[0].axhline(baseline["run_macro_mean"]["3s"]["raw_error_m"], color="gray",
                   linestyle="--", label="constant velocity baseline")
    axes[1].axhline(baseline["all_point_ade_m"], color="gray", linestyle="--",
                   label="constant velocity baseline")
    axes[0].set(xlabel="Epoch", ylabel="Validation 3s run-macro raw error (m)")
    axes[1].set(xlabel="Epoch", ylabel="Validation all-point ADE (m)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.suptitle("Proposed time model: command OFF/ON; 4 held-out runs, fixed conditions")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build_report(args.run, args.output)


if __name__ == "__main__":
    main()
