"""Honest horizon metrics for time trajectory evaluation."""
from __future__ import annotations

from collections import defaultdict
import math
from typing import Sequence

import torch

DT_SEC = 0.1
HORIZONS_SEC = (0.5, 1.0, 2.0, 3.0)


def _bools(value: torch.Tensor | None, shape: tuple[int, ...], name: str, default: bool) -> torch.Tensor:
    if value is None:
        return torch.full(shape, default, dtype=torch.bool)
    if value.shape != shape or value.dtype is not torch.bool:
        raise ValueError(f"{name} must be bool with shape {shape}")
    return value


def time_horizon_metrics(
    predictions: torch.Tensor, targets: torch.Tensor, support_mask: torch.Tensor,
    *, input_valid: torch.Tensor | None = None,
    accepted: torch.Tensor | None = None,
    stale: torch.Tensor | None = None,
    run_ids: Sequence[str] | None = None,
    horizons_sec: Sequence[float] = HORIZONS_SEC,
) -> dict[str, object]:
    """Return raw and accepted-conditional errors with explicit denominators.

    ``input_valid``, ``accepted`` and ``stale`` are per-anchor flags. All
    anchors remain in ``total_count``; unsupported horizons report ``None``.
    """
    if predictions.shape != targets.shape or predictions.ndim != 3 or predictions.shape[1:] != (30,2):
        raise ValueError("time metrics require prediction/target [N,30,2]")
    predictions, targets, support_mask = predictions.detach().cpu(), targets.detach().cpu(), support_mask.detach().cpu()
    if support_mask.shape != predictions.shape[:2] or support_mask.dtype is not torch.bool:
        raise ValueError("support mask must be bool [N,T]")
    n = predictions.shape[0]
    iv = _bools(input_valid, (n,), "input_valid", True).cpu()
    ac = _bools(accepted, (n,), "accepted", True).cpu()
    st = _bools(stale, (n,), "stale", False).cpu()
    if run_ids is not None and len(run_ids) != n:
        raise ValueError("run_ids length mismatch")
    distance = torch.linalg.vector_norm(predictions - targets.nan_to_num(0.0), dim=-1)
    pred_point_valid = torch.isfinite(predictions).all(dim=2)
    target_point_valid = torch.isfinite(targets).all(dim=2)
    pred_valid = pred_point_valid.all(dim=1)
    target_valid = (target_point_valid | ~support_mask).all(dim=1)
    out: dict[str, object] = {"anchor_count": n, "input_invalid_count": int((~iv).sum()),
                              "rejected_count": int((iv & (~ac | ~pred_valid)).sum()), "stale_count": int(st.sum()),
                              "prediction_invalid_count": int((iv & ~pred_valid).sum()),
                              "target_invalid_count": int((~target_valid).sum()),
                              "accepted_plan_count": int((iv & ac & pred_valid & ~st).sum()),
                              "acceptance_scope": "caller_selection_flags_not_runtime_safety_certification",
                              "horizons": {}}
    horizon_out: dict[str, object] = out["horizons"]  # type: ignore[assignment]
    for horizon in horizons_sec:
        if (type(horizon) not in (float,int) or not math.isfinite(horizon) or
                horizon <= 0 or horizon > predictions.shape[1] * DT_SEC or
                abs(horizon / DT_SEC-round(horizon / DT_SEC)) > 1e-6):
            raise ValueError("horizon outside trajectory grid")
        index = int(round(horizon / DT_SEC)) - 1
        support = support_mask[:, index] & pred_point_valid[:, index] & target_point_valid[:, index]
        raw_n = int(support.sum())
        cond = support & iv & ac & pred_valid & ~st
        cond_n = int(cond.sum())
        horizon_out[f"{horizon:g}s"] = {
            "raw_error_m": float(distance[support, index].mean()) if raw_n else None,
            "raw_count": raw_n,
            "accepted_error_m": float(distance[cond, index].mean()) if cond_n else None,
            "accepted_count": cond_n,
            "total_count": n,
            "teacher_support_count": int((support_mask[:,index] & target_point_valid[:,index]).sum()),
            "coverage": raw_n / n if n else None,
            "accepted_coverage": cond_n / n if n else None,
            "status": "NOT_EVALUATED" if raw_n == 0 else "OK",
        }
    all_support = support_mask & pred_point_valid & target_point_valid
    all_count = int(all_support.sum())
    out["all_point_ade_m"] = float(distance[all_support].mean()) if all_count else None
    out["all_point_count"] = all_count
    out["all_teacher_support_count"] = int((support_mask & target_point_valid).sum())
    out["error_definition"] = "Euclidean position error in metres; horizons are exact endpoints; all_point_ADE is point-weighted"
    out["all_point_coverage"] = all_count / int(support_mask.numel()) if support_mask.numel() else None
    if run_ids is not None:
        groups: dict[str, list[int]] = defaultdict(list)
        for i, run in enumerate(run_ids): groups[str(run)].append(i)
        per_run = {run: time_horizon_metrics(predictions[idx], targets[idx], support_mask[idx],
            input_valid=iv[idx], accepted=ac[idx], stale=st[idx], horizons_sec=horizons_sec)
            for run, idx in groups.items()}
        out["run_macro"] = per_run
        macro: dict[str, object] = {}
        for horizon in horizons_sec:
            key = f"{horizon:g}s"
            rows = [r["horizons"][key] for r in per_run.values()]
            raw = [r["raw_error_m"] for r in rows if r["raw_error_m"] is not None]
            accepted_errors = [r["accepted_error_m"] for r in rows if r["accepted_error_m"] is not None]
            macro[key] = {"raw_error_m": sum(raw) / len(raw) if raw else None,
                          "accepted_error_m": sum(accepted_errors) / len(accepted_errors) if accepted_errors else None,
                          "run_count": len(rows), "raw_supported_runs": len(raw),
                          "accepted_supported_runs": len(accepted_errors)}
        out["run_macro_mean"] = macro
        out["worst_run"] = {
            key: max(per_run, key=lambda run: per_run[run]["horizons"][key]["raw_error_m"]
                    if per_run[run]["horizons"][key]["raw_error_m"] is not None else float("-inf"))
            if any(r["horizons"][key]["raw_error_m"] is not None for r in per_run.values()) else None
            for key in (f"{h:g}s" for h in horizons_sec)
        }
    return out


def zero_velocity_baseline(batch_size: int, *, steps: int = 30,
                           device: torch.device | None = None,
                           dtype: torch.dtype = torch.float32) -> torch.Tensor:
    """Causal zero-motion baseline without reading teacher future points."""
    if type(batch_size) is not int or batch_size < 0 or type(steps) is not int or steps <= 0:
        raise ValueError("batch_size and steps must be valid integers")
    return torch.zeros((batch_size, steps, 2), device=device, dtype=dtype)


def constant_velocity_baseline(observed_velocity_xy_mps: torch.Tensor, *, steps: int = 30,
                               dt_sec: float = DT_SEC) -> torch.Tensor:
    """Causal constant-velocity rollout from current observed ego velocity."""
    if observed_velocity_xy_mps.ndim != 2 or observed_velocity_xy_mps.shape[1:] != (2,):
        raise ValueError("observed velocity must be [N,2] m/s")
    if (not observed_velocity_xy_mps.is_floating_point() or not torch.isfinite(observed_velocity_xy_mps).all()
            or type(steps) is not int or steps <= 0 or type(dt_sec) is not float
            or not math.isfinite(dt_sec) or dt_sec <= 0):
        raise ValueError("velocity, steps and dt_sec must be finite and positive")
    times = torch.arange(1, steps + 1, device=observed_velocity_xy_mps.device,
                         dtype=observed_velocity_xy_mps.dtype)[None, :, None] * dt_sec
    return observed_velocity_xy_mps[:, None, :] * times
