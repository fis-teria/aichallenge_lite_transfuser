"""Plot a provisional ten-site plan using verified native WSL observations."""
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path("/home/thistle/e2e_autonomous/runs/time_ten_site_recovery_plan_20260915")
audit_path = root / "candidate_window_audit.json"
audit = json.loads(audit_path.read_text())
raw = Path("/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30")
control = raw / "control.jsonl"
assert hashlib.sha256(control.read_bytes()).hexdigest() == audit["source_hashes"][raw.name]["control.jsonl"]
rows = [json.loads(line) for line in control.read_text().splitlines()]
rows = [r for r in rows if r.get("reason") == "RECOVERY_TEACHER_TRACKING" and r.get("projection")]
xy = np.asarray([[r["current_pose"]["x_m"], r["current_pose"]["y_m"]] for r in rows])
progress = np.asarray([r["projection"]["s_m"] for r in rows])
origin = xy.min(axis=0)
local_xy = xy - origin
locations_path = Path("docs/evidence/time_objective_driving_comparison_20260915/collection_locations.json")
locations = json.loads(locations_path.read_text())
stops = np.asarray([r[-1]["map_xy_m"] for r in locations["runs"].values()])
candidates = audit["maximal_spaced_candidates"]
site_xy = np.asarray([r["map_xy_m"] for r in candidates])
stop_distance = np.linalg.norm(site_xy[:, None, :] - stops[None, :, :], axis=2).min(axis=1)
pair_distance = np.linalg.norm(site_xy[:, None, :] - site_xy[None, :, :], axis=2)
np.fill_diagonal(pair_distance, np.inf)
info = {
    "scope": "PLANNING_ONLY_POINT_DISTANCES_NOT_VEHICLE_CLEARANCE",
    "candidate_audit_sha256": hashlib.sha256(audit_path.read_bytes()).hexdigest(),
    "locations_sha256": hashlib.sha256(locations_path.read_bytes()).hexdigest(),
    "progress_range_m": [float(progress.min()), float(progress.max())],
    "map_origin_xy_m": origin.tolist(),
    "minimum_site_separation_xy_m": float(pair_distance.min()),
    "candidate_to_observed_stop_pose_distance_m": [
        {"site_id": f"P{i+1:02d}", "start_s_m": row["start_s_m"], "distance_m": float(distance)}
        for i, (row, distance) in enumerate(zip(candidates, stop_distance))
    ],
    "known_single_pulse_start_limit_m": 300.0,
    "candidates_beyond_current_single_pulse_start_limit_m": [r["start_s_m"] for r in candidates if r["start_s_m"] > 300.0],
    "new_awsim_run": False,
    "new_teacher_data": False,
}
with (root / "layout_audit.json").open("x") as handle:
    json.dump(info, handle, indent=2, allow_nan=False)
png = root / "candidate_layout.png"
if png.exists():
    raise FileExistsError(png)
fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
ax.plot(*local_xy.T, color="#77818d", linewidth=2, label="Recorded normal teacher lap")
lo, hi = audit["provisional_excluded_progress_m"]
mask = (lo <= progress) & (progress <= hi)
ax.plot(*local_xy[mask].T, color="#e07b36", linewidth=6, alpha=.7,
        label=f"Provisional excluded progress {lo}-{hi} m")
ax.scatter(*(stops-origin).T, marker="x", s=75, color="#b42635", zorder=5,
           label="Observed AWSIM stop poses (4 runs)")
ax.scatter(*(site_xy-origin).T, s=65, color="#1265ae", edgecolor="white", zorder=6)
for i, (point, row) in enumerate(zip(site_xy-origin, candidates)):
    ax.annotate(f"P{i+1:02d}  s={row['start_s_m']} m", point, xytext=(7, 8),
                textcoords="offset points", fontsize=9, color="#124271",
                bbox={"facecolor": "white", "alpha": .85, "edgecolor": "none", "pad": 1.5})
ax.set_title("10 additional recovery locations: provisional plan\nNominal entry screening only; disturbances have not been tested", fontsize=13)
ax.set_xlabel("Map x from local origin [m]")
ax.set_ylabel("Map y from local origin [m]")
ax.set_aspect("equal")
ax.margins(.13)
ax.grid(alpha=.2)
ax.legend(loc="upper left", fontsize=8)
fig.savefig(png, dpi=160)
plt.close(fig)
print(json.dumps({"status": "PLANNING_ONLY", "minimum_site_separation_xy_m": info["minimum_site_separation_xy_m"],
                  "minimum_stop_pose_distance_m": float(stop_distance.min()), "plot": str(png)}))
