"""Bounded offline map feasibility; never installs a reference or starts AWSIM."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from aic_transfuser_lite.data.recovery_reference_v3 import (
    RecoveryReferenceConfigV3, RecoverySegmentRequestV3, _densify_polyline,
    _geometry_name, _offset_profile, generate_recovery_reference_v3, load_occupancy_map_v3)
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("OUTPUT_EXISTS")
    points = load_pose_course(args.inputs/"base.csv")
    occupancy = load_occupancy_map_v3(args.inputs/"occupancy_grid_map.yaml")
    s, x, y, psi, kappa = [np.array([getattr(p, key) for p in points]) for key in (
        "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm")]
    rows = []
    for offset in (.2, .3, .4):
        for side in ("left", "right"):
            request = RecoverySegmentRequestV3(f"{side}_{round(100*offset):03d}", side, offset,
                                                "left_curve", (76., 82.))
            config = RecoveryReferenceConfigV3(4., 6., 6., 8., 1.4, .015, .15, (request,), .25)
            candidates = []
            for start in np.flatnonzero((s >= 76.) & (s <= 82.)):
                end = int(np.searchsorted(s, s[start]+config.segment_length_m, side="right")-1)
                assert end-start >= 4 and s[start]+config.segment_length_m <= s[-1]
                mean_k = float(np.mean(kappa[start:end+1]))
                curvature_ok = bool(np.max(np.abs(kappa[start:end+1])) <= .25)
                geometry_ok = _geometry_name(mean_k, .015) == "left_curve"
                offset_profile = _offset_profile(s[start:end+1]-s[start], request, config)
                xx, yy = x.copy(), y.copy()
                xx[start:end+1] -= np.sin(psi[start:end+1])*offset_profile
                yy[start:end+1] += np.cos(psi[start:end+1])*offset_profile
                dense_x, dense_y = _densify_polyline(xx[start:end+1], yy[start:end+1],
                                                     maximum_step_m=occupancy.resolution_m_per_px)
                clear = occupancy.footprint_is_free(dense_x, dense_y, 1.4)
                failed_points = [(float(a), float(b)) for a,b in zip(dense_x, dense_y)
                    if not occupancy.footprint_is_free(np.array([a]), np.array([b]), 1.4)]
                joined_x, joined_y = _densify_polyline(xx[max(0,start-1):end+2], yy[max(0,start-1):end+2],
                                                      maximum_step_m=occupancy.resolution_m_per_px)
                candidates.append(dict(start_s_m=float(s[start]), end_s_m=float(s[start]+16.),
                    mean_base_curvature_inv_m=mean_k, base_curvature_pass=curvature_ok,
                    left_curve_pass=geometry_ok, center_clearance_1p4m_pass=bool(clear),
                    joined_center_clearance_1p4m_pass=bool(occupancy.footprint_is_free(joined_x,joined_y,1.4)),
                    failed_dense_points=len(failed_points), first_failed_map_xy_m=failed_points[:3]))
            entry = dict(side=side, offset_m=offset, config=asdict(config), candidates=candidates)
            try:
                generated = generate_recovery_reference_v3(points, occupancy, config)
            except ValueError as exc:
                assert str(exc) == f"no safe, non-overlapping interval satisfies segment {request.segment_id!r}"
                assert not any(c["base_curvature_pass"] and c["left_curve_pass"]
                               and c["center_clearance_1p4m_pass"] for c in candidates)
                entry.update(generation_pass=False, error=str(exc))
            else:
                selected = generated.selected_segments[0]
                candidate = next(c for c in candidates if c["start_s_m"] == selected["base_start_s_m"])
                assert candidate["center_clearance_1p4m_pass"]
                entry.update(generation_pass=True, selected=selected,
                             joined_clearance_pass=candidate["joined_center_clearance_1p4m_pass"])
            rows.append(entry)
    # Existing 20cm pilot geometry must remain reproducible under unchanged gates.
    assert all(r["generation_pass"] and r["joined_clearance_pass"] for r in rows if r["offset_m"] == .2)
    report = dict(scope="OFFLINE_REFERENCE_CENTER_CLEARANCE_NOT_FULL_BODY_OR_DRIVING_PROOF",
        source_commit=subprocess.check_output(["git", "rev-parse", "HEAD"],text=True).strip(),
        input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in args.inputs.iterdir() if p.is_file()},
        clearance_m=1.4, maximum_dense_step_m=occupancy.resolution_m_per_px,
        examined_offsets_m=[.2,.3,.4], runtime_changed=False, deployed=False,
        independent_heading_perturbation=False,
        heading_note="Existing excursion geometry derives heading from the offset polyline; no independent yaw parameter",
        rows=rows)
    with args.output.open("x") as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__ == "__main__":
    main()
