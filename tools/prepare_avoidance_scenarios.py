"""Prepare static-NPC H2H collection inputs; never launch AWSIM or a teacher.

Geometry uses map XY [m], yaw [rad] and reference arc length [m]. The H2H
boundary explicitly converts yaw to degrees. Speed caps live in the collection
manifest, NOT in a H2H field that its runner would silently ignore.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def positive(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def validate_plan(plan: dict[str, Any]) -> None:
    if plan["schema_version"] != 1 or not re.fullmatch(r"[a-z][a-z0-9_]+", plan["suite_id"]):
        raise ValueError("unsupported schema or invalid suite_id")
    if not re.fullmatch(r"[0-9a-f]{64}", plan["reference_sha256"]):
        raise ValueError("explicit reference SHA-256 required")
    if not isinstance(plan["seed"], int) or not 0 <= plan["seed"] <= 2**31 - 1:
        raise ValueError("seed must be a nonnegative int32")
    if not isinstance(plan["pilot_limit"], int) or not 1 <= plan["pilot_limit"] <= 100:
        raise ValueError("pilot_limit must be 1..100")
    for key in ("reference_resolution_m", "min_clearance_m", "finish_after_actor_m", "split_guard_distance_m"):
        positive(plan[key], key)
    if plan["min_clearance_m"] < .3 or plan["finish_after_actor_m"] < 25.:
        raise ValueError("preserve 0.30 m clearance and at least 25 m of post-obstacle travel")
    for key, limit in (("sim_timeout_s", 1800), ("wall_timeout_s", 3600)):
        if positive(plan[key], key) > limit:
            raise ValueError(f"{key} exceeds finite collection budget")
    if plan["wall_timeout_s"] <= plan["sim_timeout_s"]:
        raise ValueError("wall timeout must leave startup/shutdown time")
    speeds = plan["teacher_speed_caps_mps"]
    if not speeds or len(set(speeds)) != len(speeds):
        raise ValueError("unique speed caps required")
    if any(positive(v, "speed cap [m/s]") > 10. / 3.6 + 1e-9 for v in speeds):
        raise ValueError("this suite covers speeds up to 10 km/h")
    if len({round(v * 360) for v in speeds}) != len(speeds):
        raise ValueError("speed caps collide at scenario ID precision (0.01 km/h)")
    stations = plan["station_offsets_m"]
    if not stations or len(set(stations)) != len(stations):
        raise ValueError("unique station offsets required")
    for value in [*stations, *plan["lateral_offsets_m"].values()]:
        if isinstance(value, bool) or not math.isfinite(value) or abs(value) > 2.:
            raise ValueError("offsets must be finite metres within +/-2 m")
    if set(plan["lateral_offsets_m"]) != {"left", "center", "right"}:
        raise ValueError("left/center/right placement offsets required")
    groups: dict[str, str] = {}
    ids: set[str] = set()
    for site in plan["sites"]:
        if not re.fullmatch(r"[a-z][a-z0-9_]+", site["id"]) or site["id"] in ids:
            raise ValueError("unique safe site IDs required")
        ids.add(site["id"])
        if site["context"] not in {"straight", "entry", "apex", "exit"} or site["split"] not in {"train", "validation"}:
            raise ValueError("unknown road context or split")
        if groups.setdefault(site["group"], site["split"]) != site["split"]:
            raise ValueError("related site group crosses dataset splits")
    if not ids or set(groups.values()) != {"train", "validation"}:
        raise ValueError("nonempty train and validation groups required")


def cyclic_distance(a_m: float, b_m: float, length_m: float) -> float:
    positive(length_m, "reference length [m]")
    delta = abs(a_m - b_m) % length_m
    return min(delta, length_m - delta)


def check_split_separation(rows: list[dict[str, Any]], length_m: float, guard_m: float) -> None:
    """Check every requested station, including rejected placements, before generation."""
    for i, a in enumerate(rows):
        for b in rows[i + 1:]:
            if a["split"] != b["split"] and cyclic_distance(a["teacher_s_m"], b["teacher_s_m"], length_m) < guard_m:
                raise ValueError(f"nearby placements cross splits: {a['site']} / {b['site']}")


def finish_rule(actor_s_m: float, length_m: float, after_m: float) -> dict[str, Any]:
    """A threshold behind the actor, or a lap when that threshold crosses the seam."""
    positive(length_m, "monitor reference length [m]")
    if not math.isfinite(actor_s_m) or not 0 <= actor_s_m < length_m:
        raise ValueError("actor station must lie on the circular monitor reference")
    positive(after_m, "post-obstacle distance [m]")
    if actor_s_m + after_m >= length_m:
        # One lap would truncate recovery at the seam. Keep the first encounter
        # through the next lap; the second encounter's incomplete tail is excluded by audit.
        return {"ego_laps_completed": 2}
    if actor_s_m < 35.:
        return {"ego_laps_completed": 1}
    return {"ego_reference_s_greater_than": round(actor_s_m + after_m, 3)}


def scenario_document(plan: dict[str, Any], case_id: str, placements: list[dict[str, Any]],
                      monitor_length_m: float) -> dict[str, Any]:
    if not 1 <= len(placements) <= 3 or len({p["site"] for p in placements}) != len(placements):
        raise ValueError("one to three actors at distinct sites required")
    if len({p["split"] for p in placements}) != 1:
        raise ValueError("one complete run cannot mix train and validation sites")
    actors = []
    for index, p in enumerate(placements, 1):
        pose = p["map_pose"]
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError("map_pose must be [x_m, y_m, yaw_rad]")
        actors.append(dict(id=f"parked_{index}", profile="static_physical",
                           pose=dict(map_xy=pose[:2], yaw=math.degrees(pose[2]))))
    finish = (finish_rule(placements[0]["monitor_s_m"], monitor_length_m, plan["finish_after_actor_m"])
              if len(placements) == 1 else {"ego_laps_completed": 1})
    expect: dict[str, Any] = dict(timeout_sec=plan["sim_timeout_s"], no_collision=True,
                                  no_off_track=True, finish=finish)
    if len(placements) == 1:
        expect.update(clearance=dict(actor="parked_1", min_m=plan["min_clearance_m"]),
                      avoidance_start=dict(actor="parked_1", search_within_m=25.,
                                           lateral_threshold_m=.2, min_duration_sec=.2))
    seed = (plan["seed"] + int(hashlib.sha256(case_id.encode()).hexdigest()[:8], 16)) % (2**31)
    return dict(schema_version=1, name=case_id,
                description="Teacher collection input. Bind revised MPPI and the queue speed cap before driving.",
                seed=seed,
                simulator=dict(start_mode="count", start_count_seconds=5, laps="unlimited",
                               timeout=plan["sim_timeout_s"], collisions="on", render="window",
                               camera="gpu", lidar="gpu", npcs=0, handicap="off", ranking="off", wall_recovery="on"),
                runtime=dict(sample_rate_hz=20, ready_timeout_sec=180, playstart_stall_sec=30, rosbag=True),
                ego=dict(submission=dict(type="docker_image", image=plan["image"]), pose=dict(start_grid=1)),
                actors=actors, expect=expect)


def select_pilot(rows: list[dict[str, Any]], limit: int) -> list[str]:
    """Low-speed singles, round-robin across sites; keep startup warnings for later."""
    singles = [r for r in rows if r["stage"] == "single" and not r["warnings"]]
    if not singles:
        return []
    speed = min(r["teacher_speed_cap_mps"] for r in singles)
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in singles:
        if math.isclose(row["teacher_speed_cap_mps"], speed):
            groups[row["placements"][0]["site"]].append(row)
    # Straight precedes entry/apex/exit for the first round, but none monopolizes the pilot.
    context_order = {"straight": 0, "entry": 1, "apex": 2, "exit": 3}
    sites = sorted(groups, key=lambda s: (context_order[groups[s][0]["placements"][0]["context"]], s))
    result = []
    for i in range(max(map(len, groups.values()))):
        for site in sites:
            if i < len(groups[site]) and len(result) < limit:
                result.append(groups[site][i]["case_id"])
    return result


def prepare(plan: dict[str, Any], *, repo: Path, reference_csv: Path, output: Path) -> dict[str, Any]:
    validate_plan(plan)
    if output.exists():
        raise FileExistsError(f"preserve existing suite: {output}")
    if sha256(reference_csv) != plan["reference_sha256"]:
        raise ValueError("teacher reference differs from the reviewed plan")
    sys.path.insert(0, str(repo / "head_to_head/scenario_tool"))
    from scenario_tool import calibration, compiler, scenario, yamlio
    from scenario_tool.context import Context
    from scenario_tool.geometry import ReferenceLine, footprint, polygon_distance
    from scenario_tool.occupancy import OccupancyGrid
    from scenario_tool.regression import detect_road_sections, road_positions

    context = Context(repo)
    maps = context.map_paths()
    monitor_line = maps.reference_line()
    line = ReferenceLine.from_mpc_config(reference_csv, plan["reference_resolution_m"],
                                          plan["reference_smoothing_distance"], True)
    occupancy = OccupancyGrid(maps.occupancy_yaml)
    if not occupancy.available:
        raise ValueError(f"occupancy map unavailable: {occupancy.reason}")
    transform = calibration.load(context, require=True)
    stale = calibration.staleness(context, transform, maps.reference_csv)
    if stale:
        raise ValueError(f"calibration is stale: {stale}")
    source_files = {reference_csv, maps.reference_csv, maps.occupancy_yaml, occupancy.image_path,
                    maps.config_path, *Path(scenario.__file__).parent.glob("*.py")}
    before = {str(p): sha256(p) for p in sorted(source_files)}
    sections = {s.id: s for s in detect_road_sections(line)}
    placements = []
    used: set[tuple[float, float, float]] = set()
    for site in plan["sites"]:
        section = sections[site["section"]]
        base = (section.s_start - 4. if site["context"] == "entry" else
                section.s_end + 4. if site["context"] == "exit" else section.target_s)
        for ordinal, shift in enumerate(plan["station_offsets_m"]):
            s = (base + shift) % line.length
            corridor = road_positions(line, occupancy, replace(section, target_s=s))
            nominal = max(corridor.safe_min, min(corridor.safe_max, 0.))
            for side, delta in plan["lateral_offsets_m"].items():
                d = round(max(corridor.safe_min, min(corridor.safe_max, nominal + delta)), 3)
                pose = line.pose_at(s, d)
                polygon = footprint(pose)
                row = dict(placement_id=f"{site['id']}_s{ordinal}_{side}", site=site["id"],
                           context=site["context"], group=site["group"], split=site["split"],
                           teacher_s_m=s, station_offset_m=shift, requested_lateral_offset_m=delta,
                           actual_lateral_m=d, map_pose=[pose.x, pose.y, pose.yaw], status="PENDING")
                placements.append(row)
                key = (round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 4))
                if key in used:
                    row["status"] = "DUPLICATE_AFTER_BOUNDARY_CLIP"
                    continue
                used.add(key)
                if occupancy.polygon_overlap_depth(polygon) > 0:
                    row["status"] = "REJECTED_MAP_OVERLAP"
                    continue
                if polygon_distance(polygon, footprint(line.pose_at(s))) > .1:
                    row["status"] = "REJECTED_NO_REFERENCE_INTERACTION"
                    continue
                passing = []
                for k in range(-30, 31):
                    lateral = k / 10.
                    candidate = footprint(line.pose_at(s, lateral))
                    gap = polygon_distance(polygon, candidate)
                    if gap >= plan["min_clearance_m"] and occupancy.polygon_overlap_depth(candidate) == 0:
                        passing.append(dict(lateral_m=lateral, clearance_m=gap))
                row["cross_section_passing_candidates"] = passing
                if not passing:
                    row["status"] = "HELD_NO_CROSS_SECTION_PASSAGE"
                    continue
                row["monitor_s_m"], row["monitor_d_m"] = monitor_line.project(pose.x, pose.y)
                row["status"] = "STATIC_GEOMETRY_ONLY"
    check_split_separation(placements, line.length, plan["split_guard_distance_m"])
    accepted = [p for p in placements if p["status"] == "STATIC_GEOMETRY_ONLY"]
    if not accepted or {p["split"] for p in accepted} != {"train", "validation"}:
        raise ValueError("no usable placement in one of the dataset splits")
    rows: list[dict[str, Any]] = []
    generated: list[tuple[str, dict[str, Any], dict[str, Any]]] = []

    def add_case(members: list[dict[str, Any]], speed: float, case_id: str, stage: str) -> None:
        doc = scenario_document(plan, case_id, members, monitor_line.length)
        warnings = scenario.validate_document(doc, context)
        resolved = scenario.resolve(doc, context, monitor_line, transform, warnings=warnings)
        native = compiler.build_awsim_scenario(resolved, True)
        if len(native["vehicles"]) != len(members) + 1 or any(not v.get("static") for k, v in native["vehicles"].items() if k != "1"):
            raise ValueError("compiled static-actor identity mismatch")
        # Verify resolution did not replace a teacher tangent with another Reference's tangent.
        for p, actor in zip(members, resolved.entities[1:], strict=True):
            if actor.map_pose is None or abs(actor.map_pose.yaw - p["map_pose"][2]) > 1e-8:
                raise ValueError("H2H changed the explicit actor yaw")
        rows.append(dict(case_id=case_id, stage=stage, split=members[0]["split"],
                         split_groups=sorted({p["group"] for p in members}), placements=members,
                         teacher_speed_cap_mps=speed, teacher_speed_cap_kmh=speed * 3.6,
                         speed_source="REQUIRED_TEACHER_LAUNCH_BINDING_NOT_H2H_YAML",
                         scenario=f"scenarios/{case_id}.yaml", native=f"native/{case_id}.json",
                         sim_timeout_s=plan["sim_timeout_s"], wall_timeout_s=plan["wall_timeout_s"],
                         warnings=resolved.warnings, execution_ready=False,
                         prerequisites=["revised_teacher_identity_and_runtime_speed_verified", "placement_startup_probe",
                                        *(["all_member_single_cases_passed"] if stage == "multi_after_singles" else [])]))
        generated.append((case_id, doc, native))

    for placement in accepted:
        for speed in sorted(plan["teacher_speed_caps_mps"]):
            tag = round(speed * 3.6 * 100)
            add_case([placement], speed, f"{plan['suite_id']}_{placement['placement_id']}_v{tag:04d}", "single")
    # Three well-separated training sites; never combine validation placements with a train run.
    multi = []
    for site_id in ("straight_b", "straight_c", "entry_d"):
        candidates = [p for p in accepted if p["site"] == site_id and p["split"] == "train"]
        if candidates:
            multi.append(min(candidates, key=lambda p: (abs(p["actual_lateral_m"]), p["placement_id"])))
    if len(multi) == 3 and all(cyclic_distance(a["teacher_s_m"], b["teacher_s_m"], line.length) >= 35.
                                for i, a in enumerate(multi) for b in multi[i + 1:]):
        for speed in sorted(plan["teacher_speed_caps_mps"]):
            add_case(multi, speed, f"{plan['suite_id']}_three_sites_v{round(speed * 360):04d}", "multi_after_singles")
    pilot_ids = select_pilot(rows, plan["pilot_limit"])
    for row in rows:
        row["pilot"] = row["case_id"] in pilot_ids
    after = {str(p): sha256(p) for p in sorted(source_files)}
    if before != after:
        raise RuntimeError("H2H/map inputs changed during preparation; rerun against a stable snapshot")
    output.mkdir(parents=True, exist_ok=False)
    (output / "scenarios").mkdir()
    (output / "native").mkdir()
    for case_id, doc, native in generated:
        path = output / "scenarios" / f"{case_id}.yaml"
        yamlio.dump_file(path, doc)
        # Validate the serialized document as well; YAML float/unit changes must not be hidden.
        scenario.validate_document(yamlio.load_file(path), context)
        write_json(output / "native" / f"{case_id}.json", native)
    for row in rows:
        row["scenario_sha256"] = sha256(output / row["scenario"])
        row["native_sha256"] = sha256(output / row["native"])
    write_json(output / "plan.json", plan)
    write_json(output / "placements.json", placements)
    write_json(output / "pilot.json", pilot_ids)
    with (output / "collection_queue.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    summary = dict(schema_version=1, suite_id=plan["suite_id"], driving_started=False, training_started=False,
                   teacher_bound=False, awsim_modified=False, h2h_inputs_sha256=before,
                   reference_length_m=line.length, monitor_reference=maps.describe(),
                   requested_sites=len(plan["sites"]), placements_by_status=dict(Counter(p["status"] for p in placements)),
                   single_scenarios=sum(r["stage"] == "single" for r in rows),
                   multi_scenarios=sum(r["stage"] == "multi_after_singles" for r in rows), pilot_count=len(pilot_ids),
                   singles_by_split=dict(Counter(r["split"] for r in rows if r["stage"] == "single")),
                   singles_by_context=dict(Counter(r["placements"][0]["context"] for r in rows if r["stage"] == "single")),
                   warning_scenarios=sum(bool(r["warnings"]) for r in rows),
                   live_success_events=0, sealed_test_created=False,
                   limitations=["Cross-section clearance is not swept-path feasibility.",
                                "H2H startup and actual native actor pose require live verification.",
                                "Speed caps must be bound and checked on the revised teacher, not inferred from filenames.",
                                "Validation holds out obstacle placements, not the already-known map.",
                                "No moving vehicles, arbitrary object shapes, or forced ego deviations in this suite."])
    write_json(output / "summary.json", summary)
    files = {str(p.relative_to(output)).replace("\\", "/"): sha256(p) for p in sorted(output.rglob("*")) if p.is_file()}
    write_json(output / "manifest.json", dict(schema_version=1, files_sha256=files))
    return {k: v for k, v in summary.items() if k != "h2h_inputs_sha256"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plan", type=Path, required=True)
    ap.add_argument("--h2h-repo", type=Path, required=True)
    ap.add_argument("--teacher-reference-csv", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    result = prepare(json.loads(args.plan.read_text(encoding="utf-8")), repo=args.h2h_repo.resolve(),
                     reference_csv=args.teacher_reference_csv.resolve(), output=args.output.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
