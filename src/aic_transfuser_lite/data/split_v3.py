from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml


SPLIT_CONFIG_FORMAT = "aic_split_config_v1"
SPLIT_MANIFEST_FORMAT = "aic_split_manifest_v1"
SPLITS = ("train", "validation", "test")


@dataclass(frozen=True)
class SplitGroupKey:
    scenario_id: str
    run_family_id: str
    collection_session_id: str
    map_or_course_id: str
    vehicle_profile_id: str
    controller_profile_id: str
    source_dataset_id: str

    def normalized(self, unknown_value: str = "unknown") -> SplitGroupKey:
        return SplitGroupKey(
            **{
                name: _normalize(getattr(self, name), unknown_value)
                for name in self.__dataclass_fields__
            }
        )


@dataclass(frozen=True)
class SplitRunRecord:
    run_id: str
    source_hash: str
    group: SplitGroupKey
    trajectory_fingerprint: str = "unknown"

    def validate(self) -> None:
        if not self.run_id:
            raise ValueError("run_id must be non-empty")
        if len(self.source_hash) != 64 or any(c not in "0123456789abcdef" for c in self.source_hash):
            raise ValueError("source_hash must be a lowercase SHA-256 digest")


@dataclass(frozen=True)
class SplitConfigV3:
    split_seed: int
    ratios: Mapping[str, float]
    fixed_benchmark_group_ids: tuple[str, ...]
    unknown_value: str = "unknown"
    format_version: str = SPLIT_CONFIG_FORMAT

    def validate(self) -> None:
        if self.format_version != SPLIT_CONFIG_FORMAT:
            raise ValueError(f"unsupported split config: {self.format_version!r}")
        if set(self.ratios) != set(SPLITS):
            raise ValueError(f"split ratios must contain exactly {SPLITS}")
        values = [float(self.ratios[name]) for name in SPLITS]
        if any(value <= 0.0 for value in values) or abs(sum(values) - 1.0) > 1e-9:
            raise ValueError("split ratios must be positive and sum to one")
        if not self.unknown_value:
            raise ValueError("unknown_value must be explicit and non-empty")
        if len(set(self.fixed_benchmark_group_ids)) != len(self.fixed_benchmark_group_ids):
            raise ValueError("fixed benchmark group IDs must be unique")
        for value in self.fixed_benchmark_group_ids:
            if len(value) != 24 or any(c not in "0123456789abcdef" for c in value):
                raise ValueError("fixed benchmark group IDs must be 24 lowercase hex characters")


def load_split_config_v3(path: str | Path) -> SplitConfigV3:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("split config root must be a mapping")
    expected = {
        "format_version",
        "split_seed",
        "ratios",
        "fixed_benchmark_group_ids",
        "unknown_value",
    }
    if set(raw) != expected:
        raise ValueError(
            f"split config fields mismatch: missing={sorted(expected-set(raw))}, "
            f"unknown={sorted(set(raw)-expected)}"
        )
    if not isinstance(raw["ratios"], dict) or not isinstance(
        raw["fixed_benchmark_group_ids"], list
    ):
        raise ValueError("ratios must be a mapping and benchmark IDs must be a list")
    config = SplitConfigV3(
        format_version=str(raw["format_version"]),
        split_seed=int(raw["split_seed"]),
        ratios={str(k): float(v) for k, v in raw["ratios"].items()},
        fixed_benchmark_group_ids=tuple(str(v) for v in raw["fixed_benchmark_group_ids"]),
        unknown_value=str(raw["unknown_value"]),
    )
    config.validate()
    return config


def group_id(group: SplitGroupKey, *, unknown_value: str = "unknown") -> str:
    return _canonical_sha(asdict(group.normalized(unknown_value)))[:24]


def build_split_manifest_v3(
    runs: Iterable[SplitRunRecord],
    *,
    dataset_manifest_sha256: str,
    config: SplitConfigV3,
) -> dict[str, Any]:
    """Assign whole leakage-connected run components deterministically.

    Assignment never operates on frames. Runs sharing a source hash, collection
    session, run family, or trajectory fingerprint are unioned before hashing.
    """

    config.validate()
    if len(dataset_manifest_sha256) != 64 or any(
        c not in "0123456789abcdef" for c in dataset_manifest_sha256
    ):
        raise ValueError("dataset_manifest_sha256 must be lowercase SHA-256")
    normalized: list[SplitRunRecord] = []
    seen_runs: set[str] = set()
    for run in runs:
        run.validate()
        if run.run_id in seen_runs:
            raise ValueError(f"duplicate run_id: {run.run_id!r}")
        seen_runs.add(run.run_id)
        normalized.append(
            SplitRunRecord(
                run.run_id,
                run.source_hash,
                run.group.normalized(config.unknown_value),
                _normalize(run.trajectory_fingerprint, config.unknown_value),
            )
        )
    if not normalized:
        raise ValueError("at least one run is required for split assignment")
    normalized.sort(key=lambda run: run.run_id)
    parents = list(range(len(normalized)))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parents[max(left_root, right_root)] = min(left_root, right_root)

    identity_owner: dict[tuple[str, str], int] = {}
    for index, run in enumerate(normalized):
        identities = [("source_hash", run.source_hash)]
        optional = {
            "run_family": run.group.run_family_id,
            "collection_session": run.group.collection_session_id,
            "trajectory_fingerprint": run.trajectory_fingerprint,
        }
        identities.extend(
            (name, value)
            for name, value in optional.items()
            if value != config.unknown_value
        )
        for identity in identities:
            if identity in identity_owner:
                union(index, identity_owner[identity])
            else:
                identity_owner[identity] = index

    components: dict[int, list[int]] = {}
    for index in range(len(normalized)):
        components.setdefault(find(index), []).append(index)
    assignments: list[dict[str, str]] = []
    fixed = set(config.fixed_benchmark_group_ids)
    for indices in components.values():
        group_ids = [group_id(normalized[index].group, unknown_value=config.unknown_value) for index in indices]
        component_payload = [
            {
                "run_id": normalized[index].run_id,
                "source_hash": normalized[index].source_hash,
                "group": asdict(normalized[index].group),
                "trajectory_fingerprint": normalized[index].trajectory_fingerprint,
            }
            for index in indices
        ]
        component_id = _canonical_sha(component_payload)[:24]
        split = (
            "benchmark"
            if any(value in fixed for value in group_ids)
            else _hash_split(component_id, config)
        )
        for index, individual_group_id in zip(indices, group_ids):
            assignments.append(
                {
                    "run_id": normalized[index].run_id,
                    "group_id": individual_group_id,
                    "component_id": component_id,
                    "split": split,
                }
            )
    assignments.sort(key=lambda item: item["run_id"])
    leakage = validate_split_leakage(normalized, assignments, config=config)
    payload: dict[str, Any] = {
        "format_version": SPLIT_MANIFEST_FORMAT,
        "split_seed": config.split_seed,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "assignments": assignments,
        "fixed_benchmark_group_ids": list(config.fixed_benchmark_group_ids),
        "leakage": leakage,
    }
    payload["manifest_sha256"] = _canonical_sha(payload)
    return payload


def validate_split_leakage(
    runs: Iterable[SplitRunRecord],
    assignments: Iterable[Mapping[str, str]],
    *,
    config: SplitConfigV3,
) -> dict[str, Any]:
    by_run = {str(item["run_id"]): str(item["split"]) for item in assignments}
    categories: dict[str, dict[str, set[str]]] = {
        name: {} for name in ("source_hash", "collection_session", "run_family", "trajectory_fingerprint")
    }
    for run in runs:
        split = by_run[run.run_id]
        values = {
            "source_hash": run.source_hash,
            "collection_session": run.group.collection_session_id,
            "run_family": run.group.run_family_id,
            "trajectory_fingerprint": run.trajectory_fingerprint,
        }
        for name, value in values.items():
            if value == config.unknown_value:
                continue
            categories[name].setdefault(value, set()).add(split)
    overlaps = {
        name: sorted(value for value, splits in mapping.items() if len(splits) > 1)
        for name, mapping in categories.items()
    }
    if any(overlaps.values()):
        raise AssertionError(f"split leakage detected: {overlaps}")
    return {
        "run_id_overlap_count": 0,
        "source_hash_overlap_count": len(overlaps["source_hash"]),
        "collection_session_overlap_count": len(overlaps["collection_session"]),
        "run_family_overlap_count": len(overlaps["run_family"]),
        "trajectory_fingerprint_overlap_count": len(overlaps["trajectory_fingerprint"]),
        "status": "PASS",
    }


def _hash_split(component_id: str, config: SplitConfigV3) -> str:
    digest = hashlib.sha256(f"{config.split_seed}:{component_id}".encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], "big") / 2**64
    cumulative = 0.0
    for name in SPLITS:
        cumulative += float(config.ratios[name])
        if value < cumulative:
            return name
    return SPLITS[-1]


def _normalize(value: str, unknown_value: str) -> str:
    normalized = str(value).strip()
    return normalized if normalized else unknown_value


def _canonical_sha(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
