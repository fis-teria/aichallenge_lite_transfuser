from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
from typing import Mapping

from .lateral import LateralCalibration
from .longitudinal import LongitudinalModeFit


SCHEMA_VERSION = "aic_actuator_calibration_v1"
PROMOTION_STATES = frozenset({"candidate", "shadow", "promoted", "rolled_back"})


@dataclass(frozen=True)
class CalibrationPromotion:
    state: str = "candidate"
    previous_artifact_sha256: str | None = None

    def validate(self) -> None:
        if self.state not in PROMOTION_STATES:
            raise ValueError(f"unsupported calibration promotion state: {self.state!r}")
        if self.previous_artifact_sha256 is not None:
            _require_sha256("previous_artifact_sha256", self.previous_artifact_sha256)


@dataclass(frozen=True)
class CalibrationArtifact:
    artifact_sha256: str
    source_runs: tuple[str, ...]
    source_run_hashes: Mapping[str, str]
    vehicle_profile_sha256: str
    steering: LateralCalibration
    drive: LongitudinalModeFit
    brake: LongitudinalModeFit
    promotion: CalibrationPromotion

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": SCHEMA_VERSION,
            "artifact_sha256": self.artifact_sha256,
            "source_runs": list(self.source_runs),
            "source_run_hashes": dict(sorted(self.source_run_hashes.items())),
            "vehicle_profile_sha256": self.vehicle_profile_sha256,
            "steering": self.steering.to_dict(),
            "drive": self.drive.to_dict(),
            "brake": self.brake.to_dict(),
            "promotion": asdict(self.promotion),
        }


def _require_sha256(name: str, value: str) -> None:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_bytes(payload: Mapping[str, object]) -> bytes:
    return json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def _payload_without_hash(
    *,
    source_runs: tuple[str, ...],
    source_run_hashes: Mapping[str, str],
    vehicle_profile_sha256: str,
    steering: LateralCalibration,
    drive: LongitudinalModeFit,
    brake: LongitudinalModeFit,
    promotion: CalibrationPromotion,
) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "source_runs": list(source_runs),
        "source_run_hashes": dict(sorted(source_run_hashes.items())),
        "vehicle_profile_sha256": vehicle_profile_sha256,
        "steering": steering.to_dict(),
        "drive": drive.to_dict(),
        "brake": brake.to_dict(),
        "promotion": asdict(promotion),
    }


def build_calibration_artifact(
    *,
    source_run_hashes: Mapping[str, str],
    vehicle_profile_sha256: str,
    steering: LateralCalibration,
    drive: LongitudinalModeFit,
    brake: LongitudinalModeFit,
    promotion: CalibrationPromotion | None = None,
) -> CalibrationArtifact:
    if not source_run_hashes:
        raise ValueError("source_run_hashes must not be empty")
    if any(not run_id for run_id in source_run_hashes):
        raise ValueError("source run IDs must not be empty")
    for run_id, digest in source_run_hashes.items():
        _require_sha256(f"source_run_hashes[{run_id!r}]", digest)
    _require_sha256("vehicle_profile_sha256", vehicle_profile_sha256)
    if drive.mode != "drive" or brake.mode != "brake":
        raise ValueError("drive and brake fits must keep distinct modes")
    selected_promotion = promotion or CalibrationPromotion()
    selected_promotion.validate()
    source_runs = tuple(sorted(source_run_hashes))
    payload = _payload_without_hash(
        source_runs=source_runs,
        source_run_hashes=source_run_hashes,
        vehicle_profile_sha256=vehicle_profile_sha256,
        steering=steering,
        drive=drive,
        brake=brake,
        promotion=selected_promotion,
    )
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    return CalibrationArtifact(
        artifact_sha256=digest,
        source_runs=source_runs,
        source_run_hashes=dict(source_run_hashes),
        vehicle_profile_sha256=vehicle_profile_sha256,
        steering=steering,
        drive=drive,
        brake=brake,
        promotion=selected_promotion,
    )


def write_calibration_artifact(artifact: CalibrationArtifact, path: str | Path) -> None:
    expected = build_calibration_artifact(
        source_run_hashes=artifact.source_run_hashes,
        vehicle_profile_sha256=artifact.vehicle_profile_sha256,
        steering=artifact.steering,
        drive=artifact.drive,
        brake=artifact.brake,
        promotion=artifact.promotion,
    )
    if expected.artifact_sha256 != artifact.artifact_sha256:
        raise ValueError("calibration artifact hash mismatch before write")
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(
        json.dumps(artifact.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, destination)


def load_calibration_artifact(path: str | Path) -> CalibrationArtifact:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    required = {
        "schema_version",
        "artifact_sha256",
        "source_runs",
        "source_run_hashes",
        "vehicle_profile_sha256",
        "steering",
        "drive",
        "brake",
        "promotion",
    }
    if set(raw) != required or raw["schema_version"] != SCHEMA_VERSION:
        raise ValueError("invalid calibration artifact top-level contract")
    steering_values = dict(raw["steering"])
    steering_values["valid_speed_range_mps"] = tuple(
        steering_values["valid_speed_range_mps"]
    )
    steering_values["validity_reasons"] = tuple(steering_values["validity_reasons"])
    steering = LateralCalibration(**steering_values)

    def load_longitudinal(values: Mapping[str, object]) -> LongitudinalModeFit:
        normalized = dict(values)
        normalized["valid_speed_range_mps"] = tuple(
            normalized["valid_speed_range_mps"]
        )
        normalized["command_range_mps2"] = tuple(normalized["command_range_mps2"])
        normalized["validity_reasons"] = tuple(normalized["validity_reasons"])
        return LongitudinalModeFit(**normalized)

    drive = load_longitudinal(raw["drive"])
    brake = load_longitudinal(raw["brake"])
    promotion = CalibrationPromotion(**raw["promotion"])
    rebuilt = build_calibration_artifact(
        source_run_hashes=raw["source_run_hashes"],
        vehicle_profile_sha256=raw["vehicle_profile_sha256"],
        steering=steering,
        drive=drive,
        brake=brake,
        promotion=promotion,
    )
    if tuple(raw["source_runs"]) != rebuilt.source_runs:
        raise ValueError("calibration source_runs are not canonical")
    if raw["artifact_sha256"] != rebuilt.artifact_sha256:
        raise ValueError("calibration artifact hash mismatch")
    return rebuilt
