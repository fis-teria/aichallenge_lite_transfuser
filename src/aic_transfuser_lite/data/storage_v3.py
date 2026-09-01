from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Any, Mapping, Protocol
import uuid

import numpy as np
from PIL import Image
import yaml

from .canonical_schema_v3 import DatasetManifestV3


@dataclass(frozen=True)
class StorageSummary:
    output_root: Path
    manifest_sha256: str
    file_count: int
    size_bytes: int


class CanonicalStorageBackend(Protocol):
    def write_run(self, run: Any) -> None: ...
    def write_sample(self, sample: Any) -> None: ...
    def write_event(self, event: Mapping[str, Any]) -> None: ...
    def finalize(self) -> StorageSummary: ...


class CsvNpyJpegBackend:
    """Atomic initial V3 storage backend using CSV, NPY, JPEG, YAML, and JSONL.

    The destination must not exist. Data is written to a sibling staging
    directory marked ``.incomplete`` and becomes visible only after manifest
    hashing and an atomic rename.
    """

    def __init__(self, output_root: str | Path, manifest: DatasetManifestV3) -> None:
        self.output_root = Path(output_root).resolve()
        manifest.validate()
        self.manifest = manifest
        if self.output_root.exists():
            raise FileExistsError(f"Dataset output already exists: {self.output_root}")
        self.output_root.parent.mkdir(parents=True, exist_ok=True)
        self.staging_root = self.output_root.with_name(
            f".{self.output_root.name}.tmp-{os.getpid()}-{uuid.uuid4().hex[:8]}"
        )
        self.staging_root.mkdir()
        (self.staging_root / ".incomplete").write_text("incomplete\n", encoding="utf-8")
        for name in ("images", "lidar", "states", "trajectories", "controls"):
            (self.staging_root / name).mkdir()
        self._runs: list[dict[str, Any]] = []
        self._samples: list[dict[str, Any]] = []
        self._events: list[dict[str, Any]] = []
        self._finalized = False

    def __enter__(self) -> CsvNpyJpegBackend:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if exc_type is not None or not self._finalized:
            self.abort()

    def write_run(self, run: Any) -> None:
        self._ensure_open()
        value = _record_dict(run)
        if not value.get("run_id"):
            raise ValueError("run record requires run_id")
        self._runs.append(value)

    def write_sample(self, sample: Any) -> None:
        self._ensure_open()
        value = _record_dict(sample)
        if not value.get("sample_id"):
            raise ValueError("sample record requires sample_id")
        self._samples.append(value)

    def write_event(self, event: Mapping[str, Any]) -> None:
        self._ensure_open()
        value = dict(event)
        if not value:
            raise ValueError("event record must not be empty")
        self._events.append(value)

    def write_image(
        self, relative_path: str | Path, image: Image.Image | np.ndarray, *, quality: int = 90
    ) -> str:
        """Write one RGB JPEG below ``images/`` and return its POSIX path."""

        self._ensure_open()
        if not 1 <= quality <= 100:
            raise ValueError("JPEG quality must be within [1,100]")
        destination, relative = self._asset_path("images", relative_path, ".jpg")
        prepared = image if isinstance(image, Image.Image) else Image.fromarray(np.asarray(image))
        prepared.convert("RGB").save(destination, format="JPEG", quality=quality)
        return relative

    def write_array(
        self, category: str, relative_path: str | Path, value: np.ndarray
    ) -> str:
        """Write a non-object NPY array below a declared storage category."""

        self._ensure_open()
        if category not in {"lidar", "states", "trajectories", "controls"}:
            raise ValueError(f"unsupported array category: {category!r}")
        array = np.asarray(value)
        if array.dtype.hasobject:
            raise ValueError("object arrays are not supported")
        destination, relative = self._asset_path(category, relative_path, ".npy")
        np.save(destination, array, allow_pickle=False)
        return relative

    def finalize(self) -> StorageSummary:
        self._ensure_open()
        self._write_csv(self.staging_root / "runs.csv", self._runs)
        self._write_csv(self.staging_root / "samples.csv", self._samples)
        event_text = "".join(
            json.dumps(event, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n"
            for event in self._events
        )
        (self.staging_root / "events.jsonl").write_text(event_text, encoding="utf-8")
        incomplete = self.staging_root / ".incomplete"
        incomplete.unlink()
        files = _file_inventory(self.staging_root)
        manifest_payload = {
            **self.manifest.to_dict(),
            "complete": True,
            "files": files,
        }
        manifest_sha = _canonical_sha(manifest_payload)
        manifest_output = {**manifest_payload, "manifest_sha256": manifest_sha}
        (self.staging_root / "manifest.yaml").write_text(
            yaml.safe_dump(manifest_output, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        if self.output_root.exists():
            raise FileExistsError(f"Dataset output appeared during finalization: {self.output_root}")
        os.replace(self.staging_root, self.output_root)
        self._finalized = True
        final_files = [path for path in self.output_root.rglob("*") if path.is_file()]
        return StorageSummary(
            output_root=self.output_root,
            manifest_sha256=manifest_sha,
            file_count=len(final_files),
            size_bytes=sum(path.stat().st_size for path in final_files),
        )

    def abort(self) -> None:
        """Remove only this writer's uniquely named incomplete staging directory."""

        if self.staging_root.exists():
            expected_parent = self.output_root.parent.resolve()
            actual_parent = self.staging_root.resolve().parent
            if actual_parent != expected_parent or not self.staging_root.name.startswith(
                f".{self.output_root.name}.tmp-"
            ):
                raise RuntimeError(f"Refusing to remove unexpected staging path: {self.staging_root}")
            shutil.rmtree(self.staging_root)

    def _ensure_open(self) -> None:
        if self._finalized:
            raise RuntimeError("storage backend is already finalized")
        if not self.staging_root.is_dir():
            raise RuntimeError("storage staging directory is unavailable")

    def _asset_path(
        self, category: str, relative_path: str | Path, extension: str
    ) -> tuple[Path, str]:
        relative = Path(relative_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("asset path must be relative and stay inside its category")
        if relative.suffix.lower() != extension:
            relative = relative.with_suffix(extension)
        destination = self.staging_root / category / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            raise FileExistsError(f"asset already exists: {destination}")
        return destination, (Path(category) / relative).as_posix()

    @staticmethod
    def _write_csv(path: Path, records: list[dict[str, Any]]) -> None:
        fieldnames = sorted({key for record in records for key in record})
        with path.open("w", newline="", encoding="utf-8") as stream:
            if not fieldnames:
                return
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
            writer.writeheader()
            for record in records:
                writer.writerow({name: _csv_value(record.get(name)) for name in fieldnames})


def validate_complete_dataset(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    manifest_path = root / "manifest.yaml"
    if not root.is_dir() or not manifest_path.is_file() or (root / ".incomplete").exists():
        raise ValueError(f"Dataset is not atomically complete: {root}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("complete") is not True:
        raise ValueError("Dataset manifest is missing complete=true")
    expected = str(manifest.get("manifest_sha256", ""))
    payload = dict(manifest)
    payload.pop("manifest_sha256", None)
    actual = _canonical_sha(payload)
    if expected != actual:
        raise ValueError(f"Dataset manifest hash mismatch: actual={actual}, expected={expected}")
    return manifest


def _record_dict(value: Any) -> dict[str, Any]:
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Mapping):
        return dict(value)
    raise TypeError("storage records must be dataclasses or mappings")


def _csv_value(value: Any) -> Any:
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False)
    return value


def _file_inventory(root: Path) -> list[dict[str, Any]]:
    result = []
    for path in sorted((item for item in root.rglob("*") if item.is_file())):
        relative = path.relative_to(root).as_posix()
        result.append(
            {
                "path": relative,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    return result


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
