"""Verified bounded-memory sensor cache. Missing histories remain exactly zero."""
from __future__ import annotations
from dataclasses import asdict, fields, replace
import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from PIL import Image
import torch
from torch.utils.data import Dataset
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .image_preprocess import preprocess_image
from .mcap_converter_v2 import TimedImage, TimedLidar
from .time_archive_v1 import _rename_without_replace
from .time_corpus_v1 import EventWindows
from .time_dataset_v1 import TimeDatasetConfig, TimeSample, assemble_time_inputs, _lidar
from .time_split_v1 import assert_split_membership, content_sha256, validate_time_split
from .time_sqlite_reader_v1 import load_event, read_time_sqlite_run
from .time_teacher_v1 import TimeTeacher

FORMAT = "time_training_cache_v1"


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def _records(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _checked_path(root: Path, name: str) -> Path:
    path = (root / name).resolve(strict=True)
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError("artifact outside declared root")
    return path


def verify_time_training_cache(root: Path) -> dict[str, Any]:
    identity = _json(root / "identity.json")
    if identity.get("format") != FORMAT or identity.get("manifest_sha256") != content_sha256(
            {k: v for k, v in identity.items() if k != "manifest_sha256"}):
        raise ValueError("cache identity mismatch")
    if not identity.get("cache_files"):
        raise ValueError("cache inventory missing")
    validate_time_split(identity["split_manifest"], require_verified=True)
    for row in identity["cache_files"]:
        path = _checked_path(root, row["path"])
        if path.stat().st_size != row["bytes"] or _sha(path) != row["sha256"]:
            raise ValueError(f"cache file mismatch: {row['path']}")
    return identity


def cached_inputs(arrays: dict[str, np.ndarray], i: int, camera: np.ndarray,
                  lidar: np.ndarray, config: TimeDatasetConfig) -> ModelBatchV3 | None:
    """One input, RGB [T,3,H,W], scan [T,2,P]; -1 never indexes real data."""
    if not bool(arrays["input_valid"][i]):
        return None
    cr, lr = arrays["camera_refs"][i], arrays["lidar_refs"][i]
    cm, lm = cr >= 0, lr >= 0
    image = np.zeros((len(cr), *config.image_shape), np.float32)
    scan = np.zeros((len(lr), *config.lidar_shape), np.float32)
    for j in np.flatnonzero(cm):
        image[j] = preprocess_image(camera[cr[j]], height=config.image_shape[1], width=config.image_shape[2]).numpy()
    if lm.any():
        scan[lm] = lidar[lr[lm]]
    return ModelBatchV3(torch.from_numpy(image[None]), torch.from_numpy(cm[None].copy()),
        torch.from_numpy(scan[None]), torch.from_numpy(lm[None].copy()),
        torch.from_numpy(arrays["ego"][i:i+1].copy()), torch.from_numpy(arrays["ego_mask"][i:i+1].copy()),
        torch.from_numpy(arrays["command"][i:i+1].copy()), torch.from_numpy(arrays["command_mask"][i:i+1].copy()),
        torch.from_numpy(arrays["dt"][i:i+1].copy()), requested_outputs=frozenset({"trajectory"}))


def _prepare_run(source: Path, destination: Path, run_id: str, config: TimeDatasetConfig) -> dict[str, Any]:
    anchors = _records(source / "anchors.jsonl")
    with np.load(source / "teachers.npz", allow_pickle=False) as bundle:
        labels = {k: bundle[k].copy() for k in ("xy_m", "xy_mask", "velocity_mps", "velocity_mask", "interval_mask")}
    if len(anchors) != len(labels["xy_m"]):
        raise ValueError("anchor/label size mismatch")
    index = read_time_sqlite_run(source / "raw", run_id)
    by_seq = {e.sequence: e for e in index.events}
    windows = EventWindows(index.events)
    bounds = {e.epoch_id: (e.first_sim_stamp_ns, e.last_sim_stamp_ns) for e in index.epochs}
    destination.mkdir(parents=True, exist_ok=False)
    ids = {role: sorted({int(rid) for a in anchors for slot in a["history_row_ids"][role] for rid in slot})
           for role in ("camera", "lidar")}
    maps = {role: {rid: i for i, rid in enumerate(values)} for role, values in ids.items()}
    camera = np.lib.format.open_memmap(destination / "camera_rgb.npy", mode="w+", dtype=np.uint8,
        shape=(len(ids["camera"]), config.image_shape[1], config.image_shape[2], 3))
    lidar = np.lib.format.open_memmap(destination / "lidar.npy", mode="w+", dtype=np.float32,
                                     shape=(len(ids["lidar"]), *config.lidar_shape))
    for role, values in ids.items():
        for i, rid in enumerate(values):
            event = by_seq[rid]
            if event.role != role:
                raise ValueError("sensor reference role mismatch")
            payload = load_event(source / "raw", event).payload
            if role == "camera":
                camera[i] = np.asarray(Image.fromarray(payload.image_rgb).convert("RGB").resize(
                    (config.image_shape[2], config.image_shape[1]), getattr(Image, "Resampling", Image).BILINEAR))
            else:
                lidar[i] = _lidar(payload, config)
    camera.flush(); lidar.flush()
    n = len(anchors)
    arrays = {"ego": np.zeros((n, config.ego_history_length, 4), np.float32),
        "ego_mask": np.zeros((n, config.ego_history_length, 4), bool),
        "command": np.zeros((n, config.command_history_length, 3), np.float32),
        "command_mask": np.zeros((n, config.command_history_length), bool),
        "dt": np.zeros((n, config.camera_history_length, 2), np.float32), "input_valid": np.zeros(n, bool),
        "camera_refs": np.full((n, config.camera_history_length), -1, np.int64),
        "lidar_refs": np.full((n, config.lidar_history_length), -1, np.int64)}
    small = replace(config, image_shape=(3, 2, 2), lidar_shape=(2, 2))
    zero_rgb = np.zeros((2, 2, 3), np.uint8)
    def sensor_placeholder(e: Any) -> Any:
        if e.role == "camera":
            return replace(e, payload=TimedImage(e.capture_ns, zero_rgb))
        if e.role == "lidar":
            return replace(e, payload=TimedLidar(e.capture_ns, np.ones(2, np.float32), -np.pi, np.pi, 0., 25.))
        return e
    for i, row in enumerate(anchors):
        if row["run_id"] != run_id or row["label_index"] != i:
            raise ValueError("anchor identity mismatch")
        anchor = by_seq[row["camera_row_id"]]
        needed = {rid for role in ids for slot in row["history_row_ids"][role] for rid in slot}
        events = tuple(sensor_placeholder(e) for e in windows.at(anchor)
                       if e.role not in ids or e.sequence in needed)
        for role in ids:
            for slot, refs in enumerate(row["history_row_ids"][role]):
                if refs:
                    arrays[role + "_refs"][i, slot] = maps[role][refs[0]]
        try:
            batch, _ = assemble_time_inputs(events, sensor_placeholder(anchor), config=small,
                epoch_start_ns=bounds[anchor.epoch][0], epoch_end_ns=bounds[anchor.epoch][1], freeze_ns=row["freeze_ns"])
        except ValueError as exc:
            if row["input_invalid_reason"] != str(exc):
                raise ValueError(f"input audit drift at {run_id}:{i}: {exc}") from exc
            continue
        if row["input_invalid_reason"] is not None:
            raise ValueError("previously invalid input unexpectedly became eligible")
        arrays["input_valid"][i] = True
        for key, attr in (("ego", "ego"), ("ego_mask", "ego_feature_mask"), ("command", "command_history"),
                          ("command_mask", "command_mask"), ("dt", "sensor_dt_sec")):
            arrays[key][i] = getattr(batch, attr)[0].numpy()
        np.testing.assert_array_equal(batch.image_mask[0], arrays["camera_refs"][i] >= 0)
        np.testing.assert_array_equal(batch.lidar_mask[0], arrays["lidar_refs"][i] >= 0)
    eligible = np.flatnonzero(arrays["input_valid"])
    replay = []
    for i in (eligible[0], eligible[len(eligible)//2]) if eligible.size else ():
        row = anchors[i]; anchor = by_seq[row["camera_row_id"]]
        needed = {rid for role in ids for slot in row["history_row_ids"][role] for rid in slot}
        events = tuple(load_event(source / "raw", e) for e in windows.at(anchor)
                       if e.role not in ids or e.sequence in needed)
        real, _ = assemble_time_inputs(events, load_event(source / "raw", anchor), config=config,
            epoch_start_ns=bounds[anchor.epoch][0], epoch_end_ns=bounds[anchor.epoch][1], freeze_ns=row["freeze_ns"])
        cached = cached_inputs(arrays, int(i), camera, lidar, config)
        for field in fields(ModelBatchV3):
            value = getattr(real, field.name)
            if isinstance(value, torch.Tensor):
                torch.testing.assert_close(value, getattr(cached, field.name), rtol=0, atol=0)
        replay.append(row["anchor_id"])
    np.savez(destination / "inputs.npz", **arrays)
    np.savez_compressed(destination / "labels.npz", **labels)
    with (destination / "anchors.jsonl").open("x", encoding="utf-8") as stream:
        for row in anchors:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    del camera, lidar
    return {"run_id": run_id, "anchors": n, "input_valid": int(arrays["input_valid"].sum()),
            "real_input_replay_equal": replay}


def prepare_time_training_cache(corpus_root: Path, cache_root: Path, *,
                                splits: Sequence[str] = ("train", "validation"),
                                config: TimeDatasetConfig | None = None) -> dict[str, Any]:
    """Verify source/label hashes and publish a new cache only when complete."""
    config = config or TimeDatasetConfig()
    corpus_root, cache_root = corpus_root.resolve(), cache_root.resolve()
    if not splits or len(set(splits)) != len(splits) or not set(splits) <= {"train", "validation"}:
        raise ValueError("training cache supports train and validation only; test is sealed")
    staging = cache_root.with_name(cache_root.name + ".partial")
    if cache_root.exists() or staging.exists():
        raise FileExistsError("cache or partial cache exists")
    split_manifest = _json(corpus_root / "split_verified.json")
    validate_time_split(split_manifest, require_verified=True)
    if _json(corpus_root / "post_generation_verification.json").get("status") != "PASS":
        raise ValueError("post-generation verification missing")
    contract = _json(corpus_root / "contract.json")
    if contract.get("config") != json.loads(json.dumps(asdict(config))) or contract.get("freeze_delay_receipt_ns") != 50_000_000:
        raise ValueError("dataset config/freeze differs from verified corpus")
    if contract.get("split_sha256") != split_manifest["manifest_sha256"]:
        raise ValueError("corpus split identity mismatch")
    artifacts = _json(corpus_root / "artifact_manifest.json")
    inventory = {row["path"]: row for row in artifacts["files"]}
    runs = [r for r in split_manifest["runs"] if r["split"] in splits]
    for run in runs:
        for name in ("anchors.jsonl", "teachers.npz", "audit.json"):
            relative = f"{run['split']}/{run['run_id']}/{name}"
            record = inventory[relative]
            path = _checked_path(corpus_root, relative)
            if _sha(path) != record["sha256"] or path.stat().st_size != record["bytes"]:
                raise ValueError(f"corpus artifact mismatch: {relative}")
        raw = (corpus_root / run["split"] / run["run_id"] / "raw").resolve(strict=True)
        for source in run["sources"]:
            parts = Path(source["path"]).parts
            if parts[:2] != ("runs", run["run_id"]):
                raise ValueError("unexpected raw source path")
            if _sha(_checked_path(raw, Path(*parts[2:]).as_posix())) != source["sha256"]:
                raise ValueError("raw source content changed")
    staging.mkdir(parents=True, exist_ok=False)
    prepared = []
    for run in runs:
        result = _prepare_run(corpus_root / run["split"] / run["run_id"],
                              staging / run["split"] / run["run_id"], run["run_id"], config)
        result["split"] = run["split"]
        prepared.append(result)
        print(json.dumps({"cache_run_complete": result}), flush=True)
    files = [{"path": p.relative_to(staging).as_posix(), "bytes": p.stat().st_size, "sha256": _sha(p)}
             for p in sorted(staging.rglob("*")) if p.is_file()]
    identity = {"format": FORMAT, "corpus_artifact_manifest_sha256": _sha(corpus_root / "artifact_manifest.json"),
        "contract": contract, "config": asdict(config), "split_manifest": split_manifest,
        "splits": list(splits), "runs": prepared, "cache_files": files}
    identity["manifest_sha256"] = content_sha256(identity)
    _write_json(staging / "identity.json", identity)
    _rename_without_replace(staging, cache_root)
    return identity


class TimeTrainingCacheDataset(Dataset[TimeSample]):
    """Small arrays load once; unique sensors are read-only memory maps per worker."""
    def __init__(self, cache_root: Path, split: str, *, verify_hashes: bool = True) -> None:
        if split not in {"train", "validation"}:
            raise ValueError("test is sealed")
        self.root = cache_root.resolve()
        self.identity = verify_time_training_cache(self.root) if verify_hashes else _json(self.root / "identity.json")
        if self.identity["manifest_sha256"] != content_sha256({k: v for k, v in self.identity.items() if k != "manifest_sha256"}):
            raise ValueError("cache identity mismatch")
        self.split_manifest = self.identity["split_manifest"]
        validate_time_split(self.split_manifest, require_verified=True)
        cfg = dict(self.identity["config"])
        for name in ("image_shape", "lidar_shape"):
            cfg[name] = tuple(cfg[name])
        self.config = TimeDatasetConfig(**cfg)
        self._runs: list[dict[str, Any]] = []
        self._index: list[tuple[int, int]] = []
        self._anchors: list[dict[str, Any]] = []
        for run in self.identity["runs"]:
            if run["split"] != split:
                continue
            path = self.root / split / run["run_id"]
            anchors = _records(path / "anchors.jsonl")
            with np.load(path / "inputs.npz", allow_pickle=False) as z:
                inputs = {k: z[k].copy() for k in z.files}
            with np.load(path / "labels.npz", allow_pickle=False) as z:
                labels = {k: z[k].copy() for k in z.files}
            if len(anchors) != run["anchors"] or len(inputs["input_valid"]) != len(anchors):
                raise ValueError("cache anchor count mismatch")
            number = len(self._runs)
            self._runs.append({"path": path, "inputs": inputs, "labels": labels})
            self._index.extend((number, i) for i in range(len(anchors)))
            self._anchors.extend(anchors)
        if not self._runs:
            raise ValueError("empty cache split")
        self._sensors: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self.run_ids = [a["run_id"] for a in self._anchors]
        self.anchor_ids = [a["anchor_id"] for a in self._anchors]
        assert_split_membership(self.split_manifest, self.run_ids, split=split)
        self.targets = np.concatenate([r["labels"]["xy_m"] for r in self._runs])
        self.xy_mask = np.concatenate([r["labels"]["xy_mask"] for r in self._runs])
        self.input_valid = np.concatenate([r["inputs"]["input_valid"] for r in self._runs])

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, index: int) -> TimeSample:
        rid, i = self._index[index]; run = self._runs[rid]; row = self._anchors[index]
        lab = run["labels"]
        teacher = TimeTeacher(lab["xy_m"][i], lab["xy_mask"][i], lab["velocity_mps"][i],
                              lab["velocity_mask"][i], lab["interval_mask"][i], (), tuple(row["teacher_reasons"]))
        batch = None
        if run["inputs"]["input_valid"][i]:
            if rid not in self._sensors:
                self._sensors[rid] = tuple(np.load(run["path"] / name, mmap_mode="r", allow_pickle=False)
                                           for name in ("camera_rgb.npy", "lidar.npy"))
            batch = cached_inputs(run["inputs"], i, *self._sensors[rid], self.config)
        return TimeSample(batch, teacher, row["run_id"], row["anchor_id"], row["observation_ns"],
            row["input_invalid_reason"], tuple(row["teacher_reasons"]), stop_reason=row["stop_reason"],
            freeze_ns=row["freeze_ns"])
