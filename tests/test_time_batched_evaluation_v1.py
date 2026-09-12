from __future__ import annotations

import hashlib

import numpy as np
import pytest
import torch
from torch.utils.data import Dataset

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_teacher_v1 import TimeTeacher
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.training.train_time_v1 import evaluate_time_samples
from aic_transfuser_lite.data.time_split_v1 import build_time_split


def _manifest() -> dict:
    rows = []
    for speed in (5, 8):
        for i in range(10):
            raw = f"{speed}/{i}".encode()
            rows.append({"run_id": f"{speed}_{i}", "speed_cap_kmh": speed,
                         "sources": [{"path": f"{speed}_{i}.db3",
                                      "sha256": hashlib.sha256(raw).hexdigest()}]})
    out = build_time_split(rows, receipt_sha256="a" * 64)
    out["sources_verified"] = True
    return out


def _sample(run: str, *, invalid: bool = False, teacher: bool = True) -> TimeSample:
    if invalid:
        return TimeSample(None, None, run, "bad", 0, "CURRENT_SENSOR_MISSING",
                          teacher_reasons=("NO_TEACHER",), stop_reason="UNKNOWN")
    shape = (1,)
    batch = ModelBatchV3(
        torch.zeros((*shape, 1, 3, 4, 4)), torch.ones((*shape, 1), dtype=torch.bool),
        torch.ones((*shape, 1, 2, 4)), torch.ones((*shape, 1), dtype=torch.bool),
        torch.tensor([[[1., 0., 0., 0.], [1., 0., 0., 0.]]]),
        torch.ones((*shape, 2, 4), dtype=torch.bool), torch.zeros((*shape, 1, 3)),
        torch.ones((*shape, 1), dtype=torch.bool), torch.zeros((*shape, 1, 2)),
        targets=None, requested_outputs=frozenset({"trajectory"}))
    t = None
    if teacher:
        t = TimeTeacher(np.ones((30, 2), dtype=np.float32), np.ones(30, dtype=bool),
                        np.ones(30, dtype=np.float32), np.ones(30, dtype=bool),
                        np.ones(30, dtype=bool), (), ("OK",) * 30)
    return TimeSample(batch, t, run, "ok", 0, teacher_reasons=(), stop_reason="UNKNOWN")


class _Model(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.anchor = torch.nn.Parameter(torch.tensor(0.5))

    def forward(self, batch: ModelBatchV3) -> torch.Tensor:
        return torch.ones((batch.image.shape[0], 30, 2), device=batch.image.device) * self.anchor


def test_batched_matches_legacy_and_keeps_invalid_anchors() -> None:
    manifest = _manifest()
    run = next(r["run_id"] for r in manifest["runs"] if r["split"] == "validation")
    samples = [_sample(run), _sample(run, teacher=False), _sample(run, invalid=True)]
    model = _Model()
    expected = evaluate_time_samples(model, samples, split_manifest=manifest)
    actual, predictions = evaluate_time_batched(model, samples, run_ids=[s.run for s in samples],
                                                 split_manifest=manifest, batch_size=2)
    assert actual == expected
    assert predictions.shape == (3, 30, 2)
    assert torch.isfinite(predictions[:2]).all() and torch.isnan(predictions[2]).all()
    assert actual["anchor_count"] == 3 and actual["input_invalid_count"] == 1


class _CountingDataset(Dataset):
    def __init__(self, rows):
        self.rows = rows
        self.calls = 0

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        self.calls += 1
        return self.rows[index]


def test_dataset_is_consumed_once_and_run_ids_are_checked() -> None:
    manifest = _manifest()
    run = next(r["run_id"] for r in manifest["runs"] if r["split"] == "validation")
    dataset = _CountingDataset([_sample(run), _sample(run)])
    evaluate_time_batched(_Model(), dataset, run_ids=[run, run], split_manifest=manifest, batch_size=2)
    assert dataset.calls == 2
    other = next(r["run_id"] for r in manifest["runs"]
                 if r["split"] == "validation" and r["run_id"] != run)
    with pytest.raises(ValueError, match="run_ids"):
        evaluate_time_batched(_Model(), [_sample(run)], run_ids=[other],
                              split_manifest=manifest, batch_size=1)
