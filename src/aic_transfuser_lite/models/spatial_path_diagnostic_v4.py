"""Offline diagnostic-only adapter; no ModelOutputV3/runtime artifact interface."""
from __future__ import annotations

from dataclasses import replace
import hashlib
from typing import Mapping

import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .full_control_lite_v3 import FullControlLiteV3

REPRESENTATION_PREFIXES = ("camera.", "lidar.", "ego.", "camera_temporal.", "lidar_temporal.",
                           "ego_history.", "temporal_projection.", "fusion.")


def tensor_hash(tensor: torch.Tensor) -> str:
    return hashlib.sha256(tensor.detach().cpu().contiguous().numpy().tobytes()).hexdigest()


class SpatialPathDiagnosticV4(nn.Module):
    def __init__(self, **kwargs: object):
        super().__init__()
        kwargs = {**kwargs, "control_head_enabled": False, "control_sequence_head_enabled": False, "behavior_head_enabled": False}
        self.backbone = FullControlLiteV3(**kwargs)
        # Exclude old heads entirely from state_dict, graph and optimizer.
        self.backbone.trajectory_head = None
        self.backbone.speed_profile_head = None
        hidden = int(kwargs.get("hidden_dim", 128))
        self.path_head = nn.Sequential(nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, 40))

    def forward(self, batch: ModelBatchV3) -> torch.Tensor:
        """Inputs only: output [B,20,2] in metres; no teacher validity prediction."""
        inputs = replace(batch, targets=None, requested_outputs=frozenset({"trajectory"}))
        features = self.backbone.forward_features(inputs)
        result = self.path_head(features).reshape(batch.batch_size, 20, 2)
        assert result.shape == (batch.batch_size, 20, 2)
        return result

    def initialize_representation(self, state: Mapping[str, torch.Tensor] | None = None) -> dict:
        current = self.backbone.state_dict()
        patch, entries = {}, []
        for name, value in (state or {}).items():
            allowed = name.startswith(REPRESENTATION_PREFIXES) and name in current and current[name].shape == value.shape
            if allowed:
                if not torch.isfinite(value).all():
                    raise ValueError("nonfinite checkpoint")
                patch[name] = value
            entries.append({"name": name, "shape": list(value.shape), "numel": value.numel(),
                            "sha256": tensor_hash(value), "action": "loaded" if allowed else "skipped"})
        self.backbone.load_state_dict(patch, strict=False)
        return {"mode": "LOCAL_V3_REPRESENTATION" if state is not None else "SCRATCH",
            "reason": "allowlist only" if state is not None else "No explicit trusted checkpoint path supplied for this diagnostic; no discovery/download",
            "loaded": sorted(patch), "skipped": [e["name"] for e in entries if e["action"] == "skipped"],
            "missing": sorted(set(current)-set(patch)), "unexpected": sorted(set(state or {})-set(current)),
            "source_entries": entries, "path_head": "NEW_RANDOM_NOT_TIME_HEAD",
            "trainable": [{"name": n, "shape": list(p.shape), "numel": p.numel(), "sha256": tensor_hash(p)}
                          for n, p in self.named_parameters() if p.requires_grad]}
