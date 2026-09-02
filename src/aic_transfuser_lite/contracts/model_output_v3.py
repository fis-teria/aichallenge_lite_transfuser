from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class ModelOutputV3:
    trajectory_xy: torch.Tensor
    trajectory_speed_mps: torch.Tensor
    candidate_logits: torch.Tensor
    trajectory_log_sigma: torch.Tensor | None = None
    stop_logit: torch.Tensor | None = None
    risk_logits: torch.Tensor | None = None
    behavior_logits: torch.Tensor | None = None
    behavior_side_logits: torch.Tensor | None = None
    current_control: torch.Tensor | None = None
    control_sequence: torch.Tensor | None = None

    def validate(
        self,
        *,
        batch_size: int,
        candidates: int,
        trajectory_steps: int,
        requested_outputs: frozenset[str],
    ) -> None:
        expected_xy = (batch_size, candidates, trajectory_steps, 2)
        if self.trajectory_xy.shape != expected_xy:
            raise ValueError(f"trajectory_xy must be {expected_xy}")
        if self.trajectory_speed_mps.shape != expected_xy[:-1]:
            raise ValueError("trajectory_speed_mps must be [B,K,N]")
        if self.candidate_logits.shape != (batch_size, candidates):
            raise ValueError("candidate_logits must be [B,K]")
        for name, tensor in (
            ("trajectory_xy", self.trajectory_xy),
            ("trajectory_speed_mps", self.trajectory_speed_mps),
            ("candidate_logits", self.candidate_logits),
        ):
            if not torch.isfinite(tensor).all():
                raise ValueError(f"{name} contains non-finite values")
        if bool((self.trajectory_speed_mps < 0.0).any()):
            raise ValueError("trajectory speed must be non-negative")
        optional = {
            "trajectory_log_sigma": self.trajectory_log_sigma,
            "stop": self.stop_logit,
            "risk": self.risk_logits,
            "behavior": self.behavior_logits,
            "behavior_side": self.behavior_side_logits,
            "current_control": self.current_control,
            "control_sequence": self.control_sequence,
        }
        for name, tensor in optional.items():
            if name in requested_outputs and tensor is None:
                raise ValueError(f"requested output {name!r} is absent")
            if name not in requested_outputs and tensor is not None:
                raise ValueError(f"unrequested output {name!r} must be None")
            if tensor is not None and not torch.isfinite(tensor).all():
                raise ValueError(f"{name} contains non-finite values")
        if self.trajectory_log_sigma is not None:
            if self.trajectory_log_sigma.shape != expected_xy:
                raise ValueError("trajectory_log_sigma must match trajectory_xy")
            if bool((self.trajectory_log_sigma.abs() > 10.0).any()):
                raise ValueError("trajectory_log_sigma exceeds configured safety range")
        if self.current_control is not None and self.current_control.shape != (batch_size, candidates, 3):
            raise ValueError("current_control must be [B,K,3]")
        if self.behavior_logits is not None and self.behavior_logits.shape != (batch_size, 5):
            raise ValueError("behavior_logits must be [B,5]")
        if self.behavior_side_logits is not None and self.behavior_side_logits.shape != (batch_size, 3):
            raise ValueError("behavior_side_logits must be [B,3]")
        if self.control_sequence is not None and (
            self.control_sequence.ndim != 4
            or self.control_sequence.shape[:2] != (batch_size, candidates)
            or self.control_sequence.shape[-1] != 3
        ):
            raise ValueError("control_sequence must be [B,K,H,3]")
