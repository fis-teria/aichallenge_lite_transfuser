from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import torch
from torch import nn


@dataclass(frozen=True)
class HistorySelection:
    indices: tuple[int, ...]
    mask: tuple[bool, ...]


def select_epoch_history(
    clock_epochs: Sequence[object], *, anchor_index: int, length: int
) -> HistorySelection:
    """Select left-padded history without ever crossing a clock epoch boundary."""
    if length <= 0 or anchor_index < 0 or anchor_index >= len(clock_epochs):
        raise ValueError("invalid history selection arguments")
    epoch = clock_epochs[anchor_index]
    valid: list[int] = []
    cursor = anchor_index
    while cursor >= 0 and len(valid) < length and clock_epochs[cursor] == epoch:
        valid.append(cursor)
        cursor -= 1
    valid.reverse()
    pad = length - len(valid)
    indices = [valid[0]] * pad + valid
    return HistorySelection(tuple(indices), tuple([False] * pad + [True] * len(valid)))


def select_epoch_history_before_anchor(
    clock_epochs: Sequence[object], *, anchor_index: int, length: int
) -> HistorySelection:
    """Select causal history strictly before an anchor without crossing epochs.

    At an epoch start there is no valid command history. The anchor index is
    then used only as a safe padding index and every mask entry is false, so
    the anchor value cannot reach the temporal encoder.
    """

    if length <= 0 or anchor_index < 0 or anchor_index >= len(clock_epochs):
        raise ValueError("invalid history selection arguments")
    epoch = clock_epochs[anchor_index]
    valid: list[int] = []
    cursor = anchor_index - 1
    while cursor >= 0 and len(valid) < length and clock_epochs[cursor] == epoch:
        valid.append(cursor)
        cursor -= 1
    valid.reverse()
    if not valid:
        return HistorySelection(
            tuple([anchor_index] * length), tuple([False] * length)
        )
    pad = length - len(valid)
    indices = [valid[0]] * pad + valid
    return HistorySelection(tuple(indices), tuple([False] * pad + [True] * len(valid)))


class MaskedGRUTemporalEncoder(nn.Module):
    """GRUCell history encoder; invalid steps do not change hidden state."""

    def __init__(self, input_dim: int, hidden_dim: int, *, allow_empty: bool = False) -> None:
        super().__init__()
        if input_dim <= 0 or hidden_dim <= 0:
            raise ValueError("temporal encoder dimensions must be positive")
        self.hidden_dim = hidden_dim
        self.allow_empty = allow_empty
        self.cell = nn.GRUCell(input_dim, hidden_dim)

    def forward(self, values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        """Encode values ``[B,T,D]`` with boolean validity ``[B,T]``."""
        if values.ndim != 3 or mask.shape != values.shape[:2] or mask.dtype != torch.bool:
            raise ValueError("temporal values/mask must be [B,T,D]/bool [B,T]")
        if not torch.isfinite(values).all():
            raise ValueError("temporal input contains non-finite values")
        if not self.allow_empty and not bool(mask.any(dim=1).all()):
            raise ValueError("each temporal sequence needs at least one valid step")
        hidden = values.new_zeros((values.shape[0], self.hidden_dim))
        for step in range(values.shape[1]):
            candidate = self.cell(values[:, step], hidden)
            hidden = torch.where(mask[:, step].unsqueeze(1), candidate, hidden)
        return hidden
