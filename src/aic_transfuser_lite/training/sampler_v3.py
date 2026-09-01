from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class SamplerStateV3:
    epoch: int
    offset: int


class DeterministicSamplerV3:
    """Seeded epoch permutation with a serializable exact cursor."""

    def __init__(self, size: int, *, seed: int, epoch: int = 0, offset: int = 0) -> None:
        if size <= 0 or seed < 0 or epoch < 0 or offset < 0 or offset > size:
            raise ValueError("invalid deterministic sampler parameters")
        self.size = size
        self.seed = seed
        self.epoch = epoch
        self.offset = offset

    def next_index(self) -> int:
        if self.offset == self.size:
            self.epoch += 1
            self.offset = 0
        generator = torch.Generator().manual_seed(self.seed + self.epoch)
        permutation = torch.randperm(self.size, generator=generator)
        index = int(permutation[self.offset])
        self.offset += 1
        return index

    def state_dict(self) -> dict[str, int]:
        return {"size": self.size, "seed": self.seed, "epoch": self.epoch, "offset": self.offset}

    def load_state_dict(self, state: dict[str, int]) -> None:
        if int(state["size"]) != self.size or int(state["seed"]) != self.seed:
            raise ValueError("sampler size/seed mismatch")
        epoch = int(state["epoch"])
        offset = int(state["offset"])
        if epoch < 0 or offset < 0 or offset > self.size:
            raise ValueError("invalid sampler checkpoint cursor")
        self.epoch = epoch
        self.offset = offset
