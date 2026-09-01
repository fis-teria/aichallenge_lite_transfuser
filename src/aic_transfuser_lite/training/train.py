from __future__ import annotations

import argparse
import hashlib
import json
import random
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader, RandomSampler

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.data.dataset import DrivingDataset
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.training.losses import compute_multitask_loss


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--train-index", required=True)
    parser.add_argument("--val-index", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--epochs", type=int, default=None)
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def seed_worker(worker_id: int) -> None:
    del worker_id
    worker_seed = torch.initial_seed() % (2**32)
    random.seed(worker_seed)
    np.random.seed(worker_seed)


def seeded_random_sampler(
    dataset: Any, seed: int
) -> tuple[RandomSampler, list[int]]:
    """Return a sampler and a non-consuming preview of its first epoch order."""

    generator = torch.Generator().manual_seed(seed)
    sampler = RandomSampler(dataset, generator=generator)
    state = generator.get_state()
    first_epoch_order = [int(index) for index in sampler]
    generator.set_state(state)
    return sampler, first_epoch_order


def order_sha256(order: list[int]) -> str:
    values = np.asarray(order, dtype=np.int64)
    return hashlib.sha256(values.tobytes()).hexdigest()


def move_batch(batch: dict[str, torch.Tensor], device: torch.device) -> dict[str, torch.Tensor]:
    return {key: value.to(device) for key, value in batch.items()}


def forward_model(model: torch.nn.Module, batch: dict[str, torch.Tensor], name: str):
    if name == "lidar_only":
        return model(batch["lidar"], batch["ego"])
    return model(batch["image"], batch["lidar"], batch["ego"])


def run_epoch(
    model: torch.nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer | None,
    device: torch.device,
    model_name: str,
    loss_weights: dict[str, Any],
) -> float:
    training = optimizer is not None
    model.train(training)
    total = 0.0
    count = 0
    for batch in loader:
        batch = move_batch(batch, device)
        with torch.set_grad_enabled(training):
            outputs = forward_model(model, batch, model_name)
            loss, _ = compute_multitask_loss(outputs, batch, loss_weights)
        if training:
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
        total += float(loss.detach().cpu()) * batch["ego"].shape[0]
        count += batch["ego"].shape[0]
    return total / max(count, 1)


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    seed = int(config.get("project", {}).get("seed", 42))
    set_seed(seed)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    train_data = DrivingDataset(args.train_index, config)
    val_data = DrivingDataset(args.val_index, config)
    train_cfg = config.get("training", {})
    batch_size = int(train_cfg.get("batch_size", 32))
    num_workers = int(train_cfg.get("num_workers", 0))
    data_order_seed = int(train_cfg.get("data_order_seed", seed))
    train_sampler, first_epoch_order = seeded_random_sampler(train_data, data_order_seed)
    train_loader = DataLoader(
        train_data,
        batch_size=batch_size,
        sampler=train_sampler,
        num_workers=num_workers,
        worker_init_fn=seed_worker,
        generator=torch.Generator().manual_seed(data_order_seed + 1),
    )
    val_loader = DataLoader(
        val_data,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        worker_init_fn=seed_worker,
        generator=torch.Generator().manual_seed(data_order_seed + 2),
    )
    (output_dir / "data_order.json").write_text(
        json.dumps(
            {
                "data_order_seed": data_order_seed,
                "first_epoch_order_sha256": order_sha256(first_epoch_order),
                "first_batch_indices": first_epoch_order[:batch_size],
            },
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = build_model(config).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(train_cfg.get("learning_rate", 3e-4)),
        weight_decay=float(train_cfg.get("weight_decay", 1e-2)),
    )
    epochs = int(args.epochs or train_cfg.get("epochs", 30))
    model_name = str(config["model"]["name"])
    history = []
    best = float("inf")

    for epoch in range(1, epochs + 1):
        train_loss = run_epoch(
            model, train_loader, optimizer, device, model_name, config["loss_weights"]
        )
        val_loss = run_epoch(
            model, val_loader, None, device, model_name, config["loss_weights"]
        )
        record = {"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss}
        history.append(record)
        print(json.dumps(record))
        if val_loss < best:
            best = val_loss
            torch.save(
                {"model": model.state_dict(), "config": config, "epoch": epoch},
                output_dir / "best.pt",
            )

    (output_dir / "history.json").write_text(
        json.dumps(history, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
