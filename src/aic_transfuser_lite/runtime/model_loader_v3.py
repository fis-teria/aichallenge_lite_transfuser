from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any

import torch

from aic_transfuser_lite.contracts.behavior_v1 import BEHAVIOR_ONTOLOGY_V1

from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3


ARTIFACT_MANIFEST_FORMAT_V3 = "aic_runtime_artifact_v3"


@dataclass(frozen=True)
class LoadedRuntimeModelV3:
    model: torch.nn.Module
    checkpoint_sha256: str
    manifest_sha256: str
    contract_hash: str
    capabilities: frozenset[str]


def sha256_file_v3(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_sha256(value: str, name: str) -> str:
    normalized = str(value).strip().lower()
    if len(normalized) != 64 or any(char not in "0123456789abcdef" for char in normalized):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return normalized


def load_runtime_model_v3(
    checkpoint_path: str | Path,
    manifest_path: str | Path,
    *,
    device: torch.device,
    expected_checkpoint_sha256: str,
    expected_manifest_sha256: str,
    expected_contract_hash: str,
) -> LoadedRuntimeModelV3:
    checkpoint_path = Path(checkpoint_path)
    manifest_path = Path(manifest_path)
    expected_checkpoint = _require_sha256(expected_checkpoint_sha256, "expected_checkpoint_sha256")
    expected_manifest = _require_sha256(expected_manifest_sha256, "expected_manifest_sha256")
    contract_hash = _require_sha256(expected_contract_hash, "expected_contract_hash")
    actual_checkpoint = sha256_file_v3(checkpoint_path)
    actual_manifest = sha256_file_v3(manifest_path)
    if actual_checkpoint != expected_checkpoint:
        raise ValueError("checkpoint artifact SHA-256 mismatch")
    if actual_manifest != expected_manifest:
        raise ValueError("runtime manifest SHA-256 mismatch")
    manifest: dict[str, Any] = json.loads(manifest_path.read_text(encoding="utf-8"))
    required = {"format", "checkpoint_sha256", "contract_hash", "capabilities", "model_kwargs"}
    if set(manifest) != required or manifest.get("format") != ARTIFACT_MANIFEST_FORMAT_V3:
        raise ValueError("runtime artifact manifest contract mismatch")
    if manifest["checkpoint_sha256"] != actual_checkpoint:
        raise ValueError("manifest checkpoint hash mismatch")
    if manifest["contract_hash"] != contract_hash:
        raise ValueError("runtime contract hash mismatch")
    capabilities = manifest["capabilities"]
    if not isinstance(capabilities, list) or any(not isinstance(item, str) for item in capabilities):
        raise ValueError("runtime capabilities must be a string list")
    if not {"trajectory", "speed_profile"}.issubset(capabilities):
        raise ValueError("runtime artifact lacks trajectory capability")
    if ("behavior" in capabilities) != ("behavior_side" in capabilities):
        raise ValueError("behavior and behavior_side capabilities must be declared together")
    payload = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    identity = payload.get("identity")
    if not isinstance(identity, dict) or identity.get("contract_hash") != contract_hash:
        raise ValueError("checkpoint embedded contract hash mismatch")
    if not isinstance(manifest["model_kwargs"], dict):
        raise ValueError("model_kwargs must be a mapping")
    if "behavior" in capabilities and payload.get("behavior_ontology") != BEHAVIOR_ONTOLOGY_V1:
        raise ValueError("behavior-capable checkpoint ontology mismatch")
    model = FullControlLiteV3(**manifest["model_kwargs"]).to(device)
    if "behavior" in capabilities and model.behavior_head is None:
        raise ValueError("behavior capability requires an enabled behavior head")
    if "current_control" in capabilities and model.control_head is None:
        raise ValueError("current_control capability requires an enabled control head")
    if "control_sequence" in capabilities and model.control_sequence_head is None:
        raise ValueError("control_sequence capability requires an enabled sequence head")
    model.load_state_dict(payload["model"], strict=True)
    model.eval()
    return LoadedRuntimeModelV3(
        model=model,
        checkpoint_sha256=actual_checkpoint,
        manifest_sha256=actual_manifest,
        contract_hash=contract_hash,
        capabilities=frozenset(capabilities),
    )
