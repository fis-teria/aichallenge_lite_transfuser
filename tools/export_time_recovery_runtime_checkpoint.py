"""Complete missing runtime metadata from the hash-bound original training cache.

Trusted local checkpoints only. Model tensors and training state are preserved;
the original file is never replaced, and the runtime contract gate is unchanged.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

import torch

from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import _sha, verify_time_training_cache
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TimeCheckpointIdentity, inspect_time_checkpoint, load_time_checkpoint,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model


def export_runtime_checkpoint(source: Path, *, expected_sha256: str,
                              cache: dict[str, Any], output: Path) -> dict[str, Any]:
    """Export 30 x 2 / 0.1 s metadata; caller verifies the cache inventory first."""
    if source.resolve() == output.resolve() or output.exists():
        raise ValueError('FRESH_EXPORT_PATH_REQUIRED')
    if _sha(source) != expected_sha256:
        raise ValueError('SOURCE_CHECKPOINT_HASH_MISMATCH')
    payload = torch.load(source, map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    identity = TimeCheckpointIdentity(**payload['identity'])
    inspect_time_checkpoint(source, config=config, identity=identity)
    teacher = payload['teacher_manifest']
    supported = {'recovery_sampling_geometry_comparison_v1', 'observed_multiscale_recovery_training_v1'}
    if teacher.get('format') not in supported or 'contract' in teacher:
        raise ValueError('MISSING_COMPARISON_CONTRACT_REQUIRED')
    digest = content_sha256({k: v for k, v in cache.items() if k != 'manifest_sha256'})
    if (cache.get('format') != 'time_training_cache_v1' or cache.get('manifest_sha256') != digest
            or teacher.get('cache_sha256') != digest
            or cache['split_manifest'] != payload['split_manifest']):
        raise ValueError('ORIGINAL_TRAINING_CACHE_REQUIRED')
    contract = cache['contract']
    expected_config = json.loads(json.dumps(asdict(config.dataset_config())))
    if (config.use_command_history or cache['config'] != expected_config
            or contract.get('config') != expected_config
            or contract.get('freeze_delay_receipt_ns') != 50_000_000
            or contract.get('frame') != 'base_link_at_observation'
            or contract.get('points') != 30 or contract.get('dt_s') != .1):
        raise ValueError('CACHE_RUNTIME_CONTRACT_MISMATCH')
    original = build_time_model(config)
    load_time_checkpoint(source, config=config, identity=identity, model=original, mode='finetune')
    teacher = deepcopy(teacher)
    teacher.pop('manifest_sha256')
    teacher.update(contract=deepcopy(contract), runtime_export_source_checkpoint_sha256=expected_sha256)
    teacher['manifest_sha256'] = content_sha256(teacher)
    completed = dict(payload, teacher_manifest=teacher,
                     identity=dict(payload['identity'], teacher_manifest_sha256=teacher['manifest_sha256']))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        torch.save(completed, stream)
    exported_sha = _sha(output)
    runtime = TimeRuntimeModel(output, expected_sha256=exported_sha, device='cpu')
    left, right = original.state_dict(), runtime.model.state_dict()
    if left.keys() != right.keys():
        raise ValueError('EXPORT_STATE_KEYS_CHANGED')
    for name, value in left.items():
        if isinstance(value, torch.Tensor):
            if not torch.equal(value, right[name]):
                raise ValueError('EXPORT_TENSOR_CHANGED:' + name)
        elif value != right[name]:
            raise ValueError('EXPORT_EXTRA_STATE_CHANGED:' + name)
    if _sha(source) != expected_sha256:
        raise ValueError('SOURCE_CHANGED_DURING_EXPORT')
    return dict(status='PASS', source=str(source), source_sha256=expected_sha256, output=str(output),
                output_sha256=exported_sha, cache_manifest_sha256=digest, epoch=runtime.epoch,
                global_step=payload['global_step'], state_entries_equal=len(left),
                changed_payload_keys=['teacher_manifest', 'identity'], runtime_gate_unchanged=True,
                model_weights_changed=False, retraining=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cache = verify_time_training_cache(args.cache)
    result = export_runtime_checkpoint(args.checkpoint, expected_sha256=args.checkpoint_sha256,
                                       cache=cache, output=args.output)
    with args.output.with_suffix('.export.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
