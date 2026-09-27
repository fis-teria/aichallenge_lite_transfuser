# Frozen DINOv3 ViT-S/16 camera adapter

The Issue #4 adapter uses the official
[`facebookresearch/dinov3`](https://github.com/facebookresearch/dinov3)
implementation and its local Torch Hub entry `dinov3_vits16`. Production use
requires an explicit local repository path, local checkpoint path, and full
lowercase SHA-256. The adapter never requests a network download.

The adapter profile is `configs/models/dinov3_vits16_frozen.yaml`; it also
documents preprocessing and is not a complete training configuration. The
complete TimePath model template is
`configs/time_path_p1/command_off_dinov3.json`. Copy it outside tracked paths
and set the two null paths for the target machine. The registered official
LVD-1689M ViT-S/16 checkpoint identity is
`08c60483bc63c04f533611e34bf70b120eedb7240f469bc16e9e20bf344b941d`;
the loader hashes the actual bytes before construction.

Input is ImageNet-normalized RGB `[B,3,224,384]`. Both dimensions are divisible
by patch size 16. The adapter performs no crop: it reshapes the 14x24 patch grid,
spatially pools it to the configured 4x4 grid, and projects 384 channels to the
existing 128-dimensional fusion contract. History masking stays in the existing
`encode_valid` path, so missing frames never enter DINOv3.

The backbone is always frozen and held in eval mode. Only the projection and
downstream temporal/fusion/head modules are trainable. Unfreezing and LoRA are
outside the initial comparison.

## Model selection and checkpoint compatibility

`TimeModelConfig.from_dict(...)` / `build_time_model(...)` now forward the
optional `camera_encoder` dictionary through TimePathV1 and TimeBackboneV1
to FullControlLiteV3. The V3 training builder also accepts this dictionary at
`model.camera_encoder`. Supported fields are `backbone`, `frozen`,
`repository_path`, `checkpoint_path`, and `checkpoint_sha256`; optional
`output_dim`, `token_h`, and `token_w` must match the shared model dimensions.
Do not copy the profile's descriptive `input` or `token_pooling` fields into
this constructor dictionary. Unknown fields and mismatched shapes fail early.

Omitting `camera_encoder` preserves the legacy ResNet18 constructor, parameter
names, serialized TimeModelConfig and TimePath extra-state identity. DINO
selection and weight identity are recorded in the checkpoint configuration;
saving a DINO model under a ResNet config is rejected. The current loader
requires the configured local DINO source and pretrained checkpoint even
when restoring a trained state dictionary. Portable deployment packaging and
latency measurements remain Issue #6 work.

## Local storage and construction

This Git repository contains only the adapter, configurations and tests. It
does not contain the official DINOv3 source checkout or pretrained weights.
As of the 2026-09-27 inspection, both configured paths were null and no DINOv3
weights were found in the checked project/model/cache directories on Windows,
WSL or `graneple@192.168.3.10`. This was not a full-disk inventory.

For future placement, use Linux-native storage, for example
`/home/thistle/e2e_autonomous/third_party/dinov3` for the official source and
`/home/thistle/e2e_autonomous/e2e_lite_transfuser/weights/dinov3/` for weights.
These are suggested destinations, not installed assets. Do not put weights
in Git or train from `/mnt/e`. After acquiring the official source/weights
and filling a local config, construction can be checked under the WSL lock:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh .venv/bin/python - <<'PY'
import json
from pathlib import Path
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

config = TimeModelConfig.from_dict(json.loads(Path("tmp/dinov3_local.json").read_text()))
model = build_time_model(config).eval()
print(model.backbone.camera.pretrained_provenance())
PY
```

This constructs and verifies local weight bytes; it is not a forward, training,
CUDA latency or closed-loop driving evaluation.

Focused verification:

```powershell
.\tmp\slam_mppi_test_env\Scripts\python.exe -m pytest -q tests/test_dinov3_camera_encoder.py tests/test_dinov3_model_integration.py tests/test_time_backbone_p0.py tests/test_time_checkpoint_p1.py
```

The tests inject a ViT-S/16-compatible fake backbone, including at the local
Torch Hub boundary when constructing the full model. They verify history
masking, finite forward/backward, a frozen backbone and trainable projection /
fusion / XY head, optimizer updates, checkpoint roundtrip, legacy identity,
and missing-weight/config errors. They do not need or download licensed
weights. A real-weight forward/latency test remains required on the intended
Linux/CUDA and RTX 5060 environments.
