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
WSL or the configured SSH host. This was not a full-disk inventory.

All assets now use the **project workspace**, not a sibling directory or a
user-global installation. The same layout applies on each execution host:

| Workspace-relative path | Purpose |
| --- | --- |
| `.venv/` | Existing project Python environment |
| `third_party/dinov3/` | Official source, pinned to `6876159a11b4df116f30f667f8c9888617df0751` |
| `weights/dinov3/` | Official ViT-S/16 pretrained checkpoint |
| `.cache/` | PyTorch, Hugging Face, pip and GPU/compiler caches |
| `runs/setup/dinov3/model_config.json` | Generated host-local absolute paths |
| `runs/setup/dinov3/setup.json` | Installation status and source/weight identity |

The training workspace is
`/home/thistle/e2e_autonomous/e2e_lite_transfuser`. Windows remains the code/Git
source of truth; do not train from `/mnt/e`. Source dependencies, weights,
environment and generated outputs are excluded from this project's Git.

Install the source and prepare the configuration using the existing WSL venv:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  .venv/bin/python tools/setup_dinov3_workspace.py
```

The setup script checks the pinned official repository, creates workspace
directories and writes a local config. It preserves a differing existing
repository, checkpoint or manually edited config by stopping with an error.
It does not upgrade the Python environment, download weights or accept access
conditions. With no checkpoint its status is `SOURCE_READY_WEIGHTS_MISSING`;
that status does not mean pretrained inference is available.

At the pinned source revision, Torch Hub also imports evaluation modules.
The existing project venv needed the additional dependencies pinned in
`configs/environments/dinov3_hub_py310_requirements.txt`. They were verified
with Python 3.10, torch 2.7.1+cu128 and torchvision 0.22.1+cu128. Install them
with the existing package versions constrained, so the resolver cannot
silently replace the working CUDA stack:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  .venv/bin/python -m pip freeze --exclude-editable \
  > runs/setup/dinov3/environment-constraints.txt
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  .venv/bin/python -m pip install \
  -c runs/setup/dinov3/environment-constraints.txt \
  -r configs/environments/dinov3_hub_py310_requirements.txt
```

On 2026-09-27, the official source was installed in the WSL workspace and 13
missing packages were added to its `.venv`; all 56 previously installed package
versions were preserved. A **random-initialized official backbone** passed a
CUDA camera-adapter forward/backward check on RTX 4080: input `[1,3,224,384]`,
output `[1,16,128]`, frozen backbone and finite nonzero projection gradient.
This checks source/environment compatibility, not pretrained-weight loading
or driving quality. The uncredentialed official weight endpoint returned
HTTP 403; pretrained weights remain absent pending the user's access request.

Acquire `dinov3_vits16_pretrain_lvd1689m-08c60483.pth` through the
[official Meta access form](https://ai.meta.com/resources/models-and-libraries/dinov3-downloads/).
The reference implementation expects the original `.pth` state dictionary;
the Hugging Face Transformers `model.safetensors` is not a drop-in replacement.
After downloading the approved file, import it (replace the example source path):

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  .venv/bin/python tools/setup_dinov3_workspace.py \
  --checkpoint /path/to/dinov3_vits16_pretrain_lvd1689m-08c60483.pth
```

The importer checks the full SHA-256 before publishing the file in
`weights/dinov3/`; it never replaces an existing different weight file. Download
URLs containing access tokens must not be committed or posted to Issues.
Once the status is `READY_FOR_PRETRAINED_SMOKE`, construction can be checked:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  env PYTHONPATH=src .venv/bin/python - <<'PY'
import json
from pathlib import Path
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

config = TimeModelConfig.from_dict(json.loads(Path("runs/setup/dinov3/model_config.json").read_text()))
model = build_time_model(config).eval()
print(model.backbone.camera.pretrained_provenance())
PY
```

This constructs and verifies local weight bytes; it is not a forward, training,
CUDA latency or closed-loop driving evaluation.

Focused verification:

```powershell
.\tmp\slam_mppi_test_env\Scripts\python.exe -m pytest -q tests/test_dinov3_workspace_setup.py tests/test_dinov3_camera_encoder.py tests/test_dinov3_model_integration.py tests/test_time_backbone_p0.py tests/test_time_checkpoint_p1.py
```

The tests inject a ViT-S/16-compatible fake backbone, including at the local
Torch Hub boundary when constructing the full model. They verify history
masking, finite forward/backward, a frozen backbone and trainable projection /
fusion / XY head, optimizer updates, checkpoint roundtrip, legacy identity,
and missing-weight/config errors. They do not need or download licensed
weights. A real-weight forward/latency test remains required on the intended
Linux/CUDA and RTX 5060 environments.
