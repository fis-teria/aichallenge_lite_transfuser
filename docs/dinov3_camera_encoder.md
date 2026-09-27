# Frozen DINOv3 ViT-S/16 camera adapter

The Issue #4 adapter uses the official
[`facebookresearch/dinov3`](https://github.com/facebookresearch/dinov3)
implementation and its local Torch Hub entry `dinov3_vits16`. Production use
requires an explicit local repository path, local checkpoint path, and full
lowercase SHA-256. The adapter never requests a network download.

The initial configuration is `configs/models/dinov3_vits16_frozen.yaml`. Set
the two null paths outside Git for the target machine. The registered official
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

Focused verification:

```powershell
.\tmp\slam_mppi_test_env\Scripts\python.exe -m pytest -q tests/test_dinov3_camera_encoder.py tests/test_time_backbone_p0.py tests/test_model_v1_shapes.py
```

The tests use an injected ViT-S/16-compatible fake backbone so they do not need
or download licensed weights. A real-weight forward/latency test remains
required on the intended Linux/CUDA and RTX 5060 environments.
