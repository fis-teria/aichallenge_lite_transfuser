# AWSIM Camera 10 Hz compatibility override

## Why this override exists

The supported AI Challenge AWSIM binary publishes Camera header timestamps at
`9.5238097 Hz`: every interval is `105 ms` although the scene config requests
`10 Hz`. The binary uses a `5 ms` Unity fixed timestep and its
`CameraSensorHolder.FixedUpdateRoutine` has an extra trailing
`WaitForFixedUpdate` after the nominal `100 ms` wait.

AWSIM upstream fixed this same defect in commit
[`4dc00f7` (`fix(camera): improve camera topic hz`)](https://github.com/autowarefoundation/AWSIM/commit/4dc00f768b84e1ebaf8d8b6372f5065e0a26622f)
by deleting that trailing yield. The AI Challenge checkout contains only the
prebuilt player, so `tools/prepare_awsim_camera_hz_override.py` creates a new,
hash-verified assembly copy with the equivalent IL change. It does not modify
the installed AWSIM assembly.

Do not replace this fix with Camera-frame duplication, converter-side rate
inflation, a relaxed Dataset gate, or an interval epsilon. Those approaches
hide the real publisher rate or encode a timestep-specific magic number.

## Supported build and fail-closed behavior

- accepted input SHA-256:
  `f8553d26dadc1316143ee22f6d9a75803753ff82433876da78f48f808d49ba28`;
- expected output SHA-256:
  `129192b7ab8783d7092bca92ae8c220b95055d350468c3cd2c513d701b19cb95`;
- patched file offset: `0x1178E`;
- semantic change: remove the compiler-generated trailing fixed-update yield.

The tool refuses an unknown input hash, unexpected bytes at the audited
offset, in-place modification, a missing destination directory, or any existing
output/manifest. Both generated files are made read-only. A future AWSIM build
must be audited from source instead of adding another offset without evidence.

## Prepare the override on the AWSIM host

Use a new run-specific directory. The example leaves the original DLL
untouched and records a deterministic JSON manifest beside the override.

```bash
cd /path/to/aichallenge_lite_transfuser
override_dir=$(mktemp -d /tmp/awsim-camera-hz.XXXXXX)
PYTHONPATH=src python3 tools/prepare_awsim_camera_hz_override.py \
  --input /path/to/aichallenge-racingkart/aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll \
  --output "$override_dir/Assembly-CSharp.dll" \
  --manifest "$override_dir/manifest.json"
```

Before starting AWSIM, separately audit for another GA/AWSIM run. Do not stop
or reuse its containers. Mount the generated copy read-only over the assembly
path in the simulator container:

```bash
--volume "$override_dir/Assembly-CSharp.dll:/aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll:ro"
```

The complete launch still needs a dedicated compose project, the intended ROS
domain mapping (base `100`, vehicle `101`), and the selected race config. This
document deliberately does not hide those choices in an implicit launcher.

## Acceptance checks

The override is usable for Dataset v2 collection only after all checks pass:

1. the container-visible DLL hash is the expected output hash;
2. Camera headers are unique and the effective rate is `9.8` to `10.2 Hz`;
3. Camera-LiDAR p95 skew remains below `30 ms` in a dynamic run;
4. callback errors are zero;
5. the original DLL hash, racingkart HEAD, and pre-existing dirty files remain
   unchanged;
6. the run manifest records the override manifest and exact launch command.

Prefer an official rebuilt AWSIM player containing the upstream fix when one
becomes available. Until then, the hash-locked read-only copy is the supported
compatibility boundary.
