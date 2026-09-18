# TimePath static obstacle trial

The finite AWSIM runner accepts `--static-obstacle-scenario` for native box/cone
YAML in `configs/scenarios`. It copies the fixture to the run output, enables
existing AWSIM collisions, records its SHA-256, and checks named object creation
before official Start. Simulator assets and the normal ego spawn are unchanged.
The scenario is setup/evaluation information and is never a model/control input.

On a prepared PC10 deployment with its matching candidate checkpoint/config:

```bash
timeout --signal=TERM --kill-after=10s 710s \
  python3 tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/DEPLOYMENT \
  --run-id codex-time-obstacle-unique --display :1 --ros-launch \
  --config configs/control/time_native_replay_trial.json \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --record-video \
  --static-obstacle-scenario configs/scenarios/time_avoidance_single_cone.yaml
```

Use a new run ID and `time_avoidance_single_box.yaml` for the box condition.
The fixtures use the centre of the previously exercised s=44 m three-box barrier,
with one object each. Surrounding pavement exists, but dynamic passing clearance
and avoidance success must be measured from the actual trial.

The ordinary finite trial, sensor freshness, output checks, stop confirmation,
and cleanup remain. Static fixtures cannot be combined with built-in NPCs,
background PP cars, or the dedicated SLAM box-braking fixture. A separate SLAM
longitudinal layer is optional; record its use and do not credit its stopping as
learned steering avoidance. The baseline config keeps proximity monitoring in
its existing diagnostic/log-only mode.

Passing requires verified object creation, observed encounter and passage,
no observed contact, official penalties checked, and the completed lap.
A completed lap or zero official penalties alone does not prove avoiding native
objects, which can move on contact. Preserve video, raw predictions, control
records and official results; distinguish avoidance, contact and stopping.

After Windows commit and `tools/sync_to_wsl.ps1` synchronization:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```
