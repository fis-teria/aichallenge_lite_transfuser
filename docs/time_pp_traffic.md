# E2E ego + 2 existing Pure Pursuit cars

The built-in NPC startup failed before driving. The approved alternative uses
three normal AWSIM vehicles in the same scene, with independent ROS domains:
ego E2E on 1 and existing Pure Pursuit on 2 and 3. AWSIM files are unchanged.

On a source-verified graneple deployment:

```bash
make dev MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=10 TIME_NPCS=0 TIME_PP_VEHICLES=2 TIME_RECORD_VIDEO=1
```

The background reference speed is capped at 10 km/h by a task-local ROS launch
copy. Existing PP initialization, race-arm, stale-input and planner guards remain
enabled. All three domains must be ready for the official Start service.
The normal RViz displays the raw E2E predicted path. Occupancy remains log-only
under the previously authorized trial configuration.

Shared Unity lap messages contain no vehicle identity. This trial therefore
uses the ego domain's 7-element `/awsim/status` array and ordered section/lap
transitions. No observer information is sent to the model. Lap times from this
10 Hz stream are intervals, not the exact Unity lap time. AWSIM's lap limit is
600 so a background car does not finish and lose its collision interactions
during the ego lap; the outer trial remains bounded to 720 wall seconds and
stops when ego completes its first lap.

Validation: run `tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q`
in the native WSL checkout after syncing the committed Windows source.

Implementation is prepared; actual AWSIM trial results will be recorded below.
