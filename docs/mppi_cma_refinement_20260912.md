# MPPI CMA-ES refinement after four-car same-start validation

Continue from the two previously validated best references. Each condition uses
three generations of eight 16-anchor CMA candidates, initialized at its previous
best anchors with sigma 0.16 m. The normal target remains 10.0 m/s; the handicap
target remains 7.5 m/s. Completion/contact checks, OT avoidance, and then flying
lap time retain their original priority. The current 0.6 m/s² corner acceleration
allowance is retained.

Normal candidates run four at a time in one AWSIM, from the same native D1 pose.
One private network namespace contains the simulator/observer and four separate
controller containers. Each controller mounts its own reference CSV read-only,
uses its own ROS domain/vehicle ID, and receives only the empty ghost V2X stream.
Before start, each loaded reference is matched to that vehicle's requested first
point, all four GNSS poses must match the native D1 start within 0.05 m, and both
planner and recovery subscriptions are checked. Four different input hashes and
controller mount commands remain in each episode's evidence.

Rank-1 handicap candidates use four independent single-car AWSIM instances.
Their rank must remain 1 throughout the measured race. Native handicap logic is
unchanged. Each run is two laps, and the second lap is the timing metric.

The finite budget is 64 vehicle evaluations: 24 candidates per condition plus
four replays of the new candidate and four of the incumbent per condition. No
new evaluation starts after 90 minutes. Normal comparisons swap candidate and
incumbent across vehicle slots in two four-car races. Both routes must pass all
four runs and dense OT checks; the new route also needs at least a 0.05 s median
improvement and three wins in four ordered comparisons. Otherwise retain the
previous route. This is a practical repeatability gate, not a statistical
confidence interval or proof of a global optimum.

`refinement/state.json` and separate optimizer directories preserve the completed
earlier searches. CMA proposals and the pickled RNG/optimizer are checkpointed
together. Repeating ask returns the same pending proposals; repeating tell with
the same completed evaluation is idempotent. Changed completed evaluations or
reordered candidates are rejected. Infrastructure failure drains owned jobs and
records an error; it does not silently retry or spend an expanded budget.

```bash
# On SI26, with the existing frozen study inputs and transferred tools:
python3 /home/si26-pc008/cma_mppi_20260912/tools/refine.py
python3 /home/si26-pc008/cma_mppi_20260912/tools/prepare_viewer.py /home/si26-pc008/cma_mppi_20260912 --shared-course
python3 /home/si26-pc008/cma_mppi_20260912/tools/prepare_viewer.py /home/si26-pc008/cma_mppi_20260912
python3 /home/si26-pc008/cma_mppi_20260912/tools/show_live.py \
  /home/si26-pc008/cma_mppi_20260912 --state-subdir refinement \
  --xauthority /run/user/1000/.mutter-Xwaylandauth.V5NQV3
```

RViz follows the active group or first independent episode. The dashboard shows
all four actual odometry speeds in m/s and km/h, per-car references, generation
results, and the final comparison. Final results, verification and preservation
evidence are appended after the bounded study completes.
