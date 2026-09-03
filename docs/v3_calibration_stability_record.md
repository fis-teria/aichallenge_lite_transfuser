# V3 calibration cross-run stability record

The candidate was fitted from 18 complete Dataset V3 runs: three dedicated
drive runs, six moving steering runs, and nine brake runs. Stationary steering
runs and the earlier inconsistent brake cohort were excluded before fitting;
they are not silently averaged into the selected candidate.

The selected Dataset V3 manifest SHA-256 is
`462015e2b7148d87e6c5248b829309512d640d584eaff92ea5727e050baf07f4`.
The candidate calibration artifact SHA-256 is
`838ff71c4004a1ea4f70e5dd3225bd8feb86431f8c2cbffa846d8f9d3f0496b2`.
It remains in `candidate` state.

Cross-run cohorts were fitted independently and checked with
`configs/calibration/v3_cross_run_stability_gate.yaml`:

| Mode | Cohorts | Delay / lag (s) | Gain | Correlation | NRMSE | Samples | Result |
|---|---:|---|---|---|---|---|---|
| steering | 2 x 3 runs | 0.00 / 0.50 both | 1.0 both | 0.98284 / 0.98398 | 0.66310 / 0.65118; yaw 0.78712 / 0.77610 | 274 / 266 dynamic | pass |
| drive | 3 x 1 run | 0.00 / 0.04 all | 0.755875 / 0.749084 / 0.752403 | 0.92919 / 0.91801 / 0.92033 | 0.36961 / 0.39657 / 0.39114 | 146 / 152 / 152 | pass |
| brake | 3 x 3 runs | 0.00 all / 0.12 / 0.14 / 0.14 | 0.723790 / 0.764529 / 0.639431 | 0.884027 / 0.886511 / 0.826674 | 0.467436 / 0.462708 / 0.562681 | 56 / 58 / 55 | pass |

The rejected older brake cohort fitted delay 0.01 s, lag 0.10 s, gain
0.449154, bias -0.587564 m/s2, correlation 0.76071, and NRMSE 0.64909. It
fails the declared correlation gate and is retained as negative evidence.

Passing these offline gates means the selected runs are internally stable
enough for shadow evaluation. It does not promote the artifact and does not
prove closed-loop operation. Promotion still requires shadow evidence and an
explicit limited-ODD closed-loop report. ROS 2 and AWSIM results are recorded
only when actually executed.
