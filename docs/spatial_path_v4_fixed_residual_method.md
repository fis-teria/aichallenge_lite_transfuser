# Saved-array residual and shape diagnostic (no inference)

Input is six explicit files from the previously verified review packet:
train/validation targets and predictions NPZ, metrics JSON, annotation addendum JSON.
Only PACKAGE_MANIFEST inventory and those six files are read. No future/sensor/input
example/checkpoint/Dataset/raw read, model import, training, inference or correction.
Annotation is a fixed-ID/run/role join, never modifies original metrics or IDs.

Residual sign: prediction minus teacher; dx forward, dy left, meters. Invalid tail
not scored. Per-grid means have one value per supported anchor. Overall MAE/bias/ADE
is anchor-balanced. RMSE = sqrt(mean over anchors of within-anchor mean squared
component error). No pooled-point metric silently substituted.

Two descriptive cohorts (no resampling or re-selection): original complete role
with distance-dependent support, and the original role's all20-mask subset. The
latter keeps the same anchor IDs at every distance. Report counts separately by
role, shape, run, annotation. No 2m tail filled where unsupported.

Tangent proxy: direction of endpoint chord between grid nodes j and j+3 (0.3m
nominal distance), 17 windows from 0.1-0.4 to 1.7-2.0m. All four teacher masks
must be true. No origin window, smoothing, fitting, interpolation or correction.
Angle error wraps to [-pi,pi]; arithmetic signed wrapped mean, circular mean,
MAE/RMSE rad retained. Chord <=1e-12m is numerically undefined, not angle zero.
This is a chord direction proxy, not an infinitesimal tangent or vehicle heading.

Actual adjacent spacing and observed-prefix polyline length are computed from
saved XY; origin-to-first segment is explicitly separate from point-to-point
segments. Polyline includes origin to match existing length convention, while
origin is never a residual/ADE target. Nominal s is not actual predicted arc length.

Windows CPU NumPy/matplotlib and standard-library unittest suffice. No WSL sync
or Dataset-related preflight required for this arrays-only task. Commit source first.
Full pytest/model tests are intentionally excluded. No package downloads needed.

```powershell
python tests/test_spatial_fixed_residuals_v4.py
python tools/analyze_spatial_fixed_residuals_v4.py --packet tmp/verify_al7j8vd0/spatial_v4_validation_review_20260906_153a22a --output tmp/spatial_fixed_residuals_<execution_sha>
```

Capture stdout/stderr as new separate log files. Outputs include all per-anchor,
per-grid residual rows (unsupported = null), windows/status, spacing, role/cohort/
distance/shape/run summaries, selected visual examples and exact source snapshots.
Figure choice = worst existing ADE in predeclared role/shape; visualization only.
All inputs are rehashed after processing. No automatic push/shadow connection.
