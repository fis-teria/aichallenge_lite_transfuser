# Bounded MPPI raceline search

The reference CSVs are the exact tested geometry files. Use each with its paired MPPI YAML and `reference_execution_speed_cap_mps:=10.0` (normal) or `:=7.5` (leader). The legacy CSV vx column is not the effective target; the execution-profile parameter and the MPPI cruise/overtake/proximity parameters set the evaluated target.

Controller image: sha256:7a585a12c5593bf12466b229577b2c9e6afacd43b078b51985dbf584409001c5

The current MPPI and its corner-acceleration controller remain enabled. These are solo closed-loop references, not a guarantee for traffic/overtaking or a global optimum. See summary.json for all repeated-run outcomes; a selected path is only preferred-feasible when all three checks completed without contact/over penalties or measured OT overlap. The dense_checks and validated_with_interpolation fields additionally check interpolated footprints. No production reference was replaced.
