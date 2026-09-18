# Evidence scope

Small reports and a rendered figure only. Raw bags, models and replay shards stay
in native WSL. `summary.json` contains both retention references; `front_*` files
separate observed-motion selection from strict physical-avoidance certification.

`operators/` archives the executed diagnostic helpers, not general-purpose entry
points. They reference fixed native/remote paths and preserve existing output
directories. `audit_front.py`, `check_front_clearance.py` and
`compare_collection_paths.py` originally ran when the front collection contained
only the four completed runs (`box-a01`, `cone-a01`, `cone-a02`, `cone-b01`).
The later `cone-close6-a02` is a separate held trial and must not be added to those
frozen populations when reproducing this evidence. The script recorded in each
summary is identified by SHA-256. `stop_close6.py` records a one-time, PID- and
run-checked interrupt and is not a reusable command.

The archived collection operator shows the final scenario generator; the original
YAML/JSON for each actual run, exact source/runtime identity, transfer receipts and
all-file export manifests are preserved alongside that run in WSL. The invalid
`close6-a01` schema preview did not start AWSIM and is not counted as a drive.
