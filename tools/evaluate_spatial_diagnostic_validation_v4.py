"""Explicit fixed-checkpoint evaluation; no optimizer, training, resume or fallback."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aic_transfuser_lite.evaluation.spatial_diagnostic_validation_v4 import execute

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(execute(args.config, args.output))
