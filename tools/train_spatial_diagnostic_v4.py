"""Explicit single diagnostic run; no resume, promotion, raw access or search."""
from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aic_transfuser_lite.training.spatial_diagnostic_v4 import run_experiment

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(run_experiment(args.config, args.output))
