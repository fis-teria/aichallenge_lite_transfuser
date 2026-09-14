"""Render saved fixed-data comparison metrics; no inference or metric recomputation."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.comparison.resolve()
    summary = json.loads(source.read_bytes())
    if summary["status"] != "COMPLETE" or not summary["data_and_loss_unchanged"]:
        raise ValueError("completed fixed-data comparison required")
    cells = ("uniform/endpoint_3s", "uniform/teacher_pp", "balanced/endpoint_3s", "balanced/teacher_pp")
    models = [summary["models"][summary["cells"][key]] for key in cells]
    panels = (("nominal", "Normal validation (4 runs)", "cm"),
              ("comparison_recovery", "Recovery comparison (2 runs)", "cm"),
              ("comparison_outward", "Outward subset (6 correlated frames)", "cm"),
              ("comparison_recovery", "Teacher PP steering agreement", "rad"))
    figure, axes = plt.subplots(1, 4, figsize=(14, 4.5))
    for ax, (group, title, unit) in zip(axes, panels, strict=True):
        values = [(m["groups"][group]["pp"]["run_macro_penalized_rad"] if unit == "rad"
                   else 100 * m["groups"][group]["xy"]["run_macro_mean"]["3s"]["raw_error_m"]) for m in models]
        if not np.isfinite(values).all() or min(values) < 0:
            raise ValueError("figure requires finite nonnegative supported metrics")
        bars = ax.bar(range(4), values, color=("#64748b", "#a0aabc", "#c73586", "#e694bd"))
        for bar, value in zip(bars, values, strict=True):
            ax.annotate(f"{value:.5f}" if unit == "rad" else f"{value:.3f}",
                        (bar.get_x()+bar.get_width()/2, bar.get_height()),
                        xytext=(0, 4), textcoords="offset points", ha="center", fontsize=8)
        ax.set_xticks(range(4), [f"{letter}\ne{model['epoch']}" for letter, model in zip("ABCD", models, strict=True)])
        ax.set(title=title, ylabel=("Penalized run-macro error [rad]" if unit == "rad" else "Run-macro 3s XY error [cm]"))
        ax.set_ylim(0, max(max(values)*1.25, 1e-6)); ax.grid(axis="y", alpha=.2); ax.set_axisbelow(True)
    figure.suptitle("Fixed data and update budget / one seed / offline comparison", fontsize=14)
    figure.text(.5, .015, "A/B: uniform sampling; C/D: outward-balanced. A/C: 3s selection; B/D: teacher-PP selection.\n"
                 "Comparison runs were previously reported; these bars do not measure closed-loop lap completion.",
                 ha="center", fontsize=9)
    figure.tight_layout(rect=(0, .11, 1, .92))
    args.output.mkdir(parents=True, exist_ok=False)
    image = args.output / "method_comparison.png"
    figure.savefig(image, dpi=150); plt.close(figure)
    receipt = {"scope": "SAVED_METRIC_RENDER_ONLY", "source": str(source),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "argv": sys.argv, "image_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
        "cells": summary["cells"]}
    (args.output / "render_receipt.json").write_bytes((json.dumps(receipt, indent=2) + "\n").encode())
    print(json.dumps({"status": "FIGURE_RENDERED", "image": str(image)}))


if __name__ == "__main__":
    main()
