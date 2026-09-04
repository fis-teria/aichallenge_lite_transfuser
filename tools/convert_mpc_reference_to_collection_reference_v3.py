#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

from aic_transfuser_lite.data.collection_reference_v3 import (
    RoutePointV3,
    write_route_reference_v3,
)
from aic_transfuser_lite.data.recovery_reference_v3 import load_mpc_reference_v3


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create the fixed coverage axis from an official MPC Reference CSV."
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    points = tuple(
        RoutePointV3(
            point_id=index,
            x_m=point.x_m,
            y_m=point.y_m,
            heading_rad=point.psi_rad,
            frame_id="map",
        )
        for index, point in enumerate(load_mpc_reference_v3(args.input))
    )
    manifest = write_route_reference_v3(
        args.output,
        points,
        source_topic="/offline/official_mpc_reference",
        source_type="multi_purpose_mpc_ros/reference_csv",
        captured_utc=datetime.now(timezone.utc).isoformat(),
        source_artifact=args.input,
    )
    print(f"reference_csv={args.output.resolve()}")
    print(f"manifest_yaml={manifest.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
