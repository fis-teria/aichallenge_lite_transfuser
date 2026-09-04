#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from aic_transfuser_lite.data.recovery_reference_v3 import (
    render_official_mpc_recovery_config_v3,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Point a copied official MPC config at a staged recovery Reference."
    )
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-container-path", required=True)
    parser.add_argument(
        "--package-share-container-path",
        default="/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros",
    )
    args = parser.parse_args()
    output = render_official_mpc_recovery_config_v3(
        args.base_config,
        args.output,
        reference_container_path=args.reference_container_path,
        package_share_container_path=args.package_share_container_path,
    )
    print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
