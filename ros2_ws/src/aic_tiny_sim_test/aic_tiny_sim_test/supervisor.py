"""ROS entry point reusing the tested Tiny runtime; no new network implementation."""
from pathlib import Path
import sys


def main() -> int:
    from ament_index_python.packages import get_package_share_directory
    shared = Path(get_package_share_directory("aic_tiny_sim_test"))
    sys.path[:0] = [str(shared/"vendor"), str(shared/"vendor/tools")]
    from run_tiny_lidar_dev import main as run_supervisor
    return run_supervisor()
