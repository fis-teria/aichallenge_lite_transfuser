"""Check the ROS install manifest without requiring ROS or performing installation."""
from pathlib import Path
import runpy

import pytest
import setuptools


@pytest.mark.parametrize("filename", [
    "time_path_slam_mppi.json",
    "time_path_slam_mppi_20_15_15.json",
    "time_path_slam_mppi_20_15_15_recovery.json",
])
def test_ros_install_contains_mppi_profiles_and_recovery_source(monkeypatch, filename: str) -> None:
    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws/src/aic_e2e_runtime"
    metadata: dict = {}
    monkeypatch.chdir(package)
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: metadata.update(kwargs))
    runpy.run_path(str(package / "setup.py"), run_name="__main__")
    installed = {
        (Path(destination) / Path(source).name).as_posix(): (package / source).resolve()
        for destination, sources in metadata["data_files"]
        for source in sources
    }
    share = "share/aic_e2e_runtime"
    config = installed[f"{share}/config/{filename}"]
    assert config == root / "configs/control" / filename
    assert config.is_file()
    for name in ("slam_mppi.py", "time_path_recovery.py"):
        source = installed[f"{share}/python_src/aic_transfuser_lite/control/{name}"]
        assert source == root / "src/aic_transfuser_lite/control" / name
        assert source.is_file()
