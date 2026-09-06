"""Bundle only three explicit runtime files; never glob model/data/V4 assets."""
import os
from pathlib import Path
from setuptools import setup

here = Path(__file__).resolve().parent
repo = here.parents[2]
name = "aic_tiny_sim_test"
files = {
    "vendor/tools": [repo/"tools/run_tiny_lidar_dev.py", repo/"tools/spatial_dev_host_v4.py"],
    "vendor/aic_transfuser_lite": [repo/"src/aic_transfuser_lite/__init__.py"],
    "vendor/aic_transfuser_lite/runtime": [repo/"src/aic_transfuser_lite/runtime/__init__.py",
        repo/"src/aic_transfuser_lite/runtime/tiny_lidar_sim.py"],
}
setup(name=name, version="0.1.0", packages=[name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/"+name]),
                ("share/"+name, ["package.xml"]),
                ("share/"+name+"/launch", ["launch/guarded_tiny.launch.py"]),
                ("share/"+name+"/config", ["config/tiny_scan.rviz"])] +
        [("share/"+name+"/"+destination, [os.path.relpath(p, here) for p in paths]) for destination, paths in files.items()],
    install_requires=["setuptools"], zip_safe=False,
    maintainer="fis-teria", maintainer_email="86540421+fis-teria@users.noreply.github.com",
    description="Finite SIM-only Tiny control and stop", license="Apache-2.0",
    entry_points={"console_scripts": ["tiny_sim_supervisor = aic_tiny_sim_test.supervisor:main"]})
