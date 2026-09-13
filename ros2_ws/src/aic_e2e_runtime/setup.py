from glob import glob
import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = "aic_e2e_runtime"
canonical_source = Path(__file__).resolve().parents[3] / "src"
setup_root = Path(__file__).resolve().parent
canonical_python_data = [
    (
        "share/" + package_name + "/python_src/" + str(path.parent.relative_to(canonical_source)),
        [os.path.relpath(path, setup_root)],
    )
    for path in sorted((canonical_source / "aic_transfuser_lite").rglob("*.py"))
]

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/schemas", [os.path.relpath(canonical_source.parent / "schemas" / name, setup_root) for name in (
            "spatial_path_v4_shadow_record_v1.schema.json", "spatial_path_v4_runtime_record_v1.schema.json",
            "spatial_path_v4_live_passive_record_v1.schema.json")]),
        ("share/" + package_name + "/ckpt", glob("ckpt/*.pt")),
        ("share/" + package_name, ["aic_transfuser_lite_vendor.sha256"]),
    ] + canonical_python_data,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="fis-teria",
    maintainer_email="86540421+fis-teria@users.noreply.github.com",
    description="TransFuser Lite inference and safety runtime for AWSIM",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "time_path_node = aic_e2e_runtime.time_path_node:main",
            "time_trial_controller_node = aic_e2e_runtime.time_trial_controller_node:main",
            "v4_pp_connection_node = aic_e2e_runtime.v4_pp_connection_node:main",
            "inference_node = aic_e2e_runtime.inference_node:main",
            "inference_node_v1 = aic_e2e_runtime.inference_node_v1:main",
            "inference_node_v3 = aic_e2e_runtime.inference_node_v3:main",
            "spatial_path_shadow_node_v4 = aic_e2e_runtime.spatial_path_shadow_node_v4:main",
            "v4_shadow_node = aic_e2e_runtime.v4_shadow_node:main",
            "local_odometry_node_v4 = aic_e2e_runtime.local_odometry_node_v4:main",
            "safety_supervisor_node = aic_e2e_runtime.safety_supervisor_node:main",
            "calibration_excitation_node = aic_e2e_runtime.calibration_excitation_node:main",
        ],
    },
)
