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
            "inference_node = aic_e2e_runtime.inference_node:main",
            "inference_node_v1 = aic_e2e_runtime.inference_node_v1:main",
            "inference_node_v3 = aic_e2e_runtime.inference_node_v3:main",
            "safety_supervisor_node = aic_e2e_runtime.safety_supervisor_node:main",
            "calibration_excitation_node = aic_e2e_runtime.calibration_excitation_node:main",
        ],
    },
)
