from setuptools import setup

package_name = "mppi_recovery_controller"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AI Challenge Team",
    maintainer_email="maintainer@example.com",
    description="Reuse MPC stuck recovery on the standalone MPPI control route",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "recovery_controller = mppi_recovery_controller.node:main",
        "real_actuation_filter = mppi_recovery_controller.real_actuation_filter:main",
    ]},
)
