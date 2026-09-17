from glob import glob
from setuptools import find_packages, setup

name = "aic_lidar_v2x"
setup(name=name, version="0.2.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + name]),
                  ("share/" + name, ["package.xml"]),
                  ("share/" + name + "/launch", glob("launch/*.launch.py")),
                  ("share/" + name + "/config", glob("config/*"))],
      install_requires=["setuptools", "numpy", "PyYAML", "Pillow"], zip_safe=False,
      maintainer="fis-teria", maintainer_email="86540421+fis-teria@users.noreply.github.com",
      description="Reusable LiDAR tracking and MPPI V44 V2X collection adapter", license="Apache-2.0",
      entry_points={"console_scripts": ["lidar_v2x_node = aic_lidar_v2x.node:main"]})
