import glob
import os

from setuptools import find_packages, setup

package_name = "rosconjp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/mcl.launch.py"]),
        ("share/" + package_name, ["launch/mapping.launch.py"]),
        ("share/" + package_name, ["launch/odometry.launch.py"]),
        ("share/" + package_name, ["launch/start.launch.py"]),
        (
            os.path.join("share", package_name, "config"),
            [
                "config/mcl_config.yaml",
                "config/mapping_config_icp.yaml",
                "config/mapping_config_planar.yaml",
            ],
        ),
        (os.path.join("share", package_name, "data"), glob.glob("data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry = rosconjp.node.odometry_node:main",
            "mcl = rosconjp.node.mcl_node:main",
            "mapping = rosconjp.node.mapping_node:main",
        ],
    },
)
