from glob import glob
import os

from setuptools import find_packages, setup


package_name = "maze_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="udesa",
    maintainer_email="tadeo.casiraghi@gmail.com",
    description="Known-pose occupancy grid mapping for robot_maze_runner.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "known_pose_mapper = maze_slam.known_pose_mapper:main",
            "save_map = maze_slam.save_map:main",
        ],
    },
)
