import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grupo',
    maintainer_email='juanginer@gmail.com',
    description='TP Final Parte A - Grid-Based FastSLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_mapper = maze_slam.occupancy_mapper:main',
            'grid_fastslam = maze_slam.grid_fastslam:main',
        ],
    },
)
