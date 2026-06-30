from setuptools import setup
from glob import glob

package_name = 'maze_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Giner',
    maintainer_email='juanginer@gmail.com',
    description='Grid-Based FastSLAM (TP Final Parte A).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fastslam_node = maze_slam.fastslam_node:main',
        ],
    },
)
