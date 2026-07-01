import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'maze_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grupo',
    maintainer_email='valentinoarbelaiz@gmail.com',
    description='TP Final Parte C - percepcion visual del cono rojo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detector = maze_perception.cone_detector_node:main',
        ],
    },
)
