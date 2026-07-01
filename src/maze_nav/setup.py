from setuptools import setup
from glob import glob

package_name = 'maze_nav'

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
    description='Navegacion autonoma sobre el mapa de la Parte A (TP Final Parte B).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_publisher = maze_nav.map_publisher:main',
            'localizer = maze_nav.localizer:main',
            'navigator = maze_nav.navigator:main',
        ],
    },
)
