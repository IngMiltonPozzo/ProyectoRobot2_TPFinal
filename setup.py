import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_cobertura'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Milton Pozzo',
    maintainer_email='milton@example.com',
    description='Monitoreo autónomo por cobertura del escenario Gazebo stage4 (Proyecto de Robots II - PRIA)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navegador = tb3_cobertura.nodo_navegador:main',
            'monitor = tb3_cobertura.nodo_monitor:main',
        ],
    },
)
