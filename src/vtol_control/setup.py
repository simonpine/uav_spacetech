from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'vtol_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ceres',
    maintainer_email='ceres.roverteam@gmail.com',
    description='Controlador de vuelo y piloto de teclado para el VTOL tiltrotor de tres motores.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = vtol_control.flight_controller:main',
            'keyboard_teleop = vtol_control.keyboard_teleop:main',
            'demo_flight = vtol_control.demo_flight:main',
        ],
    },
)
