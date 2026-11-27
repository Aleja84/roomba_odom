import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roomba_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alumno',
    maintainer_email='alumno@example.com',
    description='Roomba en Gazebo: cinemática inversa, odometría (Euler/RK2/exacta), '
                'permisos, freno de emergencia y órdenes de voz.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kinematics_node = roomba_odom.kinematics_node:main',
            'odometry_node = roomba_odom.odometry_node:main',
            'experimento = roomba_odom.experimento:main',
            'gestor = roomba_odom.gestor:main',
            'guardian = roomba_odom.guardian:main',
            'voz_node = roomba_odom.voz_node:main',
            'boton_emergencia = roomba_odom.boton_emergencia:main',
        ],
    },
)
