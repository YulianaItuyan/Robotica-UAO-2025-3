from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),  # Cambiado de *.xacro a *.urdf
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),  # Mantener xacros también por si acaso
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),  # NUEVO: Agregar meshes STL
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='richilzz',
    maintainer_email='cristian.lizarazo@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pinterfaz = robot.pinterfaz:main',
            'joint_commander_deg = robot.joint_commander_deg:main',
            'serial_node = robot.serial_node:main',
        ],
    },
)
