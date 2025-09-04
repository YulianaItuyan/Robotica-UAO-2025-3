from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mi_br'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tronz_',
    maintainer_email='tronz_@todo.todo',
    description='Simulación y control de brazo robótico 3DOF',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = mi_br.joint_state_publisher:main',
            'mover_robot = mi_br.mover_robot:main',
            'ArmTeleop = mi_br.ArmTeleop:main',
            'test_arm_movement = mi_br.test_arm_movement:main',
            'joint_commander_deg = mi_br.joint_commander_deg:main',
            'fk_marker = mi_br.fk_marker:main',
            'pinterfaz = mi_br.pinterfaz:main',
            'serial_node = mi_br.serial_node:main',
            'spawn_urdf = mi_br.spawn_urdf:main',
            'mlol = mi_br.mlol:main',
        ],
    },
)

