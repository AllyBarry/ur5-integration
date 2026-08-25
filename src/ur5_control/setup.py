import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='researcher',
    maintainer_email='1056862@students.wits.ac.za',
    description='Motion, gripper and perception-bridge nodes for the UR5 + Hand-E rig.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'move_to_pose = ur5_control.move_to_pose:main',
            'move_to_named_pose = ur5_control.move_to_named_pose:main',
            'gripper_server = ur5_control.gripper_server:main',
            'hand_target_bridge = ur5_control.hand_target_bridge:main',
            'mock_hand_publisher = ur5_control.mock_hand_publisher:main',
            'planning_scene_publisher = ur5_control.planning_scene_publisher:main',
        ],
    },
)
