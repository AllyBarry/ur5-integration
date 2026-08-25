import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='researcher',
    maintainer_email='1056862@students.wits.ac.za',
    description='Description and launch files for the UR5 + Hand-E table rig.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
