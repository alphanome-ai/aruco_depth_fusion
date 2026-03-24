import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_depth_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Engineer',
    maintainer_email='robotics@example.com',
    description='ArUco marker detection using RGB-D depth fusion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_fusion_node = aruco_depth_fusion.aruco_fusion_node:main',
        ],
    },
)
