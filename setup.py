#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'path_following'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/config', glob.glob(os.path.join('config', '*.yaml'))),
        (share_dir + '/rviz', glob.glob(os.path.join('rviz', '*.rviz'))),
        (share_dir + '/path', glob.glob(os.path.join('path', '*txt')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='pby04188',
    author_email='pby04188@naver.com',
    maintainer='pby04188',
    maintainer_email='pby04188@naver.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='path following',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_planner = path_following.pid_path_following:main',
            'pose_to_odom = path_following.pose_to_odom:main',
            'pose_stamped_to_odom = path_following.pose_stamped_to_odom:main',
            'pose_test = path_following.pose_test:main'
        ],
    }, # {'console_scripts': ['node_name = path_to_python_file:main']}
)