#!/usr/bin/env python3
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mcts'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Automatically find all packages and sub-packages
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'rclpy'],
    zip_safe=True,
    maintainer='Varun Lakshmanan',
    maintainer_email='varunl11@example.com',
    description='MCTS package for multi-robot simulation',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mcts_test = mcts.mcts_single:main',
            'mcts_tb1 = mcts.mcts_tb1:main',
            'mcts_tb2 = mcts.mcts_tb2:main',
            'mcts_tb3 = mcts.mcts_tb3:main',
            'mcts_tb4 = mcts.mcts_tb4:main',
            'Centralized_MCTS = mcts.Centralized_MCTS:main',
        ],
    },
)

