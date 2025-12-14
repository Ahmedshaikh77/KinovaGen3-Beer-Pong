from setuptools import setup
from glob import glob
import os

package_name = 'beer_pong'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Beer pong throwing system for Kinova Gen3 Lite',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beer_pong_throw = beer_pong.beer_pong_throw:main',
            'gripper_node = beer_pong.gripper_node:main',
            'beer_pong_throwall = beer_pong.beer_pong_throwall:main'
        ],
    },
)