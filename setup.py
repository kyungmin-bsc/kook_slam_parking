import os
from glob import glob

from setuptools import setup

package_name = 'parking_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='유경민',
    maintainer_email='biscuit2354@khu.ac.kr',
    description='자이카 Y모델 자율주행 주차 미션',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = parking_mission.cmd_vel_bridge:main',
            'mission_manager = parking_mission.mission_manager:main',
            'obstacle_monitor = parking_mission.obstacle_monitor:main',
            'fake_hardware = parking_mission.fake_hardware:main',
        ],
    },
)
