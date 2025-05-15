import os
from glob import glob
from setuptools import setup

package_name = 'tino_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Your package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = tino_ros.robot_controller_node:main',
            'hardware_interface = tino_ros.hardware_interface_node:main',
            'gamepad_node = tino_ros.gamepad_node:main',
            'vr_interface_node = tino_ros.vr_interface_node:main',
            'pose_detection = tino_ros.pose_detection_node:main',
            'audio_node = tino_ros.audio_node:main',
            'audio_loopback = tino_ros.audio_loopback:main',
            'vr_data_recorder_node = tino_ros.vr_data_recorder_node:main',
            'vr_data_extractor = tino_ros.vr_data_extractor:main',
        ],
    },
)
