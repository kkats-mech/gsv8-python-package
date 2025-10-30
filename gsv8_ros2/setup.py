from setuptools import setup
import os
from glob import glob

package_name = 'gsv8_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkats-mech',
    maintainer_email='katsampiris.konst@gmail.com',
    description='ROS2 package for GSV-8 force-torque sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gsv8_publisher = gsv8_ros2.gsv8_publisher_node:main',
        ],
    },
)
