from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rms_ros2_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'snaps'), glob('launch/*.jpg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hasegawa',
    maintainer_email='robohase01@gmail.com',
    description='RMS Client For ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = rms_ros2_client.image_saver_node:main',
            'rms_ros2_client_eqpt_updater = rms_ros2_client.rms_ros2_client_eqpt_updater:main',
        ],
    },
)
