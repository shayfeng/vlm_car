from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'scservo_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # 这将自动包含所有Python包，包括scservo_sdk
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='ROS2 driver for SCServo motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scservo_node = scservo_driver.scservo_node:main',
            'integrated_vision_control= scservo_driver.integrated_vision_control:main',
            'track_to_obj= scservo_driver.track_to_obj:main',
            'track_to_obj_v4= scservo_driver.track_to_obj_v4:main',
        ],
    },
)
