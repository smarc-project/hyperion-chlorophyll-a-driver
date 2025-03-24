from setuptools import setup
import os
from glob import glob

package_name = 'valeport_hyperion_chlorophyll_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='ROS 2 driver for Valeport Hyperion Chlorophyll a',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hyperion_chlorophyll_node = valeport_hyperion_chlorophyll_driver.src.serial_reader.serial_reader_node:main',
            'chlorophyll_decoder = valeport_hyperion_chlorophyll_driver.src.data_processor.data_processor_node:main',
        ],
    },
)
