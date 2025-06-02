from setuptools import find_packages, setup

package_name = 'hyperion_chlorophyll_a_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'chlorophyll_decoder = hyperion_chlorophyll_a_driver.data_processor_node:main',
        ],
    },
)
