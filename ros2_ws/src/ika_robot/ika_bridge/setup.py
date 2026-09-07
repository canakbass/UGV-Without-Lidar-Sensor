from setuptools import setup
import os
from glob import glob

package_name = 'ika_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Serial Bridge for ESP32',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = ika_bridge.serial_bridge_node:main',
        ],
    },
)
