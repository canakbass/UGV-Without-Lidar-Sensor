from setuptools import setup

package_name = 'ika_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision Processing',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = ika_vision.detector_node:main',
            'obstacle_detector = ika_vision.obstacle_detector_node:main',
        ],
    },
)
