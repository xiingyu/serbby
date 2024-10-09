import os
from glob import glob
from setuptools import find_packages
from setuptools import setup


package_name = 'Odometry_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='famas',
    maintainer_email='famas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = Odometry_package.odometry_node:main',
            'drive_path = Odometry_package.drive_path:main',
            'test_node = Odometry_package.test_node:main',
            'wheel_to_odometry_node = Odometry_package.wheel_to_odometry_node:main',
            'keyboard_wheel_node = Odometry_package.keyboard_wheel_node:main',
            'cmd_vel_to_wheel_node = Odometry_package.cmd_vel_to_wheel_node:main'
        ]
    },
)
