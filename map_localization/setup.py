import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'map_localization'

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
            'coordinate_controler = map_localization.coordinate_controler:main',
            'current_coordinate = map_localization.current_coordinate:main',
            'order_keyboard = map_localization.order_keyboard:main'
        ],
    },
)
