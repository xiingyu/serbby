from setuptools import find_packages, setup

package_name = 'serbby_algo'

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
    maintainer='skh',
    maintainer_email='tls3162@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_1=serbby_algo.test_1:main',
        'dual_cam_test=serbby_algo.dual_cam_test:main',
        'camera_serbby=serbby_algo.camera_serbby:main',
        ],
    },
)
