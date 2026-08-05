import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'mst110cr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'),glob('map/*.pgm')),
        (os.path.join('share', package_name, 'map'),glob('map/*.yaml')),
        (os.path.join('share', package_name, 'params'),glob('params/*.yaml')),
        (os.path.join('share', package_name, 'params'),glob('params/*.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.xml.in')),
        (os.path.join('share', package_name, 'parameters'),glob('parameters/*.yaml')),
        (os.path.join('share', package_name, 'rviz2'),glob('rviz2/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasahara',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poseStamped2Odometry = mst110cr_navigation.poseStamped2Odometry:main',
            'odom_broadcaster = mst110cr_navigation.odom_broadcaster:main',
            'map_generator = mst110cr_navigation.map_generator:main',
            'message_converter_gnss = mst110cr_navigation.message_converter_gnss:main',
            'message_converter_odom = mst110cr_navigation.message_converter_odom:main',
        ],
    },
)
