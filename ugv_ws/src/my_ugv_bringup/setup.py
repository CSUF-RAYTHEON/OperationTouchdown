from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_ugv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob(os.path.join('behavior_trees', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ugv',
    maintainer_email='user@example.com',
    description='Bringup, relay, and launch files for the UGV platform.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_nav_relay = my_ugv_bringup.cmd_vel_nav_relay:main',
        ],
    },
)
