import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_ugv_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*')),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='meow',
    maintainer_email='meow@todo.todo',
    description='BrainChip Akida AKD1000 perception pipeline for the UGV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'akida_yolo_node = my_ugv_vision.akida_yolo_node:main',
            'object_localizer_node = my_ugv_vision.object_localizer_node:main',
            'landmark_tracker_node = my_ugv_vision.landmark_tracker_node:main',
            'obstacle_publisher_node = my_ugv_vision.obstacle_publisher_node:main',
        ],
    },
)
