import os
import glob
from setuptools import find_packages, setup

package_name = 'my_ugv_vision'

# Collect any .fbz model files placed in models/
model_files = glob.glob(os.path.join('models', '*.fbz'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/config',
            glob.glob(os.path.join('config', '*.yaml'))),
        # Install any .fbz model files from the models/ directory.
        # To add ugv_object_detect_model.fbz, either copy or symlink it into
        # ugv_ws/src/my_ugv_vision/models/ before building.
        ('share/' + package_name + '/models', model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ground',
    maintainer_email='raytheonuav@outlook.com',
    description='UGV vision package — BrainChip Akida object detection via OAK-D camera.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'object_detector = my_ugv_vision.object_detector:main',
        ],
    },
)
