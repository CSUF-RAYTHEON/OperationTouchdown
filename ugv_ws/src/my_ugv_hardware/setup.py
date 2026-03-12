from setuptools import find_packages, setup

package_name = 'my_ugv_hardware'

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
    maintainer='ground',
    maintainer_email='ground@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'motor_driver = my_ugv_hardware.roboteq_bridge:main',
        'virtual_odom = my_ugv_hardware.virtual_odom:main',
        ],
    },
)
