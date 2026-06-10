from setuptools import find_packages
from setuptools import setup

setup(
    name='my_ugv_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('my_ugv_bringup', 'my_ugv_bringup.*')),
)
