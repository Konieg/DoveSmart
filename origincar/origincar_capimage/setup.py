import os
from glob import glob

from setuptools import setup

package_name = 'origincar_capimage'

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
    maintainer='DoveCar',
    maintainer_email='3060822197@qq.com',
    description='origincar capimage',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capimage = origincar_capimage.capimage:main',
            'capvideo = origincar_capimage.capvideo:main',
        ],
    },
)
