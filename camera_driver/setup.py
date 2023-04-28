import os
from glob import glob
from setuptools import setup

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosi',
    maintainer_email='wiessel@artc.a-star.edu.sg',
    description='Package to start USB camera capture',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
