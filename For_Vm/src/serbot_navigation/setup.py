from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'serbot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/navigation', glob('config/navigation/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Serbot Team',
    maintainer_email='info@serbot.com',
    description='Navigation package for Serbot robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odometry_handler = serbot_navigation.odometry_handler:main',
        ],
    },
)
