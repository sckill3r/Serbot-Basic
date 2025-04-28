from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'serbot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='chawkigh2250@gmail.com',
    description='Serbot Controller Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serbot_controller = serbot_controller.serbot_controller:main',
        ],
    },
)
