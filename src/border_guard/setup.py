from setuptools import setup
import os
from glob import glob

package_name = 'border_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='falinux',
    maintainer_email='filip.anger@gmail.com',
    description='Geofence enforcement — blocks robot movement outside a recorded polygon',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'border_guard_node = border_guard.border_guard_node:main',
        ],
    },
)
