from setuptools import setup
import os
from glob import glob

package_name = 'border_recorder'

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
    description='Record driven border path via SLAM and export as polygon (GeoJSON/CSV)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'border_recorder_node = border_recorder.border_recorder_node:main',
        ],
    },
)
