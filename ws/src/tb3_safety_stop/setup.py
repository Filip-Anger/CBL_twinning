from setuptools import setup
import os
from glob import glob
 
package_name = 'tb3_safety_stop'
 
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Filip_anger',
    maintainer_email='filip.anger@gmail.com',
    description='TurtleBot3 Burger safety stop using lidar and TwistStamped',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_stop_node = tb3_safety_stop.safety_stop_node:main',
            'twin_safety_node = tb3_safety_stop.twin_safety_node:main', #only for the last step/mini-project
            'twin_safety_node_back   = tb3_safety_stop_back.twin_safety_node_back:main', #back as well
        ],
    },
)
