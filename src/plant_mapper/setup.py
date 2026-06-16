import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'plant_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('plant_mapper/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant_mapper = plant_mapper.plant_mapper:main',
            'farm_twin = plant_mapper.farm_twin:main',
            'farm_navigator = plant_mapper.farm_navigator:main',
            'battery = plant_mapper.battery:main',
        ],
    },
)
