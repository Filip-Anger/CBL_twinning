from setuptools import find_packages, setup

package_name = 'tb3_weather_dt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='falinux',
    maintainer_email='filip.anger@gmail.com',
    description='Gazebo-only weather-driven DT supervisor for TurtleBot3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'weather_adapter = tb3_weather_dt.weather_adapter:main',
            'twin_safety_supervisor = tb3_weather_dt.twin_safety_supervisor:main',
        ],
    },
)
