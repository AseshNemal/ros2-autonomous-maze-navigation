from setuptools import setup
from glob import glob
import os
package_name = 'maze_simulation'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Team',
    maintainer_email='maintainer@example.com',
    description='Gazebo Harmonic custom maze for TurtleBot3 Burger',
    license='Apache-2.0',
)
