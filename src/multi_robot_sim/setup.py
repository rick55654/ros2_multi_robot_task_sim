from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'multi_robot_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'opencv-python'
        'scipy',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Multi-robot simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = multi_robot_sim.simulation_node:main',
            'robot_agent = multi_robot_sim.robot_agent:main',
        ],
    },
)
