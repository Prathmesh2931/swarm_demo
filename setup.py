from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'swarm_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*')),
        (os.path.join('share', package_name, 'models', 'simple_quad'), 
            ['models/simple_quad/model.sdf']),
        (os.path.join('share', package_name, 'config'), 
            ['config/bridge_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prathmesh2931',
    maintainer_email='atkaleprathmesh@gmail.com',
    description='ROS2 Swarm Robotics Demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_node = swarm_demo.master_node:main',
            'swarm_manager = swarm_demo.swarm_manager:main',
            'drone_controller = swarm_demo.drone_controller:main',
        ],
    },
)