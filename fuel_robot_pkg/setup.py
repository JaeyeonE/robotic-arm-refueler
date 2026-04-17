from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fuel_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='gyurangs',
    maintainer_email='gyurangs@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_gateway_node = fuel_robot_pkg.ui_gateway_node:main',
            'fueling_task_manager_node = fuel_robot_pkg.fueling_task_manager_node:main',
            'doosan_commander_node = fuel_robot_pkg.doosan_commander_node:main',
            'safety_monitor_node = fuel_robot_pkg.safety_monitor_node:main',
        ],
    },
)
