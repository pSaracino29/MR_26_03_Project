from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ddr_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_detector = ddr_exploration.frontier_detector:main',
            'frontier_selector = ddr_exploration.frontier_selector:main',
            'explorer = ddr_exploration.explorer:main',
        ],
    },
)
