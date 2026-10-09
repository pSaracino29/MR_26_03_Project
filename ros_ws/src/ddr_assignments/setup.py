from setuptools import find_packages, setup
from glob import glob

package_name = 'ddr_assignments'

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
            'aruco_mapper = ddr_assignments.aruco_mapper:main',
            'map_visualizer = ddr_assignments.map_visualizer:main',
            'plotter = ddr_assignments.plotter:main',
            'fan_plot = ddr_assignments.fan_plot:main',
            'sim_supervisor = ddr_assignments.sim_supervisor:main'
        ],
    },
)
