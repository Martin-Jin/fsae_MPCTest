import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fsae_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='UoA FSAE',
    maintainer_email='fsae@auckland.ac.nz',
    description='Simulator path planning: centerline and skidpad planners',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centerline_planner = fsae_planning.centerline_planner:main',
            'skidpad_planner    = fsae_planning.special_utils.skidpad_planner:main',
        ],
    },
)
