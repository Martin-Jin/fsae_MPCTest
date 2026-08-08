from setuptools import find_packages, setup

package_name = 'fsae_sim_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='UoA FSAE',
    maintainer_email='fsae@auckland.ac.nz',
    description='Simulator perception stand-in: bridges FSDS oracle map + odom to the fsae_autonomous interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_perception = fsae_sim_perception.sim_perception:main',
            'cone_recorder  = fsae_sim_perception.cone_recorder:main',
        ],
    },
)
