from setuptools import find_packages, setup

package_name = 'fsae_control'

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
    description='Lateral/longitudinal control (Stanley + MPC) + FSDS command bridge for the simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller                = fsae_control.stanley_controller:main',
            'mpc_controller            = fsae_control.mpc_controller:main',
            'mpc_controller_standalone = fsae_control.mpc_controller_standalone:main',
            'fsds_bridge               = fsae_control.fsds_bridge:main',
            'steering_sysid            = fsae_control.steering_sysid:main',
            'steering_step             = fsae_control.steering_step:main',
        ],
    },
)
