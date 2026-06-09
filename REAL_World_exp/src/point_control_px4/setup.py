from setuptools import find_packages, setup
import os

package_name = 'point_control_px4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['mpc_BC_network_3'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'point_control_px4/models'),
            ['point_control_px4/models/mpcBc_net_298.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xuyulong',
    maintainer_email='xuyulong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nmpc_mavros           =   point_control_px4.nmpc_px4_quad_mavros:main',
            'test_thrust           =   point_control_px4.test_thrust:main',
            'nmpc_output_thrust_torque  =   point_control_px4.nmpc_output_thrust_torque:main',
            'pub_vs = point_control_px4.pub_vins:main',
        ],
    },
)


