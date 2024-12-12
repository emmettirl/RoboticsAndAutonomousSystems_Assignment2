import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur3_rviz.launch.py', 'launch/ur3_gazebo.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/ur3.urdf.xacro']),
        ('share/' + package_name + '/rviz', ['rviz/ur3.rviz']),
        ('share/' + package_name + '/models', ['models/pedestal.urdf', 'models/table.urdf', 'models/cube.sdf']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of the ur3_description package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any console scripts here
        ],
    },
)