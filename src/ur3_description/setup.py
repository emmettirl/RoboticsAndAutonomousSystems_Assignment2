import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.[pxy][yma]*')),
        ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*')),
        ('share/' + package_name + '/meshes/collision', glob('meshes/collision/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/resource', glob('resource/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/src', glob('src/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emmett.fitzharris@mycit.ie',
    maintainer_email='emmett.fitzharris@mycit.ie',
    description='Description of the ur3_description package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any console scripts here
        ],
    },
)