import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'cobrotic'

# Copy URDF files
urdf_files = glob('urdf/*.urdf')

# Copy meshes if exists
mesh_files = glob('urdf/meshes/*')
if mesh_files:
    urdf_data_files = urdf_files + mesh_files
else:
    urdf_data_files = urdf_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', urdf_data_files),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aj',
    maintainer_email='aj@todo.todo',
    description='Cobra robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit2_to_arduino = cobrotic_nodes.moveit2_to_arduino:main',
            'one_motion_to_arduino = cobrotic_nodes.one_motion_to_arduino:main',
        ],
    },
)
