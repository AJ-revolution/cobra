import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cobrotic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aj',
    maintainer_email='aj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit2_to_arduino = scripts.moveit2_to_arduino:main',
            'ros2_to_arduino = scripts.ros2_to_arduino:main',
            'pose_recorder = scripts.pose_recorder:main',
        ],
    },
)
