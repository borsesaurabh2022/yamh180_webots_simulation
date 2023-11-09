from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motoman_robotcrtl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'config_moveit'), glob('config_moveit/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*')),
        (os.path.join('share', package_name, 'meshes/mh180/collision'), glob('meshes/mh180/collision/*')),
        (os.path.join('share', package_name, 'meshes/mh180/visual'), glob('meshes/mh180/visual/*')),
        (os.path.join('share', package_name, 'meshes/mh180_120/collision'), glob('meshes/mh180_120/collision/*')),
        (os.path.join('share', package_name, 'meshes/mh180_120/visual'), glob('meshes/mh180_120/visual/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srb-jarvis',
    maintainer_email='saurabh.borse@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motoman_servocrtl = motoman_robotcrtl.motoman_servocrtl:main',
        ],
    },
)
