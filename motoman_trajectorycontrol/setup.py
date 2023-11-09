from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motoman_trajectorycontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.')),
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
            'motoman_trajcontrol_moveit = motoman_trajectorycontrol.motoman_trajcontrol_moveit:main',
            'motoman_trajcontrol_moveit_test = motoman_trajectorycontrol.motoman_trajcontrol_moveit_test:main',
            'motoman_trajcontrol = motoman_trajectorycontrol.motoman_trajcontrol:main',
            'motoman_trajcontrol_test = motoman_trajectorycontrol.motoman_trajcontrol_test:main',
        ],
    },
)
