# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "mh180_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
        (os.path.join("share", package_name, "protos"), glob("protos/*")),
        (
            os.path.join("share", package_name, "meshes/end_effector"),
            glob("meshes/end_effector/*"),
        ),
        (
            os.path.join("share", package_name, "meshes/mh180/collision"),
            glob("meshes/mh180/collision/*"),
        ),
        (
            os.path.join("share", package_name, "meshes/mh180/visual"),
            glob("meshes/mh180/visual/*"),
        ),
        (
            os.path.join("share", package_name, "meshes/mh180_120/collision"),
            glob("meshes/mh180_120/collision/*"),
        ),
        (
            os.path.join("share", package_name, "meshes/mh180_120/visual"),
            glob("meshes/mh180_120/visual/*"),
        ),
        (os.path.join("share", package_name, "world"), glob("world/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="srb-jarvis",
    maintainer_email="saurabh.borse@alumni.fh-aachen.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mh180_servocontrol = mh180_simulation.mh180_servocontrol:main",
            "mh180_servocontrol_simulation = mh180_simulation.mh180_servocontrol_simulation:main",
            "mh180_moveittrajctrl_simulation = mh180_simulation.mh180_moveittrajctrl_simulation:main",
            "mh180_moveittrajctrl_actionserver = mh180_simulation.mh180_moveittrajctrl_actionserver3:main",
            "generate_broadcast_helix = mh180_simulation.generate_broadcast_helix2:main",
            "mh180_moveittrajctrl_actionclient = mh180_simulation.mh180_moveittrajctrl_actionclient:main",
        ],
    },
)
