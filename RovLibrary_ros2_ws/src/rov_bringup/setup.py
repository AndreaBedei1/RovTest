from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rov_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrea",
    maintainer_email="andrea@example.com",
    description="Launch and configuration for the RovLibrary ROS 2 foundation.",
    license="MIT",
    tests_require=["pytest"],
)

