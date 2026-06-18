from glob import glob
import os
import subprocess
import sys

from setuptools import Command, find_packages, setup


package_name = "rov_control_manager"


class PyTestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        raise SystemExit(subprocess.call([sys.executable, "-m", "pytest", "test"]))

setup(
    name=package_name,
    version="0.2.0",
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
    description="Safe high-level command router for RovLibrary ROS 2.",
    license="MIT",
    cmdclass={"test": PyTestCommand},
    entry_points={
        "console_scripts": [
            "control_manager_node = rov_control_manager.control_manager_node:main",
        ],
    },
)
