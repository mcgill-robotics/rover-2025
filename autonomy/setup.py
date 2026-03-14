from setuptools import find_packages, setup
import os
from glob import glob

package_name = "arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["scripts"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aerin",
    maintainer_email="aerinebrown04@gmail.com",
    description="Scripts for perception, localization/mapping, and path planning during autonomous drive",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "TODO = PKG.NODE:main"
        ],
    },
)
