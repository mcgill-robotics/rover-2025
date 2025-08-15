from setuptools import find_packages, setup
import os
from glob import glob

package_name = "arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["arm", "arm.*", "test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aman",
    maintainer_email="asidhu_ca@yahoo.ca",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_control_node = arm.arm_control_node:main",
            "arm_firmware_node = arm.arm_firmware_node:main",
            "sim_bridge_node = test.sim_bridge_node:main",
            "mock_gamepad_publisher = test.mock_gamepad_publisher:main",
        ],
    },
)
