from setuptools import find_packages, setup
import os
from glob import glob

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join('launch', '*launch.py')),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mn297",
    maintainer_email="martin.nguyen3@mail.mcgill.ca",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_pub = test.pub:main",
            "test_sub = test.sub:main",
            "drive_control_node = scripts.drive_control_node:main",
            "odrive_node = scripts.node_odrive_drive:main",
            "gui_node = scripts.node_control_gui:main",
            "drive_firmware_node = scripts.drive_firmware_node:main",
            "pantilt_control_node = scripts.pantilt_control_node:main",
        ],
    },
)
