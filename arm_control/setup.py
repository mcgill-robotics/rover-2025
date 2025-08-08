from setuptools import find_packages, setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['arm_control', 'arm_control.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='alex.zimo.zh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control_node = arm_control.arm_control_node:main',
            'arm_firmware_node = arm_control.arm_firmware_node:main',
            'sim_bridge_node = test.sim_bridge_node:main',
            'mock_gamepad_publisher = test.mock_gamepad_publisher:main',
            'arm_backup_node = arm_control.arm_backup_node:main'
        ],
    },
)
