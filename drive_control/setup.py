from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'test'), glob('test/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aman',
    maintainer_email='aman.sidhu@mail.mcgill.ca',
    description='Drive Control Scripts and Controllers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pub = test.pub:main',
            'test_sub = test.sub:main'
        ],
    },
)
