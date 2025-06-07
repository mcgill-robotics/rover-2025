from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pybullet_drive_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + 'pybullet_drive_sim']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'pybullet_drive_sim', 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', 'pybullet_drive_sim', 'urdf/meshes'), glob('urdf/meshes/*')),
    ],
    install_requires=['setuptools', 'pybullet'],
    zip_safe=True,
    maintainer='barry',
    maintainer_email='barry@todo.todo',
    description='PyBullet simulation for rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = pybullet_drive_sim.sim_node:main',
        ],
    },
)

