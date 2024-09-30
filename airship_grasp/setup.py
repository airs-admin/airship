from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'airship_grasp'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'lib'), glob('lib/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
]

# List of directories to include .py files from
directories_to_include = [
    'lib/pymycobot',
    'lib/Scale_Balanced_Grasp'
]

# Find and include all .py files from the specified directories
for directory in directories_to_include:
    py_files = glob(os.path.join(directory, '**', '*.py'), recursive=True)
    for py_file in py_files:
        # Define the target path for the .py files
        target_path = os.path.join('share', package_name, 'lib', os.path.relpath(py_file, 'lib'))
        data_files.append((os.path.dirname(target_path), [py_file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airsbot2',
    maintainer_email='airship@cuhk.edu.cn',
    description='Airship Grasp package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_server = airship_grasp.grasp_server_node:main',
        ],
    },
)
