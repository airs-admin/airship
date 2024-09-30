import glob
import os

from setuptools import find_packages, setup

package_name = 'airship_perception'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
]

py_files = glob.glob(os.path.join('lib', '**', '*.py'), recursive=True)
for py_file in py_files:
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
    description='airship_perception',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seg_service_node = airship_perception.seg_service_node:main',
            'service_test = airship_perception.service_test:main',
        ],
    },
)
